"""펜·따개 없음 판정 뒤 1회 재파지 규칙을 실물 명령 없이 검사한다.

규칙
    - 들어올린 뒤 측정한 폭이 기준보다 좁으면(도구 없음) 그리퍼를 열고
      tool_regrasp_wait_sec 기다린 뒤 거치대에서 한 번 더 집는다.
    - 다시 잡으면 공정을 계속한다. 두 번째도 없으면 safe_position_1에서 대기한다.
    - 폭 측정 실패·파지 전 열림 부족·HMI 정지 중에는 재시도하지 않는다.
"""

from unittest.mock import Mock

import pytest

from cake_decorator.plate_sequence import PlateSequence
from cake_decorator.safety_exceptions import (
    OpenerMissingError,
    OpenerNotHeldError,
)
from test_integrated_pen_cycle import rack_parameters, make_executor, draw_handle


def widths(*values, default=0.080):
    """주어진 폭을 순서대로 돌려준 뒤에는 정상 폭(80mm)을 돌려주는 측정 대역."""
    remaining = list(values)
    return Mock(side_effect=lambda *args, **kwargs: remaining.pop(0) if remaining else default)


# ── 따개 ────────────────────────────────────────────────────────────


def opener_plate(*measured):
    """하중 검사는 끄고 폭 검사만 켠 플레이트 시퀀스와 명령 기록 대역."""
    params = rack_parameters()
    params.update(
        gripper_safety_check_enabled=True,
        plate_weight_check_enabled=False,
        tool_regrasp_wait_sec=0.0,
    )
    robot = Mock(stop_latched=False, gripper_output_fault=False)
    robot._logger = Mock()
    robot.move_joint.return_value = True
    robot.pick.return_value = True
    robot.gripper_open.return_value = True
    robot.get_gripper_width = widths(*measured)
    return PlateSequence(robot, params.__getitem__), robot


def test_opener_missing_once_is_regrasped_and_continues():
    """1회차 따개 없음 → 그리퍼 열기 → 2회차 파지 성공이면 True로 계속 진행한다."""
    seq, robot = opener_plate(0.010, 0.020)
    assert seq._plate_pick_opener() is True
    assert robot.pick.call_count == 2
    robot.gripper_open.assert_called_once()
    assert seq._opener_held is True


def test_opener_missing_twice_stops_at_safe_position():
    """2회차도 없으면 재시도 없이 안전 자세에서 대기 예외를 올린다."""
    seq, robot = opener_plate(0.010, 0.010)
    with pytest.raises(OpenerNotHeldError, match='안전 자세에서 대기'):
        seq._plate_pick_opener()
    assert robot.pick.call_count == 2
    assert robot.gripper_open.call_count == 2
    # 마지막 이동은 폭 검사를 한 safe_position_1이다(추가 이동 없음).
    assert robot.move_joint.call_args.args[0] == rack_parameters()['plate_safe_position_1']


def test_opener_width_unknown_is_not_retried():
    """폭 측정 실패는 따개 유무를 알 수 없으므로 재파지하지 않는다."""
    seq, robot = opener_plate(None)
    with pytest.raises(OpenerMissingError) as caught:
        seq._plate_pick_opener()
    assert not isinstance(caught.value, OpenerNotHeldError)
    assert robot.pick.call_count == 1


def test_opener_not_retried_while_stop_latched():
    """HMI 정지 중에는 거치대로 다시 가지 않는다."""
    seq, robot = opener_plate(0.010)
    robot.stop_latched = True
    with pytest.raises(OpenerNotHeldError):
        seq._plate_pick_opener()
    assert robot.pick.call_count == 1


# ── 펜 ──────────────────────────────────────────────────────────────


def pen_executor(monkeypatch, *measured):
    """통합 공정 실행 노드에 측정 폭 순서를 주입한다. 대기 시간은 0초다."""
    node = make_executor(monkeypatch, integrated_side_cycle=True, tool_regrasp_wait_sec=0.0)
    node._robot.stop_latched = False
    node._robot.get_gripper_width = widths(*measured)
    return node


@pytest.mark.parametrize('surface', ['TOP', 'SIDE'])
def test_pen_missing_once_is_regrasped_and_drawing_continues(monkeypatch, surface):
    """열림 정상 → 인출 후 펜 없음 → 그리퍼 열기·재파지 → 정상이면 드로잉까지 성공한다."""
    node = pen_executor(monkeypatch, 0.080, 0.010)
    result = node._execute_draw(draw_handle(surface))
    assert result.success, result.error_code
    kinds = [kind for kind, _ in node._robot.events]
    assert 'draw' in kinds
    # 1회차·2회차 파지에서 각각 그리퍼를 닫았어야 한다(재파지 없이는 성공할 수 없음).
    assert kinds.count('close') >= 2
    assert node._robot.tcp == ''


@pytest.mark.parametrize('surface', ['TOP', 'SIDE'])
def test_pen_missing_twice_returns_to_safe_position(monkeypatch, surface):
    """두 번 모두 펜이 없으면 그리지 않고 그리퍼를 연 뒤 safe_position_1로 간다."""
    node = pen_executor(monkeypatch, 0.080, 0.010, 0.080, 0.010)
    result = node._execute_draw(draw_handle(surface))
    assert not result.success
    assert result.error_code == 'PEN_GRASP_FAILED'
    events = node._robot.events
    assert not any(kind == 'draw' for kind, _ in events)
    assert [kind for kind, _ in events][-2:] == ['open', 'joint']
    assert events[-1][1] == rack_parameters()['plate_safe_position_1']
    assert node._robot.tcp == ''


def test_pen_open_width_failure_is_not_retried(monkeypatch):
    """파지 전 열림 부족은 그리퍼 이상이므로 재시도 없이 기존처럼 안전 자세로 간다."""
    node = pen_executor(monkeypatch, 0.010)
    result = node._execute_draw(draw_handle('TOP'))
    assert result.error_code == 'PEN_GRASP_FAILED'
    assert [kind for kind, _ in node._robot.events].count('close') == 0
