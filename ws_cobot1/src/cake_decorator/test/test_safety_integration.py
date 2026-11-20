"""실물 명령 없이 최신 속도·예외 복귀·HMI 전달 계약을 검사한다."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from sensor_msgs.msg import JointState

from cake_decorator.doosan_robot import DoosanRobotClient
from cake_decorator.plate_sequence import PlateSequence
from cake_decorator.safety_exceptions import OpenerMissingError, PenGraspError, PlateWeightError
from cake_decorator.task_manager_node import TaskManagerNode
from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
from test_integrated_pen_cycle import ServiceRobot, rack_parameters, make_executor, draw_handle


def plate():
    """명령을 기록하는 대역과 실제 팀 파라미터를 준비한다."""
    params = rack_parameters()
    params.update(plate_weight_check_enabled=True, plate_weight_wait_sec=0.0,
                  plate_weight_reads=1, plate_weight_read_interval=0.0)
    robot = Mock(stop_latched=False, gripper_output_fault=False)
    robot._logger = Mock()
    robot.move_joint.return_value = True
    robot.place.return_value = True
    robot.gripper_open.return_value = True
    return PlateSequence(robot, params.__getitem__), robot, params


def test_weight_failure_returns_at_half_speed_then_safe():
    """75g 이하이면 75/75/75mm/s 반납 후 안전 자세로 끝난다."""
    seq, robot, params = plate()
    robot.get_workpiece_weight.side_effect = [0.10, 0.15, 0.20]
    seq._weight_at_safe('prepare_start')
    seq._weight_at_safe('prepare_opener')
    seq._opener_held = True
    with pytest.raises(PlateWeightError, match='안전 자세에서 대기'):
        seq._weight_at_safe('prepare_both')
    args = robot.place.call_args.kwargs
    assert [args[key] for key in ('speed_approach', 'speed_slow', 'speed_retract')] == [75, 75, 75]
    assert robot.move_joint.call_args.args[0] == params['plate_safe_position_1']
    assert robot.move_joint.call_args.kwargs['speed_deg_s'] == 30
    assert [call[0] for call in robot.mock_calls if call[0] in ('place', 'move_joint')] == ['place', 'move_joint']


def test_weight_recovery_failure_does_not_move_again():
    """따개 반납이 실패하면 안전 자세 이동을 강행하지 않는다."""
    seq, robot, _ = plate()
    seq._opener_held = True
    seq._weight_checker = Mock()
    seq._weight_checker.check.side_effect = RuntimeError('측정 실패')
    robot.place.return_value = False
    with pytest.raises(PlateWeightError, match='반납 실패'):
        seq._weight_at_safe('prepare_both')
    robot.move_joint.assert_not_called()


def test_missing_opener_opens_at_safe_without_further_moves():
    seq, robot, _ = plate()
    robot.get_gripper_width.return_value = 0.010
    with pytest.raises(OpenerMissingError, match='안전 자세'):
        seq._guard_opener()
    robot.gripper_open.assert_called_once()
    robot.move_joint.assert_not_called()


@pytest.mark.parametrize('method,kwargs,expected', [
    ('_plate_joint', {'name': 'safe_position_1'}, (60, 120)),
    ('_plate_joint', {'name': 'cake_station_ps2'}, (15, 15)),
    ('_plate_joint', {'name': 'safe_position_1', 'cake_carrying': True}, (30, 30)),
    ('_plate_line', {'name': 'move_cake_plate_ps4'}, (25, 50)),
    ('_plate_line', {'name': 'move_cake_ps_l2'}, (150, 300)),
    ('_plate_line', {'name': 'move_cake_ps_l2', 'cake_carrying': True}, (50, 100)),
    ('_plate_joint', {'name': 'safe_position_1', 'speed_deg_s': 20, 'acc_deg_s2': 35}, (20, 35)),
    ('_plate_line', {'name': 'move_cake_ps_l2', 'speed_mm_s': 40, 'acc_mm_s2': 55}, (40, 55)),
])
def test_speed_defaults_and_explicit_override(method, kwargs, expected):
    seq, robot, _ = plate()
    getattr(seq, method)(**kwargs)
    joint = method == '_plate_joint'
    call = robot.move_joint if joint else robot.move_line
    keys = ('speed_deg_s', 'acc_deg_s2') if joint else ('speed_mm_s', 'acc_mm_s2')
    assert tuple(call.call_args.kwargs[key] for key in keys) == expected


@pytest.mark.parametrize('where', ['open', 'held'])
def test_side_pick_checks_both_width_stages(where):
    """45도 파지에서는 하강 전과 인출 후에 각각 검사한다."""
    robot = ServiceRobot()
    robot.get_gripper_width = Mock(side_effect=[0.01] if where == 'open' else [0.08, 0.01])
    cycle = IntegratedPenCycle(robot, rack_parameters().__getitem__)
    with pytest.raises(PenGraspError):
        cycle.pick_side()
    assert robot.tcp == ''
    assert not any(kind == 'draw' for kind, _ in robot.events)


@pytest.mark.parametrize('surface', ['TOP', 'SIDE'])
def test_pen_failure_restores_tcp_before_safe_recovery(monkeypatch, surface):
    node = make_executor(monkeypatch, integrated_side_cycle=True)
    node._robot.stop_latched = False
    node._robot.get_gripper_width = Mock(return_value=0.01)
    request = draw_handle(surface)
    result = node._execute_draw(request)
    assert not result.success
    assert result.error_code == 'PEN_GRASP_FAILED'
    assert node._robot.tcp == ''
    events = node._robot.events
    assert [kind for kind, _ in events][-2:] == ['open', 'joint']
    assert events[-1][1] == rack_parameters()['plate_safe_position_1']


def test_stale_or_pre_command_width_is_rejected():
    robot = object.__new__(DoosanRobotClient)
    robot.stop_latched = False
    robot._gripper_sample = (0.03, 10.0)
    robot._gripper_command_since = 0.0
    with patch('cake_decorator.doosan_robot.time.monotonic', return_value=12):
        assert robot.get_gripper_width(timeout=0) is None
        robot._gripper_sample = (0.03, 11.9)
        robot._gripper_command_since = 12.0
        assert robot.get_gripper_width(timeout=0) is None
        robot._gripper_sample = (0.03, 12.0)
        assert robot.get_gripper_width(timeout=0) > 0.06


def test_gripper_callback_ignores_unknown_joint():
    robot = object.__new__(DoosanRobotClient)
    robot._gripper_sample = None
    robot._on_gripper_joint_state(JointState(name=['wrong'], position=[0.0]))
    assert robot._gripper_sample is None


def test_failed_weight_response_is_not_a_measurement():
    robot = object.__new__(DoosanRobotClient)
    robot._call = Mock(return_value=SimpleNamespace(success=False, weight=0.2))
    assert robot.get_workpiece_weight() is None


@pytest.mark.parametrize('code', ['OPENER_MISSING', 'CAKE_WEIGHT_FAILED'])
def test_plate_error_detail_reaches_task_manager(code):
    """Trigger의 오류 코드·한국어 설명을 최종 주문 실패에 쓸 수 있게 보존한다."""
    node = SimpleNamespace(_wait_future=Mock(), get_logger=Mock(return_value=Mock()))
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.return_value.result.return_value = SimpleNamespace(success=False, message=f'{code}: 안전 자세에서 대기중')
    assert not TaskManagerNode._run_plate_service(node, client, '준비')
    assert node._plate_error == (code, '안전 자세에서 대기중')


@pytest.mark.parametrize('fault', ['stop_latched', 'gripper_output_fault'])
def test_weight_failure_does_not_recover_when_commands_are_blocked(fault):
    """정지·출력 이상 상태에서는 예외 복귀도 새 명령을 보내지 않는다."""
    seq, robot, _ = plate()
    setattr(robot, fault, True)
    seq._opener_held = True
    seq._weight_checker = Mock()
    seq._weight_checker.check.side_effect = RuntimeError('측정 실패')
    with pytest.raises(PlateWeightError, match='차단'):
        seq._weight_at_safe('prepare_both')
    robot.place.assert_not_called()
    robot.move_joint.assert_not_called()


def test_safe_joint_failure_is_reported_after_weight_recovery():
    """따개 반납 뒤 안전 자세 이동 실패를 정상 복귀로 표시하지 않는다."""
    seq, robot, _ = plate()
    seq._opener_held = True
    seq._weight_checker = Mock()
    seq._weight_checker.check.side_effect = RuntimeError('측정 실패')
    robot.move_joint.return_value = False
    with pytest.raises(PlateWeightError, match='안전 자세 복귀 실패'):
        seq._weight_at_safe('prepare_both')
    robot.place.assert_called_once()


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf'), True])
def test_invalid_speed_override_sends_no_motion(value):
    seq, robot, _ = plate()
    with pytest.raises(ValueError):
        seq._plate_joint('safe_position_1', speed_deg_s=value)
    robot.move_joint.assert_not_called()
