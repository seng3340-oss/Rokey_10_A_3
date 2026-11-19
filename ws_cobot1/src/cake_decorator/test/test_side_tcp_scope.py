"""옆면 작업의 TCP 초기화 순서와 기본 플랜지 좌표 복원을 검증한다."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def make_services(*, moving=False):
    """TCP 선택과 운전 모드만 바뀌며 실제 관절 이동은 없는 서비스 대역이다."""
    state = {"tcp": "Pen_Side_45", "mode": 1, "moving": moving}
    events = []

    def call(kind, path, request, **kwargs):
        events.append((path, request))
        if path.endswith("move_stop"):
            state["moving"] = False
        elif path.endswith("check_motion"):
            return SimpleNamespace(success=True, status=2 if state["moving"] else 0)
        elif path.endswith("get_robot_mode"):
            return SimpleNamespace(success=True, robot_mode=state["mode"])
        elif path.endswith("set_robot_mode"):
            state["mode"] = request.robot_mode
        elif path.endswith("set_current_tcp"):
            assert not state["moving"]
            assert state["mode"] == 0
            state["tcp"] = request.name
        elif path.endswith("get_current_tcp"):
            return SimpleNamespace(success=True, info=state["tcp"])
        elif path.endswith("get_current_tool_flange_posx"):
            return SimpleNamespace(success=True, pos=[400., -50., 500., 0., 180., 0.])
        elif path.endswith("get_current_posx"):
            return SimpleNamespace(
                success=True,
                task_pos_info=[SimpleNamespace(data=[400., -50., 500., 0., 180., 0.])],
            )
        return SimpleNamespace(success=True)

    return call, state, events


def test_reset_clears_tcp_without_registering_or_moving():
    """빈 이름으로 선택을 해제하고 TCP가 플랜지와 일치하는지 검증한다."""
    from cake_decorator.tcp_scope import reset_default_tcp

    call, state, events = make_services()
    reset_default_tcp(call)
    assert state["tcp"] == ""
    assert state["mode"] == 1
    names = [path for path, _ in events]
    assert not any("move_line" in path or "config_create_tcp" in path for path in names)
    selected = [request.name for path, request in events if path.endswith("set_current_tcp")]
    assert selected == [""]


def test_interrupted_work_stops_before_resetting_tcp():
    """실패·종료 정리는 정지 명령과 실제 정지 확인 후 TCP를 바꾼다."""
    from cake_decorator.tcp_scope import reset_default_tcp

    call, state, events = make_services(moving=True)
    reset_default_tcp(call, stop_running=True)
    names = [path.rsplit("/", 1)[-1] for path, _ in events]
    assert names.index("check_motion") < names.index("move_stop") < names.index("set_current_tcp")
    assert state["tcp"] == ""


def test_failed_reset_is_reported_and_restores_operating_mode():
    """기본 TCP 적용에 실패하면 성공으로 처리하지 않으며 운전 모드는 복구한다."""
    from cake_decorator.tcp_scope import reset_default_tcp

    original, state, _ = make_services()

    def call(kind, path, request, **kwargs):
        if path.endswith("set_current_tcp"):
            raise RuntimeError("TCP 설정 통신 실패")
        return original(kind, path, request, **kwargs)

    with pytest.raises(RuntimeError, match="통신 실패"):
        reset_default_tcp(call)
    assert state["mode"] == 1
    assert state["tcp"] == "Pen_Side_45"


def test_motion_not_confirmed_never_changes_tcp(monkeypatch):
    """정지 조회가 계속 BUSY면 TCP 변경을 보내지 않는다."""
    from cake_decorator.tcp_scope import reset_default_tcp

    call, state, events = make_services(moving=True)
    monkeypatch.setattr("cake_decorator.tcp_scope.time.monotonic", Mock(side_effect=[0., 6.]))
    with pytest.raises(RuntimeError, match="정지"):
        reset_default_tcp(call)
    assert state["tcp"] == "Pen_Side_45"
    assert not any(path.endswith("set_current_tcp") for path, _ in events)


def test_zero_name_with_remaining_tip_offset_is_not_success():
    """이름 해제 응답만 믿지 않고 실제 TCP의 플랜지 일치를 검사한다."""
    from cake_decorator.tcp_scope import reset_default_tcp

    original, _, _ = make_services()

    def call(kind, path, request, **kwargs):
        response = original(kind, path, request, **kwargs)
        if path.endswith("get_current_posx"):
            response.task_pos_info[0].data[2] -= 271.695
        return response

    with pytest.raises(RuntimeError, match="일치"):
        reset_default_tcp(call)


@pytest.mark.parametrize("successful", [True, False])
def test_action_resets_after_motion_and_before_success(successful):
    """반납을 포함한 본 동작 뒤 TCP 초기화를 끝내야 주문 성공을 알린다."""
    from cake_decorator.robot_executor_node import RobotExecutorNode

    events = []
    result = SimpleNamespace(success=successful, error_code="" if successful else "MOVE_FAILED")
    robot = Mock()
    robot.reset_default_tcp.side_effect = lambda **kwargs: events.append("reset")
    handle = Mock()
    handle.request.drawing_surface = "SIDE"
    handle.is_cancel_requested = False
    handle.succeed.side_effect = lambda: events.append("success")

    def motion(goal):
        events.append("motion_and_return")
        return result

    executor = SimpleNamespace(
        # 독립 SIDE 시험만 활성화한다. 새 통합 옵션까지 임의로 True로 만들지 않는다.
        _param=lambda key: key == "side_pen_45",
        _robot=robot,
        _execute_draw_motion=motion,
        get_logger=Mock(),
    )
    actual = RobotExecutorNode._execute_draw(executor, handle)
    assert actual is result
    assert events[:2] == ["motion_and_return", "reset"]
    assert ("success" in events) == successful
    assert executor._side_pen_returned
    robot.reset_default_tcp.assert_called_once_with(stop_running=not successful)


def test_action_cannot_succeed_if_tcp_reset_fails():
    """그림과 반납을 끝냈어도 초기화 실패는 주문 성공으로 숨기지 않는다."""
    from cake_decorator.robot_executor_node import RobotExecutorNode

    robot = Mock()
    robot.reset_default_tcp.side_effect = RuntimeError("초기화 실패")
    result = SimpleNamespace(success=True, error_code="")
    executor = SimpleNamespace(
        _param=lambda key: key == "side_pen_45",
        _robot=robot,
        _execute_draw_motion=lambda _: result,
        get_logger=Mock(),
    )
    handle = Mock()
    handle.request.drawing_surface = "SIDE"
    handle.is_cancel_requested = False
    actual = RobotExecutorNode._execute_draw(executor, handle)
    assert not actual.success
    assert actual.error_code == "TCP_RESET_FAILED"
    handle.succeed.assert_not_called()
    handle.abort.assert_called_once()
    robot.reset_default_tcp.assert_called_once_with(stop_running=False)


def test_tilted_monitoring_flange_angles_are_zyx():
    """드라이버 감시 자세 ZYX와 현재 TCP의 ZYZ가 같은 물리 자세임을 확인한다."""
    from cake_decorator.tcp_scope import reset_default_tcp

    original, _, _ = make_services()

    def call(kind, path, request, **kwargs):
        response = original(kind, path, request, **kwargs)
        if path.endswith("get_current_posx"):
            response.task_pos_info[0].data[3:] = [90., 135., 89.77]
        elif path.endswith("get_current_tool_flange_posx"):
            response.pos[3:] = [.162637, 179.837357, -44.999767]
        return response

    reset_default_tcp(call, flange_euler="ZYX")


def test_already_stopped_zero_tcp_does_not_send_redundant_stop_or_mode():
    """실패 정리라도 이미 기본 TCP와 정지가 확인되면 정지 명령 거부를 피한다."""
    from cake_decorator.tcp_scope import reset_default_tcp

    original, state, events = make_services()
    state['tcp'] = ''

    def call(kind, path, request, **kwargs):
        assert not path.endswith(('move_stop', 'set_robot_mode', 'set_current_tcp'))
        return original(kind, path, request, **kwargs)

    assert reset_default_tcp(call, stop_running=True) == 'already_default'
    assert state['tcp'] == ''


def test_stop_failure_can_proceed_only_when_motion_is_independently_idle():
    """정지 응답이 실패해도 이후 IDLE 조회가 확인된 경우만 TCP를 복원한다."""
    from cake_decorator.tcp_scope import reset_default_tcp

    original, state, events = make_services(moving=True)

    def call(kind, path, request, **kwargs):
        if path.endswith('move_stop'):
            # 다른 정지 원인과 겹쳐 stop 응답은 실패했지만 로봇은 이미 멈춘 경우.
            state['moving'] = False
            return SimpleNamespace(success=False)
        return original(kind, path, request, **kwargs)

    assert reset_default_tcp(call, stop_running=True) == 'restored'
    assert state['tcp'] == ''


def test_stop_failure_with_busy_robot_never_changes_tcp(monkeypatch):
    """정지 명령 거부 후 계속 BUSY라면 이름이 무엇이든 TCP를 변경하지 않는다."""
    from cake_decorator.tcp_scope import reset_default_tcp

    original, state, events = make_services(moving=True)

    def call(kind, path, request, **kwargs):
        if path.endswith('move_stop'):
            return SimpleNamespace(success=False)
        return original(kind, path, request, **kwargs)

    monkeypatch.setattr('cake_decorator.tcp_scope.time.monotonic', Mock(side_effect=[0., 6.]))
    with pytest.raises(RuntimeError, match='TCP 변경 안 함'):
        reset_default_tcp(call, stop_running=True)
    assert state['tcp'] == 'Pen_Side_45'
    assert not any(path.endswith('set_current_tcp') for path, _ in events)
