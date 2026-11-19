"""윗면·옆면 순서와 서로 다른 TCP 기준 좌표가 섞이지 않는지 검증한다."""

from types import SimpleNamespace

import pytest
from cake_decorator.task_manager_node import order_surfaces, side_reference_pose


def test_top_always_precedes_side_and_keeps_stroke_groups():
    side = SimpleNamespace(surface='SIDE')
    top = SimpleNamespace(surface='TOP')
    assert order_surfaces([side, top]) == [top, side]
    assert order_surfaces([side]) == [side]
    assert order_surfaces([top]) == [top]
    with pytest.raises(ValueError):
        order_surfaces([SimpleNamespace(surface='INVALID')])


def test_side_center_is_physical_tip_height_not_top_flange_height():
    pose = side_reference_pose([396.8743225097656, -50.559329986572266, 222., 123.66, 180., 123.43])
    assert pose.position.x == pytest.approx(.3968743225097656)
    assert pose.position.y == pytest.approx(-.050559329986572266)
    assert pose.position.z == .222
    with pytest.raises(ValueError):
        side_reference_pose([0.] * 6)


def test_combined_order_prepares_once_draws_top_then_side_finishes_once():
    """장비 없이 실제 TM/DP 변환을 거쳐 운반·두 면·후처리 순서를 확인한다."""
    import rclpy
    from rclpy.parameter import Parameter
    from concurrent.futures import Future
    from unittest.mock import Mock
    from geometry_msgs.msg import Point32
    from cake_decorator_interfaces.action import CakeOrder
    from cake_decorator_interfaces.msg import DrawingPath, Stroke2D
    from cake_decorator_interfaces.srv import GenerateTrajectory
    from cake_decorator.task_manager_node import TaskManagerNode
    from cake_decorator.drawing_path_node import DrawingPathNode

    rclpy.init()
    tm, dp = TaskManagerNode(), DrawingPathNode()
    original_client = tm._gen_traj_client
    events, heights, remaining = [], [], []
    try:
        tm.set_parameters([
            Parameter('plate_sequence_enabled', value=True),
            Parameter('integrated_side_cycle', value=True),
            Parameter('side_reference_pose', value=[396.8743225097656, -50.559329986572266, 222., 123.66, 180., 123.43]),
        ])
        dp.set_parameters([
            Parameter('side_pen_45', value=True),
            Parameter('integrated_side_cycle', value=True),
            Parameter('side_y_inset_mm', value=1.),
            Parameter('side_diameter_allowance_mm', value=8.1),
        ])
        request = CakeOrder.Goal(order_id='COMBINED-TEST')
        cake = request.cake_params
        cake.radius, cake.bottom_radius, cake.height = .05675, .060, .079
        cake.center.x, cake.center.y, cake.center.z = .3968743, -.0505593, .51039
        cake.tool_orientation.y = 1.
        for surface in ('SIDE', 'TOP'):
            request.drawing_paths.append(DrawingPath(
                canvas_width=1024, canvas_height=768, surface=surface,
                strokes=[Stroke2D(stroke_id=0, points=[Point32(x=512., y=384.), Point32(x=540., y=390.)])],
            ))

        def convert(req):
            heights.append((req.surface_type, req.cake_pose.position.z))
            result = dp._handle_request(req, GenerateTrajectory.Response())
            assert result.success, result.error_msg
            future = Future()
            future.set_result(result)
            return future

        tm._gen_traj_client = SimpleNamespace(wait_for_service=lambda **_: True, call_async=convert)
        tm._wait_future = lambda *_: None
        # HMI 표시 채널이 고장 나도 기존 운반·그리기·반납 순서를 바꾸지 않는다.
        tm._plan_pub.publish = Mock(side_effect=RuntimeError("HMI 표시 채널 장애"))
        tm._run_plate_service = lambda client, phase: events.append('PREPARE' if client is tm._plate_prepare_client else 'FINISH') or True
        tm._send_draw = lambda goal, _, *progress_context: remaining.append(goal.has_more_surfaces) or events.append(goal.drawing_surface) or True
        goal = Mock(request=request, is_cancel_requested=False)
        result = tm._execute_order(goal)
        assert result.success
        goal.succeed.assert_called_once()
        assert events == ['PREPARE', 'TOP', 'SIDE', 'FINISH']
        assert remaining == [True, False]
        assert heights == [('TOP', .51039), ('SIDE', .222)]
    finally:
        tm._gen_traj_client = original_client
        tm.destroy_node()
        dp.destroy_node()
        rclpy.try_shutdown()


def test_transport_service_does_not_accept_unsupported_cancellation():
    """취소할 수 없는 운반 서비스에서 취소 완료를 약속하지 않는다."""
    from unittest.mock import Mock
    from rclpy.action import CancelResponse
    from cake_decorator.task_manager_node import TaskManagerNode, Phase

    node = SimpleNamespace(_current_order_id='TEST', get_logger=Mock())
    for phase in (Phase.PREPARING, Phase.FINISHING):
        node._current_phase = phase
        assert TaskManagerNode._handle_cancel(node, Mock()) == CancelResponse.REJECT
    node._current_phase = Phase.DRAWING
    assert TaskManagerNode._handle_cancel(node, Mock()) == CancelResponse.ACCEPT


def test_draw_cancellation_is_forwarded_before_reporting_result():
    """부모 취소가 자식에 전달돼 종료될 때까지 새 단계로 진행하지 않는다."""
    from unittest.mock import Mock
    from concurrent.futures import Future
    from cake_decorator.task_manager_node import TaskManagerNode

    result_future = Future()
    child = Mock(accepted=True)
    child.get_result_async.return_value = result_future
    child.cancel_goal_async.side_effect = lambda: result_future.set_result(
        SimpleNamespace(result=SimpleNamespace(success=False))
    )
    send = Future()
    send.set_result(child)
    node = SimpleNamespace(
        get_parameter=lambda _: SimpleNamespace(value=1.),
        get_logger=Mock(), _on_draw_feedback=Mock(),
        _draw_client=Mock(), _check_cancel=lambda _: True,
        _wait_future=TaskManagerNode._wait_future,
    )
    node._draw_client.send_goal_async.return_value = send
    assert not TaskManagerNode._send_draw(node, Mock(), Mock())
    child.cancel_goal_async.assert_called_once()
