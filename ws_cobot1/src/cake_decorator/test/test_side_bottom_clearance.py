"""스테이지 충돌을 피하기 위한 바닥 10mm 입력 제한을 검사한다."""

import pytest
from cake_decorator.drawing_geometry import FrustumCake, validate_side_drawing_height


def test_floor_rejects_low_points_without_rescaling_allowed_points():
    cake = FrustumCake(r_top=56.75, r_bottom=60., height=79., cx=0., cy=0., cz=143.)
    validate_side_drawing_height([(0., 10.), (12., 79.)])
    for height in (0., 9.99):
        with pytest.raises(ValueError, match='10mm'):
            validate_side_drawing_height([(0., height)])
    assert cake.side_to_abs(0., 10.)[2] == 153.


def test_low_side_order_is_rejected_before_cake_transport():
    """기존 주문이나 API로 금지 좌표를 보냈어도 운반을 시작하지 않는다."""
    import rclpy
    from unittest.mock import Mock
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Point32
    from cake_decorator.task_manager_node import TaskManagerNode
    from cake_decorator_interfaces.action import CakeOrder
    from cake_decorator_interfaces.msg import DrawingPath, Stroke2D

    rclpy.init()
    node = TaskManagerNode()
    try:
        node.set_parameters([Parameter('plate_sequence_enabled', value=True)])
        request = CakeOrder.Goal(order_id='LOW-SIDE')
        cake = request.cake_params
        cake.radius, cake.bottom_radius, cake.height = .05675, .060, .079
        request.drawing_paths = [DrawingPath(
            surface='SIDE', canvas_width=1024, canvas_height=768,
            strokes=[Stroke2D(stroke_id=0, points=[Point32(x=300., y=475.), Point32(x=310., y=475.)])],
        )]
        goal = Mock(request=request, is_cancel_requested=False)
        node._run_plate_service = Mock()
        result = node._execute_order(goal)
        assert not result.success
        assert '10mm' in result.error_msg
        node._run_plate_service.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_dp_rejects_low_request_with_no_partial_waypoints():
    """작업 관리자를 우회해도 DP는 금지 높이를 포함한 경로를 반환하지 않는다."""
    import rclpy
    from geometry_msgs.msg import Point32
    from cake_decorator.drawing_path_node import DrawingPathNode
    from cake_decorator_interfaces.srv import GenerateTrajectory
    from cake_decorator_interfaces.msg import Stroke2D

    rclpy.init()
    node = DrawingPathNode()
    try:
        request = GenerateTrajectory.Request(
            surface_type='SIDE', canvas_width=1024., canvas_height=768.,
            cake_radius=.05675, cake_bottom_radius=.06, cake_height=.079,
            operation_speed_percent=100,
            strokes_2d=[Stroke2D(stroke_id=0, points=[Point32(x=300., y=380.), Point32(x=310., y=475.)])],
        )
        response = node._handle_request(request, GenerateTrajectory.Response())
        assert not response.success
        assert not response.waypoints_3d
        assert '10mm' in response.error_msg
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
