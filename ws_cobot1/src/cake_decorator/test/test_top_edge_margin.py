"""윗면 가장자리 여유와 입력 원본의 축척을 검증한다."""

import pytest
from cake_decorator.drawing_geometry import validate_top_drawing_margin, pixel_to_mm_top


def test_top_margin_has_no_rescaling_and_rejects_previous_outer_marks():
    validate_top_drawing_margin([(51.75, 0.), (0., -51.75)], 56.75)
    for point in [(51.76, 0.), (0., -52.79), (56.75, 0.)]:
        with pytest.raises(ValueError, match='5mm'):
            validate_top_drawing_margin([point], 56.75)
    assert pixel_to_mm_top(512 + 25 * 640 / 113.5, 384, 1024, 768, 56.75)[0] == pytest.approx(25)


def test_top_outer_points_are_rejected_before_transport():
    """이전 웹 데이터나 API의 외곽 좌표도 운반 전에 거부한다."""
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
        request = CakeOrder.Goal(order_id='TOP-EDGE')
        request.cake_params.radius = .05675
        request.cake_params.bottom_radius = .06
        request.cake_params.height = .079
        request.drawing_paths = [DrawingPath(
            surface='TOP', canvas_width=1024, canvas_height=768,
            strokes=[Stroke2D(stroke_id=0, points=[Point32(x=815., y=384.), Point32(x=820., y=384.)])],
        )]
        node._run_plate_service = Mock()
        result = node._execute_order(Mock(request=request, is_cancel_requested=False))
        assert not result.success
        assert '5mm' in result.error_msg
        node._run_plate_service.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
