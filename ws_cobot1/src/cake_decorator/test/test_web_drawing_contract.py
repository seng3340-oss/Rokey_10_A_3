"""실제 웹 편집 영역과 DP 출력 계약의 회귀 검증. 로봇 이동은 하지 않는다."""

import math
import unittest

import rclpy
from geometry_msgs.msg import Point32
from cake_decorator_interfaces.msg import Stroke2D
from cake_decorator_interfaces.srv import GenerateTrajectory
from cake_decorator.drawing_path_node import DrawingPathNode


class WebDrawingContractTests(unittest.TestCase):
    """100×80mm 원통과 1024×768 웹 편집 영역을 기준으로 경계와 획 상태를 확인한다."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = DrawingPathNode()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def convert(self, surface, points):
        """TM이 만드는 원본 픽셀 요청을 DP 콜백에 전달한다."""
        request = GenerateTrajectory.Request()
        request.order_id = "WEB-CONTRACT"
        request.operation_speed_percent = 100
        request.canvas_width = 1024.0
        request.canvas_height = 768.0
        request.surface_type = surface
        request.cake_radius = 0.05
        request.cake_height = 0.08
        request.cake_pose.position.x = 0.5
        request.cake_pose.position.z = 0.142
        request.strokes_2d = [
            Stroke2D(
                stroke_id=0,
                points=[Point32(x=float(x), y=float(y)) for x, y in points],
            )
        ]
        response = self.node._handle_request(request, GenerateTrajectory.Response())
        self.assertTrue(response.success, response.error_msg)
        self.assertEqual(
            response.pen_states, [p.pen_down for p in response.waypoints_3d]
        )
        # SIDE는 원통 안을 가로지르지 않도록 높은 지점 경유 두 점이 추가된다.
        expected = (
            [False, True, True, False]
            if surface == "TOP"
            else [False, False, False, False, True, True, False]
        )
        self.assertEqual(response.pen_states, expected)
        return response.waypoints_3d

    def test_top_web_circle_edge_is_fifty_mm_from_center(self):
        """웹 오른쪽 원 경계가 실제 반지름 50mm와 일치해야 한다."""
        path = self.convert("TOP", [(512, 384), (832, 384)])
        self.assertAlmostEqual(path[1].pose.position.x, 0.5)
        self.assertAlmostEqual(path[2].pose.position.x, 0.55)
        self.assertAlmostEqual(path[2].pose.position.z, 0.142)
        self.assertGreater(path[-1].pose.position.z, 0.142)

    def test_side_edit_strip_uses_entire_height_and_radius(self):
        """캔버스 여백을 제외한 옆면 띠 상하 끝이 케이크 상하단에 대응해야 한다."""
        strip_height = 896 * 80 / (math.pi * 100)
        top = (768 - strip_height) / 2
        path = self.convert("SIDE", [(512, top), (512, top + strip_height)])[3:]
        self.assertAlmostEqual(path[1].pose.position.x, 0.45, places=6)
        self.assertAlmostEqual(path[1].pose.position.z, 0.142, places=6)
        self.assertAlmostEqual(path[2].pose.position.z, 0.062, places=6)
        # 옆면에서 펜을 올리는 동작은 Z 이동이 아니라 반지름 바깥 방향이다.
        self.assertLess(path[-1].pose.position.x, path[2].pose.position.x)
        self.assertAlmostEqual(path[-1].pose.position.z, path[2].pose.position.z)


if __name__ == "__main__":
    unittest.main()


def test_default_drawing_speed_matches_test2_and_scales_once():
    """30mm/s 기준에 주문 비율을 한 번만 적용하고 이동 속도와 구분한다."""
    from rclpy.parameter import Parameter

    rclpy.init()
    node = DrawingPathNode()
    try:
        node.set_parameters([Parameter("travel_speed", value=80.0)])
        request = GenerateTrajectory.Request()
        request.canvas_width, request.canvas_height = 1024.0, 768.0
        request.cake_radius, request.cake_height = 0.05, 0.08
        request.cake_pose.position.x, request.cake_pose.position.z = 0.5, 0.142
        request.surface_type = "TOP"
        request.strokes_2d = [
            Stroke2D(
                stroke_id=0,
                points=[
                    Point32(x=512.0, y=384.0),
                    Point32(x=650.0, y=384.0),
                ],
            )
        ]
        for percent in (50, 100):
            request.operation_speed_percent = percent
            path, _ = node._convert_strokes(request)
            assert path[2].speed == 30.0 * percent / 100
            assert path[0].speed == 80.0 * percent / 100
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
