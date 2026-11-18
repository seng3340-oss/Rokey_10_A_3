"""생성된 ROS 타입이 PM 문서의 데이터를 실제로 직렬화할 수 있는지 검사한다."""

import unittest

from geometry_msgs.msg import Point32
from rclpy.serialization import deserialize_message, serialize_message

from cake_decorator_interfaces.action import (
    CakeOrder,
    DrawOnCake,
    GraspTool,
    PickAndPlace,
)
from cake_decorator_interfaces.msg import (
    CakeParams,
    DrawingPath,
    Stroke2D,
    TaskStatus,
    Waypoint3D,
    WorkspaceCoords,
)
from cake_decorator_interfaces.srv import GenerateTrajectory, SetCompliance, SwitchTCP


class InterfaceTests(unittest.TestCase):
    """노드·Docker·로봇을 실행하지 않고 ROS 생성 코드만 검증한다."""

    def roundtrip(self, message):
        """전송과 같은 직렬화 과정을 거쳐 필드 값이 보존되는지 확인한다."""
        return deserialize_message(serialize_message(message), type(message))

    def test_order_preserves_top_and_side_pixel_strokes(self):
        """두 면의 원본 픽셀과 케이크 m 치수를 서로 섞지 않고 전달한다."""
        goal = CakeOrder.Goal()
        goal.order_id = "ORD-TEST-001"
        goal.tool_type = "marker"
        goal.cake_params = CakeParams(
            radius=0.05675,
            bottom_radius=0.06,
            height=0.079,
        )
        for surface in ("TOP", "SIDE"):
            stroke = Stroke2D(stroke_id=0)
            stroke.points = [Point32(x=500.5, y=350.25, z=0.0)]
            goal.drawing_paths.append(
                DrawingPath(
                    canvas_width=1024,
                    canvas_height=768,
                    surface=surface,
                    strokes=[stroke],
                )
            )
        restored = self.roundtrip(goal)
        self.assertEqual(restored.order_id, goal.order_id)
        self.assertEqual([p.surface for p in restored.drawing_paths], ["TOP", "SIDE"])
        self.assertEqual(restored.drawing_paths[0].strokes[0].points[0].x, 500.5)
        self.assertAlmostEqual(restored.cake_params.radius, 0.05675)
        self.assertAlmostEqual(restored.cake_params.bottom_radius, 0.06)
        for path in restored.drawing_paths:
            self.assertEqual((path.canvas_width, path.canvas_height), (1024, 768))
            self.assertEqual(path.strokes[0].stroke_id, 0)
            self.assertNotIn("is_closed", path.strokes[0].get_fields_and_field_types())

    def test_trajectory_receives_canvas_size_and_multiple_stroke_ids(self):
        """두 개의 획 끝을 자동 연결하지 않고 번호·해상도를 함께 전달한다."""
        request = GenerateTrajectory.Request(
            order_id="ORD-TEST-001",
            operation_speed_percent=100,
            # 서비스 해상도는 float64다. int를 넣으면 C 직렬화에서 abort된다.
            canvas_width=1024.0,
            canvas_height=768.0,
            surface_type="TOP",
            cake_radius=0.05675,
            cake_bottom_radius=0.06,
            cake_height=0.079,
            strokes_2d=[
                Stroke2D(stroke_id=0, points=[Point32(x=50.0, y=80.0)]),
                Stroke2D(stroke_id=1, points=[Point32(x=60.0, y=90.0)]),
            ],
        )
        restored = self.roundtrip(request)
        self.assertEqual((restored.canvas_width, restored.canvas_height), (1024, 768))
        self.assertAlmostEqual(restored.cake_radius, 0.05675)
        self.assertAlmostEqual(restored.cake_bottom_radius, 0.06)
        self.assertEqual([stroke.stroke_id for stroke in restored.strokes_2d], [0, 1])
        self.assertTrue(
            all(
                "is_closed" not in stroke.get_fields_and_field_types()
                for stroke in restored.strokes_2d
            )
        )

    def test_waypoints_preserve_pose_and_pen_state(self):
        """목표 위치 m·속도 mm/s·펜 상태를 별도 필드로 보존한다."""
        waypoint = Waypoint3D(speed=15.0, pen_down=False, contact_force=0.0)
        waypoint.pose.position.x = 0.5
        waypoint.pose.orientation.w = 1.0
        response = GenerateTrajectory.Response(
            success=True,
            waypoints_3d=[waypoint],
            pen_states=[False],
            estimated_duration_sec=3.0,
        )
        restored = self.roundtrip(response)
        self.assertEqual(len(restored.waypoints_3d), len(restored.pen_states))
        self.assertFalse(restored.waypoints_3d[0].pen_down)
        self.assertEqual(restored.waypoints_3d[0].speed, 15.0)
        self.assertAlmostEqual(restored.waypoints_3d[0].pose.position.x, 0.5)

    def test_all_action_sections_and_services_serialize(self):
        """목표뿐 아니라 실행 피드백·최종 결과와 보조 서비스도 생성돼야 한다."""
        for action in (CakeOrder, PickAndPlace, GraspTool, DrawOnCake):
            for section in (action.Goal, action.Result, action.Feedback):
                self.roundtrip(section())
        for service in (GenerateTrajectory, SwitchTCP, SetCompliance):
            self.roundtrip(service.Request())
            self.roundtrip(service.Response())
        self.roundtrip(TaskStatus(order_id="ORD-TEST-001", progress_pct=25.0))
        self.roundtrip(
            WorkspaceCoords(
                cake_radius=0.05675,
                cake_bottom_radius=0.06,
                cake_height=0.079,
            )
        )

    def test_compliance_axes_are_six_values_without_enabling_control(self):
        """미사용 서비스의 고정 배열 형식만 검사하며 실제 순응제어는 하지 않는다."""
        request = SetCompliance.Request(
            enable=False,
            axis=[0.0] * 6,
            stiffness=[0.0] * 6,
        )
        restored = self.roundtrip(request)
        self.assertFalse(restored.enable)
        self.assertEqual(len(restored.axis), 6)
        self.assertEqual(len(restored.stiffness), 6)


if __name__ == "__main__":
    unittest.main()
