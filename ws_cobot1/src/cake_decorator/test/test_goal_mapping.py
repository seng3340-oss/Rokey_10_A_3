"""로봇 없이 원본 픽셀·실측 치수·TCP 영점의 ROS 변환을 검사한다."""

import math
import unittest

from cake_decorator.goal_mapping import build_goal, radius_at_depth


class GoalTests(unittest.TestCase):
    """단위와 원점 혼동으로 케이크를 관통하는 경로가 만들어지지 않도록 계약을 확인한다."""

    def claim(self):
        """하나의 획과 측정 치수를 포함하는 백엔드 인수 데이터를 만든다."""
        return {
            "order_id": "ORD-TEST-1",
            "drawing": {
                "canvas_width": 1024,
                "canvas_height": 768,
                "drawing_surface": "top",
                "strokes": [
                    {
                        "stroke_id": 0,
                        "points": [{"x": 500.5, "y": 350.25}, {"x": 520.0, "y": 360.0}],
                    }
                ],
            },
            "cake": {
                "top_diameter_mm": 113.5,
                "bottom_diameter_mm": 120.0,
                "height_mm": 79.0,
                "board_width_mm": 179.0,
                "board_depth_mm": 179.0,
                "board_thickness_mm": 2.0,
                "tool_type": "pen",
                "reference": "tcp_at_top_center",
            },
        }

    def test_mapping_keeps_pixels_and_top_center_tcp(self):
        """TCP Z는 상면 중심이다. 높이·판 두께를 다시 더하지 않는다."""
        goal = build_goal(self.claim(), [363.9, 5.4, 75.25, 0.0, 180.0, 0.0])
        self.assertAlmostEqual(goal.cake_params.center.z, 0.07525)
        self.assertAlmostEqual(goal.cake_params.radius, 0.05675)
        self.assertAlmostEqual(goal.cake_params.bottom_radius, 0.06)
        self.assertAlmostEqual(goal.cake_params.board_thickness, 0.002)
        self.assertEqual(goal.cake_params.reference_frame, "base")
        self.assertAlmostEqual(goal.cake_params.tool_orientation.y, 1.0)
        self.assertEqual(goal.drawing_paths[0].strokes[0].points[0].x, 500.5)
        self.assertEqual(goal.drawing_paths[0].surface, "TOP")
        self.assertNotIn(
            "is_closed", goal.drawing_paths[0].strokes[0].get_fields_and_field_types()
        )
        self.assertAlmostEqual(radius_at_depth(goal.cake_params, 0.079), 0.06)

    def test_invalid_or_missing_reference_never_becomes_zero_pose(self):
        """영점 누락·NaN을 BASE 원점으로 대체하지 않는다."""
        for pose in ([], [0.0] * 5, [0.0, 0.0, math.nan, 0.0, 0.0, 0.0]):
            with self.assertRaises(ValueError):
                build_goal(self.claim(), pose)
