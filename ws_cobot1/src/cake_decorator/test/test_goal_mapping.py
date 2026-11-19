"""로봇 없이 원본 픽셀·실측 치수·TCP 영점의 ROS 변환을 검사한다."""

from copy import deepcopy
import math
import unittest

from cake_decorator.goal_mapping import build_goal


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
                "diameter_mm": 100.0,
                "height_mm": 80.0,
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
        self.assertAlmostEqual(goal.cake_params.radius, 0.05)
        self.assertAlmostEqual(goal.cake_params.bottom_radius, 0.05)
        self.assertAlmostEqual(goal.cake_params.board_thickness, 0.002)
        self.assertEqual(goal.cake_params.reference_frame, "base")
        self.assertAlmostEqual(goal.cake_params.tool_orientation.y, 1.0)
        self.assertEqual(goal.drawing_paths[0].strokes[0].points[0].x, 500.5)
        self.assertEqual(goal.drawing_paths[0].surface, "TOP")
        self.assertNotIn(
            "is_closed", goal.drawing_paths[0].strokes[0].get_fields_and_field_types()
        )

    def test_mapping_preserves_frustum_top_and_bottom_radii(self):
        """원뿔대의 서로 다른 윗면·아랫면 지름을 ROS 반지름(m)으로 보낸다."""
        claim = self.claim()
        claim["cake"]["top_diameter_mm"] = claim["cake"].pop("diameter_mm")
        claim["cake"]["bottom_diameter_mm"] = 120.0
        goal = build_goal(claim, [363.9, 5.4, 75.25, 0.0, 180.0, 0.0])

        self.assertAlmostEqual(goal.cake_params.radius, 0.05)
        self.assertAlmostEqual(goal.cake_params.bottom_radius, 0.06)

    def test_invalid_or_missing_reference_never_becomes_zero_pose(self):
        """영점 누락·NaN을 BASE 원점으로 대체하지 않는다."""
        for pose in ([], [0.0] * 5, [0.0, 0.0, math.nan, 0.0, 0.0, 0.0]):
            with self.assertRaises(ValueError):
                build_goal(self.claim(), pose)

    def combined_claim(self):
        """TOP + 추가 SIDE의 두 면을 서로 다른 원본 좌표계로 준비한다."""
        claim = self.claim()
        side = deepcopy(claim["drawing"])
        side["drawing_surface"] = "side"
        side["canvas_width"] = 800
        side["strokes"][0]["points"][0]["x"] = 123.25
        claim["drawing"]["additional_paths"] = [side]
        return claim

    def test_combined_order_maps_top_then_side_in_one_goal(self):
        """주문 ID·케이크를 공유하며 두 면의 획 번호·해상도·좌표를 보존한다."""
        goal = build_goal(self.combined_claim(), [0.0] * 6)
        self.assertEqual(goal.order_id, "ORD-TEST-1")
        self.assertEqual([path.surface for path in goal.drawing_paths], ["TOP", "SIDE"])
        self.assertEqual([path.canvas_width for path in goal.drawing_paths], [1024, 800])
        self.assertEqual(
            [path.strokes[0].stroke_id for path in goal.drawing_paths], [0, 0]
        )
        self.assertEqual(goal.drawing_paths[0].strokes[0].points[0].x, 500.5)
        self.assertEqual(goal.drawing_paths[1].strokes[0].points[0].x, 123.25)

    def test_single_side_order_still_maps_one_path(self):
        """기존 SIDE 단독 요청에는 TOP 경로나 추가 동작을 만들지 않는다."""
        claim = self.claim()
        claim["drawing"]["drawing_surface"] = "side"
        goal = build_goal(claim, [0.0] * 6)
        self.assertEqual([path.surface for path in goal.drawing_paths], ["SIDE"])

    def test_additional_path_is_validated_before_goal_is_returned(self):
        """잘못된 추가 경로를 무시하고 TOP만 실행하는 부분 접수를 막는다."""
        for change in (
            {"drawing_surface": "top"},
            {"canvas_width": True},
            {"strokes": []},
            {"strokes": [{"stroke_id": 1, "points": [{"x": 1, "y": 1}] * 2}]},
            {"strokes": [{"stroke_id": 0, "points": [{"x": math.nan, "y": 1}] * 2}]},
            {"strokes": [{"stroke_id": 0, "points": [{"x": 801, "y": 1}] * 2}]},
            {"additional_paths": []},
        ):
            with self.subTest(change=change):
                claim = self.combined_claim()
                claim["drawing"]["additional_paths"][0].update(change)
                with self.assertRaises(ValueError):
                    build_goal(claim, [0.0] * 6)

    def test_combined_order_rejects_reversed_duplicate_and_null_paths(self):
        """브릿지에서도 TOP → SIDE 계약과 최대 두 면을 재검증한다."""
        for invalid_shape in ("reversed", "duplicate", "null"):
            with self.subTest(shape=invalid_shape):
                claim = self.combined_claim()
                drawing = claim["drawing"]
                if invalid_shape == "reversed":
                    drawing["drawing_surface"] = "side"
                    drawing["additional_paths"][0]["drawing_surface"] = "top"
                elif invalid_shape == "duplicate":
                    drawing["additional_paths"] *= 2
                else:
                    drawing["additional_paths"] = None
                with self.assertRaises(ValueError):
                    build_goal(claim, [0.0] * 6)

    def test_combined_order_uses_shared_point_and_stroke_limits(self):
        """면별 제한을 지켜도 두 면의 합계가 한도를 넘으면 Goal을 만들지 않는다."""
        for count, points in ((3, 4000), (51, 2)):
            with self.subTest(strokes=count, points=points):
                claim = self.combined_claim()
                drawing = claim["drawing"]
                for path in (drawing, drawing["additional_paths"][0]):
                    path["strokes"] = [
                        {"stroke_id": index, "points": [{"x": 1.0, "y": 1.0}] * points}
                        for index in range(count)
                    ]
                with self.assertRaises(ValueError):
                    build_goal(claim, [0.0] * 6)
