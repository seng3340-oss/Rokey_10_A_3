"""테스트 케이크2와 같은 mm 단순화 결과·오차 경계를 검증한다."""

import importlib.util
from pathlib import Path
import math
import unittest

from cake_decorator.path_simplification import simplify_stroke_mm


class SimplificationTests(unittest.TestCase):
    """직선은 줄이되 모서리·폐곡선·원본의 점 순서를 보존해야 한다."""

    def test_straight_line_keeps_two_endpoints(self):
        points = [(i / 10, 0.0) for i in range(1001)]
        result = simplify_stroke_mm(points)
        self.assertEqual(result.points, [points[0], points[-1]])
        self.assertAlmostEqual(result.max_error_mm, 0.0, places=10)

    def test_closed_letter_loop_is_not_collapsed_to_a_line(self):
        points = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)]
        self.assertEqual(simplify_stroke_mm(points).points, points)

    def test_duplicates_do_not_hide_error_or_remove_endpoints(self):
        points = [(0.0, 0.0), (0.001, 0.001), (5.0, 2.0), (5.0, 2.0), (10.0, 0.0)]
        result = simplify_stroke_mm(points)
        self.assertEqual(result.points[0], points[0])
        self.assertEqual(result.points[-1], points[-1])
        self.assertIn((5.0, 2.0), result.points)
        self.assertLessEqual(result.max_error_mm, 0.15)

    def test_matches_test_cake2_on_same_physical_coordinates(self):
        path = Path.home() / "test_custom_mini_cake2/drawing_core/geometry.py"
        if not path.exists():
            self.skipTest("참고 테스트 프로젝트가 없는 PC")
        spec = importlib.util.spec_from_file_location("legacy_cake_geometry", path)
        legacy = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(legacy)
        # 0.01mm 중복 제거가 발생하지 않는 연속 곡선에서 RDP 결과를 직접 대조한다.
        points = [(i / 10, math.sin(i / 25)) for i in range(500)]
        self.assertEqual(
            simplify_stroke_mm(points).points, legacy.simplify(points, 0.15)
        )
