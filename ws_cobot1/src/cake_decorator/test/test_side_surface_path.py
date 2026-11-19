"""원통 옆면을 현으로 가로지르지 않는 원호·수직 이동을 검사한다."""

import math
import unittest

from cake_decorator.drawing_geometry import CylinderCake
from cake_decorator.surface_path import side_surface_leg, continuous_side_leg


class SideSurfaceTests(unittest.TestCase):
    def setUp(self):
        self.cake = CylinderCake(radius=50.0, height=80.0, cx=500.0, cy=0.0, cz=60.0)

    def assert_surface(self, moves, start, radius):
        """각 명령의 전체 위치 궤적을 샘플링해 반지름 안쪽 진입 여부를 확인한다."""
        previous = start
        for move in moves:
            end = move.end
            if move.via is not None:
                # 생성기가 보존한 비래핑 각도 범위로 긴 호·역방향도 검사한다.
                for i in range(101):
                    theta = (
                        move.start_angle + (move.end_angle - move.start_angle) * i / 100
                    )
                    x, y = 500 + radius * math.cos(theta), radius * math.sin(theta)
                    self.assertGreaterEqual(math.hypot(x - 500, y), radius - 1e-8)
                self.assertAlmostEqual(previous[2], end[2])
                self.assertAlmostEqual(move.via[2], end[2])
            else:
                for i in range(11):
                    point = [
                        previous[j] + (end[j] - previous[j]) * i / 10 for j in range(3)
                    ]
                    self.assertGreaterEqual(
                        math.hypot(point[0] - 500, point[1]), radius - 1e-8
                    )
            previous = end

    def test_half_circle_uses_true_arcs_not_one_chord(self):
        moves = side_surface_leg(self.cake, (0.0, 40.0), (math.pi * 50, 40.0))
        self.assertEqual(len(moves), 2)
        self.assertTrue(all(move.via is not None for move in moves))
        self.assert_surface(moves, (550.0, 0.0, 100.0), 50.0)

    def test_diagonal_is_bounded_and_stays_on_surface(self):
        moves = side_surface_leg(self.cake, (0.0, 20.0), (80.0, 25.0), max_error_mm=0.1)
        self.assert_surface(moves, (550.0, 0.0, 80.0), 50.0)
        for move in moves:
            # 개발면 좌표상 원래 대각선과의 수직 거리는 허용치 이하여야 한다.
            self.assertLessEqual(
                abs((move.arc_mm / 80) * 5 + 80 - move.end[2]), 0.1 + 1e-8
            )
        self.assertAlmostEqual(moves[-1].end[2], 85.0)

    def test_full_turn_and_reverse_preserve_the_requested_direction(self):
        for end in (2 * math.pi * 50, -2 * math.pi * 50):
            moves = side_surface_leg(self.cake, (0.0, 40.0), (end, 40.0))
            self.assertEqual(len(moves), 4)
            self.assert_surface(moves, (550.0, 0.0, 100.0), 50.0)

    def test_vertical_line_is_linear_and_clearance_arc_is_outside(self):
        moves = side_surface_leg(self.cake, (0.0, 40.0), (0.0, 60.0))
        self.assertEqual(len(moves), 1)
        self.assertIsNone(moves[0].via)
        clear = side_surface_leg(
            self.cake, (0.0, 40.0), (math.pi * 50, 60.0), offset_mm=10.0
        )
        self.assert_surface(clear, (560.0, 0.0, 100.0), 60.0)

    def test_negative_penetration_and_excessive_command_count_are_rejected(self):
        with self.assertRaises(ValueError):
            side_surface_leg(self.cake, (0.0, 0.0), (1.0, 1.0), offset_mm=-0.01)
        with self.assertRaises(ValueError):
            side_surface_leg(self.cake, (0.0, 0.0), (300.0, 80.0), max_error_mm=1e-9)

    def test_continuous_helix_samples_keep_radius_and_ordered_height(self):
        moves = continuous_side_leg(self.cake, (0.0, 10.0), (2 * math.pi * 50, 70.0))
        samples = [point for move in moves for point in move.samples]
        self.assertGreater(len(moves), 1)
        self.assertTrue(all(2 <= len(move.samples) <= 100 for move in moves))
        self.assertTrue(all(a[2] < b[2] for a, b in zip(samples, samples[1:])))
        for point in samples:
            self.assertAlmostEqual(math.hypot(point[0] - 500, point[1]), 50.0)
        self.assertAlmostEqual(samples[-1][2], 130.0)

    def test_large_spline_chunks_never_leave_a_singleton(self):
        """100개 제한의 경계에서도 마지막 한 점 때문에 전체 획이 거부되면 안 된다."""
        from cake_decorator.surface_path import split_spline_samples

        samples = list(range(9901))
        groups = split_spline_samples(samples)
        self.assertTrue(all(2 <= len(group) <= 100 for group in groups))
        self.assertEqual([point for group in groups for point in group], samples)


def test_short_pen_up_diagonal_uses_outer_tangents_not_tiny_arcs():
    """높이차가 큰 짧은 획간 이동이 미세 원호 수백 개로 분할되지 않아야 한다."""
    from cake_decorator.frustum_path import DrawingFrustum
    from cake_decorator.surface_path import side_clearance_leg

    cake = DrawingFrustum(r_top=56.75, r_bottom=60., height=79., cx=400., cy=0., cz=141.)
    start, end = (0., 70.), (1., 20.)
    moves = side_clearance_leg(cake, start, end, 10.)
    assert len(moves) == 4
    assert all(move.via is None for move in moves)
    previous = cake.side_abs_with_offset(*start, 10.)
    for move in moves:
        for i in range(101):
            point = tuple(a + (b-a)*i/100 for a, b in zip(previous, move.end))
            if cake.cz <= point[2] <= cake.cz + cake.height:
                assert math.hypot(point[0]-cake.cx, point[1]-cake.cy) >= cake.radius_at(point[2]-cake.cz)
        previous = move.end
    assert moves[-1].end == cake.side_abs_with_offset(*end, 10.)


def test_y_inset_moves_both_sides_without_changing_x_or_z():
    """각 높이에서 Y 양끝만 각각 1mm 줄이고 X 양끝·높이는 보존한다."""
    from dataclasses import replace
    from cake_decorator.frustum_path import DrawingFrustum

    original = DrawingFrustum(
        r_top=56.75, r_bottom=60., height=79.,
        cx=400., cy=-50., cz=141., radial_offset_mm=4.,
    )
    corrected = replace(original, side_y_inset_mm=1.)
    for height in (0., 40., 79.):
        for angle in (0., math.pi / 2, math.pi, 3 * math.pi / 2):
            arc = (angle - original.yaw) * original.radius
            before = original.side_to_abs(arc, height)
            after = corrected.side_to_abs(arc, height)
            assert abs(after[0] - before[0]) < 1e-9
            assert abs(after[2] - before[2]) < 1e-9
            assert abs(after[1] - before[1] + math.sin(angle)) < 1e-9
            # 접근·이탈의 공중점에도 같은 보정을 적용해 접촉점과 이어지게 한다.
            clear_before = original.side_abs_with_offset(arc, height, 10.)
            clear_after = corrected.side_abs_with_offset(arc, height, 10.)
            assert abs(clear_after[1] - clear_before[1] + math.sin(angle)) < 1e-9
