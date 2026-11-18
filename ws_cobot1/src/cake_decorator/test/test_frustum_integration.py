"""서로 다른 반지름이 경로 보간에서 원통으로 축소되거나 각도가 틀어지는 회귀를 검출한다."""

import math

import pytest

from cake_decorator.frustum_path import DrawingFrustum, pixel_to_path_mm
from cake_decorator.surface_path import side_surface_leg, continuous_side_leg


def test_vertical_generator_keeps_angle_while_radius_changes():
    """고정 가로 픽셀의 위·아래 점은 같은 각도에서 반지름만 달라져야 한다."""
    cake = DrawingFrustum(56.75, 60.0, 79.0, 0.0, 0.0, 0.0)
    arc = cake.radius * math.pi / 2
    moves = side_surface_leg(cake, (arc, 79.0), (arc, 0.0))
    assert len(moves) == 1
    assert moves[0].end == pytest.approx((0.0, 60.0, 0.0))
    assert cake.side_to_abs(arc, 79.0) == pytest.approx((0.0, 56.75, 79.0))


def test_horizontal_circle_via_and_end_follow_midheight_radius():
    """PR #16의 CIRCLE 경유점이 원뿔대의 중간 반지름 위에 있어야 한다."""
    cake = DrawingFrustum(56.75, 60.0, 79.0, 0.0, 0.0, 0.0)
    moves = side_surface_leg(cake, (0.0, 39.5), (cake.radius * math.pi / 2, 39.5))
    assert len(moves) == 1
    assert moves[0].end == pytest.approx((0.0, 58.375, 39.5))
    assert math.hypot(*moves[0].via[:2]) == pytest.approx(58.375)
    assert moves[0].via[2] == pytest.approx(39.5)


def test_spline_samples_follow_sloped_surface():
    """연속 옆면 이동의 모든 목표점은 높이에 해당하는 반지름을 사용한다."""
    cake = DrawingFrustum(56.75, 60.0, 79.0, 0.0, 0.0, 0.0)
    moves = continuous_side_leg(cake, (0.0, 0.0), (cake.radius * math.pi / 2, 79.0))
    for move in moves:
        for x, y, z in move.samples:
            assert math.hypot(x, y) == pytest.approx(60.0 - 3.25 * z / 79.0)
    assert moves[-1].end == pytest.approx((0.0, 56.75, 79.0))


def test_pixel_mapping_keeps_same_angle_at_top_and_bottom():
    """화면의 같은 가로 위치를 높이와 무관한 경로 좌표로 전달한다."""
    cake = DrawingFrustum(56.75, 60.0, 79.0, 0.0, 0.0, 0.0)
    # 독립적으로 계산한 1024×768 편집 띠의 상·하단이다.
    top = pixel_to_path_mm(288.0, 287.4248167770543, 1024, 768, cake)
    bottom = pixel_to_path_mm(288.0, 480.5751832229457, 1024, 768, cake)
    assert top[0] == pytest.approx(bottom[0])
    assert top[1] == pytest.approx(79.0, abs=0.01)
    assert bottom[1] == pytest.approx(0.0, abs=0.01)
