"""원뿔대 표면 좌표·픽셀 변환·펜 방향의 순수 Python 테스트."""

import math

import pytest

from cake_decorator.drawing_geometry import (
    FrustumCake,
    direction_to_quaternion,
    pen_quat_side,
    pen_quat_top,
    pixel_to_mm_side,
    pixel_to_mm_top,
)


def _rotate_vector_by_quat(vector, quaternion):
    """쿼터니언으로 벡터를 회전해 도구 Z축 방향을 확인한다."""
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector

    tw = -(qx * vx + qy * vy + qz * vz)
    tx = qw * vx + qy * vz - qz * vy
    ty = qw * vy + qz * vx - qx * vz
    tz = qw * vz + qx * vy - qy * vx

    return (
        tw * (-qx) + tx * qw + ty * (-qz) - tz * (-qy),
        tw * (-qy) + ty * qw + tz * (-qx) - tx * (-qz),
        tw * (-qz) + tz * qw + tx * (-qy) - ty * (-qx),
    )


@pytest.fixture
def cake():
    """역사적 실측 원뿔대 크기를 mm 단위로 구성한다."""
    return FrustumCake(
        r_top=56.75,
        r_bottom=60.0,
        height=79.0,
        cx=400.0,
        cy=0.0,
        cz=71.0,
    )


class TestFrustumCake:
    def test_radius_changes_linearly_from_bottom_to_top(self, cake):
        assert cake.radius_at(0.0) == pytest.approx(60.0)
        assert cake.radius_at(cake.height) == pytest.approx(56.75)
        assert cake.radius_at(cake.height / 2.0) == pytest.approx(58.375)

    def test_top_center_uses_bottom_center_and_height(self, cake):
        assert cake.top_to_abs(0.0, 0.0) == pytest.approx((400.0, 0.0, 150.0))

    def test_side_point_uses_radius_at_its_height(self, cake):
        x_bottom, y_bottom, z_bottom = cake.side_to_abs(0.0, 0.0)
        x_top, y_top, z_top = cake.side_to_abs(0.0, cake.height)

        assert (x_bottom, y_bottom, z_bottom) == pytest.approx(
            (460.0, 0.0, 71.0),
        )
        assert (x_top, y_top, z_top) == pytest.approx(
            (456.75, 0.0, 150.0),
        )

    def test_yaw_rotates_top_and_side_coordinates(self):
        rotated = FrustumCake(
            r_top=50.0,
            r_bottom=50.0,
            height=80.0,
            cx=0.0,
            cy=0.0,
            cz=0.0,
            yaw=math.pi / 2.0,
        )

        assert rotated.top_to_abs(10.0, 0.0) == pytest.approx(
            (0.0, 10.0, 80.0),
        )
        assert rotated.side_to_abs(0.0, 40.0) == pytest.approx(
            (0.0, 50.0, 40.0),
        )

    def test_side_normal_is_unit_and_tracks_slope(self, cake):
        nx, ny, nz = cake.side_outward_normal(0.0, 40.0)
        length = math.sqrt(nx * nx + ny * ny + nz * nz)

        assert length == pytest.approx(1.0)
        assert nx > 0.0
        assert ny == pytest.approx(0.0)
        assert nz > 0.0

    def test_surface_offset_follows_local_normal(self, cake):
        arc, height, offset = 123.0, 30.0, 10.0
        surface = cake.side_to_abs(arc, height)
        outside = cake.side_abs_with_offset(arc, height, offset)
        normal = cake.side_outward_normal(arc, height)

        assert tuple(out - point for out, point in zip(outside, surface)) == pytest.approx(
            tuple(component * offset for component in normal),
        )


class TestPixelMapping:
    def test_top_canvas_center_maps_to_top_center(self):
        assert pixel_to_mm_top(400.0, 300.0, 800.0, 600.0, 50.0) == pytest.approx(
            (0.0, 0.0),
        )

    def test_top_drawing_region_radius_maps_to_top_radius(self):
        assert pixel_to_mm_top(150.0, 300.0, 800.0, 600.0, 50.0) == pytest.approx(
            (-50.0, 0.0),
        )

    def test_side_top_and_bottom_map_to_frustum_heights(self, cake):
        region_width = 400.0 * (896.0 / 1024.0)
        region_height = (
            region_width
            * cake.slant_height
            / (math.pi * (cake.r_top + cake.r_bottom))
        )
        region_x = (400.0 - region_width) / 2.0
        region_y = (200.0 - region_height) / 2.0
        quarter_x = region_x + region_width / 4.0
        top_arc, top_height = pixel_to_mm_side(
            quarter_x, region_y, 400.0, 200.0, cake,
        )
        bottom_arc, bottom_height = pixel_to_mm_side(
            quarter_x, region_y + region_height, 400.0, 200.0, cake,
        )

        assert top_height == pytest.approx(cake.height)
        assert top_arc == pytest.approx(math.pi / 2.0 * cake.r_top)
        assert bottom_height == pytest.approx(0.0)
        assert bottom_arc == pytest.approx(math.pi / 2.0 * cake.r_bottom)

    def test_side_same_canvas_angle_is_preserved_at_each_height(self, cake):
        region_width = 400.0 * (896.0 / 1024.0)
        region_height = (
            region_width
            * cake.slant_height
            / (math.pi * (cake.r_top + cake.r_bottom))
        )
        region_x = (400.0 - region_width) / 2.0
        region_y = (200.0 - region_height) / 2.0
        quarter_x = region_x + region_width / 4.0
        top_arc, _ = pixel_to_mm_side(
            quarter_x, region_y, 400.0, 200.0, cake,
        )
        bottom_arc, _ = pixel_to_mm_side(
            quarter_x, region_y + region_height, 400.0, 200.0, cake,
        )

        assert top_arc / cake.r_top == pytest.approx(math.pi / 2.0)
        assert bottom_arc / cake.r_bottom == pytest.approx(math.pi / 2.0)

    def test_nonpositive_canvas_dimensions_are_rejected(self, cake):
        with pytest.raises(ValueError):
            pixel_to_mm_top(0.0, 0.0, 0.0, 100.0, cake.r_top)
        with pytest.raises(ValueError):
            pixel_to_mm_side(0.0, 0.0, 100.0, 0.0, cake)

    def test_side_points_outside_editing_strip_are_rejected(self, cake):
        with pytest.raises(ValueError, match="옆면 편집 영역"):
            pixel_to_mm_side(0.0, 300.0, 1024.0, 768.0, cake)


class TestPenOrientation:
    def test_top_pen_points_down(self):
        assert _rotate_vector_by_quat(
            (0.0, 0.0, 1.0),
            pen_quat_top(),
        ) == pytest.approx((0.0, 0.0, -1.0))

    def test_side_pen_axis_points_inward(self, cake):
        arc, height = 50.0, 35.0
        tool_axis = _rotate_vector_by_quat(
            (0.0, 0.0, 1.0),
            pen_quat_side(arc, height, cake),
        )
        outward = cake.side_outward_normal(arc, height)

        assert tool_axis == pytest.approx(tuple(-value for value in outward))

    def test_direction_quaternion_rotates_z_axis_to_target(self):
        direction = (1.0 / math.sqrt(3.0),) * 3
        quaternion = direction_to_quaternion(*direction)

        assert _rotate_vector_by_quat((0.0, 0.0, 1.0), quaternion) == pytest.approx(
            direction,
        )
