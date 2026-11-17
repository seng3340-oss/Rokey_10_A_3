"""drawing_geometry.py 테스트 — 원통형(cylinder) 지오메트리 + 쿼터니언 + 펜 기울기.

순수 Python이므로 ROS 없이 pytest로 바로 실행 가능.
  pytest test_drawing_geometry.py -v
"""

import math
import pytest

from cake_decorator.drawing_geometry import (
    CylinderCake,
    direction_to_quaternion,
    pen_quat_side,
    pen_quat_top,
    pixel_to_mm_side,
    pixel_to_mm_top,
)


# =====================================================================
# 테스트 헬퍼
# =====================================================================


def _rotate_vector_by_quat(
    v: tuple[float, float, float],
    q: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """쿼터니언으로 벡터를 회전: v' = q * v * q_inv. (테스트 검증용)"""
    qx, qy, qz, qw = q
    vx, vy, vz = v

    tw = -(qx * vx + qy * vy + qz * vz)
    tx = qw * vx + qy * vz - qz * vy
    ty = qw * vy + qz * vx - qx * vz
    tz = qw * vz + qx * vy - qy * vx

    rx = tw * (-qx) + tx * qw + ty * (-qz) - tz * (-qy)
    ry = tw * (-qy) + ty * qw + tz * (-qx) - tx * (-qz)
    rz = tw * (-qz) + tz * qw + tx * (-qy) - ty * (-qx)

    return (rx, ry, rz)


# =====================================================================
# 픽셀 → mm 변환 테스트
# =====================================================================


class TestPixelToMmTop:
    """TOP 캔버스 픽셀 → mm 변환."""

    def test_center_pixel_gives_zero(self):
        """캔버스 정중앙 → (0, 0) mm."""
        mx, my = pixel_to_mm_top(250, 250, 500, 500, radius=100.0)
        assert abs(mx) < 1e-9
        assert abs(my) < 1e-9

    def test_top_left_pixel(self):
        """왼쪽 위 (0,0) → (-radius, +radius)."""
        mx, my = pixel_to_mm_top(0, 0, 500, 500, radius=100.0)
        assert pytest.approx(mx, abs=1e-6) == -100.0
        assert pytest.approx(my, abs=1e-6) == 100.0

    def test_bottom_right_pixel(self):
        """오른쪽 아래 (500,500) → (+radius, -radius)."""
        mx, my = pixel_to_mm_top(500, 500, 500, 500, radius=100.0)
        assert pytest.approx(mx, abs=1e-6) == 100.0
        assert pytest.approx(my, abs=1e-6) == -100.0

    def test_non_square_canvas(self):
        """비정사각형 캔버스: scale = 2*radius / min(w, h)."""
        # canvas 800x400, radius=50 → scale = 100/400 = 0.25
        mx, my = pixel_to_mm_top(400, 200, 800, 400, radius=50.0)
        assert abs(mx) < 1e-9
        assert abs(my) < 1e-9

    def test_quarter_point(self):
        """캔버스 1/4 지점."""
        # canvas 400x400, radius=80 → scale = 160/400 = 0.4
        mx, my = pixel_to_mm_top(100, 100, 400, 400, radius=80.0)
        assert pytest.approx(mx, abs=1e-6) == -40.0
        assert pytest.approx(my, abs=1e-6) == 40.0


class TestPixelToMmSide:
    """SIDE 캔버스 픽셀 → mm (호 길이, 높이) 변환."""

    def test_origin_pixel(self):
        """왼쪽 위 (0,0) → arc=0, height=cake_height."""
        arc, h = pixel_to_mm_side(0, 0, 600, 300, radius=50.0, height=100.0)
        assert abs(arc) < 1e-9
        assert pytest.approx(h, abs=1e-6) == 100.0

    def test_bottom_right_pixel(self):
        """오른쪽 아래 (600,300) → arc=2πr, height=0."""
        arc, h = pixel_to_mm_side(600, 300, 600, 300, radius=50.0, height=100.0)
        assert pytest.approx(arc, abs=1e-3) == 2 * math.pi * 50.0
        assert abs(h) < 1e-9

    def test_mid_pixel(self):
        """중앙 → arc=πr, height=50."""
        arc, h = pixel_to_mm_side(300, 150, 600, 300, radius=50.0, height=100.0)
        assert pytest.approx(arc, abs=1e-3) == math.pi * 50.0
        assert pytest.approx(h, abs=1e-6) == 50.0


# =====================================================================
# CylinderCake 테스트
# =====================================================================


class TestCylinderCake:
    """원통형 케이크 좌표 변환."""

    @pytest.fixture
    def cake(self):
        return CylinderCake(radius=100.0, height=80.0, cx=500.0, cy=300.0, cz=200.0)

    # -- TOP --

    def test_top_center(self, cake):
        x, y, z = cake.top_to_abs(0.0, 0.0)
        assert pytest.approx(x) == 500.0
        assert pytest.approx(y) == 300.0
        assert pytest.approx(z) == 280.0  # 200 + 80

    def test_top_offset(self, cake):
        x, y, z = cake.top_to_abs(10.0, -20.0)
        assert pytest.approx(x) == 510.0
        assert pytest.approx(y) == 280.0
        assert pytest.approx(z) == 280.0

    # -- SIDE --

    def test_side_arc_zero(self, cake):
        """arc=0, θ=0 → (cx+r, cy, cz+h)."""
        x, y, z = cake.side_to_abs(0.0, 40.0)
        assert pytest.approx(x) == 600.0
        assert pytest.approx(y) == 300.0
        assert pytest.approx(z) == 240.0

    def test_side_quarter_turn(self, cake):
        """θ=π/2 → (cx, cy+r, cz+h)."""
        arc = math.pi / 2 * 100.0
        x, y, z = cake.side_to_abs(arc, 0.0)
        assert pytest.approx(x, abs=1e-6) == 500.0
        assert pytest.approx(y, abs=1e-6) == 400.0
        assert pytest.approx(z) == 200.0

    def test_side_half_turn(self, cake):
        """θ=π → (cx-r, cy, cz+h)."""
        arc = math.pi * 100.0
        x, y, z = cake.side_to_abs(arc, 80.0)
        assert pytest.approx(x, abs=1e-6) == 400.0
        assert pytest.approx(y, abs=1e-3) == 300.0
        assert pytest.approx(z) == 280.0

    def test_side_radius_constant_at_all_heights(self, cake):
        """원통형: 모든 높이에서 반지름 동일."""
        x0, _, _ = cake.side_to_abs(0.0, 0.0)
        x40, _, _ = cake.side_to_abs(0.0, 40.0)
        x80, _, _ = cake.side_to_abs(0.0, 80.0)
        assert pytest.approx(x0) == x40
        assert pytest.approx(x40) == x80

    # -- SIDE NORMAL --

    def test_side_normal_always_horizontal(self, cake):
        """원통형 법선: Z 성분 항상 0."""
        for arc in [0.0, 50.0, 150.0, 300.0]:
            nx, ny, nz = cake.side_outward_normal(arc)
            assert abs(nz) < 1e-10
            length = math.sqrt(nx**2 + ny**2 + nz**2)
            assert pytest.approx(length, abs=1e-9) == 1.0

    def test_side_normal_arc_zero(self, cake):
        """arc=0 → 법선 (1, 0, 0)."""
        nx, ny, nz = cake.side_outward_normal(0.0)
        assert pytest.approx(nx, abs=1e-9) == 1.0
        assert abs(ny) < 1e-9

    def test_side_normal_quarter(self, cake):
        """θ=π/2 → 법선 (0, 1, 0)."""
        arc = math.pi / 2 * 100.0
        nx, ny, nz = cake.side_outward_normal(arc)
        assert pytest.approx(nx, abs=1e-6) == 0.0
        assert pytest.approx(ny, abs=1e-6) == 1.0

    # -- SIDE WITH OFFSET --

    def test_side_offset_positive(self, cake):
        x, y, z = cake.side_abs_with_offset(0.0, 40.0, offset=10.0)
        assert pytest.approx(x) == 610.0
        assert pytest.approx(y) == 300.0
        assert pytest.approx(z) == 240.0

    def test_side_offset_zero_equals_surface(self, cake):
        x1, y1, z1 = cake.side_to_abs(50.0, 30.0)
        x2, y2, z2 = cake.side_abs_with_offset(50.0, 30.0, 0.0)
        assert pytest.approx(x1) == x2
        assert pytest.approx(y1) == y2
        assert pytest.approx(z1) == z2

    def test_side_offset_distance(self, cake):
        """오프셋 거리가 정확한지."""
        sx, sy, sz = cake.side_to_abs(0.0, 40.0)
        offset = 15.0
        ox, oy, oz = cake.side_abs_with_offset(0.0, 40.0, offset)
        dist = math.sqrt((ox - sx)**2 + (oy - sy)**2 + (oz - sz)**2)
        assert pytest.approx(dist) == offset


# =====================================================================
# direction_to_quaternion 테스트
# =====================================================================


class TestDirectionToQuaternion:

    def test_z_positive_identity(self):
        q = direction_to_quaternion(0.0, 0.0, 1.0)
        assert q == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-6)

    def test_z_negative_flip(self):
        q = direction_to_quaternion(0.0, 0.0, -1.0)
        assert q == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-6)

    def test_unit_quaternion(self):
        for dx, dy, dz in [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (1, 1, 1), (-1, -1, -1)]:
            qx, qy, qz, qw = direction_to_quaternion(dx, dy, dz)
            norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            assert norm == pytest.approx(1.0, abs=1e-6)

    def test_zero_vector_identity(self):
        q = direction_to_quaternion(0.0, 0.0, 0.0)
        assert q == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-6)

    def test_x_direction_rotate(self):
        """(1,0,0) → z축을 x축으로 회전."""
        q = direction_to_quaternion(1.0, 0.0, 0.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] == pytest.approx(1.0, abs=1e-6)
        assert rotated[1] == pytest.approx(0.0, abs=1e-6)
        assert rotated[2] == pytest.approx(0.0, abs=1e-6)

    def test_y_direction_rotate(self):
        q = direction_to_quaternion(0.0, 1.0, 0.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] == pytest.approx(0.0, abs=1e-6)
        assert rotated[1] == pytest.approx(1.0, abs=1e-6)
        assert rotated[2] == pytest.approx(0.0, abs=1e-6)

    def test_diagonal_direction(self):
        d = 1 / math.sqrt(3)
        q = direction_to_quaternion(d, d, d)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] == pytest.approx(d, abs=1e-6)
        assert rotated[1] == pytest.approx(d, abs=1e-6)
        assert rotated[2] == pytest.approx(d, abs=1e-6)


# =====================================================================
# 펜 자세 테스트
# =====================================================================


class TestPenQuatTop:
    """TOP 면 펜 자세 (기울기 포함)."""

    def test_no_tilt_points_down(self):
        """tilt=0 → 수직 아래 (0,0,-1)."""
        q = pen_quat_top(tilt_deg=0.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] == pytest.approx(0.0, abs=1e-6)
        assert rotated[1] == pytest.approx(0.0, abs=1e-6)
        assert rotated[2] == pytest.approx(-1.0, abs=1e-6)

    def test_tilt_30_changes_direction(self):
        """tilt=30 → 수직이 아닌 방향."""
        q0 = pen_quat_top(tilt_deg=0.0)
        q30 = pen_quat_top(tilt_deg=30.0)
        diff = sum(abs(a - b) for a, b in zip(q0, q30))
        assert diff > 0.01

    def test_tilt_30_z_component(self):
        """tilt=30 → Z 성분이 아직 음수 (아래를 향하지만 완전 수직은 아님)."""
        q = pen_quat_top(tilt_deg=30.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[2] < -0.5   # 여전히 대체로 아래
        assert abs(rotated[0]) > 0.1 or abs(rotated[1]) > 0.1  # 약간 기울어짐

    def test_tilt_30_angle_accuracy(self):
        """tilt=30 → 수직(0,0,-1)과의 각도가 30도."""
        q = pen_quat_top(tilt_deg=30.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        # 수직 (0,0,-1)과의 내적 → cos(각도)
        cos_angle = -(rotated[2])  # dot with (0,0,-1) = -rz
        angle_deg = math.degrees(math.acos(min(1.0, max(-1.0, cos_angle))))
        assert pytest.approx(angle_deg, abs=0.5) == 30.0

    def test_unit_quaternion(self):
        for tilt in [0.0, 15.0, 30.0, 45.0]:
            qx, qy, qz, qw = pen_quat_top(tilt_deg=tilt)
            norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            assert norm == pytest.approx(1.0, abs=1e-6)


class TestPenQuatSide:
    """SIDE 면 펜 자세 (기울기 포함)."""

    @pytest.fixture
    def cake(self):
        return CylinderCake(radius=100.0, height=80.0, cx=0, cy=0, cz=0)

    def test_no_tilt_points_inward_at_arc_zero(self, cake):
        """tilt=0, arc=0 → 펜이 -X 방향 (안쪽)."""
        q = pen_quat_side(0.0, cake, tilt_deg=0.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] == pytest.approx(-1.0, abs=1e-6)
        assert rotated[1] == pytest.approx(0.0, abs=1e-6)
        assert rotated[2] == pytest.approx(0.0, abs=1e-6)

    def test_no_tilt_points_inward_at_quarter(self, cake):
        """tilt=0, θ=π/2 → 펜이 -Y 방향 (안쪽)."""
        arc = math.pi / 2 * 100.0
        q = pen_quat_side(arc, cake, tilt_deg=0.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] == pytest.approx(0.0, abs=1e-6)
        assert rotated[1] == pytest.approx(-1.0, abs=1e-6)

    def test_tilt_30_has_downward_component(self, cake):
        """tilt=30 → 내향에서 30도 아래 → Z 성분 음수."""
        q = pen_quat_side(0.0, cake, tilt_deg=30.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[2] < -0.1   # 아래로 기울어짐
        assert rotated[0] < 0       # 여전히 안쪽을 향함

    def test_tilt_30_angle_accuracy(self, cake):
        """tilt=30 → 내향 법선과의 각도가 30도."""
        q = pen_quat_side(0.0, cake, tilt_deg=30.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        # 내향 법선 at arc=0: (-1, 0, 0)
        cos_angle = -rotated[0]  # dot with (-1,0,0)
        angle_deg = math.degrees(math.acos(min(1.0, max(-1.0, cos_angle))))
        assert pytest.approx(angle_deg, abs=0.5) == 30.0

    def test_different_arc_different_quat(self, cake):
        q1 = pen_quat_side(0.0, cake, tilt_deg=30.0)
        q2 = pen_quat_side(math.pi * 50.0, cake, tilt_deg=30.0)
        diff = sum(abs(a - b) for a, b in zip(q1, q2))
        assert diff > 0.01

    def test_unit_quaternion(self, cake):
        for arc in [0.0, 50.0, 200.0]:
            for tilt in [0.0, 15.0, 30.0]:
                qx, qy, qz, qw = pen_quat_side(arc, cake, tilt_deg=tilt)
                norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
                assert norm == pytest.approx(1.0, abs=1e-6)


# =====================================================================
# 통합 시나리오 테스트
# =====================================================================


class TestIntegration:

    def test_top_full_pipeline(self):
        """TOP: 픽셀 → mm → 절대좌표."""
        cake = CylinderCake(radius=75.0, height=60.0, cx=400.0, cy=300.0, cz=100.0)
        mx, my = pixel_to_mm_top(300, 300, 600, 600, cake.radius)
        x, y, z = cake.top_to_abs(mx, my)
        assert pytest.approx(x) == 400.0
        assert pytest.approx(y) == 300.0
        assert pytest.approx(z) == 160.0

    def test_side_full_pipeline(self):
        """SIDE: 픽셀 → mm → 절대좌표."""
        cake = CylinderCake(radius=50.0, height=100.0, cx=0.0, cy=0.0, cz=0.0)
        arc, h = pixel_to_mm_side(0, 0, 800, 400, cake.radius, cake.height)
        x, y, z = cake.side_to_abs(arc, h)
        assert pytest.approx(x) == 50.0
        assert pytest.approx(y, abs=1e-6) == 0.0
        assert pytest.approx(z) == 100.0

    def test_top_center_to_bottom_center(self):
        """TCP 윗면 중심 → 바닥 중심 변환 시뮬레이션."""
        top_x_m, top_y_m, top_z_m = 0.5, 0.3, 0.28
        height_m, radius_m = 0.08, 0.1

        cake = CylinderCake(
            radius=radius_m * 1000,
            height=height_m * 1000,
            cx=top_x_m * 1000,
            cy=top_y_m * 1000,
            cz=(top_z_m - height_m) * 1000,  # 바닥 중심
        )

        x, y, z = cake.top_to_abs(0.0, 0.0)
        assert pytest.approx(x) == 500.0
        assert pytest.approx(y) == 300.0
        assert pytest.approx(z) == 280.0  # 원래 윗면 Z

    def test_cylinder_side_consistent_at_all_heights(self):
        """원통형: 모든 높이에서 표면 X 좌표 동일 (arc=0)."""
        cake = CylinderCake(radius=100.0, height=80.0, cx=0, cy=0, cz=0)
        xs = [cake.side_to_abs(0.0, h)[0] for h in [0, 20, 40, 60, 80]]
        for x in xs:
            assert pytest.approx(x) == 100.0

    def test_full_side_workflow(self):
        """SIDE 전체: 점 → 절대좌표 → 법선 → 오프셋 → 펜자세."""
        cake = CylinderCake(radius=100.0, height=80.0, cx=400.0, cy=300.0, cz=100.0)

        arc, h = 0.0, 40.0

        # 절대좌표
        px, py, pz = cake.side_to_abs(arc, h)
        assert pytest.approx(px) == 500.0  # 400 + 100
        assert pytest.approx(py) == 300.0
        assert pytest.approx(pz) == 140.0  # 100 + 40

        # 법선 (수평)
        nx, ny, nz = cake.side_outward_normal(arc)
        assert pytest.approx(nx) == 1.0
        assert abs(nz) < 1e-10

        # 접근 (10mm 바깥)
        ax, ay, az = cake.side_abs_with_offset(arc, h, 10.0)
        assert pytest.approx(ax) == 510.0

        # 펜 자세 (30도 기울기)
        q = pen_quat_side(arc, cake, tilt_deg=30.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] < 0   # 안쪽
        assert rotated[2] < 0   # 아래 기울기


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
