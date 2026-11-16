"""drawing_geometry.py 테스트 — 원뿔대(frustum) 지오메트리 + 쿼터니언.

순수 Python이므로 ROS 없이 pytest로 바로 실행 가능.
  pytest test_drawing_geometry.py -v
"""

import math
import pytest

from cake_decorator.drawing_geometry import (
    FrustumCake,
    direction_to_quaternion,
    pen_quat_side,
    pen_quat_top,
)


# =====================================================================
# 테스트 픽스처
# =====================================================================


@pytest.fixture
def cylinder_cake():
    """원통형 케이크 (r_top == r_bottom)."""
    return FrustumCake(
        r_top=100.0, r_bottom=100.0, height=80.0,
        cx=300.0, cy=200.0, cz=50.0,
    )


@pytest.fixture
def frustum_cake():
    """원뿔대 케이크 (r_top < r_bottom)."""
    return FrustumCake(
        r_top=80.0, r_bottom=120.0, height=100.0,
        cx=300.0, cy=200.0, cz=50.0,
    )


@pytest.fixture
def zero_height_cake():
    """높이 0인 케이크 (엣지 케이스)."""
    return FrustumCake(
        r_top=100.0, r_bottom=100.0, height=0.0,
        cx=0.0, cy=0.0, cz=0.0,
    )


# =====================================================================
# FrustumCake — radius_at
# =====================================================================


class TestRadiusAt:
    """높이별 반지름 계산 테스트."""

    def test_cylinder_any_height(self, cylinder_cake):
        """원통형: 어느 높이에서든 반지름 동일."""
        for y in [0.0, 20.0, 40.0, 80.0]:
            assert cylinder_cake.radius_at(y) == pytest.approx(100.0)

    def test_frustum_bottom(self, frustum_cake):
        """원뿔대: 바닥(y=0) → r_bottom."""
        assert frustum_cake.radius_at(0.0) == pytest.approx(120.0)

    def test_frustum_top(self, frustum_cake):
        """원뿔대: 꼭대기(y=height) → r_top."""
        assert frustum_cake.radius_at(100.0) == pytest.approx(80.0)

    def test_frustum_mid(self, frustum_cake):
        """원뿔대: 중간(y=50) → 선형 보간."""
        # r = 120 + (80-120) * 0.5 = 120 - 20 = 100
        assert frustum_cake.radius_at(50.0) == pytest.approx(100.0)

    def test_frustum_quarter(self, frustum_cake):
        """원뿔대: 1/4 높이(y=25) → 선형 보간."""
        # r = 120 + (80-120) * 0.25 = 120 - 10 = 110
        assert frustum_cake.radius_at(25.0) == pytest.approx(110.0)

    def test_clamp_below_zero(self, frustum_cake):
        """음수 높이 → y=0으로 클램프."""
        assert frustum_cake.radius_at(-10.0) == pytest.approx(120.0)

    def test_clamp_above_height(self, frustum_cake):
        """높이 초과 → y=height로 클램프."""
        assert frustum_cake.radius_at(200.0) == pytest.approx(80.0)

    def test_zero_height_cake(self, zero_height_cake):
        """높이 0 → 항상 r_bottom."""
        assert zero_height_cake.radius_at(0.0) == pytest.approx(100.0)
        assert zero_height_cake.radius_at(50.0) == pytest.approx(100.0)


# =====================================================================
# FrustumCake — slant_angle
# =====================================================================


class TestSlantAngle:
    """측면 경사각 테스트."""

    def test_cylinder_zero_slant(self, cylinder_cake):
        """원통형: 경사각 = 0."""
        assert cylinder_cake.slant_angle == pytest.approx(0.0)

    def test_frustum_positive_slant(self, frustum_cake):
        """r_bottom > r_top → 양수 경사각."""
        # atan2(120 - 80, 100) = atan2(40, 100)
        expected = math.atan2(40.0, 100.0)
        assert frustum_cake.slant_angle == pytest.approx(expected)

    def test_inverted_frustum(self):
        """r_top > r_bottom → 음수 경사각."""
        cake = FrustumCake(
            r_top=120.0, r_bottom=80.0, height=100.0,
            cx=0.0, cy=0.0, cz=0.0,
        )
        assert cake.slant_angle < 0

    def test_zero_height(self, zero_height_cake):
        """높이 0 → 경사각 0."""
        assert zero_height_cake.slant_angle == pytest.approx(0.0)


# =====================================================================
# FrustumCake — top_to_abs
# =====================================================================


class TestTopToAbs:
    """TOP 면 좌표 변환 테스트."""

    def test_center(self, frustum_cake):
        """윗면 중심 → cake_center + (0, 0, height)."""
        x, y, z = frustum_cake.top_to_abs(0.0, 0.0)
        assert x == pytest.approx(300.0)
        assert y == pytest.approx(200.0)
        assert z == pytest.approx(150.0)  # 50 + 100

    def test_offset(self, frustum_cake):
        """윗면에서 오프셋 → 절대좌표에 더해짐."""
        x, y, z = frustum_cake.top_to_abs(10.0, -5.0)
        assert x == pytest.approx(310.0)
        assert y == pytest.approx(195.0)
        assert z == pytest.approx(150.0)


# =====================================================================
# FrustumCake — side_to_abs
# =====================================================================


class TestSideToAbs:
    """SIDE 면 좌표 변환 테스트."""

    def test_arc_zero_bottom(self, frustum_cake):
        """x_arc=0, y=0 → (cx + r_bottom, cy, cz)."""
        x, y, z = frustum_cake.side_to_abs(0.0, 0.0)
        assert x == pytest.approx(300.0 + 120.0)  # cx + r_bottom
        assert y == pytest.approx(200.0)
        assert z == pytest.approx(50.0)

    def test_arc_zero_top(self, frustum_cake):
        """x_arc=0, y=height → (cx + r_top, cy, cz+height)."""
        x, y, z = frustum_cake.side_to_abs(0.0, 100.0)
        assert x == pytest.approx(300.0 + 80.0)  # cx + r_top
        assert y == pytest.approx(200.0)
        assert z == pytest.approx(150.0)

    def test_quarter_turn_bottom(self, frustum_cake):
        """x_arc = π/2 * r_bottom → 90도 회전 (바닥)."""
        r = 120.0
        x_arc = math.pi / 2 * r  # θ = π/2
        x, y, z = frustum_cake.side_to_abs(x_arc, 0.0)
        assert x == pytest.approx(300.0, abs=0.1)  # cos(π/2) ≈ 0
        assert y == pytest.approx(200.0 + 120.0, abs=0.1)  # sin(π/2) = 1
        assert z == pytest.approx(50.0)

    def test_radius_varies_with_height(self, frustum_cake):
        """높이에 따라 반지름이 달라짐을 확인."""
        # 바닥: r=120, x_arc=0 → x = cx+120
        x_bot, _, _ = frustum_cake.side_to_abs(0.0, 0.0)
        # 꼭대기: r=80, x_arc=0 → x = cx+80
        x_top, _, _ = frustum_cake.side_to_abs(0.0, 100.0)
        assert x_bot > x_top  # 바닥이 더 멀리

    def test_cylinder_consistent_radius(self, cylinder_cake):
        """원통형: 바닥과 꼭대기 반지름 동일."""
        x_bot, _, _ = cylinder_cake.side_to_abs(0.0, 0.0)
        x_top, _, _ = cylinder_cake.side_to_abs(0.0, 80.0)
        assert x_bot == pytest.approx(x_top)


# =====================================================================
# FrustumCake — side_outward_normal
# =====================================================================


class TestSideOutwardNormal:
    """SIDE 면 법선 벡터 테스트."""

    def test_cylinder_horizontal(self, cylinder_cake):
        """원통형: 법선이 수평 (z 성분 = 0)."""
        nx, ny, nz = cylinder_cake.side_outward_normal(0.0, 40.0)
        assert nz == pytest.approx(0.0, abs=1e-10)

    def test_cylinder_unit_vector(self, cylinder_cake):
        """법선이 단위 벡터인지 확인."""
        nx, ny, nz = cylinder_cake.side_outward_normal(0.0, 40.0)
        length = math.sqrt(nx**2 + ny**2 + nz**2)
        assert length == pytest.approx(1.0)

    def test_frustum_has_z_component(self, frustum_cake):
        """원뿔대: 법선에 z 성분 존재 (경사)."""
        nx, ny, nz = frustum_cake.side_outward_normal(0.0, 50.0)
        assert nz > 0  # r_bottom > r_top → 법선이 위를 향함

    def test_frustum_unit_vector(self, frustum_cake):
        """원뿔대 법선이 단위 벡터인지 확인."""
        nx, ny, nz = frustum_cake.side_outward_normal(30.0, 50.0)
        length = math.sqrt(nx**2 + ny**2 + nz**2)
        assert length == pytest.approx(1.0)

    def test_arc_zero_points_x(self, cylinder_cake):
        """x_arc=0 → 법선이 +X 방향."""
        nx, ny, nz = cylinder_cake.side_outward_normal(0.0, 40.0)
        assert nx == pytest.approx(1.0, abs=1e-6)
        assert ny == pytest.approx(0.0, abs=1e-6)

    def test_quarter_turn_points_y(self, cylinder_cake):
        """θ=π/2 → 법선이 +Y 방향."""
        r = 100.0
        x_arc = math.pi / 2 * r
        nx, ny, nz = cylinder_cake.side_outward_normal(x_arc, 40.0)
        assert nx == pytest.approx(0.0, abs=1e-6)
        assert ny == pytest.approx(1.0, abs=1e-6)


# =====================================================================
# FrustumCake — side_abs_with_offset
# =====================================================================


class TestSideAbsWithOffset:
    """법선 방향 오프셋 테스트."""

    def test_zero_offset_equals_surface(self, frustum_cake):
        """offset=0 → 표면 위치와 동일."""
        sx, sy, sz = frustum_cake.side_to_abs(10.0, 50.0)
        ox, oy, oz = frustum_cake.side_abs_with_offset(10.0, 50.0, 0.0)
        assert ox == pytest.approx(sx)
        assert oy == pytest.approx(sy)
        assert oz == pytest.approx(sz)

    def test_positive_offset_moves_outward(self, cylinder_cake):
        """양수 offset → 표면 바깥으로."""
        sx, sy, sz = cylinder_cake.side_to_abs(0.0, 40.0)
        ox, oy, oz = cylinder_cake.side_abs_with_offset(0.0, 40.0, 10.0)
        # x_arc=0 → 법선이 +X 방향 → ox > sx
        assert ox > sx
        assert oy == pytest.approx(sy)

    def test_offset_distance(self, cylinder_cake):
        """오프셋 거리가 정확한지 확인."""
        sx, sy, sz = cylinder_cake.side_to_abs(0.0, 40.0)
        offset = 15.0
        ox, oy, oz = cylinder_cake.side_abs_with_offset(0.0, 40.0, offset)
        dist = math.sqrt((ox - sx)**2 + (oy - sy)**2 + (oz - sz)**2)
        assert dist == pytest.approx(offset)


# =====================================================================
# direction_to_quaternion
# =====================================================================


class TestDirectionToQuaternion:
    """방향벡터 → 쿼터니언 변환 테스트."""

    def test_z_positive_identity(self):
        """(0,0,1) → 항등 쿼터니언."""
        q = direction_to_quaternion(0.0, 0.0, 1.0)
        assert q == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-6)

    def test_z_negative_flip(self):
        """(0,0,-1) → 180° 회전."""
        q = direction_to_quaternion(0.0, 0.0, -1.0)
        assert q == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-6)

    def test_unit_quaternion(self):
        """결과가 항상 단위 쿼터니언인지 확인."""
        directions = [
            (1, 0, 0), (0, 1, 0), (-1, 0, 0),
            (1, 1, 0), (1, 1, 1), (-1, -1, -1),
        ]
        for dx, dy, dz in directions:
            qx, qy, qz, qw = direction_to_quaternion(dx, dy, dz)
            norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            assert norm == pytest.approx(1.0, abs=1e-6), \
                f'direction ({dx},{dy},{dz}) → norm={norm}'

    def test_zero_vector_identity(self):
        """영벡터 → 항등 쿼터니언."""
        q = direction_to_quaternion(0.0, 0.0, 0.0)
        assert q == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-6)

    def test_x_direction(self):
        """(1,0,0) → 쿼터니언으로 z→x 회전."""
        qx, qy, qz, qw = direction_to_quaternion(1.0, 0.0, 0.0)
        # 회전 적용: q * (0,0,1) * q_inv ≈ (1,0,0)
        rotated = _rotate_vector_by_quat((0, 0, 1), (qx, qy, qz, qw))
        assert rotated[0] == pytest.approx(1.0, abs=1e-6)
        assert rotated[1] == pytest.approx(0.0, abs=1e-6)
        assert rotated[2] == pytest.approx(0.0, abs=1e-6)

    def test_y_direction(self):
        """(0,1,0) → 쿼터니언으로 z→y 회전."""
        qx, qy, qz, qw = direction_to_quaternion(0.0, 1.0, 0.0)
        rotated = _rotate_vector_by_quat((0, 0, 1), (qx, qy, qz, qw))
        assert rotated[0] == pytest.approx(0.0, abs=1e-6)
        assert rotated[1] == pytest.approx(1.0, abs=1e-6)
        assert rotated[2] == pytest.approx(0.0, abs=1e-6)

    def test_diagonal_direction(self):
        """대각선 방향도 정확히 회전."""
        d = 1 / math.sqrt(3)
        qx, qy, qz, qw = direction_to_quaternion(d, d, d)
        rotated = _rotate_vector_by_quat((0, 0, 1), (qx, qy, qz, qw))
        assert rotated[0] == pytest.approx(d, abs=1e-6)
        assert rotated[1] == pytest.approx(d, abs=1e-6)
        assert rotated[2] == pytest.approx(d, abs=1e-6)


# =====================================================================
# pen_quat_top / pen_quat_side
# =====================================================================


class TestPenOrientation:
    """펜 자세 테스트."""

    def test_top_points_down(self):
        """TOP: 펜이 아래를 향함 (Z축 → (0,0,-1))."""
        q = pen_quat_top()
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] == pytest.approx(0.0, abs=1e-6)
        assert rotated[1] == pytest.approx(0.0, abs=1e-6)
        assert rotated[2] == pytest.approx(-1.0, abs=1e-6)

    def test_side_points_inward_at_arc_zero(self, cylinder_cake):
        """SIDE(x_arc=0): 펜이 -X 방향 (안쪽)."""
        q = pen_quat_side(0.0, 40.0, cylinder_cake)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] == pytest.approx(-1.0, abs=1e-6)
        assert rotated[1] == pytest.approx(0.0, abs=1e-6)

    def test_side_points_inward_at_quarter(self, cylinder_cake):
        """SIDE(θ=π/2): 펜이 -Y 방향 (안쪽)."""
        x_arc = math.pi / 2 * 100.0  # r=100
        q = pen_quat_side(x_arc, 40.0, cylinder_cake)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] == pytest.approx(0.0, abs=1e-6)
        assert rotated[1] == pytest.approx(-1.0, abs=1e-6)

    def test_side_varies_by_position(self, cylinder_cake):
        """SIDE: 위치에 따라 자세가 달라짐."""
        q1 = pen_quat_side(0.0, 40.0, cylinder_cake)
        q2 = pen_quat_side(50.0, 40.0, cylinder_cake)
        # 다른 x_arc → 다른 쿼터니언
        assert q1 != q2

    def test_frustum_side_has_tilt(self, frustum_cake):
        """원뿔대 SIDE: 펜 방향에 Z 성분 존재 (경사 반영)."""
        q = pen_quat_side(0.0, 50.0, frustum_cake)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        # 내향 법선 = -외향 법선, 외향 법선의 z > 0
        # → 내향의 z < 0
        assert rotated[2] < 0


# =====================================================================
# 통합 테스트 — 실제 시나리오
# =====================================================================


class TestIntegration:
    """실제 케이크 시나리오 통합 테스트."""

    def test_full_side_workflow(self):
        """실제 케이크: SIDE 면 점 → 절대좌표 → 법선 → 펜 자세."""
        cake = FrustumCake(
            r_top=75.0, r_bottom=100.0, height=80.0,
            cx=400.0, cy=300.0, cz=100.0,
        )

        # 중간 높이, 호 길이 0
        y = 40.0
        x_arc = 0.0

        # 반지름 확인
        r = cake.radius_at(y)
        assert r == pytest.approx(87.5)  # 100 + (75-100)*0.5

        # 절대좌표 확인
        px, py, pz = cake.side_to_abs(x_arc, y)
        assert px == pytest.approx(400.0 + 87.5)
        assert py == pytest.approx(300.0)
        assert pz == pytest.approx(140.0)

        # 법선 확인 (단위벡터)
        nx, ny, nz = cake.side_outward_normal(x_arc, y)
        norm = math.sqrt(nx**2 + ny**2 + nz**2)
        assert norm == pytest.approx(1.0)
        assert nx > 0  # +X 방향
        assert nz > 0  # 위를 향함 (r_bottom > r_top)

        # 접근 위치 (10mm 바깥)
        ax, ay, az = cake.side_abs_with_offset(x_arc, y, 10.0)
        assert ax > px  # 바깥으로

        # 펜 자세 (내향)
        q = pen_quat_side(x_arc, y, cake)
        rotated = _rotate_vector_by_quat((0, 0, 1), q)
        assert rotated[0] < 0  # -X (안쪽)

    def test_cylinder_equals_same_radius(self):
        """원통형(r_top==r_bottom): 모든 높이에서 동일한 결과."""
        cake = FrustumCake(
            r_top=100.0, r_bottom=100.0, height=80.0,
            cx=0.0, cy=0.0, cz=0.0,
        )

        p1 = cake.side_to_abs(0.0, 0.0)
        p2 = cake.side_to_abs(0.0, 40.0)
        p3 = cake.side_to_abs(0.0, 80.0)

        # X좌표(반지름 방향) 동일
        assert p1[0] == pytest.approx(p2[0])
        assert p2[0] == pytest.approx(p3[0])

    def test_top_surface_independent_of_radius(self):
        """TOP 면: 반지름과 무관하게 중심 오프셋만 적용."""
        cake1 = FrustumCake(80, 120, 100, 0, 0, 0)
        cake2 = FrustumCake(50, 150, 100, 0, 0, 0)

        # 같은 오프셋 → 같은 절대좌표 (반지름 무관)
        p1 = cake1.top_to_abs(10.0, 5.0)
        p2 = cake2.top_to_abs(10.0, 5.0)
        assert p1 == pytest.approx(p2)


# =====================================================================
# 테스트 헬퍼
# =====================================================================


def _rotate_vector_by_quat(
    v: tuple[float, float, float],
    q: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """쿼터니언으로 벡터를 회전: v' = q * v * q_inv.

    테스트 검증용.
    """
    qx, qy, qz, qw = q
    vx, vy, vz = v

    # q * v (v를 순허수 쿼터니언으로)
    # (qw + qi + qj + qk) * (0 + vxi + vyj + vzk)
    tw = -(qx * vx + qy * vy + qz * vz)
    tx = qw * vx + qy * vz - qz * vy
    ty = qw * vy + qz * vx - qx * vz
    tz = qw * vz + qx * vy - qy * vx

    # (result) * q_inv, q_inv = (-qx, -qy, -qz, qw)
    rx = tw * (-qx) + tx * qw + ty * (-qz) - tz * (-qy)
    ry = tw * (-qy) + ty * qw + tz * (-qx) - tx * (-qz)
    rz = tw * (-qz) + tz * qw + tx * (-qy) - ty * (-qx)

    return (rx, ry, rz)