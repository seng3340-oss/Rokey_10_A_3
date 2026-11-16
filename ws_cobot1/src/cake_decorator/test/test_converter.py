"""converter.py 단위 테스트.

ROS 없이 pytest만으로 실행한다:
  cd ws_cobot1/src/cake_decorator
  python -m pytest test/ -v
"""

import pytest

from cake_decorator.drawing_geometry import (
    CakePose,
    ConvertConfig,
    Point2D,
    Stroke2D,
    generate_trajectory,
    pixel_to_side,
    pixel_to_top,
)


# ---------------------------------------------------------------------------
# 공통 픽스처
# ---------------------------------------------------------------------------

CAKE = CakePose(x=0.4, y=0.0, z=0.15)  # 로봇 앞 40cm, 윗면 높이 15cm
RADIUS = 0.10   # 10cm
HEIGHT = 0.08   # 8cm
CANVAS = 500    # 500x500 px


def _make_stroke(points_xy: list[tuple[float, float]], sid: int = 0) -> Stroke2D:
    """테스트용 Stroke2D 헬퍼."""

    return Stroke2D(
        stroke_id=sid,
        points=[Point2D(x=x, y=y) for x, y in points_xy],
    )


# ---------------------------------------------------------------------------
# pixel_to_top 테스트
# ---------------------------------------------------------------------------


class TestPixelToTop:
    """pixel_to_top 함수 테스트."""

    def test_center_maps_to_cake_center(self):
        """캔버스 정중앙 → 케이크 중심."""
        x, y, z = pixel_to_top(250, 250, CANVAS, CANVAS, CAKE, RADIUS)
        assert abs(x - CAKE.x) < 1e-9
        assert abs(y - CAKE.y) < 1e-9
        assert abs(z - CAKE.z) < 1e-9

    def test_right_edge(self):
        """캔버스 오른쪽 끝 → 케이크 +X 방향 가장자리."""
        x, y, z = pixel_to_top(CANVAS, 250, CANVAS, CANVAS, CAKE, RADIUS)
        assert x == pytest.approx(CAKE.x + RADIUS, abs=1e-9)
        assert y == pytest.approx(CAKE.y, abs=1e-9)

    def test_bottom_edge(self):
        """캔버스 아래쪽 끝 → 로봇 -Y 방향 (Y 반전)."""
        x, y, z = pixel_to_top(250, CANVAS, CANVAS, CANVAS, CAKE, RADIUS)
        assert x == pytest.approx(CAKE.x, abs=1e-9)
        assert y == pytest.approx(CAKE.y - RADIUS, abs=1e-9)


    def test_top_left_corner(self):
        """캔버스 (0,0) → (-radius, +radius) 방향."""
        x, y, z = pixel_to_top(0, 0, CANVAS, CANVAS, CAKE, RADIUS)
        assert x == pytest.approx(CAKE.x - RADIUS, abs=1e-9)
        assert y == pytest.approx(CAKE.y + RADIUS, abs=1e-9)


# ---------------------------------------------------------------------------
# pixel_to_side 테스트
# ---------------------------------------------------------------------------


class TestPixelToSide:
    """pixel_to_side 함수 테스트."""

    def test_left_edge_angle_zero(self):
        """캔버스 x=0 → 각도 0 (orientation=0일 때 +X 방향)."""
        x, y, z = pixel_to_side(0, 250, CANVAS, CANVAS, CAKE, RADIUS, HEIGHT)
        assert x == pytest.approx(CAKE.x + RADIUS, abs=1e-6)
        assert y == pytest.approx(CAKE.y, abs=1e-6)

    def test_quarter_angle(self):
        """캔버스 x=125 (1/4) → 각도 π/2 → +Y 방향."""
        x, y, z = pixel_to_side(125, 250, CANVAS, CANVAS, CAKE, RADIUS, HEIGHT)
        assert x == pytest.approx(CAKE.x, abs=1e-6)
        assert y == pytest.approx(CAKE.y + RADIUS, abs=1e-6)

    def test_z_range(self):
        """캔버스 y=0 → 윗면 근처, y=max → 밑면 근처."""
        _, _, z_top = pixel_to_side(0, 0, CANVAS, CANVAS, CAKE, RADIUS, HEIGHT)
        _, _, z_bot = pixel_to_side(0, CANVAS, CANVAS, CANVAS, CAKE, RADIUS, HEIGHT)

        # 마진 적용으로 정확히 높이 끝은 아님
        assert z_top > z_bot
        assert z_top < CAKE.z  # 윗면보다는 아래
        assert z_bot > CAKE.z - HEIGHT  # 밑면보다는 위


# ---------------------------------------------------------------------------
# generate_trajectory 통합 테스트
# ---------------------------------------------------------------------------


class TestGenerateTrajectory:
    """generate_trajectory 함수 테스트."""

    def test_simple_top_stroke(self):
        """윗면에 한 획 → 성공, 웨이포인트 생성."""
        stroke = _make_stroke([(100, 100), (200, 200), (300, 100)])
        result = generate_trajectory(
            CANVAS, CANVAS, [stroke], 'TOP', CAKE, RADIUS, HEIGHT,
        )
        assert result.success
        assert len(result.waypoints) > 0
        assert len(result.pen_states) == len(result.waypoints)
        assert result.estimated_duration_sec > 0

    def test_pen_up_down_pattern(self):
        """한 획: 접근(up) → 하강(down) → 그리기(down) → 올림(up)."""
        stroke = _make_stroke([(100, 100), (200, 200)])
        result = generate_trajectory(
            CANVAS, CANVAS, [stroke], 'TOP', CAKE, RADIUS, HEIGHT,
        )
        assert result.success

        states = result.pen_states
        # 접근(False), 하강(True), 그리기(True), 올림(False)
        assert states[0] is False   # 접근
        assert states[1] is True    # 하강
        assert states[2] is True    # 드로잉
        assert states[-1] is False  # 올림

    def test_two_strokes_have_travel_between(self):
        """두 획 사이에 펜 올림 이동이 있다."""
        s1 = _make_stroke([(100, 100), (200, 200)], sid=0)
        s2 = _make_stroke([(300, 300), (400, 400)], sid=1)
        result = generate_trajectory(
            CANVAS, CANVAS, [s1, s2], 'TOP', CAKE, RADIUS, HEIGHT,
        )
        assert result.success
        # 첫 획 마지막은 pen_up, 두번째 획 처음도 pen_up
        pen = result.pen_states
        # pen_up 구간이 있어야 함
        assert False in pen

    def test_side_surface(self):
        """옆면 변환도 성공한다."""
        stroke = _make_stroke([(50, 50), (450, 450)])
        result = generate_trajectory(
            CANVAS, CANVAS, [stroke], 'SIDE', CAKE, RADIUS, HEIGHT,
        )
        assert result.success
        assert len(result.waypoints) > 0

    def test_invalid_surface(self):
        """잘못된 surface_type → 실패."""
        stroke = _make_stroke([(100, 100), (200, 200)])

        result = generate_trajectory(
            CANVAS, CANVAS, [stroke], 'BOTTOM', CAKE, RADIUS, HEIGHT,
        )
        assert not result.success
        assert 'surface_type' in result.error_msg

    def test_empty_strokes(self):
        """빈 스트로크 목록 → 실패."""
        result = generate_trajectory(CANVAS, CANVAS, [], 'TOP', CAKE, RADIUS, HEIGHT)
        assert not result.success

    def test_single_point_stroke_skipped(self):
        """점 1개짜리 획은 건너뛴다."""
        stroke = Stroke2D(stroke_id=0, points=[Point2D(100, 100)])
        result = generate_trajectory(
            CANVAS, CANVAS, [stroke], 'TOP', CAKE, RADIUS, HEIGHT,
        )
        # 유효한 웨이포인트 없음
        assert not result.success

    def test_waypoint_z_matches_surface(self):
        """윗면 드로잉 중 Z가 cake.z 근처다."""
        stroke = _make_stroke([(250, 250), (300, 300)])
        config = ConvertConfig(drawing_z_offset=0.0, approach_height=0.01)
        result = generate_trajectory(
            CANVAS, CANVAS, [stroke], 'TOP', CAKE, RADIUS, HEIGHT, config,
        )
        # pen_down인 웨이포인트의 Z는 cake.z
        for wp, pen in zip(result.waypoints, result.pen_states):
            if pen:
                assert wp.pose.z == pytest.approx(CAKE.z, abs=1e-9)


    def test_lowercase_surface_accepted(self):
        """소문자 'top'도 동작한다."""
        stroke = _make_stroke([(100, 100), (200, 200)])
        result = generate_trajectory(
            CANVAS, CANVAS, [stroke], 'top', CAKE, RADIUS, HEIGHT,
        )
        assert result.success