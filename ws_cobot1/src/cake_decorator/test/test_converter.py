"""경로 변환(2D → 3D) 테스트 — 픽셀 입력, 원뿔대·다중 획 처리.

과거 모듈 단위 `generate_trajectory()` 함수는 현재 `DrawingPathNode`의
서비스 콜백으로 대체됐으므로, 이 테스트도 노드 객체로 실제 요청을 만들어
통과시킨다. ROS 인터페이스 빌드가 필요하다.

실행:
  source /opt/ros/jazzy/setup.bash
  source ws_cobot1/install/local_setup.bash
  cd ws_cobot1/src/cake_decorator
  python -m pytest test/ -v
"""

import math

import pytest
import rclpy

from cake_decorator.drawing_geometry import pixel_to_mm_top
from cake_decorator.drawing_motion import plan_drawing_motion
from cake_decorator.drawing_path_node import DrawingPathNode
from cake_decorator_interfaces.msg import Stroke2D
from cake_decorator_interfaces.srv import GenerateTrajectory
from geometry_msgs.msg import Point32

# ---------------------------------------------------------------------------
# 공통 픽스처
# ---------------------------------------------------------------------------

# 캔버스는 프론트엔드 원본 해상도(왼쪽 위 = (0,0))를 그대로 쓴다.
CANVAS_W = 1024
CANVAS_H = 768
CANVAS_CX = CANVAS_W / 2.0
CANVAS_CY = CANVAS_H / 2.0

# 기본 케이크는 원통이며 윗면·아랫면 반지름이 같다.
CAKE_TOP = (0.4, 0.0, 0.15)   # x, y, z (m)
RADIUS_M = 0.05
HEIGHT_M = 0.08

# 윗면 원 안쪽 픽셀 좌표들. 세 번째 획까지 이어져야 정상이다.
STROKE_A = [(480.0, 300.0), (520.0, 340.0), (560.0, 300.0)]
STROKE_B = [(600.0, 420.0), (640.0, 460.0)]
STROKE_C = [(420.0, 470.0), (460.0, 500.0), (500.0, 470.0)]


@pytest.fixture(scope="module")
def dp_node():
    """서비스 콜백을 직접 호출할 수 있는 drawing_path_node 하나를 만든다.

    이미 rclpy가 켜져 있으면 그 컨텍스트를 그대로 쓰고, 테스트가 처음
    켠 경우에만 마지막에 종료한다.
    """
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = DrawingPathNode()
    yield node
    node.destroy_node()
    if owns_context:
        rclpy.shutdown()


def _request(strokes_px, surface="TOP"):
    """픽셀 획 목록으로 GenerateTrajectory 요청을 만든다.

    실제 파이프라인이 넘기는 값과 같은 형태다: 캔버스 픽셀(왼쪽 위 = (0,0)),
    케이크 윗면 중심(m), 반지름·높이(m), 해상도(px).
    """
    request = GenerateTrajectory.Request()
    request.order_id = "ORD-TEST"
    request.operation_speed_percent = 100
    request.surface_type = surface
    request.cake_pose.position.x = CAKE_TOP[0]
    request.cake_pose.position.y = CAKE_TOP[1]
    request.cake_pose.position.z = CAKE_TOP[2]
    request.cake_pose.orientation.w = 1.0
    request.cake_radius = RADIUS_M
    request.cake_bottom_radius = RADIUS_M
    request.cake_height = HEIGHT_M
    request.canvas_width = int(CANVAS_W)
    request.canvas_height = int(CANVAS_H)
    for stroke_id, points in enumerate(strokes_px):
        request.strokes_2d.append(
            Stroke2D(
                stroke_id=stroke_id,
                points=[
                    Point32(x=x, y=y, z=0.0) for x, y in points
                ],
            )
        )
    return request


def _convert(node, strokes_px, surface="TOP"):
    """요청을 노드 콜백에 넣어 응답을 돌려받는다."""
    return node._handle_request(
        _request(strokes_px, surface),
        GenerateTrajectory.Response(),
    )


def _pen_segments(pen_states):
    """펜이 처음 내려간 구간 수 = 실제로 그려질 획 수."""
    count = 0
    previous = False
    for pen in pen_states:
        if pen and not previous:
            count += 1
        previous = pen
    return count


def _executor_dicts(waypoints):
    """실행 노드가 읽는 형태(waypoint.pose → dict)로 바꾼다.

    robot_executor_node._convert_waypoints와 같은 값·순서를 만든다.
    로봇 드라이버 import가 없는 테스트에서도 계획을 세울 수 있게 여기서 만든다.
    """
    return [
        {
            "x": wp.pose.position.x,
            "y": wp.pose.position.y,
            "z": wp.pose.position.z,
            "qx": wp.pose.orientation.x,
            "qy": wp.pose.orientation.y,
            "qz": wp.pose.orientation.z,
            "qw": wp.pose.orientation.w,
            "speed": wp.speed,
            "pen_down": wp.pen_down,
            "contact_force": wp.contact_force,
        }
        for wp in waypoints
    ]


def _expected_top_xy(px, py):
    """픽셀 → 윗면 중심 기준 m 좌표 (기대값 계산용)."""
    mm_x, mm_y = pixel_to_mm_top(
        px, py, CANVAS_W, CANVAS_H, RADIUS_M * 1000.0,
    )
    return (
        (CAKE_TOP[0] * 1000.0 + mm_x) / 1000.0,
        (CAKE_TOP[1] * 1000.0 + mm_y) / 1000.0,
    )


# ---------------------------------------------------------------------------
# 펜 상태·획 수 테스트
# ---------------------------------------------------------------------------


class TestPenPattern:
    """펜 들기/내림 패턴이 획 단위로 올바르게 나오는지 검사한다."""

    def test_single_stroke_pen_pattern(self, dp_node):
        """한 획: 접근(up) → 하강(down) → 그리기(down) → 이탈(up)."""
        response = _convert(dp_node, [STROKE_A])
        assert response.success, response.error_msg

        states = list(response.pen_states)
        assert states[0] is False      # 접근
        assert states[1] is True       # 하강
        assert states[2] is True       # 첫 그리기 점
        assert states[-1] is False     # 이탈

    def test_pen_states_match_waypoints(self, dp_node):
        """응답 계약: pen_states와 waypoints_3d[i].pen_down이 같아야 한다."""
        response = _convert(dp_node, [STROKE_A, STROKE_B])
        assert response.success, response.error_msg
        assert len(response.pen_states) == len(response.waypoints_3d)
        assert list(response.pen_states) == [
            wp.pen_down for wp in response.waypoints_3d
        ]


class TestMultipleStrokes:
    """여러 획이 끝까지 남는지 검사한다. 첫 획만 남는 회귀를 막는 테스트다."""

    def test_three_strokes_become_three_pen_segments(self, dp_node):
        """요청한 세 획 모두 펜 내림 구간으로 살아남는다."""
        response = _convert(dp_node, [STROKE_A, STROKE_B, STROKE_C])
        assert response.success, response.error_msg
        assert _pen_segments(response.pen_states) == 3

    def test_stroke_order_is_preserved(self, dp_node):
        """획 순서와 각 획의 첫 점 위치가 요청 순서를 따른다."""
        strokes = [STROKE_A, STROKE_B, STROKE_C]
        response = _convert(dp_node, strokes)
        assert response.success, response.error_msg

        waypoints = list(response.waypoints_3d)
        # 각 획은 접근(up) → 하강(down) 순서로 시작하므로 두 번째 웨이포인트가
        # 그 획의 첫 점 위에 있다.
        starts = []
        index = 0
        for points in strokes:
            approach, pen_down = waypoints[index], waypoints[index + 1]
            assert approach.pen_down is False
            assert pen_down.pen_down is True
            starts.append(
                (pen_down.pose.position.x, pen_down.pose.position.y)
            )
            index += 1 + len(points) + 1  # 접근 + 하강 + 나머지 점 + 이탈

        for (px, py), (x, y) in zip(
            [points[0] for points in strokes], starts,
        ):
            expected = _expected_top_xy(px, py)
            assert x == pytest.approx(expected[0], abs=1e-9)
            assert y == pytest.approx(expected[1], abs=1e-9)

    def test_all_strokes_reach_motion_plan(self, dp_node):
        """실행 단계(동작 계획)에서도 획 수가 그대로 유지된다."""
        response = _convert(dp_node, [STROKE_A, STROKE_B, STROKE_C])
        assert response.success, response.error_msg

        plan = plan_drawing_motion(
            _executor_dicts(response.waypoints_3d),
            list(response.pen_states),
            "TOP",
        )
        assert plan.success, plan.error_msg
        assert plan.total_strokes == 3
        assert len(plan.commands) == len(response.waypoints_3d)

    def test_side_surface_keeps_all_strokes(self, dp_node):
        """옆면도 여러 획을 전부 변환한다."""
        side_strokes = [
            [(x, min(y, 490.0)) for x, y in stroke]
            for stroke in (STROKE_A, STROKE_B, STROKE_C)
        ]
        response = _convert(
            dp_node, side_strokes, surface="SIDE",
        )
        assert response.success, response.error_msg
        assert _pen_segments(response.pen_states) == 3

    def test_frustum_side_path_uses_radius_at_each_height(self, dp_node):
        """옆면의 같은 각도에서 위·아랫면 반지름과 높이가 다르게 반영된다."""
        radius_top = 56.75
        radius_bottom = 60.0
        cake_height = 79.0
        strip_width = CANVAS_W * (896.0 / 1024.0)
        slant_height = math.hypot(
            cake_height,
            radius_bottom - radius_top,
        )
        strip_height = (
            strip_width
            * slant_height
            / (math.pi * (radius_top + radius_bottom))
        )
        strip_top = (CANVAS_H - strip_height) / 2.0
        request = _request(
            [[
                (288.0, strip_top),
                (288.0, strip_top + strip_height),
            ]],
            surface="SIDE",
        )
        request.cake_radius = 0.05675
        request.cake_bottom_radius = 0.06
        request.cake_height = cake_height / 1000.0
        response = dp_node._handle_request(
            request,
            GenerateTrajectory.Response(),
        )

        assert response.success, response.error_msg
        contact_points = [wp for wp in response.waypoints_3d if wp.pen_down]
        assert len(contact_points) == 2
        assert contact_points[0].pose.position.x == pytest.approx(
            CAKE_TOP[0] + 0.05675 * math.cos(math.pi / 2.0),
            abs=3e-5,
        )
        assert contact_points[0].pose.position.y == pytest.approx(
            CAKE_TOP[1] + 0.05675,
            abs=3e-5,
        )
        assert contact_points[0].pose.position.z == pytest.approx(
            CAKE_TOP[2],
            abs=3e-5,
        )
        assert contact_points[1].pose.position.x == pytest.approx(
            CAKE_TOP[0] + 0.06 * math.cos(math.pi / 2.0),
            abs=3e-5,
        )
        assert contact_points[1].pose.position.y == pytest.approx(
            CAKE_TOP[1] + 0.06,
            abs=3e-5,
        )
        assert contact_points[1].pose.position.z == pytest.approx(
            CAKE_TOP[2] - cake_height / 1000.0,
            abs=3e-5,
        )


# ---------------------------------------------------------------------------
# 좌표·입력 검증 테스트
# ---------------------------------------------------------------------------


class TestConversion:
    """픽셀 원본이 케이크 위에 정확히 놓이는지, 잘못된 입력은 거절되는지."""

    def test_center_pixel_maps_to_cake_top_center(self, dp_node):
        """캔버스 정중앙 → 케이크 윗면 중심."""
        response = _convert(dp_node, [[(CANVAS_CX, CANVAS_CY),
                                       (CANVAS_CX + 10.0, CANVAS_CY)]])
        assert response.success, response.error_msg

        pen_down = [
            wp for wp in response.waypoints_3d if wp.pen_down
        ]
        assert pen_down
        assert pen_down[0].pose.position.x == pytest.approx(CAKE_TOP[0])
        assert pen_down[0].pose.position.y == pytest.approx(CAKE_TOP[1])

    def test_pen_down_z_stays_on_top_surface(self, dp_node):
        """그리는 동안의 Z는 윗면 높이에서 벗어나지 않는다."""
        response = _convert(dp_node, [STROKE_A, STROKE_B])
        assert response.success, response.error_msg

        for wp in response.waypoints_3d:
            if wp.pen_down:
                assert wp.pose.position.z == pytest.approx(CAKE_TOP[2])

    def test_invalid_surface_fails(self, dp_node):
        """알 수 없는 surface_type은 SIDE로 둔갑하지 않고 실패한다."""
        response = _convert(dp_node, [STROKE_A], surface="BOTTOM")
        assert not response.success
        assert "surface_type" in response.error_msg

    def test_empty_strokes_fails_with_reason(self, dp_node):
        """획이 하나도 없으면 실행 단계로 넘기지 않고 실패한다."""
        response = _convert(dp_node, [])
        assert not response.success
        assert "웨이포인트" in response.error_msg

    def test_single_point_stroke_fails(self, dp_node):
        """점 하나짜리는 이동 경로가 없어 웨이포인트가 안 나온다."""
        response = _convert(dp_node, [[(500.0, 350.0)]])
        assert not response.success
