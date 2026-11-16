"""2D 픽셀 좌표 → 3D 케이크 표면 좌표 변환.

ROS 의존성 없이 순수 Python + math만 사용한다.
pytest로 단독 테스트가 가능하다.

좌표 규약
---------
- 캔버스: 왼쪽 위 (0,0), 오른쪽 아래 (canvas_width, canvas_height), 단위 px
- 로봇: BASE 기준, 단위 m, 자세는 쿼터니언
- 케이크: center(x,y,z)는 케이크 윗면 중심, radius/height는 m
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# 데이터 클래스 (ROS 메시지 대신 테스트에서 사용)
# ---------------------------------------------------------------------------


@dataclass
class Point2D:
    """2D 픽셀 좌표."""

    x: float
    y: float


@dataclass

class Stroke2D:
    """2D 획 데이터."""

    stroke_id: int
    points: list[Point2D]
    is_closed: bool = False


@dataclass
class Pose3D:
    """위치(m) + 쿼터니언."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


@dataclass
class Waypoint3D:
    """3D 웨이포인트."""

    pose: Pose3D
    speed: float = 20.0     # mm/s
    pen_down: bool = False
    contact_force: float = 0.0


@dataclass

class CakePose:
    """케이크 윗면 중심 위치(m)와 방향(rad)."""

    x: float
    y: float
    z: float        # 윗면 높이
    orientation: float = 0.0


@dataclass
class TrajectoryResult:
    """경로 변환 결과."""

    success: bool
    waypoints: list[Waypoint3D] = field(default_factory=list)
    pen_states: list[bool] = field(default_factory=list)
    estimated_duration_sec: float = 0.0
    error_msg: str = ''


# ---------------------------------------------------------------------------
# 변환 설정
# ---------------------------------------------------------------------------


@dataclass
class ConvertConfig:
    """경로 변환에 필요한 파라미터."""

    drawing_speed: float = 20.0         # mm/s, 펜 내림 상태 이동 속도
    travel_speed: float = 50.0          # mm/s, 펜 올림 상태 이동 속도
    approach_height: float = 0.01       # m, 표면 위 접근 높이

    drawing_z_offset: float = 0.0       # m, 표면 접촉 시 Z 오프셋
    side_margin_ratio: float = 0.05     # 옆면 상하 여백 비율


# ---------------------------------------------------------------------------
# 핵심 변환 함수
# ---------------------------------------------------------------------------


def pixel_to_top(
    px: float,
    py: float,
    canvas_w: int,
    canvas_h: int,
    cake: CakePose,
    cake_radius: float,
) -> tuple[float, float, float]:
    """윗면 픽셀 → 로봇 BASE 좌표 (m).

    캔버스 중심 = 케이크 중심, 캔버스 전체가 케이크 지름에 매핑된다.
    캔버스는 정사각형으로 가정하고, 짧은 변 기준으로 스케일한다.
    """
    # 캔버스 중심 기준 정규화 (-1 ~ +1)
    half_w = canvas_w / 2.0
    half_h = canvas_h / 2.0
    # 짧은 변 기준 스케일 → 긴 변 방향은 케이크 밖으로 나갈 수 있음
    half_min = min(half_w, half_h)

    nx = (px - half_w) / half_min   # 오른쪽 +
    ny = (py - half_h) / half_min   # 아래 + (캔버스) → 로봇 -Y로 매핑

    # 케이크 표면 좌표 (m)

    x = cake.x + nx * cake_radius
    y = cake.y - ny * cake_radius   # 캔버스 Y↓ = 로봇 Y↑ 반전
    z = cake.z

    return x, y, z


def pixel_to_side(
    px: float,
    py: float,
    canvas_w: int,
    canvas_h: int,
    cake: CakePose,
    cake_radius: float,
    cake_height: float,
    margin_ratio: float = 0.05,
) -> tuple[float, float, float]:
    """옆면 픽셀 → 로봇 BASE 좌표 (m).

    캔버스 X축 → 케이크 둘레 (0 ~ 2π)
    캔버스 Y축 → 케이크 높이 (위 → 아래)
    옆면은 원통 좌표로 변환한 뒤 직교 좌표로 바꾼다.
    """
    # X → 각도 (0 ~ 2π), cake.orientation을 시작점으로
    theta = (px / canvas_w) * 2 * math.pi + cake.orientation

    # Y → 높이 (윗면에서 아래로). 상하 여백 적용
    usable_h = cake_height * (1.0 - 2 * margin_ratio)
    top_margin = cake_height * margin_ratio
    h_from_top = top_margin + (py / canvas_h) * usable_h
    z = cake.z - h_from_top


    # 원통 → 직교
    x = cake.x + cake_radius * math.cos(theta)
    y = cake.y + cake_radius * math.sin(theta)

    return x, y, z


def _tool_orientation_top() -> tuple[float, float, float, float]:
    """윗면 드로잉용 도구 자세: Z축 아래 방향 (펜이 수직으로 내려감).

    쿼터니언: 180° around X → (1, 0, 0, 0)
    """
    return (1.0, 0.0, 0.0, 0.0)


def _tool_orientation_side(theta: float) -> tuple[float, float, float, float]:
    """옆면 드로잉용 도구 자세: 케이크 중심을 향해 수평으로.

    펜 축이 케이크 반경 방향(안쪽)을 가리키도록 한다.
    """
    # 도구가 -X 방향을 기본으로 가리킨다고 가정
    # theta만큼 Z축 회전 + 수평 자세
    # Z축 회전 쿼터니언: (0, 0, sin(θ/2), cos(θ/2))
    # 그 뒤 Y축 90° 회전으로 수평 → 케이크 중심 향함
    # 합성: Rz(theta) * Ry(pi/2)
    half_t = theta / 2.0
    half_y = math.pi / 4.0  # pi/2 의 절반

    # Rz
    cz, sz = math.cos(half_t), math.sin(half_t)
    # Ry(pi/2)
    cy, sy = math.cos(half_y), math.sin(half_y)


    # 쿼터니언 곱: Rz * Ry
    qw = cz * cy
    qx = -sz * sy
    qy = cz * sy
    qz = sz * cy

    return (qx, qy, qz, qw)


# ---------------------------------------------------------------------------
# 전체 변환 파이프라인
# ---------------------------------------------------------------------------


def generate_trajectory(
    canvas_width: int,
    canvas_height: int,
    strokes: list[Stroke2D],
    surface_type: str,
    cake: CakePose,
    cake_radius: float,
    cake_height: float,
    config: ConvertConfig | None = None,
) -> TrajectoryResult:
    """Stroke2D 목록을 Waypoint3D 경로로 변환한다.

    Parameters
    ----------
    canvas_width, canvas_height : 캔버스 해상도 (px)
    strokes : 2D 획 목록
    surface_type : 'TOP' 또는 'SIDE'

    cake : 케이크 위치/방향
    cake_radius, cake_height : 케이크 치수 (m)
    config : 변환 설정 (None이면 기본값)

    Returns
    -------
    TrajectoryResult
    """
    if config is None:
        config = ConvertConfig()

    surface = surface_type.upper()
    if surface not in ('TOP', 'SIDE'):
        return TrajectoryResult(
            success=False,
            error_msg=f'지원하지 않는 surface_type: {surface_type}',
        )

    if not strokes:
        return TrajectoryResult(
            success=False,
            error_msg='빈 스트로크 목록입니다.',
        )

    if canvas_width <= 0 or canvas_height <= 0:
        return TrajectoryResult(
            success=False,
            error_msg='캔버스 크기가 유효하지 않습니다.',
        )

    waypoints: list[Waypoint3D] = []
    pen_states: list[bool] = []

    total_distance = 0.0

    for stroke in strokes:
        if len(stroke.points) < 2:
            continue

        # --- 획의 첫 점으로 이동 (펜 올림) ---
        first_pt = stroke.points[0]
        x, y, z = _convert_point(
            first_pt, canvas_width, canvas_height,
            surface, cake, cake_radius, cake_height, config,
        )
        qx, qy, qz, qw = _get_orientation(surface, first_pt, canvas_width, cake)

        # 접근 높이로 이동
        approach_z = z + config.approach_height if surface == 'TOP' else z
        waypoints.append(Waypoint3D(
            pose=Pose3D(x, y, approach_z, qx, qy, qz, qw),
            speed=config.travel_speed,
            pen_down=False,
        ))
        pen_states.append(False)

        # 표면으로 하강 (펜 내림 시작)
        draw_z = z + config.drawing_z_offset
        waypoints.append(Waypoint3D(
            pose=Pose3D(x, y, draw_z, qx, qy, qz, qw),
            speed=config.drawing_speed,
            pen_down=True,
        ))
        pen_states.append(True)


        prev_x, prev_y, prev_z = x, y, draw_z

        # --- 획을 따라 그리기 (펜 내림) ---
        for pt in stroke.points[1:]:
            x, y, z = _convert_point(
                pt, canvas_width, canvas_height,
                surface, cake, cake_radius, cake_height, config,
            )
            qx, qy, qz, qw = _get_orientation(surface, pt, canvas_width, cake)
            draw_z = z + config.drawing_z_offset

            waypoints.append(Waypoint3D(
                pose=Pose3D(x, y, draw_z, qx, qy, qz, qw),
                speed=config.drawing_speed,
                pen_down=True,
            ))
            pen_states.append(True)

            total_distance += math.sqrt(
                (x - prev_x) ** 2 + (y - prev_y) ** 2 + (draw_z - prev_z) ** 2
            )
            prev_x, prev_y, prev_z = x, y, draw_z

        # --- 획 끝: 펜 올림 ---
        waypoints.append(Waypoint3D(
            pose=Pose3D(
                prev_x, prev_y, prev_z + config.approach_height,
                qx, qy, qz, qw,
            ),
            speed=config.travel_speed,
            pen_down=False,
        ))

        pen_states.append(False)

    if not waypoints:
        return TrajectoryResult(
            success=False,
            error_msg='유효한 웨이포인트를 생성하지 못했습니다.',
        )

    # 대략적 소요 시간 추정 (거리 / 평균 속도)
    avg_speed_m = config.drawing_speed / 1000.0  # mm/s → m/s
    estimated_sec = total_distance / avg_speed_m if avg_speed_m > 0 else 0.0

    return TrajectoryResult(
        success=True,
        waypoints=waypoints,
        pen_states=pen_states,
        estimated_duration_sec=estimated_sec,
    )


# ---------------------------------------------------------------------------
# 내부 헬퍼
# ---------------------------------------------------------------------------


def _convert_point(
    pt: Point2D,
    canvas_w: int,
    canvas_h: int,
    surface: str,
    cake: CakePose,
    cake_radius: float,

    cake_height: float,
    config: ConvertConfig,
) -> tuple[float, float, float]:
    """픽셀 좌표를 3D 좌표로 변환한다."""
    if surface == 'TOP':
        return pixel_to_top(pt.x, pt.y, canvas_w, canvas_h, cake, cake_radius)
    else:
        return pixel_to_side(
            pt.x, pt.y, canvas_w, canvas_h,
            cake, cake_radius, cake_height, config.side_margin_ratio,
        )


def _get_orientation(
    surface: str,
    pt: Point2D,
    canvas_w: int,
    cake: CakePose,
) -> tuple[float, float, float, float]:
    """표면 타입에 따른 도구 자세 쿼터니언을 반환한다."""
    if surface == 'TOP':
        return _tool_orientation_top()
    else:
        theta = (pt.x / canvas_w) * 2 * math.pi + cake.orientation
        return _tool_orientation_side(theta)