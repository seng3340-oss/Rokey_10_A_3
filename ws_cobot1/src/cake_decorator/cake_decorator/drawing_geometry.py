"""원뿔대(frustum) 케이크 지오메트리 유틸리티.

순수 Python — ROS 의존성 없음.
SIDE 면의 2D→3D 좌표 변환과 표면 법선·펜 자세 계산을 담당한다.

원뿔대 정의::

       ╭───────╮  ← r_top (윗면 반지름)
      ╱         ╲
     ╱           ╲  height
    ╱             ╲
   ╰───────────────╯  ← r_bottom (아랫면 반지름)

  높이 y에서의 반지름: r(y) = r_bottom + (r_top - r_bottom) × y / height

SIDE 면 좌표계:
  x = 원주 방향 호 길이 (mm), 0에서 시작
  y = 바닥부터 높이 (mm)
  → θ = x / r(y), 3D = (r(y)·cosθ, r(y)·sinθ, y)
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# =====================================================================
# 원뿔대 케이크 데이터 클래스
# =====================================================================


@dataclass(frozen=True)
class FrustumCake:
    """원뿔대 케이크 파라미터 (모두 mm 단위).

    Parameters
    ----------
    r_top : 윗면 반지름 (mm)
    r_bottom : 아랫면 반지름 (mm)
    height : 케이크 높이 (mm)
    cx, cy, cz : 케이크 바닥면 중심의 로봇 절대 좌표 (mm)
    """

    r_top: float
    r_bottom: float
    height: float
    cx: float
    cy: float
    cz: float
    yaw: float = 0.0

    # ------------------------------------------------------------------
    # 기본 지오메트리
    # ------------------------------------------------------------------

    @property
    def slant_angle(self) -> float:
        """측면 경사각 (rad). r_bottom > r_top이면 양수."""
        if self.height <= 0:
            return 0.0
        return math.atan2(self.r_bottom - self.r_top, self.height)

    @property
    def slant_height(self) -> float:
        """바닥에서 윗면까지 원뿔대 측면을 따라 잰 길이(mm)."""
        return math.hypot(self.height, self.r_bottom - self.r_top)

    def radius_at(self, y: float) -> float:
        """높이 y (바닥 기준 mm)에서의 반지름."""
        if self.height <= 0:
            return self.r_bottom
        t = max(0.0, min(1.0, y / self.height))
        return self.r_bottom + (self.r_top - self.r_bottom) * t

    # ------------------------------------------------------------------
    # TOP 면 좌표 변환
    # ------------------------------------------------------------------

    def top_to_abs(self, x: float, y: float) -> tuple[float, float, float]:
        """TOP 면: (x, y) 오프셋 mm → 로봇 절대 좌표 mm.

        윗면 중심 기준 오프셋을 로봇 절대 좌표로 변환한다.
        Z = 바닥 + 높이 (윗면).
        """
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return (
            self.cx + x * cos_yaw - y * sin_yaw,
            self.cy + x * sin_yaw + y * cos_yaw,
            self.cz + self.height,
        )

    # ------------------------------------------------------------------
    # SIDE 면 좌표 변환
    # ------------------------------------------------------------------

    def side_to_abs(
        self, x_arc: float, y_height: float,
    ) -> tuple[float, float, float]:
        """SIDE 면: (x_arc, y_height) → 로봇 절대 좌표 mm.

        x_arc: 원주 방향 호 길이 (mm)
        y_height: 바닥부터 높이 (mm)
        """
        r = self.radius_at(y_height)
        theta = x_arc / r if r > 1e-6 else 0.0
        local_x = r * math.cos(theta)
        local_y = r * math.sin(theta)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return (
            self.cx + local_x * cos_yaw - local_y * sin_yaw,
            self.cy + local_x * sin_yaw + local_y * cos_yaw,
            self.cz + y_height,
        )

    def side_outward_normal(
        self, x_arc: float, y_height: float,
    ) -> tuple[float, float, float]:
        """SIDE 면 외향 법선 단위 벡터 (원뿔대 경사 반영).

        수평 방향(반지름 바깥)과 수직 방향을 경사각에 따라 합성한다.
        r_bottom > r_top이면 법선이 약간 위를 향한다.
        """
        r = self.radius_at(y_height)
        theta = x_arc / r if r > 1e-6 else 0.0
        alpha = self.slant_angle
        ca, sa = math.cos(alpha), math.sin(alpha)
        local_x = ca * math.cos(theta)
        local_y = ca * math.sin(theta)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return (
            local_x * cos_yaw - local_y * sin_yaw,
            local_x * sin_yaw + local_y * cos_yaw,
            sa,
        )

    def side_abs_with_offset(
        self, x_arc: float, y_height: float, offset: float,
    ) -> tuple[float, float, float]:
        """SIDE 면: 표면에서 법선 방향으로 offset만큼 떨어진 절대 좌표.

        offset > 0: 표면 바깥 (접근), offset ≈ 0: 표면 위 (접촉)
        """
        px, py, pz = self.side_to_abs(x_arc, y_height)
        nx, ny, nz = self.side_outward_normal(x_arc, y_height)
        return (px + nx * offset, py + ny * offset, pz + nz * offset)


def pixel_to_mm_top(
    px_x: float,
    px_y: float,
    canvas_w: float,
    canvas_h: float,
    radius: float,
) -> tuple[float, float]:
    """윗면 캔버스 픽셀을 윗면 중심 기준 mm 오프셋으로 바꾼다."""
    if canvas_w <= 0 or canvas_h <= 0:
        raise ValueError("캔버스 너비와 높이는 양수여야 합니다.")

    radius_px = canvas_h * (320.0 / 768.0)
    scale = radius / radius_px
    return (
        (px_x - canvas_w / 2.0) * scale,
        (canvas_h / 2.0 - px_y) * scale,
    )


def pixel_to_mm_side(
    px_x: float,
    px_y: float,
    canvas_w: float,
    canvas_h: float,
    cake: FrustumCake,
) -> tuple[float, float]:
    """옆면 픽셀을 원뿔대 표면의 호 길이와 바닥 기준 높이로 바꾼다.

    가로 픽셀은 전체 원주에 대응한다. 세로 위치에서의 반지름을 사용해
    호 길이를 계산하므로 같은 픽셀 각도는 원뿔대 높이에 관계없이 유지된다.
    """
    if canvas_w <= 0 or canvas_h <= 0:
        raise ValueError("캔버스 너비와 높이는 양수여야 합니다.")

    region_width = canvas_w * (896.0 / 1024.0)
    region_x = (canvas_w - region_width) / 2.0
    region_height = (
        region_width
        * cake.slant_height
        / (math.pi * (cake.r_top + cake.r_bottom))
    )
    region_y = (canvas_h - region_height) / 2.0
    if not (
        region_x <= px_x <= region_x + region_width
        and region_y <= px_y <= region_y + region_height
    ):
        raise ValueError("옆면 픽셀이 캔버스의 옆면 편집 영역 밖입니다.")

    u = (px_x - region_x) / region_width
    v = (px_y - region_y) / region_height
    height_mm = (1.0 - v) * cake.height
    theta = 2.0 * math.pi * u
    arc_mm = theta * cake.radius_at(height_mm)
    return arc_mm, height_mm


# =====================================================================
# 쿼터니언 유틸리티
# =====================================================================


def direction_to_quaternion(
    dx: float, dy: float, dz: float,
) -> tuple[float, float, float, float]:
    """방향 벡터 → 쿼터니언 (tool Z축을 해당 방향으로 정렬).

    로봇 도구 프레임의 Z축이 (dx, dy, dz) 방향을 가리키도록 하는
    회전 쿼터니언을 반환한다.

    Returns: (qx, qy, qz, qw)
    """
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    dx, dy, dz = dx / length, dy / length, dz / length

    # (0,0,1) → (dx,dy,dz) 회전 쿼터니언
    dot = dz  # (0,0,1) · (dx,dy,dz)

    if dot > 0.9999:
        return (0.0, 0.0, 0.0, 1.0)  # 같은 방향: 항등

    if dot < -0.9999:
        return (1.0, 0.0, 0.0, 0.0)  # 반대 방향: X축 180° 회전

    # cross((0,0,1), (dx,dy,dz)) = (-dy, dx, 0)
    cx, cy = -dy, dx
    w = 1.0 + dot
    norm = math.sqrt(cx * cx + cy * cy + w * w)
    return (cx / norm, cy / norm, 0.0, w / norm)


# =====================================================================
# 펜 자세 (표면 타입별)
# =====================================================================


def pen_quat_top() -> tuple[float, float, float, float]:
    """TOP 면 펜 자세: 아래를 향함 (Z축 → (0,0,-1))."""
    return direction_to_quaternion(0.0, 0.0, -1.0)


def pen_quat_side(
    x_arc: float, y_height: float, cake: FrustumCake,
) -> tuple[float, float, float, float]:
    """SIDE 면 펜 자세: 내향 법선 방향 (원뿔대 경사 반영).

    각 점마다 자세가 달라진다 — 원주 위치(θ)와 경사각에 따라.
    """
    nx, ny, nz = cake.side_outward_normal(x_arc, y_height)
    # 내향 법선 = 바깥 법선의 반대 방향
    return direction_to_quaternion(-nx, -ny, -nz)
