"""원통·원뿔대 지오메트리와 픽셀→mm 변환 유틸리티.

순수 Python — ROS 의존성 없음.

역할:
  1. 캔버스 픽셀 좌표 → mm 좌표 변환  (pixel_to_mm_*)
  2. mm 좌표 → 로봇 절대 3D 좌표 변환  (CylinderCake 메서드)
  3. 펜 자세(쿼터니언) 계산 + 기울기 지원

좌표계 약속:
  - 프론트엔드 캔버스: 왼쪽 위 = (0, 0), 오른쪽 아래 = (canvas_w, canvas_h)
  - 케이크 기준점(srv): TCP로 기록한 **윗면 중심**
  - 내부 계산: **바닥면 중심** 기준 → DP에서 변환:
      바닥 중심 Z = 윗면 중심 Z − 케이크 높이

원통형 정의::

       ╭───────╮  ← radius
       │       │
       │       │  height
       │       │
       ╰───────╯  ← radius (동일)

  반지름은 높이에 관계없이 일정.

픽셀→mm 매핑:
  전체 캔버스는 1024×768이며 여백을 포함한다.
  TOP: 중심 (512,384), 반지름 320px인 원을 실제 반지름으로 환산한다.
  SIDE: 폭896px인 중앙 띠가 전체 둘레에 대응한다.
        띠 높이는 폭×실제 높이/실제 둘레이며 상하 여백을 제외한다.
  캔버스 해상도가 비례 변경되면 같은 비율로 편집 영역을 계산한다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# =====================================================================
# 픽셀 → mm 변환
# =====================================================================


def pixel_to_mm_top(
    px_x: float, px_y: float,
    canvas_w: float, canvas_h: float,
    radius: float,
) -> tuple[float, float]:
    """TOP 캔버스 픽셀 → mm 오프셋 (윗면 중심 기준).

    Returns: (mm_x, mm_y) — 윗면 중심에서의 오프셋
    """
    if canvas_w <= 0 or canvas_h <= 0:
        raise ValueError("캔버스 너비와 높이는 양수여야 합니다.")
    # 고객 화면의 원 지름은 전체 캔버스 높이의 640/768이다.
    # 전체 캔버스를 원으로 간주하면 같은 그림이 약 17% 작아진다.
    scale = (2.0 * radius) / (canvas_h * 640.0 / 768.0)
    mm_x = (px_x - canvas_w / 2.0) * scale
    mm_y = (canvas_h / 2.0 - px_y) * scale   # Y축 반전
    return (mm_x, mm_y)


def _pixel_to_mm_cylinder_side(
    px_x: float, px_y: float,
    canvas_w: float, canvas_h: float,
    radius: float, height: float,
) -> tuple[float, float]:
    """SIDE 캔버스 픽셀 → mm (호 길이, 높이).

    Returns: (arc_mm, height_mm)
      arc_mm: 원주 방향 호 길이 (mm), 0부터 시작
      height_mm: 바닥부터 높이 (mm), 0=바닥 ~ height=윗면
    """
    # 웹은 좌우 여백이 있는 둘레×높이 띠를 중앙에 배치한다.
    # 여백을 빼고 편집 영역만 물리 치수로 환산한다.
    drawing_width = canvas_w * 896.0 / 1024.0
    drawing_height = drawing_width * height / (2.0 * math.pi * radius)
    left = (canvas_w - drawing_width) / 2.0
    top = (canvas_h - drawing_height) / 2.0
    scale_x = (2.0 * math.pi * radius) / drawing_width
    scale_y = height / drawing_height
    arc_mm = (px_x - left) * scale_x
    height_mm = (top + drawing_height - px_y) * scale_y   # Y축 반전
    return (arc_mm, height_mm)


# =====================================================================
# 원통형 케이크 데이터 클래스
# =====================================================================


@dataclass(frozen=True)
class CylinderCake:
    """원통형 케이크 파라미터 (모두 mm 단위).

    Parameters
    ----------
    radius : 케이크 반지름 (mm)
    height : 케이크 높이 (mm)
    cx, cy, cz : 케이크 **바닥면** 중심의 로봇 절대 좌표 (mm)
                  (DP에서 윗면 중심 → 바닥 중심으로 변환한 값)
    """

    radius: float
    height: float
    cx: float
    cy: float
    cz: float

    # ------------------------------------------------------------------
    # TOP 면 좌표 변환
    # ------------------------------------------------------------------

    def top_to_abs(self, mm_x: float, mm_y: float) -> tuple[float, float, float]:
        """TOP 면: mm 오프셋 → 로봇 절대 좌표 mm.

        Z = 바닥 + 높이 (윗면).
        """
        return (self.cx + mm_x, self.cy + mm_y, self.cz + self.height)

    # ------------------------------------------------------------------
    # SIDE 면 좌표 변환
    # ------------------------------------------------------------------

    def side_to_abs(
        self, arc_mm: float, height_mm: float,
    ) -> tuple[float, float, float]:
        """SIDE 면: (호 길이, 높이) → 로봇 절대 좌표 mm.

        원통형: 반지름이 높이에 관계없이 일정.
        θ = arc_mm / radius
        """
        theta = arc_mm / self.radius if self.radius > 1e-6 else 0.0
        return (
            self.cx + self.radius * math.cos(theta),
            self.cy + self.radius * math.sin(theta),
            self.cz + height_mm,
        )

    def side_outward_normal(self, arc_mm: float) -> tuple[float, float, float]:
        """SIDE 면 외향 법선 단위 벡터.

        원통형: 수평 방향만 (경사 없음, 수직 성분 = 0).
        """
        theta = arc_mm / self.radius if self.radius > 1e-6 else 0.0
        return (math.cos(theta), math.sin(theta), 0.0)

    def side_abs_with_offset(
        self, arc_mm: float, height_mm: float, offset: float,
    ) -> tuple[float, float, float]:
        """SIDE 면: 표면에서 법선 방향으로 offset만큼 떨어진 절대 좌표.

        offset > 0: 표면 바깥 (접근), offset ≈ 0: 표면 위 (접촉)
        """
        px, py, pz = self.side_to_abs(arc_mm, height_mm)
        nx, ny, nz = self.side_outward_normal(arc_mm)
        return (px + nx * offset, py + ny * offset, pz + nz * offset)


# =====================================================================
# 쿼터니언 유틸리티
# =====================================================================


def direction_to_quaternion(
    dx: float, dy: float, dz: float,
) -> tuple[float, float, float, float]:
    """방향 벡터 → 쿼터니언 (tool Z축을 해당 방향으로 정렬).

    Returns: (qx, qy, qz, qw)
    """
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    dx, dy, dz = dx / length, dy / length, dz / length

    dot = dz  # (0,0,1) · (dx,dy,dz)

    if dot > 0.9999:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -0.9999:
        return (1.0, 0.0, 0.0, 0.0)

    cx_q, cy_q = -dy, dx
    w = 1.0 + dot
    norm = math.sqrt(cx_q * cx_q + cy_q * cy_q + w * w)
    return (cx_q / norm, cy_q / norm, 0.0, w / norm)


# =====================================================================
# 벡터 회전 유틸리티
# =====================================================================


def _tilt_vector(
    dx: float, dy: float, dz: float,
    tilt_axis_x: float, tilt_axis_y: float, tilt_axis_z: float,
    tilt_rad: float,
) -> tuple[float, float, float]:
    """방향 벡터를 지정 축 기준으로 tilt_rad만큼 회전 (Rodrigues)."""
    if abs(tilt_rad) < 1e-10:
        return (dx, dy, dz)

    al = math.sqrt(tilt_axis_x**2 + tilt_axis_y**2 + tilt_axis_z**2)
    if al < 1e-10:
        return (dx, dy, dz)
    kx, ky, kz = tilt_axis_x / al, tilt_axis_y / al, tilt_axis_z / al

    cos_t = math.cos(tilt_rad)
    sin_t = math.sin(tilt_rad)
    dot = kx * dx + ky * dy + kz * dz

    rx = dx * cos_t + (ky * dz - kz * dy) * sin_t + kx * dot * (1 - cos_t)
    ry = dy * cos_t + (kz * dx - kx * dz) * sin_t + ky * dot * (1 - cos_t)
    rz = dz * cos_t + (kx * dy - ky * dx) * sin_t + kz * dot * (1 - cos_t)
    return (rx, ry, rz)


# =====================================================================
# 펜 자세 (표면 타입별 + 기울기 각도)
# =====================================================================


def pen_quat_top(
    tilt_deg: float = 0.0,
    tilt_direction_x: float = 1.0,
    tilt_direction_y: float = 0.0,
) -> tuple[float, float, float, float]:
    """TOP 면 펜 자세.

    tilt_deg=0: 수직 아래 (0, 0, -1)
    tilt_deg=30: 수직에서 30도 기울어짐

    기울기 방향: (tilt_direction_x, tilt_direction_y)
    기본값 (1, 0): X축 양의 방향(로봇 앞쪽)으로 기울어짐.
    """
    pen_dir = (0.0, 0.0, -1.0)

    if abs(tilt_deg) > 0.01:
        tilt_rad = math.radians(tilt_deg)
        ax = -tilt_direction_y
        ay = tilt_direction_x
        pen_dir = _tilt_vector(
            pen_dir[0], pen_dir[1], pen_dir[2],
            ax, ay, 0.0, tilt_rad,
        )

    return direction_to_quaternion(pen_dir[0], pen_dir[1], pen_dir[2])


def _pen_quat_cylinder_side(
    arc_mm: float, cake: CylinderCake,
    tilt_deg: float = 0.0,
) -> tuple[float, float, float, float]:
    """SIDE 면 펜 자세: 내향 법선에서 tilt_deg만큼 아래로 기울어짐.

    tilt_deg=0: 표면 법선 방향 (수평으로 찌름)
    tilt_deg=30: 법선에서 30도 아래로 기울어짐

    원통형이므로 높이(y)에 따른 경사 변화 없음 — arc_mm만 필요.
    """
    nx, ny, nz = cake.side_outward_normal(arc_mm)
    inx, iny, inz = -nx, -ny, -nz  # 내향 법선

    if abs(tilt_deg) < 0.01:
        return direction_to_quaternion(inx, iny, inz)

    tilt_rad = math.radians(tilt_deg)
    # 기울기 축: 원주 접선 방향
    tilt_ax = -iny
    tilt_ay = inx

    pen_dir = _tilt_vector(inx, iny, inz, tilt_ax, tilt_ay, 0.0, tilt_rad)
    return direction_to_quaternion(pen_dir[0], pen_dir[1], pen_dir[2])

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



def _pixel_to_mm_frustum_side(
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
        region_x - 1e-4 <= px_x <= region_x + region_width + 1e-4
        and region_y - 1e-4 <= px_y <= region_y + region_height + 1e-4
    ):
        raise ValueError("옆면 픽셀이 캔버스의 옆면 편집 영역 밖입니다.")

    # Point32 직렬화의 경계 반올림만 허용하고 비율을 0~1로 복원한다.
    u = max(0.0, min(1.0, (px_x - region_x) / region_width))
    v = max(0.0, min(1.0, (px_y - region_y) / region_height))
    height_mm = (1.0 - v) * cake.height
    theta = 2.0 * math.pi * u
    arc_mm = theta * cake.radius_at(height_mm)
    return arc_mm, height_mm



def pixel_to_mm_side(px_x, px_y, canvas_w, canvas_h, radius, height=None):
    """기존 원통 호출과 원뿔대 호출을 구분해 원본 픽셀을 mm로 변환한다.

    원통은 반지름·높이를 별도 인자로, 원뿔대는 FrustumCake 객체를 받는다.
    원뿔대 반환 호 길이는 해당 높이의 실제 반지름 기준이며 경로 보간용이 아니다.
    """
    if isinstance(radius, FrustumCake):
        return _pixel_to_mm_frustum_side(px_x, px_y, canvas_w, canvas_h, radius)
    return _pixel_to_mm_cylinder_side(px_x, px_y, canvas_w, canvas_h, radius, height)


def pen_quat_side(arc_mm, height_or_cake, cake=None, tilt_deg=0.0):
    """두 형상 API의 펜 자세를 지원한다. 경로 어댑터는 기존 기울기 계약을 유지한다.

    (arc, height, FrustumCake)는 팀원 API의 내향 법선 자세다.
    (arc, cake, tilt_deg=...)는 PR #16의 수직 기준 기울기를 사용하는 경로 API다.
    """
    if cake is None:
        return _pen_quat_cylinder_side(arc_mm, height_or_cake, tilt_deg=tilt_deg)
    nx, ny, nz = cake.side_outward_normal(arc_mm, height_or_cake)
    return direction_to_quaternion(-nx, -ny, -nz)
