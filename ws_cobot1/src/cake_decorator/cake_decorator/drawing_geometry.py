"""원통형(cylinder) 케이크 지오메트리 + 픽셀→mm 변환 유틸리티.

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
  TOP 캔버스:
    캔버스 = 윗면 원의 바운딩 박스 (2·radius × 2·radius)
    scale = 2·radius / min(canvas_w, canvas_h)
    mm_x = (px_x − canvas_w/2) × scale
    mm_y = (canvas_h/2 − px_y) × scale   ← Y축 반전

  SIDE 캔버스:
    캔버스 = 둘레 × 높이 직사각형 펼침
    scale_x = 2π·radius / canvas_w
    scale_y = height / canvas_h
    arc_mm = px_x × scale_x
    height_mm = (canvas_h − px_y) × scale_y   ← Y축 반전
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
    *, drawing_diameter_px: float | None = None,
) -> tuple[float, float]:
    """TOP 캔버스 픽셀 → mm 오프셋 (윗면 중심 기준).

    drawing_diameter_px가 있으면 가운데 원의 지름을 기준으로 환산한다.
    반환하는 두 값은 윗면 중심에서의 mm 오프셋이다.
    """
    # 기본은 전체 캔버스를 쓰며, 웹 서비스는 점선 원의 픽셀 지름을 전달한다.
    diameter_px = min(canvas_w, canvas_h) if drawing_diameter_px is None else drawing_diameter_px
    scale = (2.0 * radius) / diameter_px
    mm_x = (px_x - canvas_w / 2.0) * scale
    mm_y = (canvas_h / 2.0 - px_y) * scale   # Y축 반전
    return (mm_x, mm_y)


def side_drawing_rect(canvas_w, canvas_h, radius, height):
    """웹과 동일한 측면 입력 띠의 (왼쪽, 위, 폭, 높이)를 원본 px로 반환한다.

    1024px 중 896px를 둘레 전체에 대응시키고, 띠 높이는 실제 높이/둘레로 정한다.
    Canvas 전체를 표면으로 쓰면 웹 미리보기와 각도·높이 축척이 달라진다.
    """
    if min(canvas_w, canvas_h, radius, height) <= 0:
        raise ValueError("캔버스와 케이크 치수는 양수여야 합니다.")
    width = canvas_w * (896.0 / 1024.0)
    strip_height = width * height / (2.0 * math.pi * radius)
    return ((canvas_w - width) / 2.0, (canvas_h - strip_height) / 2.0, width, strip_height)


def pixel_to_mm_side(
    px_x: float, px_y: float,
    canvas_w: float, canvas_h: float,
    radius: float, height: float,
    *, drawing_rect=None,
) -> tuple[float, float]:
    """SIDE 픽셀을 (원주 호 길이, 바닥부터 높이) mm로 변환한다.

    일반 좌표 유틸의 기본은 전체 캔버스다. 웹 요청을 처리하는 DP는 가운데
    입력 띠를 drawing_rect로 명시하며 표면 밖의 픽셀을 거절한다.
    """
    left, top, width, strip_height = drawing_rect or (0.0, 0.0, canvas_w, canvas_h)
    if not (left - 1e-4 <= px_x <= left + width + 1e-4 and
            top - 1e-4 <= px_y <= top + strip_height + 1e-4):
        raise ValueError("획에 케이크 측면 입력 띠를 벗어난 점이 있습니다.")
    arc_mm = (px_x - left) * (2.0 * math.pi * radius) / width
    height_mm = (top + strip_height - px_y) * height / strip_height
    return arc_mm, height_mm


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


def pen_quat_side(
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