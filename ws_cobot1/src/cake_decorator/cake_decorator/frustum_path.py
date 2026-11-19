"""원뿔대 실제 형상과 PR #16의 원호·스플라인 경로 좌표를 연결한다.

높이마다 기준 반지름이 다른 호 길이를 그대로 보간하면 수직선도 각도가 변한다.
경로에서는 평균 반지름 기준 호 길이를 쓰고 3D 변환에서 실제 반지름을 적용한다.
위치는 mm, yaw는 rad다.
"""

import math
from dataclasses import dataclass

from .drawing_geometry import FrustumCake, pixel_to_mm_side


@dataclass(frozen=True)
class DrawingFrustum(FrustumCake):
    """높이와 무관한 호 좌표를 소비하는 실행용 형상. 원본 형상 API와 분리한다."""

    # 실제 형상과 픽셀 매핑은 유지하고 옆면 목표의 반지름만 보정한다(mm).
    radial_offset_mm: float = 0.0
    # BASE Y 양쪽을 각각 안으로 옮기는 거리(mm).
    # X/Z와 중심은 유지하고 sin(theta)에 따라 부드럽게 보정한다.
    side_y_inset_mm: float = 0.0

    @property
    def radius(self):
        """각도 환산 기준인 평균 반지름(mm). 접촉 반지름은 radius_at으로 구한다."""
        return (self.r_top + self.r_bottom) / 2.0

    def side_to_abs(self, arc_mm, height_mm):
        """고정 기준 호 좌표를 해당 높이의 실제 표면 좌표(mm)로 바꾼다."""
        local_arc = arc_mm / self.radius * self.radius_at(height_mm)
        point = FrustumCake.side_to_abs(self, local_arc, height_mm)
        theta = arc_mm / self.radius + self.yaw
        return (
            point[0] + self.radial_offset_mm * math.cos(theta),
            point[1] + (self.radial_offset_mm - self.side_y_inset_mm) * math.sin(theta),
            point[2],
        )

    def side_outward_normal(self, arc_mm, height_mm=None):
        """경사각·yaw를 포함한 단위 법선. 같은 각도의 법선은 높이와 무관하다."""
        theta = arc_mm / self.radius + self.yaw
        return (
            math.cos(self.slant_angle) * math.cos(theta),
            math.cos(self.slant_angle) * math.sin(theta),
            math.sin(self.slant_angle),
        )

    def side_abs_with_offset(self, arc_mm, height_mm, offset):
        """표면 바깥 법선으로 접근·이탈한다. offset과 반환 위치 단위는 mm다."""
        point = self.side_to_abs(arc_mm, height_mm)
        normal = self.side_outward_normal(arc_mm)
        return tuple(p + n * offset for p, n in zip(point, normal))


def pixel_to_path_mm(px_x, px_y, canvas_w, canvas_h, cake):
    """팀원 편집 영역 검증 후 호 길이만 경로의 고정 기준으로 바꾼다."""
    local_arc, height = pixel_to_mm_side(px_x, px_y, canvas_w, canvas_h, cake)
    return local_arc / cake.radius_at(height) * cake.radius, height
