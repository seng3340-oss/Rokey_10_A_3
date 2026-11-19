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


def overlap_side_seam(
    strokes, seam_arc_mm, overlap_mm, height_tolerance_mm=3.0, seam_epsilon_mm=0.05,
):
    """옆면 띠의 좌우 끝이 이어지는 획에 이음매를 넘는 짧은 겹침 구간을 붙인다.

    호출 시점: 획마다 mm 변환·단순화를 마친 뒤, 로봇 명령으로 바꾸기 전.
    입력 strokes: 획별 (경로 기준 호 길이 mm, 바닥 기준 높이 mm) 점 목록.
    seam_arc_mm: 한 바퀴에 해당하는 경로 기준 호 길이(2π × 평균 반지름, mm).
    반환값: 같은 순서의 새 획 목록. 입력 목록은 수정하지 않는다.

    웹에서 띠 오른쪽 끝(호 = 한 바퀴)에 닿은 획 끝점이 있고, 어느 획이든 왼쪽 끝(호 = 0)에서
    비슷한 높이로 닿아 있으면 링이 닫힌 것으로 본다. 이때 오른쪽 끝점 바깥에
    (한 바퀴 + overlap_mm, 왼쪽 끝 높이) 점을 추가해 0도를 지나 조금 더 그린다.
    펜이 닿고 떨어지는 지점은 잉크가 약하므로 정확히 맞닿게만 그리면 이음매가 비어 보인다.
    왼쪽 끝 높이로 맞추므로 두 끝의 높이가 조금 달라도 겹침 구간에서 선이 이어진다.

    한쪽 끝만 이음매에 닿은 획은 반대편에 의도하지 않은 표시가 생기지 않도록 그대로 둔다.
    """
    if not all(
        math.isfinite(value)
        for value in (seam_arc_mm, overlap_mm, height_tolerance_mm, seam_epsilon_mm)
    ) or seam_arc_mm <= 0 or not 0 <= overlap_mm <= 20:
        raise ValueError("옆면 이음매 둘레는 양수, 겹침 길이는 0~20mm여야 합니다.")
    result = [list(stroke) for stroke in strokes]
    if overlap_mm == 0:
        return result

    def at_left(point):
        return abs(point[0]) <= seam_epsilon_mm

    def at_right(point):
        return abs(point[0] - seam_arc_mm) <= seam_epsilon_mm

    # 왼쪽 이음매(호 0)에 닿은 모든 획 끝점의 높이. 같은 획의 반대쪽 끝도 포함한다.
    left_heights = [
        point[1]
        for stroke in result
        if len(stroke) >= 2
        for point in (stroke[0], stroke[-1])
        if at_left(point)
    ]
    for stroke in result:
        if len(stroke) < 2:
            continue
        # 시작점과 끝점 중 오른쪽 이음매에 닿은 쪽을 찾는다. 획 방향은 사용자가 그린 순서를 따른다.
        for is_end in (True, False):
            point = stroke[-1] if is_end else stroke[0]
            if not at_right(point):
                continue
            partner = min(
                left_heights, key=lambda height: abs(height - point[1]), default=None,
            )
            if partner is None or abs(partner - point[1]) > height_tolerance_mm:
                continue
            extra = (seam_arc_mm + overlap_mm, partner)
            if is_end:
                stroke.append(extra)
            else:
                stroke.insert(0, extra)
            # 한 획에서 한 번만 연장한다. 시작·끝이 모두 오른쪽인 경우 겹침이 두 번 생기지 않게 한다.
            break
    return result
