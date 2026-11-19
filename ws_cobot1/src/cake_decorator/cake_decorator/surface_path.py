"""원통 옆면의 개발면 선분을 표면 위 원호·수직 이동으로 구성한다.

수평 선분은 정확한 원호다. 대각선은 높이 반간격 → 원호 → 높이 반간격으로
나눠 개발면에서의 추가 편차를 max_error_mm 이하로 제한한다.
직선 현으로 원통 안을 통과하거나 일반 3D 원호를 나선으로 가정하지 않는다.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SurfaceMove:
    """순수 mm 좌표 명령. via가 있으면 start→via→end의 원호다."""

    end: tuple[float, float, float]
    arc_mm: float
    via: tuple[float, float, float] | None = None
    via_arc_mm: float | None = None
    start_angle: float = 0.0
    end_angle: float = 0.0


def side_surface_leg(cake, start, end, offset_mm=0.0, max_error_mm=0.1):
    """(호 길이 mm, 바닥 기준 높이 mm) 사이 이동을 원통 밖/표면에 구성한다."""
    values = [
        cake.radius,
        cake.height,
        cake.cx,
        cake.cy,
        cake.cz,
        *start,
        *end,
        offset_mm,
        max_error_mm,
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("옆면 이동 입력은 유한 값이어야 합니다.")
    if cake.radius <= 0 or cake.height <= 0 or offset_mm < 0 or max_error_mm <= 0:
        raise ValueError(
            "양의 반지름·높이·허용 오차와 0 이상의 표면 오프셋이 필요합니다."
        )
    if not (0 <= start[1] <= cake.height and 0 <= end[1] <= cake.height):
        raise ValueError("옆면 높이가 케이크 밖입니다.")
    arc0, height0 = start
    arc1, height1 = end
    delta = arc1 - arc0
    if abs(delta) < 1e-9:
        if abs(height1 - height0) < 1e-9:
            return []
        return [SurfaceMove(cake.side_abs_with_offset(arc1, height1, offset_mm), arc1)]
    # 180도나 완전한 원은 세 점이 퇴화할 수 있어 한 명령을 최대 90도로 제한한다.
    count = max(1, math.ceil(abs(delta / cake.radius) / (math.pi / 2)))
    if abs(height1 - height0) > 1e-9:
        count = max(count, math.ceil(abs(height1 - height0) / (2 * max_error_mm)))
    if count * 3 > 4096:
        raise ValueError("옆면 분할 명령 한도 초과: 경로 또는 허용 오차를 확인하세요.")
    moves = []
    previous = (arc0, height0)
    for i in range(1, count + 1):
        arc = arc0 + delta * i / count
        height = height0 + (height1 - height0) * i / count
        mid_height = (previous[1] + height) / 2
        if abs(mid_height - previous[1]) > 1e-9:
            moves.append(
                SurfaceMove(
                    cake.side_abs_with_offset(previous[0], mid_height, offset_mm),
                    previous[0],
                )
            )
        via_arc = (previous[0] + arc) / 2
        moves.append(
            SurfaceMove(
                end=cake.side_abs_with_offset(arc, mid_height, offset_mm),
                arc_mm=arc,
                via=cake.side_abs_with_offset(via_arc, mid_height, offset_mm),
                via_arc_mm=via_arc,
                start_angle=previous[0] / cake.radius,
                end_angle=arc / cake.radius,
            )
        )
        if abs(height - mid_height) > 1e-9:
            moves.append(
                SurfaceMove(cake.side_abs_with_offset(arc, height, offset_mm), arc)
            )
        previous = (arc, height)
    return moves


@dataclass(frozen=True)
class SplineSurfaceMove:
    """연속 높이 변화의 표면 샘플. 실제 보간 오차는 가상 관측 궤적으로 별도 검증한다."""

    samples: tuple
    sample_arcs: tuple

    @property
    def end(self):
        return self.samples[-1]

    @property
    def arc_mm(self):
        return self.sample_arcs[-1]

    via = None


def split_spline_samples(samples):
    """순서대로 최대 100개씩 고르게 나눠 마지막 묶음이 한 점이 되지 않게 한다."""
    if not samples:
        return []
    if len(samples) < 2:
        raise ValueError("스플라인 묶음에는 두 점 이상이 필요합니다.")
    groups = math.ceil(len(samples) / 100)
    size, extra = divmod(len(samples), groups)
    result, offset = [], 0
    for index in range(groups):
        count = size + (1 if index < extra else 0)
        result.append(samples[offset : offset + count])
        offset += count
    return result


def continuous_side_leg(
    cake, start, end, offset_mm=0.0, max_angle_deg=1.0, max_height_mm=0.5,
):
    """원통 반지름을 유지하며 각도와 높이를 함께 변화시킨 표본을 생성한다.

    movesx는 표본 사이를 자체 보간한다. 이 함수는 목표점의 표면 일치만 보장하며
    실제 제어기의 곡선 전체가 일치한다는 보장은 하지 않는다.
    """
    if not all(
        math.isfinite(v)
        for v in (*start, *end, cake.radius, cake.height, offset_mm, max_angle_deg, max_height_mm)
    ):
        raise ValueError("연속 표면 입력은 유한 값이어야 합니다.")
    if (
        cake.radius <= 0
        or cake.height <= 0
        or offset_mm < 0
        or not 0 < max_angle_deg <= 5
        or not 0 < max_height_mm <= 1.0
    ):
        raise ValueError("연속 표면 치수·오프셋·각도 간격이 유효하지 않습니다.")
    if not all(0 <= point[1] <= cake.height for point in (start, end)):
        raise ValueError("연속 표면 높이가 케이크 밖입니다.")
    delta_arc, delta_height = end[0] - start[0], end[1] - start[1]
    count = max(
        2,
        math.ceil(abs(delta_arc / cake.radius) / math.radians(max_angle_deg)),
        math.ceil(abs(delta_height) / max_height_mm),
    )
    if count > 4096:
        raise ValueError("연속 옆면 표본 한도를 초과했습니다.")
    arcs = [start[0] + delta_arc * i / count for i in range(1, count + 1)]
    samples = [
        cake.side_abs_with_offset(arc, start[1] + delta_height * i / count, offset_mm)
        for i, arc in enumerate(arcs, start=1)
    ]
    return [
        SplineSurfaceMove(
            tuple(sample for sample, _ in group), tuple(arc for _, arc in group)
        )
        for group in split_spline_samples(list(zip(samples, arcs)))
    ]


def side_clearance_leg(cake, start, end, clearance_mm):
    """펜을 뗀 획 사이 이동을 상면 위·최대 반지름 밖에서 구성한다.

    높이가 다른 두 점을 미세 원호·수직 이동 수백 개로 나누지 않는다.
    짧은 각도는 원의 두 접선으로 돌아가므로 현으로 내부를 통과하지 않는다.
    """
    if not math.isfinite(clearance_mm) or clearance_mm <= 0:
        raise ValueError("공중 이동 여유 거리는 양수여야 합니다.")
    radius = max(cake.r_top, cake.r_bottom) + cake.radial_offset_mm + clearance_mm
    z = cake.cz + cake.height + clearance_mm
    arc0, arc1 = start[0], end[0]
    theta0, theta1 = arc0 / cake.radius, arc1 / cake.radius
    count = max(1, math.ceil(abs(theta1 - theta0) / (math.pi / 2)))

    def point(theta, distance=radius):
        angle = theta + cake.yaw
        return (cake.cx + distance * math.cos(angle),
                cake.cy + distance * math.sin(angle), z)

    moves = [SurfaceMove(point(theta0), arc0)]
    previous = theta0
    for i in range(1, count + 1):
        theta = theta0 + (theta1 - theta0) * i / count
        if abs(theta - previous) < 1e-10:
            continue
        mid = (previous + theta) / 2
        if abs(theta - previous) < math.radians(5):
            # 작은 원호 대신 두 접선의 교점을 경유한다. 선분 전체가 원 밖이다.
            corner_radius = radius / math.cos((theta - previous) / 2)
            moves.append(SurfaceMove(point(mid, corner_radius), mid * cake.radius))
            moves.append(SurfaceMove(point(theta), theta * cake.radius))
        else:
            moves.append(SurfaceMove(
                end=point(theta), arc_mm=theta * cake.radius,
                via=point(mid), via_arc_mm=mid * cake.radius,
                start_angle=previous, end_angle=theta,
            ))
        previous = theta
    moves.append(SurfaceMove(cake.side_abs_with_offset(*end, clearance_mm), arc1))
    return moves
