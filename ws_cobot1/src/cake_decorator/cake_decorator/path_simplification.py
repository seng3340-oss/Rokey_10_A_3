"""테스트 케이크2의 mm 단순화 정책을 독립 모듈로 옮긴다.

원본: test_custom_mini_cake2/drawing_core/geometry.py, planner.py.
중복 제거 → 반복 스택 RDP → 원본 대비 순서별 오차 검사 순서를 유지한다.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SimplifiedStroke:
    """선별된 원본 부분 수열과 실제 최대 오차(mm)."""

    points: list[tuple[float, float]]
    max_error_mm: float
    input_count: int


def _distance(point, start, end):
    """무한 직선이 아닌 유한 선분까지의 거리를 계산한다."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = dx * dx + dy * dy
    ratio = (
        max(
            0.0,
            min(
                1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length
            ),
        )
        if length
        else 0.0
    )
    return math.hypot(
        point[0] - start[0] - ratio * dx, point[1] - start[1] - ratio * dy
    )


def _rdp(points, tolerance):
    """재귀 제한을 피하며, 과도한 계산 시 점을 버리지 않고 원본을 반환한다."""
    if len(points) < 3:
        return points[:]
    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    work = 0
    while stack:
        start, end = stack.pop()
        if end - start < 2:
            continue
        work += end - start - 1
        if work > 100_000:
            return points[:]
        distance, index = max(
            (_distance(points[i], points[start], points[end]), i)
            for i in range(start + 1, end)
        )
        if distance > tolerance:
            keep.add(index)
            stack.extend([(start, index), (index, end)])
    return [points[i] for i in sorted(keep)]


def _ordered_error(raw, selected):
    """원본과 선별 결과의 같은 순서 구간끼리 비교한다. 자기 교차 획도 순서를 보존한다."""
    ends = [0]
    cursor = 1
    for point in selected[1:-1]:
        while cursor < len(raw) - 1 and raw[cursor] != point:
            cursor += 1
        ends.append(cursor)
        cursor += 1
    ends.append(len(raw) - 1)
    return max(
        (
            _distance(raw[i], selected[j], selected[j + 1])
            for j, (start, end) in enumerate(zip(ends, ends[1:]))
            for i in range(start, end + 1)
        ),
        default=0.0,
    )


def simplify_stroke_mm(points, tolerance_mm=0.15, duplicate_mm=0.01):
    """실제 mm 좌표에서 점을 선별한다. 허용 오차 초과 시 원본으로 되돌린다."""
    raw = [tuple(point) for point in points]
    if (
        not all(math.isfinite(value) for value in (tolerance_mm, duplicate_mm))
        or min(tolerance_mm, duplicate_mm) < 0
    ):
        raise ValueError("단순화 오차와 중복점 거리는 0 이상의 유한 값이어야 합니다.")
    if not all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in raw):
        raise ValueError("단순화 입력은 유한한 2차원 mm 좌표여야 합니다.")
    if len(raw) < 3:
        return SimplifiedStroke(raw, 0.0, len(raw))
    clean = [raw[0]]
    for point in raw[1:-1]:
        if math.dist(point, clean[-1]) > duplicate_mm:
            clean.append(point)
    clean.append(raw[-1])
    selected = _rdp(clean, tolerance_mm)
    if raw[0] == raw[-1] and len(set(selected[:-1])) < 3:
        selected = raw[:]
    error = _ordered_error(raw, selected)
    if error > tolerance_mm + 1e-9:
        selected, error = raw[:], 0.0
    return SimplifiedStroke(selected, error, len(raw))
