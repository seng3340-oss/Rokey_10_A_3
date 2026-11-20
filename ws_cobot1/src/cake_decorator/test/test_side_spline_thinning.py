"""옆면 movesx 표본 솎기(thin_spline_samples) 규칙을 검사한다.

규칙
    - 처음·끝 표본은 항상 남는다.
    - 빠진 표본은 남은 이웃 두 표본을 잇는 직선에서 허용치 이내다.
    - 남은 이웃 표본의 원주 거리 차이는 max_arc_mm를 넘지 않는다.
    - 허용치 0이면 그대로 돌려준다.
"""

import math

from cake_decorator.surface_path import _point_to_segment_mm, thin_spline_samples


def ring_samples(radius=60.0, step_deg=0.5, span_deg=180.0, z=100.0):
    """원통 위 수평선(같은 높이 원호) 표본. arc는 반지름 × 각도(mm)."""
    count = int(span_deg / step_deg)
    samples = []
    for i in range(count + 1):
        theta = math.radians(i * step_deg)
        samples.append(((radius * math.cos(theta), radius * math.sin(theta), z), radius * theta))
    return samples


def assert_within(samples, kept, tolerance, max_arc):
    """빠진 점은 허용치 안, 남은 이웃 간 원주 간격은 한도 안인지 확인한다."""
    assert kept[0] == samples[0] and kept[-1] == samples[-1]
    index = 0
    for start, end in zip(kept, kept[1:]):
        assert abs(end[1] - start[1]) <= max_arc + 1e-9
        while samples[index] != end:
            assert _point_to_segment_mm(samples[index][0], start[0], end[0]) <= tolerance + 1e-9
            index += 1


def test_vertical_line_collapses_to_endpoints():
    """같은 원주 위치의 수직선은 직선이므로 처음·끝 두 점만 남는다."""
    samples = [((60.0, 0.0, 100.0 + h), 0.0) for h in range(80)]
    kept = thin_spline_samples(samples, 0.05, 4.0)
    assert kept == [samples[0], samples[-1]]


def test_horizontal_arc_keeps_arc_spacing_limit():
    """0.5도 간격 원호 361점은 4도(약 4.19mm) 원주 한도로 줄되 처짐 허용치를 지킨다."""
    samples = ring_samples()
    max_arc = math.radians(4.0) * 60.0
    kept = thin_spline_samples(samples, 0.05, max_arc)
    assert len(kept) < 100 < len(samples)
    assert_within(samples, kept, 0.05, max_arc)


def test_sharp_corner_is_kept():
    """꺾이는 꼭짓점은 직선에서 멀어 반드시 남는다."""
    samples = [((float(i), 0.0, 0.0), float(i)) for i in range(11)]
    samples += [((10.0, float(i), 0.0), 10.0) for i in range(1, 11)]
    kept = thin_spline_samples(samples, 0.05, 100.0)
    assert ((10.0, 0.0, 0.0), 10.0) in kept
    assert len(kept) == 3


def test_zero_tolerance_returns_input():
    samples = ring_samples(span_deg=10.0)
    assert thin_spline_samples(samples, 0.0, 4.0) == samples
