"""옆면 띠 좌우 이음매 겹침(overlap_side_seam)의 연장 조건을 검증한다.

호 길이는 경로 기준(평균 반지름) mm, 높이는 바닥 기준 mm다.
한 바퀴 SEAM을 100mm로 단순화해 연장 위치를 숫자로 바로 확인한다.
"""

import math

import pytest

from cake_decorator.frustum_path import overlap_side_seam
from test_frustum_command_contract import node  # noqa: F401  실제 DP 노드 픽스처 재사용

SEAM = 100.0


def test_full_width_stroke_extends_past_seam_at_left_height():
    """왼쪽 끝에서 오른쪽 끝까지 그은 한 획은 왼쪽 시작 높이로 3mm 더 그린다."""
    strokes = [[(0.0, 40.0), (50.0, 41.0), (SEAM, 42.0)]]
    result = overlap_side_seam(strokes, SEAM, 3.0)
    assert result[0][-1] == (SEAM + 3.0, 40.0)
    # 입력 목록은 바꾸지 않는다.
    assert len(strokes[0]) == 3


def test_right_to_left_stroke_prepends_overlap():
    """오른쪽 끝에서 시작한 획은 시작 앞에 겹침 점을 넣어 이음매를 먼저 지난다."""
    result = overlap_side_seam([[(SEAM, 30.0), (0.0, 30.5)]], SEAM, 3.0)
    assert result[0][0] == (SEAM + 3.0, 30.5)
    assert result[0][1:] == [(SEAM, 30.0), (0.0, 30.5)]


def test_two_strokes_meeting_at_seam_extend_right_end():
    """두 획이 각각 오른쪽·왼쪽 끝에 닿으면 오른쪽 끝 획만 연장한다."""
    strokes = [[(0.0, 50.0), (40.0, 50.0)], [(60.0, 51.0), (SEAM, 51.0)]]
    result = overlap_side_seam(strokes, SEAM, 3.0)
    assert result[0] == strokes[0]
    assert result[1][-1] == (SEAM + 3.0, 50.0)


@pytest.mark.parametrize(
    "strokes",
    [
        # 오른쪽 끝에만 닿은 획: 반대편에 의도하지 않은 표시를 만들지 않는다.
        [[(60.0, 40.0), (SEAM, 40.0)]],
        # 양쪽 끝 높이 차이가 허용치(3mm)보다 크면 닫힌 링이 아니다.
        [[(0.0, 20.0), (SEAM, 30.0)]],
        # 이음매에서 떨어진 획.
        [[(10.0, 40.0), (90.0, 40.0)]],
    ],
)
def test_open_strokes_are_unchanged(strokes):
    assert overlap_side_seam(strokes, SEAM, 3.0) == strokes


def test_zero_overlap_disables_extension():
    strokes = [[(0.0, 40.0), (SEAM, 40.0)]]
    assert overlap_side_seam(strokes, SEAM, 0.0) == strokes


@pytest.mark.parametrize("overlap", [-1.0, 21.0, math.nan])
def test_invalid_overlap_is_rejected(overlap):
    with pytest.raises(ValueError):
        overlap_side_seam([], SEAM, overlap)


def _last_pen_down_angle_deg(response):
    """마지막 펜 접촉 목표점의 케이크 중심(BASE 0.4, 0) 기준 방위각(도)."""
    last = [w for w in response.waypoints_3d if w.pen_down][-1]
    return math.degrees(
        math.atan2(last.pose.position.y, last.pose.position.x - 0.4)
    )


@pytest.mark.parametrize("overlap, expected_deg", [(0.0, 0.0), (3.0, 3.0 / 58.375)])
def test_full_ring_from_web_edges_overlaps_seam_in_dp(node, overlap, expected_deg):
    """웹 띠 x=64→960 한 획이 실제 DP에서 0도를 지나 겹침 길이만큼 더 그려진다.

    expected_deg의 3.0/58.375는 라디안 값이다. 평균 반지름 58.375mm에서 3mm 호의 각도다.
    """
    from rclpy.parameter import Parameter
    from test_frustum_command_contract import convert

    node.set_parameters([Parameter("side_seam_overlap_mm", value=overlap)])
    response, _ = convert(node, [[(64, 384), (512, 384), (960, 384)]])
    assert _last_pen_down_angle_deg(response) == pytest.approx(
        math.degrees(expected_deg), abs=0.05,
    )
