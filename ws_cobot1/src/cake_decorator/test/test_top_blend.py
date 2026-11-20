"""윗면 연속 직선을 moveb(MoveBlending) 묶음으로 바꾸는 규칙을 실물 명령 없이 검사한다.

규칙
    - 펜이 닿은 채 이어지는 TOP 직선(DRAWING)만 묶는다. 접촉 진입·이탈·공중 이동은 movel 그대로.
    - 한 묶음은 50세그먼트 이하이며, 넘으면 고르게 나눈다.
    - 꼭짓점 반경은 최대 반경과 앞뒤 세그먼트 길이 × 0.45 중 가장 작은 값, 묶음 끝은 0.
    - 그 값이 제어기 최소 반경(1mm) 미만인 꼭짓점에서는 묶음을 끊어 정지한다.
    - 목표점 좌표는 바뀌지 않는다.
"""

import math

import pytest

from cake_decorator.drawing_motion import (
    BLEND_MAX_SEGMENTS,
    MotionType,
    Phase,
    blend_top_drawing,
    plan_drawing_motion,
)
from test_surface_command_contract import RequestRecorder


def top_plan(points, speed=40.0):
    """상공 → 접촉 → points 직선들 → 이탈 순서의 TOP 계획을 만든다(단위 mm)."""
    x0, y0 = points[0]
    raw = [(x0, y0, 160.0, False), (x0, y0, 150.0, True)]
    raw += [(x, y, 150.0, True) for x, y in points[1:]]
    raw.append((points[-1][0], points[-1][1], 160.0, False))
    waypoints = [
        dict(x=x / 1000.0, y=y / 1000.0, z=z / 1000.0,
             qx=0.0, qy=1.0, qz=0.0, qw=0.0, speed=speed, pen_down=pen)
        for x, y, z, pen in raw
    ]
    plan = plan_drawing_motion(waypoints, [w["pen_down"] for w in waypoints], "TOP")
    assert plan.success, plan.error_msg
    return plan


def test_contact_lines_become_one_moveb_and_targets_are_kept():
    """접촉 직선 4개는 moveb 하나가 되고 접근·접촉·이탈은 movel로 남는다."""
    plan = top_plan([(400, 0), (410, 0), (410, 10), (400, 10), (400, 20)])
    blended = blend_top_drawing(plan.commands, 1.5)
    kinds = [c.motion_type for c in blended]
    assert kinds == [
        MotionType.MOVE_LINE,   # 상공
        MotionType.MOVE_LINE,   # 접촉 진입(PEN_DOWN)
        MotionType.MOVE_BLEND,  # 그리기 4세그먼트
        MotionType.MOVE_LINE,   # 이탈(PEN_UP)
    ]
    moveb = blended[2]
    drawing = [c.pos for c in plan.commands if c.phase == Phase.DRAWING]
    assert [pos for pos, _ in moveb.blend] == drawing
    # 세그먼트 길이 10mm → 최대 반경 1.5mm가 그대로 쓰이고 마지막만 0이다.
    assert [r for _, r in moveb.blend] == [1.5, 1.5, 1.5, 0.0]
    assert moveb.phase == Phase.DRAWING and moveb.pen_down


def test_radius_shrinks_but_never_below_controller_minimum():
    """3mm 세그먼트 옆 꼭짓점은 1.35mm로 줄어 구간 겹침 오류를 피한다(1mm 이상이라 허용)."""
    plan = top_plan([(400, 0), (410, 0), (413, 0), (423, 0)])
    moveb = blend_top_drawing(plan.commands, 1.5)[2]
    assert [r for _, r in moveb.blend] == pytest.approx([1.35, 1.35, 0.0])


def test_corner_under_1mm_radius_splits_the_group():
    """1mm 세그먼트 옆 꼭짓점은 0.45mm라 제어기가 거부(경보 1205)하므로 거기서 끊는다.

    10,10,1,10,10mm 세그먼트 → moveb(2) · movel(1mm 구간) · moveb(2).
    """
    plan = top_plan([(400, 0), (410, 0), (420, 0), (421, 0), (431, 0), (441, 0)])
    blended = blend_top_drawing(plan.commands, 1.5)
    drawing = [c for c in blended if c.phase == Phase.DRAWING]
    assert [c.motion_type for c in drawing] == [
        MotionType.MOVE_BLEND, MotionType.MOVE_LINE, MotionType.MOVE_BLEND,
    ]
    assert [r for _, r in drawing[0].blend] == [1.5, 0.0]
    assert [r for _, r in drawing[2].blend] == [1.5, 0.0]
    # 모든 moveb 꼭짓점 반경은 0(끝) 또는 1mm 이상이다.
    for command in blended:
        for _, radius in command.blend or []:
            assert radius == 0.0 or radius >= 1.0


def test_long_stroke_is_split_evenly_under_segment_limit():
    """51세그먼트는 50+1이 아니라 26+25로 나눠 두 묶음 모두 moveb가 된다."""
    # 세그먼트를 모두 1mm 반경 이상이 나오는 길이(약 5.4mm)로 만든다.
    points = [(400 + 5 * i, (i % 2) * 2) for i in range(52)]
    blended = blend_top_drawing(top_plan(points).commands, 1.5)
    moves = [c for c in blended if c.motion_type == MotionType.MOVE_BLEND]
    assert [len(c.blend) for c in moves] == [26, 25]
    assert all(len(c.blend) <= BLEND_MAX_SEGMENTS for c in moves)
    # 첫 묶음의 끝점에서 정지한 뒤 두 번째 묶음이 이어진다.
    assert moves[0].blend[-1][1] == 0.0


def test_single_contact_line_stays_movel_and_zero_radius_disables():
    """그리기 직선이 하나뿐이면 묶지 않고, 반경 0이면 계획을 그대로 둔다."""
    plan = top_plan([(400, 0), (410, 0)])
    assert all(
        c.motion_type == MotionType.MOVE_LINE
        for c in blend_top_drawing(plan.commands, 1.5)
    )
    plan = top_plan([(400, 0), (410, 0), (410, 10)])
    assert blend_top_drawing(plan.commands, 0.0) == plan.commands
    # 제어기 최소(1mm)보다 작은 설정은 블렌딩할 수 없으므로 끈 것과 같다.
    assert blend_top_drawing(plan.commands, 0.5) == plan.commands


def test_moveb_request_matches_driver_segment_format():
    """드라이버 moveb_cb가 읽는 [목표 6, posx2 6, LINE=0, 반경] 형식으로 보낸다."""
    plan = top_plan([(400, 0), (410, 0), (410, 10)])
    moveb = blend_top_drawing(plan.commands, 1.5)[2]
    robot = RequestRecorder()
    assert robot.execute_drawing_command(moveb)
    key, request = robot.requests[-1]
    assert key == "move_blending"
    assert request.pos_cnt == 2
    assert list(request.vel) == [moveb.vel, moveb.angular_vel]
    first = list(request.segment[0].data)
    assert first[:3] == pytest.approx([410.0, 0.0, 150.0])
    assert first[6:12] == [0.0] * 6
    assert first[12:] == [0.0, 1.5]
    assert list(request.segment[-1].data)[13] == 0.0
    assert math.isclose(request.segment[-1].data[1], 10.0)
