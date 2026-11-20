"""원뿔대·회전·별도 SIDE 교정점의 표시를 검사한다. ROS 노드는 실행하지 않는다."""

import math
from types import SimpleNamespace as NS

import pytest
from cake_decorator.drawing_geometry import FrustumCake
from cake_decorator.hmi_plan import build_hmi_plan


def point(x, y, z):
    """메시지와 같은 위치 속성만 만들어 표시 어댑터에 전달한다."""
    return NS(x=x, y=y, z=z)


def waypoint(position, down=True, motion=0, samples=()):
    """명령 제어점 사본을 만들며 드라이버 의존성을 갖지 않는다."""
    return NS(pose=NS(position=position), pen_down=down, motion_type=motion,
              spline_poses=[NS(position=p) for p in samples])


def test_side_uses_frustum_yaw_and_actual_side_reference_with_spline():
    """TOP 영점과 다른 SIDE 영점에서 생성한 스플라인도 같은 캔버스로 복원한다."""
    cake = NS(radius=.05675, bottom_radius=.06, height=.079, orientation=.8,
              center=point(.3, .1, .51))
    reference = NS(position=point(.399, -.05, .222))
    shape = FrustumCake(.05675, .06, .079, .399, -.05, .143, yaw=.8)
    positions = [point(*shape.side_to_abs(t * shape.radius_at(h), h))
                 for t, h in [(1., .02), (1.2, .03), (1.4, .04)]]
    commands = [waypoint(positions[0]), waypoint(positions[-1], motion=2, samples=positions[1:])]
    drawing = NS(canvas_width=1024, canvas_height=768, surface="SIDE")
    plan = build_hmi_plan("side", drawing, cake, commands, reference_pose=reference)
    projected = plan['strokes'][0]['points']
    assert len(projected) == 3
    for result, angle, h in zip(projected, (1., 1.2, 1.4), (.02, .03, .04)):
        strip_h = 896 * math.hypot(.079, .06-.05675) / (math.pi * (.06+.05675))
        assert result['x'] == pytest.approx(64 + 896 * angle / (2*math.pi))
        assert result['y'] == pytest.approx((768-strip_h)/2 + strip_h*(1-h/.079))
    assert commands[1].spline_poses[0].position is positions[1]


def test_pen_lifts_split_strokes_and_bad_positions_are_rejected():
    """펜올림을 화면에서 잇지 않으며 잘못된 좌표를 실제 계획처럼 표시하지 않는다."""
    cake = NS(radius=.05675, bottom_radius=.06, height=.079, orientation=0.,
              center=point(.4, 0., .22))
    drawing = NS(canvas_width=1024, canvas_height=768, surface="TOP")
    a, b = point(.4, 0., .22), point(.41, 0., .22)
    commands = [waypoint(a), waypoint(b), waypoint(a, down=False), waypoint(a), waypoint(b)]
    assert len(build_hmi_plan('top', drawing, cake, commands)['strokes']) == 2
    commands[-1].pose.position.x = float('nan')
    with pytest.raises(ValueError, match='유한'):
        build_hmi_plan('bad', drawing, cake, commands)
