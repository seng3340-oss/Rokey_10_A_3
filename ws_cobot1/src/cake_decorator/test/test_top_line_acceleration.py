"""윗면 직선 전환만 빠르게 하고 옆면·접촉·좌표 계약을 보존하는지 검사한다."""

from cake_decorator.drawing_motion import plan_drawing_motion


def plan(surface):
    """접근→접촉→짧은 직선 두 개→펜 올림을 실제 계획 함수에 전달한다."""
    points = [
        (0.4, 0.11),
        (0.4, 0.1),
        (0.401, 0.1),
        (0.402, 0.1),
        (0.402, 0.11),
    ]
    waypoints = [
        dict(x=x, y=0.0, z=z, qx=0.0, qy=1.0, qz=0.0, qw=0.0, speed=30.0)
        for x, z in points
    ]
    return plan_drawing_motion(waypoints, [False, True, True, True, False], surface)


def test_short_top_lines_accelerate_faster_without_changing_geometry():
    top, side = plan("TOP"), plan("SIDE")
    assert top.success and side.success

    # TOP과 SIDE 모두 기존 그리기 가속도 300을 유지하고 접촉·이탈은 100을 쓴다.
    assert [c.acc for c in top.commands] == [300.0, 100.0, 300.0, 300.0, 100.0]
    assert [c.acc for c in side.commands] == [300.0, 100.0, 300.0, 300.0, 100.0]
    assert top.total_strokes == side.total_strokes == 1

    # 속도 조정으로 경로·선속도·획 경계가 달라지지 않아야 한다.
    for first, second in zip(top.commands, side.commands):
        assert first.pos == second.pos
        assert first.vel == second.vel
        assert first.phase == second.phase
        assert first.pen_down == second.pen_down
