"""케이크 중심 상공 → 첫 획 시작점 상공 → 수직 하강 순서를 검사한다.

실제 로봇·ROS 노드 없이 RobotExecutorNode의 메서드만 가짜 로봇으로 호출한다.
"""

from types import SimpleNamespace
import unittest

from cake_decorator.drawing_motion import DsrPos
from cake_decorator.robot_executor_node import RobotExecutorNode


class FakeRobot:
    """이동 명령을 기록하고, 현재 TCP를 마지막 move_line 목표로 갱신한다."""

    def __init__(self, pose):
        self.pose = pose
        self.calls = []

    def move_joint(self, joints, **kwargs):
        self.calls.append(("move_joint", list(joints)))
        return True

    def move_line(self, pos, **kwargs):
        self.calls.append(("move_line", pos.to_list()))
        self.pose = pos.to_list()
        return True

    def current_pose(self):
        return self.pose


def make_executor(robot):
    """노드 생성 없이 필요한 속성만 가진 실행기 대역을 만든다."""
    failures = []
    params = {
        # 자유 이동 설정을 명시해 실제 노드처럼 공중 이동만 빠르게 실행한다.
        "free_joint_speed_deg_s": 60.0,
        "free_joint_acc_deg_s2": 120.0,
        "pen_travel_speed_mm_s": 180.0,
        "pen_motion_acc_mm_s2": 400.0,
        "pen_rotation_speed_deg_s": 90.0,
        "pen_rotation_acc_deg_s2": 180.0,
        "cake_center_approach_pose": [-8.65, 8.16, 63.77, 0.0, 108.71, -9.48],
        "cake_center_pose": [401.8, -55.57, 510.39, 179.76, 179.35, 179.02],
    }
    executor = SimpleNamespace(
        _robot=robot,
        _param=params.__getitem__,
        _fail=failures.append,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )
    return executor, failures


def plan_starting_at(x, y, z):
    """DP의 첫 접근점(펜 올림) 하나만 가진 계획."""
    return SimpleNamespace(
        commands=[SimpleNamespace(pos=DsrPos(x, y, z, 179.76, 179.35, 179.02))]
    )


class CakeCenterApproachTests(unittest.TestCase):
    def test_go_to_cake_center_only_moves_above_center(self):
        """케이크 중심 함수는 상공 관절 이동만 하고 하강하지 않는다."""
        robot = FakeRobot([400.0, -50.0, 700.0, 0.0, 180.0, 0.0])
        executor, failures = make_executor(robot)
        self.assertTrue(RobotExecutorNode._go_to_cake_center(executor, None, None))
        self.assertEqual(
            robot.calls,
            [("move_joint", [-8.65, 8.16, 63.77, 0.0, 108.71, -9.48])],
        )
        self.assertEqual(failures, [])

    def test_moves_horizontally_above_first_stroke_at_current_height(self):
        """상공 높이는 유지하고 XY·자세만 첫 접근점에 맞춘다."""
        robot = FakeRobot([400.0, -50.0, 700.0, 0.0, 180.0, 0.0])
        executor, failures = make_executor(robot)
        plan = plan_starting_at(380.0, -40.0, 520.39)
        self.assertTrue(RobotExecutorNode._move_above_first_stroke(executor, plan))
        self.assertEqual(
            robot.calls,
            [("move_line", [380.0, -40.0, 700.0, 179.76, 179.35, 179.02])],
        )
        self.assertEqual(failures, [])

    def test_refuses_when_hover_height_is_below_first_approach(self):
        """상공이 접근점보다 낮으면 케이크 위를 가로지르지 않도록 이동하지 않는다."""
        robot = FakeRobot([400.0, -50.0, 515.0, 0.0, 180.0, 0.0])
        executor, failures = make_executor(robot)
        plan = plan_starting_at(380.0, -40.0, 520.39)
        self.assertFalse(RobotExecutorNode._move_above_first_stroke(executor, plan))
        self.assertEqual(robot.calls, [])
        self.assertEqual(len(failures), 1)

    def test_refuses_when_current_pose_is_unknown(self):
        """현재 TCP 조회에 실패하면 이동하지 않는다."""
        robot = FakeRobot(None)
        executor, failures = make_executor(robot)
        plan = plan_starting_at(380.0, -40.0, 520.39)
        self.assertFalse(RobotExecutorNode._move_above_first_stroke(executor, plan))
        self.assertEqual(robot.calls, [])
        self.assertEqual(len(failures), 1)


if __name__ == "__main__":
    unittest.main()
