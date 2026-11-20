"""첫 접근 특이점 보완이 가상 모드에만 적용되는지 검사한다."""

from types import SimpleNamespace
import unittest
import pytest

from cake_decorator.doosan_robot import DoosanRobotClient
from cake_decorator.drawing_motion import DsrPos, MotionCommand, MotionType, Phase
from cake_decorator.robot_executor_node import RobotExecutorNode


class RecordingClient(DoosanRobotClient):
    def __init__(self, mode="virtual"):
        self.mode = mode
        self.calls = []
        self.pose = [500.0, 0.0, 142.0, 40.0, 180.0, 40.0]
        self._logger = SimpleNamespace(error=lambda message: None)

    def _call(self, key, request, timeout=180.0):
        self.calls.append((key, request))
        if key == "description":
            xml = f'<robot><ros2_control><hardware><param name="mode">{self.mode}</param><param name="host">127.0.0.1</param></hardware></ros2_control></robot>'
            return SimpleNamespace(values=[SimpleNamespace(string_value=xml)])
        if key == "current_pose":
            return SimpleNamespace(
                success=True,
                task_pos_info=[SimpleNamespace(data=self.pose)],
            )
        if key == "current_joints":
            return SimpleNamespace(success=True, pos=[0.0, 20.0, 70.0, 0.0, 90.0, 0.0])
        return SimpleNamespace(success=True, sol_space=2)

    def _call_checked(self, key, request, timeout=180.0):
        self.calls.append((key, request))
        if key in ("move_jointx", "move_line"):
            self.pose = list(request.pos)
        return True


class InitialApproachTests(unittest.TestCase):
    def test_virtual_approach_lifts_then_changes_pose_at_joint_speed(self):
        robot = RecordingClient()
        target = DsrPos(500.0, 0.0, 152.0, -148.0, 150.0, 148.0)
        self.assertTrue(robot.prepare_virtual_approach(target))
        moves = [r for key, r in robot.calls if key == "move_jointx"]
        self.assertEqual(len(moves), 3)
        self.assertEqual(list(moves[0].pos), [500.0, 0.0, 252.0, 40.0, 180.0, 40.0])
        self.assertEqual(list(moves[1].pos), [350.0, 0.0, 252.0, -148.0, 150.0, 148.0])
        # 높은 지점에서 회전한 다음 첫 접근점으로 이동한다.
        self.assertEqual(list(moves[2].pos), target.to_list())
        self.assertTrue(all(r.vel <= 20.0 and r.sync_type == 0 for r in moves))

    def test_real_hardware_refuses_virtual_joint_approach(self):
        robot = RecordingClient("real")
        self.assertFalse(
            robot.prepare_virtual_approach(DsrPos(500.0, 0.0, 152.0, 0.0, 150.0, 0.0))
        )
        self.assertEqual([key for key, _ in robot.calls], ["description"])

    def test_only_first_noncontact_command_uses_virtual_approach(self):
        calls = []
        robot = SimpleNamespace(
            prepare_virtual_approach=lambda pos: calls.append("joint") or True,
            execute_drawing_command=lambda command: calls.append("normal") or True,
        )
        owner = SimpleNamespace(
            _param=lambda key: key == "virtual_initial_approach",
            _robot=robot,
            get_logger=lambda: SimpleNamespace(info=lambda message: None),
        )
        up = SimpleNamespace(
            pen_down=False, pos=DsrPos(500.0, 0.0, 152.0, 0.0, 150.0, 0.0)
        )
        down = SimpleNamespace(pen_down=True, pos=up.pos)
        for index, command in enumerate((up, down, up)):
            RobotExecutorNode._execute_motion_command(owner, command, index)
        self.assertEqual(calls, ["joint", "normal", "normal"])

    def test_disabled_option_preserves_original_approach(self):
        calls = []
        robot = SimpleNamespace(
            prepare_virtual_approach=lambda pos: self.fail("비활성화된 관절 접근 호출"),
            execute_drawing_command=lambda command: calls.append(command) or True,
        )
        owner = SimpleNamespace(_param=lambda key: False, _robot=robot)
        command = SimpleNamespace(
            pen_down=False, pos=DsrPos(500.0, 0.0, 152.0, 0.0, 150.0, 0.0)
        )
        self.assertTrue(RobotExecutorNode._execute_motion_command(owner, command, 0))
        self.assertEqual(calls, [command])


def test_success_response_without_arrival_stops_before_rotation():
    """서비스의 success=True만 믿고 다음 자세 변경을 시작하지 않아야 한다."""

    class FrozenRobot(RecordingClient):
        def _call_checked(self, key, request, timeout=180.0):
            self.calls.append((key, request))
            return True

    robot = FrozenRobot()
    assert not robot.prepare_virtual_approach(
        DsrPos(500.0, 0.0, 152.0, -148.0, 150.0, 148.0)
    )
    assert len([key for key, _ in robot.calls if key == "move_jointx"]) == 1
    assert not any(key == "move_line" for key, _ in robot.calls)


def test_side_45_does_not_stack_vertical_pen_virtual_approach():
    """45도 검증에 수직 펜용 추가100mm 이탈이 겹쳐 도달 불가가 되지 않는다."""
    from unittest.mock import Mock

    robot = Mock()
    robot._call.return_value = SimpleNamespace(
        success=True, task_pos_info=[SimpleNamespace(data=[399., -55., 270., 0., 180., 0.])],
    )
    owner = SimpleNamespace(_param=lambda key: key != 'integrated_side_cycle', _robot=robot, get_logger=Mock())
    command = MotionCommand(
        motion_type=MotionType.MOVE_LINE,
        pos=DsrPos(399., -55., 290., 0., 180., 0.),
        vel=180.0,
        acc=100.0,
        phase=Phase.REPOSITIONING,
        pen_down=False,
    )
    assert RobotExecutorNode._execute_motion_command(owner, command, 0)
    robot.prepare_virtual_approach.assert_not_called()
    assert robot.move_line.call_args.args[0].z == 320.


@pytest.mark.parametrize("index, pen_down", [(0, False), (1, False), (2, True), (3, False)])
def test_side_entry_uses_free_speed_and_preserves_drawing_interval(index, pen_down):
    """상공 도착까지만 일반 속도이며 이후 비접촉·그리기·최종 이탈은 원본을 쓴다."""
    from unittest.mock import Mock

    params = {
        **RobotExecutorNode._PARAM_DEFAULTS,
        "integrated_side_cycle": True,
        "free_line_speed_mm_s": 135.0,
        "free_line_acc_mm_s2": 270.0,
        "free_joint_speed_deg_s": 55.0,
        "free_joint_acc_deg_s2": 110.0,
    }
    robot = Mock()
    robot._call.return_value = SimpleNamespace(
        success=True,
        task_pos_info=[SimpleNamespace(data=[399., -55., 700., 0., 180., 0.])],
    )
    owner = SimpleNamespace(_param=params.__getitem__, _robot=robot)
    command = MotionCommand(
        motion_type=MotionType.MOVE_LINE,
        pos=DsrPos(399., -55., 290., 0., 180., 0.),
        vel=40.0,
        acc=100.0,
        angular_vel=20.0,
        angular_acc=40.0,
        phase=Phase.REPOSITIONING,
        pen_down=pen_down,
    )
    assert RobotExecutorNode._execute_motion_command(owner, command, index, side_45=True)
    sent = robot.execute_drawing_command.call_args.args[0]
    expected = (135., 270., 55., 110.) if index == 0 else (40., 100., 20., 40.)
    assert (sent.vel, sent.acc, sent.angular_vel, sent.angular_acc) == expected
    assert (command.vel, command.acc, command.angular_vel, command.angular_acc) == (
        40., 100., 20., 40.,
    )
