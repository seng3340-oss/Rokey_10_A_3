"""그리퍼 DO 포트 설정과 실패 시 후속 이동 중단 계약을 검사한다.

실제 로봇·ROS 서비스 없이 DoosanRobotClient의 그리퍼 로직만 검증한다.
서비스 응답은 set_digital_output을 대체해 기록하며, sleep은 테스트에서
즉시 반환하도록 바꿔 펄스 대기 때문에 테스트가 느려지지 않게 한다.
"""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cake_decorator.doosan_robot as doosan_robot
from cake_decorator.doosan_robot import (
    GRIPPER_CLOSE_DO_DEFAULT,
    GRIPPER_OPEN_DO_DEFAULT,
    GRIPPER_PULSE_SEC_DEFAULT,
    DoosanRobotClient,
    validate_gripper_settings,
)
from cake_decorator.drawing_motion import DsrPos, TaskState
from cake_decorator.plate_sequence import PlateSequence
from cake_decorator.robot_executor_node import RobotExecutorNode


def _logger():
    """테스트에서 출력 없이 로거 인터페이스만 흉내 낸다."""
    return SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warn=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )


class FakeGripperClient(DoosanRobotClient):
    """생성자 없이 그리퍼 명령만 실행하는 기록용 객체.

    실제 DoosanRobotClient.__init__은 ROS 노드와 서비스 클라이언트를
    요구하므로, 필요한 상태만 직접 채운다. gripper_open/close와
    _gripper_pulse는 실제 구현을 그대로 사용한다.
    """

    def __init__(self, open_do=GRIPPER_OPEN_DO_DEFAULT,
                 close_do=GRIPPER_CLOSE_DO_DEFAULT,
                 pulse_sec=GRIPPER_PULSE_SEC_DEFAULT):
        self._logger = _logger()
        self._gripper_open_do = open_do
        self._gripper_close_do = close_do
        self._gripper_pulse_sec = pulse_sec
        self._gripper_backend = "digital"
        self._gripper_virtual_fallback = False
        self.gripper_output_fault = False
        self.do_calls = []
        self.fail_on = False
        self.fail_off = False

    def set_digital_output(self, index, value):
        """서비스 호출 대신 (포트, 값)을 기록하고 지정된 실패를 재현한다."""
        self.do_calls.append((index, value))
        if value == 1:
            return not self.fail_on
        return not self.fail_off


class GripperPortValidationTests(unittest.TestCase):
    def test_defaults_are_do2_open_do1_close(self):
        self.assertEqual(GRIPPER_OPEN_DO_DEFAULT, 2)
        self.assertEqual(GRIPPER_CLOSE_DO_DEFAULT, 1)
        self.assertEqual(GRIPPER_PULSE_SEC_DEFAULT, 0.5)
        self.assertEqual(
            validate_gripper_settings(2, 1, 0.5),
            (2, 1, 0.5),
        )

    def test_swapped_ports_are_allowed(self):
        self.assertEqual(validate_gripper_settings(1, 2, 0.5), (1, 2, 0.5))

    def test_port_must_be_in_range(self):
        for open_do, close_do in ((0, 1), (2, 17), (-1, 1), (2, 99)):
            with self.subTest(open_do=open_do, close_do=close_do):
                with self.assertRaises(ValueError):
                    validate_gripper_settings(open_do, close_do, 0.5)

    def test_open_and_close_must_differ(self):
        with self.assertRaises(ValueError):
            validate_gripper_settings(2, 2, 0.5)

    def test_pulse_must_be_positive_finite(self):
        for pulse in (0.0, -0.5, float("inf"), float("nan")):
            with self.subTest(pulse=pulse):
                with self.assertRaises(ValueError):
                    validate_gripper_settings(2, 1, pulse)

    def test_port_change_requires_robot_stopped(self):
        robot = FakeGripperClient()
        self.assertFalse(
            robot.configure_gripper(1, 2, 0.5, robot_stopped=False)
        )
        # 거부된 설정은 기존 포트를 바꾸지 않는다.
        self.assertEqual(robot._gripper_open_do, 2)
        self.assertEqual(robot._gripper_close_do, 1)


class GripperPulseTests(unittest.TestCase):
    def setUp(self):
        # 0.5초 펄스 대기를 실제로 기다리지 않도록 모듈의 sleep만 바꾼다.
        sleep_patch = patch.object(doosan_robot.time, "sleep", lambda _sec: None)
        sleep_patch.start()
        self.addCleanup(sleep_patch.stop)

    def test_open_pulses_do2_then_off(self):
        robot = FakeGripperClient()
        self.assertTrue(robot.gripper_open())
        self.assertEqual(robot.do_calls, [(2, 1), (2, 0)])
        self.assertFalse(robot.gripper_output_fault)

    def test_close_pulses_do1_then_off(self):
        robot = FakeGripperClient()
        self.assertTrue(robot.gripper_close())
        self.assertEqual(robot.do_calls, [(1, 1), (1, 0)])

    def test_fixed_ports_are_used_regardless_of_configuration(self):
        # 움찔거림 수정 이후 열기=DO2, 닫기=DO1이 코드에 고정됐다.
        robot = FakeGripperClient(open_do=1, close_do=2)
        self.assertTrue(robot.gripper_open())
        self.assertTrue(robot.gripper_close())
        self.assertEqual(robot.do_calls, [(2, 1), (2, 0), (1, 1), (1, 0)])

    def test_off_failure_stops_without_trying_other_port(self):
        robot = FakeGripperClient()
        robot.fail_off = True
        self.assertFalse(robot.gripper_open())
        # 같은 포트의 ON/OFF만 시도하고 반대 포트로 자동 전환하지 않는다.
        self.assertEqual(robot.do_calls, [(2, 1), (2, 0)])
        self.assertFalse(robot.gripper_output_fault)

    def test_on_failure_stops_without_trying_other_port(self):
        robot = FakeGripperClient()
        robot.fail_on = True
        self.assertFalse(robot.gripper_close())
        self.assertEqual(robot.do_calls, [(1, 1)])
        self.assertFalse(robot.gripper_output_fault)


class MovementStopsOnGripperFailureTests(unittest.TestCase):
    """pick/place가 그리퍼 실패 뒤 다음 이동을 보내지 않는지 확인한다."""

    class RecordingMovement(FakeGripperClient):
        def __init__(self, open_result=True, close_result=True):
            super().__init__()
            self.moves = []
            self.open_result = open_result
            self.close_result = close_result

        def move_line(self, pos, **kwargs):
            self.moves.append(tuple(pos.to_list()))
            return True

        def gripper_open(self, width_mm=100.0):
            self.moves.append("open")
            return self.open_result

        def gripper_close(self, width_mm=10.0, force_n=20.0):
            self.moves.append("close")
            return self.close_result

    @staticmethod
    def _poses():
        approach = DsrPos(100.0, 100.0, 200.0, 0.0, 180.0, 0.0)
        target = DsrPos(100.0, 100.0, 150.0, 0.0, 180.0, 0.0)
        return approach, target

    def test_pick_does_not_pulse_open_before_descent(self):
        approach, target = self._poses()
        robot = self.RecordingMovement(open_result=False)
        self.assertTrue(
            doosan_robot.DoosanRobotClient.pick(
                robot, target=target, approach=approach, grip_width=10.0
            )
        )
        # 접근 직후 하강하며 그리퍼 열기 펄스를 보내지 않는다.
        self.assertEqual(
            robot.moves,
            [
                tuple(approach.to_list()),
                tuple(target.to_list()),
                "close",
                tuple(approach.to_list()),
            ],
        )

    def test_pick_stops_after_close_failure(self):
        approach, target = self._poses()
        robot = self.RecordingMovement(open_result=True, close_result=False)
        self.assertFalse(
            doosan_robot.DoosanRobotClient.pick(
                robot, target=target, approach=approach, grip_width=10.0
            )
        )
        # 하강 후 닫기 실패: 들어올리기(접근 위치 복귀)를 보내지 않는다.
        self.assertEqual(
            robot.moves,
            [tuple(approach.to_list()), tuple(target.to_list()), "close"],
        )

    def test_place_stops_after_open_failure(self):
        approach, target = self._poses()
        robot = self.RecordingMovement(open_result=False)
        self.assertFalse(
            doosan_robot.DoosanRobotClient.place(
                robot, target=target, approach=approach
            )
        )
        # 하강 후 열기 실패: 후퇴 이동을 보내지 않는다.
        self.assertEqual(
            robot.moves,
            [tuple(approach.to_list()), tuple(target.to_list()), "open"],
        )


class FailSkipsHomeOnOutputFaultTests(unittest.TestCase):
    """그리퍼 OFF 실패 상태에서는 _fail이 홈 복귀를 보내지 않는지 확인한다."""

    class _Robot:
        def __init__(self, output_fault):
            self.gripper_output_fault = output_fault
            self.home_calls = 0

        def move_home(self):
            self.home_calls += 1

    def _make_node(self, output_fault):
        # rclpy 노드를 초기화하지 않고 _fail의 분기만 검사하기 위해
        # 필요한 속성만 가진 객체를 만든다.
        node = object.__new__(RobotExecutorNode)
        node._state = TaskState.DRAWING
        node._robot = self._Robot(output_fault)
        node.get_logger = _logger
        return node

    def test_output_fault_skips_home(self):
        node = self._make_node(output_fault=True)
        node._fail("그리퍼 OFF 실패")
        self.assertEqual(node._state, TaskState.FAILED)
        self.assertEqual(node._robot.home_calls, 0)

    def test_normal_failure_still_homes(self):
        node = self._make_node(output_fault=False)
        node._fail("일반 실패")
        self.assertEqual(node._state, TaskState.FAILED)
        self.assertEqual(node._robot.home_calls, 1)


class PlateSequenceReuseTests(unittest.TestCase):
    """플레이트 동작이 전용 함수 대신 기존 pick/place를 쓰는지 확인한다."""

    PARAM_NAMES = (
        "plate_safe_position_1",
        "plate_pick_opener1",
        "plate_pick_opener2",
        "plate_move_cake_ps_l2",
        "plate_move_cake_plate_ps1",
        "plate_move_cake_plate_ps2",
        "plate_move_cake_plate_ps3",
        "plate_move_cake_plate_ps4",
        "plate_safe_position_2",
        "plate_cake_station_ps1",
        "plate_cake_station_ps2",
        "plate_cake_station_ps3",
        "plate_cake_station_ps4",
        "plate_safe_position_3",
        "plate_pickup_cake_station_ps1",
        "plate_pickup_cake_station_ps2",
        "plate_pickup_cake_station_ps3",
        "plate_pickup_cake_station_ps4",
        "plate_safe_position_4",
        "plate_deliver_cake_station_ps1",
        "plate_deliver_cake_station_ps2",
        "plate_deliver_cake_station_ps3",
        "plate_deliver_cake_station_ps4",
        "plate_deliver_cake_station_ps5",
        "plate_safe_position_5",
    )

    class PlateRobot:
        def __init__(self, pick_result=True):
            self.calls = []
            self.pick_kwargs = {}
            self.place_kwargs = {}
            self.pick_result = pick_result

        def move_joint(
            self, joints, speed_deg_s=30.0, acc_deg_s2=30.0, mode=None,
        ):
            self.calls.append("move_joint")
            return True

        def move_line(
            self, pos, speed_mm_s=50.0, acc_mm_s2=100.0, ref=None, mode=None,
        ):
            self.calls.append("move_line")
            return True

        def pick(self, **kwargs):
            self.calls.append("pick")
            self.pick_kwargs = kwargs
            return self.pick_result

        def place(self, **kwargs):
            self.calls.append("place")
            self.place_kwargs = kwargs
            return True

    def _params(self):
        params = {name: [0.0] * 6 for name in self.PARAM_NAMES}
        params.update(
            {
                "plate_pick_opener1": [426.73, 252.72, 379.41, 30.09, 179.36, 28.13],
                "plate_pick_opener2": [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
                "opener_grip_width_mm": 10.0,
                "opener_grip_force_n": 20.0,
                "opener_open_width_mm": 100.0,
                "opener_close_wait_sec": 0.5,
                "opener_close_settle_sec": 0.5,
                "opener_open_wait_sec": 1.0,
                "opener_pre_open_wait_sec": 1.0,
                # 실제 노드가 선언하는 플레이트 속도 파라미터도 제공한다.
                "free_joint_speed_deg_s": 45.0,
                "free_joint_acc_deg_s2": 45.0,
                "free_line_speed_mm_s": 75.0,
                "free_line_acc_mm_s2": 150.0,
                "cake_lift_speed_mm_s": 25.0,
                "cake_lift_acc_mm_s2": 50.0,
                "cake_seat_joint_speed_deg_s": 15.0,
                "cake_seat_joint_acc_deg_s2": 15.0,
            }
        )
        return params

    def test_plate_functions_are_not_reintroduced(self):
        self.assertFalse(hasattr(DoosanRobotClient, "plate_pick"))
        self.assertFalse(hasattr(DoosanRobotClient, "plate_place"))
        self.assertFalse(hasattr(DoosanRobotClient, "plate_gripper_open"))

    def test_prepare_uses_existing_pick(self):
        params = self._params()
        robot = self.PlateRobot()
        sequence = PlateSequence(robot, params.__getitem__)
        self.assertTrue(sequence._plate_prepare_steps())
        self.assertIn("pick", robot.calls)
        # 드로잉 전 그리퍼를 비우기 위해 prepare 마지막에서 place를 호출한다.
        self.assertIn("place", robot.calls)
        # 플레이트 전용 함수가 아니라 기존 pick의 확장 인수를 사용한다.
        self.assertIn("close_wait_sec", robot.pick_kwargs)
        self.assertIn("pre_open_wait_sec", robot.pick_kwargs)
        self.assertIn("open_wait_sec", robot.place_kwargs)

    def test_finish_uses_existing_place(self):
        params = self._params()
        robot = self.PlateRobot()
        sequence = PlateSequence(robot, params.__getitem__)
        self.assertTrue(sequence._plate_finish_steps())
        # 드로잉 후 케이크를 옮기기 전에 opener를 다시 파지한다.
        self.assertIn("pick", robot.calls)
        self.assertIn("place", robot.calls)
        self.assertIn("open_wait_sec", robot.place_kwargs)

    def test_pick_failure_stops_remaining_plate_moves(self):
        params = self._params()
        robot = self.PlateRobot(pick_result=False)
        sequence = PlateSequence(robot, params.__getitem__)
        self.assertFalse(sequence._plate_prepare_steps())
        # 안전 자세 이동과 pick 시도까지만 하고 이후 좌표 이동은 없다.
        self.assertEqual(robot.calls, ["move_joint", "pick"])


if __name__ == "__main__":
    unittest.main()
