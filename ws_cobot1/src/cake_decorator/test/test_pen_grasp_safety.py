"""펜 파지 안전예외(A 열림/B 보유)와 서비스 메시지 계약을 검사한다.

실제 로봇 없이 가짜 로봇으로 폭 기준과 안전 정지 호출만 확인한다.
- A: 펜 상공에서 그리퍼가 60mm 이상 열렸는지
- B: 펜을 들어올린 뒤 30mm 이상 보유 중인지
- 실패 시 그리퍼 열기 + plate safe_position_1 이동 + PenGraspError
"""

import threading
import unittest
from types import SimpleNamespace

from cake_decorator.robot_executor_node import RobotExecutorNode
from cake_decorator.safety_exceptions import (
    OPENER_MISSING_MESSAGE,
    PEN_GRASP_FAILED_MESSAGE,
    OpenerMissingError,
    PenGraspError,
)


class FakePenRobot:
    """폭/그리퍼/관절 이동만 기록하는 대역 로봇."""

    def __init__(self, width):
        self.width = width
        self.open_calls = []
        self.joint_calls = []

    def get_gripper_width(self):
        return self.width

    def gripper_open(self, width_mm):
        self.open_calls.append(width_mm)
        return True

    def move_joint(self, joints):
        self.joint_calls.append(list(joints))
        return True


class FakePenNode:
    """RobotExecutorNode의 펜 안전 helper만 재사용하는 대역 노드."""

    _PEN_OPEN_MIN_WIDTH_M = RobotExecutorNode._PEN_OPEN_MIN_WIDTH_M
    _PEN_HOLD_MIN_WIDTH_M = RobotExecutorNode._PEN_HOLD_MIN_WIDTH_M
    _pen_safety_stop = RobotExecutorNode._pen_safety_stop
    _check_pen_open = RobotExecutorNode._check_pen_open
    _check_pen_held = RobotExecutorNode._check_pen_held

    def __init__(self, width):
        self._robot = FakePenRobot(width)
        self._params = {
            'gripper_open_width': 100.0,
            'plate_safe_position_1': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        }
        self._logger = SimpleNamespace(
            info=lambda *args, **kwargs: None,
            error=lambda *args, **kwargs: None,
        )

    def _param(self, name):
        return self._params[name]

    def get_logger(self):
        return self._logger


class PenOpenCheckTests(unittest.TestCase):
    def test_open_below_60mm_triggers_safe_stop(self):
        node = FakePenNode(0.050)
        with self.assertRaises(PenGraspError) as ctx:
            node._check_pen_open()
        self.assertEqual(str(ctx.exception), PEN_GRASP_FAILED_MESSAGE)
        self.assertEqual(node._robot.open_calls, [100.0])
        self.assertEqual(
            node._robot.joint_calls,
            [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]],
        )

    def test_open_at_or_above_60mm_passes(self):
        node = FakePenNode(0.080)
        node._check_pen_open()
        self.assertEqual(node._robot.open_calls, [])
        self.assertEqual(node._robot.joint_calls, [])


class PenHeldCheckTests(unittest.TestCase):
    def test_held_below_30mm_triggers_safe_stop(self):
        node = FakePenNode(0.020)
        with self.assertRaises(PenGraspError) as ctx:
            node._check_pen_held()
        self.assertEqual(str(ctx.exception), PEN_GRASP_FAILED_MESSAGE)
        self.assertEqual(len(node._robot.open_calls), 1)
        self.assertEqual(len(node._robot.joint_calls), 1)

    def test_held_at_or_above_30mm_passes(self):
        node = FakePenNode(0.080)
        node._check_pen_held()
        self.assertEqual(node._robot.open_calls, [])
        self.assertEqual(node._robot.joint_calls, [])


class FakePlateRunNode:
    """_plate_run의 예외 → 서비스 응답 변환만 검사하는 대역 노드."""

    _plate_run = RobotExecutorNode._plate_run

    def __init__(self):
        self._plate_lock = threading.Lock()
        self._logger = SimpleNamespace(
            error=lambda *args, **kwargs: None,
        )

    def _param(self, name):
        return True

    def get_logger(self):
        return self._logger


class PlateRunMessageTests(unittest.TestCase):
    def test_opener_missing_returns_message(self):
        node = FakePlateRunNode()
        response = SimpleNamespace(success=None, message=None)

        def callback():
            raise OpenerMissingError(OPENER_MISSING_MESSAGE)

        result = node._plate_run(response, 'test', callback)
        self.assertFalse(result.success)
        self.assertEqual(result.message, OPENER_MISSING_MESSAGE)


if __name__ == '__main__':
    unittest.main()
