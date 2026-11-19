"""플레이트 구간 속도 override와 그리퍼 각도 안전예외 계약을 검사한다.

실제 로봇 없이 가짜 로봇으로 move_joint/move_line에 전달되는 인자만
확인한다. 속도 단위와 우선순위(기본값 → 이름별 override → 호출 인자)가
바뀌지 않도록 하는 회귀 테스트다.
"""

import unittest

import cake_decorator.plate_sequence as plate_sequence
from cake_decorator.plate_sequence import PlateSequence


class RecordingRobot:
    """move_joint/move_line에 전달된 kwargs를 기록하는 대역 객체."""

    def __init__(self):
        self.kwargs = {}

    def move_joint(self, joints, **kwargs):
        self.kwargs = kwargs
        return True

    def move_line(self, pos, **kwargs):
        self.kwargs = kwargs
        return True


def param(name):
    """PlateSequence가 요구하는 최소 파라미터만 돌려준다."""
    if name.startswith("plate_"):
        return [0.0] * 6
    return 0.0


class PlateSpeedOverrideTests(unittest.TestCase):
    def setUp(self):
        self.robot = RecordingRobot()
        self.sequence = PlateSequence(self.robot, param)
        # 모듈 전역 override를 테스트마다 비우고 원래 값으로 복원한다.
        self._saved = dict(plate_sequence._SPEED_OVERRIDES)
        plate_sequence._SPEED_OVERRIDES.clear()

    def tearDown(self):
        plate_sequence._SPEED_OVERRIDES.clear()
        plate_sequence._SPEED_OVERRIDES.update(self._saved)

    def test_joint_default_is_30(self):
        self.sequence._plate_joint("cake_station_ps3")
        self.assertEqual(self.robot.kwargs["speed_deg_s"], 30.0)
        self.assertEqual(self.robot.kwargs["acc_deg_s2"], 30.0)

    def test_joint_explicit_override(self):
        self.sequence._plate_joint(
            "cake_station_ps3", speed_deg_s=10.0, acc_deg_s2=10.0
        )
        self.assertEqual(self.robot.kwargs["speed_deg_s"], 10.0)
        self.assertEqual(self.robot.kwargs["acc_deg_s2"], 10.0)

    def test_line_default_is_30_100(self):
        self.sequence._plate_line("move_cake_ps_l2")
        self.assertEqual(self.robot.kwargs["speed_mm_s"], 30.0)
        self.assertEqual(self.robot.kwargs["acc_mm_s2"], 100.0)

    def test_line_explicit_override(self):
        self.sequence._plate_line(
            "move_cake_ps_l2", speed_mm_s=20.0, acc_mm_s2=60.0
        )
        self.assertEqual(self.robot.kwargs["speed_mm_s"], 20.0)
        self.assertEqual(self.robot.kwargs["acc_mm_s2"], 60.0)

    def test_named_override_applies_only_to_that_segment(self):
        plate_sequence._SPEED_OVERRIDES["cake_station_ps3"] = {
            "speed_deg_s": 10.0,
            "acc_deg_s2": 10.0,
        }
        self.sequence._plate_joint("cake_station_ps3")
        self.assertEqual(self.robot.kwargs["speed_deg_s"], 10.0)

        self.sequence._plate_joint("cake_station_ps4")
        self.assertEqual(self.robot.kwargs["speed_deg_s"], 30.0)

    def test_invalid_key_is_ignored(self):
        plate_sequence._SPEED_OVERRIDES["cake_station_ps3"] = {
            "mode": 99,
            "speed_deg_s": 10.0,
        }
        self.sequence._plate_joint("cake_station_ps3")
        # 허용 키가 아니므로 mode는 무시되고 함수의 mode(0)가 유지된다.
        self.assertNotEqual(self.robot.kwargs.get("mode"), 99)
        self.assertEqual(self.robot.kwargs["speed_deg_s"], 10.0)


class GripperAngleGuardTests(unittest.TestCase):
    def setUp(self):
        self.sequence = PlateSequence(RecordingRobot(), param)

    def test_none_raises(self):
        with self.assertRaises(RuntimeError):
            self.sequence._check_gripper_angle(None, "test")

    def test_range_pass(self):
        self.sequence._check_gripper_angle(10.0, "test", 0.0, 20.0)

    def test_below_min_raises(self):
        with self.assertRaises(RuntimeError):
            self.sequence._check_gripper_angle(-1.0, "test", 0.0, 20.0)

    def test_above_max_raises(self):
        with self.assertRaises(RuntimeError):
            self.sequence._check_gripper_angle(21.0, "test", 0.0, 20.0)


if __name__ == "__main__":
    unittest.main()
