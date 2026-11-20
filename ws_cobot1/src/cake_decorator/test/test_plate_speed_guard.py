"""플레이트 구간 속도 override와 그리퍼 각도 안전예외 계약을 검사한다.

실제 로봇 없이 가짜 로봇으로 move_joint/move_line에 전달되는 인자만
확인한다. 속도 단위와 우선순위(기본값 → 이름별 override → 호출 인자)가
바뀌지 않도록 하는 회귀 테스트다.
"""

import math
import unittest
from types import SimpleNamespace

import cake_decorator.plate_sequence as plate_sequence
from cake_decorator.doosan_robot import DoosanRobotClient
from cake_decorator.plate_sequence import PlateSequence
from cake_decorator.plate_weight_check import PlateWeightChecker
from cake_decorator.safety_exceptions import OpenerMissingError


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


class WeightRobot:
    """정해진 하중값을 순서대로 돌려주는 대역 객체."""

    def __init__(self, values):
        self.values = list(values)

    def get_workpiece_weight(self):
        return self.values.pop(0)


class CakeMinDeltaTests(unittest.TestCase):
    """케이크 보유 단계는 최소 변화량(75g)만 차단 기준으로 사용한다."""

    def _checker(self, values):
        return PlateWeightChecker(
            WeightRobot(values),
            opener_kg=0.05,
            cake_kg=0.20,
            tolerance_percent=10.0,
            cake_min_delta_kg=0.075,
            phase='prepare',
        )

    def _run(self, values):
        checker = self._checker(values)
        checker.start()
        opener = checker.check('prepare_opener')
        both = checker.check('prepare_both')
        return opener, both

    def test_cake_delta_below_min_fails(self):
        _, both = self._run([1.200, 1.250, 1.300])
        self.assertFalse(both.passed)
        self.assertAlmostEqual(both.cake_delta_kg, 0.050)

    def test_cake_delta_above_min_passes(self):
        _, both = self._run([1.200, 1.250, 1.340])
        self.assertTrue(both.passed)
        self.assertAlmostEqual(both.cake_delta_kg, 0.090)

    def test_noisy_opener_does_not_block_cake_check(self):
        # 따개 단독 측정이 기준보다 낮아도 차단하지 않고 기준값으로만 쓴다.
        opener, both = self._run([1.200, 1.150, 1.250])
        self.assertTrue(opener.passed)
        self.assertTrue(both.passed)
        self.assertAlmostEqual(both.cake_delta_kg, 0.100)

    def test_exact_min_delta_fails(self):
        _, both = self._run([1.200, 1.250, 1.325])
        self.assertFalse(both.passed)
        self.assertAlmostEqual(both.cake_delta_kg, 0.075)

    def test_two_reads_are_averaged(self):
        checker = PlateWeightChecker(
            WeightRobot([1.0, 1.2]),
            opener_kg=0.05,
            cake_kg=0.20,
            tolerance_percent=10.0,
            cake_min_delta_kg=0.075,
            phase='prepare',
            reads=2,
            read_interval=0.0,
        )
        self.assertAlmostEqual(checker.start(), 1.1)


class RecoveryRobot:
    """무게 실패 복구 경로를 검사하기 위한 대역 로봇."""

    def __init__(self, weights, place_result=True, width=0.1):
        self.weights = list(weights)
        self.place_result = place_result
        self.width = width
        self.calls = []
        self.place_kwargs = {}
        self._logger = SimpleNamespace(
            info=lambda *args, **kwargs: None,
            warn=lambda *args, **kwargs: None,
            error=lambda *args, **kwargs: None,
        )

    def get_workpiece_weight(self):
        return self.weights.pop(0)

    def get_gripper_width(self):
        return self.width

    def get_gripper_angle_deg(self):
        return 0.0

    def gripper_open(self, width_mm):
        self.calls.append('open')
        return True

    def place(self, **kwargs):
        self.calls.append('place')
        self.place_kwargs = kwargs
        return self.place_result


def _weight_params():
    return {
        'plate_weight_check_enabled': True,
        'plate_weight_wait_sec': 0.0,
        'plate_opener_weight_kg': 0.05,
        'plate_cake_weight_kg': 0.20,
        'plate_weight_tolerance_percent': 10.0,
        'plate_cake_min_delta_kg': 0.075,
        'plate_weight_reads': 1,
        'plate_weight_read_interval': 0.0,
        'plate_pick_opener1': [0.0] * 6,
        'plate_pick_opener2': [0.0] * 6,
        'opener_open_width_mm': 100.0,
        'opener_open_wait_sec': 0.0,
        'opener_pre_open_wait_sec': 0.0,
    }


class WeightFailureRecoveryTests(unittest.TestCase):
    """무게 75g 실패 시 opener 복구가 실제로 호출되는지 검사한다."""

    def _sequence_until_both(self, robot):
        params = _weight_params()
        sequence = PlateSequence(robot, params.__getitem__)
        self.assertTrue(sequence._weight_at_safe('prepare_start'))
        self.assertTrue(sequence._weight_at_safe('prepare_opener'))
        sequence._opener_held = True
        return sequence

    def test_both_failure_recovers_opener_at_half_speed(self):
        robot = RecoveryRobot([1.200, 1.250, 1.300])
        sequence = self._sequence_until_both(robot)

        self.assertFalse(sequence._weight_at_safe('prepare_both'))
        self.assertIn('place', robot.calls)
        self.assertAlmostEqual(robot.place_kwargs['speed_approach'], 15.0)
        self.assertAlmostEqual(robot.place_kwargs['speed_slow'], 15.0)
        self.assertAlmostEqual(robot.place_kwargs['speed_retract'], 15.0)
        self.assertFalse(sequence._opener_held)

    def test_recovery_failure_keeps_opener_flag(self):
        robot = RecoveryRobot([1.200, 1.250, 1.300], place_result=False)
        sequence = self._sequence_until_both(robot)

        self.assertFalse(sequence._weight_at_safe('prepare_both'))
        self.assertIn('place', robot.calls)
        self.assertTrue(sequence._opener_held)

    def test_missing_opener_opens_and_raises(self):
        robot = RecoveryRobot([], width=0.0125)
        params = _weight_params()
        sequence = PlateSequence(robot, params.__getitem__)
        with self.assertRaises(OpenerMissingError):
            sequence._check_gripper_grasp('prepare_opener')
        self.assertIn('open', robot.calls)
        self.assertFalse(sequence._opener_held)


class GripperWidthLocalTests(unittest.TestCase):
    """/onrobot/pose 제한 없이 각도로 폭을 계산하는지 검사한다."""

    def _client(self):
        # __init__ 없이 순수 계산 메서드만 검사한다.
        return object.__new__(DoosanRobotClient)

    def test_open_angle_matches_observed_width(self):
        width = self._client().gripper_width_from_angle(
            math.radians(1.9000511035933385)
        )
        self.assertAlmostEqual(width, 0.0817, delta=0.001)

    def test_closed_angle_bypasses_service_limit(self):
        # 38.98°는 /onrobot/pose의 36° 제한을 넘지만 로컬 계산은 가능하다.
        width = self._client().gripper_width_from_angle(
            math.radians(38.976613858262915)
        )
        self.assertAlmostEqual(width, 0.0185, delta=0.001)


if __name__ == "__main__":
    unittest.main()
