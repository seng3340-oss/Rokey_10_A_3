"""몇 개의 좋은 점만 관측하고 전체 파형 성공으로 오인하지 않게 검사한다."""

import math
import unittest
from cake_decorator.surface_wave_test import analyse


class WaveAnalysisTests(unittest.TestCase):
    def sample(self, arc):
        # 파형 시작점과 같은 위치 한 개만 반복한 잘못된 관측이다.
        qx, qy, qz, qw = (
            0.0,
            math.sin(math.radians(75)),
            0.0,
            math.cos(math.radians(75)),
        )
        return [
            500 + 50 * math.cos(arc / 50),
            50 * math.sin(arc / 50),
            82.0,
            qx,
            qy,
            qz,
            qw,
        ]

    def test_partial_wave_observation_cannot_pass(self):
        result = analyse([self.sample(113.0)] * 5, 1.0, 4.0, 113.0, 20.0, 24.0)
        self.assertFalse(result["passed"])
        self.assertGreater(result["max_reference_gap_mm"], 0.5)

    def test_full_exact_wave_passes_and_outward_axis_fails(self):
        """원본 전체를 지나도 펜 축이 바깥을 향하면 성공으로 처리하지 않는다."""
        from cake_decorator.drawing_geometry import CylinderCake, pen_quat_side

        cake = CylinderCake(radius=50.0, height=80.0, cx=500.0, cy=0.0, cz=62.0)
        samples = []
        for i in range(481):
            arc = 113.0 + 24 * i / 480
            z = 82.0 + math.sin(2 * math.pi * (arc - 113.0) / 4.0)
            q = pen_quat_side(arc, cake, tilt_deg=60.0)
            samples.append(
                [500 + 50 * math.cos(arc / 50), 50 * math.sin(arc / 50), z, *q]
            )
        good = analyse(samples, 1.0, 4.0, 113.0, 20.0, 24.0)
        self.assertTrue(good["passed"], good)
        # 수직 기울기를 유지하면서 수평 축만 뒤집는다.
        for sample in samples:
            sample[3] *= -1
            sample[4] *= -1
        bad = analyse(samples, 1.0, 4.0, 113.0, 20.0, 24.0)
        self.assertFalse(bad["passed"])
        self.assertGreater(bad["max_axis_direction_error_deg"], 50.0)

    def test_target_wait_requires_fresh_observation_after_command(self):
        """위치가 같아도 명령 전 표본이나 오래된 TF로 종점 확인을 통과시키지 않는다."""
        from types import SimpleNamespace
        from cake_decorator.surface_wave_test import WaveVerifier
        from cake_decorator.drawing_motion import DsrPos

        target = DsrPos(500.0, 0.0, 100.0, 0.0, 150.0, 0.0)
        clock = SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
        )
        node = SimpleNamespace(
            latest_tip=(500.0, 0.0, 100.0),
            latest_tip_stamp_ns=100_000_000,
            get_clock=lambda: clock,
        )
        with self.assertRaises(RuntimeError):
            WaveVerifier.wait_observed_target(
                node, target, after_ns=50_000_000, timeout_sec=0.025
            )
        node.latest_tip_stamp_ns = 950_000_000
        with self.assertRaises(RuntimeError):
            WaveVerifier.wait_observed_target(
                node, target, after_ns=960_000_000, timeout_sec=0.025
            )
        # 같은 위치라도 명령 이후의 신선한 관측이 있으면 정상 통과한다.
        WaveVerifier.wait_observed_target(
            node, target, after_ns=900_000_000, timeout_sec=0.025
        )
