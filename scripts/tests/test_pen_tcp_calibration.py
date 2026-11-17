"""장비 없이 합성 접촉점으로 TCP 식별성과 좌표계 계약을 검사한다."""

import copy
import importlib.util
import json
from pathlib import Path
import unittest
import warnings

import numpy as np
from numpy.testing import assert_allclose
from scipy.spatial.transform import Rotation


# 실행 위치와 pytest의 import 모드에 무관하게 순수 계산 모듈만 읽는다.
MODULE_PATH = Path(__file__).resolve().parents[1] / "pen_tcp_calibration.py"
SPEC = importlib.util.spec_from_file_location("pen_tcp_calibration", MODULE_PATH)
calibration = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration)


def make_samples(offset=(32.0, -18.0, 145.0), rotations=None):
    """알려진 t와 c로 접촉 자세를 생성하며 실제 SDK나 보정 함수를 쓰지 않는다."""
    if rotations is None:
        rotations = Rotation.from_euler(
            "ZYZ",
            [[25, 40, -15], [-70, 85, 30], [135, 115, -60], [15, 150, 110]],
            degrees=True,
        )
    reference = np.array([430.0, -125.0, 210.0])
    positions = reference - rotations.apply(offset)
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected.*")
        angles = rotations.as_euler("ZYZ", degrees=True)
    return np.column_stack((positions, angles)).tolist()


class FitTcpTests(unittest.TestCase):
    """복원 결과와 거부 조건을 공용 fit_tcp API를 통해 검증한다."""

    def test_recovers_offset_reference_and_json_without_mutating_samples(self):
        """부호·회전 순서 오류와 원본 자세 손실을 검출한다."""
        samples = make_samples()
        original = copy.deepcopy(samples)
        result = calibration.fit_tcp(samples)
        self.assertEqual(
            set(result),
            {
                "version", "tcp_pose_mm_deg", "reference_point_base_mm",
                "max_residual_mm", "rms_residual_mm", "condition_number", "samples",
            },
        )
        self.assertEqual(result["version"], 1)
        assert_allclose(result["tcp_pose_mm_deg"][:3], [32, -18, 145], atol=1e-9)
        assert_allclose(result["reference_point_base_mm"], [430, -125, 210], atol=1e-9)
        self.assertLess(result["max_residual_mm"], 1e-9)
        self.assertLess(result["rms_residual_mm"], 1e-9)
        self.assertTrue(1 <= result["condition_number"] <= 50)
        self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), result)
        self.assertEqual(samples, original)
        self.assertEqual(result["samples"], original)
        result["samples"][0][0] += 1
        self.assertEqual(samples, original)

    def test_first_tcp_z_points_down_with_projected_flange_x(self):
        """펜 축을 플랜지 Z로 가정하거나 상대 회전 곱 순서를 바꾸는 오류를 잡는다."""
        samples = make_samples()
        result = calibration.fit_tcp(samples)
        flange = Rotation.from_euler("ZYZ", samples[0][3:], degrees=True)
        relative = Rotation.from_euler("ZYZ", result["tcp_pose_mm_deg"][3:], degrees=True)
        world = (flange * relative).as_matrix()
        expected_x = flange.as_matrix()[:, 0].copy()
        expected_x[2] = 0
        expected_x /= np.linalg.norm(expected_x)
        assert_allclose(world[:, 2], [0, 0, -1], atol=1e-12)
        assert_allclose(world[:, 0], expected_x, atol=1e-12)
        assert_allclose(world[:, 1], np.cross([0, 0, -1], expected_x), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(world), 1.0)

    def test_vertical_flange_x_falls_back_to_projected_flange_y(self):
        """첫 플랜지 X의 수평 투영이 0이어도 오른손 좌표계를 만든다."""
        rotations = Rotation.from_euler(
            "ZYZ", [[0, 90, 0], [-70, 85, 30], [135, 115, -60], [15, 150, 110]],
            degrees=True,
        )
        result = calibration.fit_tcp(make_samples(rotations=rotations))
        world = (
            rotations[0]
            * Rotation.from_euler("ZYZ", result["tcp_pose_mm_deg"][3:], degrees=True)
        ).as_matrix()
        assert_allclose(world, [[0, 1, 0], [1, 0, 0], [0, 0, -1]], atol=1e-12)

    def test_returned_tcp_produces_correct_contact_and_new_real_tcp_target(self):
        """c를 플랜지 위치로 오인하거나 실제 TCP 목표에 오프셋을 중복 적용하지 않는다."""
        samples = make_samples()
        profile = calibration.fit_tcp(samples)
        tcp_offset = np.array(profile["tcp_pose_mm_deg"][:3])
        tcp_rotation = Rotation.from_euler(
            "ZYZ", profile["tcp_pose_mm_deg"][3:], degrees=True,
        )
        for pose in samples:
            flange = Rotation.from_euler("ZYZ", pose[3:], degrees=True)
            assert_allclose(
                np.array(pose[:3]) + flange.apply(tcp_offset),
                [430, -125, 210], atol=1e-9,
            )
        # 새 TCP 목표에서 역산한 플랜지에 보정값을 다시 적용하면 목표가 복원된다.
        target_position = np.array([510.0, 35.0, 320.0])
        target_rotation = Rotation.from_euler("ZYZ", [65, 120, -35], degrees=True)
        flange_rotation = target_rotation * tcp_rotation.inv()
        flange_position = target_position - flange_rotation.apply(tcp_offset)
        assert_allclose(
            flange_position + flange_rotation.apply([32, -18, 145]),
            target_position, atol=1e-9,
        )
        assert_allclose(
            (flange_rotation * tcp_rotation).as_matrix(),
            target_rotation.as_matrix(), atol=1e-12,
        )

    def test_accepts_small_xyz_noise_and_reports_point_distance_statistics(self):
        """좌표별 오차 대신 3차원 접촉점 거리로 최대값과 RMS를 보고한다."""
        samples = np.array(make_samples())
        samples[:, :3] += [
            [0.04, -0.03, 0.02], [-0.02, 0.05, -0.04],
            [0.03, 0.02, -0.01], [-0.04, -0.02, 0.05],
        ]
        profile = calibration.fit_tcp(samples.tolist())
        assert_allclose(profile["tcp_pose_mm_deg"][:3], [32, -18, 145], atol=0.1)
        rotations = Rotation.from_euler("ZYZ", samples[:, 3:], degrees=True)
        errors = np.linalg.norm(
            samples[:, :3] + rotations.apply(profile["tcp_pose_mm_deg"][:3])
            - profile["reference_point_base_mm"], axis=1,
        )
        self.assertAlmostEqual(profile["max_residual_mm"], float(errors.max()))
        self.assertAlmostEqual(profile["rms_residual_mm"], float(np.sqrt(np.mean(errors**2))))
        self.assertGreater(profile["max_residual_mm"], 0)
        # RMS만 검사하는 구현은 이 임계값을 잘못 통과하므로 최대 거리 계약을 고정한다.
        between_rms_and_max = (
            profile["rms_residual_mm"] + profile["max_residual_mm"]
        ) / 2
        with self.assertRaisesRegex(ValueError, "residual"):
            calibration.fit_tcp(samples.tolist(), max_error_mm=between_rms_and_max)
        with self.assertRaisesRegex(ValueError, "residual"):
            calibration.fit_tcp(samples.tolist(), max_error_mm=0.001)

    def test_uses_all_samples_in_overdetermined_fit(self):
        """최소 네 자세 이후에 추가한 접촉도 계산과 검증에서 누락하지 않는다."""
        samples = make_samples()
        extra = make_samples(
            rotations=Rotation.from_euler("ZYZ", [[-35, 65, 80]], degrees=True),
        )[0]
        samples.append(extra)
        profile = calibration.fit_tcp(samples)
        self.assertEqual(profile["samples"], samples)
        assert_allclose(profile["tcp_pose_mm_deg"][:3], [32, -18, 145], atol=1e-9)
        samples[-1][2] += 8
        with self.assertRaisesRegex(ValueError, "residual"):
            calibration.fit_tcp(samples)

    def test_rejects_large_contact_error_even_when_only_one_sample_is_bad(self):
        """최소제곱이 잘못 찍은 한 점을 평균으로 숨기지 못하게 한다."""
        samples = make_samples()
        samples[-1][0] += 8
        with self.assertRaisesRegex(ValueError, "residual"):
            calibration.fit_tcp(samples)

    def test_rejects_identical_and_single_axis_rotations(self):
        """공통 회전축 방향의 오프셋이 식별되지 않는 입력을 거부한다."""
        rotations = [
            Rotation.from_euler("ZYZ", [[25, 40, -15]] * 4, degrees=True),
            Rotation.from_euler("Z", [0, 45, 90, 135], degrees=True),
            # 공통 축이 BASE Z가 아닌 경우에도 rank 결손을 검출해야 한다.
            Rotation.from_euler("X", [0, 45, 90, 135], degrees=True)
            * Rotation.from_euler("ZYZ", [25, 40, -15], degrees=True),
        ]
        for rotation in rotations:
            with self.subTest(rotation=rotation.as_quat().tolist()):
                with self.assertRaisesRegex(ValueError, "rank"):
                    calibration.fit_tcp(make_samples(rotations=rotation))

    def test_rejects_full_rank_but_insufficient_rotation_variation(self):
        """rank만 통과하는 미세 회전 집합의 오차 증폭을 조건수 50으로 제한한다."""
        rotations = Rotation.from_rotvec(
            [[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]],
        )
        with self.assertRaisesRegex(ValueError, "condition"):
            calibration.fit_tcp(make_samples(rotations=rotations))

    def test_rejects_nonfinite_coordinates_and_angles(self):
        """위치와 각도 모두 유한해야 하며 NaN이 통계 비교를 우회할 수 없다."""
        for index in range(6):
            for value in [float("nan"), float("inf"), -float("inf")]:
                samples = make_samples()
                samples[0][index] = value
                with self.subTest(index=index, value=value):
                    with self.assertRaises(ValueError):
                        calibration.fit_tcp(samples)

    def test_rejects_bad_shape_non_numeric_values_and_fewer_than_four_samples(self):
        """부모 위저드의 불완전한 수집·직렬화 결과를 계산 전에 거부한다."""
        samples = make_samples()
        malformed = [None, [], samples[:3], [samples[0]] * 3, samples + [[1] * 5]]
        for bad_row in [[1] * 7, [1] * 5, [1, 2, 3, 4, 5, "bad"], [[1]] * 6]:
            malformed.append([bad_row] + samples[1:])
        for bad in malformed:
            with self.subTest(samples=bad):
                with self.assertRaises(ValueError):
                    calibration.fit_tcp(bad)

    def test_rejects_invalid_residual_threshold(self):
        """음수·비유한 임계값이 오차 검증을 무효화하지 못하게 한다."""
        for threshold in [-1, float("nan"), float("inf"), None, "0.5", True]:
            with self.subTest(threshold=threshold):
                with self.assertRaises(ValueError):
                    calibration.fit_tcp(make_samples(), max_error_mm=threshold)

    def test_enforces_offset_norm_lower_and_controller_upper_bounds(self):
        """CAD 추정 없이 식별한 오프셋도 1 mm 초과·10000 mm 이하이어야 한다."""
        for offset in [[0, 0, 0], [0.5, 0, 0], [1, 0, 0], [10001, 0, 0], [8000, 8000, 0]]:
            with self.subTest(offset=offset):
                with self.assertRaisesRegex(ValueError, "offset"):
                    calibration.fit_tcp(make_samples(offset=offset))
        for offset in [[1.01, 0, 0], [9999.99, 0, 0], [10000, 0, 0]]:
            with self.subTest(offset=offset):
                profile = calibration.fit_tcp(make_samples(offset=offset))
                assert_allclose(profile["tcp_pose_mm_deg"][:3], offset, atol=1e-8)
                self.assertLessEqual(np.linalg.norm(profile["tcp_pose_mm_deg"][:3]), 10000)

    def test_expected_zyz_gimbal_lock_warning_does_not_escape(self):
        """결과가 ZYZ 특이점이어도 호출자의 경고 정책을 바꾸지 않는다."""
        rotations = Rotation.from_euler(
            "ZYZ", [[0, 180, 0], [-70, 85, 30], [135, 115, -60], [15, 150, 110]],
            degrees=True,
        )
        samples = make_samples(rotations=rotations)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            filters = list(warnings.filters)
            profile = calibration.fit_tcp(samples)
            self.assertEqual(warnings.filters, filters)
        self.assertEqual(caught, [])
        relative = Rotation.from_euler("ZYZ", profile["tcp_pose_mm_deg"][3:], degrees=True)
        assert_allclose((rotations[0] * relative).apply([0, 0, 1]), [0, 0, -1], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
