"""실물 명령 없이 45도 파지·인출·수직 복귀와 TCP 변환을 검증한다."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location("pen_pick", SCRIPTS / "real_pen_pick_45.py")
PICK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PICK)


class PenPickTests(unittest.TestCase):
    """고정 파지 중심, 회전 후 펜 방향과 동작 실패 시 후속 명령 차단을 확인한다."""

    def setUp(self):
        """외부 팀원 폴더에 의존하지 않고 DRL에서 확인한 자세를 주입한다."""
        rack = {
            "robot_executor_node": {
                "ros__parameters": {
                    "pen_stand_approach_pose": [43.61, -1.88, 99.63, 0.09, 82.26, 43.29],
                    "pen_stand_pose": [248.86, 244.77, 312.99, 123.66, 180.0, 123.43],
                    "pen_gripper_pose": [248.91, 244.76, 548.06, 37.07, -179.99, 36.84],
                }
            }
        }
        source = Mock()
        source.read_text.return_value = yaml.safe_dump(rack)
        self.settings = PICK.read_rack_and_poses(
            SimpleNamespace(
                rack_config=source,
                tilt_heading_deg=90.0,
                jaw_center_offset=PICK.GRIP_CENTER_MM,
            )
        )

    def center(self, pose):
        """초기 TCP에서 220mm 앞인 지문 중앙의 시험 기준점을 복원한다."""
        rotation = Rotation.from_euler("ZYZ", pose[3:], degrees=True)
        return np.array(pose[:3]) + rotation.apply(PICK.GRIP_CENTER_MM)

    def test_pivot_and_final_pen_angle(self):
        """하강 뒤 회전과 공중 복귀에서 중심을 유지하며 최종 펜 각도는 45도다."""
        _, stand, turns, upper, upright = self.settings
        for pose in turns:
            np.testing.assert_allclose(self.center(pose), self.center(stand))
        for pose in upright:
            np.testing.assert_allclose(self.center(pose), self.center(upper))
        down = Rotation.from_euler("ZYZ", stand[3:], degrees=True)
        tilted = Rotation.from_euler("ZYZ", turns[-1][3:], degrees=True)
        np.testing.assert_allclose(
            tilted.apply([0, 0, 1]),
            [0, np.sqrt(0.5), -np.sqrt(0.5)],
            atol=1e-12,
        )
        final = Rotation.from_euler("ZYZ", upright[-1][3:], degrees=True)
        np.testing.assert_allclose(final.apply([0, 0, 1]), [0, 0, -1], atol=1e-12)
        # 펜은 파지 당시 BASE 수직이고, 이후 그리퍼에 고정돼 함께 회전한다.
        pen_at_grasp = tilted.inv().apply([0, 0, -1])
        pen_final = final.apply(pen_at_grasp)
        self.assertAlmostEqual(float(pen_final @ [0, 0, -1]), np.sqrt(0.5))
        self.assertLess((final.inv() * down).magnitude(), 1e-12)

    def test_open_close_lift_then_upright(self):
        """닫은 뒤 인출하고, 인출 뒤에만 수직 복귀 명령을 보낸다."""
        events = []
        robot = Mock()
        robot.move_joint.return_value = True
        robot.move_line.side_effect = lambda pose, **_: events.append(pose) or True
        robot.verify_pose.return_value = True
        grip = lambda action: events.append(action)
        with patch("builtins.print"):
            PICK.run_pick(robot, self.settings, lambda *p: list(p), grip, Mock())
        _, stand, turns, upper, upright = self.settings
        self.assertEqual(events, ["open", stand, *turns, "close", upper, *upright])

    def test_close_failure_prevents_lift_and_upright(self):
        """닫기 호출이 실패하면 인출과 수직 복귀를 실행하지 않는다."""
        robot = Mock()
        robot.move_joint.return_value = True
        robot.move_line.return_value = True
        robot.verify_pose.return_value = True

        def grip(action):
            if action == "close":
                raise RuntimeError("그리퍼 명령 실패")

        with patch("builtins.print"), self.assertRaises(RuntimeError):
            PICK.run_pick(robot, self.settings, lambda *p: list(p), grip, Mock())
        self.assertEqual(robot.move_line.call_count, 1 + len(self.settings[2]))

    def test_default_pick_runs_without_intermediate_input(self):
        """기본 집기는 입력을 기다리지 않고 높인 속도로 마지막 수직 복귀까지 간다."""
        robot = Mock()
        robot.move_joint.return_value = True
        robot.move_line.return_value = True
        robot.verify_pose.return_value = True
        with patch("builtins.print"), patch("builtins.input", side_effect=AssertionError("중간 입력 발생")):
            PICK.run_pick(robot, self.settings, lambda *p: list(p), Mock())
        self.assertEqual(robot.move_line.call_args.args[0], self.settings[4][-1])
        self.assertEqual(robot.move_line.call_args.kwargs["speed_mm_s"], 50.0)
        self.assertEqual(robot.move_joint.call_args.kwargs["speed_deg_s"], 30.0)

    def test_single_turn_uses_fixed_pivot_tcp(self):
        """220mm TCP에서 기울이기·세우기는 각각 위치 변화 없는 한 명령이다."""
        flange = [400, 50, 500, 13, 27, 44]
        rotation = Rotation.from_euler("ZYZ", flange[3:], degrees=True)
        tcp = [
            *(np.array(flange[:3]) + rotation.apply(PICK.GRIP_CENTER_MM)),
            *flange[3:],
        ]
        _, stand, turns, upper, upright = PICK.adapt_to_active_tcp(
            self.settings, flange, tcp,
        )
        self.assertEqual(len(turns), 1)
        self.assertEqual(len(upright), 1)
        np.testing.assert_allclose(stand[:3], turns[0][:3], atol=1e-9)
        np.testing.assert_allclose(upper[:3], upright[0][:3], atol=1e-9)

    def test_temporary_tcp_restores_on_success_and_failure(self):
        """실제 로봇 없이 임시 TCP 적용·정상/예외 복원·삭제를 확인한다."""
        for fail in (False, True):
            state = {"name": "grip", "mode": 1, "deleted": False}

            def call(kind, path, request, **kwargs):
                if path.endswith("get_current_tcp"):
                    return SimpleNamespace(info=state["name"])
                if path.endswith("get_robot_mode"):
                    return SimpleNamespace(robot_mode=state["mode"])
                if path.endswith("get_current_tool_flange_posx"):
                    return SimpleNamespace(pos=[0., 0., 0., 0., 0., 0.])
                if path.endswith("set_current_tcp"):
                    state["name"] = request.name
                if path.endswith("set_robot_mode"):
                    state["mode"] = request.robot_mode
                if path.endswith("config_delete_tcp"):
                    state["deleted"] = True
                return SimpleNamespace(success=True)

            session = SimpleNamespace(
                call=call, pose=lambda: [0., 0., 240., 0., 0., 0.],
            )
            try:
                with patch("builtins.print"), PICK.pivot_tcp(session, PICK.GRIP_CENTER_MM):
                    self.assertTrue(state["name"].startswith("PickPivot_"))
                    self.assertEqual(state["mode"], 1)
                    if fail:
                        raise ValueError("모의 동작 중단")
            except ValueError:
                self.assertTrue(fail)
            self.assertEqual(state["name"], "grip")
            self.assertEqual(state["mode"], 1)
            self.assertTrue(state["deleted"])

    def test_active_grip_tcp_keeps_flange_targets(self):
        """현재 grip TCP를 사용해도 티칭 당시 플랜지 경로가 바뀌지 않는다."""
        flange = [400, 50, 500, 13, 27, 44]
        rotation = Rotation.from_euler("ZYZ", flange[3:], degrees=True)
        # 로봇의 grip TCP는 240mm이고 시험의 회전 중심만 220mm인 경우를 검증한다.
        active_offset = [0.0, 0.0, 240.0]
        tcp = [*(np.array(flange[:3]) + rotation.apply(active_offset)), *flange[3:]]
        adapted = PICK.adapt_to_active_tcp(self.settings, flange, tcp)
        for original, converted in zip(
            [self.settings[1], *self.settings[2], self.settings[3], *self.settings[4]],
            [adapted[1], *adapted[2], adapted[3], *adapted[4]],
        ):
            current_rotation = Rotation.from_euler("ZYZ", converted[3:], degrees=True)
            recovered = np.array(converted[:3]) - current_rotation.apply(active_offset)
            np.testing.assert_allclose(recovered, original[:3], atol=1e-9)


if __name__ == "__main__":
    unittest.main()
