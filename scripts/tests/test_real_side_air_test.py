"""새 펜촉 TCP를 기준으로 공중 원주와 실제 펜 방향을 확인한다."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from real_side_air_test import continuous_j6, make_path


class SideAirTests(unittest.TestCase):
    """원주 끝점·반지름·펜축·그리퍼 수직과 초기 이탈을 확인한다."""

    def setUp(self):
        """원본 파일을 바꾸지 않고 기존 실물 좌표 구조를 모의 입력한다."""
        rack = Mock()
        rack.read_text.return_value = yaml.safe_dump({
            "robot_executor_node": {"ros__parameters": {
                "pen_stand_approach_pose": [43.61,-1.88,99.63,.09,82.26,43.29],
                "pen_stand_pose": [248.86,244.77,312.99,123.66,180.,123.43],
                "pen_gripper_pose": [248.91,244.76,548.06,37.07,-179.99,36.84],
            }}
        })
        reference = Mock()
        reference.read_text.return_value = json.dumps({
            "mode":"real", "pose":[399.,-54.,220.,0.,180.,0.],
        })
        self.args = SimpleNamespace(
            reference=reference, rack_config=rack, tilt_heading_deg=90.,
            jaw_center_offset=[0.,0.,220.], height_ratio=.5,
            gap_mm=1., sweep_deg=360., direction="cw",
        )

    def test_circle_and_pen_axis(self):
        """모든 점에서 그리퍼는 수직이고 실제 펜은 중심 쪽 아래 45도를 향한다."""
        _, samples, _, pen_local, summary = make_path(self.args,[250.,245.,400.,0.,180.,0.])
        cx,cy,_,z,radius = summary
        self.assertEqual(len(samples),361)
        np.testing.assert_allclose(samples[0][:3],samples[-1][:3],atol=1e-9)
        for point in samples:
            outward = np.array([point[0]-cx,point[1]-cy,0.])/radius
            self.assertAlmostEqual(np.linalg.norm(outward),1.)
            self.assertAlmostEqual(point[2],z)
            rotation = Rotation.from_euler("ZYZ",point[3:],degrees=True)
            np.testing.assert_allclose(rotation.apply([0.,0.,1.]),[0.,0.,-1.],atol=1e-10)
            np.testing.assert_allclose(
                rotation.apply(pen_local),
                -np.sqrt(.5)*(outward+np.array([0.,0.,1.])),
                atol=1e-10,
            )

    def test_approach_and_exit_outside(self):
        """현재 위치에서 먼저 위로 이탈하고 원주 바깥에서 접근·후퇴한다."""
        initial=[250.,245.,400.,0.,180.,0.]
        entry,_,exit_path,_,summary=make_path(self.args,initial)
        cx,cy,_,_,radius=summary
        self.assertEqual(entry[0][:2],initial[:2])
        self.assertGreater(entry[0][2],initial[2])
        self.assertEqual(entry[1][3:],initial[3:])
        for point in (entry[2],entry[3],*exit_path):
            self.assertGreater(np.linalg.norm(np.array(point[:2])-[cx,cy]),radius+39.)

    def test_reverse_direction_preserves_start_and_surface(self):
        """같은 시작점에서 반대 순서로 돌며 같은 위치의 펜 자세도 유지한다."""
        initial = [250., 245., 400., 0., 180., 0.]
        _, clockwise, _, _, summary = make_path(self.args, initial)
        self.args.direction = "ccw"
        _, counterclockwise, _, _, _ = make_path(self.args, initial)
        np.testing.assert_allclose(clockwise[0][:3], counterclockwise[0][:3])
        cx, cy = summary[:2]
        start = np.array(clockwise[0][:2]) - [cx, cy]
        next_point = np.array(clockwise[1][:2]) - [cx, cy]
        self.assertLess(start[0] * next_point[1] - start[1] * next_point[0], 0)
        for first, second in zip(clockwise, reversed(counterclockwise)):
            np.testing.assert_allclose(first[:3], second[:3], atol=1e-9)
            rotation = Rotation.from_euler("ZYZ", first[3:], degrees=True).inv()
            rotation *= Rotation.from_euler("ZYZ", second[3:], degrees=True)
            self.assertLess(rotation.magnitude(), 1e-10)

    def test_j6_crosses_positive_180_continuously(self):
        """실물 로그의 179.86 → -179.13을 약 1도 회전으로 계산한다."""
        self.assertAlmostEqual(continuous_j6(-179.13, 179.86), 180.87)

    def test_j6_crosses_negative_180_continuously(self):
        """반대 진행에서도 음수 회전수를 유지한다."""
        self.assertAlmostEqual(continuous_j6(179.13, -179.86), -180.87)

    def test_j6_does_not_hide_real_360_limit(self):
        """연속 경로가 사양 한계를 넘으면 0도 근처 값으로 감추지 않는다."""
        self.assertGreater(continuous_j6(1.0, 359.0), 360.0)
        self.assertLess(continuous_j6(-1.0, -359.0), -360.0)


if __name__=="__main__":
    unittest.main()


def make_contact_session():
    """펜 거치대 위 현재 위치와 실제 이동을 반영하는 로봇 대역을 만든다."""
    from cake_decorator.bent_pen import SIDE_TCP_OFFSET_MM

    current = [247.948, 198.288, 276.361, 0., 180., 0.]
    saved_center = [399.274, -54.559, 220., 123.66, 180., 123.43]
    state = {"pose": list(current)}
    targets = []

    def call(kind, service, request, **kwargs):
        """위치가 바뀌어도 등록된 TCP 교정값은 같은 플랜지 응답을 돌려준다."""
        if service.endswith("get_robot_mode"):
            return SimpleNamespace(robot_mode=1)
        if service.endswith("get_current_tcp"):
            return SimpleNamespace(info="Pen_Side_45")
        if service.endswith("get_current_tool_flange_posx"):
            pose = state["pose"]
            rotation = Rotation.from_euler("ZYZ", pose[3:], degrees=True)
            xyz = np.array(pose[:3]) - rotation.apply(SIDE_TCP_OFFSET_MM)
            return SimpleNamespace(pos=[*xyz, *pose[3:]])
        return SimpleNamespace(success=True)

    def move(pos, **kwargs):
        """물리 로봇 없이 명령 목표를 기록하고 관측 위치를 갱신한다."""
        target = pos.to_list()
        targets.append(target)
        state["pose"] = target
        return True

    robot = Mock()
    robot.move_line.side_effect = move
    robot.verify_pose.return_value = True
    session = SimpleNamespace(
        node=Mock(),
        call=call,
        pose=lambda: list(state["pose"]),
    )
    return session, robot, state, targets, current, saved_center


def test_web_reference_visits_saved_center_before_yes():
    """거치대 위에서 시작해도 YES 전에 케이크 중앙에 도착하며 거치대를 중심으로 쓰지 않는다."""
    from system_launcher import prepare_side_pen_reference

    session, robot, state, targets, current, saved_center = make_contact_session()

    def confirm(_prompt):
        """운영자에게 확인을 물을 때 펜촉은 케이크 중앙에 있어야 한다."""
        assert state["pose"][:3] == saved_center[:3]
        assert targets[0][:2] == current[:2]
        assert targets[0][2] > current[2]
        assert targets[1][:2] == saved_center[:2]
        assert targets[1][2] >= saved_center[2] + 70.
        assert len(targets) == 3
        return "YES"

    with patch("cake_decorator.doosan_robot.DoosanRobotClient", return_value=robot):
        reference, start = prepare_side_pen_reference(session, saved_center, answer=confirm)

    assert reference == saved_center
    assert start[:3] == [*saved_center[:2], saved_center[2] + 50.]
    assert len(targets) == 4
    # 긴 공중 이동은 빠르게, 중앙 접촉과 접촉 후 이탈은 구분해서 실행한다.
    assert [
        call.kwargs["speed_mm_s"] for call in robot.move_line.call_args_list
    ] == [100.0, 100.0, 30.0, 60.0]
    # 중앙 접촉과 이탈 중에는 자세를 회전하지 않는다.
    for target in targets:
        assert target[3:] == current[3:]


def test_web_reference_accepts_pendant_correction_at_center():
    """자동으로 중앙에 온 뒤 펜던트로 보정한 위치를 YES로 기록한다."""
    from system_launcher import prepare_side_pen_reference

    session, robot, state, targets, _, saved_center = make_contact_session()
    corrected = [400., -56., 221.]

    def confirm(_prompt):
        state["pose"][:3] = corrected
        return "YES"

    with patch("cake_decorator.doosan_robot.DoosanRobotClient", return_value=robot):
        reference, start = prepare_side_pen_reference(session, saved_center, answer=confirm)
    assert reference[:3] == corrected
    assert start[:3] == [400., -56., 271.]


def test_web_reference_cancel_stops_at_center():
    """중앙 확인을 취소하면 이탈이나 그리기를 시작하지 않는다."""
    from system_launcher import prepare_side_pen_reference

    session, robot, _, targets, _, saved_center = make_contact_session()
    with patch("cake_decorator.doosan_robot.DoosanRobotClient", return_value=robot):
        with np.testing.assert_raises_regex(RuntimeError, "취소"):
            prepare_side_pen_reference(session, saved_center, answer=lambda _: "NO")
    assert len(targets) == 3
    assert targets[-1][:3] == saved_center[:3]


def test_web_reference_failed_approach_never_asks_yes():
    """중앙 도착에 실패하면 운영자 확인이나 다음 이동으로 진행하지 않는다."""
    from system_launcher import prepare_side_pen_reference

    session, robot, _, _, _, saved_center = make_contact_session()
    robot.verify_pose.return_value = False
    confirm = Mock(return_value="YES")
    with patch("cake_decorator.doosan_robot.DoosanRobotClient", return_value=robot):
        with np.testing.assert_raises_regex(RuntimeError, "실패"):
            prepare_side_pen_reference(session, saved_center, answer=confirm)
    assert robot.move_line.call_count == 1
    confirm.assert_not_called()


def test_web_reference_failed_lift_stops_preparation():
    """YES 뒤 이탈에 실패하면 웹 준비를 완료하지 않는다."""
    from system_launcher import prepare_side_pen_reference

    session, robot, _, _, _, saved_center = make_contact_session()
    robot.verify_pose.side_effect = [True, True, True, False]
    with patch("cake_decorator.doosan_robot.DoosanRobotClient", return_value=robot):
        with np.testing.assert_raises_regex(RuntimeError, "실패"):
            prepare_side_pen_reference(session, saved_center, answer=lambda _: "YES")
    assert robot.move_line.call_count == 4


def test_web_reference_can_continue_without_any_yes_input():
    """자동 진행에서도 중앙 접촉·기록·이탈은 유지하고 입력 함수는 호출하지 않는다."""
    from system_launcher import prepare_side_pen_reference

    session, robot, state, targets, current, saved_center = make_contact_session()
    answer = Mock(side_effect=AssertionError("자동 진행 중 입력을 요청했습니다."))
    with patch("cake_decorator.doosan_robot.DoosanRobotClient", return_value=robot):
        reference, start = prepare_side_pen_reference(
            session, saved_center, answer=answer, skip_confirm=True,
        )
    answer.assert_not_called()
    assert reference == saved_center
    assert targets[2][:3] == saved_center[:3]
    assert start[:3] == [*saved_center[:2], saved_center[2] + 50.]
