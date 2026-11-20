"""실물 연결 없이 펜 기울임 속도와 기존 이동·좌표 계약을 검증한다."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_integrated_pen_cycle import ServiceRobot, rack_parameters


class MotionRobot(ServiceRobot):
    """기존 TCP 서비스 대역을 유지하면서 실제 이동 호출의 속도를 기록한다."""

    def __init__(self):
        """로봇 통신 없이 선형·관절 목표와 속도 기록만 준비한다."""
        super().__init__()
        self.lines = []
        self.joints = []

    def move_line(self, pose, **kwargs):
        """TCP 좌표 역산은 부모에 맡기고 전달된 운동 제한을 보관한다."""
        self.lines.append((self.tcp, pose.to_list(), kwargs))
        return super().move_line(pose, **kwargs)

    def move_joint(self, joints, **kwargs):
        """되감기 목표와 속도를 복사해 후속 리스트 변경의 영향을 막는다."""
        self.joints.append((list(joints), kwargs))
        return super().move_joint(joints, **kwargs)


@pytest.mark.parametrize("tilt_speed, tilt_acc", [(45.0, 90.0), (30.0, 60.0)])
def test_pick_slows_only_tilt_and_upright_with_fixed_pivot(tilt_speed, tilt_acc):
    """기울임 두 구간만 감속하고 인출 속도·220mm 중심·티칭을 보존한다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle

    rack = rack_parameters()
    rack.update(pen_tilt_speed_deg_s=tilt_speed, pen_tilt_acc_deg_s2=tilt_acc)
    robot = MotionRobot()
    IntegratedPenCycle(robot, rack.__getitem__).pick_side()

    assert len(robot.lines) == 4
    assert [motion[2] for motion in robot.lines] == [
        {
            "speed_mm_s": speed,
            "acc_mm_s2": 400.0,
            "angular_speed_deg_s": angular_speed,
            "angular_acc_deg_s2": angular_acc,
        }
        for speed, angular_speed, angular_acc in [
            (120.0, 90.0, 180.0),
            (150.0, tilt_speed, tilt_acc),
            (150.0, 90.0, 180.0),
            (150.0, tilt_speed, tilt_acc),
        ]
    ]
    assert robot.joints == [(
        rack["pen_stand_approach_pose"],
        {"speed_deg_s": 60.0, "acc_deg_s2": 120.0},
    )]

    # 각 선형 목표는 파지 중심 TCP다. 기울이기 전후 중심이 같아야 한다.
    poses = [target for _, target, _ in robot.lines]
    np.testing.assert_allclose(poses[0][:3], poses[1][:3], atol=1e-9)
    np.testing.assert_allclose(poses[2][:3], poses[3][:3], atol=1e-9)
    clearance = rack["pen_gripper_pose"][2] - rack["pen_stand_pose"][2]
    np.testing.assert_allclose(
        np.array(poses[2][:3]) - poses[1][:3], [0.0, 0.0, clearance], atol=1e-9,
    )
    for index in (0, 3):
        rotation = Rotation.from_euler("ZYZ", poses[index][3:], degrees=True)
        flange = np.array(poses[index][:3]) - rotation.apply([0.0, 0.0, 220.0])
        expected = np.array(rack["pen_stand_pose"][:3])
        if index == 3:
            expected[2] += clearance
        np.testing.assert_allclose(flange, expected, atol=1e-9)
    for index in (1, 2):
        rotation = Rotation.from_euler("ZYZ", poses[index][3:], degrees=True)
        np.testing.assert_allclose(
            rotation.apply([0.0, 0.0, 1.0]),
            [0.0, np.sqrt(0.5), -np.sqrt(0.5)], atol=1e-9,
        )


@pytest.mark.parametrize("standalone", [False, True])
def test_return_slows_orientation_but_keeps_free_unwind_and_coordinates(
    monkeypatch, standalone,
):
    """통합·독립 반납 모두 기울임만 감속하고 J6·이탈·반납 중심을 유지한다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
    import cake_decorator.side_pen_return as returning

    monkeypatch.setattr(returning.time, "sleep", lambda _: None)
    rack = rack_parameters()
    rack.update(
        pen_tilt_speed_deg_s=45.0,
        pen_tilt_acc_deg_s2=90.0,
        pen_drop_pose=[280.0, 210.0, 330.0, 40.25, 180.0, 40.02],
        pen_drop_after_pose=[280.0, 210.0, 570.0, 37.07, -179.99, 36.84],
        pen_drop_approach_pose=[30.0, 1.0, 60.0, 0.0, 110.0, 42.0],
    )
    robot = MotionRobot()
    robot.tcp = "Pen_Side_45"
    robot.joint_values[5] = 402.0
    initial_pose = robot.current_pose()
    initial_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 42.0]
    if standalone:
        returning.return_pen(robot, initial_joints, rack=rack)
    else:
        IntegratedPenCycle(robot, rack.__getitem__).return_side(initial_joints)

    assert len(robot.joints) == 3
    assert [target for target, _ in robot.joints] == [
        [35.0, 40.0, 35.0, -20.0, 120.0, 402.0],
        [35.0, 40.0, 35.0, -20.0, 120.0, 42.0],
        rack["pen_drop_approach_pose"],
    ]
    assert [settings for _, settings in robot.joints] == [
        {"speed_deg_s": 60.0, "acc_deg_s2": 120.0},
        {"speed_deg_s": 60.0, "acc_deg_s2": 120.0},
        {"speed_deg_s": 45.0, "acc_deg_s2": 90.0},
    ]
    assert len(robot.lines) == 4
    assert [settings for _, _, settings in robot.lines] == [
        {
            "speed_mm_s": speed,
            "acc_mm_s2": 400.0,
            "angular_speed_deg_s": 45.0,
            "angular_acc_deg_s2": 90.0,
        }
        for speed in (150.0, 180.0, 120.0, 150.0)
    ]

    raised = list(initial_pose)
    raised[2] += 100.0
    np.testing.assert_allclose(robot.lines[0][1], raised, atol=1e-9)
    above, drop, after = robot.lines[1:]
    np.testing.assert_allclose(
        np.array(above[1][:3]) - drop[1][:3], [0.0, 0.0, 240.0], atol=1e-9,
    )
    np.testing.assert_allclose(above[1][3:], drop[1][3:], atol=1e-9)

    # 펜촉 목표를 플랜지로 역산한 뒤 220mm 파지 중심을 티칭과 비교한다.
    rotation = Rotation.from_euler("ZYZ", drop[1][3:], degrees=True)
    flange = np.array(drop[1][:3]) - rotation.apply([1.095, -46.474, 271.695])
    center = flange + rotation.apply([0.0, 0.0, 220.0])
    taught = rack["pen_drop_pose"]
    expected = np.array(taught[:3]) + Rotation.from_euler(
        "ZYZ", taught[3:], degrees=True,
    ).apply([0.0, 0.0, 220.0])
    np.testing.assert_allclose(center, expected, atol=1e-9)
    np.testing.assert_allclose(
        rotation.apply([0.0, 0.0, 1.0]),
        [0.0, np.sqrt(0.5), -np.sqrt(0.5)], atol=1e-9,
    )
    expected_after = np.array(above[1])
    if not standalone:
        expected_after[:3] = flange + [0.0, 0.0, 240.0]
        assert ("open", "Pen_Side_45") in robot.events
    assert after[0] == ("Pen_Side_45" if standalone else "")
    np.testing.assert_allclose(after[1], expected_after, atol=1e-9)
