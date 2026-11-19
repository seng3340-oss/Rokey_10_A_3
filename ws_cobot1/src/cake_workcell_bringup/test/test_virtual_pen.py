"""가상 전용 모드 제한과 입력 길이·표시 모델·제어 TCP의 일치를 검사한다."""

import runpy
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

HELPERS = runpy.run_path(str(Path(__file__).parents[1] / "scripts/virtual_pen.py"))


def robot_description(mode="virtual", host="127.0.0.1"):
    """모드 보호 검증용 최소 URDF다. 제조사 운동학을 대체하는 모델은 아니다."""
    return f"""<robot name="test"><ros2_control><hardware>
    <param name="mode">{mode}</param><param name="host">{host}</param>
    </hardware></ros2_control></robot>"""


class VirtualPenTests(unittest.TestCase):
    def test_four_cm_is_added_to_gripper_tip(self):
        """4cm는 플랜지에서의 총 길이가 아닌 손가락 끝부터의 추가 길이다."""
        extension = HELPERS["parse_extension_cm"]("4")
        description = HELPERS["add_pen"](robot_description(), extension)
        pose = HELPERS["tcp_from_description"](description)
        angle, gripper_tip = HELPERS["grip_geometry"]()
        self.assertAlmostEqual(pose[2] / 1000 - gripper_tip, 0.04)
        self.assertGreater(angle, 0)
        root = ET.fromstring(description)
        self.assertIsNotNone(root.find("link[@name='virtual_pen_tip']/visual"))

    def test_input_length_changes_both_tcp_and_model(self):
        """입력한 길이 차이가 모델과 TCP의 위치 차이에 동일하게 반영된다."""
        first = HELPERS["add_pen"](robot_description(), 0.04)
        second = HELPERS["add_pen"](robot_description(), 0.07)
        self.assertAlmostEqual(
            HELPERS["tcp_from_description"](second)[2]
            - HELPERS["tcp_from_description"](first)[2],
            30,
        )

    def test_real_target_is_rejected_before_tcp_configuration(self):
        """실물 모드나 외부 호스트에는 가상 펜 TCP를 등록하지 않는다."""
        for mode, host in [
            ("real", "192.168.1.100"),
            ("real", "127.0.0.1"),
            ("virtual", "192.168.1.100"),
        ]:
            with self.subTest(mode=mode, host=host), self.assertRaises(RuntimeError):
                HELPERS["add_pen"](robot_description(mode, host), 0.04)

    def test_only_known_rounded_virtual_joint_origins_are_aligned(self):
        """FK 비교로 확인한 가상 모델만 고치며 다른 치수·벡터는 보존한다."""
        import math

        source = robot_description().replace(
            "</robot>",
            """
        <joint name="joint_1"><origin xyz="0 0 .1345" rpy="0 0 0"/></joint>
        <joint name="joint_2"><origin xyz=".01 .0062 0" rpy="-1.571 0 0"/></joint>
        <joint name="joint_3"><origin xyz="0 0 .411" rpy=".5 0 0"/></joint>
        </robot>""",
        )
        changed = ET.fromstring(HELPERS["add_pen"](source, 0.04))
        self.assertEqual(
            float(changed.find("joint[@name='joint_1']/origin").get("xyz").split()[2]),
            0.135,
        )
        origin = changed.find("joint[@name='joint_2']/origin")
        self.assertEqual(float(origin.get("xyz").split()[1]), 0.00625)
        self.assertEqual(float(origin.get("rpy").split()[0]), -math.pi / 2)
        self.assertEqual(
            float(changed.find("joint[@name='joint_3']/origin").get("rpy").split()[0]),
            0.5,
        )

    def test_invalid_extension_is_rejected(self):
        """유효하지 않은 길이로 모델이나 TCP를 만들지 않는다."""
        for value in ("0", "-1", "nan", "inf", "abc", "21"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                HELPERS["parse_extension_cm"](value)


if __name__ == "__main__":
    unittest.main()


def test_side_pen_visual_tip_matches_calibrated_tcp():
    """45도 펜의 표시 끝점은 실물 자동 교정값과 같고 몸통은 45도로 기울어진다."""
    import math

    text = HELPERS["add_pen"](robot_description(), .04, side_pen_45=True)
    pose = HELPERS["tcp_from_description"](text)
    for actual, expected in zip(pose, [1.095, -46.474, 271.695, 0., 0., 0.]):
        assert math.isclose(actual, expected, abs_tol=1e-8)
    root = ET.fromstring(text)
    visual = root.find("link[@name='virtual_pen_tip']/visual/origin")
    rpy = list(map(float, visual.get("rpy").split()))
    assert math.isclose(rpy[0], math.pi / 4, abs_tol=1e-8)


def test_side_workcell_cake_top_uses_execution_center():
    """화면 케이크의 상면 중심과 실제 실행에 쓰는 중심이 일치한다."""
    source = '''<robot name="scene">
    <joint name="station_mount"><origin xyz=".399 -.049 0"/></joint>
    <link name="cake_tray_surface"><visual name="drawing_cake">
      <origin xyz="0 0 0"/><geometry><mesh filename="cake.stl"/></geometry>
    </visual></link></robot>'''
    changed = ET.fromstring(HELPERS["align_side_workcell"](
        source, [.399274, -.054559, .220], .079, .002,
    ))
    mount = list(map(float, changed.find("joint/origin").get("xyz").split()))
    cake = changed.find("link/visual")
    origin = list(map(float, cake.find("origin").get("xyz").split()))
    scale = list(map(float, cake.find("geometry/mesh").get("scale").split()))
    assert mount[:2] == [.399274, -.054559]
    assert abs(mount[2] + .139 + origin[2] + .080 * scale[2] - .220) < 1e-10
