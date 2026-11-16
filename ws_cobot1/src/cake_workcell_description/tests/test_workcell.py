"""가상 배치의 단위·TF 구조·이식 가능한 메시 경로와 원본 보존을 확인한다."""

import hashlib
import json
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class WorkcellTests(unittest.TestCase):
    """ROS 노드나 Docker를 띄우지 않고 렌더링 입력 모델을 검사한다."""

    def model(self, *arguments):
        """설치된 xacro로 치환한 결과를 읽는다. 인자는 m/rad 단위다."""
        result = subprocess.check_output(
            [
                "xacro",
                str(ROOT / "urdf/workcell.urdf.xacro"),
                *arguments,
            ],
            text=True,
        )
        return ET.fromstring(result)

    def test_table_dimensions_and_no_robot_frame_collision(self):
        """독립 환경 루트·16개 구멍과 바닥 높이를 유지하는지 확인한다."""
        robot = self.model()
        names = [link.get("name") for link in robot.findall("link")]
        self.assertEqual(len(names), len(set(names)))
        self.assertNotIn("base_link", names)
        self.assertNotIn("Work_Cylinder", names)
        self.assertEqual(sum(name.startswith("Mounting_Insert_") for name in names), 16)
        floor = robot.find("joint[@name='Floor_fixed']/origin")
        self.assertEqual(float(floor.get("xyz").split()[2]), -0.97)
        # floor 시각 형상 두께 20mm의 윗면이 -960mm에 놓인다.
        self.assertAlmostEqual(float(floor.get("xyz").split()[2]) + 0.01, -0.96)

    def test_fixture_mount_is_editable(self):
        """프레임 메시를 수정하지 않고 장착 위치와 방향만 조정할 수 있어야 한다."""
        robot = self.model("fixture_x:=0.6", "fixture_y:=0.1", "fixture_yaw:=1.2")
        mount = robot.find("joint[@name='fixture_a_mount']/origin")
        self.assertEqual(mount.get("xyz"), "0.6 0.1 0.0")
        self.assertEqual(mount.get("rpy"), "0 0 1.2")

    def test_meshes_are_portable_and_have_correct_units(self):
        """mm인 A안만 축소하고 테이블 자산은 원래 m 단위를 유지한다."""
        for mesh in self.model().findall(".//mesh"):
            filename = mesh.get("filename")
            self.assertTrue(
                filename.startswith("package://cake_workcell_description/meshes/")
            )
            path = ROOT / "meshes" / filename.rsplit("/", 1)[1]
            self.assertTrue(path.exists())
            if path.name.startswith("A_"):
                self.assertEqual(mesh.get("scale"), "0.001 0.001 0.001")

    def test_copied_assets_match_recorded_originals(self):
        """이관한 형상이 실수로 덮어써지지 않았는지 체크섬으로 확인한다."""
        checksums = json.loads((ROOT / "mesh_checksums.json").read_text())
        for name, checksum in checksums.items():
            self.assertEqual(
                hashlib.sha256((ROOT / "meshes" / name).read_bytes()).hexdigest(),
                checksum,
            )


if __name__ == "__main__":
    unittest.main()


class MeasuredCakeTests(unittest.TestCase):
    """원본 자산 보존 검사와 별도로 실측 원뿔대·판의 높이 관계를 확인한다."""

    def test_frustum_mesh_and_board_match_shared_dimensions(self):
        """상면과 밑면 반지름이 서로 다르고 케이크 밑면이 판 위에 놓여야 한다."""
        config = json.loads((ROOT.parents[2] / "config/test_cake.json").read_text())
        vertices = [
            tuple(map(float, line.split()[1:]))
            for line in (ROOT / "meshes/test_cake.stl").read_text().splitlines()
            if line.startswith("vertex ")
        ]
        self.assertAlmostEqual(max(p[2] for p in vertices), config["height_mm"] / 1000)
        top = [p for p in vertices if abs(p[2] - config["height_mm"] / 1000) < 1e-9]
        bottom = [p for p in vertices if p[2] == 0]
        self.assertAlmostEqual(
            max((p[0] ** 2 + p[1] ** 2) ** 0.5 for p in top),
            config["top_diameter_mm"] / 2000,
        )
        self.assertAlmostEqual(
            max((p[0] ** 2 + p[1] ** 2) ** 0.5 for p in bottom),
            config["bottom_diameter_mm"] / 2000,
        )
        robot = ET.fromstring(
            subprocess.check_output(
                ["xacro", str(ROOT / "urdf/workcell.urdf.xacro")], text=True
            )
        )
        board = robot.find("link[@name='fixture_a_board_link']")
        self.assertEqual(
            list(map(float, board.find("visual/geometry/box").get("size").split())),
            [
                config["board_width_mm"] / 1000,
                config["board_depth_mm"] / 1000,
                config["board_thickness_mm"] / 1000,
            ],
        )
        board_z = float(board.find("visual/origin").get("xyz").split()[2])
        cake = robot.find("link[@name='fixture_a_cake_link']")
        cake_z = float(cake.find("visual/origin").get("xyz").split()[2])
        self.assertAlmostEqual(board_z + config["board_thickness_mm"] / 2000, cake_z)
        self.assertIsNotNone(cake.find("collision/geometry/mesh"))
