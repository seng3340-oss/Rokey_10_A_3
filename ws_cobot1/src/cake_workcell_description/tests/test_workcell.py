"""가상 배치의 단위·TF 구조·이식 가능한 메시 경로와 원본 보존을 확인한다."""

import hashlib
import json
import math
import struct
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
        robot = self.model(
            "layout:=legacy",
            "fixture_x:=0.6",
            "fixture_y:=0.1",
            "fixture_yaw:=1.2",
        )
        self.assertIsNone(robot.find("link[@name='station_base']"))
        mount = robot.find("joint[@name='fixture_a_mount']/origin")
        self.assertEqual(mount.get("xyz"), "0.6 0.1 0.0")
        self.assertEqual(mount.get("rpy"), "0 0 1.2")

    def test_meshes_are_portable_and_have_correct_units(self):
        """각 배치에서 원본이 mm인 메시만 0.001로 축소한다."""
        for layout in ("station", "legacy"):
            with self.subTest(layout=layout):
                for mesh in self.model(f"layout:={layout}").findall(".//mesh"):
                    filename = mesh.get("filename")
                    self.assertTrue(
                        filename.startswith("package://cake_workcell_description/meshes/")
                    )
                    path = ROOT / "meshes" / filename.rsplit("/", 1)[1]
                    self.assertTrue(path.exists())
                    in_millimeters = path.name.startswith("A_") or path.name == "cake_opener.stl"
                    self.assertEqual(
                        mesh.get("scale", "1 1 1"),
                        "0.001 0.001 0.001" if in_millimeters else "1 1 1",
                    )

    def test_default_layout_uses_approved_station_and_shelf(self):
        """기본 배치의 기준 프레임, 틀 회전과 가이드 2개를 검사한다."""
        robot = self.model()
        self.assertIsNone(robot.find("link[@name='fixture_a_base_link']"))
        mounts = {
            "station_mount": (0.399, -0.049, 0.0),
            "station_to_tray": (0.0, 0.0, 0.139),
            "cake_pen_holder_mount": (0.275, 0.225, 0.0),
            "cake_opener_preview_mount": (0.425, 0.225, 0.0),
            "cake_shared_shelf_mount": (0.755, 0.05, 0.0),
        }
        for name, expected in mounts.items():
            with self.subTest(joint=name):
                joint = robot.find(f"joint[@name='{name}']/origin")
                self.assertEqual(tuple(map(float, joint.get("xyz").split())), expected)

        shelf = robot.find("link[@name='cake_shared_shelf']")
        self.assertEqual(shelf.find("visual/geometry/box").get("size"), "0.15 0.55 0.15")
        guides = [
            visual for visual in shelf.findall("visual")
            if visual.find("geometry/mesh") is not None
        ]
        self.assertEqual(
            [g.find("origin").get("xyz") for g in guides],
            ["0 0.15 0.15", "0 -0.15 0.15"],
        )
        for tray, y in (("cake_input_tray", 0.15), ("cake_output_tray", -0.15)):
            with self.subTest(tray=tray):
                origin = robot.find(f"joint[@name='{tray}_mount']/origin")
                self.assertEqual(tuple(map(float, origin.get("xyz").split())), (0.0, y, 0.15))
                self.assertAlmostEqual(float(origin.get("rpy").split()[2]), math.pi / 4)
                box = robot.find(f"link[@name='{tray}']/visual/geometry/box")
                self.assertEqual(box.get("size"), "0.15 0.15 0.003")

    def test_station_opening_and_base_contacts(self):
        """실제 메시에서 아랫판 접합과 40mm 개구부, 10mm 돌출을 확인한다."""
        def bounds(stem):
            # 바이너리 STL 좌표는 m다. 좌표 극값을 mm로 변환해 도면과 비교한다.
            mesh = (ROOT / "meshes" / f"{stem}.stl").read_bytes()
            vertices = [
                facet[i : i + 3]
                for facet in struct.iter_unpack("<12fH", mesh[84:])
                for i in (3, 6, 9)
            ]
            return [
                (round(min(axis) * 1000, 4), round(max(axis) * 1000, 4))
                for axis in zip(*vertices)
            ]

        base = bounds("bottom_plate")
        left = bounds("wall_x_minus")
        right = bounds("wall_x_plus")
        bottom = bounds("wall_y_minus")
        top = bounds("wall_y_plus")
        self.assertEqual((left[0][1], right[0][0]), base[0])
        self.assertEqual((bottom[1][1], top[1][0]), base[1])
        self.assertEqual(right[0][0] - top[0][1], 40)
        self.assertEqual(top[1][0] - right[1][1], 40)
        self.assertEqual(bottom[1][0] - right[1][0], 10)

    def test_copied_assets_match_recorded_originals(self):
        """이관한 형상이 실수로 덮어써지지 않았는지 체크섬으로 확인한다."""
        checksums = json.loads((ROOT / "mesh_checksums.json").read_text())
        for name, checksum in checksums.items():
            self.assertEqual(
                hashlib.sha256((ROOT / "meshes" / name).read_bytes()).hexdigest(),
                checksum,
            )


class MeasuredCakeTests(unittest.TestCase):
    """원본 자산 보존 검사와 별도로 실측 원뿔대·판의 높이 관계를 확인한다."""

    def test_frustum_mesh_and_board_match_shared_dimensions(self):
        """실측 상·하단 반지름이 서로 다르고 케이크 밑면이 판 위에 놓여야 한다."""
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
                ["xacro", str(ROOT / "urdf/workcell.urdf.xacro"), "layout:=legacy"], text=True
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


if __name__ == "__main__":
    unittest.main()
