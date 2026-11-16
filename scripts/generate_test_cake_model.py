"""공통 원통형 설정으로 케이크 STL과 케이크판의 시각·충돌 형상을 갱신한다."""

import json
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "ws_cobot1/src/cake_workcell_description"


def triangle(a, b, c):
    """삼각형 외적을 정규화해 STL 면 법선을 계산한다."""
    u = [b[i] - a[i] for i in range(3)]
    v = [c[i] - a[i] for i in range(3)]
    normal = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ]
    length = math.sqrt(sum(value * value for value in normal))
    lines = [
        "facet normal " + " ".join(f"{value/length:.10g}" for value in normal),
        "outer loop",
    ]
    lines.extend(
        "vertex " + " ".join(f"{value:.10g}" for value in point) for point in (a, b, c)
    )
    return "\n".join(lines + ["endloop", "endfacet"])


def generate():
    """원본 A안 자산은 보존하고 새 실측 모델만 생성한다. 위치는 임시 시각화 배치다."""
    cake = json.loads((ROOT / "config/test_cake.json").read_text())
    top = bottom = cake["diameter_mm"] / 2000
    height = cake["height_mm"] / 1000
    width = cake["board_width_mm"] / 1000
    depth = cake["board_depth_mm"] / 1000
    thickness = cake["board_thickness_mm"] / 1000
    faces = []
    for i in range(96):
        a, b = 2 * math.pi * i / 96, 2 * math.pi * (i + 1) / 96
        lo = (bottom * math.cos(a), bottom * math.sin(a), 0.0)
        ln = (bottom * math.cos(b), bottom * math.sin(b), 0.0)
        hi = (top * math.cos(a), top * math.sin(a), height)
        hn = (top * math.cos(b), top * math.sin(b), height)
        faces.extend(
            [
                triangle(lo, ln, hn),
                triangle(lo, hn, hi),
                triangle((0.0, 0.0, height), hi, hn),
                triangle((0.0, 0.0, 0.0), ln, lo),
            ]
        )
    (MODEL / "meshes/test_cake.stl").write_text(
        "solid test_cake\n" + "\n".join(faces) + "\nendsolid test_cake\n"
    )
    path = MODEL / "urdf/workcell.urdf.xacro"
    text = path.read_text()

    # 이전 고정구 원점과 판 아래 Z=60mm는 임시 배치로 유지한다.
    # 실측 TCP 영점과 연결하지 않으며 원본 지그의 적합성도 보장하지 않는다.
    def body(kind):
        geometry = (
            f'<box size="{width:.6f} {depth:.6f} {thickness:.6f}" />'
            if kind == "board"
            else '<mesh filename="package://cake_workcell_description/meshes/test_cake.stl" />'
        )
        origin = (
            f"0.085 0.085 {0.06 + thickness/2:.6f}"
            if kind == "board"
            else f"0.085 0.085 {0.06 + thickness:.6f}"
        )
        material = "board_mat" if kind == "board" else "cake_mat"
        color = "0.97 0.96 0.93 1" if kind == "board" else "0.93 0.82 0.63 1"
        return f"""  <link name="fixture_a_{kind}_link">
    <visual>
      <origin xyz="{origin}" rpy="0 0 0" />
      <geometry>{geometry}</geometry>
      <material name="{material}"><color rgba="{color}" /></material>
    </visual>
    <collision>
      <origin xyz="{origin}" rpy="0 0 0" />
      <geometry>{geometry}</geometry>
    </collision>
  </link>"""

    for kind in ("board", "cake"):
        text = re.sub(
            rf'  <link name="fixture_a_{kind}_link">.*?</link>',
            body(kind),
            text,
            flags=re.S,
        )
    path.write_text(text)


if __name__ == "__main__":
    generate()
