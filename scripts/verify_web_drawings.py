"""서비스가 받은 실제 펜촉 궤적을 주문 원본과 비교하고 충돌과 재생 파일을 확인한다."""
from collections import Counter
import csv
import json
import math
from pathlib import Path
import shlex
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from simulate_cake_lettering import JOINTS, build_models, collision_report
from virtual_draw_demo import LETTER_R

root = Path(__file__).resolve().parents[1]
run = Path(sys.argv[2]).resolve()
output = Path(sys.argv[1])
output.mkdir(parents=True, exist_ok=True)
records = [json.loads(line) for line in (run / "move-line.jsonl").read_text().splitlines()]
robot, scene, packages = build_models(root)
flags = subprocess.check_output(["pkg-config", "--cflags", "--libs", "fcl", "assimp"], text=True)
checker = run / "mesh-check"
subprocess.run([
    "c++", "-O2", "-std=c++17", str(root / "scripts/cake_mesh_check.cpp"),
    *(part for part in shlex.split(flags) if not part.startswith("-std=")),
    "-o", str(checker),
], check=True)
start = 0
summaries = {}
fig, charts = plt.subplots(1, 3, figsize=(10.4, 3.7), layout="constrained")
for name, label, n_strokes, axis in zip(
    ("heart", "r", "love"), ("Web order", "R demo", "Lettering order"),
    (1, 3, 13), charts,
):
    end = int((run / f"{name}-end").read_text())
    commands = records[start:end]
    start = end
    phases = Counter(command["phase"] for command in commands)
    assert phases["PEN_DOWN"] == phases["PEN_UP"] == n_strokes
    assert commands[-1]["phase"] == "PEN_UP"
    contact = [command["command"][:3] for command in commands
               if command["phase"] in ("PEN_DOWN", "DRAWING")]
    order = None
    if name == "r":
        # Point32의 float32 직렬화 반올림을 원본 픽셀 단계에서 적용한다.
        expected = [
            (
                500 + (float(np.float32(512 + x * 6.4)) - 512) * 100 / 640,
                -(float(np.float32(384 - y * 6.4)) - 384) * 100 / 640,
                142.0,
            )
            for stroke in LETTER_R for x, y in stroke
        ]
    else:
        order = json.loads((run / f"artifacts/{name}-order.json").read_text())
        drawing = order["drawing"]
        expected = [
            (500 + (float(np.float32(point["x"])) - 512) * 100 / 640, -(float(np.float32(point["y"])) - 384) * 100 / 640, 142.0)
            for stroke in drawing["strokes"] for point in stroke["points"]
        ]
    np.testing.assert_allclose(contact, expected, atol=1e-7, rtol=0)
    timestamps, joint_positions, rows = [], [], []
    time_offset = 0.0
    for block in commands:
        positions = block["rows"]
        joints = block["joints"]
        if rows:
            np.testing.assert_allclose(joints[0], joint_positions[-1], atol=1e-7, rtol=0)
        for row, joint in zip(positions[bool(rows):], joints[bool(rows):], strict=True):
            time = time_offset + row[0]
            timestamps.append(time)
            joint_positions.append(joint)
            rows.append((time, *row[1:]))
        time_offset += positions[-1][0]
    assert all(0 < later - earlier <= 0.04000001 for earlier, later in zip(timestamps, timestamps[1:]))
    assert all(math.dist(a[2:5], b[2:5]) <= 0.00100001 for a, b in zip(rows, rows[1:]))
    assert all(abs(row[4] - 0.142) < 1e-8 for row in rows if row[1] == "DRAWING")
    pairs, hits, fixed = collision_report(robot, scene, packages, joint_positions, checker)
    assert not hits, (name, hits[:5])
    scene_folder = run / "artifacts" / name
    scene_folder.mkdir(exist_ok=True)
    path = output / f"{name}-trajectory.csv"
    header = ["time_s", "phase", "tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_speed_mm_s"]
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([*header, *(f"{joint}_rad" for joint in JOINTS)])
        writer.writerows(
            [f"{row[0]:.8f}", row[1], *[f"{v:.8f}" for v in (*row[2:], *joint)]]
            for row, joint in zip(rows, joint_positions, strict=True)
        )
    replay_csv = scene_folder / "lettering_trajectory.csv"
    replay_csv.unlink(missing_ok=True)
    replay_csv.symlink_to(path)
    clearance = 50 - max(math.hypot(x - 500, y) for x, y, z in contact)
    report = dict(
        order_id=order["order_id"] if order else None,
        order_status=order["status"] if order else None,
        strokes=n_strokes,
        move_line_calls=len(commands),
        duration_s=timestamps[-1],
        samples=len(rows),
        minimum_rim_clearance_mm=clearance,
        max_ik_error_mm=max(block["ik_error_mm"] for block in commands),
        collision_pairs_per_sample=pairs,
        moving_collision_hits=hits,
        static_base_overlaps=[dict(links=pair, samples=count) for pair, count in sorted(fixed.items())],
    )
    summaries[name] = report
    (scene_folder / "lettering_validation.json").write_text(json.dumps(report, indent=2))
    print(name, n_strokes, len(commands), len(rows), round(timestamps[-1], 2), round(clearance, 2), flush=True)
    axis.add_patch(plt.Circle((0, 0), 50, facecolor="#f8e8cb", edgecolor="#ad7a4c"))
    for p, q in zip(rows, rows[1:]):
        if q[1] == "DRAWING":
            axis.plot([(p[2] - 0.5) * 1000, (q[2] - 0.5) * 1000],
                      [p[3] * 1000, q[3] * 1000], color="#a12f50", linewidth=2.5,
                      solid_capstyle="round")
    axis.set(xlim=(-53, 53), ylim=(-53, 53), aspect="equal", title=label)
    axis.set_axis_off()
assert start == len(records)
(output / "validation.json").write_text(json.dumps(summaries, indent=2) + "\n")
fig.savefig(output / "top-drawings.png", dpi=160)
