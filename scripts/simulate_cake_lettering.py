#!/usr/bin/env python3
"""M0609·RG2 모델에서 ‘사랑해’의 펜촉 경로·관절 자세를 오프라인으로 검증한다."""

import argparse
from dataclasses import dataclass
import csv
import io
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


SOURCE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE_ROOT / "ws_cobot1/src/cake_decorator"))
from cake_decorator.drawing_geometry import (
    CylinderCake, pen_quat_top, pixel_to_mm_top,
)
from cake_decorator.drawing_motion import plan_drawing_motion


DRAW_SPEED = 20.0  # mm/s
TRAVEL_SPEED = 40.0  # mm/s
LINEAR_ACCEL = 100.0  # mm/s²
LIFT = 0.010  # m
PEN_TIP = 0.240  # 가상 펜촉만의 RG2 베이스 +Z 오프셋(m)
JOINTS = [f"joint_{i}" for i in range(1, 7)]
DOWN = Rotation.from_quat([1, 0, 0, 0])


def ellipse(cx, cy, rx, ry):
    """ㅇ 한 획을 동일한 각도 간격의 닫힌 폴리라인으로 만든다."""
    return [
        (cx + rx * math.cos(theta), cy + ry * math.sin(theta))
        for theta in np.linspace(-math.pi / 2, 3 * math.pi / 2, 25)
    ]


def letter_strokes(radius):
    """한글 중심선을 70×40mm 기준 상자에 맞춰 기존 정사각 캔버스로 보낸다."""
    # 문자마다 펜을 떼는 지점에서 획을 분리한다. 좌표 원점은 좌상단, 단위 mm다.
    shapes = [
        [(0, 28), (5, 12), (10, 28)],
        [(15, 5), (15, 35)],
        [(15, 18), (20, 18)],
        [(25, 4), (36, 4), (36, 10), (25, 10), (25, 16), (36, 16)],
        [(42, 0), (42, 20)],
        [(42, 9), (46, 9)],
        ellipse(35, 31, 9, 9),
        [(54, 6), (58, 6)],
        [(50, 12), (62, 12)],
        ellipse(56, 24, 5, 7),
        [(65, 5), (65, 35)],
        [(65, 19), (70, 19)],
        [(70, 5), (70, 35)],
    ]
    # 현재 원통형 변환식은 정사각 캔버스 전체를 윗면 지름에 매핑한다.
    canvas = 1000
    pixels_per_mm = canvas / (2 * radius * 1000)
    return canvas, [
        [
            (canvas / 2 + (x - 35) * pixels_per_mm,
             canvas / 2 + (y - 20) * pixels_per_mm)
            for x, y in shape
        ]
        for shape in shapes
    ]


@dataclass(frozen=True)
class Pose:
    """검증과 미리보기에 사용하는 위치(m)와 자세(쿼터니언)."""

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class Waypoint:
    """접촉 여부와 이동 속도를 담는 오프라인 목표."""

    pose: Pose
    pen_down: bool
    speed: float


@dataclass(frozen=True)
class Trajectory:
    """기존 미리보기와 검증기가 읽는 순서대로 목표를 저장한다."""

    waypoints: list[Waypoint]

    @property
    def pen_states(self):
        """동작 계획기에 목표마다 펜 접촉 여부를 전달한다."""
        return [waypoint.pen_down for waypoint in self.waypoints]



def origin(element):
    """URDF origin(m/rad)을 자식에서 부모로의 동차변환으로 바꾼다."""
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        angles = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        result[:3, :3] = Rotation.from_euler("xyz", angles).as_matrix()
    return result


class Model:
    """펼친 URDF의 링크 트리를 저장하고 BASE 기준 링크 자세를 계산한다."""

    def __init__(self, xml, root):
        self.xml = xml
        self.root = root
        self.joints = {}
        for joint in xml.findall("joint"):
            axis = joint.find("axis")
            name = joint.get("name")
            self.joints[name] = (
                joint.find("parent").get("link"),
                joint.find("child").get("link"),
                origin(joint.find("origin")),
                None if axis is None else np.fromstring(axis.get("xyz"), sep=" "),
            )

        # 반복 역기구학에서는 RG2 베이스까지의 체인만 계산한다.
        by_child = {data[1]: name for name, data in self.joints.items()}
        self.tcp_chain = []
        link = "rg2_base_link"
        while link in by_child and link != root:
            name = by_child[link]
            self.tcp_chain.insert(0, name)
            link = self.joints[name][0]

    def frames(self, angles):
        """고정된 RG2 관절 0rad와 팔 좌표로 모든 링크의 BASE 변환을 구한다."""
        frames = {self.root: np.eye(4)}
        pending = list(self.joints)
        while pending:
            ready = [name for name in pending if self.joints[name][0] in frames]
            if not ready:
                raise ValueError("BASE와 연결되지 않은 URDF 링크가 있습니다.")
            for name in ready:
                parent, child, _, _ = self.joints[name]
                frames[child] = frames[parent] @ self.joint_pose(name, angles)
                pending.remove(name)
        return frames

    def joint_pose(self, name, angles):
        """관절 원점 뒤에 URDF의 로컬 회전축을 적용한다."""
        _, _, frame, axis = self.joints[name]
        if name not in angles:
            return frame
        rotation = np.eye(4)
        rotation[:3, :3] = Rotation.from_rotvec(axis * angles[name]).as_matrix()
        return frame @ rotation

    def tcp(self, q):
        """가상 TCP의 위치와 자세를 RG2 베이스 프레임에서 계산한다."""
        angles = dict(zip(JOINTS, q))
        frame = np.eye(4)
        for name in self.tcp_chain:
            frame = frame @ self.joint_pose(name, angles)
        tip = frame[:3, 3] + PEN_TIP * frame[:3, 2]
        return tip, Rotation.from_matrix(frame[:3, :3])


@dataclass
class Shape:
    """충돌 메시·기본 도형과 링크 내 로컬 변환(m)을 담는다."""

    link: str
    pose: np.ndarray
    spec: str


def shapes(model, packages):
    """URDF 충돌 형상과 작업대의 시각 전용 고정구 메시를 검사기에 전달한다."""
    result = []
    fixture_meshes = {"fixture_a_frame_link", "fixture_a_pegboard_link"}
    for link in model.xml.findall("link"):
        name = link.get("name")
        entries = link.findall("visual" if name in fixture_meshes else "collision")
        for entry in entries:
            geometry = entry.find("geometry")
            child = geometry[0]
            if child.tag == "mesh":
                package, filename = child.get("filename").removeprefix("package://").split("/", 1)
                path = packages[package] / filename
                spec = f'mesh {child.get("scale", "1 1 1")} {json.dumps(str(path), ensure_ascii=False)}'
            elif child.tag == "box":
                spec = f'box {child.get("size")}'
            elif child.tag == "cylinder":
                spec = f'cylinder {child.get("radius")} {child.get("length")} 0'
            else:
                raise ValueError(f"지원하지 않는 충돌 형상: {child.tag}")
            result.append(Shape(name, origin(entry.find("origin")), spec))
    return result


def build_models(project):
    """실행용 제어 설정을 시작하지 않고 저장소의 시각화 Xacro만 펼친다."""
    workcell = project / "ws_cobot1/src/cake_workcell_description"
    dsr = project / "ws_dsr/src"
    packages = {
        "cake_workcell_description": workcell,
        "m0609_rg2_bringup": dsr / "rg2/m0609_rg2_bringup",
        "dsr_description2": dsr / "doosan-robot2/dsr_description2",
        "onrobot_rg_description": dsr / "onrobot-ros2/onrobot_rg_description",
    }

    # 임시 ament 색인에서 현재 저장소의 include를 찾는다. 홈의 다른 설치본은 사용하지 않는다.
    with TemporaryDirectory(prefix="cake-xacro-") as directory:
        prefix = Path(directory)
        index = prefix / "share/ament_index/resource_index/packages"
        index.mkdir(parents=True)
        for name, path in packages.items():
            (index / name).touch()
            (prefix / "share" / name).symlink_to(path, target_is_directory=True)
        env = {**os.environ, "AMENT_PREFIX_PATH": str(prefix)}

        def expand(path, *args):
            """개별 Xacro의 XML만 읽고 실행 노드와 ROS 그래프는 열지 않는다."""
            return ET.fromstring(subprocess.check_output(
                ["xacro", str(path), *args], env=env,
            ))

        robot_xml = expand(
            packages["m0609_rg2_bringup"] / "xacro/my_m0609_rg2.urdf.xacro",
            "mode:=virtual", "model:=m0609", "host:=127.0.0.1",
        )
        # 기존 필기 경로의 케이크 중심과 높이를 재현하는 배치를 명시한다.
        scene_xml = expand(workcell / "urdf/workcell.urdf.xacro", "layout:=legacy")
    # Xacro의 world→base 고정 관절을 빼고 BASE를 두 모델의 공통 원점으로 사용한다.
    for joint in robot_xml.findall("joint"):
        if joint.find("child").get("link") == "base_link":
            robot_xml.remove(joint)
    for tag in robot_xml.findall("ros2_control"):
        robot_xml.remove(tag)
    robot_xml.remove(robot_xml.find("link[@name='world']"))
    return Model(robot_xml, "base_link"), Model(scene_xml, "workcell_root"), packages


def plan(scene, dimensions):
    """모델 윗면을 기존 변환기로 읽고 모든 TCP 목표의 형상과 단계 규칙을 검사한다."""
    workcell = scene.frames({})
    cake_link = scene.xml.find("link[@name='fixture_a_cake_link']")
    cake_bottom = workcell["fixture_a_cake_link"] @ origin(cake_link.find("collision/origin"))
    height = dimensions["height_mm"] / 1000
    center = (cake_bottom @ [0, 0, height, 1])[:3]
    radius = dimensions["top_diameter_mm"] / 2000
    canvas, strokes = letter_strokes(radius)
    base_mm = 1000 * cake_bottom[:3, 3]
    cake = CylinderCake(radius * 1000, height * 1000, *base_mm)
    quaternion = pen_quat_top(tilt_deg=0.0)
    goals = []
    for stroke in strokes:
        # 현행 원통형 변환 결과 위에서만 접촉하고 획 사이에는 10mm 들어 올린다.
        positions_mm = [
            cake.top_to_abs(*pixel_to_mm_top(x, y, canvas, canvas, cake.radius))
            for x, y in stroke
        ]
        first_x, first_y, first_z = positions_mm[0]
        last_x, last_y, last_z = positions_mm[-1]
        goals.append(Waypoint(
            Pose(first_x / 1000, first_y / 1000, first_z / 1000 + LIFT, *quaternion),
            False, TRAVEL_SPEED,
        ))
        for x, y, z in positions_mm:
            goals.append(Waypoint(Pose(x / 1000, y / 1000, z / 1000, *quaternion), True, DRAW_SPEED))
        goals.append(Waypoint(
            Pose(last_x / 1000, last_y / 1000, last_z / 1000 + LIFT, *quaternion),
            False, TRAVEL_SPEED,
        ))
    trajectory = Trajectory(goals)
    waypoints = [{**vars(wp.pose), "speed": wp.speed} for wp in goals]
    motion = plan_drawing_motion(waypoints, trajectory.pen_states)
    if not motion.success or motion.total_strokes != len(strokes):
        raise ValueError(motion.error_msg or "획 수 불일치")

    points = np.array([[wp.pose.x, wp.pose.y, wp.pose.z] for wp in trajectory.waypoints])
    flags = np.array(trajectory.pen_states)
    drawn = points[flags]
    if not np.allclose(np.ptp(drawn[:, :2], axis=0), [0.07, 0.04], atol=1e-9):
        raise ValueError("필기 폭 또는 높이가 요구치와 다릅니다.")
    if not np.allclose(points[:, 2], np.where(flags, center[2], center[2] + LIFT)):
        raise ValueError("펜의 접촉·이송 높이가 올바르지 않습니다.")
    rim_margin = radius - max(np.linalg.norm(points[:, :2] - center[:2], axis=1))
    # 원판은 볼록하므로 선분의 양 끝이 이 반경 안에 있으면 전체 선분도 안에 있다.
    if rim_margin < 0.003:
        raise ValueError("펜촉과 케이크 가장자리 간격이 3mm 미만입니다.")
    if any(not np.allclose(wp.pose.qx, 1.0) or
           not np.allclose([wp.pose.qy, wp.pose.qz, wp.pose.qw], 0)
           for wp in trajectory.waypoints):
        raise ValueError("펜촉 +Z 방향이 수직 아래를 향하지 않습니다.")

    phases = [command.phase.value for command in motion.commands]
    phases[0] = "INITIAL_ABOVE_STROKE"
    rows = [(0.0, phases[0], *points[0], 0.0)]
    for i, (start, end) in enumerate(zip(points, points[1:]), start=1):
        delta = end - start
        length = np.linalg.norm(delta)
        if flags[i] != flags[i - 1] and not np.allclose(delta[:2], 0):
            raise ValueError("펜을 올리거나 내릴 때 XY가 이동합니다.")
        if length < 1e-10:
            continue
        # 상승도 필기 속도로 제한한다. 각 선분은 정지-정지 5차 보간으로 재생한다.
        cap = DRAW_SPEED if flags[i] or flags[i - 1] else TRAVEL_SPEED
        duration = max(
            1.875 * length / (cap / 1000),
            math.sqrt((10 / math.sqrt(3)) * length / (LINEAR_ACCEL / 1000)),
        )
        intervals = math.ceil(max(duration / 0.04, 1.875 * length / 0.001))
        t0 = rows[-1][0]
        for j in range(1, intervals + 1):
            u = j / intervals
            s = u**3 * (10 + u * (-15 + 6 * u))
            speed = 1000 * length / duration * 30 * u**2 * (1 - u)**2
            rows.append((t0 + u * duration, phases[i], *(start + s * delta), speed))
    return center, radius, rim_margin, trajectory, rows


def solve(robot, rows):
    """관절 한계 내에서 연속 IK를 계산하고 TCP 오차와 관절 속도를 검사한다."""
    limits = [robot.xml.find(f"joint[@name='{name}']/limit") for name in JOINTS]
    lower = [float(limit.get("lower")) for limit in limits]
    upper = [float(limit.get("upper")) for limit in limits]
    speed_limits = np.array([float(limit.get("velocity")) for limit in limits])
    target_positions = np.array([row[2:5] for row in rows])

    def ik(target, seed):
        """선형 위치·회전 오차를 동시에 최소화해 하나의 관절 자세를 만든다."""
        def residual(q):
            tip, orientation = robot.tcp(q)
            return np.r_[tip - target, 0.2 * (DOWN.inv() * orientation).as_rotvec()]

        result = least_squares(
            residual, seed, bounds=(lower, upper),
            ftol=1e-10, xtol=1e-10, gtol=1e-11, max_nfev=80,
        )
        error = residual(result.x)
        if np.linalg.norm(error[:3]) > 0.00005 or np.linalg.norm(error[3:]) > 0.0002:
            raise ValueError("역기구학 오차가 위치·자세 허용치를 넘었습니다.")
        return result.x

    # 정지한 첫 필기 시작점 위에서 시뮬레이션을 시작한다.
    # 검증 중 충돌이 발견되면 다른 자세에서 다시 이어갈 수 있도록 시작 seed를 고정한다.
    seed = np.radians([0, 30, 95, 0, 55, 0])
    joints = []
    max_error_mm = 0.0
    for position in target_positions:
        seed = ik(position, seed)
        joints.append(seed)
        max_error_mm = max(max_error_mm, 1000 * np.linalg.norm(robot.tcp(seed)[0] - position))
    joints = np.array(joints)
    times = np.array([row[0] for row in rows])
    sampled_speeds = np.abs(np.diff(joints, axis=0) / np.diff(times)[:, None])
    if np.any(sampled_speeds > speed_limits):
        raise ValueError("샘플 간 평균 관절 속도가 URDF 한계를 넘었습니다.")
    return joints, max_error_mm, np.max(sampled_speeds, axis=0), speed_limits


def collision_report(robot, scene, packages, joints, checker):
    """펜 몸체, 로봇 링크 및 환경의 삼각형 충돌을 전 자세에서 확인한다."""
    arm = shapes(robot, packages)
    pen = np.eye(4)
    # 펜이 그리퍼와 겹치는 부분은 고정 체결을 나타낸다. 마지막 1mm는 점 접촉 모델로 검사한다.
    pen[:3, 3] = [0, 0, (0.175 + PEN_TIP - 0.001) / 2]
    arm.append(Shape("virtual_pen", pen, f"cylinder 0.003 {PEN_TIP - 0.001 - 0.175} 0"))
    obstacles = shapes(scene, packages)
    scene_frames = scene.frames({})
    source = io.StringIO()
    all_shapes = arm + obstacles
    print(len(all_shapes), file=source)
    for shape in all_shapes:
        print(shape.spec, file=source)

    # 조립부가 원래 겹치는 직접 연결 팔 링크, 고정 RG2 부품끼리의 접촉을 제외한다.
    adjacent = {frozenset((data[0], data[1])) for data in robot.joints.values()}
    adjacent.add(frozenset(("link_6", "rg2_base_link")))
    pairs = []
    for i, first in enumerate(arm):
        for j in range(i + 1, len(all_shapes)):
            second = all_shapes[j]
            a, b = first.link, second.link
            fixed_tool_parts = a.startswith("rg2_") and (
                b.startswith("rg2_") or b == "virtual_pen"
            )
            if a == b or frozenset((a, b)) in adjacent or fixed_tool_parts:
                continue
            pairs.append((i, j))

    print(len(pairs), file=source)
    for i, j in pairs:
        print(i, j, file=source)
    print(len(joints), file=source)
    for q in joints:
        frames = robot.frames(dict(zip(JOINTS, q)))
        frames["virtual_pen"] = frames["rg2_base_link"]
        frames.update(scene_frames)
        for shape in all_shapes:
            pose = frames[shape.link] @ shape.pose
            print(*pose[:3].ravel(), file=source)
    run = subprocess.run(
        [str(checker)], input=source.getvalue(), text=True, stdout=subprocess.PIPE, check=True,
    )
    hits = []
    for line in run.stdout.splitlines():
        sample, i, j = map(int, line.split())
        hits.append((sample, all_shapes[i].link, all_shapes[j].link))
    # 고정 베이스 겹침도 숨기지 않고 보고한다. 움직이는 링크의 검출만 궤적 실패다.
    fixed_overlaps = {}
    moving_hits = []
    for sample, first, second in hits:
        if first == "base_link" and second in scene_frames:
            pair = (first, second)
            fixed_overlaps[pair] = fixed_overlaps.get(pair, 0) + 1
        else:
            moving_hits.append((sample, first, second))
    return len(pairs), moving_hits, fixed_overlaps


def preview(path, radius, center, trajectory):
    """필기와 펜 이송을 구별할 수 있는 윗면 미리보기를 저장한다."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 6), layout="constrained")
    ax.add_patch(plt.Circle((0, 0), radius * 1000, facecolor="#fff7e9", edgecolor="#8d542e"))
    prior = None
    drawn = None
    travel = None
    for waypoint in trajectory.waypoints:
        xy = (np.array([waypoint.pose.x, waypoint.pose.y]) - center[:2]) * 1000
        if prior is not None:
            prev_xy, prev_down = prior
            if waypoint.pen_down and prev_down:
                drawn, = ax.plot(*np.stack((prev_xy, xy)).T, color="#a12f50", linewidth=2.7)
            elif not waypoint.pen_down and not prev_down:
                travel, = ax.plot(*np.stack((prev_xy, xy)).T, color="#677482",
                                  linestyle="--", linewidth=0.8, alpha=0.65)
        prior = xy, waypoint.pen_down
    ax.set(xlim=(-62, 62), ylim=(-62, 62), aspect="equal", xlabel="BASE X from cake center (mm)",
           ylabel="BASE Y from cake center (mm)")
    ax.legend([drawn, travel], ["Pen down", "Transfer at Z + 10 mm"], loc="lower center")
    fig.savefig(path, dpi=155)
    plt.close(fig)


def main():
    """모델을 계산하고 검증 결과를 파일로 남긴다. 로봇 통신은 열지 않는다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=SOURCE_ROOT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay", action="store_true", help="검증 CSV를 가상 관절 상태와 RViz 마커로 재생한다.")
    args = parser.parse_args()
    if args.replay:
        from cake_lettering_replay import replay
        replay(args.project.resolve(), args.output.resolve())
        return
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output.resolve()
    robot, scene, packages = build_models(args.project.resolve())
    dimensions = json.loads((args.project / "config/test_cake.json").read_text())
    center, radius, margin, trajectory, rows = plan(scene, dimensions)
    joints, max_error, joint_speeds, joint_limits = solve(robot, rows)

    # 컴파일 파일은 실행이 중단되어도 결과 디렉토리에 남지 않는다.
    flags = subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "fcl", "assimp"], text=True,
    )
    with TemporaryDirectory(prefix="cake-fcl-") as directory:
        checker = Path(directory) / "cake_mesh_check"
        subprocess.run([
            "c++", "-O2", "-std=c++17", str(Path(__file__).with_name("cake_mesh_check.cpp")),
            *(flag for flag in shlex.split(flags) if not flag.startswith("-std=")),
            "-o", str(checker),
        ], check=True)
        pair_count, hits, fixed_overlaps = collision_report(robot, scene, packages, joints, checker)

    with (output / "lettering_trajectory.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "phase", "tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_speed_mm_s",
                         *(f"{name}_rad" for name in JOINTS)])
        for row, q in zip(rows, joints):
            writer.writerow([f"{row[0]:.8f}", row[1], *[f"{v:.8f}" for v in (*row[2:], *q)]])
    preview(output / "lettering_preview.png", radius, center, trajectory)
    summary = {
        "verification": "offline_kinematics_sampled_collision",
        "cake_center_base_mm": (center * 1000).tolist(),
        "cake_dimensions_mm": dimensions,
        "draw_size_mm": [70, 40],
        "drawing_z_base_mm": center[2] * 1000,
        "lifted_z_base_mm": (center[2] + LIFT) * 1000,
        "rim_clearance_mm": margin * 1000,
        "strokes": int(np.count_nonzero(np.diff([False, *trajectory.pen_states]) & trajectory.pen_states)),
        "xyz_waypoints": len(trajectory.waypoints),
        "samples": len(rows),
        "duration_s": rows[-1][0],
        "max_tcp_speed_mm_s": max(row[-1] for row in rows),
        "linear_acceleration_cap_mm_s2": LINEAR_ACCEL,
        "max_sample_tcp_position_error_mm": max_error,
        "max_sample_interval_joint_speed_deg_s": np.degrees(joint_speeds).tolist(),
        "urdf_joint_speed_limits_deg_s": np.degrees(joint_limits).tolist(),
        "virtual_tcp_rg2_base_mm": [0, 0, PEN_TIP * 1000],
        "collision_shape_pairs_per_sample": pair_count,
        "moving_collision_hits": hits,
        "static_base_overlaps": [
            {"links": pair, "samples": count}
            for pair, count in sorted(fixed_overlaps.items())
        ],
    }
    report = json.dumps(summary, indent=2, ensure_ascii=False)
    (output / "lettering_validation.json").write_text(report + "\n")
    print(report)
    if hits:
        raise SystemExit("충돌 검사 실패. lettering_validation.json에서 충돌 링크를 확인하세요.")


if __name__ == "__main__":
    main()
