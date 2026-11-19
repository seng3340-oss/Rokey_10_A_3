"""45도로 물린 펜으로 케이크가 없는 작업 위치의 옆면을 공중 추종한다."""

import argparse
import json
import math
import os
from pathlib import Path
import time
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

from real_pen_pick_45 import confirm, check_step, read_rack_and_poses, ROOT
from system_launcher import RobotSession, configure_discovery, validate_existing_nodes

TIP_MM = np.array([1.095, -46.474, 271.695])


def euler(rotation):
    """ZYZ의 동등한 표현 경고만 처리하며 회전 행렬의 방향은 보존한다."""
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected.*")
        return rotation.as_euler("ZYZ", degrees=True)


def make_path(args, initial):
    """등록한 펜촉 위치를 쓰고, 알려진 45도 파지 방향으로 몸체 자세를 계산한다."""
    from cake_decorator.frustum_path import DrawingFrustum

    dimensions = json.loads((ROOT / "config/test_cake.json").read_text())
    reference = json.loads(args.reference.read_text())
    if reference.get("mode") != "real":
        raise ValueError("실물에서 기록한 케이크 기준점 파일이 필요합니다.")
    cx, cy, top = map(float, reference["pose"][:3])
    height = float(dimensions["height_mm"])
    cake = DrawingFrustum(
        r_top=dimensions["top_diameter_mm"] / 2,
        r_bottom=dimensions["bottom_diameter_mm"] / 2,
        height=height, cx=cx, cy=cy, cz=top-height,
    )
    # 성공한 파지 시험의 마지막 수직 자세와 파지 당시 45도 자세의 차이로
    # 플랜지 안에서의 펜축을 구한다. 원점 교정값 XYZ로 축 방향을 추측하지 않는다.
    settings = read_rack_and_poses(args)
    held = Rotation.from_euler("ZYZ", settings[2][-1][3:], degrees=True)
    vertical = Rotation.from_euler("ZYZ", settings[4][-1][3:], degrees=True)
    pen_local = held.inv().apply([0., 0., -1.])
    base_pen = vertical.apply(pen_local)
    inward_heading = math.atan2(base_pen[1], base_pen[0])
    start = inward_heading - math.pi
    level = height * args.height_ratio
    z = cake.cz + level
    radius = cake.radius_at(level) + args.gap_mm
    # 이번 파지의 실제 J6는 약 +43도다. 시계 경로에서 +360도를 넘었으므로
    # ±180도 표현 보정 후 반시계 경로를 기본으로 다시 검사한다.
    sweep = math.radians(args.sweep_deg) * (-1 if args.direction == "cw" else 1)

    def point(theta, distance=radius, z_value=z):
        """펜은 중심 쪽을 향하고 그리퍼 길이축은 수직으로 유지한다."""
        yaw = theta + math.pi - inward_heading
        rotation = Rotation.from_rotvec([0., 0., yaw]) * vertical
        return [
            cx + distance * math.cos(theta),
            cy + distance * math.sin(theta),
            z_value,
            *map(float, euler(rotation)),
        ]

    # 현재 위치에서 위로 이탈한 후 상면보다 높은 곳으로 횡이동한다.
    # 케이크 바깥에서 목표 자세로 바꾸고 옆면 높이까지 내려온다.
    safe_z = max(initial[2] + 50., top + 150.)
    high = list(initial)
    high[2] = safe_z
    outside = point(start, radius + 40., safe_z)
    overhead = outside.copy()
    overhead[3:] = initial[3:]
    entry = [high, overhead, outside, point(start, radius + 40.), point(start)]
    samples = [
        point(start + sweep * i / math.ceil(args.sweep_deg))
        for i in range(math.ceil(args.sweep_deg) + 1)
    ]
    end_angle = start + sweep
    exit_path = [point(end_angle, radius + 40.), point(end_angle, radius + 40., safe_z)]
    return entry, samples, exit_path, pen_local, (cx, cy, top, z, radius)


def continuous_j6(raw, previous):
    """±180도 표현 경계를 넘어도 이전 실제 각도에서 이어지는 회전수를 유지한다.

    예를 들어 179.86 다음의 -179.13은 180.87로 해석한다.
    연속값이 360도를 넘으면 다른 회전수로 바꿔 숨기지 않고 한계로 판정한다.
    """
    if not math.isfinite(raw) or not math.isfinite(previous):
        raise ValueError("J6 각도가 유효하지 않습니다.")
    return raw + 360.0 * round((previous - raw) / 360.0)


def preflight(robot, poses, initial):
    """현재 관절 해에서 모든 표본을 IK/FK 왕복 검사한다. 이동은 수행하지 않는다."""
    from dsr_msgs2.srv import (
        Fkin, GetCurrentPosj, GetCurrentToolFlangePosx, GetSolutionSpace, Ikin,
    )

    def query(key, request):
        result = robot._call(key, request, timeout=5)
        if result is None or not result.success:
            raise RuntimeError(f"사전 조회 실패 {key}")
        return result

    def matches(a, b):
        delta = Rotation.from_euler("ZYZ", a[3:], degrees=True).inv() * Rotation.from_euler("ZYZ", b[3:], degrees=True)
        return math.dist(a[:3], b[:3]) < .5 and math.degrees(delta.magnitude()) < .5

    joints = list(query("current_joints", GetCurrentPosj.Request()).pos)
    print(
        f"현재 실제 관절각 {[round(value, 2) for value in joints]}\n"
        f"현재 J6={joints[5]:.2f}도, 사양 범위 -360~+360도. "
        f"남은 각도 +방향 {360-joints[5]:.2f}도 / -방향 {joints[5]+360:.2f}도",
        flush=True,
    )
    solution = query("solution", GetSolutionSpace.Request(pos=joints)).sol_space
    flange = list(query("flange", GetCurrentToolFlangePosx.Request(ref=0)).pos)
    fk = list(query("forward", Fkin.Request(pos=joints, ref=0)).conv_posx)
    flange_basis = not matches(fk, initial)
    if flange_basis and not matches(fk, flange):
        raise RuntimeError("제어기 FK 기준을 판별하지 못했습니다.")
    deadline = time.monotonic() + 180.
    previous = joints
    j6 = [joints[5]]
    for i, pose in enumerate(poses):
        if time.monotonic() > deadline:
            raise RuntimeError("사전 검사가 180초를 넘었습니다.")
        target = list(pose)
        if flange_basis:
            rotation = Rotation.from_euler("ZYZ", pose[3:], degrees=True)
            target[:3] = list(np.array(pose[:3]) - rotation.apply(TIP_MM))
        solved = list(query("inverse", Ikin.Request(pos=target, sol_space=solution, ref=0)).conv_posj)
        raw_j6 = solved[5]
        solved[5] = continuous_j6(raw_j6, previous[5])
        # 보정한 연속 관절값 자체로 FK를 다시 확인한다.
        checked = list(query("forward", Fkin.Request(pos=solved, ref=0)).conv_posx)
        reasons = []
        if not matches(checked, target):
            reasons.append("IK/FK가 목표와 불일치")
        if not all(math.isfinite(v) and abs(v) <= bound for v, bound in zip(solved, [360,360,150,360,360,360])):
            reasons.append("관절 한계 또는 유효하지 않은 관절값 (J6 사양 ±360도)")
        if max(abs(a-b) for a,b in zip(previous, solved)) > 90.:
            reasons.append("이웃 표본 사이 관절 변화가 90도 초과")
        if reasons:
            raise RuntimeError(
                f"경로 {i}번: {', '.join(reasons)}. "
                f"펜촉 XYZ={[round(v, 2) for v in pose[:3]]}, "
                f"J6 이전={previous[5]:.2f} → 연속 목표={solved[5]:.2f}도 "
                f"(IK 원본 {raw_j6:.2f}도). "
                "이동을 시작하지 않습니다."
            )
        previous = solved
        j6.append(solved[5])
        if i % 60 == 0:
            print(f"도달 검사 {i+1}/{len(poses)}, 예상 J6={solved[5]:.2f}도", flush=True)
    after = list(query("current_joints", GetCurrentPosj.Request()).pos)
    if max(abs(a-b) for a,b in zip(joints,after)) > .1:
        raise RuntimeError("사전 검사 중 팔이 움직였습니다. 다시 준비하세요.")
    print(f"검사 완료. 예상 J6 {min(j6):.1f} ~ {max(j6):.1f}도")
    robot._side_validated_solution = int(solution)


def main():
    """이미 파지한 펜에 등록 TCP를 선택하고, 공중 원주 한 번을 저속 실행한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    files = sorted((ROOT / "log/system").glob("*-real-*/reference.json"))
    parser.add_argument("--reference", type=Path, default=files[-1] if files else None)
    parser.add_argument("--tcp", default="Pen_Side_45")
    parser.add_argument("--host", default="192.168.1.100")
    parser.add_argument("--sweep-deg", type=float, default=360.)
    parser.add_argument(
        "--direction", choices=("cw", "ccw"), default="ccw",
        help="위에서 본 원주 방향. 기본 ccw=반시계, cw=시계",
    )
    parser.add_argument("--height-ratio", type=float, default=.5)
    parser.add_argument("--gap-mm", type=float, default=1.)
    parser.add_argument("--speed", type=float, default=30., help="공중 이동·원주 속도 mm/s, 기본 30")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--check-path", action="store_true", help="선택 시 이동 전 IK/FK 사전 검사 수행")
    parser.add_argument("--rack-config", type=Path, default=ROOT.parent / "ws_cobot_pjt_frustum_review/ws_cobot1/src/cake_decorator/config/workspace_config.yaml")
    args = parser.parse_args()
    args.tilt_heading_deg = 90.
    args.jaw_center_offset = [0., 0., 220.]
    print(f"원주 진행 방향 {'시계' if args.direction == 'cw' else '반시계'}", flush=True)
    if (
        args.reference is None
        or not all(math.isfinite(x) for x in (args.sweep_deg,args.height_ratio,args.gap_mm,args.speed))
        or not 0 < args.sweep_deg <= 360
        or not 0 <= args.height_ratio <= 1
        or not 0 <= args.gap_mm <= 50
        or not 0 < args.speed <= 60
    ):
        parser.error("실물 기준점과 유효한 각도·높이·속도가 필요합니다.")

    if args.preview:
        path = make_path(args, [400.,0.,400.,0.,180.,0.])
        print(f"참조 {args.reference}\n중심 XY·상면 Z·시험 Z·반지름 {path[4]}\n원주 표본 {len(path[1])}")
        return

    from dsr_msgs2.srv import (
        GetCurrentTcp, GetCurrentToolFlangePosx, GetRobotMode, SetCurrentTcp, SetRobotMode,
    )
    from contact_tcp import relative_tcp
    from cake_decorator.doosan_robot import DoosanRobotClient
    from cake_decorator.drawing_motion import DsrPos
    from cake_decorator.surface_path import split_spline_samples

    configure_discovery("real", os.environ)
    session = RobotSession()
    try:
        session.identity("real", args.host)
        validate_existing_nodes(session.node.get_node_names_and_namespaces(), reuse_driver=True)
        confirm("케이크를 치웠고 성공한 +Y 방향 45° 파지 후 RG2가 수직이며, 펜이 고정돼 있다면")
        prefix = "/dsr01/dsr_controller2/"
        mode = session.call(GetRobotMode, prefix+"system/get_robot_mode", GetRobotMode.Request()).robot_mode
        if mode not in (0,1):
            raise RuntimeError("수동 또는 자동 모드로 준비하세요.")
        try:
            if mode != 0:
                session.call(SetRobotMode,prefix+"system/set_robot_mode",SetRobotMode.Request(robot_mode=0))
            session.call(SetCurrentTcp,prefix+"tcp/set_current_tcp",SetCurrentTcp.Request(name=args.tcp))
        finally:
            if mode != 0:
                session.call(SetRobotMode,prefix+"system/set_robot_mode",SetRobotMode.Request(robot_mode=int(mode)))
        selected = session.call(GetCurrentTcp,prefix+"tcp/get_current_tcp",GetCurrentTcp.Request()).info
        initial = session.pose()
        flange = session.call(GetCurrentToolFlangePosx,prefix+"aux_control/get_current_tool_flange_posx",GetCurrentToolFlangePosx.Request(ref=0)).pos
        actual = relative_tcp(list(flange),initial)
        if selected != args.tcp or math.dist(actual[:3],TIP_MM) > 1.0 or Rotation.from_euler("ZYZ",actual[3:],degrees=True).magnitude() > math.radians(1.):
            raise RuntimeError("적용한 펜촉 TCP가 교정값 1.095 -46.474 271.695 / 회전 0과 다릅니다.")
        entry, samples, exit_path, _, summary = make_path(args, initial)
        print(f"참조 파일 {args.reference}\n중심 XY, 상면 Z, 시험 높이 Z, 반지름 [mm] {summary}\n원주 {args.sweep_deg}도 / {args.speed}mm/s")
        robot = DoosanRobotClient(session.node)
        if args.check_path:
            preflight(robot,[*entry,*samples,*exit_path],initial)
        else:
            print("IK/FK 사전 검사 생략. 시작 확인 후 실제 경로를 실행합니다.", flush=True)
        confirm("표시된 케이크 위치와 팔·펜의 이동 공간을 확인했고 공중 원주 시험을 시작하려면")
        def line(point):
            check_step(
                robot.move_line(
                    DsrPos(*point), speed_mm_s=args.speed, acc_mm_s2=150.,
                    angular_speed_deg_s=30., angular_acc_deg_s2=60.,
                ),
                "공중 이동",
            )
            check_step(robot.verify_pose(point),"목표 도달")
        for point in entry:
            line(point)
        for group in split_spline_samples(samples):
            check_step(
                robot.move_spline(
                    [DsrPos(*p) for p in group],
                    speed_mm_s=args.speed, acc_mm_s2=150.,
                    angular_speed_deg_s=30., angular_acc_deg_s2=60.,
                ),
                "옆면 원주",
            )
            check_step(robot.verify_pose(group[-1]),"원주 도달")
        for point in exit_path:
            line(point)
        print(f"공중 원주 시험 완료. 활성 TCP는 {args.tcp}입니다. 펜은 계속 잡고 있습니다.")
    finally:
        session.close()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError,ValueError,OSError,KeyError,EOFError,KeyboardInterrupt) as exc:
        raise SystemExit(f"시험 중단. {exc} 현재 팔·펜 상태를 확인하세요.")
