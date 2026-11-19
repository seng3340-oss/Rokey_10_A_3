"""실제 RG2로 45도 파지·인출 후 RG2를 수직으로 세우는 임시 시험."""

import argparse
from contextlib import contextmanager
from datetime import datetime
import math
import os
from pathlib import Path
import time

from system_launcher import RobotSession, configure_discovery, validate_existing_nodes

ROOT = Path(__file__).resolve().parents[1]
# 기존 grip의 240mm는 손끝이다. 사용자가 요청한 지문 중앙 추정값 220mm로 회전한다.
# 이 값은 시험의 회전 중심이며 로봇에 등록된 grip TCP 값과 별도로 사용한다.
GRIP_CENTER_MM = (0.0, 0.0, 220.0)


def confirm(message):
    """다음 명령은 작업자가 눈으로 확인하고 YES를 입력했을 때만 진행한다."""
    if input(f"{message} [YES / 그 외 종료] ").strip() != "YES":
        raise RuntimeError("시험을 중단했습니다.")


def check_step(success, label):
    """이동이나 그리퍼 서비스가 실패하면 이후 명령을 보내지 않는다."""
    if not success:
        raise RuntimeError(f"{label} 실패. 현재 로봇과 펜 상태를 확인하세요.")


def read_rack_and_poses(args):
    """팀원 티칭 자세에서 파지 중심을 고정한 채 BASE 수직 기준 45° 자세를 계산한다."""
    import numpy as np
    import yaml
    from scipy.spatial.transform import Rotation

    from cake_decorator.goal_mapping import validate_reference

    rack = yaml.safe_load(args.rack_config.read_text())["robot_executor_node"]["ros__parameters"]
    joints = validate_reference(rack["pen_stand_approach_pose"])
    stand = validate_reference(rack["pen_stand_pose"])
    lift = validate_reference(rack["pen_gripper_pose"])
    original = Rotation.from_euler("ZYZ", stand[3:], degrees=True)
    original_axis = original.apply([0.0, 0.0, 1.0])
    if original_axis[2] > -math.cos(math.radians(2.0)):
        raise ValueError("팀원 티칭 파지 자세의 TCP +Z가 아래를 향하는지 확인하세요.")

    # 이번 시험은 기존 파지의 좌우 개폐 방향(BASE X)에 직각인 Y 방향으로 눕힌다.
    # 기본 90도는 손끝 +Y / 손목 -Y이며 반대편은 270도로 지정한다.
    # 개폐 방향은 모델 장착과 티칭 당시 TCP 방향이 일치하는 조건으로 해석한다.
    if not math.isfinite(args.tilt_heading_deg):
        raise ValueError("기울임 방향에는 유한한 각도를 입력하세요.")
    heading = math.radians(args.tilt_heading_deg)
    diagonal = math.sqrt(0.5)
    target_axis = np.array([
        diagonal * math.cos(heading),
        diagonal * math.sin(heading),
        -diagonal,
    ])
    axis = np.cross(original_axis, target_axis)
    angle = math.atan2(float(np.linalg.norm(axis)), float(original_axis @ target_axis))
    turn_axis = axis / np.linalg.norm(axis)

    # TCP 원점과 실제 파지 중심이 다르면 회전 후 TCP XYZ를 보정한다.
    # offset은 원본 DRL의 플랜지 초기 TCP에서 파지 중심까지의 TOOL mm 값이다.
    offset = np.asarray(args.jaw_center_offset, dtype=float)
    if not np.isfinite(offset).all():
        raise ValueError("파지 중심 오프셋은 유한한 mm 값 3개여야 합니다.")
    center = np.asarray(stand[:3]) + original.apply(offset)
    # 220mm 파지 중심을 실제 활성 TCP로 적용한 상태에서 한 번에 회전한다.
    # 플랜지 TCP로 이 목표를 한 번에 보내면 파지 중심이 보간 중 크게 벗어난다.
    rotation = Rotation.from_rotvec(turn_axis * angle) * original
    xyz = center - rotation.apply(offset)
    euler = rotation.as_euler("ZYZ", degrees=True)
    turns = [[float(value) for value in (*xyz, *euler)]]
    upper = turns[-1].copy()
    clearance = lift[2] - stand[2]
    if clearance <= 0.0:
        raise ValueError("팀원 설정의 들어올리기 높이가 파지 높이보다 낮습니다.")
    upper[2] += clearance
    # 거치대를 벗어난 높이에서 집을 때의 회전을 역순으로 푼다.
    # grip 중심은 공중의 같은 점에 유지되고, 펜은 RG2 안에서 비스듬히 물려 있다.
    upright = []
    for pose in [*reversed(turns[:-1]), stand]:
        raised = pose.copy()
        raised[2] += clearance
        upright.append(raised)
    return joints, stand, turns, upper, upright


def adapt_to_active_tcp(settings, flange_pose, tcp_pose):
    """초기 TCP로 기록한 목표를 현재 적용 TCP 기준으로 변환한다.

    플랜지와 활성 TCP를 같은 BASE 기준에서 조회해 두 좌표계의 관계를 구한다.
    현재 grip이 적용돼 있어도 원래 티칭한 플랜지 경로와 파지 중심을 유지한다.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    flange_rotation = Rotation.from_euler("ZYZ", flange_pose[3:], degrees=True)
    tcp_rotation = Rotation.from_euler("ZYZ", tcp_pose[3:], degrees=True)
    offset = flange_rotation.inv().apply(np.array(tcp_pose[:3]) - np.array(flange_pose[:3]))
    local_rotation = flange_rotation.inv() * tcp_rotation

    def convert(pose):
        """목표 플랜지 자세에 현재 TCP의 고정 장착 변환을 합성한다."""
        rotation = Rotation.from_euler("ZYZ", pose[3:], degrees=True)
        xyz = np.array(pose[:3]) + rotation.apply(offset)
        euler = (rotation * local_rotation).as_euler("ZYZ", degrees=True)
        return [float(value) for value in (*xyz, *euler)]

    joints, stand, turns, upper, upright = settings
    return (
        joints,
        convert(stand),
        [convert(pose) for pose in turns],
        convert(upper),
        [convert(pose) for pose in upright],
    )


@contextmanager
def pivot_tcp(session, offset):
    """단일 회전의 기준을 파지 중심에 두고 종료 시 원래 TCP·모드를 복원한다."""
    from dsr_msgs2.srv import (
        ConfigCreateTcp, ConfigDeleteTcp, GetCurrentTcp,
        GetCurrentToolFlangePosx, GetRobotMode, SetCurrentTcp, SetRobotMode,
    )
    from contact_tcp import relative_tcp

    prefix = "/dsr01/dsr_controller2/"

    def call(kind, path, **fields):
        """TCP 등록·선택·모드 설정은 기존 서비스 호출 검사를 재사용한다."""
        return session.call(kind, prefix + path, kind.Request(**fields), timeout=5)

    original = call(GetCurrentTcp, "tcp/get_current_tcp").info
    mode = call(GetRobotMode, "system/get_robot_mode").robot_mode
    if mode not in (0, 1):
        raise RuntimeError("TCP 설정이 가능한 수동 또는 자동 모드로 준비하세요.")
    flange = list(call(GetCurrentToolFlangePosx, "aux_control/get_current_tool_flange_posx", ref=0).pos)
    old_pose = session.pose()
    stamp = datetime.now().strftime("%m%d%H%M%S%f")
    name = "PickPivot_" + stamp
    backup = original or "PickPrev_" + stamp
    attempted = False
    registered = False
    restored = False

    def select(tcp_name):
        """TCP 변경에 필요한 수동 모드를 거쳐 원래 운전 모드로 복귀한다."""
        try:
            if mode != 0:
                call(SetRobotMode, "system/set_robot_mode", robot_mode=0)
            call(SetCurrentTcp, "tcp/set_current_tcp", name=tcp_name)
        finally:
            if mode != 0:
                call(SetRobotMode, "system/set_robot_mode", robot_mode=int(mode))

    try:
        try:
            if mode != 0:
                call(SetRobotMode, "system/set_robot_mode", robot_mode=0)
            if not original:
                call(ConfigCreateTcp, "tcp/config_create_tcp", name=backup, pos=relative_tcp(flange, old_pose))
            call(ConfigCreateTcp, "tcp/config_create_tcp", name=name, pos=[*offset, 0.0, 0.0, 0.0])
            registered = True
        finally:
            if mode != 0:
                call(SetRobotMode, "system/set_robot_mode", robot_mode=int(mode))
        attempted = True
        select(name)
        if call(GetCurrentTcp, "tcp/get_current_tcp").info != name:
            raise RuntimeError("파지 중심 TCP 적용을 확인하지 못했습니다.")
        print(f"단일 회전 기준 TCP 적용 {offset}. 종료 후 복원할 TCP {backup}")
        yield
    finally:
        # 이름 없는 초기 TCP는 수치 설정을 보관한 backup 이름으로 복원한다.
        if attempted:
            select(backup)
            if call(GetCurrentTcp, "tcp/get_current_tcp").info != backup:
                raise RuntimeError(f"원래 TCP 복원을 확인하세요. 복원 이름 {backup}")
            restored = True
            print(f"원래 TCP 복원 완료 {backup}")
        if registered and (restored or not attempted):
            try:
                if mode != 0:
                    call(SetRobotMode, "system/set_robot_mode", robot_mode=0)
                call(ConfigDeleteTcp, "tcp/config_delete_tcp", name=name)
            finally:
                if mode != 0:
                    call(SetRobotMode, "system/set_robot_mode", robot_mode=int(mode))


def pulse_gripper(write_output, channel, wait=time.sleep):
    """펜던트 DRL과 같은 ON 1초 → OFF → 1초 대기 순서로 RG2를 작동시킨다."""
    try:
        write_output(channel, 1)
        wait(1.0)
    finally:
        # ON 요청 응답이 끊기거나 작업을 중단해도 OFF 복귀를 시도한다.
        write_output(channel, 0)
    wait(1.0)


def run_pick(robot, settings, position_type, grip, confirm_step=print):
    """수직 하강 → 기울임 → 닫기 → 인출 → 공중에서 RG2 수직 복귀 순서다."""
    joints, stand, turns, upper, upright = settings

    def line(pose, label):
        """저속 직선 이동 뒤 목표 TCP의 위치와 방향이 맞는지 확인한다."""
        print(label, flush=True)
        check_step(
            robot.move_line(
                position_type(*pose),
                speed_mm_s=50.0,
                acc_mm_s2=100.0,
                angular_speed_deg_s=20.0,
                angular_acc_deg_s2=40.0,
            ),
            label,
        )
        check_step(robot.verify_pose(pose), f"{label} 도달 확인")

    check_step(robot.move_joint(joints, speed_deg_s=30.0, acc_deg_s2=60.0), "거치대 상공 이동")
    grip("open")
    confirm_step("그리퍼 열기 완료. 펜 위치로 하강합니다.")
    line(stand, "기존 티칭 자세로 수직 하강")
    confirm_step("하강 완료. 물리는 중심을 기준으로 기울입니다.")
    for index, pose in enumerate(turns, start=1):
        line(pose, f"물리는 중심을 유지하며 기울이기 {index}/{len(turns)}")
    grip("close")
    confirm_step("그리퍼 닫기 완료. 펜을 들어올립니다.")
    line(upper, "45° 파지 자세를 유지해 수직으로 들어올리기")
    confirm_step("인출 완료. 공중에서 RG2를 수직으로 세웁니다.")
    for index, pose in enumerate(upright, start=1):
        line(pose, f"공중에서 RG2를 수직으로 세우기 {index}/{len(upright)}")


def main():
    """실제 드라이버와 활성 TCP를 확인한 뒤 조회 또는 파지 시험을 한 번 실행한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.100", help="실제 브링업의 로봇 IP")
    parser.add_argument("--read-pose", action="store_true", help="활성 TCP 이름과 BASE 자세만 읽기")
    parser.add_argument("--tcp", help="선택 사항. 입력하면 현재 TCP 이름과 비교하고, 생략하면 현재 적용 TCP 사용")
    parser.add_argument(
        "--rack-config",
        type=Path,
        default=ROOT.parent / "ws_cobot_pjt_frustum_review/ws_cobot1/src/cake_decorator/config/workspace_config.yaml",
        help="팀원 브랜치 거치대 YAML 경로",
    )
    parser.add_argument("--preview", action="store_true", help="계산한 자세만 출력하고 ROS에 연결하지 않기")
    # 검증된 실행 스크립트는 시작 확인 없이 파지 시퀀스를 진행한다.
    parser.add_argument("--no-confirm", action="store_true", help="YES 입력 없이 바로 실행")
    parser.add_argument("--step-confirm", action="store_true", help="선택 시 중간 단계마다 YES 대기")
    parser.add_argument(
        "--tilt-heading-deg",
        type=float,
        default=90.0,
        help="손끝의 기울임 방향. 기본 90=+Y(손목 -Y), 270=-Y(손목 +Y)",
    )
    parser.add_argument(
        "--jaw-center-offset",
        type=float,
        nargs=3,
        default=GRIP_CENTER_MM,
        metavar=("X", "Y", "Z"),
        help="초기 TCP 기준 파지 중심 (TOOL mm). 기본은 지문 중앙 추정값 0 0 220",
    )
    parser.add_argument(
        "--gripper-only", choices=("open", "close"),
        help="팔 이동 없이 RG2 열기 또는 닫기만 시험",
    )
    parser.add_argument(
        "--gripper-interface", choices=("modbus", "digital"), default="modbus",
        help="기본은 OnRobot 서비스. digital은 DRL의 출력 1/2 펄스",
    )
    args = parser.parse_args()
    if args.read_pose and args.preview:
        parser.error("--read-pose와 --preview는 각각 실행하세요.")
    settings = None
    if not args.read_pose and not args.gripper_only:
        if args.tilt_heading_deg is None or args.jaw_center_offset is None:
            parser.error("--tilt-heading-deg와 --jaw-center-offset이 필요합니다.")
        settings = read_rack_and_poses(args)
        print(f"손끝의 BASE 수평 방향 {args.tilt_heading_deg:g}도 / 수직 아래에서 45도 기울임")
        print(f"거치대 상공 관절 {settings[0]}")
        print(f"수직 하강 자세 {settings[1]}")
        print(f"계산한 45° 파지 {[round(value, 3) for value in settings[2][-1]]}")
        print(f"계산한 인출 자세 {[round(value, 3) for value in settings[3]]}")
        print(f"RG2 수직 복귀 자세 {[round(value, 3) for value in settings[4][-1]]}", flush=True)
        if args.preview:
            return

    from importlib.metadata import version
    from dsr_msgs2.srv import GetCurrentTcp, GetCurrentToolFlangePosx, SetCtrlBoxDigitalOutput

    installed = version("pymodbus")
    if installed != "3.9.2":
        raise RuntimeError(f"이 시험에서 검증한 pymodbus는 3.9.2입니다. 현재 {installed}")

    configure_discovery("real", os.environ)
    session = RobotSession()
    try:
        session.identity("real", args.host)
        tcp = session.call(
            GetCurrentTcp,
            "/dsr01/dsr_controller2/tcp/get_current_tcp",
            GetCurrentTcp.Request(),
            timeout=5,
        ).info
        print(f"활성 TCP {tcp!r} / BASE mm·ZYZ° {session.pose()}", flush=True)
        if args.read_pose:
            return
        # 이름 지정은 선택 검사다. 기본 동작은 펜던트에서 현재 적용한 TCP를 사용한다.
        if args.tcp is not None and tcp != args.tcp:
            raise RuntimeError(f"활성 TCP가 티칭한 이름과 다릅니다. 기대값 {args.tcp!r}")

        from cake_decorator.doosan_robot import DoosanRobotClient
        from cake_decorator.drawing_motion import DsrPos

        validate_existing_nodes(session.node.get_node_names_and_namespaces(), reuse_driver=True)
        robot = DoosanRobotClient(session.node)
        io_path = "/dsr01/dsr_controller2/io/set_ctrl_box_digital_output"

        def write_output(channel, value):
            """DRL에서 쓴 두산 제어함 출력을 ROS 서비스로 전달한다."""
            session.call(
                SetCtrlBoxDigitalOutput,
                io_path,
                SetCtrlBoxDigitalOutput.Request(index=channel, value=value),
                timeout=5,
            )

        def send_gripper(action):
            """현재 설치된 OnRobot 명령 형식으로 열기·닫기를 요청한다."""
            if args.gripper_interface == "digital":
                write_output(1, 0)
                write_output(2, 0)
                pulse_gripper(write_output, 2 if action == "open" else 1)
            else:
                from onrobot_rg_msgs.srv import SetCommand

                # 실제 서버는 o/c 또는 0.1mm 단위 숫자를 받는다.
                # close(폭,힘) 문자열은 이 서버에서 처리되지 않는다.
                session.call(
                    SetCommand,
                    "/onrobot/sendCommand",
                    SetCommand.Request(command="o" if action == "open" else "c"),
                    timeout=5,
                )
                time.sleep(1.0)
            print(f"RG2 {action} 명령 전달. 실제 열림·파지는 눈으로 확인하세요.")

        def gripper_open():
            """RG2 드라이버에 실제 열기 명령 o를 보낸다."""
            send_gripper("open")

        def gripper_close():
            """RG2 드라이버에 실제 닫기 명령 c를 보낸다."""
            send_gripper("close")

        def grip(action):
            """시험 단계에서 호출한 동작을 명시적인 열기·닫기 함수로 연결한다."""
            if action == "open":
                gripper_open()
            else:
                gripper_close()

        if args.gripper_only:
            if not args.no_confirm:
                confirm(f"팔 이동 없이 RG2를 {args.gripper_only}합니다. 손가락 주변을 확인했다면")
            grip(args.gripper_only)
            return

        if not args.no_confirm:
            confirm("실제 M0609입니다. 티칭 당시 TCP와 파지 중심 오프셋, 상공 접근 경로를 확인했다면")
        with pivot_tcp(session, args.jaw_center_offset):
            flange = session.call(
                GetCurrentToolFlangePosx,
                "/dsr01/dsr_controller2/aux_control/get_current_tool_flange_posx",
                GetCurrentToolFlangePosx.Request(ref=0),
                timeout=5,
            )
            pivot_settings = adapt_to_active_tcp(settings, list(flange.pos), session.pose())
            run_pick(
                robot, pivot_settings, DsrPos, grip,
                confirm_step=confirm if args.step_confirm and not args.no_confirm else print,
            )
        print("RG2는 수직, 펜은 비스듬히 물린 상태로 시험을 마쳤습니다. 펜촉 TCP를 교정하세요.")
    finally:
        session.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit("시험 종료. 진행 중이던 동작과 현재 파지 상태를 확인하세요.")
    except (RuntimeError, ValueError, OSError, KeyError, EOFError) as exc:
        raise SystemExit(f"시험 종료. {exc}")
