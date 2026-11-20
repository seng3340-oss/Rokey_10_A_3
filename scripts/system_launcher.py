"""가상/실제 실행기의 공통 처리. 로봇 동작 로직은 기존 ROS 코드를 재사용한다."""

import argparse
import fcntl
from datetime import datetime
import importlib.util
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import signal
import socket
import sqlite3
import subprocess
import sys
import threading
import time
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
POSE_SERVICE = "/dsr01/dsr_controller2/aux_control/get_current_posx"
# ROS 노드 파라미터 파일. 드로잉 기준 자세(cake_center_pose)를 여기서 읽는다.
WORKSPACE_CONFIG = ROOT / "ws_cobot1/src/cake_decorator/config/workspace_config.yaml"
# 실물 케이크 상면 위치·YES 펜촉 TCP 설정 파일(사용자가 실측값을 관리).
REAL_ROBOT_CONFIG = ROOT / "config/real_robot.json"


def configure_discovery(mode, env):
    """ROS 초기화 전에 이번 실행의 노드 검색 환경만 정한다."""
    # 드라이버·TM·DP·RE·브릿지는 같은 PC에 있다. 실제 로봇과의 연결은
    # 드라이버의 TCP 통신이므로 외부 DDS 검색 서버를 필요로 하지 않는다.
    # 기존 sodreal 드라이버나 다른 PC의 ROS 노드를 재사용할 때만 별도로
    # 옵션을 지정해 기존 검색 서버 설정을 유지한다.
    if mode == "real" and env.get("CAKE_USE_DISCOVERY_SERVER") == "1":
        return
    env.pop("ROS_DISCOVERY_SERVER", None)
    env.pop("ROS_SUPER_CLIENT", None)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"


def configure_side_test(args):
    """가상 옆면 검증을 도메인187과 localhost로 제한한다."""
    if args.side_pen_45 and args.mode == "virtual":
        if args.domain_id not in (None, 187):
            raise ValueError("가상 옆면 검증은 ROS_DOMAIN_ID=187 전용입니다.")
        if args.host not in (None, "127.0.0.1"):
            raise ValueError("가상 옆면 검증은 localhost 에뮬레이터만 사용합니다.")
        args.domain_id = 187


def validate_identity(description, mode, host):
    """조회한 모델의 모드·주소가 실행 대상과 일치할 때만 다음 단계로 진행한다."""
    robot = ET.fromstring(description)
    params = {
        p.get("name"): p.text for p in robot.findall(".//ros2_control/hardware/param")
    }
    if params.get("mode") != mode or params.get("host") != host:
        raise RuntimeError(f"로봇 대상 불일치: 요청={mode}/{host}, 조회={params}")


def pen_length(value):
    """그리퍼 끝부터 펜촉까지 돌출 길이를 cm로 검사한다."""
    length = float(value)
    if not math.isfinite(length) or not 0 < length <= 20:
        raise ValueError("펜 돌출 길이는 0보다 크고 20cm 이하여야 합니다.")
    return length


def require_free_port(port):
    """이미 켜진 서버를 강제로 끄지 않고 중복 실행을 거부한다."""
    with socket.socket() as connection:
        connection.settimeout(0.2)
        if connection.connect_ex(("127.0.0.1", port)) == 0:
            raise RuntimeError(
                f"{port} 포트가 사용 중입니다. 기존 서버를 끄거나 포트 옵션을 바꾸세요."
            )


def validate_existing_nodes(nodes, reuse_driver):
    """다른 업무 노드와 겹치면 주문을 두 번 실행할 수 있으므로 중복을 거부한다."""
    names = {name for name, namespace in nodes if namespace in ("/", "")}
    application = names & {
        "ros_bridge_node",
        "task_manager_node",
        "drawing_path_node",
        "robot_executor_node",
    }
    if application:
        raise RuntimeError(f"업무 노드가 이미 켜져 있습니다: {sorted(application)}")
    if ("robot_state_publisher", "/dsr01") in nodes and not reuse_driver:
        raise RuntimeError(
            "로봇 드라이버가 이미 켜져 있습니다. --reuse-driver로 기존 연결을 사용하세요."
        )


def resolve_pending_orders(database, backend_port, bridge_token, answer=input):
    """중단된 주문을 운영자가 확인한 경우에만 기존 내부 API로 해제한다.

    주문 기록과 결과는 보존한다. 자동 재실행·DB 삭제·로봇 정지 명령은 없다.
    브릿지를 시작하기 전이므로 이전 주문을 새 영점에서 실행하지 않는다.
    """
    with sqlite3.connect(f"file:{Path(database).resolve()}?mode=ro", uri=True) as connection:
        pending = connection.execute(
            "SELECT order_id,state FROM order_execution "
            "WHERE state NOT IN ('completed','failed','rejected','resolved')"
        ).fetchall()
    if not pending:
        return
    print(f"이전 미완료 주문: {pending}", flush=True)
    if any(state not in {"uncertain", "claimed", "dispatching", "running"} for _, state in pending):
        raise RuntimeError("미접수·미실행 주문이 남아 있습니다. docs/BRIDGE_TEST.md를 확인하세요.")
    print("로봇 작업이 멈췄고 이전 주문을 다시 실행하지 않을 때만 RESET을 입력하세요.", flush=True)
    if answer("이전 작업 중단 확인 후 RESET (그 외 종료): ").strip() != "RESET":
        raise RuntimeError("이전 주문을 보존하고 실행을 취소했습니다.")
    for order_id, _ in pending:
        payload = json.dumps(dict(
            confirmed_not_running=True,
            reason="전체 실행기에서 운영자가 작업 중단을 확인하고 RESET 입력",
        )).encode()
        request = Request(
            f"http://127.0.0.1:{backend_port}/internal/bridge/orders/{order_id}/resolve",
            data=payload, method="POST",
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {bridge_token}"},
        )
        with urlopen(request, timeout=10) as response:
            result = json.load(response)
        if result.get("status") != "resolved":
            raise RuntimeError(f"이전 주문 해제를 확인할 수 없습니다: {order_id}")
    print("이전 주문을 해제했습니다. 기록은 보존되며 자동 재실행하지 않습니다.", flush=True)


def prepare_reference(
    session, mode, answer=input, *, calibrate_pen_tcp=False,
    cake_top_z_mm=None, calibration_log=None, no_prompt=False,
):
    """가상은 자동 배치, 실제는 YES 직후 조회한 현재 TCP를 영점으로 기록한다.

    no_prompt=True는 주문을 넣지 않는 미리보기 전용이다. YES 입력 없이 현재
    TCP를 그대로 기록하므로 이 기준점으로 실제 주문을 실행하면 안 된다.
    """
    if mode == "virtual":
        return session.prepare_virtual()
    if no_prompt:
        print(
            "[경고] 미리보기 모드: 주문을 보내지 않습니다. "
            "YES 확인 없이 현재 TCP를 기준점으로 기록합니다.",
            flush=True,
        )
        print(
            "이 기준점은 케이크 상면 중앙이 아닐 수 있으므로 실제 주문에 사용하지 마세요.",
            flush=True,
        )
        return session.pose()
    if calibrate_pen_tcp:
        print(f"펜을 수직으로 세워 상면 중앙에 닿게 두세요. 상면 BASE Z={cake_top_z_mm}mm")
        print("YES 후 현재 TCP의 XY가 펜촉과 같다고 가정해 별도 테스트 TCP를 등록·적용합니다.")
        print("설정 적용으로 좌표 표시가 바뀝니다. 실제 펜촉 고정 품질은 별도 확인이 필요합니다.")
    else:
        print("펜촉 TCP를 교정하고 케이크 상면 중앙에 가볍게 닿게 두세요.")
    print("YES 후 기록한 위치를 기준으로 웹 제작 요청이 실제 로봇을 움직입니다.")
    if (
        answer("케이크 상면 중앙·TCP 확인 후 YES (그 외 종료): ").strip().upper()
        != "YES"
    ):
        raise RuntimeError("영점 확인을 취소했습니다.")
    # TCP 추정은 명시적으로 선택한 real 테스트에서만 사용한다. 이동 명령은 없다.
    if calibrate_pen_tcp:
        from contact_tcp import apply_contact_tcp

        return apply_contact_tcp(session, cake_top_z_mm, calibration_log)
    return session.pose()


def prepare_side_pen_reference(session, reference, answer=input, *, skip_confirm=False):
    """설정한 중앙으로 이동하고 선택적으로 확인을 기다린 뒤 위로 이탈한다."""
    from dsr_msgs2.srv import (
        GetRobotMode,
        SetRobotMode,
        SetCurrentTcp,
        GetCurrentTcp,
        GetCurrentToolFlangePosx,
    )
    from scipy.spatial.transform import Rotation
    from cake_decorator.bent_pen import (
        SIDE_TCP_NAME,
        SIDE_TCP_OFFSET_MM,
        VERTICAL_GRIPPER_ZYZ,
    )
    from cake_decorator.doosan_robot import DoosanRobotClient
    from cake_decorator.drawing_motion import DsrPos
    from cake_decorator.goal_mapping import validate_reference
    from contact_tcp import relative_tcp

    prefix = "/dsr01/dsr_controller2/"
    # 펜 거치대 위 현재 위치와 케이크 중심을 절대로 혼용하지 않는다.
    # 접근 목표는 명시한 설정값에서만 가져온다.
    reference = validate_reference(reference)
    mode = session.call(
        GetRobotMode,
        prefix + "system/get_robot_mode",
        GetRobotMode.Request(),
    ).robot_mode
    if mode not in (0, 1):
        raise RuntimeError("TCP 설정이 가능한 수동 또는 자동 모드로 준비하세요.")

    # 위치를 맞추기 전에 실제 교정한 펜촉 TCP를 적용한다.
    # TCP 설정은 좌표 기준을 바꾸며, 관절 이동 명령을 보내지 않는다.
    try:
        if mode != 0:
            session.call(
                SetRobotMode,
                prefix + "system/set_robot_mode",
                SetRobotMode.Request(robot_mode=0),
            )
        session.call(
            SetCurrentTcp,
            prefix + "tcp/set_current_tcp",
            SetCurrentTcp.Request(name=SIDE_TCP_NAME),
        )
    finally:
        if mode != 0:
            session.call(
                SetRobotMode,
                prefix + "system/set_robot_mode",
                SetRobotMode.Request(robot_mode=int(mode)),
            )

    # 자동 이동 전에 펜촉 TCP 이름과 교정값을 대조한다.
    selected = session.call(
        GetCurrentTcp,
        prefix + "tcp/get_current_tcp",
        GetCurrentTcp.Request(),
    ).info
    current = session.pose()
    flange = session.call(
        GetCurrentToolFlangePosx,
        prefix + "aux_control/get_current_tool_flange_posx",
        GetCurrentToolFlangePosx.Request(ref=0),
    ).pos
    measured = relative_tcp(list(flange), current)
    orientation_error = Rotation.from_euler(
        "ZYZ", measured[3:], degrees=True,
    ).magnitude()
    if (
        selected != SIDE_TCP_NAME
        or math.dist(measured[:3], SIDE_TCP_OFFSET_MM) > 1.0
        or orientation_error > math.radians(1.0)
    ):
        raise RuntimeError("Pen_Side_45의 실제 교정값을 확인하세요.")

    robot = DoosanRobotClient(session.node)

    def move_checked(target, label, speed_mm_s=30.0):
        """모든 접근·이탈 단계에서 성공 응답과 실제 도달을 확인한다."""
        latest_mode = session.call(
            GetRobotMode,
            prefix + "system/get_robot_mode",
            GetRobotMode.Request(),
        ).robot_mode
        if latest_mode not in (0, 1):
            raise RuntimeError("접근 전 로봇 모드를 확인하세요.")
        if latest_mode == 0:
            session.call(
                SetRobotMode,
                prefix + "system/set_robot_mode",
                SetRobotMode.Request(robot_mode=1),
            )
        print(f"{label}: 펜촉 XYZ [mm] {target[:3]}", flush=True)
        if not robot.move_line(
            DsrPos(*target),
            speed_mm_s=speed_mm_s,
            acc_mm_s2=200.0,
            angular_speed_deg_s=30.0,
            angular_acc_deg_s2=60.0,
        ) or not robot.verify_pose(target):
            raise RuntimeError(f"{label} 실패. 현재 로봇 상태를 확인하세요.")

    # 실제 파지 후 자세를 유지하고, 거치대에서 먼저 위로 빠져나온다.
    # 케이크 상공은 검증한 상면 +70mm로 접근하며 접촉 중 회전하지 않는다.
    raised = list(current)
    raised[2] = max(current[2] + 50.0, reference[2] + 70.0)
    above = [*reference[:2], reference[2] + 70.0, *current[3:]]
    contact = [*reference[:3], *current[3:]]
    print(f"사용 TCP {SIDE_TCP_NAME}. 설정된 케이크 상면 중심으로 이동합니다.")
    move_checked(raised, "거치대 위로 이탈", speed_mm_s=100.0)
    move_checked(above, "케이크 중심 상공으로 접근", speed_mm_s=100.0)
    move_checked(contact, "케이크 상면 중앙 확인 위치로 하강")

    if skip_confirm:
        # 검증된 설정 중심을 사용하며, 접촉 위치에서 사용자 입력을 기다리지 않는다.
        print("설정한 케이크 중앙 기록 후 바로 위로 50mm 올립니다.")
    else:
        print("펜촉이 실제 상면 중앙에 가볍게 닿는지 확인하세요.")
        print("위치가 다르면 펜던트로 보정한 뒤 YES를 입력하세요.")
        confirmed = answer("케이크 중앙 확인 후 YES (그 외 종료): ")
        if confirmed.strip().upper() != "YES":
            raise RuntimeError("옆면 영점 확인을 취소했습니다.")

    # 확인 중 수동으로 보정했다면 보정한 좌표를 이번 주문의 중심으로 쓴다.
    selected = session.call(
        GetCurrentTcp,
        prefix + "tcp/get_current_tcp",
        GetCurrentTcp.Request(),
    ).info
    if selected != SIDE_TCP_NAME:
        raise RuntimeError("중앙 확인 중 TCP가 변경됐습니다. 다시 실행하세요.")
    current = session.pose()
    pose = validate_reference(current)
    pose[3:] = VERTICAL_GRIPPER_ZYZ
    print(f"확인한 케이크 상면 중심 XYZ [mm] {pose[:3]}", flush=True)
    lifted = list(current)
    lifted[2] += 50.0
    move_checked(lifted, "중앙 확인 후 펜 이탈", speed_mm_s=60.0)

    # 브릿지는 접촉 영점 대신 펜을 뗀 시작 위치가 유지되는지 확인한다.
    return list(pose), session.pose()


def prepare_virtual_side_pen(session):
    """가상 모델 초기화 뒤 교정한 45도 펜 TCP와 파지 완료 관절 자세를 준비한다."""
    from dsr_msgs2.srv import ConfigCreateTcp, GetCurrentTcp, SetCurrentTcp, SetRobotMode
    from cake_decorator.bent_pen import SIDE_TCP_NAME, SIDE_TCP_OFFSET_MM
    from cake_decorator.doosan_robot import DoosanRobotClient

    # 도메인 분리 외에도 드라이버 description으로 실제 연결 대상을 확인한다.
    session.identity("virtual", "127.0.0.1")
    prefix = "/dsr01/dsr_controller2/"
    deadline = time.monotonic() + 60
    while session.call(GetCurrentTcp, prefix + "tcp/get_current_tcp", GetCurrentTcp.Request()).info != "Virtual_Pen_TCP":
        if time.monotonic() > deadline:
            raise RuntimeError("가상 펜 모델 초기화를 확인하지 못했습니다.")
        time.sleep(.2)
    try:
        session.call(SetRobotMode, prefix + "system/set_robot_mode", SetRobotMode.Request(robot_mode=0))
        session.call(ConfigCreateTcp, prefix + "tcp/config_create_tcp", ConfigCreateTcp.Request(
            name=SIDE_TCP_NAME, pos=[*SIDE_TCP_OFFSET_MM, 0., 0., 0.],
        ))
        session.call(SetCurrentTcp, prefix + "tcp/set_current_tcp", SetCurrentTcp.Request(name=SIDE_TCP_NAME))
    finally:
        session.call(SetRobotMode, prefix + "system/set_robot_mode", SetRobotMode.Request(robot_mode=1))
    robot = DoosanRobotClient(session.node)
    try:
        if not robot.move_joint(
            [43.59, .54, 69.87, .09, 109.62, 43.31],
            speed_deg_s=30., acc_deg_s2=60.,
        ):
            raise RuntimeError("가상 45도 파지 완료 자세로 이동하지 못했습니다.")
    finally:
        for client in robot._clients.values():
            session.node.destroy_client(client)


def restore_side_tcp_after_exit(args, env, folder):
    """업무 노드를 회수한 뒤 별도 ROS 프로세스로 TCP를 초기화한다.

    Ctrl+C로 부모의 ROS context가 이미 닫혔더라도 복원을 시도할 수 있다.
    드라이버는 기존 sodreal 연결을 재사용하므로 계속 살아 있다.
    """
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--mode", args.mode,
        "--host", args.host,
        "--reset-side-tcp",
    ]
    try:
        completed = subprocess.run(
            command,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=25,
        )
        (folder / "tcp_restore.log").write_text(completed.stdout)
        print(completed.stdout, end="", flush=True)
        if completed.returncode != 0:
            raise RuntimeError("기본 TCP 초기화를 확인하지 못했습니다. tcp_restore.log를 확인하세요.")
    except subprocess.TimeoutExpired as error:
        raise RuntimeError("기본 TCP 초기화 응답이 없습니다. 펜던트에서 TCP를 확인하세요.") from error


def load_real_robot_config(config_path=REAL_ROBOT_CONFIG):
    """config/real_robot.json의 실물 설정을 읽고 검증한다. 파일만 읽고 부수 효과는 없다.

    키(단위 mm)
      - cake_top_z_mm: 케이크 상면의 BASE Z. 펜촉 TCP 기준이므로 펜촉이 닿는 높이다.
      - configure_pen_tcp_on_yes: start_real.sh 단위 테스트에서 YES 때 펜촉 TCP를
        새로 등록할지 여부. 전체 공정(--cake-center-from-config)에서는 쓰지 않는다.
      - side_cake_center_xy_mm: [x, y] 케이크 상면 중심의 BASE XY.
        단위 테스트만 하는 설정에는 없을 수 있어 여기서는 필수로 검사하지 않는다.
    """
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    z = float(config["cake_top_z_mm"])
    if not math.isfinite(z) or not 0 <= z <= 900:
        raise RuntimeError("config/real_robot.json의 cake_top_z_mm는 BASE 기준 0~900mm여야 합니다.")
    if not isinstance(config.get("configure_pen_tcp_on_yes", False), bool):
        raise RuntimeError("config/real_robot.json의 configure_pen_tcp_on_yes는 true/false여야 합니다.")
    return config


def load_config_cake_reference(
    config_path=REAL_ROBOT_CONFIG, workspace_path=WORKSPACE_CONFIG,
):
    """케이크 상면 중심을 드로잉 기준점(BASE mm·ZYZ deg 6값)으로 만든다.

    호출 시점: 실제 모드에서 --cake-center-from-config를 지정했을 때 YES 영점 대신 호출한다.
    XY: real_robot.json의 top_cake_center_xy_mm. 없으면 기존 side_cake_center_xy_mm를 사용한다.
    Z : workspace_config.yaml의 robot_executor_node.cake_center_pose Z.
      펜을 쥐고 케이크 중심에 펜촉을 댈 때 티칭한 TCP 높이다. 펜을 내리는 깊이를
      기존 '케이크 중심 하강'과 같게 하라는 요청에 따라 이 값을 접촉 높이로 쓴다.
      real_robot.json의 cake_top_z_mm(실측 상면 높이)는 현재 TCP 기준 값과 달라
      전체 공정의 드로잉 높이로 쓰지 않는다. goal_mapping.build_goal도 Z에 높이를 더하지 않는다.
    자세: 같은 cake_center_pose의 자세 3값(실제로 도달한 펜 자세).
    반환: [x, y, z, a, b, c].
    """
    import yaml

    config = load_real_robot_config(config_path)
    xy = config.get("top_cake_center_xy_mm", config.get("side_cake_center_xy_mm"))
    if not isinstance(xy, (list, tuple)) or len(xy) != 2:
        raise RuntimeError("config/real_robot.json에 side_cake_center_xy_mm [x, y]가 필요합니다.")
    with open(workspace_path, encoding="utf-8") as file:
        workspace = yaml.safe_load(file) or {}
    executor = workspace.get("robot_executor_node", {}).get("ros__parameters", {})
    center_pose = executor.get("cake_center_pose")
    if not isinstance(center_pose, (list, tuple)) or len(center_pose) != 6:
        raise RuntimeError("workspace_config.yaml에 robot_executor_node.cake_center_pose 6값이 필요합니다.")
    reference = [float(xy[0]), float(xy[1]), *map(float, center_pose[2:])]
    if not all(math.isfinite(value) for value in reference):
        raise RuntimeError("케이크 기준 좌표에 유한하지 않은 값이 있습니다.")
    # 작업 영역 밖 값은 오타일 가능성이 높으므로 로봇을 움직이기 전에 거부한다.
    if math.hypot(reference[0], reference[1]) > 900 or not 0 <= reference[2] <= 900:
        raise RuntimeError(f"케이크 기준 좌표가 작업 영역 밖입니다: {reference[:3]}")
    return reference


def load_side_cake_reference(config_path=REAL_ROBOT_CONFIG):
    """파일만 읽어 SIDE 펜촉의 BASE mm·ZYZ deg 기준점을 검증한다.

    TOP 티칭 TCP 높이와 구분해 실측 상면 높이를 사용한다. 통합 실행과
    --check에서 호출하며 로봇 조회·TCP 변경·이동은 하지 않는다.
    """
    from cake_decorator.bent_pen import VERTICAL_GRIPPER_ZYZ

    config = load_real_robot_config(config_path)
    xy = config.get("side_cake_center_xy_mm")
    if not isinstance(xy, (list, tuple)) or len(xy) != 2:
        raise RuntimeError("side_cake_center_xy_mm는 X, Y 두 값이어야 합니다.")
    reference = [
        *map(float, xy), float(config["cake_top_z_mm"]), *VERTICAL_GRIPPER_ZYZ,
    ]
    if not all(math.isfinite(value) for value in reference):
        raise RuntimeError("SIDE 기준 좌표에 유한하지 않은 값이 있습니다.")
    if math.hypot(*reference[:2]) > 900:
        raise RuntimeError(f"SIDE 기준 좌표가 작업 영역 밖입니다: {reference[:3]}")
    return reference


def load_runtime_parameters(args, workspace_path=WORKSPACE_CONFIG):
    """독립 옆면 시험 또는 통합 공정의 실행용 파라미터를 메모리에 만든다.

    원본 yaml은 보존한다. 통합 모드는 DP의 옆면 경로만 45도 펜으로 만들고
    RE가 주문 표면별로 동작을 선택하게 해 TOP의 기존 실행을 유지한다.
    """
    import yaml

    runtime = yaml.safe_load(Path(workspace_path).read_text(encoding="utf-8"))
    dp = runtime["drawing_path_node"]["ros__parameters"]
    executor = runtime["robot_executor_node"]["ros__parameters"]
    if args.integrated_side_cycle:
        # 지름 유격은 현장 yaml 값을 그대로 쓴다. 누락·오타는 기동 전에 알린다.
        allowance = float(dp["side_diameter_allowance_mm"])
        if not math.isfinite(allowance) or allowance < 0:
            raise RuntimeError("side_diameter_allowance_mm는 0 이상의 유한한 값이어야 합니다.")
    dp.update(
        side_pen_45=True,
        side_drawing_speed=40.0,
        travel_speed=180.0,
        side_y_inset_mm=1.0 if args.integrated_side_cycle else args.side_y_inset_mm,
    )
    if args.integrated_side_cycle:
        dp["integrated_side_cycle"] = True
        executor.update(integrated_side_cycle=True, side_pen_45=False)
    else:
        executor["side_pen_45"] = True
        if args.mode == "virtual":
            # 가상 독립 검증은 이미 펜을 잡은 상태이므로 반납하지 않는다.
            executor["side_return_pen"] = False
    return runtime


class RobotSession:
    """ROS 조회와 가상 초기 배치를 담당한다. 노드는 별도 스레드에서 응답을 처리한다."""

    def __init__(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        self.ros = rclpy
        rclpy.init()
        self.node = rclpy.create_node("cake_system_launcher")
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()

    def call(self, kind, path, request, timeout=60):
        """응답을 확인하지 못한 요청은 재전송하지 않고 실행을 중단한다."""
        client = self.node.create_client(kind, path)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"서비스 준비 시간 초과: {path}")
            future = client.call_async(request)
            ready = threading.Event()
            future.add_done_callback(lambda _: ready.set())
            if not ready.wait(10):
                raise RuntimeError(f"서비스 응답 없음: {path}")
            result = future.result()
            if result is None or not getattr(result, "success", True):
                raise RuntimeError(f"서비스 실패: {path}")
            return result
        finally:
            self.node.destroy_client(client)

    def identity(self, mode, host):
        """드라이버가 사용하는 robot_description을 읽어 모드 전환 실수를 막는다."""
        from rcl_interfaces.srv import GetParameters

        result = self.call(
            GetParameters,
            "/dsr01/robot_state_publisher/get_parameters",
            GetParameters.Request(names=["robot_description"]),
        )
        validate_identity(result.values[0].string_value, mode, host)

    def pose(self):
        """현재 활성 펜촉 TCP를 BASE mm·ZYZ degree로 읽는다."""
        from dsr_msgs2.srv import GetCurrentPosx
        from cake_decorator.goal_mapping import validate_reference

        result = self.call(GetCurrentPosx, POSE_SERVICE, GetCurrentPosx.Request(ref=0))
        if not result.task_pos_info:
            raise RuntimeError("현재 TCP 위치가 없습니다.")
        return validate_reference(list(result.task_pos_info[0].data[:6]))

    def prepare_virtual(self):
        """가상 펜 초기화가 끝난 뒤에만 홈→상면 위→상면 중심으로 이동한다."""
        from dsr_msgs2.srv import (
            GetCurrentTcp,
            GetRobotMode,
            GetSolutionSpace,
            MoveJointx,
            Fkin,
        )
        from cake_decorator.doosan_robot import DoosanRobotClient
        import yaml

        # 이 함수 자체에서도 모드를 검사해 실제 대상에서는 이동할 수 없게 한다.
        self.identity("virtual", "127.0.0.1")
        deadline = time.monotonic() + 75
        while True:
            tcp = self.call(
                GetCurrentTcp,
                "/dsr01/dsr_controller2/tcp/get_current_tcp",
                GetCurrentTcp.Request(),
            )
            mode = self.call(
                GetRobotMode,
                "/dsr01/dsr_controller2/system/get_robot_mode",
                GetRobotMode.Request(),
            )
            if tcp.info == "Virtual_Pen_TCP" and mode.robot_mode == 1:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("가상 펜 TCP 초기화가 끝나지 않았습니다.")
            time.sleep(0.2)
        fixture = yaml.safe_load(
            (
                ROOT / "ws_cobot1/src/cake_workcell_bringup/config/workcell.yaml"
            ).read_text()
        )["fixture"]
        cake = json.loads((ROOT / "config/test_cake.json").read_text())
        angle = math.radians(fixture["yaw_deg"])
        x = fixture["x_mm"] + 85 * math.cos(angle) - 85 * math.sin(angle)
        y = fixture["y_mm"] + 85 * math.sin(angle) + 85 * math.cos(angle)
        z = fixture["z_mm"] + 62 + cake["height_mm"]
        robot = DoosanRobotClient(self.node)
        home = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        home_pose = self.call(
            Fkin, "/dsr01/dsr_controller2/motion/fkin", Fkin.Request(pos=home, ref=0)
        )
        if home_pose.conv_posx[2] < 10:
            raise RuntimeError(
                "이 펜 길이는 기본 홈 자세에서 상판과 간섭합니다. 더 짧은 돌출 길이를 사용하세요."
            )
        if not robot.move_joint(home, speed_deg_s=20.0):
            raise RuntimeError("가상 홈 이동 실패")
        # 초기 홈에서 상면 위까지는 관절 경로를 쓴다. 직선 접근은 중간에
        # 손목 특이점을 통과하며 관절 속도 한계에 걸릴 수 있다.
        solution = self.call(
            GetSolutionSpace,
            "/dsr01/dsr_controller2/aux_control/get_solution_space",
            GetSolutionSpace.Request(pos=home),
        )
        self.call(
            MoveJointx,
            "/dsr01/dsr_controller2/motion/move_jointx",
            MoveJointx.Request(
                pos=[x, y, z + 50, 0.0, 180.0, 0.0],
                vel=20.0,
                acc=40.0,
                sol=solution.sol_space,
            ),
        )
        if math.dist(self.pose()[:3], [x, y, z + 50]) > 0.1:
            raise RuntimeError("가상 상면 위 접근점에 도달하지 않았습니다.")
        # 초기 영점 배치의 끝 구간도 관절 경로로 연결한다. 이 동작은 가상
        # 초기화에만 쓰며, 사용자 그림의 경로는 기존 DP·RE가 실행한다.
        self.call(
            MoveJointx,
            "/dsr01/dsr_controller2/motion/move_jointx",
            MoveJointx.Request(
                pos=[x, y, z, 0.0, 180.0, 0.0],
                vel=10.0,
                acc=20.0,
                sol=solution.sol_space,
            ),
        )
        pose = self.pose()
        if math.dist(pose[:3], [x, y, z]) > 0.1:
            raise RuntimeError(f"가상 펜촉이 중심에 도달하지 않았습니다: {pose}")
        return pose

    def wait_controller(self):
        """드라이버가 실제로 연결돼 TCP 조회 서비스를 열 때까지 기다린다."""
        from dsr_msgs2.srv import GetCurrentPosx

        client = self.node.create_client(GetCurrentPosx, POSE_SERVICE)
        try:
            if not client.wait_for_service(timeout_sec=60):
                raise RuntimeError(
                    "로봇 연결이 준비되지 않았습니다. IP·LAN과 driver.log를 확인하세요."
                )
        finally:
            self.node.destroy_client(client)

    def ready_application(self):
        """TM·DP·RE가 연결됐는지만 검사하며 주문 Goal은 보내지 않는다."""
        from rclpy.action import ActionClient
        from cake_decorator_interfaces.action import CakeOrder, DrawOnCake
        from cake_decorator_interfaces.srv import GenerateTrajectory

        for kind, name in ((CakeOrder, "/cake/order"), (DrawOnCake, "/draw_on_cake")):
            client = ActionClient(self.node, kind, name)
            try:
                if not client.wait_for_server(timeout_sec=20):
                    raise RuntimeError(f"Action 준비 시간 초과: {name}")
            finally:
                client.destroy()
        client = self.node.create_client(GenerateTrajectory, "/generate_trajectory")
        try:
            if not client.wait_for_service(timeout_sec=20):
                raise RuntimeError("DP 준비 시간 초과")
        finally:
            self.node.destroy_client(client)

    def close(self):
        """조회 노드만 정리한다. 실제 로봇을 홈으로 보내거나 그리퍼를 움직이지 않는다."""
        self.executor.shutdown()
        self.thread.join()
        self.node.destroy_node()
        self.ros.try_shutdown()


def bridge_worker(args):
    """확인한 영점으로 기존 브릿지를 실행한다. 경로·주문 처리 구현은 재사용한다."""
    from cake_decorator.backend_client import BackendClient
    from cake_decorator.ros_bridge_node import RosBridgeNode

    reference = json.loads(Path(args.bridge_reference).read_text())
    if reference["mode"] != args.mode:
        raise RuntimeError("영점 파일과 실행 모드가 다릅니다.")
    session = RobotSession()
    try:
        session.identity(args.mode, args.host)
        # 설정 기준점은 현재 TCP와 무관하다. 로봇 위치 조회·이동 검사는
        # YES 영점과 독립 옆면 시험의 기록된 시작 자세에만 적용한다.
        if reference.get("source") != "config":
            observed = session.pose()
            recorded = reference.get("start_pose", reference["pose"])
            # 독립 옆면 시험은 접촉 후 이탈한 start_pose를 검사한다.
            distance = math.dist(observed[:3], recorded[:3])
            moved = distance >= 0.5 if "start_pose" in reference else distance > 0.1
            if moved:
                raise RuntimeError(
                    f"영점 기록 후 TCP 위치 차이 {distance:.3f}mm: "
                    f"기록={recorded[:3]}, 관측={observed[:3]}. 실행기를 다시 시작하세요."
                )
    finally:
        session.close()
    import rclpy

    rclpy.init()
    node = RosBridgeNode(
        BackendClient(
            f"http://127.0.0.1:{args.backend_port}", os.environ["CAKE_BRIDGE_TOKEN"]
        ),
        reference["pose"],
        result_timeout=1800.0,
    )
    try:
        node.start()
        Path(args.bridge_reference + ".ready").touch()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


class Processes:
    """이 실행기가 시작한 프로세스 그룹만 추적하고 Ctrl+C 시 역순으로 종료한다."""

    def __init__(self, folder, env):
        self.folder, self.env = folder, env.copy()
        # 파이프로 바뀐 Python 자식도 print를 종료 시점까지 버퍼링하지 않는다.
        self.env["PYTHONUNBUFFERED"] = "1"
        self.children = []
        self._readers = []
        self._output_lock = threading.Lock()

    def _tee(self, name, process, log):
        """자식의 합쳐진 출력을 원문 파일과 접두어를 붙인 터미널로 전달한다."""
        try:
            for line in iter(process.stdout.readline, b""):
                # 파일은 원래 바이트를 보존하고 매 줄 flush하여 실행 중에도 읽게 한다.
                log.write(line)
                log.flush()
                with self._output_lock:
                    try:
                        sys.stdout.write(
                            f"[{name}] " + line.decode("utf-8", errors="replace")
                        )
                        sys.stdout.flush()
                    except (OSError, ValueError):
                        # 터미널이 닫혀도 pipe를 계속 비워 자식과 파일 기록을 유지한다.
                        pass
        finally:
            process.stdout.close()
            log.close()

    def start(self, name, command, cwd=ROOT):
        """소유 그룹으로 자식을 시작하고 로그 자원의 수명을 reader에 넘긴다."""
        log = open(self.folder / f"{name}.log", "wb")
        try:
            process = subprocess.Popen(
                command,
                cwd=cwd,
                env=self.env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except BaseException:
            log.close()
            raise
        self.children.append((name, process))
        reader = threading.Thread(
            target=self._tee,
            args=(name, process, log),
            name=f"log-{name}",
        )
        try:
            reader.start()
        except BaseException:
            # 스레드를 만들지 못한 자식도 pipe가 차서 멈추기 전에 회수한다.
            self._signal_group(process.pid, signal.SIGKILL)
            process.wait()
            process.stdout.close()
            log.close()
            raise
        self._readers.append(reader)
        print(f"시작: {name}", flush=True)

    def check(self):
        for name, process in self.children:
            if process.poll() is not None:
                raise RuntimeError(
                    f"{name} 종료. 로그 확인: {self.folder / (name + '.log')}"
                )

    def wait(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check()
            if condition():
                return
            time.sleep(0.2)
        raise RuntimeError("준비 시간 초과. 실행 로그를 확인하세요.")

    @staticmethod
    def _signal_group(group, signum):
        """리더가 먼저 끝났더라도 같은 그룹의 하위 노드까지 종료한다."""
        try:
            os.killpg(group, signum)
        except ProcessLookupError:
            pass

    def _wait_groups(self, timeout, children=None):
        """종료한 직접 자식은 회수하고, 남은 하위 프로세스 그룹을 기다린다."""
        children = self.children if children is None else children
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            alive = False
            for _, process in children:
                process.poll()
                try:
                    os.killpg(process.pid, 0)
                    alive = True
                except ProcessLookupError:
                    pass
            if not alive:
                return True
            time.sleep(0.05)
        return False

    def close(self, grace=3.0, terminate_grace=2.0, before_driver_close=None):
        """정리 중 추가 Ctrl+C는 무시하고 소유 그룹을 끝까지 회수한다."""
        # 두 번째 Ctrl+C가 finally를 끊으면 ros2 부모만 끝나고 노드가 남는다.
        # 신호 변경은 메인 스레드에서만 허용되므로 다른 스레드에서는 생략한다.
        main_thread = threading.current_thread() is threading.main_thread()
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN) if main_thread else None
        try:
            if before_driver_close is None:
                self._close_groups(grace, terminate_grace)
            else:
                application = [(name, process) for name, process in self.children if name != "driver"]
                driver = [(name, process) for name, process in self.children if name == "driver"]
                self._close_groups(grace, terminate_grace, application)
                try:
                    # 가상 드라이버가 살아 있을 때 TCP를 초기화한다.
                    before_driver_close()
                finally:
                    self._close_groups(grace, terminate_grace, driver)
        finally:
            if main_thread:
                signal.signal(signal.SIGINT, previous)

    def _close_groups(self, grace, terminate_grace, children=None):
        """정상 종료를 기다린 뒤 응답하지 않는 자식 그룹에 TERM·KILL을 보낸다."""
        children = self.children if children is None else children
        # 그룹 ID는 start_new_session으로 만든 이 실행기 소유의 값만 사용한다.
        # 부모 ros2가 먼저 종료돼도 남은 실제 ROS 노드가 빠지지 않도록 한다.
        for _, process in reversed(children):
            self._signal_group(process.pid, signal.SIGINT)
        if not self._wait_groups(grace, children):
            for _, process in reversed(children):
                self._signal_group(process.pid, signal.SIGTERM)
            if not self._wait_groups(terminate_grace, children):
                for _, process in reversed(children):
                    self._signal_group(process.pid, signal.SIGKILL)
        for _, process in children:
            process.wait()
        # 그룹 종료 후 EOF까지 남은 로그를 비우고 pipe·파일 해제를 기다린다.
        names = {f"log-{name}" for name, _ in children}
        for reader in self._readers:
            if reader.name in names:
                reader.join()


def healthy(port):
    """백엔드가 요청을 받을 수 있는지 확인한다. ROS 연결 상태를 대신 판정하지 않는다."""
    try:
        with urlopen(f"http://127.0.0.1:{port}/api/health", timeout=1) as response:
            return response.status == 200
    except OSError:
        return False


def main():
    """의존성 확인 → 드라이버 → 영점 → 웹·업무 노드 → 브릿지 순서로 시작한다."""
    parser = argparse.ArgumentParser(
        description="웹부터 로봇까지 함께 실행합니다. Ctrl+C로 종료합니다."
    )
    parser.add_argument("--mode", choices=("virtual", "real"), required=True)
    parser.add_argument(
        "--pen-extension-cm",
        type=pen_length,
        help="가상 펜 돌출 길이(cm), 생략하면 입력, 기본 4",
    )
    parser.add_argument("--host", help="실제 로봇 IP, 기본 192.168.1.100")
    parser.add_argument("--frontend-port", type=int, default=8080)
    parser.add_argument("--backend-port", type=int, default=5000)
    parser.add_argument("--domain-id", type=int, help="기본값은 ROS_DOMAIN_ID 또는 30")
    parser.add_argument(
        "--reuse-driver",
        action="store_true",
        help="이미 켠 sodreal/가상 작업대의 드라이버 사용",
    )
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument(
        "--calibrate-pen-tcp", action="store_true",
        help="real에서 YES 때 수직 접촉 펜촉을 추정해 테스트 TCP 등록·적용",
    )
    parser.add_argument(
        "--cake-top-z-mm", type=float,
        help="실제 케이크 상면의 BASE 기준 Z(mm). 케이크 자체 높이가 아님",
    )
    parser.add_argument(
        "--preview-no-order",
        action="store_true",
        help=(
            "real 미리보기: YES 확인 없이 현재 TCP를 기록하고 주문 없이 전체 화면만 띄움. "
            "이 기준점으로 실제 주문을 실행하지 말 것"
        ),
    )
    parser.add_argument(
        "--cake-center-from-config",
        action="store_true",
        help=(
            "real: YES 영점 대신 config/real_robot.json의 side_cake_center_xy_mm·"
            "yaml cake_center_pose를 드로잉 기준점으로 사용(YES·펜촉 TCP 등록 없음)"
        ),
    )
    parser.add_argument(
        "--integrated-side-cycle",
        action="store_true",
        help="real: 설정 기준점으로 TOP/SIDE 전체 공정 실행(기동 중앙 이동·YES 없음)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="준비 상태만 검사하고 아무것도 실행하지 않음",
    )
    parser.add_argument("--side-pen-45", action="store_true", help="교정한 45도 파지 펜으로 옆면 웹 검증")
    parser.add_argument(
        "--side-y-inset-mm",
        type=float,
        default=0.0,
        help="옆면 BASE Y 양쪽을 각각 안으로 줄이는 거리(mm), 0~5",
    )
    parser.add_argument("--skip-center-confirm", action="store_true", help="옆면 중앙 기록 후 YES 없이 바로 이탈")
    parser.add_argument("--reset-side-tcp", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--bridge-reference", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not math.isfinite(args.side_y_inset_mm) or not 0 <= args.side_y_inset_mm <= 5:
        parser.error("--side-y-inset-mm는 0~5mm여야 합니다.")
    configure_side_test(args)
    if args.side_pen_45 and args.mode == "real" and not args.reuse_driver:
        parser.error("옆면 검증은 sodreal을 먼저 켜고 --reuse-driver로 실행하세요.")
    if args.integrated_side_cycle:
        # 전체 공정은 펜을 집기 전이므로 중앙 접근·YES·TCP 교정을 하지 않는다.
        if args.mode != "real" or args.side_pen_45 or args.reset_side_tcp:
            parser.error("--integrated-side-cycle는 real 전체 공정 전용입니다.")
        args.cake_center_from_config = True
        os.environ["CAKE_PEN_TOOL_CYCLE"] = "1"
        os.environ["CAKE_PLATE_SEQUENCE"] = "1"
    if args.side_pen_45 and (
        args.cake_center_from_config or args.calibrate_pen_tcp or args.preview_no_order
    ):
        parser.error("--side-pen-45는 설정 기준점·TCP 교정·미리보기와 함께 쓰지 않습니다.")
    real_config = None
    if args.side_pen_45 or args.integrated_side_cycle:
        try:
            real_config = load_real_robot_config()
        except (OSError, KeyError, ValueError, RuntimeError) as error:
            parser.error(f"config/real_robot.json을 읽지 못했습니다: {error}")
        if args.side_pen_45:
            args.cake_top_z_mm = float(real_config["cake_top_z_mm"])
    # 직접 실행하는 실물 단위 테스트(YES 영점)에서는 real_robot.json 설정대로
    # YES 직후 펜촉 TCP를 등록한다. 명령줄 옵션을 직접 준 경우에는 그 값을 우선한다.
    # 다음 경우에는 TCP를 바꾸지 않는다.
    #   - 설정 파일 기준점(전체 공정): YES 자체가 없다.
    #   - 펜 순환·플레이트 시퀀스: pen_*·plate_* 직선 좌표는 현재 TCP로 티칭했으므로
    #     TCP가 바뀌면 목표가 펜 길이만큼 어긋난다.
    #   - 미리보기: 주문을 넣지 않는 점검 전용이다.
    if (
        args.mode == "real"
        and not args.calibrate_pen_tcp
        and args.cake_top_z_mm is None
        and not args.cake_center_from_config
        and not args.preview_no_order
        and not args.bridge_reference
        and not args.reset_side_tcp
        and not args.side_pen_45
        and os.environ.get("CAKE_PEN_TOOL_CYCLE") != "1"
        and os.environ.get("CAKE_PLATE_SEQUENCE") != "1"
    ):
        try:
            real_config = load_real_robot_config()
        except (OSError, KeyError, ValueError, RuntimeError) as error:
            parser.error(f"config/real_robot.json을 읽지 못했습니다: {error}")
        if real_config.get("configure_pen_tcp_on_yes", False):
            args.calibrate_pen_tcp = True
            args.cake_top_z_mm = float(real_config["cake_top_z_mm"])
    if args.calibrate_pen_tcp:
        if args.mode != "real" or args.cake_top_z_mm is None:
            parser.error("펜촉 TCP 추정은 real 모드에서 --cake-top-z-mm와 함께 사용하세요.")
        if not math.isfinite(args.cake_top_z_mm) or not 0 <= args.cake_top_z_mm <= 900:
            parser.error("상면 BASE Z는 0~900mm의 유한한 값이어야 합니다.")
    elif args.cake_top_z_mm is not None and not args.side_pen_45:
        parser.error("--cake-top-z-mm는 --calibrate-pen-tcp와 함께 사용하세요.")
    if args.preview_no_order:
        if args.mode != "real":
            parser.error("--preview-no-order는 real 모드에서만 사용하세요.")
        if args.calibrate_pen_tcp:
            parser.error(
                "--preview-no-order와 --calibrate-pen-tcp는 함께 사용할 수 없습니다."
            )
    # 설정 파일 기준점은 YES·TCP 추정·미리보기와 기준점을 정하는 방식이 겹치므로 함께 쓰지 않는다.
    if args.cake_center_from_config and (
        args.mode != "real" or args.calibrate_pen_tcp or args.preview_no_order
    ):
        parser.error(
            "--cake-center-from-config는 real 모드에서 "
            "--calibrate-pen-tcp·--preview-no-order 없이 사용하세요."
        )
    entry_mode = os.environ.get("CAKE_SYSTEM_MODE")
    if entry_mode and args.mode != entry_mode:
        parser.error(
            "Bash 실행기의 모드는 바꿀 수 없습니다. 해당 모드의 실행 파일을 사용하세요."
        )
    if args.mode == "virtual" and args.host not in (None, "127.0.0.1"):
        parser.error("가상 모드 주소는 127.0.0.1로 고정합니다.")
    args.host = args.host or (
        "127.0.0.1" if args.mode == "virtual" else "192.168.1.100"
    )
    if args.domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    # 별도 실행되는 브릿지 프로세스도 동일한 검색 정책을 적용한다.
    configure_discovery(args.mode, os.environ)
    if args.reset_side_tcp:
        if args.check:
            parser.error("--check는 TCP 복원을 실행하지 않습니다.")
        from cake_decorator.tcp_scope import reset_default_tcp

        cleanup_session = RobotSession()
        try:
            cleanup_session.identity(args.mode, args.host)
            reset_default_tcp(
                cleanup_session.call, stop_running=True,
                flange_euler="ZYX" if args.mode == "virtual" else "ZYZ",
            )
            print("기본 TCP 복원 완료: 선택 없음, 0,0,0,0,0,0", flush=True)
        finally:
            cleanup_session.close()
        return
    if args.bridge_reference:
        bridge_worker(args)
        return
    print(
        f"ROS 검색: {os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '기존 설정')}, "
        f"DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}, "
        f"외부 검색 서버={'사용' if os.environ.get('ROS_DISCOVERY_SERVER') else '미사용'}",
        flush=True,
    )
    if args.frontend_port == args.backend_port or not all(
        1 <= p <= 65535 for p in (args.frontend_port, args.backend_port)
    ):
        parser.error("프론트·백 포트는 서로 다른 1~65535 값이어야 합니다.")
    for port in (args.frontend_port, args.backend_port):
        require_free_port(port)
    node_bin = Path.home() / ".local/share/cake-web-tooling/node_modules/.bin/node"
    node = str(node_bin) if node_bin.exists() else shutil.which("node")
    vite = ROOT / "frontend/node_modules/vite/bin/vite.js"
    backend_python = ROOT / "backend/.venv/bin/python"
    if not node or not vite.exists() or not backend_python.exists():
        raise RuntimeError("프론트 npm ci와 백엔드 가상환경 설치를 먼저 완료하세요.")
    version = subprocess.check_output([node, "--version"], text=True).strip()
    if tuple(int(v) for v in version.lstrip("v").split(".")[:2]) < (22, 12):
        raise RuntimeError("Node.js 22.12 이상이 필요합니다.")
    for module in ("rclpy", "dsr_msgs2", "cake_decorator", "cake_decorator_interfaces"):
        if importlib.util.find_spec(module) is None:
            raise RuntimeError(
                f"ROS 패키지가 없습니다: {module}. 빌드와 source를 확인하세요."
            )
    from cake_decorator_interfaces.srv import GenerateTrajectory

    if "strokes_2d" not in GenerateTrajectory.Request.get_fields_and_field_types():
        raise RuntimeError(
            "이전 인터페이스가 로드됐습니다. ws_cobot1을 다시 빌드하세요."
        )
    if args.mode == "real" and not args.reuse_driver:
        if importlib.util.find_spec("pymodbus") is None:
            raise RuntimeError(
                "실제 RG2 드라이버에 필요한 pymodbus가 없습니다. 교육장 ROS Python 환경에 설치한 뒤 실행하세요."
            )
    if args.mode == "virtual" and not args.reuse_driver:
        running = subprocess.check_output(
            [
                "docker",
                "ps",
                "--filter",
                "name=^/dsr01_emulator$",
                "--format",
                "{{.ID}}",
            ],
            text=True,
        ).strip()
        if running:
            raise RuntimeError(
                "가상 에뮬레이터가 이미 켜져 있습니다. 종료하거나 --reuse-driver를 사용하세요."
            )
    # 기준 좌표가 빠졌거나 잘못됐으면 드라이버를 켜기 전에 알린다(--check 포함).
    config_reference = (
        load_config_cake_reference() if args.cake_center_from_config else None
    )
    if config_reference is not None:
        print(f"real_robot.json 케이크 기준점 [mm, deg]: {config_reference}", flush=True)
    # 기준점·경로 설정 검사는 ROS 세션 생성보다 앞에 둔다. --check는
    # 두 표면의 파일 설정을 검증한 뒤 드라이버·업무 노드를 시작하지 않고 끝난다.
    side_reference = load_side_cake_reference() if args.integrated_side_cycle else None
    runtime_parameters = (
        load_runtime_parameters(args)
        if args.side_pen_45 or args.integrated_side_cycle
        else None
    )
    if side_reference is not None:
        print(f"SIDE 실측 기준점 [mm, deg]: {side_reference}", flush=True)
        dp = runtime_parameters["drawing_path_node"]["ros__parameters"]
        print(
            "통합 공정: 펜 순환·플레이트 사용, SIDE 속도 40mm/s, 공중 120mm/s, "
            f"Y inset 1mm, 지름 유격 {dp['side_diameter_allowance_mm']}mm",
            flush=True,
        )
    if args.check:
        print(
            f"준비 검사 통과: {args.mode}, ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '30')}"
        )
        return
    if (
        args.mode == "virtual"
        and args.pen_extension_cm is None
        and not args.reuse_driver
    ):
        args.pen_extension_cm = pen_length(
            input("가상 펜 돌출 길이(cm) [기본 4]: ").strip() or "4"
        )
    folder = (
        ROOT
        / "log/system"
        / f"{datetime.now():%Y%m%d-%H%M%S}-{args.mode}-{os.getpid()}"
    )
    folder.mkdir(parents=True)
    env = os.environ.copy()
    env["CAKE_BRIDGE_TOKEN"] = env.get("CAKE_BRIDGE_TOKEN") or secrets.token_urlsafe(32)
    if args.side_pen_45 and args.mode == "virtual":
        # 상속된 실물 DB 환경변수가 있어도 가상 주문은 별도 DB에만 저장한다.
        database = str(folder / "orders.sqlite3")
    else:
        database = env.get("CAKE_DATABASE_PATH") or str(
            folder / "orders.sqlite3"
            if args.mode == "virtual"
            else ROOT / "backend/data/orders.sqlite3"
        )
    env["CAKE_DATABASE_PATH"] = database
    env["API_PROXY_TARGET"] = f"http://127.0.0.1:{args.backend_port}"
    # 다른 포트로 동시에 실행해도 같은 로봇 컨트롤러를 두 번 띄우지 않는다.
    lock_name = "run-virtual-side-187.lock" if args.side_pen_45 and args.mode == "virtual" else "run.lock"
    lock = open(ROOT / "log/system" / lock_name, "a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError(
            "다른 전체 실행기가 이미 켜져 있습니다. 기존 실행기를 종료하세요."
        )
    processes = Processes(folder, env)
    session = None
    side_tcp_started = False

    def request_shutdown(_signal, _frame):
        """터미널 종료 신호도 Ctrl+C와 같은 정리 경로로 보낸다."""
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_shutdown)
    print(f"실행 로그: {folder}", flush=True)
    try:
        session = RobotSession()
        # 동일 네임스페이스의 드라이버를 두 번 띄우지 않는다.
        time.sleep(1)
        validate_existing_nodes(
            session.node.get_node_names_and_namespaces(), args.reuse_driver
        )
        if not args.reuse_driver:
            if args.mode == "virtual":
                command = [
                    "ros2",
                    "launch",
                    "cake_workcell_bringup",
                    "virtual.launch.py",
                    f"pen_extension_cm:={args.pen_extension_cm}",
                ]
                if args.side_pen_45:
                    center_xy = real_config["side_cake_center_xy_mm"]
                    command += [
                        "side_pen_45:=true",
                        f"side_center_x:={float(center_xy[0]) / 1000}",
                        f"side_center_y:={float(center_xy[1]) / 1000}",
                        f"side_top_z:={args.cake_top_z_mm / 1000}",
                    ]
            else:
                command = [
                    "ros2",
                    "launch",
                    "m0609_rg2_bringup",
                    "bringup.launch.py",
                    "mode:=real",
                    f"host:={args.host}",
                    "port:=12345",
                    "model:=m0609",
                ]
            processes.start("driver", command)
        session.identity(args.mode, args.host)
        session.wait_controller()
        # 통합 공정에서도 SIDE TCP가 남지 않게 종료 시 복원한다. 드라이버의
        # 대상·연결 확인 전 실패하거나 --check만 실행하면 복원을 시도하지 않는다.
        side_tcp_started = args.integrated_side_cycle
        processes.start(
            "backend",
            [
                str(backend_python),
                "-m",
                "uvicorn",
                "app.main:app",
                "--host",
                "127.0.0.1",
                "--port",
                str(args.backend_port),
            ],
            ROOT / "backend",
        )
        processes.wait(lambda: healthy(args.backend_port), 20)
        resolve_pending_orders(database, args.backend_port, env["CAKE_BRIDGE_TOKEN"])
        start_pose = None
        reference_source = "tcp"
        # 전체 공정은 TOP 티칭 자세를 파일에서 읽는다. SIDE의 물리적 상면
        # 기준점은 TM에 따로 전달하며 여기서 TOP 높이를 바꾸지 않는다.
        if config_reference is not None:
            reference = config_reference
            reference_source = "config"
        elif args.side_pen_45:
            # 고정 중심은 설정에서 전달하고 최신 로그나 현재 위치로 추정하지 않는다.
            center_xy = real_config["side_cake_center_xy_mm"]
            if (
                len(center_xy) != 2
                or not all(math.isfinite(float(value)) for value in center_xy)
            ):
                raise ValueError("side_cake_center_xy_mm는 유효한 X, Y 두 값이어야 합니다.")
            from cake_decorator.bent_pen import VERTICAL_GRIPPER_ZYZ

            # 적용 도중 오류가 나거나 YES 대기 중 종료돼도 복원을 실행한다.
            side_tcp_started = True
            if args.mode == "virtual":
                prepare_virtual_side_pen(session)
            reference, start_pose = prepare_side_pen_reference(
                session,
                [*map(float, center_xy), args.cake_top_z_mm, *VERTICAL_GRIPPER_ZYZ],
                skip_confirm=args.skip_center_confirm,
            )
        else:
            reference = prepare_reference(
                session, args.mode,
                calibrate_pen_tcp=args.calibrate_pen_tcp,
                cake_top_z_mm=args.cake_top_z_mm,
                calibration_log=folder / "contact_tcp.json",
                no_prompt=args.preview_no_order,
            )
        print(f"상면 중심 TCP [mm, deg]: {reference}", flush=True)
        reference_file = folder / "reference.json"
        # 브릿지는 설정 기준점에서 TCP 위치 검사를 생략한다. 독립 옆면 시험은
        # 기존처럼 이탈 후 start_pose를 별도로 보존해 시작 위치를 확인한다.
        record = {"mode": args.mode, "pose": reference, "source": reference_source}
        if start_pose is not None:
            record["start_pose"] = start_pose
            record["side_pen_45"] = True
        reference_file.write_text(json.dumps(record))
        config = str(WORKSPACE_CONFIG)
        if runtime_parameters is not None:
            import yaml

            filename = (
                "integrated_side_cycle.yaml"
                if args.integrated_side_cycle
                else "side_pen_45.yaml"
            )
            config = str(folder / filename)
            Path(config).write_text(
                yaml.safe_dump(runtime_parameters, allow_unicode=True), encoding="utf-8",
            )
        for name in ("drawing_path_node", "robot_executor_node", "task_manager_node"):
            command = ["ros2", "run", "cake_decorator", name, "--ros-args"]
            command += (
                ["-p", "action_timeout:=1800.0"]
                if name == "task_manager_node"
                else ["--params-file", config]
            )
            if name == "robot_executor_node" and args.mode == "virtual" and not args.side_pen_45:
                command += ["-p", "virtual_initial_approach:=true"]
            # 실물 전체 공정과 명시적 펜 순환 실행에서 집기·반납을 켠다.
            if (
                name == "robot_executor_node"
                and os.environ.get("CAKE_PEN_TOOL_CYCLE") == "1"
            ):
                command += ["-p", "pen_tool_cycle:=true"]
            # 실물 전체 공정은 드로잉 앞뒤 플레이트 준비·후처리를 켠다.
            # 실행 노드는 서비스를 제공하고 작업 관리 노드는 주문마다 호출하므로
            # 두 노드에 같은 값을 넘겨야 한다. task_manager_node는 yaml을
            # 읽지 않으므로 -p로 직접 전달한다.
            if (
                name in ("robot_executor_node", "task_manager_node")
                and os.environ.get("CAKE_PLATE_SEQUENCE") == "1"
            ):
                command += ["-p", "plate_sequence_enabled:=true"]
            if name == "task_manager_node" and args.integrated_side_cycle:
                # TM은 yaml을 읽지 않는다. SIDE의 실측 펜촉 기준점을 따로
                # 넘겨 TOP 티칭 자세와 주문 표면에 맞게 구분하도록 한다.
                command += [
                    "-p", "integrated_side_cycle:=true",
                    "-p", f"side_reference_pose:={json.dumps(side_reference)}",
                ]
            processes.start(name, command)
        session.ready_application()
        processes.start(
            "bridge",
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--mode",
                args.mode,
                "--host",
                args.host,
                "--backend-port",
                str(args.backend_port),
                "--bridge-reference",
                str(reference_file),
            ],
        )
        processes.wait(lambda: Path(str(reference_file) + ".ready").exists(), 20)
        processes.start(
            "frontend",
            [
                node,
                str(vite),
                "--host",
                "0.0.0.0",
                "--port",
                str(args.frontend_port),
                "--strictPort",
            ],
            ROOT / "frontend",
        )
        processes.wait(lambda: healthy(args.frontend_port), 20)
        print(
            f"준비 완료: http://localhost:{args.frontend_port} / HMI: /hmi", flush=True
        )
        print(
            "태블릿은 같은 Wi-Fi의 노트북 주소와 위 포트로 접속하세요. 종료: Ctrl+C",
            flush=True,
        )
        if (
            not args.no_browser
            and os.environ.get("DISPLAY")
            and shutil.which("xdg-open")
        ):
            subprocess.Popen(
                ["xdg-open", f"http://localhost:{args.frontend_port}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        session.close()
        session = None
        while True:
            processes.check()
            time.sleep(0.5)
    finally:
        # 정리 순서는 자식 프로세스 회수를 우선한다. ROS 조회 노드 종료가
        # 실패하거나 추가 Ctrl+C가 와도 다른 프로세스가 남지 않게 한다.
        def restore_tcp():
            try:
                restore_side_tcp_after_exit(args, env, folder)
            except (RuntimeError, OSError) as error:
                print(f"기본 TCP 복원 실패: {error}", file=sys.stderr, flush=True)

        try:
            processes.close(before_driver_close=restore_tcp if side_tcp_started else None)
        finally:
            try:
                if session:
                    session.close()
            finally:
                lock.close()
        print("실행기가 시작한 서비스들을 종료했습니다.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, OSError) as error:
        print(f"실행 중단: {error}", file=sys.stderr)
        raise SystemExit(1)
