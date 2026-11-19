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
        if math.dist(session.pose()[:3], reference["pose"][:3]) > 0.1:
            raise RuntimeError(
                "영점 기록 후 TCP가 이동했습니다. 실행기를 다시 시작하세요."
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

    def _wait_groups(self, timeout):
        """종료한 직접 자식은 회수하고, 남은 하위 프로세스 그룹을 기다린다."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            alive = False
            for _, process in self.children:
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

    def close(self, grace=3.0, terminate_grace=2.0):
        """정리 중 추가 Ctrl+C는 무시하고 소유 그룹을 끝까지 회수한다."""
        # 두 번째 Ctrl+C가 finally를 끊으면 ros2 부모만 끝나고 노드가 남는다.
        # 신호 변경은 메인 스레드에서만 허용되므로 다른 스레드에서는 생략한다.
        main_thread = threading.current_thread() is threading.main_thread()
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN) if main_thread else None
        try:
            self._close_groups(grace, terminate_grace)
        finally:
            if main_thread:
                signal.signal(signal.SIGINT, previous)

    def _close_groups(self, grace, terminate_grace):
        """정상 종료를 기다린 뒤 응답하지 않는 자식 그룹에 TERM·KILL을 보낸다."""
        # 그룹 ID는 start_new_session으로 만든 이 실행기 소유의 값만 사용한다.
        # 부모 ros2가 먼저 종료돼도 남은 실제 ROS 노드가 빠지지 않도록 한다.
        for _, process in reversed(self.children):
            self._signal_group(process.pid, signal.SIGINT)
        if not self._wait_groups(grace):
            for _, process in reversed(self.children):
                self._signal_group(process.pid, signal.SIGTERM)
            if not self._wait_groups(terminate_grace):
                for _, process in reversed(self.children):
                    self._signal_group(process.pid, signal.SIGKILL)
        for _, process in self.children:
            process.wait()
        # 그룹 종료 후 EOF까지 남은 로그를 비우고 pipe·파일 해제를 기다린다.
        for reader in self._readers:
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
        "--check",
        action="store_true",
        help="준비 상태만 검사하고 아무것도 실행하지 않음",
    )
    parser.add_argument("--bridge-reference", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.calibrate_pen_tcp:
        if args.mode != "real" or args.cake_top_z_mm is None:
            parser.error("펜촉 TCP 추정은 real 모드에서 --cake-top-z-mm와 함께 사용하세요.")
        if not math.isfinite(args.cake_top_z_mm) or not 0 <= args.cake_top_z_mm <= 900:
            parser.error("상면 BASE Z는 0~900mm의 유한한 값이어야 합니다.")
    elif args.cake_top_z_mm is not None:
        parser.error("--cake-top-z-mm는 --calibrate-pen-tcp와 함께 사용하세요.")
    if args.preview_no_order:
        if args.mode != "real":
            parser.error("--preview-no-order는 real 모드에서만 사용하세요.")
        if args.calibrate_pen_tcp:
            parser.error(
                "--preview-no-order와 --calibrate-pen-tcp는 함께 사용할 수 없습니다."
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
    database = env.get("CAKE_DATABASE_PATH") or str(
        folder / "orders.sqlite3"
        if args.mode == "virtual"
        else ROOT / "backend/data/orders.sqlite3"
    )
    env["CAKE_DATABASE_PATH"] = database
    env["API_PROXY_TARGET"] = f"http://127.0.0.1:{args.backend_port}"
    # 다른 포트로 동시에 실행해도 같은 로봇 컨트롤러를 두 번 띄우지 않는다.
    lock = open(ROOT / "log/system/run.lock", "a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError(
            "다른 전체 실행기가 이미 켜져 있습니다. 기존 실행기를 종료하세요."
        )
    processes = Processes(folder, env)
    session = None

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
        reference = prepare_reference(
            session, args.mode,
            calibrate_pen_tcp=args.calibrate_pen_tcp,
            cake_top_z_mm=args.cake_top_z_mm,
            calibration_log=folder / "contact_tcp.json",
            no_prompt=args.preview_no_order,
        )
        print(f"상면 중심 TCP [mm, deg]: {reference}", flush=True)
        reference_file = folder / "reference.json"
        reference_file.write_text(json.dumps({"mode": args.mode, "pose": reference}))
        config = str(ROOT / "ws_cobot1/src/cake_decorator/config/workspace_config.yaml")
        for name in ("drawing_path_node", "robot_executor_node", "task_manager_node"):
            command = ["ros2", "run", "cake_decorator", name, "--ros-args"]
            command += (
                ["-p", "action_timeout:=1800.0"]
                if name == "task_manager_node"
                else ["--params-file", config]
            )
            if name == "robot_executor_node" and args.mode == "virtual":
                command += ["-p", "virtual_initial_approach:=true"]
            # start_real_pen_cycle.sh로 실행한 경우에만 드로잉 전후 펜 집기·반납을 켠다.
            # 기존 start_real.sh·start_virtual.sh의 동작은 바뀌지 않는다.
            if (
                name == "robot_executor_node"
                and os.environ.get("CAKE_PEN_TOOL_CYCLE") == "1"
            ):
                command += ["-p", "pen_tool_cycle:=true"]
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
        try:
            processes.close()
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
