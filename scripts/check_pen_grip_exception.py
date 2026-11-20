#!/usr/bin/env python3
"""펜 파지 안전예외만 확인하는 단독 점검 스크립트.

기존 펜 좌표(workspace_config.yaml)를 그대로 사용해 아래만 수행한다.
  A: pen_stand_approach_pose 도착 후 그리퍼 폭 < 60mm → 예외
  B: pen_stand_pose에서 닫고 pen_gripper_pose로 들어올린 뒤 폭 < 30mm → 예외
예외 시 그리퍼를 열고 plate_safe_position_1로 이동한 뒤 종료한다.
성공 시 펜을 원래 거치대에 반납하고 종료한다(기본값).

드로잉, 웹, 태스크 매니저, 기존 펜 사이클 실행기는 사용하지 않는다.
"""

import argparse
from pathlib import Path
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml

from dsr_msgs2.srv import GetRobotState


ROOT = Path(__file__).resolve().parent.parent
SOURCE_PACKAGE_ROOT = ROOT / "ws_cobot1/src/cake_decorator"
DEFAULT_CONFIG = (
    ROOT / "ws_cobot1/src/cake_decorator/config/workspace_config.yaml"
)
STATE_SERVICE = "/dsr01/dsr_controller2/system/get_robot_state"
STATE_STANDBY = 1

# 펜 안전예외 기준(m). 실물 로그를 보고 조정한다.
PEN_OPEN_MIN_WIDTH_M = 0.060
PEN_HOLD_MIN_WIDTH_M = 0.030

sys.path.insert(0, str(SOURCE_PACKAGE_ROOT))

from cake_decorator.doosan_robot import DoosanRobotClient  # noqa: E402
from cake_decorator.drawing_motion import DsrPos  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="펜 파지 안전예외(A 열림/B 보유)만 확인",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--timeout", type=float, default=120.0, help="서비스 응답 제한(초)"
    )
    parser.add_argument(
        "--keep-pen",
        action="store_true",
        help="성공 후 펜을 반납하지 않고 그리퍼에 둔다.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="이동 없이 좌표와 기준만 출력한다.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="실제 로봇 이동을 승인한다.",
    )
    return parser.parse_args()


def load_params(config_path: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    return (
        document.get("robot_executor_node", {}).get("ros__parameters", {})
    )


def call_state(client, timeout: float):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(GetRobotState.Request())
    ready = threading.Event()
    future.add_done_callback(lambda _: ready.set())
    if not ready.wait(timeout):
        return None
    return future.result()


def read_width(robot, retries: int = 20):
    """조인트 상태 수신 전이면 잠시 기다렸다가 폭을 읽는다."""
    for _ in range(retries):
        width = robot.get_gripper_width()
        if width is not None:
            return width
        time.sleep(0.1)
    return None


def print_width(label: str, width_m):
    if width_m is None:
        print(f"{label}: 폭 측정 실패")
    else:
        print(f"{label}: 폭={width_m * 1000.0:.1f}mm")


def safe_stop(robot, safe_pose, open_width):
    """예외 시 그리퍼를 열고 plate_safe_position_1로 이동한다."""
    print("안전 정지: 그리퍼 열기 → Safe Position 이동")
    robot.gripper_open(open_width)
    if not robot.move_joint(safe_pose, speed_deg_s=20.0, acc_deg_s2=20.0):
        print("Safe Position 이동 실패", file=sys.stderr)


def return_pen(robot, params):
    """성공 후 기존 펜 반납 좌표로 펜을 돌려놓는다."""
    approach = params["pen_drop_approach_pose"]
    drop = params["pen_drop_pose"]
    after = params["pen_drop_after_pose"]
    open_width = float(params["gripper_open_width"])
    print("펜 반납: 상공 이동")
    if not robot.move_joint(approach, speed_deg_s=20.0, acc_deg_s2=20.0):
        return False
    print("펜 반납: 하강")
    if not robot.move_line(DsrPos(*drop), speed_mm_s=30.0):
        return False
    if not robot.gripper_open(open_width):
        return False
    print("펜 반납: 상공 후퇴")
    return robot.move_line(DsrPos(*after), speed_mm_s=30.0)


def main():
    args = parse_args()
    params = load_params(args.config)

    approach = params["pen_stand_approach_pose"]
    stand = params["pen_stand_pose"]
    gripper_up = params["pen_gripper_pose"]
    safe_pose = params["plate_safe_position_1"]
    open_width = float(params["gripper_open_width"])
    grip_width = float(params["pen_grip_width"])
    grip_force = float(params["pen_grip_force"])

    print("펜 파지 안전예외 단독 점검")
    print(f"  A 기준: {PEN_OPEN_MIN_WIDTH_M * 1000.0:.0f}mm 이상")
    print(f"  B 기준: {PEN_HOLD_MIN_WIDTH_M * 1000.0:.0f}mm 이상")
    print(f"  상공 좌표: {approach}")
    print(f"  파지 좌표: {stand}")
    print(f"  Safe 좌표: {safe_pose}")

    if args.dry_run or not args.yes:
        print("\n이동하지 않았습니다. 실제 점검은 --yes가 필요합니다.")
        return 0

    rclpy.init()
    node = Node("pen_grip_exception_check")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        state_client = node.create_client(GetRobotState, STATE_SERVICE)
        state = call_state(state_client, args.timeout)
        if state is None or not state.success:
            print("로봇 상태를 읽지 못했습니다.", file=sys.stderr)
            return 2
        if int(state.robot_state) != STATE_STANDBY:
            print(
                f"STANDBY(1)가 아닙니다: {state.robot_state}",
                file=sys.stderr,
            )
            return 2

        robot = DoosanRobotClient(
            node,
            service_timeout=float(params.get("service_timeout", 5.0)),
            gripper_open_do=int(params.get("gripper_open_do", 2)),
            gripper_close_do=int(params.get("gripper_close_do", 1)),
            gripper_pulse_sec=float(params.get("gripper_pulse_sec", 0.5)),
            gripper_virtual_fallback=bool(
                params.get("gripper_virtual_fallback", True)
            ),
        )

        print("\n[A] 펜 상공 이동")
        if not robot.move_joint(
            approach, speed_deg_s=20.0, acc_deg_s2=20.0
        ):
            print("펜 상공 이동 실패", file=sys.stderr)
            return 3
        width_a = read_width(robot)
        print_width("A 그리퍼 폭", width_a)
        if width_a is None or width_a < PEN_OPEN_MIN_WIDTH_M:
            print("PEN_GRASP_FAILED: 그리퍼 열림 감지", file=sys.stderr)
            safe_stop(robot, safe_pose, open_width)
            return 3

        print("\n[B] 펜 위치 하강")
        if not robot.move_line(DsrPos(*stand), speed_mm_s=30.0):
            print("펜 하강 실패", file=sys.stderr)
            return 3

        print("[B] 펜 닫기")
        if not robot.gripper_close(grip_width, grip_force):
            print("펜 그리퍼 닫기 실패", file=sys.stderr)
            return 3

        print("[B] 수직 상공으로 들어올리기")
        if not robot.move_line(DsrPos(*gripper_up), speed_mm_s=30.0):
            print("펜 들어올리기 실패", file=sys.stderr)
            return 3

        width_b = read_width(robot)
        print_width("B 그리퍼 폭", width_b)
        if width_b is None or width_b < PEN_HOLD_MIN_WIDTH_M:
            print("PEN_GRASP_FAILED: 파지 보유 확인 실패", file=sys.stderr)
            safe_stop(robot, safe_pose, open_width)
            return 4

        print("\nPEN_GRASP_OK: 펜 파지 성공")
        if not args.keep_pen:
            if not return_pen(robot, params):
                print("펜 반납 실패", file=sys.stderr)
                return 5
            print("펜 반납 완료")
        return 0
    finally:
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
