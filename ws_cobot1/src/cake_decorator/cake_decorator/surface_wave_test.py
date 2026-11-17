"""가상 전용 파형 검증. 실제 장비 모드에서는 이동 요청을 거부한다."""

import argparse
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Point32, PoseStamped, Point
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray
from cake_decorator_interfaces.msg import Stroke2D
from cake_decorator_interfaces.srv import GenerateTrajectory
from dsr_msgs2.srv import GetCurrentTcp, GetRobotMode

from .drawing_path_node import DrawingPathNode
from .drawing_motion import DsrPos, Phase, plan_drawing_motion
from .robot_executor_node import RobotExecutorNode
from .doosan_robot import DoosanRobotClient


class WaveVerifier(Node):
    """관측한 펜촉 위치를 기록하고 가상 컨트롤러에서만 계산된 명령을 실행한다."""

    def __init__(self):
        super().__init__("surface_wave_verifier")
        self.robot = DoosanRobotClient(self)
        self.pose_subscription = self.create_subscription(
            PoseStamped, "/cake/observed_tip", self.observe, 100
        )
        self.planned = self.create_publisher(
            MarkerArray,
            "/cake/wave_plan",
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.clear = self.create_publisher(Empty, "/cake/ink_clear", 1)
        self.samples = []
        self.record = False
        self.last_stamp = None
        self.latest_tip = None
        self.latest_tip_stamp_ns = None

    def observe(self, message):
        """명령점이 아니라 실제 TF를 읽은 시각·자세를 저장한다."""
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        p, q = message.pose.position, message.pose.orientation
        self.latest_tip = (p.x * 1000, p.y * 1000, p.z * 1000)
        self.latest_tip_stamp_ns = stamp[0] * 1_000_000_000 + stamp[1]
        if not self.record or stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        self.samples.append([p.x * 1000, p.y * 1000, p.z * 1000, q.x, q.y, q.z, q.w])

    def verify_virtual(self):
        """동작 전에 실제 RSP의 하드웨어 설정이 localhost 가상 모드인지 검사한다."""
        client = self.create_client(
            GetParameters, "/dsr01/robot_state_publisher/get_parameters"
        )
        try:
            if not client.wait_for_service(timeout_sec=60):
                raise RuntimeError("가상 로봇 RSP가 없습니다.")
            done = threading.Event()
            future = client.call_async(
                GetParameters.Request(names=["robot_description"])
            )
            future.add_done_callback(lambda _: done.set())
            if not done.wait(10) or future.result() is None:
                raise RuntimeError("가상 모드 확인 응답이 없습니다.")
            description = ET.fromstring(future.result().values[0].string_value)
            params = {
                p.get("name"): p.text
                for p in description.findall(".//ros2_control/hardware/param")
            }
            if params.get("mode") != "virtual" or params.get("host") != "127.0.0.1":
                raise RuntimeError(
                    "localhost virtual 모드가 아니므로 이동하지 않습니다."
                )
            if description.find("link[@name='virtual_pen_tip']") is None:
                raise RuntimeError("관측할 가상 펜촉 모델이 없습니다.")
        finally:
            self.destroy_client(client)

    def wait_pen_ready(self):
        """TCP 등록과 자동 모드 복원이 끝나기 전에는 홈 이동도 보내지 않는다."""
        clients = [
            self.create_client(
                GetCurrentTcp, "/dsr01/dsr_controller2/tcp/get_current_tcp"
            ),
            self.create_client(
                GetRobotMode, "/dsr01/dsr_controller2/system/get_robot_mode"
            ),
        ]
        deadline = time.monotonic() + 75
        try:
            while time.monotonic() < deadline:
                values = []
                for client, request in zip(
                    clients, (GetCurrentTcp.Request(), GetRobotMode.Request())
                ):
                    if not client.wait_for_service(timeout_sec=1):
                        break
                    ready = threading.Event()
                    future = client.call_async(request)
                    future.add_done_callback(lambda _, event=ready: event.set())
                    if not ready.wait(3) or future.result() is None:
                        break
                    values.append(future.result())
                if (
                    len(values) == 2
                    and values[0].success
                    and values[0].info == "Virtual_Pen_TCP"
                    and values[1].robot_mode == 1
                ):
                    # 그리퍼 초기화의 마지막 응답까지 기다려 초기화와 이동을 분리한다.
                    time.sleep(1)
                    return
                time.sleep(0.2)
            raise RuntimeError(
                "가상 펜 TCP/자동 모드 초기화가 끝나지 않았습니다. 이동하지 않습니다."
            )
        finally:
            for client in clients:
                self.destroy_client(client)

    def wait_observed_target(self, target, after_ns, timeout_sec=4.0):
        """서비스 응답 이후에도 TF가 종점에 도달할 때까지 실제 관측을 받는다.

        서비스 반환과 joint_states→TF 발행 시각은 같지 않다. 이 확인 없이
        다음 명령/수집 종료로 넘어가면 정상적으로 지난 획 끝이 통계에서 빠진다.
        """
        deadline = time.monotonic() + timeout_sec
        expected = (target.x, target.y, target.z)
        while time.monotonic() < deadline:
            stamp = self.latest_tip_stamp_ns
            now_ns = self.get_clock().now().nanoseconds
            if (
                self.latest_tip is not None
                and stamp is not None
                and stamp > after_ns
                and 0 <= now_ns - stamp <= 250_000_000
                and math.dist(self.latest_tip, expected) <= 0.05
            ):
                return
            time.sleep(0.02)
        raise RuntimeError(
            f"관측 TCP가 목표점에 도달하지 않았습니다: {expected}, observed={self.latest_tip}"
        )

    def lift_to_start(self, previous):
        """직전 옆면 밖 위치에서 먼저 수직 상승한 뒤 상면 위로 이동한다."""
        if previous is not None:
            above = DsrPos(
                previous.x, previous.y, 180.0, previous.rz1, previous.ry, previous.rz2
            )
            if not self.robot.move_line(above, speed_mm_s=30.0):
                raise RuntimeError("가상 펜 이탈 실패")
            # 다음 DP가 상면 위 접근점을 준다. 중간에 수직 TCP로 되돌리면
            # 손목 특이점에서 6축 회전이 누적될 수 있으므로 옆면 자세를 유지한다.
            return
        if not self.robot.move_line(
            DsrPos(500.0, 0.0, 180.0, 0.0, 180.0, 0.0), speed_mm_s=30.0
        ):
            raise RuntimeError("가상 상면 위 접근 실패")


def wave_request(amplitude, wavelength, row, column):
    """앞쪽 반원 안에 3×3 파형을 배치한다. 캔버스 좌표 계약을 거꾸로 적용한다."""
    radius, height = 50.0, 80.0
    arc_start, width = 113.0 + column * 32.0, 24.0
    base_height = 20.0 + row * 20.0
    request = GenerateTrajectory.Request()
    request.order_id = f"WAVE-A{amplitude}-L{wavelength}"
    request.operation_speed_percent = 50
    request.surface_type = "SIDE"
    request.cake_radius, request.cake_height = radius / 1000, height / 1000
    request.cake_pose.position.x, request.cake_pose.position.z = 0.5, 0.142
    request.canvas_width, request.canvas_height = 1024.0, 768.0
    strip_height = 896 * height / (2 * math.pi * radius)
    top = (768 - strip_height) / 2
    points = []
    for i in range(241):
        arc = arc_start + width * i / 240
        h = base_height + amplitude * math.sin(
            2 * math.pi * (arc - arc_start) / wavelength
        )
        points.append(
            Point32(
                x=float(64 + arc / (2 * math.pi * radius) * 896),
                y=float(top + strip_height - h / height * strip_height),
                z=0.0,
            )
        )
    request.strokes_2d = [Stroke2D(stroke_id=0, points=points)]
    return request, arc_start, base_height, width


def analyse(samples, amplitude, wavelength, arc_start, base_height, width):
    """관측 위치의 표면 거리·원래 파형 오차·수직 기준 펜 각도를 수치화한다."""
    raw = np.asarray(samples, dtype=float)
    if not len(raw):
        return {"passed": False, "reason": "관측 펜촉 표본 없음", "observed_samples": 0}
    radius = np.hypot(raw[:, 0] - 500, raw[:, 1])
    contact = raw[np.abs(radius - 50.0) <= 0.5]
    if not len(contact):
        return {
            "passed": False,
            "reason": "표면 접촉 표본 없음",
            "observed_samples": len(raw),
        }
    arc = np.linspace(arc_start, arc_start + width, 6001)
    reference = np.column_stack(
        (
            500 + 50 * np.cos(arc / 50),
            50 * np.sin(arc / 50),
            62
            + base_height
            + amplitude * np.sin(2 * np.pi * (arc - arc_start) / wavelength),
        )
    )
    errors = cKDTree(reference).query(contact[:, :3])[0]
    # 관측점이 원본에 가까운지뿐 아니라 원본 전체가 실제로 지나갔는지도 검사한다.
    gaps = cKDTree(contact[:, :3]).query(reference)[0]
    max_gap = float(gaps.max())
    qx, qy, qz, qw = contact[:, 3:].T
    z_axis_z = 1 - 2 * (qx * qx + qy * qy)
    angles = np.degrees(np.arccos(np.clip(-z_axis_z, -1, 1)))
    radial = np.hypot(contact[:, 0] - 500, contact[:, 1]) - 50
    # 수직 기울기만 맞고 펜이 바깥을 향하는 잘못된 자세도 구분한다.
    axis = np.column_stack((2 * (qx * qz + qy * qw), 2 * (qy * qz - qx * qw), z_axis_z))
    nx = (contact[:, 0] - 500) / (radial + 50)
    ny = contact[:, 1] / (radial + 50)
    expected = np.column_stack(
        (-0.5 * nx, -0.5 * ny, np.full(len(nx), -math.sqrt(3) / 2))
    )
    axis_error = float(
        np.max(np.degrees(np.arccos(np.clip(np.sum(axis * expected, axis=1), -1, 1))))
    )
    max_error = float(errors.max())
    angle_error = float(np.max(np.abs(angles - 30)))
    return {
        "passed": bool(
            max_error <= 0.5
            and max_gap <= 0.5
            and angle_error <= 2.0
            and axis_error <= 2.0
            and float(radius.min()) >= 49.5
        ),
        "max_reference_gap_mm": max_gap,
        "minimum_observed_radius_mm": float(radius.min()),
        "max_inward_error_mm": float(max(0.0, 50.0 - radius.min())),
        "observed_samples": len(raw),
        "contact_samples": len(contact),
        "max_wave_error_mm": max_error,
        "rms_wave_error_mm": float(np.sqrt(np.mean(errors**2))),
        "max_abs_radial_error_mm": float(np.max(np.abs(radial))),
        "max_tilt_error_deg": angle_error,
        "max_axis_direction_error_deg": axis_error,
        "observed_tip_mm_quaternion": contact.tolist(),
    }


def main(args=None):
    """파형 9개를 실행한다. 원래 RE의 획 완료·정지 로직은 호출하지 않는다."""
    parser = argparse.ArgumentParser(description="가상 전용 진폭·파장 파형 접촉 검증")
    parser.add_argument("--execute-virtual", action="store_true")
    parser.add_argument("--output", default="results/side_wave_verification.json")
    parser.add_argument(
        "--cases", default="0,1,2,3,4,5,6,7,8", help="재검증할 파형 번호(행 우선 0~8)"
    )
    parser.add_argument(
        "--resume-results", action="store_true", help="기존 결과에서 실행한 파형만 교체"
    )
    parser.add_argument(
        "--keep-ink", action="store_true", help="기존 가상 접촉 잉크를 지우지 않음"
    )
    parsed, ros_args = parser.parse_known_args(args)
    selected = {int(value) for value in parsed.cases.split(",")}
    if not selected or not selected <= set(range(9)):
        parser.error("--cases는 0~8의 쉼표 구분 번호여야 합니다.")
    if not parsed.execute_virtual:
        parser.error("가상 컨트롤러 이동을 허용하려면 --execute-virtual을 지정하세요.")
    rclpy.init(args=ros_args)
    node = WaveVerifier()
    dp = DrawingPathNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(dp)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    results = []
    if parsed.resume_results and Path(parsed.output).exists():
        results = json.loads(Path(parsed.output).read_text())["cases"]
    previous = None
    try:
        node.verify_virtual()
        node.wait_pen_ready()
        if not node.robot.move_joint(
            [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], speed_deg_s=20.0
        ):
            raise RuntimeError("가상 홈 자세 이동 실패")
        # TF 관측 노드가 준비되지 않았으면 명령 경로를 관측 결과로 대체하지 않는다.
        if node.count_publishers("/cake/observed_tip") == 0:
            raise RuntimeError("virtual_ink_node를 먼저 실행하세요.")
        if not parsed.keep_ink:
            node.clear.publish(Empty())
        for row, amplitude in enumerate((1.0, 3.0, 6.0)):
            for column, wavelength in enumerate((4.0, 8.0, 16.0)):
                if row * 3 + column not in selected:
                    continue
                request, start, base, width = wave_request(
                    amplitude, wavelength, row, column
                )
                response = dp._handle_request(request, GenerateTrajectory.Response())
                if not response.success:
                    raise RuntimeError(response.error_msg)
                raw, states = RobotExecutorNode._convert_waypoints(
                    response.waypoints_3d
                )
                plan = plan_drawing_motion(raw, states, "SIDE")
                if not plan.success:
                    raise RuntimeError(plan.error_msg)
                if previous is not None:
                    # 파형들은 서로 독립된 시험이다. 손목 회전 누적이 다음 시험의
                    # 시작 관절 상태에 영향을 주지 않도록 표면 밖에서 상승 후 홈으로 복귀한다.
                    safe = DsrPos(
                        previous.x,
                        previous.y,
                        300.0,
                        previous.rz1,
                        previous.ry,
                        previous.rz2,
                    )
                    if not node.robot.move_line(safe, speed_mm_s=30.0):
                        raise RuntimeError("시험 사이 가상 펜 상승 실패")
                    if not node.robot.move_joint(
                        [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], speed_deg_s=20.0
                    ):
                        raise RuntimeError("독립 파형 시험의 관절 초기화 실패")
                node.lift_to_start(None)
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.ns = "wave_reference"
                marker.id = row * 3 + column
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.0004
                marker.color.r = marker.color.g = marker.color.b = 0.55
                marker.color.a = 0.8
                for i in range(601):
                    arc = start + width * i / 600
                    h = base + amplitude * math.sin(
                        2 * math.pi * (arc - start) / wavelength
                    )
                    marker.points.append(
                        Point(
                            x=(500 + 50 * math.cos(arc / 50)) / 1000,
                            y=50 * math.sin(arc / 50) / 1000,
                            z=(62 + h) / 1000,
                        )
                    )
                node.planned.publish(MarkerArray(markers=[marker]))
                node.samples = []
                for command in plan.commands:
                    # 접촉 접근은 파형 자체가 아니므로 형상 오차 수집에서 제외한다.
                    node.record = command.phase == Phase.DRAWING
                    issued_ns = node.get_clock().now().nanoseconds
                    if not node.robot.execute_drawing_command(command):
                        raise RuntimeError(
                            f"{request.order_id}: 가상 이동 실패 ({command.motion_type.value})"
                        )
                    node.wait_observed_target(command.pos, issued_ns)
                node.record = False
                previous = plan.commands[-1].pos
                result = analyse(
                    node.samples, amplitude, wavelength, start, base, width
                )
                result.update(
                    order_id=request.order_id,
                    amplitude_mm=amplitude,
                    wavelength_mm=wavelength,
                    input_points=241,
                    commands=len(plan.commands),
                )
                results = [
                    old for old in results if old["order_id"] != result["order_id"]
                ]
                results.append(result)
                results.sort(
                    key=lambda item: (
                        (1.0, 3.0, 6.0).index(item["amplitude_mm"]),
                        (4.0, 8.0, 16.0).index(item["wavelength_mm"]),
                    )
                )
                output = Path(parsed.output)
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text(
                    json.dumps(
                        {
                            "source": "observed virtual_pen_tip TF; virtual geometry contact, not physical sensor",
                            "cases": results,
                        },
                        ensure_ascii=False,
                        indent=2,
                    )
                )
                print(
                    json.dumps(
                        {
                            k: v
                            for k, v in result.items()
                            if k != "observed_tip_mm_quaternion"
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
        node.lift_to_start(previous)
    finally:
        node.record = False
        output = Path(parsed.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(
                {
                    "source": "observed virtual_pen_tip TF; virtual geometry contact, not physical sensor",
                    "cases": results,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        executor.shutdown(timeout_sec=10)
        thread.join(timeout=10)
        node.destroy_node()
        dp.destroy_node()
        rclpy.try_shutdown()
    if len(results) != 9 or not all(result["passed"] for result in results):
        raise SystemExit(
            "일부 파형이 관측 오차 기준을 통과하지 못했습니다. 결과 파일을 확인하세요."
        )


if __name__ == "__main__":
    main()
