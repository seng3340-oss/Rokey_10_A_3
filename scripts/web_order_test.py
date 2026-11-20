"""로컬 웹 주문과 M0609 모델을 실제 ROS 서비스 형식으로 연결하는 테스트 실행기."""

import argparse
import json
import math
import os
from pathlib import Path
import signal

import numpy as np
import rclpy
from dsr_msgs2.srv import MoveLine
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation

from simulate_cake_lettering import DOWN, build_models, solve
from cake_decorator.backend_client import BackendClient
from cake_decorator.drawing_path_node import DrawingPathNode
from cake_decorator.robot_executor_node import RobotExecutorNode
from cake_decorator.ros_bridge_node import RosBridgeNode
from cake_decorator.task_manager_node import TaskManagerNode


class VirtualController(Node):
    """절대 BASE MoveLine마다 M0609 관절이 직선 펜촉 경로를 따라갈 수 있는지 계산한다."""

    def __init__(self, robot, center, radius, trace):
        super().__init__("virtual_m0609")
        self.robot = robot
        self.center = center
        self.radius = radius
        self.trace = trace
        self.position = None
        self.service = self.create_service(MoveLine, "/cake_sim/move_line", self.move_line)

    def move_line(self, request, response):
        """샘플마다 역기구학과 관절 제한을 확인하고 성공한 경로만 기록한다."""
        try:
            self.record_move(request)
            response.success = True
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(str(error))
            response.success = False
        return response

    def record_move(self, request):
        """시뮬레이션에서 지원하는 고정 수직 자세·윗면 높이 범위를 검사한다."""
        goal = np.asarray(request.pos[:3]) / 1000
        orientation = Rotation.from_euler("ZYZ", request.pos[3:], degrees=True)
        if request.ref or request.mode or request.sync_type:
            raise ValueError("가상 시뮬레이터는 BASE 절대 동기 이동만 받습니다.")
        if not np.isfinite(goal).all() or not np.allclose(orientation.as_matrix(), DOWN.as_matrix()):
            raise ValueError(f"가상 펜촉 위치 또는 자세 오류: {list(request.pos)}")
        current = goal if self.position is None else self.position
        z = self.center[2]
        if self.position is None and not math.isclose(goal[2], z + 0.010, abs_tol=1e-8):
            raise ValueError("첫 접근 목표는 케이크 표면보다 10 mm 높아야 합니다.")
        if not all(z - 1e-8 <= point[2] <= z + 0.01000001 for point in (current, goal)):
            raise ValueError("가상 펜촉이 시뮬레이션 높이 범위를 벗어났습니다.")
        at_surface = math.isclose(goal[2], z, abs_tol=1e-8)
        from_surface = math.isclose(current[2], z, abs_tol=1e-8)
        if (at_surface or from_surface) and max(
            np.linalg.norm(point[:2] - self.center[:2]) for point in (current, goal)
        ) > self.radius:
            raise ValueError("펜촉이 케이크 가장자리를 벗어났습니다.")
        if at_surface != from_surface and not np.allclose(current[:2], goal[:2], atol=1e-8):
            raise ValueError("접촉 높이에서 획 사이를 이동할 수 없습니다.")
        if self.position is None:
            phase = "INITIAL_ABOVE_STROKE"
        elif at_surface and from_surface:
            phase = "DRAWING"
        elif at_surface:
            phase = "PEN_DOWN"
        elif from_surface:
            phase = "PEN_UP"
        else:
            phase = "REPOSITIONING"

        # 양 끝이 정지하는 5차 보간으로 시간·속도와 1 mm 이하의 자세 간격을 정한다.
        delta = goal - current
        length = np.linalg.norm(delta)
        speed = float(request.vel[0])
        accel = float(request.acc[0])
        if not math.isfinite(speed) or not math.isfinite(accel) or speed <= 0 or accel <= 0:
            raise ValueError("양수 속도·가속도가 필요합니다.")
        duration = max(
            0.04,
            1.875 * length / (speed / 1000),
            math.sqrt(10 / math.sqrt(3) * length / (accel / 1000)),
        )
        intervals = math.ceil(max(duration / 0.04, 1.875 * length / 0.001))
        rows = []
        for j in range(intervals + 1):
            u = j / intervals
            fraction = u**3 * (10 + u * (-15 + 6 * u))
            v = 1000 * length / duration * 30 * u**2 * (1 - u)**2
            rows.append((u * duration, phase, *(current + fraction * delta), v))
        joints, error, speeds, limits = solve(self.robot, rows)
        record = dict(
            phase=phase, command=list(request.pos), speed_mm_s=speed, rows=rows,
            joints=joints.tolist(), ik_error_mm=error,
        )
        with self.trace.open("a") as stream:
            stream.write(json.dumps(record) + "\n")
        self.position = goal


def main():
    """테스트 DB와 고정 모델 기준점만 사용하며 실제 장비 드라이버를 실행하지 않는다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--backend-url", required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    robot, scene, _ = build_models(project)
    cake_frame = scene.frames({})["fixture_a_cake_link"]
    from simulate_cake_lettering import origin
    cake_link = scene.xml.find("link[@name='fixture_a_cake_link']")
    cake_bottom = cake_frame @ origin(cake_link.find("collision/origin"))
    dimensions = json.loads((project / "config/test_cake.json").read_text())
    center = (cake_bottom @ [0, 0, dimensions["height_mm"] / 1000, 1])[:3]
    radius = dimensions["top_diameter_mm"] / 2000

    os.environ["ROS_DOMAIN_ID"] = "97"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    for key in ("ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT"):
        os.environ.pop(key, None)
    rclpy.init(args=[
        "--ros-args", "-r", "__ns:=/cake_sim",
        "-r", "robot_executor_node:/dsr01/dsr_controller2/motion/move_line:=/cake_sim/move_line",
    ], signal_handler_options=SignalHandlerOptions.NO)
    controller = VirtualController(robot, center, radius, args.trace)
    path_node = DrawingPathNode()
    configured = path_node.set_parameters([
        Parameter("pen_tilt_deg", value=0.0),
        Parameter("travel_speed", value=40.0),
    ])
    assert all(result.successful for result in configured)
    executor_node = RobotExecutorNode()
    task_node = TaskManagerNode()
    reference = [*(1000 * center), 90.0, 180.0, -90.0]
    bridge = RosBridgeNode(
        BackendClient(args.backend_url, os.environ["CAKE_BRIDGE_TOKEN"]),
        reference,
        action_name="/cake_sim/cake/order",
    )
    executor = MultiThreadedExecutor(num_threads=8)
    nodes = (controller, path_node, executor_node, task_node, bridge)
    for node in nodes:
        executor.add_node(node)
    print(f"가상 케이크 윗면 [BASE mm]: {reference[:3]} / 반지름 {radius * 1000}", flush=True)
    bridge.start()
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.20)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        bridge.close()
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    main()
