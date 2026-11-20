"""실제 TM·DP·브릿지·FastAPI를 연결하되 로봇 드라이버는 실행하지 않는 명시적 통합 검사."""

import json
import os
from pathlib import Path
import sys
import socket
import subprocess
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from cake_decorator_interfaces.action import DrawOnCake
from cake_decorator.backend_client import BackendClient
from cake_decorator.ros_bridge_node import RosBridgeNode
from cake_decorator.task_manager_node import TaskManagerNode
from cake_decorator.drawing_path_node import DrawingPathNode

ROOT = Path(__file__).resolve().parents[4]


class HardwareFreeDraw(Node):
    """오직 테스트용 피드백과 결과만 생성한다. DSR 서비스/토픽을 사용하지 않는다."""

    def __init__(self):
        super().__init__("test_hardware_free_draw")
        self.fail = False
        self.goals = []
        self.server = ActionServer(self, DrawOnCake, "draw_on_cake", self.execute)

    def execute(self, handle):
        self.goals.append(handle.request)
        for percent in [0.0, 25.0, 50.0, 75.0]:
            handle.publish_feedback(
                DrawOnCake.Feedback(
                    current_stroke_idx=0,
                    total_strokes=1,
                    progress_pct=percent,
                    phase="DRAWING",
                )
            )
            time.sleep(0.15)
        if self.fail:
            handle.abort()
            return DrawOnCake.Result(
                success=False, error_code="TEST_RE_FAILURE", completed_strokes=0
            )
        handle.succeed()
        return DrawOnCake.Result(success=True, error_code="", completed_strokes=1)


@unittest.skipUnless(
    os.environ.get("CAKE_RUN_ROS_TESTS") == "1",
    "CAKE_RUN_ROS_TESTS=1인 장비 없는 별도 도메인에서 실행",
)
class HmiPipelineTest(unittest.TestCase):
    def test_real_tm_plan_status_success_and_failure(self):
        """상단 성공·측면 실패를 실제 프로토콜로 전달하고 하드웨어 서비스 부재를 확인한다."""
        self.assertEqual(os.environ.get("ROS_DOMAIN_ID"), "231")
        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"), "1")
        with tempfile.TemporaryDirectory(prefix="cake-production-hmi-") as directory:
            temporary = Path(directory)
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            url = f"http://127.0.0.1:{port}"
            env = {
                **os.environ,
                "CAKE_DATABASE_PATH": str(temporary / "orders.sqlite3"),
                "CAKE_BRIDGE_TOKEN": "hardware-free-test",
                "PYTHONDONTWRITEBYTECODE": "1",
            }
            env.pop("CAKE_FRONTEND_DIST", None)
            log = (temporary / "api.log").open("w")
            server = subprocess.Popen(
                [
                    sys.executable,
                    "-m",
                    "uvicorn",
                    "app.main:app",
                    "--app-dir",
                    str(ROOT / "backend"),
                    "--host",
                    "127.0.0.1",
                    "--port",
                    str(port),
                    "--log-level",
                    "warning",
                ],
                cwd=ROOT / "backend",
                env=env,
                stdout=log,
                stderr=log,
            )

            def http(path, body=None):
                request = Request(
                    url + path,
                    data=json.dumps(body).encode() if body is not None else None,
                    headers={"Content-Type": "application/json"},
                )
                with urlopen(request, timeout=3) as response:
                    return json.load(response)

            nodes = []
            executor = None
            bridge = None
            thread = None
            try:
                # 같은 Python 환경을 사용하고 실제 경과 시간으로 준비를 기다린다.
                # 프로세스가 먼저 종료되면 불필요하게 기다리지 않고 원인 로그를 보존한다.
                deadline = time.monotonic() + 30.0
                while time.monotonic() < deadline:
                    if server.poll() is not None:
                        raise RuntimeError(
                            "테스트 API 종료: "
                            + (temporary / "api.log").read_text()[-4000:]
                        )
                    try:
                        http("/api/health")
                        break
                    except OSError:
                        time.sleep(0.1)
                else:
                    raise RuntimeError(
                        "테스트 API 준비 시간 초과: "
                        + (temporary / "api.log").read_text()[-4000:]
                    )
                rclpy.init()
                tm = TaskManagerNode()
                dp = DrawingPathNode()
                draw = HardwareFreeDraw()
                bridge = RosBridgeNode(
                    BackendClient(
                        url, "hardware-free-test", execution_mode="simulation"
                    ),
                    [399.0, -49.0, 219.0, 0.0, 180.0, 0.0],
                )
                nodes = [tm, dp, draw, bridge]
                executor = MultiThreadedExecutor(num_threads=8)
                for node in nodes:
                    executor.add_node(node)
                thread = threading.Thread(target=executor.spin, daemon=True)
                thread.start()
                bridge.start()
                results = []
                for surface, fail in [("top", False), ("side", True)]:
                    draw.fail = fail
                    body = {
                        "canvas_width": 1024,
                        "canvas_height": 768,
                        "drawing_surface": surface,
                        "strokes": [
                            {
                                "stroke_id": 0,
                                "points": [
                                    {"x": 500.0, "y": 350.0},
                                    {"x": 520.0, "y": 360.0},
                                ],
                            }
                        ],
                    }
                    receipt = http("/api/drawing", body)
                    oid = receipt["order_id"]
                    seen = []
                    for _ in range(200):
                        detail = http("/api/orders/" + oid)
                        seen.append(detail["status"])
                        if detail["status"] in ("completed", "failed"):
                            break
                        time.sleep(0.1)
                    self.assertEqual(
                        detail["status"], "failed" if fail else "completed"
                    )
                    self.assertEqual(
                        detail["error_code"], "TEST_RE_FAILURE" if fail else ""
                    )
                    plan = http("/api/orders/" + oid + "/plan")
                    self.assertEqual(
                        plan["source"], "task_manager_node/GenerateTrajectory"
                    )
                    self.assertEqual(plan["drawing_surface"], surface)
                    self.assertFalse(plan["measurement_available"])
                    self.assertEqual(plan["execution_mode"], "simulation")
                    # 최신 DP는 두 끝점 사이에 곡선 표본을 추가하므로 끝점을 비교한다.
                    for a, b in zip(
                        body["strokes"][0]["points"],
                        [plan["strokes"][0]["points"][0], plan["strokes"][0]["points"][-1]],
                    ):
                        self.assertAlmostEqual(a["x"], b["x"], places=3)
                        # SIDE 계획에는 DP의 접촉/지름 보정도 포함돼 원본과 미소 차이가 난다.
                        if surface == "side":
                            self.assertLess(abs(a["y"] - b["y"]), 1.0)
                        else:
                            self.assertAlmostEqual(a["y"], b["y"], places=3)
                    results.append(
                        {
                            "order_id": oid,
                            "surface": surface,
                            "status": detail["status"],
                            "error_code": detail["error_code"],
                            "plan_source": plan["source"],
                            "observed_states": seen,
                        }
                    )
                self.assertEqual(len(draw.goals), 2)
                services = [name for name, _ in tm.get_service_names_and_types()]
                self.assertFalse(
                    any("/dsr01/" in name for name in services),
                    "검사 도메인에 로봇 드라이버가 없어야 합니다.",
                )
                for _ in range(50):
                    runtime = http("/api/monitor")
                    if runtime["task_server_ready"]:
                        break
                    time.sleep(0.1)
                self.assertTrue(runtime["bridge_fresh"])
                self.assertTrue(runtime["task_server_ready"])
                self.assertIsNone(runtime["robot_connected"])
                output = os.environ.get("CAKE_HMI_TEST_ARTIFACTS")
                if output:
                    output = Path(output)
                    output.mkdir(parents=True, exist_ok=True)
                    (output / "ros-pipeline-results.json").write_text(
                        json.dumps(
                            {
                                "orders": results,
                                "runtime": runtime,
                                "driver_services_present": False,
                            },
                            ensure_ascii=False,
                            indent=2,
                        )
                    )
                    # 브라우저 확인용 사본은 테스트 DB에만 만든다. 토큰·환경 파일은 복사하지 않는다.
                    import sqlite3

                    source = sqlite3.connect(temporary / "orders.sqlite3")
                    target = sqlite3.connect(output / "test-preview.sqlite3")
                    source.backup(target)
                    source.close()
                    target.close()
            finally:
                if bridge:
                    bridge.close()
                if executor:
                    executor.shutdown(timeout_sec=5)
                for node in nodes:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.try_shutdown()
                if thread:
                    thread.join(timeout=5)
                server.terminate()
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.wait()
                log.close()
                output = os.environ.get("CAKE_HMI_TEST_ARTIFACTS")
                if output:
                    output = Path(output)
                    output.mkdir(parents=True, exist_ok=True)
                    (output / "ros-api-startup.log").write_text(
                        (temporary / "api.log").read_text()
                    )


if __name__ == "__main__":
    unittest.main()
