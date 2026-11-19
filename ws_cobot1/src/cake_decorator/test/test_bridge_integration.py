"""임시 HTTP 백엔드와 실제 ROS Action 통신을 연결한다. 실제 장비는 사용하지 않는다."""

import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from uuid import uuid4

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from cake_decorator_interfaces.action import CakeOrder
from cake_decorator_interfaces.msg import TaskStatus

from cake_decorator.backend_client import BackendClient
from cake_decorator.ros_bridge_node import RosBridgeNode

ROOT = Path(__file__).resolve().parents[4]
BACKEND_PYTHON = Path(
    os.environ.get("CAKE_TEST_BACKEND_PYTHON", ROOT / "backend/.venv/bin/python")
)
DRAWING = {
    "canvas_width": 1024,
    "canvas_height": 768,
    "drawing_surface": "top",
    "strokes": [
        {
            "stroke_id": 0,
            "points": [{"x": 500.5, "y": 350.25}, {"x": 520.0, "y": 360.0}],
        }
    ],
}


class TestTaskManager(Node):
    """테스트 전용 Action 서버. 로봇 이동 없이 접수·피드백·최종 응답만 검증한다."""

    __test__ = False

    def __init__(self, action_name, accept=True, success=True, delay=0.35):
        super().__init__("test_task_manager")
        self.goals = []
        self.success = success
        self.delay = delay
        self.status = self.create_publisher(TaskStatus, "/task_status", 10)
        self.server = ActionServer(
            self,
            CakeOrder,
            action_name,
            self.execute,
            goal_callback=lambda request: (
                GoalResponse.ACCEPT if accept else GoalResponse.REJECT
            ),
        )

    def execute(self, handle):
        """실제 전달받은 Goal을 보존하고 테스트 결과를 반환한다."""
        self.goals.append(handle.request)
        handle.publish_feedback(
            CakeOrder.Feedback(
                current_phase="DRAWING_TOP",
                progress_pct=25.0,
                message="가짜 TM 통신 검증",
            )
        )
        time.sleep(self.delay / 2)
        self.status.publish(
            TaskStatus(
                order_id=handle.request.order_id,
                phase="TOPIC_DRAWING",
                progress_pct=40.0,
                message="토픽 상태 확인",
            )
        )
        time.sleep(self.delay / 2)
        if self.success:
            handle.succeed()
        else:
            handle.abort()
        return CakeOrder.Result(
            success=self.success,
            error_code="" if self.success else "TEST_ABORT",
            elapsed_time_sec=self.delay,
        )


@unittest.skipUnless(BACKEND_PYTHON.exists(), "백엔드 테스트 가상환경이 필요합니다.")
class BridgeIntegrationTests(unittest.TestCase):
    """주문당 단일 전송, 확정 결과, 불명확 잠금과 TM 부재를 실제 프로토콜로 확인한다."""

    def setUp(self):
        """포트·DB·Action 주소를 테스트마다 독립시킨다."""
        self.temp = tempfile.TemporaryDirectory()
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            port = listener.getsockname()[1]
        self.url = f"http://127.0.0.1:{port}"
        environment = {
            **os.environ,
            "CAKE_DATABASE_PATH": str(Path(self.temp.name) / "orders.sqlite3"),
            "CAKE_BRIDGE_TOKEN": "integration-secret",
        }
        self.process = subprocess.Popen(
            [
                str(BACKEND_PYTHON),
                "-m",
                "uvicorn",
                "app.main:app",
                "--host",
                "127.0.0.1",
                "--port",
                str(port),
                "--log-level",
                "error",
            ],
            cwd=ROOT / "backend",
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        deadline = time.monotonic() + 10
        while True:
            try:
                self.http("/api/health")
                break
            except (URLError, ConnectionError):
                if time.monotonic() > deadline or self.process.poll() is not None:
                    raise RuntimeError("테스트 백엔드가 시작되지 않았습니다.")
                time.sleep(0.05)
        rclpy.init()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.backend = BackendClient(self.url, "integration-secret")
        self.action_name = f"/test_cake/action_{uuid4().hex}"
        self.tm = None
        self.bridge = None
        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.start()

    def http(self, path, body=None, headers=None):
        """웹 API를 실제 HTTP로 호출한다. 서버·DB를 모킹하지 않는다."""
        request = Request(
            self.url + path,
            data=None if body is None else json.dumps(body).encode(),
            headers={"Content-Type": "application/json", **(headers or {})},
        )
        with urlopen(request, timeout=3) as response:
            return json.load(response)

    def start_nodes(
        self, accept=True, success=True, timeout=10, with_tm=True, delay=0.35
    ):
        """영점은 테스트 입력으로 주입하고 실제 TCP 조회·로봇 연결은 하지 않는다."""
        if with_tm:
            self.tm = TestTaskManager(
                self.action_name, accept=accept, success=success, delay=delay
            )
            self.executor.add_node(self.tm)
        self.bridge = RosBridgeNode(
            self.backend,
            [363.9, 5.4, 75.25, 0.0, 180.0, 0.0],
            self.action_name,
            timeout,
        )
        self.executor.add_node(self.bridge)
        self.bridge.start()

    def wait_state(self, order_id, expected):
        """진행 시간에 의존하지 않고 제한 시간 안에 저장 상태를 관찰한다."""
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            detail = self.http(f"/api/orders/{order_id}")
            if detail["status"] == expected:
                return detail
            time.sleep(0.05)
        self.fail(f"{expected} 대기 시간 초과: {detail}")

    def tearDown(self):
        """테스트가 띄운 프로세스와 ROS만 종료한다."""
        if self.bridge:
            self.bridge.close()
        self.executor.shutdown(timeout_sec=5)
        self.spin_thread.join(timeout=5)
        if self.bridge:
            self.bridge.destroy_node()
        if self.tm:
            self.tm.server.destroy()
            self.tm.destroy_node()
        rclpy.try_shutdown()
        self.process.terminate()
        self.process.wait(timeout=5)
        self.process.stderr.close()
        self.temp.cleanup()

    def test_goal_reaches_tm_once_and_result_releases_order(self):
        """HTTP 접수 → ROS Goal → 완료 저장을 실제로 통과한다."""
        headers = {"Idempotency-Key": "same-web-request"}
        receipt = self.http("/api/drawing", DRAWING, headers)
        self.start_nodes()
        detail = self.wait_state(receipt["order_id"], "completed")
        self.assertEqual(detail["phase"], "COMPLETED")
        self.assertEqual(detail["message"], "TM 작업 완료")
        self.assertEqual(self.http("/api/drawing", DRAWING, headers), receipt)
        self.assertEqual(len(self.tm.goals), 1)
        goal = self.tm.goals[0]
        self.assertAlmostEqual(goal.cake_params.center.z, 0.07525)
        # HTTP 주문의 실측 형상이 브릿지와 ROS 직렬화를 거쳐도 보존된다.
        self.assertAlmostEqual(goal.cake_params.radius, 0.05675)
        self.assertAlmostEqual(goal.cake_params.bottom_radius, 0.06)
        self.assertAlmostEqual(goal.cake_params.height, 0.079)
        self.assertEqual(goal.drawing_paths[0].strokes[0].points[0].x, 500.5)
        self.assertTrue(
            self.http("/api/drawing", DRAWING)["order_id"] != receipt["order_id"]
        )

    def test_rejected_goal_is_recorded_without_execution(self):
        """TM 거절은 제작 성공으로 기록하지 않는다."""
        receipt = self.http("/api/drawing", DRAWING)
        self.start_nodes(accept=False)
        self.wait_state(receipt["order_id"], "rejected")
        self.assertEqual(self.tm.goals, [])

    def test_aborted_action_is_recorded_as_failure(self):
        """확정된 Action 실패는 완료와 구분한다."""
        receipt = self.http("/api/drawing", DRAWING)
        self.start_nodes(success=False)
        detail = self.wait_state(receipt["order_id"], "failed")
        self.assertEqual(detail["phase"], "FAILED")
        self.assertEqual(detail["error_code"], "TEST_ABORT")
        self.assertEqual(len(self.tm.goals), 1)

    def test_result_timeout_keeps_lock_and_never_resends(self):
        """응답 불명확 시 자동 재전송하지 않고 새 접수를 막는다."""
        receipt = self.http("/api/drawing", DRAWING)
        self.start_nodes(timeout=0.03)
        self.wait_state(receipt["order_id"], "uncertain")
        with self.assertRaises(HTTPError) as caught:
            self.http("/api/drawing", DRAWING)
        self.assertEqual(caught.exception.code, 409)
        self.assertEqual(len(self.tm.goals), 1)

    def test_missing_tm_does_not_claim_order(self):
        """서버가 없는 상태에서 주문을 전송 중으로 바꾸지 않는다."""
        receipt = self.http("/api/drawing", DRAWING)
        self.start_nodes(with_tm=False)
        time.sleep(0.2)
        self.wait_state(receipt["order_id"], "received")
        self.assertIsNotNone(self.backend.claim())

    def test_task_status_topic_is_forwarded_only_for_active_order(self):
        """별도 TaskStatus 발행도 백엔드의 활성 주문 진행 상태로 전달한다."""
        receipt = self.http("/api/drawing", DRAWING)
        self.start_nodes(delay=1.2)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            detail = self.http(f"/api/orders/{receipt['order_id']}")
            if detail["phase"] == "TOPIC_DRAWING":
                self.assertEqual(detail["progress_pct"], 40.0)
                break
            time.sleep(0.03)
        else:
            self.fail("TaskStatus가 백엔드에 전달되지 않았습니다.")
        self.wait_state(receipt["order_id"], "completed")
