"""실제 드라이버 없는 도메인231에서 전체 공정 관측→API 이력을 검증한다.

TM·DP·RE·브릿지는 실제 코드이고 DoosanRobotClient만 명령을 전송하지 않는 대역이다.
"""

import json
import os
import threading
import time
from pathlib import Path
import pytest

pytestmark = pytest.mark.skipif(
    os.getenv("CAKE_RUN_HMI_ROS_TESTS") != "1", reason="명시적 별도 도메인 검사만 허용"
)


def test_full_process_stage_events_and_failure(monkeypatch, tmp_path):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from fastapi.testclient import TestClient
    from app.main import create_app
    from cake_decorator import robot_executor_node as re
    from cake_decorator.task_manager_node import TaskManagerNode
    from cake_decorator.drawing_path_node import DrawingPathNode
    from cake_decorator.ros_bridge_node import RosBridgeNode
    from cake_decorator.backend_client import BackendClient, BackendError
    from cake_decorator.process_monitor import STAGES

    assert os.environ["ROS_DOMAIN_ID"] == "231"
    assert not os.getenv("ROS_DISCOVERY_SERVER")

    class FakeRobot:
        """이 테스트는 네트워크 로봇 클라이언트를 생성하지 않는다."""

        fail_pick = False

        def __init__(self, *args, **kwargs):
            self.calls = []

        def move_joint(self, *a, **k):
            self.calls.append("joint")
            return True

        def move_line(self, *a, **k):
            self.calls.append("line")
            return True

        def move_home(self, *a, **k):
            self.calls.append("home")
            return True

        def pick(self, *a, **k):
            self.calls.append("pick")
            return not self.fail_pick

        def place(self, *a, **k):
            self.calls.append("place")
            return True

        def gripper_open(self, *a, **k):
            self.calls.append("open")
            return True

        def gripper_close(self, *a, **k):
            self.calls.append("close")
            return True

        def current_pose(self):
            return [400.0, 0.0, 700.0, 0.0, 180.0, 0.0]

        def execute_drawing_command(self, *a, **k):
            self.calls.append("draw")
            return True

    monkeypatch.setattr(re, "DoosanRobotClient", FakeRobot)
    monkeypatch.setenv("CAKE_SYSTEM_MODE", "real")
    monkeypatch.setenv("CAKE_PLATE_SEQUENCE", "1")
    cfg = Path(__file__).parents[1] / "config/workspace_config.yaml"
    rclpy.init(args=["--ros-args", "--params-file", str(cfg)])
    executor = MultiThreadedExecutor(num_threads=8)
    nodes = []
    bridge = None
    with TestClient(
        create_app(tmp_path / "db", bridge_token="integration-secret")
    ) as client:

        class LocalBackend(BackendClient):
            """실제 인증·API·SQLite를 통과하되 HTTP 외부 서버는 만들지 않는다."""

            def post(self, path, body=None):
                response = client.post(
                    path,
                    headers={"Authorization": "Bearer integration-secret"},
                    json=body or {},
                )
                if response.status_code == 204:
                    return None
                if response.status_code >= 400:
                    raise BackendError(response.text, response.status_code)
                return response.json()

        try:
            robot = re.RobotExecutorNode()
            path = DrawingPathNode()
            tm = TaskManagerNode()
            robot.set_parameters(
                [
                    Parameter("plate_sequence_enabled", value=True),
                    Parameter("pen_tool_cycle", value=True),
                ]
            )
            tm.set_parameters([Parameter("plate_sequence_enabled", value=True)])
            bridge = RosBridgeNode(
                LocalBackend("http://localhost", "integration-secret"),
                [400.0, 0.0, 300.0, 0.0, 180.0, 0.0],
            )
            nodes = [robot, path, tm, bridge]
            for node in nodes:
                executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            bridge.start()

            def wait(predicate, timeout=25):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    result = predicate()
                    if result:
                        return result
                    time.sleep(0.05)
                raise AssertionError("대기 시간 초과")

            wait(lambda: client.get("/api/hmi").json()["runtime"]["task_server_ready"])
            # 실제 ROS 오류 토픽도 모터 대역과 분리해 heartbeat→HMI로 전달되는지 확인한다.
            from rcl_interfaces.msg import Log

            publisher = bridge.create_publisher(Log, "/rosout", 10)
            error = Log(
                level=Log.ERROR, name="dsr_hw_interface2", msg="검사용 통신 오류"
            )
            for _ in range(10):
                publisher.publish(error)
                time.sleep(0.05)
            wait(
                lambda: client.get("/api/hmi")
                .json()["runtime"]
                .get("last_error_message")
                == "검사용 통신 오류"
            )
            bridge.destroy_publisher(publisher)
            drawing = dict(
                canvas_width=1024,
                canvas_height=768,
                drawing_surface="top",
                strokes=[
                    dict(
                        stroke_id=0,
                        points=[dict(x=500.0, y=380.0), dict(x=520.0, y=380.0)],
                    )
                ],
            )
            oid = client.post("/api/drawing", json=drawing).json()["order_id"]
            wait(
                lambda: client.get("/api/hmi").json()["selected"]["state"]
                == "completed"
            )
            snap = client.get("/api/hmi").json()
            done = {
                e["stage_key"]
                for e in snap["events"]
                if e["stage_state"] == "completed"
            }
            assert done == set(STAGES), (done, snap)
            assert snap["selected"]["order_id"] == oid
            assert snap["selected"]["process_mode"] == "full"
            assert snap["runtime"]["robot_connected"] is None
            assert robot._robot.calls.count("pick") == 2
            assert robot._robot.calls.count("place") == 2
            # 두 번째 주문은 오프너 파지 실패로 이후 운반·드로잉을 실행하지 않는다.
            robot._robot.fail_pick = True
            call_count = len(robot._robot.calls)
            oid2 = client.post("/api/drawing", json=drawing).json()["order_id"]
            wait(lambda: client.get("/api/hmi").json()["selected"]["state"] == "failed")
            snap = client.get("/api/hmi").json()
            assert snap["selected"]["order_id"] == oid2
            assert snap["selected"]["error_code"] == "PLATE_PREPARE_FAILED"
            assert any(
                e["stage_key"] == "opener_pick" and e["stage_state"] == "failed"
                for e in snap["events"]
            )
            assert robot._robot.calls[call_count:] == ["joint", "pick"]
            assert snap["runtime"]["safety_status"] == "unknown"
        finally:
            if bridge:
                bridge.close()
            executor.shutdown(timeout_sec=5)
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()
