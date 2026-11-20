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
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from cake_decorator_interfaces.action import DrawOnCake
from cake_decorator.backend_client import BackendClient
from cake_decorator.ros_bridge_node import RosBridgeNode
from cake_decorator.task_manager_node import TaskManagerNode
from cake_decorator.drawing_path_node import DrawingPathNode

ROOT = Path(__file__).resolve().parents[4]


class HardwareFreeDraw(Node):
    """오직 테스트용 피드백과 결과만 생성한다. DSR 서비스/토픽을 사용하지 않는다."""

    def __init__(self):
        """드로잉 Action과 준비·후처리 대역만 제공하며 각 호출 이력을 보관한다."""
        super().__init__("test_hardware_free_draw")
        self.fail = ""
        self.goals = []
        self.server = ActionServer(self, DrawOnCake, "draw_on_cake", self.execute)
        # 실제 플레이트 시퀀스 대신 서비스 결과만 주입한다. 장비 명령은 생성하지 않는다.
        self.plate_failure = None
        self.cake_weight_reads = []
        self.tool_regrasp_attempts = []
        self.safe_checks = 0
        self.plate_calls = []
        self.prepare_server = self.create_service(
            Trigger, "plate_prepare", self.prepare
        )
        self.finish_server = self.create_service(
            Trigger, "plate_finish", self.finish
        )
        self.ready_server = self.create_service(
            Trigger, 'workcell_resume_ready', self.resume_ready
        )

    def resume_ready(self, request, response):
        """실제 장비 없는 도메인에서 재개 전 안전 확인 서비스 호출만 기록한다."""
        self.safe_checks += 1
        response.success = True
        response.message = '테스트용 안전 자세 확인'
        return response

    def prepare(self, request, response):
        """준비 서비스 호출을 기록하고 지정된 예외 응답을 반환한다."""
        return self.plate_response("prepare", response)

    def finish(self, request, response):
        """후처리 서비스 호출을 기록하고 지정된 예외 응답을 반환한다."""
        return self.plate_response("finish", response)

    def plate_response(self, phase, response):
        """해당 단계에만 코드·상세 문구를 주입해 TM의 원문 보존을 검증한다."""
        self.plate_calls.append(phase)
        failure = self.plate_failure
        response.success = failure is None or failure[0] != phase
        response.message = "완료" if response.success else f"{failure[1]}: {failure[2]}"
        if not response.success and failure[1] == 'OPENER_MISSING':
            # 팀원의 실제 따개 재파지까지 소진한 결과가 기존 Trigger 계약을 통해
            # TM 최종 실패·주문 차단으로 이어지는지 검사한다. 장비 경계는 대역이다.
            from test_tool_regrasp import opener_plate
            from cake_decorator.safety_exceptions import OpenerMissingError

            sequence, robot = opener_plate(0.010, 0.010)
            try:
                sequence._plate_pick_opener()
            except OpenerMissingError as error:
                response.message = f'{error.code}: {error}'
                self.tool_regrasp_attempts.append(('opener', robot.pick.call_count))
        if not response.success and phase == 'prepare' and failure[1] == 'CAKE_WEIGHT_FAILED':
            # 실제 재집기·최종 복귀 로직을 서비스에 연결한다. 로봇 서비스 경계는
            # 기록용 대역이므로 실제 장비 명령 없이 TM·브릿지·DB까지 검증한다.
            from test_cake_retry import weight_sequence
            from cake_decorator.safety_exceptions import PlateWeightError

            sequence, robot, _ = weight_sequence([0.10, 0.15, 0.20, 0.20])
            parameter = sequence._param
            sequence._param = lambda name: (
                0.0 if name in (
                    'opener_pre_open_wait_sec', 'opener_close_wait_sec',
                    'opener_close_settle_sec', 'opener_open_wait_sec',
                ) else parameter(name)
            )
            try:
                sequence._plate_prepare_steps()
            except PlateWeightError as error:
                response.message = f'{error.code}: {error}'
                if error.resume_allowed:
                    response.message += '\nRESUME_CAKE_PICK'
                self.cake_weight_reads.append(robot.get_workpiece_weight.call_count)
        return response

    def execute(self, handle):
        """드로잉 요청을 기록하고 진행률 뒤에 설정한 성공·실패 결과를 반환한다."""
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
            if self.fail == 'PEN_GRASP_FAILED':
                # 펜도 최종 코드를 임의로 만들지 않고 TOP/SIDE 공통 실행기의
                # 재파지 1회·기본 TCP 정리·안전 자세 복귀 결과를 통과시킨다.
                from pytest import MonkeyPatch
                from test_tool_regrasp import pen_executor
                from test_integrated_pen_cycle import draw_handle

                with MonkeyPatch.context() as patches:
                    node = pen_executor(patches, 0.080, 0.010, 0.080, 0.010)
                    result = node._execute_draw(draw_handle(handle.request.drawing_surface))
                    attempts = sum(kind == 'close' for kind, _ in node._robot.events)
                    self.tool_regrasp_attempts.append(('pen', attempts))
                    return result
            return DrawOnCake.Result(
                success=False,
                error_code=self.fail,
                completed_strokes=0,
            )
        handle.succeed()
        return DrawOnCake.Result(success=True, error_code="", completed_strokes=1)


@unittest.skipUnless(
    os.environ.get("CAKE_RUN_ROS_TESTS") == "1",
    "CAKE_RUN_ROS_TESTS=1인 장비 없는 별도 도메인에서 실행",
)
class HmiPipelineTest(unittest.TestCase):
    def test_real_tm_plan_status_success_and_failure(self):
        """성공·예외 3종·복귀 실패를 주문 조회까지 전달하고 장비 서비스 부재를 확인한다."""
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
                tm.set_parameters([
                    Parameter("plate_sequence_enabled", value=True),
                ])
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
                # 서비스 예외는 준비/후처리에, 펜 예외는 Action 결과에 주입한다.
                # 복귀 실패 문구도 별도로 넣어 성공 복귀 문구로 덮어쓰지 않는지 확인한다.
                cases = [
                    ("top", "", "", None),
                    (
                        "side",
                        "TEST_RE_FAILURE",
                        "DrawOnCake 실패: TEST_RE_FAILURE",
                        None,
                    ),
                    (
                        "side",
                        "PEN_GRASP_FAILED",
                        "펜 파지 실패, 안전 자세에서 대기중",
                        None,
                    ),
                    (
                        "side",
                        "PEN_GRASP_RECOVERY_FAILED",
                        "펜 파지 실패, 안전 자세 복귀 실패. 현재 자세를 확인하세요",
                        None,
                    ),
                    (
                        "top",
                        "OPENER_MISSING",
                        "따개 없음, 안전 자세에서 대기중",
                        "prepare",
                    ),
                    (
                        "top",
                        "CAKE_WEIGHT_FAILED",
                        "케이크 무게 확인 실패: 입고함 집기 재시도 1/1 후에도 케이크 없음 "
                        "(무게 증가량 0.0500kg, 기준 0.0750kg 초과 필요). 안전 자세에서 대기중",
                        "prepare",
                    ),
                    (
                        "side",
                        "CAKE_WEIGHT_FAILED",
                        "케이크 무게 확인 실패: 하중 측정 실패. 안전 자세 복귀 실패",
                        "finish",
                    ),
                ]
                for surface, code, message, plate_phase in cases:
                    draw.fail = code if plate_phase is None else ""
                    draw.plate_failure = (
                        (plate_phase, code, message) if plate_phase else None
                    )
                    draw.plate_calls.clear()
                    goals_before = len(draw.goals)
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
                        if detail["status"] in ("completed", "paused"):
                            break
                        time.sleep(0.1)
                    self.assertEqual(
                        detail["status"], "paused" if code else "completed"
                    )
                    self.assertEqual(detail["error_code"], code)
                    if code:
                        self.assertEqual(detail["message"], message)
                        # 실패 확정과 같은 트랜잭션으로 잠긴다. 다음 시나리오도
                        # 관리자 확인을 명시하기 전에는 새 주문을 만들 수 없다.
                        recovery = http('/api/workcell/recovery')
                        self.assertTrue(recovery['requires_reset'])
                        self.assertEqual(recovery['order_id'], oid)
                        with self.assertRaises(HTTPError) as denied:
                            http('/api/drawing', body)
                        self.assertEqual(denied.exception.code, 409)
                        denied.exception.close()
                    expected_calls = ["prepare"]
                    if not code or plate_phase == "finish":
                        expected_calls.append("finish")
                    self.assertEqual(draw.plate_calls, expected_calls)
                    self.assertEqual(
                        len(draw.goals) - goals_before,
                        0 if plate_phase == "prepare" else 1,
                    )
                    results.append(
                        {
                            "order_id": oid,
                            "surface": surface,
                            "status": detail["status"],
                            "error_code": detail["error_code"],
                            "message": detail["message"],
                            "observed_states": seen,
                        }
                    )
                    if code:
                        decision = {
                            'revision': recovery['revision'],
                            'confirmed_reset': True,
                        }
                        if recovery['can_resume']:
                            draw.fail = ''
                            draw.plate_failure = None
                            draw.plate_calls.clear()
                            before_resume = len(draw.goals)
                            http('/api/workcell/resume', decision)
                            for _ in range(200):
                                continued = http('/api/orders/' + oid)
                                if continued['status'] == 'completed':
                                    break
                                time.sleep(0.1)
                            self.assertEqual(continued['status'], 'completed')
                            self.assertEqual(continued['order_id'], oid)
                            self.assertEqual(len(draw.goals) - before_resume, 1)
                            self.assertEqual(draw.plate_calls, (
                                ['prepare', 'finish'] if recovery['stopped_stage'] == 'PREPARING'
                                else ['finish']
                            ))
                            self.assertEqual(http('/api/orders')['total'], len(results))
                            results[-1]['resumed_status'] = continued['status']
                        else:
                            http('/api/workcell/initialize', decision)
                            self.assertEqual(http('/api/orders/' + oid)['status'], 'failed')
                    # 준비 실패 주문은 DP를 호출하지 않으므로 계획을 조회하지 않는다.
                    if plate_phase == "prepare":
                        continue
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
                    results[-1]["plan_source"] = plan["source"]
                self.assertEqual(len(draw.goals), 9)
                self.assertEqual(draw.safe_checks, 4)
                self.assertEqual(draw.cake_weight_reads, [4])
                self.assertEqual(
                    draw.tool_regrasp_attempts, [('pen', 2), ('opener', 2)]
                )
                # 전체 그래프 조회에는 조회용 클라이언트 이름도 보일 수 있다.
                # 실제 각 노드가 제공하는 서비스 서버만 조사해 장비 부재를 검사한다.
                services = [
                    service_name
                    for node_name, namespace in tm.get_node_names_and_namespaces()
                    for service_name, _ in tm.get_service_names_and_types_by_node(
                        node_name, namespace
                    )
                ]
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
