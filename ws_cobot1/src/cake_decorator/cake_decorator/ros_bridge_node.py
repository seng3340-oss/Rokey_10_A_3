"""단일 주문을 CakeOrder로 전달한다. 로봇 이동과 경로 계산은 수행하지 않는다."""

import argparse
import json
from std_msgs.msg import String
from rcl_interfaces.msg import Log
from datetime import datetime, timezone
from .process_monitor import STAGES, stage_payload
import math
import os
from queue import Empty, Full, Queue
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from cake_decorator_interfaces.action import CakeOrder
from cake_decorator_interfaces.msg import TaskStatus
from types import SimpleNamespace

from .backend_client import BackendClient
from .goal_mapping import build_goal, validate_reference


class RosBridgeNode(Node):
    """ROS 콜백은 executor가, HTTP·대기는 별도 작업 스레드가 처리한다."""

    def __init__(
        self, backend, reference, action_name="/cake/order", result_timeout=1800.0
    ):
        super().__init__("ros_bridge_node")
        self.backend = backend
        self.reference = validate_reference(reference)
        self.action_client = ActionClient(self, CakeOrder, action_name)
        self.action_name = action_name
        self.result_timeout = result_timeout
        self.stop_event = threading.Event()
        self.feedback = Queue(maxsize=1)
        self.worker = None
        self.process_events = Queue(maxsize=512)
        self.monitor_worker = None
        self.last_ros_error = {}
        self.rosout_subscription = self.create_subscription(Log, '/rosout', self._on_rosout, 100)
        self.process_subscription = self.create_subscription(
            String, '/cake/process_events', self._on_process, 100,
        )
        self.active_order_id = None
        self.status_subscription = self.create_subscription(
            TaskStatus, "/task_status", self._on_status, 10
        )

    def start(self):
        """사용자가 영점을 확정한 뒤에만 주문 인수 루프를 시작한다."""
        self.worker = threading.Thread(
            target=self._run, name="cake-bridge-http", daemon=True
        )
        self.worker.start()
        self.monitor_worker = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_worker.start()
        self.get_logger().info(
            f"TM {self.action_name} 대기 중. 서버 준비 후에만 새 주문을 인수합니다."
        )

    def _on_rosout(self, message):
        """이번 작업 범위의 오류 수준 ROS 로그만 표시한다. 상태·동작은 변경하지 않는다."""
        allowed = {'robot_executor_node', 'task_manager_node', 'drawing_path_node',
                   'dsr_hw_interface2', 'dsr_controller2', 'dsr01.dsr_controller2',
                   'dsr01.controller_manager'}
        if message.level >= Log.ERROR and message.name in allowed:
            self.last_ros_error = {
                'last_error_message': message.msg[:2048],
                'last_error_source': message.name[:128],
                'last_error_at': datetime.now(timezone.utc).isoformat(),
            }

    def _monitor_loop(self):
        """주문 작업 스레드와 분리한 heartbeat. 실패해도 주문을 재전송하지 않는다."""
        while not self.stop_event.is_set():
            try:
                self.backend.heartbeat(self.action_client.server_is_ready(), self.active_order_id, **self.last_ros_error)
            except Exception as error:
                self.get_logger().warning(f'HMI 관측 전송 실패: {error}')
            self.stop_event.wait(2)

    def _on_process(self, message):
        """현재 주문 ID가 일치하는 실제 단계 이벤트만 받는다. 오래된 주문은 버린다."""
        try:
            event = json.loads(message.data)
            if not isinstance(event, dict):
                return
            if event.get('order_id') != self.active_order_id or event.get('stage_key') not in STAGES:
                return
            if event.get('stage_state') not in {'running', 'completed', 'failed'}:
                return
            # 표시용 입력 오류가 주문 실행에 영향을 주지 않게 값을 정규화한다.
            clean = stage_payload(self.active_order_id, event['stage_key'], event['stage_state'], str(event.get('message', ''))[:2048])
            self.process_events.put_nowait(clean)
        except (ValueError, TypeError, Full):
            self.get_logger().warning('HMI 공정 이벤트를 저장하지 못했습니다.')

    def _wait(self, future, timeout, claim=None):
        """ROS future를 작업 스레드에서 기다린다. ROS spin을 중첩 호출하지 않는다."""
        deadline = time.monotonic() + timeout
        while not future.done():
            if self.stop_event.wait(0.1) or time.monotonic() >= deadline:
                raise TimeoutError(
                    "TM 응답을 확정할 수 없습니다. 자동 재전송하지 않습니다."
                )
            if claim is not None:
                self._flush_feedback(claim)
        return future.result()

    def _on_status(self, status):
        """작업 상태 Topic도 활성 주문에 한해 같은 상태 기록 경로로 전달한다."""
        if status.order_id != self.active_order_id:
            return
        self._on_feedback(
            status.order_id,
            SimpleNamespace(
                feedback=SimpleNamespace(
                    current_phase=status.phase,
                    progress_pct=status.progress_pct,
                    message=status.message,
                )
            ),
        )

    def _on_feedback(self, order_id, response):
        """가장 최근 상태만 큐에 넣어 ROS 콜백에서 HTTP 대기를 하지 않는다."""
        if order_id != self.active_order_id:
            return
        try:
            self.feedback.get_nowait()
        except Empty:
            pass
        try:
            self.feedback.put_nowait((order_id, response.feedback))
        except Full:
            # 상태 큐가 이미 차 있으면 다음 상태로 갱신한다. 주문 실행에는 영향이 없다.
            pass

    def _flush_feedback(self, claim):
        """TM Feedback을 작업 스레드에서 백엔드에 전달한다."""
        # 시작/완료 이벤트는 최신1개 큐와 분리해 짧은 단계도 영구 이력에 남긴다.
        while True:
            try:
                event = self.process_events.get_nowait()
            except Empty:
                break
            if event['order_id'] == claim['order_id']:
                try:
                    self.backend.event(claim, 'feedback', **{
                        key: event[key] for key in ('phase', 'progress_pct', 'message',
                                                   'error_code', 'stage_key', 'stage_state')
                    })
                except Exception as error:
                    # HMI 기록 실패는 Goal 취소·재전송이나 추가 로봇 명령을 유발하지 않는다.
                    self.get_logger().warning(f'HMI 단계 기록 실패: {error}')
        try:
            order_id, feedback = self.feedback.get_nowait()
        except Empty:
            return
        if order_id != claim["order_id"]:
            return
        self.backend.event(
            claim,
            "feedback",
            phase=feedback.current_phase[:128],
            progress_pct=max(0.0, min(100.0, float(feedback.progress_pct))),
            message=feedback.message[:2048],
        )

    def _dispatch(self, claim):
        """전송 시작을 DB에 먼저 남긴 뒤 정확히 한 번만 Goal 전송을 시도한다."""
        goal = build_goal(claim, self.reference)
        self.backend.event(claim, "dispatching")
        # 상태 기록 응답을 기다리는 중 종료됐다면 그 뒤에 새 Goal을 보내지 않는다.
        if self.stop_event.is_set():
            raise RuntimeError("브릿지 종료 요청으로 Goal을 보내지 않았습니다.")
        # 이 호출 이후의 실패는 요청이 TM에 도달했을 수도 있으므로 자동 재전송 금지다.
        handle = self._wait(
            self.action_client.send_goal_async(
                goal,
                feedback_callback=lambda response: self._on_feedback(
                    claim["order_id"], response
                ),
            ),
            10.0,
        )
        if handle is None:
            raise RuntimeError("Goal 수락 여부를 확인할 수 없습니다.")
        if not handle.accepted:
            self.backend.event(claim, "rejected", message="TM이 주문을 거절했습니다.")
            return
        self.backend.event(claim, "accepted", message="TM이 주문을 수락했습니다.")
        result = self._wait(handle.get_result_async(), self.result_timeout, claim)
        # 결과와 관측은 서로 다른 DDS 경로다. 마지막 안전 위치 완료 관측을 짧게 수집한다.
        for _ in range(3):
            self._flush_feedback(claim)
            self.stop_event.wait(0.1)
        if result.status not in (
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        ):
            raise RuntimeError("TM 최종 상태가 확정되지 않았습니다.")
        succeeded = (
            result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
        )
        elapsed = float(result.result.elapsed_time_sec)
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("TM 소요 시간이 유효하지 않습니다.")
        self.backend.event(
            claim,
            "completed" if succeeded else "failed",
            progress_pct=100.0 if succeeded else 0.0,
            message=result.result.error_msg[:2048],
            error_code=result.result.error_code[:256],
            elapsed_time_sec=elapsed,
        )
        self.get_logger().info(
            f"주문 {claim['order_id']} 결과 기록: {'완료' if succeeded else '실패'}"
        )

    def _run(self):
        """TM이 준비됐을 때만 인수한다. 재시작 시 claimed 주문을 다시 가져오지 않는다."""
        while not self.stop_event.is_set():
            claim = None
            try:
                if not self.action_client.server_is_ready():
                    self.stop_event.wait(1.0)
                    continue
                claim = self.backend.claim()
                if claim is not None:
                    self.active_order_id = claim["order_id"]
                    self._dispatch(claim)
            except Exception as error:
                self.get_logger().error(str(error))
                if claim is not None:
                    try:
                        self.backend.event(
                            claim, "uncertain", message=str(error)[:2048]
                        )
                    except Exception:
                        self.get_logger().error(
                            "상태 기록 실패. DB 인수 잠금을 유지합니다. 수동 확인이 필요합니다."
                        )
            self.active_order_id = None
            self.stop_event.wait(1.0)

    def close(self):
        """종료는 자동 취소·홈 이동을 하지 않는다. 진행 중 주문은 확인 대기로 남긴다."""
        self.stop_event.set()
        if self.worker:
            self.worker.join(timeout=25.0)
        if self.monitor_worker:
            self.monitor_worker.join(timeout=5.0)
        self.action_client.destroy()


def read_contact_pose(node, service_name):
    """현재 활성 TCP의 BASE 위치를 읽기만 한다. 접촉은 사용자가 확인한 가정이다."""
    from dsr_msgs2.srv import GetCurrentPosx

    client = node.create_client(GetCurrentPosx, service_name)
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                "현재 TCP 조회 서비스가 없습니다. 같은 ROS 도메인과 브링업을 확인하세요."
            )
        future = client.call_async(GetCurrentPosx.Request(ref=0))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("현재 TCP 조회 응답이 없습니다.")
        response = future.result()
        if not response.success or not response.task_pos_info:
            raise RuntimeError("현재 TCP를 조회하지 못했습니다.")
        return validate_reference(list(response.task_pos_info[0].data[:6]))
    finally:
        node.destroy_client(client)


def main(args=None):
    """YES 후 영점을 읽고 브릿지만 실행한다. 드라이버·TM·로봇 동작은 자동 실행하지 않는다."""
    parser = argparse.ArgumentParser(
        description="현재 TCP를 케이크 상면 중심으로 기록하는 주문 브릿지"
    )
    parser.add_argument("--backend-url", default="http://127.0.0.1:5000")
    parser.add_argument("--action-name", default="/cake/order")
    parser.add_argument(
        "--pose-service", default="/dsr01/dsr_controller2/aux_control/get_current_posx"
    )
    parser.add_argument("--result-timeout", type=float, default=1800.0)
    parsed, ros_args = parser.parse_known_args(args)
    backend = BackendClient(parsed.backend_url, os.environ.get("CAKE_BRIDGE_TOKEN", ""))
    if not math.isfinite(parsed.result_timeout) or parsed.result_timeout <= 0:
        raise ValueError("최종 결과 대기 시간은 양수여야 합니다.")
    print(
        "활성 TCP가 펜촉에 교정돼 있어야 합니다. 수평 케이크의 윗면 중심에 가볍게 닿게 하세요."
    )
    print(
        "YES 후 현재 BASE TCP를 기록합니다. 브릿지가 주문을 TM에 보내면 TM이 로봇을 움직일 수 있습니다."
    )
    if input("상면 중심 접촉·TCP 확인 후 YES (그 외 종료): ").strip().upper() != "YES":
        return
    rclpy.init(args=ros_args)
    calibration = Node("cake_contact_reference")
    node = None
    try:
        pose = read_contact_pose(calibration, parsed.pose_service)
        print(f"케이크 윗면 중심 TCP [BASE mm, ZYZ deg]: {pose}")
        print(
            "이 영점은 이번 브릿지 실행 동안 고정합니다. 케이크를 옮겼다면 재실행해 영점을 기록하세요."
        )
        node = RosBridgeNode(backend, pose, parsed.action_name, parsed.result_timeout)
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        calibration.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
