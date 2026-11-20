"""단일 주문을 CakeOrder로 전달한다. 로봇 이동과 경로 계산은 수행하지 않는다."""

import argparse
import json
import math
import os
from queue import Empty, Full, Queue
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from dsr_msgs2.srv import MoveStop

from cake_decorator_interfaces.action import CakeOrder
from cake_decorator_interfaces.msg import TaskStatus
from types import SimpleNamespace

from .backend_client import BackendClient
from .goal_mapping import build_goal, validate_reference

# HMI 소프트웨어 정지 래치 토픽. True면 실행 노드가 이동·그리퍼 명령을 거부한다.
# TRANSIENT_LOCAL이라 실행 노드가 늦게 구독해도 마지막 상태를 받는다.
STOP_LATCH_TOPIC = "/cake/stop_latch"
# 드라이버의 즉시 정지 서비스. 드라이버가 별도 콜백 그룹으로 등록해
# 다른 동기 이동 명령이 실행 중이어도 처리된다.
MOVE_STOP_SERVICE = "/dsr01/dsr_controller2/motion/move_stop"
# Doosan stop_mode: DR_QSTOP(1) = Quick stop, Stop Category 2 (서보 유지).
DR_QSTOP = 1


def stop_latch_qos():
    """정지 래치 토픽 QoS. 브릿지·실행 노드가 같은 설정을 써야 매칭된다."""
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


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
        self.active_order_id = None
        self.plans = Queue(maxsize=1)
        self.last_tm_feedback_at = None
        self.last_command = ""
        self.monitor_worker = None
        self.plan_subscription = self.create_subscription(
            String,
            "/hmi/drawing_plan",
            self._on_plan,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.status_subscription = self.create_subscription(
            TaskStatus, "/task_status", self._on_status, 10
        )
        # ── HMI 소프트웨어 정지 ──
        # stop_latched: 정지 중이면 새 주문을 인수하지 않는다(작업 스레드끼리 공유).
        # active_goal_handle: 정지 시 취소할 진행 중 TM Goal. _dispatch가 갱신한다.
        self.stop_latched = False
        self.active_goal_handle = None
        self.stop_worker = None
        self.stop_publisher = self.create_publisher(
            Bool, STOP_LATCH_TOPIC, stop_latch_qos()
        )
        self.move_stop_client = self.create_client(MoveStop, MOVE_STOP_SERVICE)

    def start(self):
        """사용자가 영점을 확정한 뒤에만 주문 인수 루프를 시작한다."""
        self.worker = threading.Thread(
            target=self._run, name="cake-bridge-http", daemon=True
        )
        self.worker.start()
        self.monitor_worker = threading.Thread(
            target=self._monitor_loop, name="cake-bridge-monitor", daemon=True
        )
        self.monitor_worker.start()
        self.stop_worker = threading.Thread(
            target=self._stop_loop, name="cake-bridge-stop", daemon=True
        )
        self.stop_worker.start()
        self.get_logger().info(
            f"TM {self.action_name} 대기 중. 서버 준비 후에만 새 주문을 인수합니다."
        )

    def _monitor_loop(self):
        """주문 HTTP 대기와 독립적으로 TM 발견 여부·최근 피드백 나이를 전송한다."""
        failed = False
        while not self.stop_event.is_set():
            try:
                age = (
                    None
                    if self.last_tm_feedback_at is None
                    else max(0.0, time.monotonic() - self.last_tm_feedback_at)
                )
                self.backend.heartbeat(
                    self.action_client.server_is_ready(), self.active_order_id, age
                )
                failed = False
            except Exception as error:
                if not failed:
                    self.get_logger().warning(f"HMI 연결 관측 전송 실패: {error}")
                failed = True
            self.stop_event.wait(2.0)

    def _stop_loop(self):
        """HMI 정지 요청을 롱폴링으로 받아 즉시 적용한다. 주문 처리 스레드와 독립이다.

        처음에는 after=-1로 현재 상태를 받아 래치 토픽을 동기화한다.
        서버가 응답하지 않으면 0.5초 뒤 다시 연결하며, 실패 로그는 한 번만 남긴다.
        """
        after = -1
        failed = False
        while not self.stop_event.is_set():
            try:
                state = self.backend.wait_stop(after)
                failed = False
            except Exception as error:
                if not failed:
                    self.get_logger().warning(f"HMI 정지 요청 수신 실패: {error}")
                failed = True
                self.stop_event.wait(0.5)
                continue
            if state["seq"] == after:
                continue
            after = state["seq"]
            applied, message = self._apply_stop(bool(state["stopped"]))
            try:
                self.backend.stop_applied(after, applied, message)
            except Exception as error:
                self.get_logger().warning(f"HMI 정지 적용 결과 보고 실패: {error}")

    def _apply_stop(self, stopped):
        """정지(True)를 로봇 쪽에 적용하고 (성공 여부, 문구)를 반환한다.

        False는 HMI 해제 기능이 아니라 실행기 시작 시 초기 동기화(정지 아님)에만 온다.

        정지 순서
            1. 래치 토픽 True 발행: 실행 노드가 이후 이동·그리퍼 명령을 거부한다.
            2. 드라이버 move_stop(Quick stop): 현재 이동을 즉시 멈춘다.
            3. 진행 중 TM Goal 취소: 주문을 실패로 끝내 다음 단계를 진행하지 않는다.
        래치를 먼저 거는 이유는 move_stop 직후 실행 노드가 다음 이동을 보내는 경쟁을 막기 위해서다.
        """
        self.stop_latched = stopped
        self.stop_publisher.publish(Bool(data=stopped))
        if not stopped:
            return True, "정지 상태 아님"

        self.get_logger().warning("HMI 정지 요청 수신: 로봇 즉시 정지")
        stopped_ok = self._call_move_stop()
        handle = self.active_goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().error(f"진행 중 주문 취소 요청 실패: {error}")
        if stopped_ok:
            return True, "로봇 정지 명령 적용. 이동 명령 차단 중"
        return (
            False,
            "로봇 정지 명령 응답을 확인하지 못했습니다. 이동 명령은 차단했습니다. "
            "물리 E-stop을 사용하세요.",
        )

    def _call_move_stop(self, timeout=3.0):
        """드라이버 move_stop을 호출한다. spin은 메인 스레드가 하므로 future만 기다린다."""
        client = self.move_stop_client
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"정지 서비스 없음: {MOVE_STOP_SERVICE}")
            return False
        future = client.call_async(MoveStop.Request(stop_mode=DR_QSTOP))
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout) or future.result() is None:
            self.get_logger().error("정지 서비스 응답 시간 초과")
            return False
        if not future.result().success:
            self.get_logger().error("정지 서비스가 실패를 반환했습니다.")
            return False
        return True

    def _on_plan(self, message):
        """ROS 콜백에서는 계획을 큐에만 넣는다. HTTP 대기는 하지 않는다."""
        try:
            if len(message.data) > 2 * 1024 * 1024:
                raise ValueError("계획 메시지가 너무 큽니다.")
            plan = json.loads(message.data)
            if plan.get("order_id") != self.active_order_id or plan.get("version") != 1:
                return
            try:
                self.plans.get_nowait()
            except Empty:
                pass
            self.plans.put_nowait(plan)
        except (ValueError, TypeError, AttributeError, Full):
            self.get_logger().warning(
                "HMI 계획 메시지의 주문/형식을 확인할 수 없습니다."
            )

    def _flush_plan(self, claim):
        """표시용 계획 전송 실패는 로봇 주문의 실행 결과를 바꾸지 않는다."""
        plans = getattr(self, "plans", None)
        if plans is None:
            return
        try:
            plan = plans.get_nowait()
        except Empty:
            return
        if plan.get("order_id") != claim["order_id"]:
            return
        try:
            self.backend.plan(claim, plan)
        except Exception as error:
            self.get_logger().warning(f"HMI 계획 전송 실패: {error}")

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
        self.last_command = status.current_command
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
        self.last_tm_feedback_at = time.monotonic()
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
        self._flush_plan(claim)
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
            current_command=getattr(self, "last_command", "")[:128],
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
        # 정지 스레드가 취소할 수 있게 진행 중 Goal을 공개한다.
        self.active_goal_handle = handle
        try:
            # Goal 수락 응답을 기다리는 사이 정지가 들어왔다면 바로 취소한다.
            if self.stop_latched:
                handle.cancel_goal_async()
            result = self._wait(handle.get_result_async(), self.result_timeout, claim)
        finally:
            self.active_goal_handle = None
        self._flush_feedback(claim)
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
            # 최종 결과가 직전 공정 피드백을 덮을 때 단계·문구를 비우지 않는다.
            phase="COMPLETED" if succeeded else "FAILED",
            progress_pct=100.0 if succeeded else 0.0,
            message=(
                result.result.error_msg
                or ("TM 작업 완료" if succeeded else "TM 작업 실패")
            )[:2048],
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
                # HMI 정지 중이거나 TM이 준비되지 않았으면 주문을 인수하지 않는다.
                if self.stop_latched or not self.action_client.server_is_ready():
                    self.stop_event.wait(1.0)
                    continue
                claim = self.backend.claim()
                if claim is not None:
                    self.active_order_id = claim["order_id"]
                    self.last_command = ""
                    self.last_tm_feedback_at = None
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
            self.monitor_worker.join(timeout=4.0)
        # 정지 스레드는 롱폴링 중일 수 있어 오래 기다리지 않는다(daemon 스레드).
        if self.stop_worker:
            self.stop_worker.join(timeout=1.0)
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
    parser.add_argument(
        "--execution-mode",
        choices=("unknown", "real", "simulation"),
        default="unknown",
        help="운영자가 선택한 운전 설정. 장비 연결/안전 확인 신호가 아닙니다.",
    )
    parsed, ros_args = parser.parse_known_args(args)
    backend = BackendClient(
        parsed.backend_url,
        os.environ.get("CAKE_BRIDGE_TOKEN", ""),
        execution_mode=parsed.execution_mode,
    )
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
