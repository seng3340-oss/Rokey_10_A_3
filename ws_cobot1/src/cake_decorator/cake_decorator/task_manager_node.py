from __future__ import annotations

from threading import Event
import time
import json
import math
import traceback
from enum import Enum
from typing import Optional

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from .hmi_plan import build_hmi_plan

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose, Point, Quaternion

from cake_decorator_interfaces.action import (
    CakeOrder,
    DrawOnCake,
)
from cake_decorator_interfaces.msg import (
    TaskStatus,
)
from cake_decorator_interfaces.srv import GenerateTrajectory


class Phase(str, Enum):
    DRAWING = 'DRAWING'
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'


class TaskManagerNode(Node):
    """드로잉 전용 오케스트레이터."""

    def __init__(self):
        super().__init__('task_manager_node')

        # ── 파라미터 ──
        self.declare_parameter('drawing_speed', 20.0)     # mm/s
        self.declare_parameter('contact_force', 1.0)      # N
        # DP로 넘길 작업 속도 퍼센트. 0을 그대로 보내면 DP가 1%로
        # 계산해 드로잉 속도가 1/100로 떨어지므로 기본 100을 쓴다.
        self.declare_parameter('operation_speed_percent', 100)
        self.declare_parameter('action_timeout', 120.0)   # 초
        self.declare_parameter('service_timeout', 30.0)   # 초

        # ── 콜백 그룹 ──
        self._cb = ReentrantCallbackGroup()

        # ── CakeOrder 액션 서버 (ros_bridge에서 호출) ──
        self._order_server = ActionServer(
            self,
            CakeOrder,
            'cake/order',
            execute_callback=self._execute_order,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=self._cb,
        )

        # ── DrawOnCake 액션 클라이언트 ──
        self._draw_client = ActionClient(
            self, DrawOnCake, 'draw_on_cake',
            callback_group=self._cb,
        )

        # ── GenerateTrajectory 서비스 클라이언트 ──
        self._gen_traj_client = self.create_client(
            GenerateTrajectory, 'generate_trajectory',
            callback_group=self._cb,
        )

        # ── TaskStatus 퍼블리셔 ──
        self._status_pub = self.create_publisher(
            TaskStatus, '/task_status', 10,
        )

        # ── 내부 상태 ──
        self._current_phase = Phase.DRAWING
        self._current_order_id: Optional[str] = None
        self._current_progress_pct = 0.0
        self._is_busy = False
        self._current_command = ""
        self._last_status_message = ""
        self._drawing_error = None
        self._plan_pub = self.create_publisher(
            String,
            "/hmi/drawing_plan",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_timer(1.0, self._status_heartbeat, callback_group=self._cb)

        self.get_logger().info('task_manager_node 시작 (드로잉 전용)')

    # ── Goal 콜백 ──

    def _handle_goal(self, goal_request) -> GoalResponse:
        if self._is_busy:
            self.get_logger().warn(
                f'주문 거절: 작업 진행 중 (order={self._current_order_id})'
            )
            return GoalResponse.REJECT
        self.get_logger().info(f'주문 수락: order_id={goal_request.order_id}')
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info(f'취소 요청: order_id={self._current_order_id}')
        return CancelResponse.ACCEPT

    # ── 메인 실행 (액션 서버 execute_callback) ──

    def _execute_order(self, goal_handle) -> CakeOrder.Result:
        """Goal 수락 후 자동 호출되는 진입점. 드로잉을 실행하고 결과를 반환."""
        request = goal_handle.request
        self._current_phase = Phase.DRAWING
        self._current_progress_pct = 0.0
        self._last_status_message = "주문 준비"
        self._current_order_id = request.order_id
        self._is_busy = True
        self._drawing_error = None
        self._current_command = ""
        result = CakeOrder.Result()
        start_time = time.time()

        self.get_logger().info(
            f'=== 주문 시작: {request.order_id} ===\n'
            f'  drawing_paths: {len(request.drawing_paths)}개\n'
            f'  cake: r={request.cake_params.radius}, '
            f'h={request.cake_params.height}'
        )

        try:
            if self._check_cancel(goal_handle):
                return self._cancel(goal_handle, result, start_time)

            self._set_phase(Phase.DRAWING, goal_handle, 0.0, '드로잉 시작')

            ok = self._execute_drawing(request, goal_handle)
            if not ok:
                code, message = self._drawing_error or ("DRAWING_FAILED", "드로잉 실패")
                return self._fail(goal_handle, result, code, message, start_time)

            # ── 완료 ──
            self._set_phase(Phase.COMPLETED, goal_handle, 100.0, '완료')

            elapsed = time.time() - start_time
            result.success = True
            result.error_code = ''
            result.error_msg = ''
            result.elapsed_time_sec = elapsed
            goal_handle.succeed()

            self.get_logger().info(
                f'=== 주문 완료: {request.order_id} ({elapsed:.1f}초) ==='
            )
            return result

        except Exception as exc:
            self.get_logger().error(
                f'예외 발생: {exc}\n{traceback.format_exc()}'
            )
            return self._fail(goal_handle, result, 'EXCEPTION',
                              str(exc), start_time)
        finally:
            self._is_busy = False
            self._current_order_id = None

    # ── 드로잉 실행 (GenerateTrajectory → DrawOnCake) ──

    def _execute_drawing(self, request: CakeOrder.Goal, goal_handle) -> bool:
        cake = request.cake_params

        cake_pose = Pose()
        cake_pose.position = Point(
            x=cake.center.x, y=cake.center.y, z=cake.center.z,
        )
        cake_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        total_paths = len(request.drawing_paths)
        drawing_speed = self.get_parameter('drawing_speed').value
        contact_force = self.get_parameter('contact_force').value

        for idx, dp in enumerate(request.drawing_paths):
            if not dp.strokes:
                self.get_logger().info(f'DrawingPath[{idx}] strokes 비어 있음 — 건너뜀')
                continue

            if self._check_cancel(goal_handle):
                return False

            self.get_logger().info(
                f'DrawingPath[{idx+1}/{total_paths}]: '
                f'surface={dp.surface}, strokes={len(dp.strokes)}'
            )

            # 1) GenerateTrajectory 서비스 호출
            srv_timeout = self.get_parameter('service_timeout').value
            if not self._gen_traj_client.wait_for_service(timeout_sec=srv_timeout):
                self._drawing_error = (
                    "PATH_SERVICE_UNAVAILABLE",
                    "generate_trajectory 서비스 연결 실패",
                )
                self.get_logger().error('generate_trajectory 서비스 연결 실패')
                return False

            traj_req = GenerateTrajectory.Request()
            # 주문번호는 DP 로그에서 획과 함께 주문을 추적하기 위한 식별자다.
            traj_req.order_id = request.order_id
            # 속도 퍼센트는 기본 100을 쓴다. 비워두면 DP가 0%로 계산한다.
            traj_req.operation_speed_percent = int(
                self.get_parameter('operation_speed_percent').value
            )
            # canvas_*는 uint32, DP 요청의 canvas_*는 float64다. 정수를 그대로
            # 대입하면 런타임에 값이 int로 남았다가 서비스 직렬화 중 프로세스가
            # abort되므로 float로 바꿔 넘긴다.
            traj_req.canvas_width = float(dp.canvas_width)
            traj_req.canvas_height = float(dp.canvas_height)
            # 원본 픽셀 획을 순서·점 개수 그대로 넘긴다. DP에서 변환을 수행한다.
            traj_req.strokes_2d = dp.strokes
            traj_req.surface_type = dp.surface
            traj_req.cake_pose = cake_pose
            traj_req.cake_radius = cake.radius
            traj_req.cake_height = cake.height

            future = self._gen_traj_client.call_async(traj_req)
            self._wait_for_result(future, srv_timeout)

            if future.result() is None:
                self._drawing_error = (
                    "PATH_SERVICE_TIMEOUT",
                    f"GenerateTrajectory 응답 없음 (path={idx})",
                )
                self.get_logger().error(f'GenerateTrajectory 호출 실패 (path={idx})')
                return False

            resp = future.result()
            if not resp.success:
                self._drawing_error = (
                    "PATH_GENERATION_FAILED",
                    str(resp.error_msg)[:2048],
                )
                self.get_logger().error(f'궤적 생성 실패: {resp.error_msg}')
                return False

            # 펜이 처음 내려간 지점 수 = 실제로 그려질 획 수.
            # 첫 획만 만들어지는 문제를 로그에서 바로 알 수 있게 센다.
            stroke_count = sum(
                1
                for i, wp in enumerate(resp.waypoints_3d)
                if wp.pen_down and (i == 0 or not resp.waypoints_3d[i - 1].pen_down)
            )
            self.get_logger().info(
                f'궤적 생성 완료: waypoints={len(resp.waypoints_3d)}, '
                f'strokes={stroke_count}'
            )

            # HMI에는 실제 DP 응답의 사본만 보낸다. 표시 실패가 로봇 경로를 변경하지 않는다.
            try:
                plan = build_hmi_plan(request.order_id, dp, cake, resp.waypoints_3d)
                self._plan_pub.publish(String(data=json.dumps(plan, allow_nan=False)))
            except Exception as error:
                self.get_logger().warning(f"HMI 계획 생성 실패: {error}")

            # 2) DrawOnCake 액션 호출
            draw_goal = DrawOnCake.Goal()
            draw_goal.waypoints = resp.waypoints_3d
            draw_goal.drawing_surface = dp.surface
            draw_goal.contact_force = contact_force
            draw_goal.speed = drawing_speed

            ok = self._send_draw(draw_goal, goal_handle, idx, total_paths)
            if not ok:
                return False

            # 진행률
            pct = 100.0 * (idx + 1) / total_paths
            self._publish_feedback(goal_handle, Phase.DRAWING, pct,
                                   f'DrawingPath[{idx+1}/{total_paths}] 완료')

        return True

    # ── DrawOnCake 액션 전송 ──

    def _send_draw(self, goal, goal_handle, path_index=0, total_paths=1) -> bool:
        timeout = self.get_parameter('action_timeout').value

        if not self._draw_client.wait_for_server(timeout_sec=10.0):
            self._drawing_error = (
                "DRAW_SERVER_UNAVAILABLE",
                "DrawOnCake 서버 연결 실패",
            )
            self.get_logger().error('DrawOnCake 서버 연결 실패')
            return False

        self.get_logger().info('DrawOnCake Goal 전송')

        send_future = self._draw_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._on_draw_feedback(
                fb, goal_handle, path_index, total_paths
            ),
        )
        self._wait_for_result(send_future, 10.0)

        gh = send_future.result()
        if gh is None or not gh.accepted:
            self._drawing_error = ("DRAW_GOAL_REJECTED", "DrawOnCake Goal 거절/실패")
            self.get_logger().error('DrawOnCake Goal 거절/실패')
            return False

        result_future = gh.get_result_async()
        self._wait_for_result(result_future, timeout)

        if result_future.result() is None:
            self._drawing_error = ("DRAW_RESULT_TIMEOUT", "DrawOnCake 결과 수신 실패")
            self.get_logger().error('DrawOnCake 결과 수신 실패 (timeout)')
            return False

        action_result = result_future.result().result
        if not action_result.success:
            code = getattr(action_result, "error_code", "") or "DRAWING_FAILED"
            self._drawing_error = (code[:256], f"DrawOnCake 실패: {code}"[:2048])
            self.get_logger().error(
                f'DrawOnCake 실패: {getattr(action_result, "error_code", "")}'
            )
            return False

        self.get_logger().info('DrawOnCake 성공')
        return True

    @staticmethod
    def _wait_for_result(future, timeout):
        """다른 executor 스레드가 응답을 처리하도록 콜백 안에서는 대기만 한다."""
        completed = Event()
        future.add_done_callback(lambda _: completed.set())
        if not future.done() and not completed.wait(timeout):
            return None
        return future.result()

    def _on_draw_feedback(self, feedback_msg, goal_handle, path_index=0, total_paths=1):
        """RE의 면별 진행률을 주문 드로잉 구간으로 환산해 두 상태 채널에 전달한다.

        장비 명령을 추가하지 않는다. 이 로컬 버전은 드로잉 공정만 포함하며,
        유한하지 않은 피드백이나 다른 주문의 늦은 응답은 화면 상태에 반영하지 않는다.
        """
        if goal_handle.request.order_id != self._current_order_id:
            return
        fb = feedback_msg.feedback
        progress = float(fb.progress_pct)
        if not math.isfinite(progress) or total_paths <= 0:
            return
        self._current_command = fb.phase[:128]
        drawing_limit = 100.0
        pct = drawing_limit * (
            path_index + max(0.0, min(100.0, progress)) / 100.0
        ) / total_paths
        message = (
            f"DrawingPath[{path_index + 1}/{total_paths}] {fb.phase} · "
            f"획 {fb.current_stroke_idx}/{fb.total_strokes}"
        )
        self._publish_feedback(goal_handle, Phase.DRAWING, pct, message)

    # ── Phase / Feedback / Status ──

    def _set_phase(self, phase, goal_handle, pct, msg):
        self._current_phase = phase
        self.get_logger().info(f'[{phase.value}] {msg} ({pct:.0f}%)')
        self._publish_feedback(goal_handle, phase, pct, msg)

    def _publish_feedback(self, goal_handle, phase, pct, msg):
        """CakeOrder 피드백과 Topic에 같은 공정 값을 발행한다."""
        self._current_phase = phase
        self._current_progress_pct = float(pct)
        self._last_status_message = msg
        if phase != Phase.DRAWING:
            self._current_command = ""
        fb = CakeOrder.Feedback()
        fb.current_phase = phase.value
        fb.progress_pct = float(pct)
        fb.message = msg
        goal_handle.publish_feedback(fb)
        self._publish_status(msg)

    def _publish_status(self, msg):
        status = TaskStatus()
        status.order_id = self._current_order_id or ''
        status.phase = self._current_phase.value
        status.progress_pct = self._current_progress_pct
        status.current_command = getattr(self, "_current_command", "")
        status.message = msg
        now = self.get_clock().now()
        status.timestamp = TimeMsg(
            sec=now.seconds_nanoseconds()[0],
            nanosec=now.seconds_nanoseconds()[1],
        )
        self._status_pub.publish(status)

    def _status_heartbeat(self):
        """작업 대기 중에도 마지막 TM 상태를 재발행한다. 진행률·명령을 새로 만들지 않는다."""
        if self._current_order_id:
            self._publish_status(self._last_status_message)

    # ── 취소 / 실패 ──

    def _check_cancel(self, goal_handle) -> bool:
        return goal_handle.is_cancel_requested

    def _fail(self, goal_handle, result, code, msg, start_time):
        self._set_phase(Phase.FAILED, goal_handle, 0.0, msg)
        result.success = False
        result.error_code = code
        result.error_msg = msg
        result.elapsed_time_sec = time.time() - start_time
        goal_handle.abort()
        self.get_logger().error(f'실패: [{code}] {msg}')
        return result

    def _cancel(self, goal_handle, result, start_time):
        self._set_phase(Phase.CANCELED, goal_handle, 0.0, '취소됨')
        result.success = False
        result.error_code = 'CANCELED'
        result.error_msg = '사용자 취소'
        result.elapsed_time_sec = time.time() - start_time
        goal_handle.canceled()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
