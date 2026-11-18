from __future__ import annotations

import time
import threading
import traceback
from enum import Enum
from typing import Optional

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

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
        self.declare_parameter('drawing_speed', 30.0)     # mm/s, DP 기본값과 통일
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
            '/cake/order',
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
        self._is_busy = False

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
        self._current_order_id = request.order_id
        self._is_busy = True
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
                return self._fail(goal_handle, result, 'DRAWING_FAILED',
                                  '드로잉 실패', start_time)

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

    @staticmethod
    def _reference_pose(cake):
        """브릿지가 YES 시점에 기록한 펜촉 위치·자세를 DP까지 보존한다."""
        return Pose(position=cake.center, orientation=cake.tool_orientation)

    def _execute_drawing(self, request: CakeOrder.Goal, goal_handle) -> bool:
        cake = request.cake_params
        cake_pose = self._reference_pose(cake)

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
            # 펜 자세는 cake_pose에 보존하고 케이크 평면 회전은 별도로 보낸다.
            traj_req.cake_yaw = cake.orientation
            traj_req.cake_radius = cake.radius
            traj_req.cake_bottom_radius = cake.bottom_radius
            traj_req.cake_height = cake.height

            future = self._gen_traj_client.call_async(traj_req)
            self._wait_future(future, srv_timeout)

            if future.result() is None:
                self.get_logger().error(f'GenerateTrajectory 호출 실패 (path={idx})')
                return False

            resp = future.result()
            if not resp.success:
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

            # 2) DrawOnCake 액션 호출
            draw_goal = DrawOnCake.Goal()
            draw_goal.waypoints = resp.waypoints_3d
            draw_goal.drawing_surface = dp.surface
            draw_goal.contact_force = contact_force
            draw_goal.speed = drawing_speed

            ok = self._send_draw(draw_goal, goal_handle)
            if not ok:
                return False

            # 진행률
            pct = 100.0 * (idx + 1) / total_paths
            self._publish_feedback(goal_handle, Phase.DRAWING, pct,
                                   f'DrawingPath[{idx+1}/{total_paths}] 완료')

        return True

    # ── DrawOnCake 액션 전송 ──

    @staticmethod
    def _wait_future(future, timeout):
        """다른 executor 스레드가 응답을 처리하게 하고 현재 콜백만 대기한다."""
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(timeout):
            raise TimeoutError("ROS 응답 대기 시간 초과: 작업 상태를 확인하세요.")
        if future.exception():
            raise future.exception()

    def _send_draw(self, goal, goal_handle) -> bool:
        timeout = self.get_parameter('action_timeout').value

        if not self._draw_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('DrawOnCake 서버 연결 실패')
            return False

        self.get_logger().info('DrawOnCake Goal 전송')

        send_future = self._draw_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._on_draw_feedback(fb, goal_handle),
        )
        self._wait_future(send_future, 10.0)

        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('DrawOnCake Goal 거절/실패')
            return False

        result_future = gh.get_result_async()
        self._wait_future(result_future, timeout)

        if result_future.result() is None:
            self.get_logger().error('DrawOnCake 결과 수신 실패 (timeout)')
            return False

        action_result = result_future.result().result
        if not action_result.success:
            self.get_logger().error(
                f'DrawOnCake 실패: {getattr(action_result, "error_code", "")}'
            )
            return False

        self.get_logger().info('DrawOnCake 성공')
        return True

    def _on_draw_feedback(self, feedback_msg, goal_handle):
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f'DrawOnCake feedback: phase={getattr(fb, "phase", "")}, '
            f'progress={getattr(fb, "progress_pct", 0):.1f}%'
        )

    # ── Phase / Feedback / Status ──

    def _set_phase(self, phase, goal_handle, pct, msg):
        self._current_phase = phase
        self.get_logger().info(f'[{phase.value}] {msg} ({pct:.0f}%)')
        self._publish_feedback(goal_handle, phase, pct, msg)
        self._publish_status(msg)

    def _publish_feedback(self, goal_handle, phase, pct, msg):
        fb = CakeOrder.Feedback()
        fb.current_phase = phase.value
        fb.progress_pct = float(pct)
        fb.message = msg
        goal_handle.publish_feedback(fb)

    def _publish_status(self, msg):
        status = TaskStatus()
        status.order_id = self._current_order_id or ''
        status.phase = self._current_phase.value
        status.progress_pct = 0.0
        status.current_command = ''
        status.message = msg
        now = self.get_clock().now()
        status.timestamp = TimeMsg(
            sec=now.seconds_nanoseconds()[0],
            nanosec=now.seconds_nanoseconds()[1],
        )
        self._status_pub.publish(status)

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
