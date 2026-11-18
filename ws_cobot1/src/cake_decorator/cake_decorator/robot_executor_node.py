"""로봇 실행 노드 — 전체 작업 상태 머신 + 그리기·운반·펜 파지 액션 서버.

TM(task_manager)에서 ExecuteTask Goal을 보내면 전체 작업 흐름을
상태 머신으로 관리하고, 각 단계에서 Doosan M0609를 실제로 움직인다.

작업 상태 흐름:
  IDLE → RECEIVED → VALIDATING → PICKING_CAKE → PLACING_CAKE
  → GRASPING_TOOL → DRAWING → RETURNING_TOOL → DELIVERING
  → COMPLETED / FAILED / CANCELED
"""

from __future__ import annotations

import traceback

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cake_decorator_interfaces.action import DrawOnCake, GraspTool, PickAndPlace

from .doosan_robot import DoosanRobotClient
from .drawing_motion import (
    DsrPos,
    TaskState,
    can_transition,
    get_substeps,
    plan_drawing_motion,
    pose_msg_to_dsr,
)


class RobotExecutorNode(Node):
    """그리기·운반·펜 파지 + 전체 작업 상태 머신 노드."""

    # 파라미터 기본값
    _PARAM_DEFAULTS = {
        'service_timeout': 5.0,
        'pen_grip_width': 10.0,    # mm
        'pen_grip_force': 20.0,    # N
        'gripper_open_width': 100.0,  # mm
        'approach_height': 0.05,   # m
        'virtual_initial_approach': False,  # 실행기가 가상 모드에서만 활성화
    }

    def __init__(self):
        super().__init__('robot_executor_node')

        # 파라미터 선언
        for name, default in self._PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        # Doosan + 그리퍼 클라이언트
        self._robot = DoosanRobotClient(
            self,
            service_timeout=self._param('service_timeout'),
        )

        # 콜백 그룹
        self._cb = ReentrantCallbackGroup()

        # ── 상태 머신 ──
        self._state = TaskState.IDLE
        self._cancel_requested = False

        # --- 액션 서버 일괄 등록 ---
        actions = [
            (DrawOnCake,  'draw_on_cake',  self._execute_draw),
            (GraspTool,   'grasp_tool',    self._execute_grasp),
            (PickAndPlace, 'pick_and_place', self._execute_pick_place),
        ]
        self._action_servers = []
        for action_type, name, callback in actions:
            srv = ActionServer(
                self, action_type, name,
                execute_callback=callback,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._cb,
            )
            self._action_servers.append(srv)

        self.get_logger().info(
            'robot_executor_node 시작 — 상태 머신 + 액션 서버 대기 중'
        )

    # ------------------------------------------------------------------
    # 파라미터 헬퍼
    # ------------------------------------------------------------------

    def _param(self, name: str):
        """파라미터 값을 간결하게 가져온다."""
        return self.get_parameter(name).value

    # ==================================================================
    # 상태 머신 관리
    # ==================================================================

    @property
    def current_state(self) -> TaskState:
        return self._state

    def _transition_to(self, target: TaskState) -> bool:
        """상태 전이를 시도한다."""
        if not can_transition(self._state, target):
            self.get_logger().error(
                f'상태 전이 불가: {self._state.value} → {target.value}'
            )
            return False
        prev = self._state
        self._state = target
        self.get_logger().info(f'상태 전이: {prev.value} → {target.value}')
        return True

    def _fail(self, reason: str) -> None:
        """FAILED 상태로 전이 + 안전 홈 복귀."""
        self.get_logger().error(f'작업 실패: {reason}')
        if can_transition(self._state, TaskState.FAILED):
            self._state = TaskState.FAILED
        try:
            self._robot.move_home()
        except Exception:
            pass

    def _cancel(self) -> None:
        """CANCELED 상태로 전이."""
        self.get_logger().info('작업 취소됨')
        if can_transition(self._state, TaskState.CANCELED):
            self._state = TaskState.CANCELED

    def _reset_to_idle(self) -> None:
        """종료/실패/취소 후 IDLE로 복귀."""
        if self._state in (
            TaskState.COMPLETED, TaskState.FAILED, TaskState.CANCELED,
        ):
            self._state = TaskState.IDLE
            self._cancel_requested = False
            self.get_logger().info('IDLE 상태로 복귀')

    def _check_cancel(self, goal_handle) -> bool:
        """취소 요청 확인."""
        if goal_handle.is_cancel_requested or self._cancel_requested:
            self._cancel()
            return True
        return False

    # ------------------------------------------------------------------
    # 결과 생성
    # ------------------------------------------------------------------

    def _result(self, success: bool, error_code: str = '') -> dict:
        """결과 딕셔너리를 생성한다."""
        return {
            'success': success,
            'error_code': error_code,
            'state_at_failure': '' if success else self._state.value,
        }

    # ==================================================================
    # Pose → DsrPos 변환 + 접근/목표 쌍 헬퍼
    # ==================================================================

    def _approach_pair(self, pose) -> tuple[DsrPos, DsrPos]:
        """geometry_msgs/Pose → (접근 위치, 목표 위치) 쌍을 반환한다.

        접근 위치 = 목표 위치에서 approach_height만큼 위.
        """
        h = self._param('approach_height')
        above = pose_msg_to_dsr(pose, z_offset_m=h)
        target = pose_msg_to_dsr(pose)
        return above, target

    # ==================================================================
    # 전체 작업 실행 (상태 머신)
    # ==================================================================

    def execute_full_task(self, goal_handle, task_goal) -> dict:
        """전체 작업 흐름을 상태 머신으로 실행한다.

        TM(task_manager) 노드에서 호출하는 메서드.
        """
        self._cancel_requested = False

        # IDLE → RECEIVED
        if not self._transition_to(TaskState.RECEIVED):
            return self._result(False, 'NOT_IDLE')

        # 상태별 실행 핸들러
        steps = [
            (self._do_validating,     'VALIDATION_FAILED'),
            (self._do_picking_cake,   'PICKING_CAKE_FAILED'),
            (self._do_placing_cake,   'PLACING_CAKE_FAILED'),
            (self._do_grasping_tool,  'GRASPING_TOOL_FAILED'),
            (self._do_drawing,        'DRAWING_FAILED'),
            (self._do_returning_tool, 'RETURNING_TOOL_FAILED'),
            (self._do_delivering,     'DELIVERING_FAILED'),
        ]

        try:
            for step_fn, err_code in steps:
                if not step_fn(goal_handle, task_goal):
                    return self._result(False, err_code)
                if self._check_cancel(goal_handle):
                    return self._result(False, 'CANCELED')

            self._transition_to(TaskState.COMPLETED)
            return self._result(True)

        except Exception as exc:
            self.get_logger().error(
                f'작업 중 예외: {exc}\n{traceback.format_exc()}'
            )
            self._fail(str(exc))
            return self._result(False, f'EXCEPTION: {exc}')

    # ------------------------------------------------------------------
    # 각 상태별 실행 로직
    # ------------------------------------------------------------------

    def _do_validating(self, goal_handle, task_goal) -> bool:
        """VALIDATING: 경로 데이터 검증, 로봇 상태 확인."""
        if not self._transition_to(TaskState.VALIDATING):
            return False

        self.get_logger().info(f'검증 단계: {get_substeps(TaskState.VALIDATING)}')

        # 1. 경로 데이터 검증
        if not hasattr(task_goal, 'waypoints') or not task_goal.waypoints:
            self._fail('경로 데이터가 비어 있습니다.')
            return False

        # 2. 로봇 서비스 연결 확인
        if not self._robot.wait_for_services(timeout=5.0):
            self._fail('로봇 서비스 연결 실패')
            return False

        # 3. 교정치 확인 — 차후 센서 데이터와 연동
        # 4. 재료/도구 확인 — 차후 센서 데이터와 연동
        self.get_logger().info('검증 완료')
        return True

    def _do_picking_cake(self, goal_handle, task_goal) -> bool:
        """PICKING_CAKE: 케이크 집기."""
        if not self._transition_to(TaskState.PICKING_CAKE):
            return False

        self.get_logger().info(
            f'케이크 집기: {get_substeps(TaskState.PICKING_CAKE)}'
        )

        if not hasattr(task_goal, 'cake_pick_pose'):
            self.get_logger().warn('cake_pick_pose 없음 — 건너뜀')
            return True

        above, target = self._approach_pair(task_goal.cake_pick_pose)

        if not self._robot.pick(
            target=target,
            approach=above,
            grip_width=self._param('gripper_open_width') - 20.0,
            open_width=self._param('gripper_open_width'),
        ):
            self._fail('케이크 집기 실패')
            return False

        self.get_logger().info('케이크 집기 완료')
        return True

    def _do_placing_cake(self, goal_handle, task_goal) -> bool:
        """PLACING_CAKE: 케이크 작업대 배치."""
        if not self._transition_to(TaskState.PLACING_CAKE):
            return False

        self.get_logger().info(
            f'케이크 배치: {get_substeps(TaskState.PLACING_CAKE)}'
        )

        if not hasattr(task_goal, 'cake_place_pose'):
            self.get_logger().warn('cake_place_pose 없음 — 건너뜀')
            return True

        above, target = self._approach_pair(task_goal.cake_place_pose)

        if not self._robot.place(
            target=target,
            approach=above,
            open_width=self._param('gripper_open_width'),
        ):
            self._fail('케이크 배치 실패')
            return False

        self.get_logger().info('케이크 배치 완료')
        return True

    def _do_grasping_tool(self, goal_handle, task_goal) -> bool:
        """GRASPING_TOOL: 펜/도구 집기 (TCP 전환 포함)."""
        if not self._transition_to(TaskState.GRASPING_TOOL):
            return False

        self.get_logger().info(
            f'도구 집기: {get_substeps(TaskState.GRASPING_TOOL)}'
        )

        if not hasattr(task_goal, 'tool_pose'):
            self.get_logger().warn('tool_pose 없음 — 건너뜀')
            return True

        # 1. TCP 전환
        # TODO: DSR TCP 전환 서비스 호출

        # 2~4. 도구 접근 → 파지 → 후퇴
        above = pose_msg_to_dsr(task_goal.tool_pose, z_offset_m=0.05)
        target = pose_msg_to_dsr(task_goal.tool_pose)

        if not self._robot.pick(
            target=target,
            approach=above,
            grip_width=self._param('pen_grip_width'),
            grip_force=self._param('pen_grip_force'),
            open_width=self._param('gripper_open_width'),
            speed_approach=50.0,
        ):
            self._fail('도구 집기 실패')
            return False

        self.get_logger().info('도구 집기 완료')
        return True

    def _do_drawing(self, goal_handle, task_goal) -> bool:
        """DRAWING: 케이크 위에 레터링 드로잉."""
        if not self._transition_to(TaskState.DRAWING):
            return False

        self.get_logger().info(
            f'드로잉 시작: {get_substeps(TaskState.DRAWING)}'
        )

        # 웨이포인트 → 딕셔너리 변환
        waypoints_dict, pen_states = self._convert_waypoints(task_goal.waypoints)

        surface = getattr(task_goal, 'drawing_surface', 'TOP')
        plan = plan_drawing_motion(waypoints_dict, pen_states, surface)

        if not plan.success:
            self._fail(f'동작 계획 실패: {plan.error_msg}')
            return False

        # 명령 실행
        total = len(plan.commands)
        for i, cmd in enumerate(plan.commands):
            if self._check_cancel(goal_handle):
                return False

            self.get_logger().info(
                f'드로잉 {i + 1}/{total}: '
                f'{cmd.phase.value} pos={cmd.pos.to_list()}'
            )

            if not self._execute_motion_command(cmd, i):
                self._fail(f'{cmd.motion_type.value} 실패 (#{i})')
                return False

        self.get_logger().info('드로잉 완료')
        return True

    def _do_returning_tool(self, goal_handle, task_goal) -> bool:
        """RETURNING_TOOL: 도구를 거치대에 반납."""
        if not self._transition_to(TaskState.RETURNING_TOOL):
            return False

        self.get_logger().info(
            f'도구 반납: {get_substeps(TaskState.RETURNING_TOOL)}'
        )

        if not hasattr(task_goal, 'tool_pose'):
            self.get_logger().warn('tool_pose 없음 — 건너뜀')
            return True

        above = pose_msg_to_dsr(task_goal.tool_pose, z_offset_m=0.05)
        target = pose_msg_to_dsr(task_goal.tool_pose)

        if not self._robot.place(
            target=target,
            approach=above,
            open_width=self._param('gripper_open_width'),
            speed_approach=50.0,
        ):
            self._fail('도구 반납 실패')
            return False

        # TCP 복원
        # TODO: DSR TCP 원래 값으로 복원

        self.get_logger().info('도구 반납 완료')
        return True

    def _do_delivering(self, goal_handle, task_goal) -> bool:
        """DELIVERING: 케이크를 출고 위치로 이동."""
        if not self._transition_to(TaskState.DELIVERING):
            return False

        self.get_logger().info(
            f'출고 배달: {get_substeps(TaskState.DELIVERING)}'
        )

        if not hasattr(task_goal, 'delivery_pose'):
            self.get_logger().warn('delivery_pose 없음 — 건너뜀')
            return True

        open_w = self._param('gripper_open_width')

        # 1. 작업대에서 케이크 집기
        if hasattr(task_goal, 'cake_place_pose'):
            above, target = self._approach_pair(task_goal.cake_place_pose)
            if not self._robot.pick(
                target=target,
                approach=above,
                grip_width=open_w - 20.0,
                open_width=open_w,
            ):
                self._fail('출고 집기 실패')
                return False

        # 2. 출고 위치에 배치
        above, target = self._approach_pair(task_goal.delivery_pose)
        if not self._robot.place(
            target=target,
            approach=above,
            open_width=open_w,
        ):
            self._fail('출고 배치 실패')
            return False

        self.get_logger().info('출고 배달 완료')
        return True

    # ==================================================================
    # 웨이포인트 변환 유틸
    # ==================================================================

    @staticmethod
    def _convert_waypoints(
        waypoints,
        default_speed: float = 20.0,
    ) -> tuple[list[dict], list[bool]]:
        """ROS Waypoint 메시지 리스트 → (dict 리스트, pen_states)."""
        dicts = []
        pen_states = []
        for wp in waypoints:
            dicts.append({
                'x': wp.pose.position.x,
                'y': wp.pose.position.y,
                'z': wp.pose.position.z,
                'qx': wp.pose.orientation.x,
                'qy': wp.pose.orientation.y,
                'qz': wp.pose.orientation.z,
                'qw': wp.pose.orientation.w,
                'speed': wp.speed if wp.speed > 0 else default_speed,
                'pen_down': wp.pen_down,
                'contact_force': wp.contact_force,
                # DP 원호 정보를 전달한다. 획의 완료·중단 판단은 변경하지 않는다.
                'motion_type': wp.motion_type,
                'spline': [
                    {
                        'x': p.position.x,
                        'y': p.position.y,
                        'z': p.position.z,
                        'qx': p.orientation.x,
                        'qy': p.orientation.y,
                        'qz': p.orientation.z,
                        'qw': p.orientation.w,
                    }
                    for p in wp.spline_poses
                ],
                'via': {
                    'x': wp.via_pose.position.x,
                    'y': wp.via_pose.position.y,
                    'z': wp.via_pose.position.z,
                    'qx': wp.via_pose.orientation.x,
                    'qy': wp.via_pose.orientation.y,
                    'qz': wp.via_pose.orientation.z,
                    'qw': wp.via_pose.orientation.w,
                },
            })
            pen_states.append(wp.pen_down)
        return dicts, pen_states

    # ==================================================================
    # 공통 액션 콜백
    # ==================================================================

    def _execute_motion_command(self, command, index):
        """가상 첫 접근의 특이점 보완만 분리하며 획 완료·취소 판단은 유지한다."""
        if (
            index == 0
            and not command.pen_down
            and self._param('virtual_initial_approach')
        ):
            self.get_logger().info('가상 첫 접근: 현재 자세로 이탈 후 movejx로 접근합니다.')
            return self._robot.prepare_virtual_approach(command.pos)
        return self._robot.execute_drawing_command(command)

    def _goal_callback(self, goal_request):
        self.get_logger().info('액션 Goal 수신')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('액션 취소 요청 수신')
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    # ==================================================================
    # DrawOnCake 액션 실행
    # ==================================================================

    def _execute_draw(self, goal_handle):
        """DrawOnCake 액션 — 웨이포인트를 하나씩 MoveLine으로 실행."""
        request = goal_handle.request
        result = DrawOnCake.Result()

        self.get_logger().info(
            f'드로잉 시작: waypoints={len(request.waypoints)}, '
            f'surface={request.drawing_surface}'
        )

        waypoints_dict, pen_states = self._convert_waypoints(
            request.waypoints, default_speed=request.speed,
        )

        plan = plan_drawing_motion(
            waypoints_dict, pen_states, request.drawing_surface,
        )

        if not plan.success:
            self.get_logger().error(f'동작 계획 실패: {plan.error_msg}')
            result.success = False
            result.error_code = plan.error_msg
            result.completed_strokes = 0
            goal_handle.abort()
            return result

        if request.drawing_surface == 'SIDE':
            try:
                plan.commands = self._robot.select_reachable_side_commands(
                    plan.commands, lambda: goal_handle.is_cancel_requested,
                )
            except (RuntimeError, ValueError) as error:
                self.get_logger().error(f'옆면 자세 선택 실패: {error}')
                result.success = False
                result.completed_strokes = 0
                if goal_handle.is_cancel_requested:
                    result.error_code = 'CANCELLED'
                    goal_handle.canceled()
                else:
                    result.error_code = 'SIDE_POSE_UNREACHABLE'
                    goal_handle.abort()
                return result

        # 명령 실행
        total = len(plan.commands)
        completed_strokes = 0
        prev_stroke_idx = 0

        for i, cmd in enumerate(plan.commands):
            # 취소 확인
            if goal_handle.is_cancel_requested:
                self.get_logger().info('드로잉 취소됨')
                result.success = False
                result.error_code = 'CANCELLED'
                result.completed_strokes = completed_strokes
                goal_handle.canceled()
                return result

            # feedback
            feedback = DrawOnCake.Feedback()
            feedback.current_stroke_idx = cmd.stroke_idx
            feedback.total_strokes = plan.total_strokes
            feedback.progress_pct = float(i) / total * 100.0
            feedback.phase = cmd.phase.value
            goal_handle.publish_feedback(feedback)

            # 획·펜 상태가 바뀔 때만 주요 진행 상황을 터미널에 남긴다.
            if i == 0 or cmd.phase != plan.commands[i - 1].phase or cmd.stroke_idx != plan.commands[i - 1].stroke_idx:
                self.get_logger().info(
                    f'동작 {i+1}/{total}, 획 {cmd.stroke_idx+1}: '
                    f'{cmd.phase.value}, {cmd.motion_type.value}'
                )
            # DP가 지정한 직선·원호·스플라인 명령을 실행한다.
            side_rotation = request.drawing_surface == 'SIDE' and i == 2 and not cmd.pen_down
            if side_rotation:
                # 표면 바깥에서만 펜 자세를 바꾸고 현재 관절 해를 사용한다.
                moved = self._robot.move_task_joint(cmd.pos)
            else:
                moved = self._execute_motion_command(cmd, i)
            if not moved:
                self.get_logger().error(
                    f'{"movejx(옆면 자세 전환)" if side_rotation else cmd.motion_type.value} '
                    f'실패 (#{i}): {cmd.pos.to_list()}'
                )
                result.success = False
                result.error_code = f'MOVE_FAILED_AT_{i}'
                result.completed_strokes = completed_strokes
                goal_handle.abort()
                return result

            if cmd.stroke_idx > prev_stroke_idx:
                completed_strokes += 1
                prev_stroke_idx = cmd.stroke_idx

        self.get_logger().info(f'드로잉 완료: {plan.total_strokes} 획')
        result.success = True
        result.completed_strokes = plan.total_strokes
        result.error_code = ''
        goal_handle.succeed()
        return result

    # ==================================================================
    # GraspTool 액션 실행
    # ==================================================================

    def _execute_grasp(self, goal_handle):
        """GraspTool 액션 — 펜 집기/놓기."""
        request = goal_handle.request
        result = GraspTool.Result()

        action = request.action.upper()
        self.get_logger().info(f'도구 {action}: {request.tool_id}')

        above = pose_msg_to_dsr(
            request.tool_pose,
            z_offset_m=request.approach_dir.z * 0.05,
        )
        target = pose_msg_to_dsr(request.tool_pose)

        # feedback 헬퍼
        def _fb(phase: str, pct: float):
            fb = GraspTool.Feedback()
            fb.phase = phase
            fb.progress_pct = pct
            goal_handle.publish_feedback(fb)

        # 1. 접근
        _fb('APPROACHING', 0.0)
        if not self._robot.move_line(above, speed_mm_s=50.0):
            result.success = False
            result.error_code = 'APPROACH_FAILED'
            goal_handle.abort()
            return result

        # 2. 집기 or 놓기
        open_w = self._param('gripper_open_width')

        if action == 'GRASP':
            _fb('GRASPING', 30.0)
            self._robot.gripper_open(open_w)
            if not self._robot.move_line(target, speed_mm_s=30.0):
                result.success, result.error_code = False, 'MOVE_TO_TOOL_FAILED'
                goal_handle.abort()
                return result
            self._robot.gripper_close(
                self._param('pen_grip_width'),
                self._param('pen_grip_force'),
            )

        elif action == 'RELEASE':
            _fb('RELEASING', 30.0)
            if not self._robot.move_line(target, speed_mm_s=30.0):
                result.success, result.error_code = False, 'MOVE_TO_TOOL_FAILED'
                goal_handle.abort()
                return result
            self._robot.gripper_open(open_w)

        else:
            result.success = False
            result.error_code = f'UNKNOWN_ACTION: {action}'
            goal_handle.abort()
            return result

        # 3. 후퇴
        _fb('RETRACTING', 80.0)
        if not self._robot.move_line(above, speed_mm_s=50.0):
            result.success, result.error_code = False, 'RETRACT_FAILED'
            goal_handle.abort()
            return result

        result.success = True
        result.error_code = ''
        goal_handle.succeed()
        return result

    # ==================================================================
    # PickAndPlace 액션 실행
    # ==================================================================

    def _execute_pick_place(self, goal_handle):
        """PickAndPlace 액션 — 케이크 운반."""
        request = goal_handle.request
        result = PickAndPlace.Result()

        h = request.approach_height
        grip_w = request.grip_width

        self.get_logger().info(
            f'PickAndPlace: grip={grip_w}mm, approach_h={h}m'
        )

        # feedback 헬퍼
        def _fb(phase: str, pct: float):
            fb = PickAndPlace.Feedback()
            fb.phase = phase
            fb.progress_pct = pct
            goal_handle.publish_feedback(fb)

        # --- Pick ---
        pick_above = pose_msg_to_dsr(request.pick_pose, z_offset_m=h)
        pick_target = pose_msg_to_dsr(request.pick_pose)

        _fb('APPROACHING', 0.0)
        if not self._robot.move_line(pick_above, speed_mm_s=80.0):
            result.success, result.error_code = False, 'PICK_APPROACH_FAILED'
            goal_handle.abort()
            return result

        self._robot.gripper_open(grip_w + 20.0)

        _fb('GRIPPING', 20.0)
        if not self._robot.move_line(pick_target, speed_mm_s=30.0):
            result.success, result.error_code = False, 'PICK_DESCEND_FAILED'
            goal_handle.abort()
            return result

        self._robot.gripper_close(grip_w)

        _fb('LIFTING', 40.0)
        if not self._robot.move_line(pick_above, speed_mm_s=30.0):
            result.success, result.error_code = False, 'PICK_LIFT_FAILED'
            goal_handle.abort()
            return result

        # --- Place ---
        place_above = pose_msg_to_dsr(request.place_pose, z_offset_m=h)
        place_target = pose_msg_to_dsr(request.place_pose)

        _fb('MOVING', 60.0)
        if not self._robot.move_line(place_above, speed_mm_s=80.0):
            result.success, result.error_code = False, 'PLACE_MOVE_FAILED'
            goal_handle.abort()
            return result

        _fb('PLACING', 80.0)
        if not self._robot.move_line(place_target, speed_mm_s=30.0):
            result.success, result.error_code = False, 'PLACE_DESCEND_FAILED'
            goal_handle.abort()
            return result

        _fb('RELEASING', 90.0)
        self._robot.gripper_open(grip_w + 20.0)

        if not self._robot.move_line(place_above, speed_mm_s=50.0):
            result.success, result.error_code = False, 'PLACE_RETRACT_FAILED'
            goal_handle.abort()
            return result

        result.success = True
        result.actual_place_pose = request.place_pose
        result.error_code = ''
        goal_handle.succeed()
        return result


# ------------------------------------------------------------------
# 엔트리포인트
# ------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = RobotExecutorNode()
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
