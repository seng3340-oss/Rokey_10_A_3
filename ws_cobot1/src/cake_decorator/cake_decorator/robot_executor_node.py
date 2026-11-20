"""로봇 실행 노드 — 전체 작업 상태 머신 + 그리기·운반·펜 파지 액션 서버.

TM(task_manager)에서 ExecuteTask Goal을 보내면 전체 작업 흐름을
상태 머신으로 관리하고, 각 단계에서 Doosan M0609를 실제로 움직인다.

작업 상태 흐름:
  IDLE → RECEIVED → VALIDATING → PICKING_CAKE → PLACING_CAKE
  → GRASPING_TOOL → DRAWING → RETURNING_TOOL → DELIVERING
  → COMPLETED / FAILED / CANCELED
"""

from __future__ import annotations

import time
import traceback
import threading

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from cake_decorator_interfaces.action import DrawOnCake, GraspTool, PickAndPlace

from .doosan_robot import DoosanRobotClient
from .plate_sequence import PlateSequence
from .safety_exceptions import (
    OpenerMissingError,
    PEN_GRASP_FAILED_CODE,
    PEN_GRASP_FAILED_MESSAGE,
    PenGraspError,
)
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
        'plate_weight_check_enabled': False,  # 실물 검증 전 OFF
        'plate_opener_weight_kg': 0.05,  # 예시값: 실측 필요
        'plate_cake_weight_kg': 0.20,  # 예시값: 실측 필요
        'plate_weight_tolerance_percent': 10.0,
        # 케이크 보유(both) 단계의 최소 변화량. 이 값 이하면 실패한다.
        'plate_cake_min_delta_kg': 0.075,
        'plate_weight_wait_sec': 1.0,
        # Safe Position 도착 후 2회 읽어 평균한다.
        'plate_weight_reads': 2,
        'plate_weight_read_interval': 0.2,
        'plate_sequence_enabled': False,  # [변경] 기본 비활성: 실물 충돌 검증 전 작동 금지
        'opener_grip_width_mm': 10.0,
        'opener_grip_force_n': 20.0,
        'opener_open_width_mm': 100.0,
        'opener_close_wait_sec': 0.5,
        'opener_close_settle_sec': 0.5,
        'opener_open_wait_sec': 1.0,
        'opener_pre_open_wait_sec': 1.0,  # [변경] pick 전 열림 안정화
        'plate_safe_position_1': [35.0, 40.0, 35.0, -20.0, 120.0, 120.0],
        'plate_pick_opener1': [426.73, 252.72, 379.41, 30.09, 179.36, 28.13],
        'plate_pick_opener2': [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
        'plate_move_cake_ps_l': [426.73, 252.72, 379.41, 30.09, 179.36, 28.13],
        'plate_move_cake_ps_l2': [606.73, 252.72, 379.41, 30.09, 179.36, 28.13],
        'plate_move_cake_plate_ps1': [-0.12, 38.25, 45.36, 0.15, 96.65, -0.02],
        'plate_move_cake_plate_ps2': [0.0, 0.0, 0.0, 0.0, -7.0, 0.0],
        'plate_move_cake_plate_ps3': [-0.15, 34.97, 45.68, 0.15, 88.96, -0.02],
        'plate_move_cake_plate_ps4': [0.0, 0.0, 80.0, 0.0, 0.0, 0.0],
        'plate_safe_position_2': [35.0, 40.0, 35.0, -20.0, 120.0, 120.0],
        'plate_cake_station_ps1': [6.75, 25.95, 57.94, -7.64, 109.05, 140.45],
        'plate_cake_station_ps2': [6.76, 26.11, 62.27, -7.39, 105.25, 140.45],
        'plate_cake_station_ps3': [8.14, 23.79, 66.06, -5.0, 97.29, 145.51],
        'plate_cake_station_ps4': [13.26, 24.26, 62.83, -3.12, 93.19, 138.85],
        'plate_safe_position_3': [35.0, 40.0, 35.0, -20.0, 120.0, 120.0],
        'plate_pickup_cake_station_ps1': [13.26, 24.26, 62.83, -3.12, 93.19, 138.85],
        'plate_pickup_cake_station_ps2': [8.14, 23.79, 66.06, -5.0, 97.29, 145.51],
        'plate_pickup_cake_station_ps3': [10.23, 28.3, 60.43, -11.81, 107.74, 145.51],
        'plate_pickup_cake_station_ps4': [10.08, 25.88, 55.17, -12.15, 111.92, 145.51],
        'plate_safe_position_4': [35.0, 40.0, 35.0, -20.0, 120.0, 120.0],
        'plate_deliver_cake_station_ps1': [-0.17, 41.12, 21.84, 0.16, 106.65, 0.01],
        'plate_deliver_cake_station_ps2': [0.5, 33.55, 47.47, -1.71, 87.35, 1.79],
        'plate_deliver_cake_station_ps3': [0.86, 36.47, 48.75, -2.0, 86.71, -0.01],
        'plate_deliver_cake_station_ps4': [-0.09, 32.95, 54.3, 0.15, 93.31, -0.02],
        'plate_deliver_cake_station_ps5': [0.0, 0.0, 80.0, 0.0, 0.0, 0.0],
        'plate_safe_position_5': [35.0, 40.0, 35.0, -20.0, 120.0, 120.0],
        'plate_deliver_opener_ps1': [426.73, 252.72, 379.41, 30.09, 179.36, 28.13],
        'plate_deliver_opener_ps2': [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
        'service_timeout': 5.0,
        'pen_grip_width': 10.0,    # mm
        'pen_grip_force': 20.0,    # N
        'gripper_open_width': 100.0,  # mm
        # [변경 2026-11-19] PM 최신 그리퍼 제어: 제어박스 DO 펄스.
        # 기본 DO2=Open, DO1=Close, 0.5초 유지. 배선이 반대인 장비는
        # YAML에서 두 포트 값만 교환하며 코드는 수정하지 않는다.
        'gripper_open_do': 2,
        'gripper_close_do': 1,
        'gripper_pulse_sec': 0.5,
        # 가상환경에 set_ctrl_box_digital_output 서비스가 없을 때만
        # /onrobot/sendCommand를 대체 경로로 사용한다.
        'gripper_virtual_fallback': True,
        'approach_height': 0.05,   # m
        'virtual_initial_approach': False,  # 실행기가 가상 모드에서만 활성화
        # true면 웹 드로잉 전후에 _do_grasping_tool·_do_returning_tool을 호출한다.
        # scripts/start_real_pen_cycle.sh로 실행할 때만 켜진다.
        'pen_tool_cycle': False,
    }

    # 펜 파지 안전예외 기준(m).
    # A: 펜 상공에서 그리퍼가 충분히 열렸는지 확인한다.
    _PEN_OPEN_MIN_WIDTH_M = 0.060
    # B: 펜을 들어올린 뒤 실제로 보유 중인지 확인한다(초기값).
    _PEN_HOLD_MIN_WIDTH_M = 0.030

    def __init__(self):
        super().__init__('robot_executor_node')

        # 파라미터 선언
        for name, default in self._PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('pen_stand_pose', [0.0]*6)
        self.declare_parameter('pen_stand_approach_pose', [0.0]*6)
        self.declare_parameter('pen_gripper_pose', [0.0]*6)
        self.declare_parameter('pen_drop_approach_pose', [0.0]*6)
        self.declare_parameter('pen_drop_pose', [0.0]*6)
        self.declare_parameter('pen_drop_after_pose', [0.0]*6)
        # 케이크 중심 좌표: approach는 관절 각도(deg), center는 BASE 작업 좌표(mm, deg)
        self.declare_parameter('cake_center_approach_pose', [0.0]*6)
        self.declare_parameter('cake_center_pose', [0.0]*6)

        # Doosan + 그리퍼 클라이언트
        # 포트 검증에 실패하면 노드 생성 단계에서 ValueError로 중단된다.
        # 아직 어떤 모터 명령도 보내지 않은 정지 상태이므로 포트 설정을
        # 적용하기에 안전한 시점이다.
        self._robot = DoosanRobotClient(
            self,
            service_timeout=self._param('service_timeout'),
            gripper_open_do=int(self._param('gripper_open_do')),
            gripper_close_do=int(self._param('gripper_close_do')),
            gripper_pulse_sec=float(self._param('gripper_pulse_sec')),
            gripper_virtual_fallback=bool(
                self._param('gripper_virtual_fallback')
            ),
        )

        self._plate_sequence = PlateSequence(self._robot, self._param)

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

        # 기존 액션을 유지하며 준비/후처리 작업만 서비스로 추가.
        # 두 요청의 동시 실행을 방지한다. 드로잉 중에는 Task Manager가 요청하지 않는다.
        self._plate_lock = threading.Lock()
        self._plate_prepare_srv = self.create_service(
            Trigger, 'plate_prepare', self._execute_plate_prepare, callback_group=self._cb)
        self._plate_finish_srv = self.create_service(
            Trigger, 'plate_finish', self._execute_plate_finish, callback_group=self._cb)
        self.get_logger().info(
            'robot_executor_node 시작 — 상태 머신 + 액션 서버 대기 중'
        )

    def _plate_run(self, response, phase: str, callback):
        if not self._param('plate_sequence_enabled'):
            response.success = False
            response.message = 'plate_sequence_enabled=false: 플레이트 시퀀스 비활성(기존 드로잉만 실행).' 
            return response
        if not self._plate_lock.acquire(blocking=False):
            response.success = False
            response.message = '다른 플레이트 작업 실행 중'
            return response
        try:
            response.success = bool(callback())
            response.message = f'{phase} 완료' if response.success else f'{phase} 이동/그리퍼 실패, 즉시 중단'
        except OpenerMissingError as exc:
            # 따개 없음은 예상된 안전 정지다. 트레이스백/홈 복귀 없이
            # 서비스 응답만 실패와 지정 메시지로 반환한다.
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
        except Exception as exc:
            self.get_logger().error(f'{phase} 예외: {exc}\n{traceback.format_exc()}')
            response.success = False
            response.message = f'{phase} 예외: {exc}'
        finally:
            self._plate_lock.release()
        return response

    def _execute_plate_prepare(self, request, response):
        return self._plate_run(response, '드로잉 준비', self._plate_sequence._plate_prepare_steps)

    def _execute_plate_finish(self, request, response):
        return self._plate_run(response, '드로잉 후처리', self._plate_sequence._plate_finish_steps)

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
        """FAILED 상태로 전이 + 안전 홈 복귀.

        [변경 2026-11-19] 그리퍼 DO OFF가 실패하면 출력이 켜진 채 남아 있을
        수 있다. 이때 홈 복귀 같은 후속 자동 이동을 보내면 그리퍼가 물체를
        놓지 않은 상태로 움직이므로, 출력 오류 플래그를 확인해 이동을 멈춘다.
        """
        self.get_logger().error(f'작업 실패: {reason}')
        if can_transition(self._state, TaskState.FAILED):
            self._state = TaskState.FAILED

        if getattr(self._robot, 'gripper_output_fault', False):
            self.get_logger().error(
                '그리퍼 출력 OFF 실패 상태이므로 홈 복귀 자동 이동을 중단합니다. '
                '제어박스 출력과 장비 상태를 직접 확인하세요.'
            )
            return

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
            (self._go_to_cake_center,  'MOVING_TO_CAKE_CENTER_FAILED'),
            (self._do_drawing,        'DRAWING_FAILED'),
            (self._go_to_cake_center,  'MOVING_TO_CAKE_CENTER_FAILED'),
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

    # def _do_grasping_tool(self, goal_handle, task_goal) -> bool:
    #     """GRASPING_TOOL: 펜/도구 집기 (TCP 전환 포함)."""
    #     if not self._transition_to(TaskState.GRASPING_TOOL):
    #         return False

    #     self.get_logger().info(
    #         f'도구 집기: {get_substeps(TaskState.GRASPING_TOOL)}'
    #     )

    #     if not hasattr(task_goal, 'tool_pose'):
    #         self.get_logger().warn('tool_pose 없음 — 건너뜀')
    #         return True

    #     # 1. TCP 전환
    #     # TODO: DSR TCP 전환 서비스 호출

    #     # 2~4. 도구 접근 → 파지 → 후퇴
    #     above = pose_msg_to_dsr(task_goal.tool_pose, z_offset_m=0.05)
    #     target = pose_msg_to_dsr(task_goal.tool_pose)

    #     if not self._robot.pick(
    #         target=target,
    #         approach=above,
    #         grip_width=self._param('pen_grip_width'),
    #         grip_force=self._param('pen_grip_force'),
    #         open_width=self._param('gripper_open_width'),
    #         speed_approach=50.0,
    #     ):
    #         self._fail('도구 집기 실패')
    #         return False

    #     self.get_logger().info('도구 집기 완료')
    #     return True

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

    # def _do_returning_tool(self, goal_handle, task_goal) -> bool:
    #     """RETURNING_TOOL: 도구를 거치대에 반납."""
    #     if not self._transition_to(TaskState.RETURNING_TOOL):
    #         return False

    #     self.get_logger().info(
    #         f'도구 반납: {get_substeps(TaskState.RETURNING_TOOL)}'
    #     )

    #     if not hasattr(task_goal, 'tool_pose'):
    #         self.get_logger().warn('tool_pose 없음 — 건너뜀')
    #         return True

    #     above = pose_msg_to_dsr(task_goal.tool_pose, z_offset_m=0.05)
    #     target = pose_msg_to_dsr(task_goal.tool_pose)

    #     if not self._robot.place(
    #         target=target,
    #         approach=above,
    #         open_width=self._param('gripper_open_width'),
    #         speed_approach=50.0,
    #     ):
    #         self._fail('도구 반납 실패')
    #         return False

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

        # 펜 순환 모드: 윗면 그림만 받고, 그리기 전에 거치대에서 펜을 집는다.
        pen_cycle = self._param('pen_tool_cycle')
        if pen_cycle:
            if request.drawing_surface != 'TOP':
                self.get_logger().error('펜 순환 모드는 윗면(TOP)만 그립니다.')
                result.success = False
                result.error_code = 'TOP_ONLY'
                result.completed_strokes = 0
                goal_handle.abort()
                return result
            # _do_grasping_tool은 상태 머신의 GRASPING_TOOL 단계 함수라서
            # 바로 앞 단계(PLACING_CAKE)에서만 전이할 수 있다. 케이크 운반은 하지 않으므로
            # 상태만 그 단계로 맞춘 뒤 호출한다.
            self._state = TaskState.PLACING_CAKE
            try:
                pen_ok = self._do_grasping_tool(goal_handle, request)
            except PenGraspError as exc:
                # 펜 안전예외: _do_grasping_tool이 그리퍼 열기와
                # Safe Position 이동을 마친 뒤 발생한다. 추가 이동 없이
                # 액션만 실패로 종료한다.
                self.get_logger().error(str(exc))
                self._state = TaskState.IDLE
                result.success = False
                result.error_code = PEN_GRASP_FAILED_CODE
                result.completed_strokes = 0
                goal_handle.abort()
                return result
            if not pen_ok:
                self._state = TaskState.IDLE
                result.success = False
                result.error_code = 'PEN_PICK_FAILED'
                result.completed_strokes = 0
                goal_handle.abort()
                return result
            # 펜을 집은 뒤 케이크 중심 상공 → 첫 획 시작점 상공으로 이동한다.
            # 하강과 펜 내림은 아래 드로잉 명령(접근점 → PEN_DOWN)이 수행한다.
            if not (
                self._go_to_cake_center(goal_handle, request)
                and self._move_above_first_stroke(plan)
            ):
                self._state = TaskState.IDLE
                result.success = False
                result.error_code = 'MOVING_TO_CAKE_CENTER_FAILED'
                result.completed_strokes = 0
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

        # 펜 순환 모드: 그리기가 끝나면 펜을 거치대에 반납한다.
        # 드로잉 중 실패·취소했다면 위에서 이미 반환했으므로 반납하지 않는다(펜은 그리퍼에 남음).
        if pen_cycle:
            # GRASPING_TOOL → DRAWING → RETURNING_TOOL 순서로만 전이할 수 있다.
            self._transition_to(TaskState.DRAWING)
            returned = self._do_returning_tool(goal_handle, request)
            # 다음 웹 주문에서 다시 PLACING_CAKE부터 시작할 수 있게 대기 상태로 돌린다.
            self._state = TaskState.IDLE
            if not returned:
                result.success = False
                result.error_code = 'PEN_RETURN_FAILED'
                result.completed_strokes = plan.total_strokes
                goal_handle.abort()
                return result

        result.success = True
        result.completed_strokes = plan.total_strokes
        result.error_code = ''
        goal_handle.succeed()
        return result

    # ==================================================================
    # GraspTool 액션 실행
    # ==================================================================

    def _execute_grasp(self, goal_handle):
        """GraspTool 액션 콜백 — __init__에서 등록하므로 없으면 노드가 시작되지 않는다.

        GRASP는 _do_grasping_tool, RELEASE는 _do_returning_tool을 그대로 호출한다.
        """
        request = goal_handle.request
        result = GraspTool.Result()
        action = request.action.upper()
        if action == 'GRASP':
            self._state = TaskState.PLACING_CAKE
            ok = self._do_grasping_tool(goal_handle, request)
        elif action == 'RELEASE':
            self._state = TaskState.DRAWING
            ok = self._do_returning_tool(goal_handle, request)
        else:
            ok = False
        self._state = TaskState.IDLE
        result.success = ok
        result.error_code = '' if ok else f'{action}_FAILED'
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _pen_safety_stop(self, reason: str) -> None:
        """펜 안전예외: 그리퍼를 열고 plate safe_position_1로 이동한다.

        추가 복구/자동 재개는 하지 않는다. 호출부는 PenGraspError를 발생시켜
        액션을 실패로 종료한다.
        """
        self.get_logger().error(reason)
        if not self._robot.gripper_open(self._param('gripper_open_width')):
            self.get_logger().error('펜 안전예외: 그리퍼 열기 실패')
        if not self._robot.move_joint(
            self._param('plate_safe_position_1')
        ):
            self.get_logger().error('펜 안전예외: Safe Position 이동 실패')

    def _check_pen_open(self) -> None:
        """펜 상공에서 그리퍼가 충분히 열렸는지 확인한다."""
        width_m = self._robot.get_gripper_width()
        self.get_logger().info(f'펜 상공 그리퍼 폭={width_m}')
        if width_m is None:
            self._pen_safety_stop('펜 파지 실패: 그리퍼 폭 측정 실패')
            raise PenGraspError(PEN_GRASP_FAILED_MESSAGE)
        if width_m < self._PEN_OPEN_MIN_WIDTH_M:
            self._pen_safety_stop(
                f'펜 파지 실패: 그리퍼 열림 감지(폭={width_m:.4f}m)'
            )
            raise PenGraspError(PEN_GRASP_FAILED_MESSAGE)

    def _check_pen_held(self) -> None:
        """펜을 들어올린 뒤 실제로 보유 중인지 확인한다."""
        width_m = self._robot.get_gripper_width()
        self.get_logger().info(f'펜 파지 후 그리퍼 폭={width_m}')
        if width_m is None:
            self._pen_safety_stop('펜 파지 실패: 그리퍼 폭 측정 실패')
            raise PenGraspError(PEN_GRASP_FAILED_MESSAGE)
        if width_m < self._PEN_HOLD_MIN_WIDTH_M:
            self._pen_safety_stop(
                f'펜 파지 실패: 파지 보유 확인 실패(폭={width_m:.4f}m)'
            )
            raise PenGraspError(PEN_GRASP_FAILED_MESSAGE)

    def _do_grasping_tool(self, goal_handle, task_goal) -> bool:
        """펜 집기: 상공 → 펜 위치 → 그리퍼 닫기 → 수직 상공"""
        if not self._transition_to(TaskState.GRASPING_TOOL):
            return False

        approach = self._param('pen_stand_approach_pose')
        stand = self._param('pen_stand_pose')
        gripper_up = self._param('pen_gripper_pose')

        # 1. 펜 상공으로 이동 (관절)
        self.get_logger().info('펜 상공 이동')
        if not self._robot.move_joint(approach):
            self._fail('펜 상공 이동 실패')
            return False

        # 펜 상공 도착 직후 그리퍼가 충분히 열렸는지 확인한다.
        # 60mm 미만이면 그리퍼를 열고 Safe Position에서 대기한다.
        self._check_pen_open()

        # 2. 그리퍼 열기
        # time.sleep(0.5)
        # # [변경 2026-11-19] DO ON/OFF 결과를 확인한다. 실패하면 아래로
        # # 하강하지 않고 즉시 중단해 그리퍼 상태가 불명확한 채 움직이지 않는다.
        # if not self._robot.gripper_open(self._param('gripper_open_width')):
        #     self._fail('펜 그리퍼 열기 실패')
        #     return False
        # time.sleep(0.5)  # 대기 시간 추가

        # 3. 펜 위치로 하강 (직선)
        self.get_logger().info('펜 위치로 하강')
        if not self._robot.move_line(DsrPos(*stand), speed_mm_s=30.0):
            self._fail('펜 하강 실패')
            return False

        # 4. 펜 잡기
        if not self._robot.gripper_close(
            self._param('pen_grip_width'),
            self._param('pen_grip_force'),
        ):
            self._fail('펜 그리퍼 닫기 실패')
            return False

        # 5. 수직 상공으로 올리기 (직선)
        self.get_logger().info('펜 들어올리기')
        if not self._robot.move_line(DsrPos(*gripper_up), speed_mm_s=30.0):
            self._fail('펜 들어올리기 실패')
            return False

        # 들어올린 뒤에도 펜을 보유하고 있는지 30mm 기준으로 확인한다.
        self._check_pen_held()

        self.get_logger().info('펜 집기 완료')
        return True

    def _go_to_cake_center(self, goal_handle, task_goal) -> bool:
        """케이크 중심 상공(cake_center_approach_pose, 관절)으로만 이동한다.

        호출 시점: 펜을 집은 뒤 드로잉 직전, 그리고 전체 상태 머신의 드로잉 전후.
        케이크로 하강하지 않는다. 실제 하강은 드로잉 경로가 맡는다.
          1) _move_above_first_stroke가 이 상공 높이 그대로 첫 획 시작점 위로 수평 이동한다.
          2) 드로잉 계획의 첫 명령들이 수직으로 내려가 상면(cake_center_pose의 Z)에 펜을 댄다.
        TaskState에 MOVING_TO_CAKE_CENTER 상태가 없어 전이 검사는 하지 않는다.
        """
        approach = self._param('cake_center_approach_pose')

        # approach는 관절 각도라서 move_line이 아닌 move_joint로 이동한다.
        self.get_logger().info('케이크 중심 상공 이동')
        if not self._robot.move_joint(approach):
            self._fail('케이크 중심 상공 이동 실패')
            return False

        self.get_logger().info('케이크 중심 상공 도착')
        return True

    def _move_above_first_stroke(self, plan) -> bool:
        """케이크 중심 상공 높이를 유지한 채 첫 획 시작점 바로 위로 직선 이동한다.

        호출 시점: _go_to_cake_center 직후, 드로잉 명령 실행 전.
        입력: plan.commands[0]은 DP가 만든 첫 접근점(시작점 + approach_height, 펜 올림)이다.
        동작: 현재 TCP의 Z(상공 높이)와 첫 접근점의 XY·자세로 move_line한다.
          이후 첫 명령이 그 자리에서 수직으로 내려가고, 다음 PEN_DOWN 명령이 상면에 닿는다.
          그래서 펜은 '중심 상공 → 시작점 상공 → 수직 하강' 순서로 움직이고,
          케이크 위를 비스듬히 가로지르며 내려가지 않는다.
        반환: 이동 성공 여부. 현재 TCP를 읽지 못하면 이동하지 않고 실패한다.
        """
        first = plan.commands[0].pos
        current = self._robot.current_pose()
        if current is None:
            self._fail('현재 TCP를 읽지 못해 첫 획 상공으로 이동하지 않습니다.')
            return False
        # 상공이 첫 접근점보다 낮으면 수평 이동 중 케이크와 부딪힐 수 있으므로 멈춘다.
        if current[2] <= first.z:
            self._fail(
                f'케이크 중심 상공(Z={current[2]:.1f})이 첫 접근점'
                f'(Z={first.z:.1f})보다 낮습니다. cake_center_approach_pose를 확인하세요.'
            )
            return False
        above = DsrPos(first.x, first.y, current[2], first.rz1, first.ry, first.rz2)
        self.get_logger().info(
            f'첫 획 시작점 상공 이동: {above.to_list()} '
            f'(이후 Z {current[2]:.1f} → {first.z:.1f} 수직 하강)'
        )
        if not self._robot.move_line(above, speed_mm_s=50.0):
            self._fail('첫 획 시작점 상공 이동 실패')
            return False
        return True

    def _do_returning_tool(self, goal_handle, task_goal) -> bool:
        """펜 반납: 상공 → 펜 놓을 위치 → 그리퍼 열기 → 상공"""
        if not self._transition_to(TaskState.RETURNING_TOOL):
            return False

        drop_approach = self._param('pen_drop_approach_pose')
        drop = self._param('pen_drop_pose')
        drop_after = self._param('pen_drop_after_pose')

        # 1. 놓을 위치 상공으로 이동 (관절)
        self.get_logger().info('펜 반납 상공 이동')
        if not self._robot.move_joint(drop_approach):
            self._fail('펜 반납 상공 이동 실패')
            return False

        # 2. 펜 놓을 위치로 하강 (직선)
        self.get_logger().info('펜 놓을 위치로 하강')
        if not self._robot.move_line(DsrPos(*drop), speed_mm_s=30.0):
            self._fail('펜 하강 실패')
            return False

        # 3. 그리퍼 열기 (펜 놓기)
        time.sleep(0.5)
        # [변경 2026-11-19] 열기 실패 시 후퇴 이동을 보내지 않는다.
        if not self._robot.gripper_open(self._param('gripper_open_width')):
            self._fail('펜 그리퍼 열기 실패')
            return False
        time.sleep(0.5)

        # 4. 상공으로 후퇴 (직선)
        self.get_logger().info('펜 반납 후 상공 이동')
        if not self._robot.move_line(DsrPos(*drop_after), speed_mm_s=50.0):
            self._fail('펜 반납 후퇴 실패')
            return False

        self.get_logger().info('펜 반납 완료')
        return True

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

        # [변경 2026-11-19] 그리퍼 명령 실패 시 하강·상승 등 후속 이동을 중단한다.
        if not self._robot.gripper_open(grip_w + 20.0):
            result.success, result.error_code = False, 'PICK_GRIPPER_OPEN_FAILED'
            goal_handle.abort()
            return result

        _fb('GRIPPING', 20.0)
        if not self._robot.move_line(pick_target, speed_mm_s=30.0):
            result.success, result.error_code = False, 'PICK_DESCEND_FAILED'
            goal_handle.abort()
            return result

        if not self._robot.gripper_close(grip_w):
            result.success, result.error_code = False, 'PICK_GRIPPER_CLOSE_FAILED'
            goal_handle.abort()
            return result

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
        # [변경 2026-11-19] 열기 실패 시 후퇴 이동을 보내지 않는다.
        if not self._robot.gripper_open(grip_w + 20.0):
            result.success, result.error_code = False, 'PLACE_GRIPPER_OPEN_FAILED'
            goal_handle.abort()
            return result

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
