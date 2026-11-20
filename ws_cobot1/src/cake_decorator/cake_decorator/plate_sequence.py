"""플레이트 준비 및 후처리 동작 순서.

ROS2 노드가 아닌 일반 Python 모듈입니다.
로봇 실행 노드의 제어 객체와 파라미터 조회 함수를 재사용합니다.
"""

import logging
import time
import math

from .doosan_robot import DR_MV_MOD_REL, DR_MV_MOD_ABS, DR_BASE
from .drawing_motion import DsrPos
from .plate_weight_check import PlateWeightChecker
from .safety_exceptions import OpenerMissingError, PlateWeightError


class PlateSequence:
    """플레이트 전용 이동 시퀀스를 관리합니다."""

    def __init__(self, robot, param):
        """실행 노드의 로봇·파라미터 조회기를 받아 단계별 이동을 준비한다."""
        self._robot = robot
        self._param = param
        self._logger = getattr(robot, '_logger', logging.getLogger(__name__))
        self._weight_phase = "prepare"
        self._weight_checker = None
        self._opener_held = False

    def _option(self, name, default):
        """기존 호출자의 선택 설정 누락은 명시된 기본값으로 처리한다."""
        try:
            return self._param(name)
        except KeyError:
            return default

    def _guard_opener(self):
        """따개를 들고 있어야 하는 안전 자세에서만 파지 폭을 검사한다."""
        if not self._option('gripper_safety_check_enabled', False):
            return
        width = self._robot.get_gripper_width()
        minimum = self._param('opener_hold_min_width_m')
        if width is not None and width >= minimum:
            return
        reason = '따개 없음' if width is not None else '따개 파지 폭 측정 실패'
        # 이미 안전 자세에 있다. 열기에 실패하면 후속 이동을 보내지 않는다.
        if not self._robot.gripper_open(self._param('opener_open_width_mm')):
            raise OpenerMissingError(reason + ': 그리퍼 열기 실패, 현재 자세에서 중단')
        self._opener_held = False
        raise OpenerMissingError(reason + ', 안전 자세에서 대기중')

    def _weight_at_safe(self, stage):
        """같은 안전 자세에서 하중을 측정하고 실패 시 감속 반납 후 안전 자세로 복귀한다."""
        if not self._option('plate_weight_check_enabled', False):
            return
        try:
            time.sleep(float(self._param('plate_weight_wait_sec')))
            if stage in ('prepare_start', 'finish_start'):
                self._weight_checker = PlateWeightChecker(
                    self._robot,
                    self._param('plate_opener_weight_kg'),
                    self._param('plate_cake_weight_kg'),
                    self._param('plate_weight_tolerance_percent'),
                    self._param('plate_cake_min_delta_kg'),
                    phase='prepare' if stage == 'prepare_start' else 'finish',
                    reads=self._param('plate_weight_reads'),
                    read_interval=self._param('plate_weight_read_interval'),
                )
                self._weight_checker.start()
                return
            if self._weight_checker is None:
                raise RuntimeError('하중 기준값 미설정')
            result = self._weight_checker.check(stage)
            self._logger.info(f'하중 검사 {stage}: {result}')
            if not result.passed:
                raise RuntimeError(f'케이크 무게 증가량 {result.cake_delta_kg:.4f}kg, 기준 {result.cake_min_delta_kg:.4f}kg 초과 필요')
        except (RuntimeError, ValueError, TypeError) as error:
            if getattr(self._robot, 'stop_latched', False) or getattr(self._robot, 'gripper_output_fault', False):
                raise PlateWeightError(f'케이크 무게 확인 실패: {error}. 복구 명령 차단 중') from error
            if self._opener_held:
                speed = self._param('free_line_speed_mm_s')
                if not self._plate_place_opener(
                    speed_approach=speed * 0.5,
                    speed_slow=speed * 0.5,
                    speed_retract=speed * 0.5,
                ):
                    raise PlateWeightError(f'케이크 무게 확인 실패: {error}. 따개 반납 실패, 현재 자세에서 중단') from error
            if not self._plate_joint(
                'safe_position_1',
                speed_deg_s=self._param('free_joint_speed_deg_s') * 0.5,
            ):
                raise PlateWeightError(f'케이크 무게 확인 실패: {error}. 안전 자세 복귀 실패') from error
            raise PlateWeightError(f'케이크 무게 확인 실패: {error}. 안전 자세에서 대기중') from error


    def _plate_joint(
        self, name: str, relative: bool = False, cake_carrying: bool = False,
        *, speed_deg_s: float | None = None, acc_deg_s2: float | None = None,
    ) -> bool:
        """기본은 현재 구간 속도이며 None이 아닌 인자만 이번 호출에 적용한다.

        관절 속도는 deg/s, 가속도는 deg/s²다. 상대/절대 모드는 그대로 전달한다.
        """
        # 안착·들어올림은 저속, 나머지 케이크 취급은 기존 30도/s를 보존한다.
        if name in (
            'cake_station_ps2', 'pickup_cake_station_ps3',
            'pickup_cake_station_ps4', 'deliver_cake_station_ps2',
        ):
            speed = self._param('cake_seat_joint_speed_deg_s')
            acc = self._param('cake_seat_joint_acc_deg_s2')
        elif cake_carrying:
            speed, acc = 30.0, 30.0
        else:
            speed = self._param('free_joint_speed_deg_s')
            acc = self._param('free_joint_acc_deg_s2')
        speed = speed if speed_deg_s is None else speed_deg_s
        acc = acc if acc_deg_s2 is None else acc_deg_s2
        self._validate_motion(speed, acc)
        self._logger.info(f'plate {name}: speed={speed:g}deg/s acc={acc:g}deg/s2')
        return self._robot.move_joint(
            list(self._param('plate_' + name)),
            speed_deg_s=speed,
            acc_deg_s2=acc,
            mode=DR_MV_MOD_REL if relative else DR_MV_MOD_ABS,
        )

    def _plate_line(
        self, name: str, relative: bool = False, cake_carrying: bool = False,
        *, speed_mm_s: float | None = None, acc_mm_s2: float | None = None,
    ) -> bool:
        """기본은 현재 구간 속도이며 None이 아닌 인자만 이번 호출에 적용한다.

        직선 속도는 mm/s, 가속도는 mm/s²다. BASE 상대/절대 모드를 보존한다.
        """
        # 케이크를 받친 직후의 상대 상승만 별도 저속 파라미터를 사용한다.
        if name == 'move_cake_plate_ps4':
            speed = self._param('cake_lift_speed_mm_s')
            acc = self._param('cake_lift_acc_mm_s2')
        elif cake_carrying:
            speed, acc = 50.0, 100.0
        else:
            speed = self._param('free_line_speed_mm_s')
            acc = self._param('free_line_acc_mm_s2')
        speed = speed if speed_mm_s is None else speed_mm_s
        acc = acc if acc_mm_s2 is None else acc_mm_s2
        self._validate_motion(speed, acc)
        self._logger.info(f'plate {name}: speed={speed:g}mm/s acc={acc:g}mm/s2')
        # 보호 구간에는 기존 move_line 회전 제한을 유지한다. 일반 직선만 통일한다.
        angular = {} if cake_carrying else {
            'angular_speed_deg_s': self._param('free_joint_speed_deg_s'),
            'angular_acc_deg_s2': self._param('free_joint_acc_deg_s2'),
        }
        return self._robot.move_line(
            DsrPos(*self._param('plate_' + name)),
            speed_mm_s=speed,
            acc_mm_s2=acc,
            **angular,
            ref=DR_BASE,
            mode=DR_MV_MOD_REL if relative else DR_MV_MOD_ABS,
        )

    @staticmethod
    def _validate_motion(*values):
        """속도·가속도의 잘못된 override를 이동 전에 거절한다."""
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            and value > 0
            for value in values
        ):
            raise ValueError('속도와 가속도는 유한한 양수여야 합니다')

    def _plate_opener_target(self) -> DsrPos:
        """오프너 접근 위치에 BASE 상대 변위를 합산해 절대 목표를 반환한다."""
        # [변경] pick()은 절대 목표를 받으므로 원본 BASE 상대 Z -100mm를 합산.
        # 원본 pick_opener1 / pick_opener2 값을 모두 파라미터에서 읽는다.
        approach = self._param('plate_pick_opener1')
        delta = self._param('plate_pick_opener2')
        return DsrPos(*(a + b for a, b in zip(approach, delta)))

    def _plate_pick_opener(
        self, *, speed_approach=None, speed_slow=None,
    ) -> bool:
        """공통 pick()을 재사용해 케이크가 없는 오프너를 파지한다.

        접근·랙 진입·파지 후 후퇴는 모두 일반 직선 속도를 쓴다.
        그리퍼 명령과 닫힘·안정화 대기는 공통 pick()에 맡겨 보존한다.
        거치대를 벗어나는 클리어런스 이동은 호출부에서 추가한다.
        """
        # 따개를 집기 전 같은 안전 자세에서 빈 그리퍼의 기준 하중을 기록한다.
        if not self._plate_joint('safe_position_1'):
            return False
        self._weight_at_safe(self._weight_phase + '_start')
        # 케이크를 집기 전까지 따개 거치대 이동도 일반 속도로 통일한다.
        speed = (
            self._param('free_line_speed_mm_s')
            if speed_approach is None else speed_approach
        )
        rack_speed = (
            self._param('free_line_speed_mm_s')
            if speed_slow is None else speed_slow
        )
        self._validate_motion(speed, rack_speed)
        self._logger.info(
            f'plate pick_opener: approach={speed:g} slow={rack_speed:g} '
            f'retract={rack_speed:g}mm/s'
        )
        picked = self._robot.pick(
            target=self._plate_opener_target(),
            approach=DsrPos(*self._param('plate_pick_opener1')),
            grip_width=self._param('opener_grip_width_mm'),
            grip_force=self._param('opener_grip_force_n'),
            speed_approach=speed,
            speed_slow=rack_speed,
            open_width=self._param('opener_open_width_mm'),
            close_wait_sec=self._param('opener_close_wait_sec'),
            close_settle_sec=self._param('opener_close_settle_sec'),
            pre_open_wait_sec=self._param('opener_pre_open_wait_sec'),
            acc_mm_s2=self._param('free_line_acc_mm_s2'),
            angular_speed_deg_s=self._param('free_joint_speed_deg_s'),
            angular_acc_deg_s2=self._param('free_joint_acc_deg_s2'),
        )

        if not picked:
            return False
        self._opener_held = True
        # 들어올린 뒤 안전 자세에서 따개 파지를 확인한다.
        if self._option('gripper_safety_check_enabled', False) or self._option('plate_weight_check_enabled', False):
            if not self._plate_joint('safe_position_1'):
                return False
            self._guard_opener()
            self._weight_at_safe(self._weight_phase + '_opener')
        return True

    def _plate_place_opener(
        self, *, speed_approach=None, speed_slow=None, speed_retract=None,
    ) -> bool:
        """현재 파지 중인 opener를 원래 거치대에 내려놓는다.

        place() 내부에서 접근 위치로 이동한 뒤 목표 위치에서 그리퍼를 열어
        opener를 해제하고 다시 접근 위치로 후퇴한다. 따라서 호출부는 별도의
        그리퍼 열기 명령을 보내지 않는다.
        """
        # 케이크를 내려놓은 뒤 따개만 반납하므로 모든 구간을 일반 속도로 맞춘다.
        base_speed = self._param('free_line_speed_mm_s')
        speed = base_speed if speed_approach is None else speed_approach
        rack_speed = base_speed if speed_slow is None else speed_slow
        retract = base_speed if speed_retract is None else speed_retract
        self._validate_motion(speed, rack_speed, retract)
        self._logger.info(
            f'plate place_opener: approach={speed:g} slow={rack_speed:g} '
            f'retract={retract:g}mm/s'
        )
        placed = self._robot.place(
            target=self._plate_opener_target(),
            approach=DsrPos(*self._param('plate_pick_opener1')),
            open_width=self._param('opener_open_width_mm'),
            speed_approach=speed,
            speed_slow=rack_speed,
            speed_retract=retract,
            open_wait_sec=self._param('opener_open_wait_sec'),
            acc_mm_s2=self._param('free_line_acc_mm_s2'),
            angular_speed_deg_s=self._param('free_joint_speed_deg_s'),
            angular_acc_deg_s2=self._param('free_joint_acc_deg_s2'),
        )

        if placed:
            self._opener_held = False
        return placed

    def _plate_prepare_steps(self) -> bool:
        """오프너를 파지하고 케이크를 작업대에 안착한 뒤 오프너를 반납한다."""
        self._weight_phase = 'prepare'
        if not self._plate_pick_opener():
            return False
        # opener 거치대에서 벗어나 케이크 작업대 쪽으로 진입하기 위한 이동.
        if not self._plate_line('move_cake_ps_l2'):
            return False
        # 원본 순서 정정: ps5 삭제, safe position 2 복원, station ps4 추가.
        # 각 위치별 설명은 workspace_config.yaml의 원본 주석 참조.
        # 마지막 값은 케이크 취급 구간 표시: ps1부터 station ps3까지 보수적으로 유지.
        for name, kind, relative, cake_carrying in (
            ('move_cake_plate_ps1', 'joint', False, True),
            ('move_cake_plate_ps2', 'joint', True, True),
            ('move_cake_plate_ps3', 'joint', False, True),
            ('move_cake_plate_ps4', 'line', True, True),
            ('safe_position_2', 'joint', False, True),
            ('cake_station_ps1', 'joint', False, True),
            ('cake_station_ps2', 'joint', False, True),
            ('cake_station_ps3', 'joint', False, True),
            ('cake_station_ps4', 'joint', False, False),
            ('safe_position_3', 'joint', False, False),
        ):
            move = self._plate_joint if kind == 'joint' else self._plate_line
            if not move(name, relative, cake_carrying):
                return False
            stages = {
                'safe_position_2': 'prepare_both',
                'safe_position_3': 'prepare_opener_after_cake',
                'safe_position_4': 'finish_both',
                'safe_position_5': 'finish_opener_after_cake',
            }
            if name in stages:
                self._weight_at_safe(stages[name])
        # 드로잉 전에 그리퍼를 비워야 마커를 파지할 수 있으므로 opener를
        # 거치대에 내려놓는다. place()가 그리퍼 열기까지 처리한다.
        if not self._plate_place_opener():
            return False
        if self._option('plate_weight_check_enabled', False):
            if not self._plate_joint('safe_position_1'):
                return False
            self._weight_at_safe('prepare_end')
        return True

    def _plate_finish_steps(self) -> bool:
        """작업대 ps2부터 케이크를 회수·출고하고 오프너 반납 후 안전 자세로 간다."""
        # 스테이션에서 케이크를 들어 올린 뒤 출고하고 오프너를 반환한다.
        # 드로잉으로 그리퍼가 빈 상태이므로 케이크를 옮기기 전에 opener를
        # 다시 파지한다. 공통 pick()이 닫힘과 안정화 대기를 처리한다.
        self._weight_phase = 'finish'
        if not self._plate_pick_opener():
            return False
        # 재파지 후 opener 거치대에서 벗어나기 위한 클리어런스 이동.
        if not self._plate_line('move_cake_ps_l2'):
            return False
        # 승인된 경유점 삭제: pickup ps1을 생략하고 ps2부터 출고 ps3까지 보호한다.
        for name, kind, relative, cake_carrying in (
            ('pickup_cake_station_ps2', 'joint', False, True),
            ('pickup_cake_station_ps3', 'joint', False, True),
            ('pickup_cake_station_ps4', 'joint', False, True),
            ('safe_position_4', 'joint', False, True),
            ('deliver_cake_station_ps1', 'joint', False, True),
            ('deliver_cake_station_ps2', 'joint', False, True),
            ('deliver_cake_station_ps3', 'joint', False, True),
            ('deliver_cake_station_ps4', 'joint', False, False),
            ('deliver_cake_station_ps5', 'line', True, False),
            ('safe_position_5', 'joint', False, False),
        ):
            move = self._plate_joint if kind == 'joint' else self._plate_line
            if not move(name, relative, cake_carrying):
                return False
            stages = {
                'safe_position_2': 'prepare_both',
                'safe_position_3': 'prepare_opener_after_cake',
                'safe_position_4': 'finish_both',
                'safe_position_5': 'finish_opener_after_cake',
            }
            if name in stages:
                self._weight_at_safe(stages[name])
        # deliver_opener_ps1/ps2는 pick_opener1/2와 동일 좌표: 공통값 재사용.
        # 기존 place()는 접근 위치로 후퇴한다(원본은 해제 직후 종료).
        # [변경 2026-11-19] 전용 plate_place 대신 기존 place()을 그대로 재사용한다.
        # 그리퍼는 place() 내부의 DO2 Open(기본값) 펄스로 오프너를 해제한다.
        if not self._plate_place_opener():
            return False
        # 오프너 반납 후 safe position 5로 최종 이동.
        if not self._plate_joint('safe_position_5'):
            return False
        self._weight_at_safe('finish_end')
        return True
