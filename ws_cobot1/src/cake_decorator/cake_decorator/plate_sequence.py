"""플레이트 준비 및 후처리 동작 순서.

ROS2 노드가 아닌 일반 Python 모듈입니다.
로봇 실행 노드의 제어 객체와 파라미터 조회 함수를 재사용합니다.
"""

from .doosan_robot import DR_MV_MOD_REL, DR_MV_MOD_ABS, DR_BASE
from .drawing_motion import DsrPos
from .plate_weight_check import PlateWeightChecker
from .safety_exceptions import (
    OPENER_MISSING_MESSAGE,
    OpenerMissingError,
    PlateWeightError,
)
import time


# 플레이트 구간 속도 설정.
# 각 이동 함수는 speed_* 인수를 받는다. None이면 아래 기본값(30)을 쓰고,
# 값을 주면 그 호출만 우선 적용한다. 루프 구간처럼 호출부를 고치기 어려운
# 곳은 _SPEED_OVERRIDES에 구간 이름으로 추가한다.
#
# 단위:
#   관절 이동 : speed_deg_s [deg/s], acc_deg_s2 [deg/s²]
#   직선 이동 : speed_mm_s [mm/s],  acc_mm_s2 [mm/s²]
#   pick/place: speed_approach/slow/retract [mm/s] (가속도 인자 없음)
#
# 실제 값은 실물 시험 후 확정한다. 아래는 키 이름 참고용이며 값은 비워 둔다.
#   'safe_position_2': {'speed_deg_s': ..., 'acc_deg_s2': ...}
#   'move_cake_ps_l2': {'speed_mm_s': ..., 'acc_mm_s2': ...}
#   'opener_pick': {'speed_approach': ..., 'speed_slow': ...}
#   'opener_place': {'speed_approach': ..., 'speed_slow': ...,
#                    'speed_retract': ...}
_PLATE_JOINT_SPEED_DEG_S = 30.0
_PLATE_JOINT_ACC_DEG_S2 = 30.0
_PLATE_LINE_SPEED_MM_S = 30.0
_PLATE_LINE_ACC_MM_S2 = 100.0
_PLATE_PICK_SPEED_MM_S = 30.0
_PLATE_PLACE_RETRACT_SPEED_MM_S = 30.0

# 시작 전 그리퍼가 이미 열려 있는지 판단하는 임시 폭 기준(m).
# RG2의 실제 개방/폐쇄 폭을 실물에서 측정한 뒤 확정한다.
_PLATE_GRIPPER_OPEN_MIN_WIDTH_M = 0.015

# 파지 후 그리퍼가 완전히 닫혔는지 판단하는 임시 최소 폭(m).
# 따개 두께를 실측한 뒤 확정한다.
_PLATE_GRIPPER_HOLD_MIN_WIDTH_M = 0.015

_SPEED_OVERRIDES = {}

_JOINT_SPEED_KEYS = {'speed_deg_s', 'acc_deg_s2'}
_LINE_SPEED_KEYS = {'speed_mm_s', 'acc_mm_s2'}
_PICK_SPEED_KEYS = {'speed_approach', 'speed_slow'}
_PLACE_SPEED_KEYS = {
    'speed_approach', 'speed_slow', 'speed_retract',
}


def _speed_kwargs(name: str, allowed: set[str], defaults: dict) -> dict:
    """구간 이름별 override에서 허용된 속도 인자만 골라 기본값에 덮어쓴다.

    허용 키를 제한해 mode/ref 같은 인자와 충돌하지 않게 한다. override가
    없으면 기본값만 반환하므로 기존 동작과 동일하다.
    """
    kwargs = dict(defaults)
    for key, value in _SPEED_OVERRIDES.get(name, {}).items():
        if key in allowed:
            kwargs[key] = value
    return kwargs


class PlateSequence:
    """플레이트 전용 이동 시퀀스를 관리합니다."""

    def __init__(self, robot, param):
        self._robot = robot
        self._param = param
        self._weight_checker = None
        # pick 성공 후 place 전까지 opener를 들고 있음을 표시한다.
        # 안전예외 복구 시 이 플래그가 True일 때만 opener를 반납한다.
        self._opener_held = False

    def _weight_at_safe(self, stage: str) -> bool:
        """현재 Safe 자세에서 대기 후 하중 확인. 이상이면 False로 다음 작업 차단."""
        if not self._param('plate_weight_check_enabled'):
            return True
        try:
            time.sleep(float(self._param('plate_weight_wait_sec')))
            if stage in ('prepare_start', 'finish_start'):
                self._weight_checker = PlateWeightChecker(
                    self._robot,
                    float(self._param('plate_opener_weight_kg')),
                    float(self._param('plate_cake_weight_kg')),
                    float(self._param('plate_weight_tolerance_percent')),
                    float(self._param('plate_cake_min_delta_kg')),
                    phase='prepare' if stage == 'prepare_start' else 'finish',
                    reads=int(self._param('plate_weight_reads')),
                    read_interval=float(
                        self._param('plate_weight_read_interval')
                    ),
                )
                self._weight_checker.start()
                return True
            if self._weight_checker is None:
                raise RuntimeError('하중 기준값 미설정')
            result = self._weight_checker.check(stage)
            # 성공/실패와 무관하게 측정값을 남긴다. 실패 시에만 로그를 남기면
            # 정상 통과한 케이크 변화량을 확인할 수 없다.
            if result.cake_delta_kg is not None:
                self._robot._logger.info(
                    f'{stage}: 측정={result.current_kg:.4f}, '
                    f'케이크 변화량={result.cake_delta_kg:.4f}, '
                    f'최소={result.cake_min_delta_kg:.4f}'
                )
            else:
                self._robot._logger.info(
                    f'{stage}: 측정={result.current_kg:.4f}, '
                    f'기대={result.expected_kg:.4f}, '
                    f'허용=±{result.tolerance_kg:.4f}'
                )
            if not result.passed:
                if result.cake_delta_kg is not None:
                    raise PlateWeightError(
                        f'{stage}: 케이크 변화량='
                        f'{result.cake_delta_kg:.4f}, '
                        f'최소={result.cake_min_delta_kg:.4f}'
                    )
                raise PlateWeightError(
                    f'{stage}: 측정={result.current_kg:.4f}, '
                    f'예상={result.expected_kg:.4f}, '
                    f'허용=±{result.tolerance_kg:.4f}'
                )
            return True
        except (RuntimeError, ValueError, TypeError) as exc:
            self._robot._logger.error(f'Plate 하중 검사 실패: {exc}')
            # 안전예외 시 opener를 원위치로 되돌린다. opener를 들고 있지
            # 않으면 _recover_opener가 아무 동작도 하지 않는다.
            try:
                if not self._recover_opener(stage):
                    self._robot._logger.error(
                        f'Plate 하중 복구 실패({stage})'
                    )
            except Exception as recovery_exc:
                self._robot._logger.error(
                    f'Plate 하중 복구 예외({stage}): {recovery_exc}'
                )
            return False

    def _plate_safe_check(self, position: str, stage: str) -> bool:
        """Safe 위치 이동 후 그리퍼와 하중을 함께 검사한다.

        그리퍼 안전예외는 파지 위치가 아니라 Safe Position 도착 후에만
        수행한다. 그리퍼 폭이 닫힘 수준이면 다음 동작으로 넘어가지 않는다.
        """
        if not self._plate_joint(position):
            return False
        if not self._check_gripper_grasp(stage):
            return False
        return self._weight_at_safe(stage)

    def _plate_joint(
        self,
        name: str,
        relative: bool = False,
        speed_deg_s: float | None = None,
        acc_deg_s2: float | None = None,
    ) -> bool:
        """관절 이동. 인수를 주면 그 호출만 속도를 덮어쓴다.

        speed_deg_s/acc_deg_s2가 None이면 이름별 override 또는 기본 30을 쓴다.
        """
        kwargs = _speed_kwargs(
            name,
            _JOINT_SPEED_KEYS,
            {
                'speed_deg_s': _PLATE_JOINT_SPEED_DEG_S,
                'acc_deg_s2': _PLATE_JOINT_ACC_DEG_S2,
            },
        )
        if speed_deg_s is not None:
            kwargs['speed_deg_s'] = speed_deg_s
        if acc_deg_s2 is not None:
            kwargs['acc_deg_s2'] = acc_deg_s2
        return self._robot.move_joint(
            list(self._param('plate_' + name)),
            mode=DR_MV_MOD_REL if relative else DR_MV_MOD_ABS,
            **kwargs,
        )

    def _plate_line(
        self,
        name: str,
        relative: bool = False,
        speed_mm_s: float | None = None,
        acc_mm_s2: float | None = None,
    ) -> bool:
        """직선 이동. 인수를 주면 그 호출만 속도를 덮어쓴다."""
        kwargs = _speed_kwargs(
            name,
            _LINE_SPEED_KEYS,
            {
                'speed_mm_s': _PLATE_LINE_SPEED_MM_S,
                'acc_mm_s2': _PLATE_LINE_ACC_MM_S2,
            },
        )
        if speed_mm_s is not None:
            kwargs['speed_mm_s'] = speed_mm_s
        if acc_mm_s2 is not None:
            kwargs['acc_mm_s2'] = acc_mm_s2
        return self._robot.move_line(
            DsrPos(*self._param('plate_' + name)),
            ref=DR_BASE,
            mode=DR_MV_MOD_REL if relative else DR_MV_MOD_ABS,
            **kwargs,
        )

    def _check_gripper_angle(
        self,
        angle_deg: float | None,
        stage: str,
        min_deg: float | None = None,
        max_deg: float | None = None,
    ) -> None:
        """그리퍼 각도 안전 예외.

        가벼운 물품의 안전예외를 확인하기 위해 사용한다. 각도가 정상
        파지/개방 범위를 벗어나면 RuntimeError로 다음 이동을 막는다.
        실물 검증 전에는 호출하지 않으며, 정상 범위는 실측 후
        min_deg/max_deg로 확정한다.
        """
        if angle_deg is None:
            raise RuntimeError(f"{stage}: 그리퍼 각도를 읽지 못했습니다")
        if min_deg is not None and angle_deg < min_deg:
            raise RuntimeError(
                f"{stage}: 그리퍼 각도 {angle_deg:.1f}° < 하한 {min_deg:.1f}°"
            )
        if max_deg is not None and angle_deg > max_deg:
            raise RuntimeError(
                f"{stage}: 그리퍼 각도 {angle_deg:.1f}° > 상한 {max_deg:.1f}°"
            )

    def _ensure_gripper_open(self, stage: str) -> bool:
        """플레이트 시작 전에 그리퍼를 열어 둔다.

        0bc4a70에서 pick()의 자동 열기가 제거되어, 파지 전에 명시적으로
        열어 충돌을 막는다. 그리퍼 폭을 읽어 이미 열려 있으면 불필요한
        DO 펄스를 보내지 않는다. 폭을 읽지 못하면 안전하게 열기 명령을
        보낸다.
        """
        width_m = self._robot.get_gripper_width()
        angle_deg = self._robot.get_gripper_angle_deg()
        self._robot._logger.info(
            f"{stage}: 시작 전 그리퍼 폭={width_m}, 각도={angle_deg}"
        )
        # 폭 기준은 실물 측정 전 임시값이다. 이미 충분히 열려 있으면
        # 불필요한 열기 펄스를 보내지 않는다.
        if (
            width_m is not None
            and width_m >= _PLATE_GRIPPER_OPEN_MIN_WIDTH_M
        ):
            return True
        if not self._robot.gripper_open(
            self._param('opener_open_width_mm')
        ):
            self._robot._logger.error(f"{stage}: 그리퍼 열기 실패")
            return False
        time.sleep(float(self._param('opener_pre_open_wait_sec')))
        return True

    def _check_gripper_grasp(self, stage: str) -> bool:
        """Safe Position에서 그리퍼 폭으로 물품 존재를 확인한다.

        가벼운 물품의 안전예외를 확인하기 위해 사용한다. 폭이 완전히 닫힌
        수준(15mm)이면 파지 실패로 본다. 정상 파지 폭은 실측 후
        _PLATE_GRIPPER_HOLD_MIN_WIDTH_M를 확정한다.
        """
        width_m = self._robot.get_gripper_width()
        angle_deg = self._robot.get_gripper_angle_deg()
        self._robot._logger.info(
            f"{stage}: 그리퍼 폭={width_m}, 각도={angle_deg}"
        )
        if width_m is None:
            self._robot._logger.error(f"{stage}: 그리퍼 폭 측정 실패")
            return False
        if width_m < _PLATE_GRIPPER_HOLD_MIN_WIDTH_M:
            # 따개가 없으면 그리퍼를 열고 현재 Safe Position에서 대기한다.
            # 추가 이동/복구는 하지 않고 서비스는 실패로 반환한다.
            if not self._robot.gripper_open(
                self._param('opener_open_width_mm')
            ):
                self._robot._logger.error(
                    f"{stage}: 따개 없음, 그리퍼 열기 실패"
                )
            self._opener_held = False
            self._robot._logger.error(
                f"{stage}: {OPENER_MISSING_MESSAGE} (폭={width_m:.4f}m)"
            )
            raise OpenerMissingError(OPENER_MISSING_MESSAGE)
        return True

    def _recover_opener(self, stage: str) -> bool:
        """안전예외 발생 시 opener를 원위치로 되돌린다.

        기존 _plate_place_opener()를 재사용하고, 복구 속도는 현재
        opener_place 속도의 0.5배로 낮춘다. opener를 들고 있지 않으면
        아무 동작도 하지 않는다.
        """
        if not self._opener_held:
            return True
        kwargs = _speed_kwargs(
            'opener_place',
            _PLACE_SPEED_KEYS,
            {
                'speed_approach': _PLATE_PICK_SPEED_MM_S,
                'speed_slow': _PLATE_PICK_SPEED_MM_S,
                'speed_retract': _PLATE_PLACE_RETRACT_SPEED_MM_S,
            },
        )
        half = {key: value * 0.5 for key, value in kwargs.items()}
        self._robot._logger.warn(
            f"{stage}: 안전예외 - opener 복구 (0.5x 속도: {half})"
        )
        if not self._plate_place_opener(**half):
            self._robot._logger.error(f"{stage}: opener 복구 실패")
            return False
        self._opener_held = False
        return True

    def _plate_opener_target(self) -> DsrPos:
        # [변경] pick()은 절대 목표를 받으므로 원본 BASE 상대 Z -100mm를 합산.
        # 원본 pick_opener1 / pick_opener2 값을 모두 파라미터에서 읽는다.
        approach = self._param('plate_pick_opener1')
        delta = self._param('plate_pick_opener2')
        return DsrPos(*(a + b for a, b in zip(approach, delta)))

    def _plate_pick_opener(
        self,
        speed_approach: float | None = None,
        speed_slow: float | None = None,
    ) -> bool:
        """opener 거치대로 이동해 opener를 파지한다.

        인수를 주면 그 호출만 pick 속도를 덮어쓴다. None이면 이름별
        override 또는 기본 30 mm/s를 쓴다.

        Safe Position 이동은 상위 시퀀스에서 수행한다. 기존 pick() 내부에서
        접근 위치의 그리퍼 열기, 목표 위치의 그리퍼 닫기, 접근 위치 복귀까지
        수행하므로 이 함수는 파지 성공 여부만 반환한다. 거치대를 벗어나는
        클리어런스 이동은 호출부에서 상황에 맞게 추가한다.
        """
        # [검증 전] 그리퍼 각도 안전 예외. 실물에서 정상 범위를 측정한 뒤
        # 아래 주석을 해제하고 min_deg/max_deg를 확정한다.
        # self._check_gripper_angle(
        #     self._robot.get_gripper_angle_deg(),
        #     stage='opener_pick_before',
        # )
        # [변경 2026-11-19] 전용 plate_pick 대신 기존 pick()을 그대로 재사용한다.
        # 플레이트 이동에 필요한 대기 시간만 pick()의 추가 인수로 전달한다.
        kwargs = _speed_kwargs(
            'opener_pick',
            _PICK_SPEED_KEYS,
            {
                'speed_approach': _PLATE_PICK_SPEED_MM_S,
                'speed_slow': _PLATE_PICK_SPEED_MM_S,
            },
        )
        if speed_approach is not None:
            kwargs['speed_approach'] = speed_approach
        if speed_slow is not None:
            kwargs['speed_slow'] = speed_slow
        result = self._robot.pick(
            target=self._plate_opener_target(),
            approach=DsrPos(*self._param('plate_pick_opener1')),
            grip_width=self._param('opener_grip_width_mm'),
            grip_force=self._param('opener_grip_force_n'),
            open_width=self._param('opener_open_width_mm'),
            close_wait_sec=self._param('opener_close_wait_sec'),
            close_settle_sec=self._param('opener_close_settle_sec'),
            pre_open_wait_sec=self._param('opener_pre_open_wait_sec'),
            **kwargs,
        )
        if not result:
            return False
        self._opener_held = True
        return True

    def _plate_place_opener(
        self,
        speed_approach: float | None = None,
        speed_slow: float | None = None,
        speed_retract: float | None = None,
    ) -> bool:
        """현재 파지 중인 opener를 원래 거치대에 내려놓는다.

        인수를 주면 그 호출만 place 속도를 덮어쓴다. None이면 이름별
        override 또는 기본 30 mm/s를 쓴다. place() 내부에서 접근 위치로
        이동한 뒤 목표 위치에서 그리퍼를 열어 opener를 해제하고 다시 접근
        위치로 후퇴한다. 따라서 호출부는 별도의 그리퍼 열기 명령을 보내지
        않는다.
        """
        # [검증 전] 그리퍼 각도 안전 예외. 실물에서 정상 범위를 측정한 뒤
        # 아래 주석을 해제하고 min_deg/max_deg를 확정한다.
        # self._check_gripper_angle(
        #     self._robot.get_gripper_angle_deg(),
        #     stage='opener_place_before',
        # )
        kwargs = _speed_kwargs(
            'opener_place',
            _PLACE_SPEED_KEYS,
            {
                'speed_approach': _PLATE_PICK_SPEED_MM_S,
                'speed_slow': _PLATE_PICK_SPEED_MM_S,
                'speed_retract': _PLATE_PLACE_RETRACT_SPEED_MM_S,
            },
        )
        if speed_approach is not None:
            kwargs['speed_approach'] = speed_approach
        if speed_slow is not None:
            kwargs['speed_slow'] = speed_slow
        if speed_retract is not None:
            kwargs['speed_retract'] = speed_retract
        result = self._robot.place(
            target=self._plate_opener_target(),
            approach=DsrPos(*self._param('plate_pick_opener1')),
            open_width=self._param('opener_open_width_mm'),
            open_wait_sec=self._param('opener_open_wait_sec'),
            **kwargs,
        )
        if not result:
            return False
        self._opener_held = False
        return True

    def _plate_prepare_steps(self) -> bool:
        if not self._ensure_gripper_open('prepare_start'):
            return False
        if not self._plate_safe_check('safe_position_1', 'prepare_start'):
            return False
        if not self._plate_pick_opener():
            return False
        # 파지 후 기존 작업 자세에서 Safe1으로 이동해 따개 하중을 확인합니다.
        if not self._plate_safe_check('safe_position_1', 'prepare_opener'):
            return False
        # opener 거치대에서 벗어나 케이크 작업대 쪽으로 진입하기 위한 이동.
        if not self._plate_line('move_cake_ps_l2'):
            return False
        # 원본 순서 정정: ps5 삭제, safe position 2 복원, station ps4 추가.
        # 각 위치별 설명은 workspace_config.yaml의 원본 주석 참조.
        for name, kind, relative in (
            ('move_cake_plate_ps1', 'joint', False),
            ('move_cake_plate_ps2', 'joint', True),
            ('move_cake_plate_ps3', 'joint', False),
            ('move_cake_plate_ps4', 'line', True),
            ('safe_position_2', 'joint', False),
            ('cake_station_ps1', 'joint', False),
            ('cake_station_ps2', 'joint', False),
            ('cake_station_ps3', 'joint', False),
            ('cake_station_ps4', 'joint', False),
            ('safe_position_3', 'joint', False),
        ):
            if not (self._plate_joint(name, relative) if kind == 'joint'
                    else self._plate_line(name, relative)):
                return False
            if name == 'safe_position_2' and not self._weight_at_safe('prepare_both'):
                return False
            if name == 'safe_position_3' and not self._weight_at_safe('prepare_opener_after_cake'):
                return False
        # 드로잉 전에 그리퍼를 비워야 마커를 파지할 수 있으므로 opener를
        # 거치대에 내려놓는다. place()가 그리퍼 열기까지 처리한다.
        if not self._plate_place_opener():
            return False
        # Plate Prepare 종료 경계: 따개 반환 후 빈 그리퍼 상태를 검증합니다.
        return self._plate_safe_check('safe_position_1', 'prepare_end')

    def _plate_finish_steps(self) -> bool:
        # 스테이션에서 케이크를 들어 올린 뒤 출고하고 오프너를 반환한다.
        # 드로잉으로 그리퍼가 빈 상태이므로 케이크를 옮기기 전에 opener를
        # 다시 파지한다. pick()가 그리퍼 열기/닫기를 모두 처리한다.
        # Drawing 내부에 Safe Position을 삽입하지 않습니다.
        # 펜 반환, TCP/그리퍼 자세 복귀 완료 후 Plate Finish가 호출되어야 합니다.
        if not self._ensure_gripper_open('finish_start'):
            return False
        if not self._plate_safe_check('safe_position_1', 'finish_start'):
            return False
        if not self._plate_pick_opener():
            return False
        if not self._plate_safe_check('safe_position_1', 'finish_opener'):
            return False
        # 재파지 후 opener 거치대에서 벗어나기 위한 클리어런스 이동.
        if not self._plate_line('move_cake_ps_l2'):
            return False
        for name, kind, relative in (
            ('pickup_cake_station_ps1', 'joint', False),
            ('pickup_cake_station_ps2', 'joint', False),
            ('pickup_cake_station_ps3', 'joint', False),
            ('pickup_cake_station_ps4', 'joint', False),
            ('safe_position_4', 'joint', False),
            ('deliver_cake_station_ps1', 'joint', False),
            ('deliver_cake_station_ps2', 'joint', False),
            ('deliver_cake_station_ps3', 'joint', False),
            ('deliver_cake_station_ps4', 'joint', False),
            ('deliver_cake_station_ps5', 'line', True),
            ('safe_position_5', 'joint', False),
        ):
            if not (self._plate_joint(name, relative) if kind == 'joint'
                    else self._plate_line(name, relative)):
                return False
            if name == 'safe_position_4' and not self._weight_at_safe('finish_both'):
                return False
            if name == 'safe_position_5' and not self._weight_at_safe('finish_opener_after_cake'):
                return False
        # deliver_opener_ps1/ps2는 pick_opener1/2와 동일 좌표: 공통값 재사용.
        # 기존 place()는 접근 위치로 후퇴한다(원본은 해제 직후 종료).
        # [변경 2026-11-19] 전용 plate_place 대신 기존 place()을 그대로 재사용한다.
        # 그리퍼는 place() 내부의 DO2 Open(기본값) 펄스로 오프너를 해제한다.
        if not self._plate_place_opener():
            return False
        # 기존 최종 Safe5 이동 유지. 빈 그리퍼를 검사하고 종료합니다.
        return self._plate_safe_check('safe_position_5', 'finish_end')

