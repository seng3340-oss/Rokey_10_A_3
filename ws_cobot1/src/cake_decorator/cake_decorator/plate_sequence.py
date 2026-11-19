"""플레이트 준비 및 후처리 동작 순서.

ROS2 노드가 아닌 일반 Python 모듈입니다.
로봇 실행 노드의 제어 객체와 파라미터 조회 함수를 재사용합니다.
"""

from .doosan_robot import DR_MV_MOD_REL, DR_MV_MOD_ABS, DR_BASE
from .drawing_motion import DsrPos


class PlateSequence:
    """플레이트 전용 이동 시퀀스를 관리합니다."""

    def __init__(self, robot, param):
        self._robot = robot
        self._param = param

    def _plate_joint(self, name: str, relative: bool = False) -> bool:
        return self._robot.move_joint(
            list(self._param('plate_' + name)),
            mode=DR_MV_MOD_REL if relative else DR_MV_MOD_ABS,
        )

    def _plate_line(self, name: str, relative: bool = False) -> bool:
        return self._robot.move_line(
            DsrPos(*self._param('plate_' + name)),
            ref=DR_BASE,
            mode=DR_MV_MOD_REL if relative else DR_MV_MOD_ABS,
        )

    def _plate_opener_target(self) -> DsrPos:
        # [변경] pick()은 절대 목표를 받으므로 원본 BASE 상대 Z -100mm를 합산.
        # 원본 pick_opener1 / pick_opener2 값을 모두 파라미터에서 읽는다.
        approach = self._param('plate_pick_opener1')
        delta = self._param('plate_pick_opener2')
        return DsrPos(*(a + b for a, b in zip(approach, delta)))

    def _plate_pick_opener(self) -> bool:
        """opener 거치대로 이동해 opener를 파지한다.

        safe position 1을 거친 뒤 기존 pick()을 재사용한다. pick() 내부에서
        접근 위치의 그리퍼 열기, 목표 위치의 그리퍼 닫기, 접근 위치 복귀까지
        수행하므로 이 함수는 파지 성공 여부만 반환한다. 거치대를 벗어나는
        클리어런스 이동은 호출부에서 상황에 맞게 추가한다.
        """
        # safe position 1 == 시작후 첫번째 안전자세 무게측정 필요(기준무게)
        # TODO: 기준 무게 측정 기능은 주석만 유지(미구현).
        if not self._plate_joint('safe_position_1'):
            return False
        # [변경 2026-11-19] 전용 plate_pick 대신 기존 pick()을 그대로 재사용한다.
        # 플레이트 이동에 필요한 대기 시간만 pick()의 추가 인수로 전달한다.
        return self._robot.pick(
            target=self._plate_opener_target(),
            approach=DsrPos(*self._param('plate_pick_opener1')),
            grip_width=self._param('opener_grip_width_mm'),
            grip_force=self._param('opener_grip_force_n'),
            open_width=self._param('opener_open_width_mm'),
            close_wait_sec=self._param('opener_close_wait_sec'),
            close_settle_sec=self._param('opener_close_settle_sec'),
            pre_open_wait_sec=self._param('opener_pre_open_wait_sec'),
        )

    def _plate_place_opener(self) -> bool:
        """현재 파지 중인 opener를 원래 거치대에 내려놓는다.

        place() 내부에서 접근 위치로 이동한 뒤 목표 위치에서 그리퍼를 열어
        opener를 해제하고 다시 접근 위치로 후퇴한다. 따라서 호출부는 별도의
        그리퍼 열기 명령을 보내지 않는다.
        """
        return self._robot.place(
            target=self._plate_opener_target(),
            approach=DsrPos(*self._param('plate_pick_opener1')),
            open_width=self._param('opener_open_width_mm'),
            open_wait_sec=self._param('opener_open_wait_sec'),
        )

    def _plate_prepare_steps(self) -> bool:
        if not self._plate_pick_opener():
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
        # 드로잉 전에 그리퍼를 비워야 마커를 파지할 수 있으므로 opener를
        # 거치대에 내려놓는다. place()가 그리퍼 열기까지 처리한다.
        return self._plate_place_opener()

    def _plate_finish_steps(self) -> bool:
        # 스테이션에서 케이크를 들어 올린 뒤 출고하고 오프너를 반환한다.
        # 드로잉으로 그리퍼가 빈 상태이므로 케이크를 옮기기 전에 opener를
        # 다시 파지한다. pick()가 그리퍼 열기/닫기를 모두 처리한다.
        if not self._plate_pick_opener():
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
        # deliver_opener_ps1/ps2는 pick_opener1/2와 동일 좌표: 공통값 재사용.
        # 기존 place()는 접근 위치로 후퇴한다(원본은 해제 직후 종료).
        # [변경 2026-11-19] 전용 plate_place 대신 기존 place()을 그대로 재사용한다.
        # 그리퍼는 place() 내부의 DO2 Open(기본값) 펄스로 오프너를 해제한다.
        if not self._plate_place_opener():
            return False
        # 오프너 반납 후 safe position 5로 최종 이동.
        return self._plate_joint('safe_position_5')
