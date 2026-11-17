"""Doosan M0609 + OnRobot RG2 제어 래퍼.

DSR(Doosan Service Robot) ROS2 서비스 클라이언트를 감싸서
MoveLine, MoveJoint, 그리퍼 제어 등을 단순한 메서드로 제공한다.
이 모듈은 rclpy에 의존한다.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dsr_msgs2.srv import MoveHome, MoveJoint, MoveLine, MoveSplineTask
from onrobot_rg_msgs.srv import SetCommand

from .drawing_motion import (
    DEFAULT_ACC_ANGULAR,
    DEFAULT_ACC_LINEAR,
    DEFAULT_VEL_ANGULAR,
    DsrPos,
)

# Doosan 상수
DR_BASE = 0
DR_TOOL = 1
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1
SYNC = 0
ASYNC = 1


class DoosanRobotClient:
    """Doosan M0609 + RG2 ROS2 서비스 클라이언트.

    모든 이동/그리퍼 명령을 동기 호출로 래핑한다.
    """

    # 서비스 이름 상수.
    # 도선 모션 서비스는 /dsr01 네임스페이스의 dsr_controller2 컨트롤러 노드
    # 아래에 등록되므로 실제 경로는 '/dsr01/dsr_controller2/motion/...'이다
    # (가상 에뮬레이터에서 ros2 service list로 확인). '/dsr01/motion/...'으로
    # 두면 클라이언트가 서비스를 찾지 못해 모션이 한 번도 실행되지 않는다.
    # 그리퍼는 가상 GripperVirtualNode가 '/onrobot/sendCommand'로 제공한다.
    _SERVICES = {
        'move_line':   ('/dsr01/dsr_controller2/motion/move_line', MoveLine),
        'move_joint':  ('/dsr01/dsr_controller2/motion/move_joint', MoveJoint),
        'move_home':   ('/dsr01/dsr_controller2/motion/move_home', MoveHome),
        'move_spline': ('/dsr01/dsr_controller2/motion/move_spline_task', MoveSplineTask),
        'gripper':     ('/onrobot/sendCommand', SetCommand),
    }

    def __init__(self, node: Node, service_timeout: float = 5.0):
        self._node = node
        self._timeout = service_timeout
        self._logger = node.get_logger()

        # 서비스 클라이언트 일괄 생성
        self._clients: dict[str, object] = {}
        for key, (topic, srv_type) in self._SERVICES.items():
            self._clients[key] = node.create_client(srv_type, topic)

    # ------------------------------------------------------------------
    # 서비스 호출 헬퍼
    # ------------------------------------------------------------------

    def _call(self, key: str, request, timeout: float = 10.0) -> Optional[object]:
        """동기 서비스 호출 래퍼.

        Returns
        -------
        서비스 응답 객체. 실패 시 None.
        """
        client = self._clients[key]

        if not client.wait_for_service(timeout_sec=self._timeout):
            self._logger.error(f'{key} 서비스 연결 대기 시간 초과')
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)

        if future.result() is None:
            self._logger.error(f'{key} 서비스 호출 실패')
            return None

        return future.result()

    def _call_checked(self, key: str, request, timeout: float = 10.0) -> bool:
        """서비스 호출 후 result.success를 반환한다."""
        result = self._call(key, request, timeout)
        if result is None:
            return False
        if not result.success:
            self._logger.warn(f'{key} 명령 실패')
        return result.success

    # ------------------------------------------------------------------
    # 이동 명령
    # ------------------------------------------------------------------

    def move_line(
        self,
        pos: DsrPos,
        speed_mm_s: float = 50.0,
        acc_mm_s2: float = DEFAULT_ACC_LINEAR,
        ref: int = DR_BASE,
        mode: int = DR_MV_MOD_ABS,
        sync: bool = True,
    ) -> bool:
        """직선 이동 (MoveLine).

        Parameters
        ----------
        pos : 목표 좌표 (mm, ZYZ°)
        speed_mm_s : 선형 속도 (mm/s)
        acc_mm_s2 : 선형 가속도 (mm/s²)
        ref : 좌표 기준 (0=BASE, 1=TOOL)
        mode : 절대(0) / 상대(1)
        sync : 동기(True) / 비동기(False)
        """
        req = MoveLine.Request()
        req.pos = pos.to_list()
        req.vel = [speed_mm_s, DEFAULT_VEL_ANGULAR]
        req.acc = [acc_mm_s2, DEFAULT_ACC_ANGULAR]
        req.time = 0.0
        req.radius = 0.0
        req.ref = ref
        req.mode = mode
        req.blend_type = 0
        req.sync_type = SYNC if sync else ASYNC
        return self._call_checked('move_line', req)

    def move_joint(
        self,
        joints: list[float],
        speed_deg_s: float = 30.0,
        acc_deg_s2: float = 30.0,
        sync: bool = True,
    ) -> bool:
        """관절 이동 (MoveJoint).

        Parameters
        ----------
        joints : 6축 관절 각도 (°)
        speed_deg_s : 각속도 (°/s)
        acc_deg_s2 : 각가속도 (°/s²)
        """
        req = MoveJoint.Request()
        req.pos = joints
        req.vel = speed_deg_s
        req.acc = acc_deg_s2
        req.time = 0.0
        req.radius = 0.0
        req.mode = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = SYNC if sync else ASYNC
        return self._call_checked('move_joint', req)

    def move_home(self) -> bool:
        """홈 위치로 이동."""
        return self._call_checked('move_home', MoveHome.Request())

    def move_spline(
        self,
        positions: list[DsrPos],
        speed_mm_s: float = 20.0,
        acc_mm_s2: float = DEFAULT_ACC_LINEAR,
    ) -> bool:
        """스플라인 경로 이동 (MoveSplineTask).

        여러 웨이포인트를 부드럽게 연결하여 이동한다.
        드로잉 중 연속 경로에 적합하다.
        """
        if not positions:
            return False

        req = MoveSplineTask.Request()
        req.pos_cnt = len(positions)
        req.vel = [speed_mm_s, DEFAULT_VEL_ANGULAR]
        req.acc = [acc_mm_s2, DEFAULT_ACC_ANGULAR]
        req.time = 0.0
        req.ref = DR_BASE
        req.mode = DR_MV_MOD_ABS
        req.opt = 1   # SPLINE_VELOCITY_OPTION_CONST
        req.sync_type = SYNC

        for p in positions:
            msg = Float64MultiArray()
            msg.data = p.to_list()
            req.pos.append(msg)

        return self._call_checked('move_spline', req)

    # ------------------------------------------------------------------
    # 그리퍼 제어
    # ------------------------------------------------------------------

    def gripper_open(self, width_mm: float = 100.0) -> bool:
        """그리퍼 열기. RG2 최대 110mm."""
        return self._gripper_command(f'open({width_mm})')

    def gripper_close(self, width_mm: float = 10.0, force_n: float = 20.0) -> bool:
        """그리퍼 닫기. RG2 범위: 3~40N."""
        return self._gripper_command(f'close({width_mm},{force_n})')

    def _gripper_command(self, command: str) -> bool:
        """OnRobot RG2 SetCommand 서비스 호출."""
        req = SetCommand.Request()
        req.command = command
        result = self._call('gripper', req)
        if result is None:
            return False
        if not result.success:
            self._logger.warn(f'그리퍼 명령 실패: {command} → {result.message}')
        return result.success

    # ------------------------------------------------------------------
    # 상태 확인
    # ------------------------------------------------------------------

    def wait_for_services(self, timeout: float = 10.0) -> bool:
        """필수 서비스 연결 대기.

        Returns
        -------
        모든 서비스 연결 성공 여부
        """
        required = ['move_line', 'move_joint', 'gripper']
        for key in required:
            if not self._clients[key].wait_for_service(timeout_sec=timeout):
                self._logger.error(f'{key} 서비스를 찾을 수 없습니다.')
                return False
            self._logger.info(f'{key} 서비스 연결 완료')
        return True

    # ------------------------------------------------------------------
    # 복합 동작 헬퍼 (pick / place 패턴)
    # ------------------------------------------------------------------

    def pick(
        self,
        target: DsrPos,
        approach: DsrPos,
        grip_width: float,
        grip_force: float = 20.0,
        open_width: float = 100.0,
        speed_approach: float = 80.0,
        speed_slow: float = 30.0,
    ) -> bool:
        """접근 → 그리퍼 열기 → 하강 → 그리퍼 닫기 → 들어올리기.

        Parameters
        ----------
        target : 목표 위치 (물체 파지 높이)
        approach : 접근 위치 (목표 위 높이)
        grip_width : 파지 폭 (mm)
        grip_force : 파지 힘 (N)
        open_width : 그리퍼 열림 폭 (mm)
        """
        if not self.move_line(approach, speed_mm_s=speed_approach):
            return False
        self.gripper_open(open_width)
        if not self.move_line(target, speed_mm_s=speed_slow):
            return False
        self.gripper_close(grip_width, grip_force)
        if not self.move_line(approach, speed_mm_s=speed_slow):
            return False
        return True

    def place(
        self,
        target: DsrPos,
        approach: DsrPos,
        open_width: float = 100.0,
        speed_approach: float = 80.0,
        speed_slow: float = 30.0,
        speed_retract: float = 50.0,
    ) -> bool:
        """접근 → 하강 → 그리퍼 열기 → 후퇴.

        Parameters
        ----------
        target : 배치 위치
        approach : 접근/후퇴 위치
        """
        if not self.move_line(approach, speed_mm_s=speed_approach):
            return False
        if not self.move_line(target, speed_mm_s=speed_slow):
            return False
        self.gripper_open(open_width)
        if not self.move_line(approach, speed_mm_s=speed_retract):
            return False
        return True