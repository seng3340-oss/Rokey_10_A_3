"""Doosan M0609 + OnRobot RG2 제어 래퍼.

DSR(Doosan Service Robot) ROS2 서비스 클라이언트를 감싸서
MoveLine, MoveJoint, 그리퍼 제어 등을 단순한 메서드로 제공한다.
이 모듈은 rclpy에 의존한다.
"""

from __future__ import annotations

from typing import Optional

import threading
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dsr_msgs2.srv import (
    MoveHome,
    MoveJoint,
    MoveLine,
    MoveSplineTask,
    MoveCircle,
    MoveJointx,
    GetCurrentPosx,
    GetCurrentPosj,
    GetSolutionSpace,
    GetCurrentToolFlangePosx,
    Ikin,
    Fkin,
)
from rcl_interfaces.srv import GetParameters
from onrobot_rg_msgs.srv import SetCommand

from .drawing_motion import (
    DEFAULT_ACC_ANGULAR,
    DEFAULT_ACC_LINEAR,
    DEFAULT_VEL_ANGULAR,
    DsrPos,
    MotionType,
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
        "description": ("/dsr01/robot_state_publisher/get_parameters", GetParameters),
        "current_pose": (
            "/dsr01/dsr_controller2/aux_control/get_current_posx",
            GetCurrentPosx,
        ),
        "current_joints": (
            "/dsr01/dsr_controller2/aux_control/get_current_posj",
            GetCurrentPosj,
        ),
        "solution": (
            "/dsr01/dsr_controller2/aux_control/get_solution_space",
            GetSolutionSpace,
        ),
        "inverse": ("/dsr01/dsr_controller2/motion/ikin", Ikin),
        "forward": ("/dsr01/dsr_controller2/motion/fkin", Fkin),
        "flange": ("/dsr01/dsr_controller2/aux_control/get_current_tool_flange_posx", GetCurrentToolFlangePosx),
        "move_jointx": ("/dsr01/dsr_controller2/motion/move_jointx", MoveJointx),
        "move_circle": ("/dsr01/dsr_controller2/motion/move_circle", MoveCircle),
        "move_line": ("/dsr01/dsr_controller2/motion/move_line", MoveLine),
        "move_joint": ("/dsr01/dsr_controller2/motion/move_joint", MoveJoint),
        "move_home": ("/dsr01/dsr_controller2/motion/move_home", MoveHome),
        "move_spline": (
            "/dsr01/dsr_controller2/motion/move_spline_task",
            MoveSplineTask,
        ),
        "gripper": ("/onrobot/sendCommand", SetCommand),
    }

    def __init__(self, node: Node, service_timeout: float = 5.0):
        self._node = node
        self._timeout = service_timeout
        self._logger = node.get_logger()

        # 서비스 클라이언트 일괄 생성
        # 액션 실행 콜백과 서비스 응답을 별도 스레드에서 처리한다.
        self._service_group = ReentrantCallbackGroup()
        self._clients: dict[str, object] = {}
        for key, (topic, srv_type) in self._SERVICES.items():
            self._clients[key] = node.create_client(
                srv_type, topic, callback_group=self._service_group
            )

    # ------------------------------------------------------------------
    # 서비스 호출 헬퍼
    # ------------------------------------------------------------------

    def _call(self, key: str, request, timeout: float = 180.0) -> Optional[object]:
        """동기 서비스 호출 래퍼.

        Returns
        -------
        서비스 응답 객체. 실패 시 None.
        """
        client = self._clients[key]

        if not client.wait_for_service(timeout_sec=self._timeout):
            self._logger.error(f"{key} 서비스 연결 대기 시간 초과")
            return None

        future = client.call_async(request)
        # 이미 실행 중인 executor를 중첩 spin하면 응답 처리 노드가 빠질 수 있다.
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(timeout):
            self._logger.error(f"{key} 응답 시간 초과: 완료 여부 확인 필요")
            return None

        if future.result() is None:
            self._logger.error(f"{key} 서비스 호출 실패")
            return None

        return future.result()

    def _call_checked(self, key: str, request, timeout: float = 180.0) -> bool:
        """서비스 호출 후 result.success를 반환한다."""
        result = self._call(key, request, timeout)
        if result is None:
            return False
        if not result.success:
            self._logger.warn(f"{key} 명령 실패")
            return False
        # 실제 제어기도 도달 불가 경고와 함께 success=True를 반환한 사례가 있다.
        # 동기 BASE 절대 이동은 TCP 위치·자세까지 확인해야 완료로 처리할 수 있다.
        if (
            key in {"move_line", "move_circle", "move_spline"}
            and request.sync_type == SYNC
            and request.ref == DR_BASE
            and request.mode == DR_MV_MOD_ABS
        ):
            target = (
                list(request.pos)
                if key == "move_line" else list(request.pos[-1].data)
            )
            return self.verify_pose(target)
        return True

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
        *,
        angular_speed_deg_s: float = DEFAULT_VEL_ANGULAR,
        angular_acc_deg_s2: float = DEFAULT_ACC_ANGULAR,
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
        req.vel = [speed_mm_s, angular_speed_deg_s]
        req.acc = [acc_mm_s2, angular_acc_deg_s2]
        req.time = 0.0
        req.radius = 0.0
        req.ref = ref
        req.mode = mode
        req.blend_type = 0
        req.sync_type = SYNC if sync else ASYNC
        return self._call_checked("move_line", req)

    def move_circle(
        self,
        via: DsrPos,
        target: DsrPos,
        speed_mm_s: float,
        acc_mm_s2: float = DEFAULT_ACC_LINEAR,
        *,
        angular_speed_deg_s: float = DEFAULT_VEL_ANGULAR,
        angular_acc_deg_s2: float = DEFAULT_ACC_ANGULAR,
    ) -> bool:
        """현재점→경유점→목표점의 원호를 요청한다. 표면을 깎는 블렌딩은 사용하지 않는다."""
        request = MoveCircle.Request()
        for point in (via, target):
            message = Float64MultiArray()
            message.data = point.to_list()
            request.pos.append(message)
        request.vel = [speed_mm_s, angular_speed_deg_s]
        request.acc = [acc_mm_s2, angular_acc_deg_s2]
        request.time = 0.0
        request.radius = 0.0
        request.ref = DR_BASE
        request.mode = DR_MV_MOD_ABS
        request.angle1 = 0.0
        request.angle2 = 0.0
        request.blend_type = 0
        request.sync_type = SYNC
        return self._call_checked("move_circle", request)

    def prepare_virtual_approach(self, target: DsrPos) -> bool:
        """가상 초기 접근만 관절 경로로 실행한다. 실제 장비에서는 항상 거부한다.

        접촉 영점의 수직 자세에서 곧바로 옆면 자세로 movel하면 손목 특이점에
        걸릴 수 있다. 먼저 현재 자세로 위로 이탈하고, 그 다음 목표 자세로 간다.
        그리기 명령과 실제 모드의 접근 방식은 이 함수에서 변경하지 않는다.
        """
        description = self._call(
            "description", GetParameters.Request(names=["robot_description"])
        )
        if description is None or not description.values:
            return False
        root = ET.fromstring(description.values[0].string_value)
        params = {
            item.get("name"): item.text
            for item in root.findall(".//ros2_control/hardware/param")
        }
        if params.get("mode") != "virtual" or params.get("host") != "127.0.0.1":
            self._logger.error(
                "가상 초기 관절 접근은 virtual/127.0.0.1에서만 허용됩니다."
            )
            return False
        pose = self._call("current_pose", GetCurrentPosx.Request(ref=DR_BASE))
        joints = self._call("current_joints", GetCurrentPosj.Request())
        if (
            pose is None
            or joints is None
            or not pose.success
            or not joints.success
            or not pose.task_pos_info
        ):
            return False
        current = list(pose.task_pos_info[0].data[:6])
        solution = self._call(
            "solution", GetSolutionSpace.Request(pos=list(joints.pos))
        )
        if solution is None or not solution.success:
            return False
        lifted = current[:]
        # 펜촉뿐 아니라 기울어지는 펜 몸체의 공간도 확보한다. 낮은 목표점에서
        # 곧바로 자세를 바꾸면 관절 경로가 케이크 안으로 휘어질 수 있다.
        lifted[2] = max(current[2], target.z) + 100.0
        rotated = target.to_list()
        rotated[2] = lifted[2]
        # 긴 펜을 기울이면 플랜지가 더 멀어진다. 높은 위치에서 BASE 쪽으로
        # 물러나 회전한 다음 작업점으로 접근해 도달 범위를 확보한다.
        rotated[0] *= 0.7
        rotated[1] *= 0.7
        # 가상 제어기의 직선 접근은 높은 지점에서도 손목 특이점을 통과했다.
        # 마지막 접근도 관절 속도로 제한하고, 각 지점의 실제 도달을 검사한다.
        for goal in (lifted, rotated, target.to_list()):
            request = MoveJointx.Request(
                pos=goal,
                vel=20.0,
                acc=40.0,
                sol=solution.sol_space,
                ref=DR_BASE,
                mode=DR_MV_MOD_ABS,
                sync_type=SYNC,
            )
            if not self._call_checked("move_jointx", request):
                return False
            if not self.verify_pose(goal):
                return False
        return True

    def verify_pose(self, target) -> bool:
        """가상·실제 모두 응답만 성공한 이동을 걸러내고 TCP 도달을 확인한다."""
        from scipy.spatial.transform import Rotation

        deadline = time.monotonic() + 1.0
        expected = Rotation.from_euler("ZYZ", target[3:], degrees=True)
        while True:
            actual = None
            result = self._call("current_pose", GetCurrentPosx.Request(ref=DR_BASE))
            if result and result.success and result.task_pos_info:
                actual = list(result.task_pos_info[0].data[:6])
                # 같은 자세도 ZYZ 표현은 달라질 수 있어 회전 차이로 비교한다.
                rotation = Rotation.from_euler("ZYZ", actual[3:], degrees=True)
                angle = math.degrees((expected.inv() * rotation).magnitude())
                if math.dist(actual[:3], target[:3]) < 0.5 and angle < 0.5:
                    return True
            if time.monotonic() >= deadline:
                self._logger.error(
                    f"이동 목표 미도달: 목표={target}, "
                    f"관측={actual if actual is not None else '조회 실패'}"
                )
                return False
            time.sleep(0.05)

    def select_reachable_side_commands(self, commands, cancelled):
        """이동 전에 제어기의 IK/FK를 조회해 현재 관절 해에서 가능한 펜 자세를 찾는다."""
        import numpy as np
        from scipy.spatial.transform import Rotation
        from .side_pose_selection import select_side_roll

        self._side_validated_solution = None
        joints = self._call("current_joints", GetCurrentPosj.Request(), timeout=3)
        pose = self._call("current_pose", GetCurrentPosx.Request(ref=0), timeout=3)
        flange = self._call("flange", GetCurrentToolFlangePosx.Request(ref=0), timeout=3)
        if not (joints and joints.success and pose and pose.success and pose.task_pos_info and flange and flange.success):
            raise RuntimeError("옆면 도달 검사에 필요한 현재 로봇 상태를 읽지 못했습니다.")
        solution = self._call("solution", GetSolutionSpace.Request(pos=list(joints.pos)), timeout=3)
        current_fk = self._call("forward", Fkin.Request(pos=list(joints.pos), ref=0), timeout=3)
        if not (solution and solution.success and current_fk and current_fk.success):
            raise RuntimeError("현재 관절 해·순기구학 조회 실패")
        tcp = list(pose.task_pos_info[0].data[:6])
        flange_pose = list(flange.pos)

        def matches(first, second):
            distance = math.dist(first[:3], second[:3])
            rotation = (Rotation.from_euler("ZYZ", first[3:], degrees=True).inv()
                        * Rotation.from_euler("ZYZ", second[3:], degrees=True))
            return distance < 0.5 and math.degrees(rotation.magnitude()) < 0.5

        # 드라이버 버전마다 FK의 출력 기준이 다를 수 있어 현재 실제 조회값으로 확인한다.
        flange_basis = False
        if not matches(list(current_fk.conv_posx), tcp):
            if not matches(list(current_fk.conv_posx), flange_pose):
                raise RuntimeError("현재 FK가 TCP·플랜지 어느 쪽과도 맞지 않아 도달 검사를 중단합니다.")
            flange_basis = True
        flange_rotation = Rotation.from_euler("ZYZ", flange_pose[3:], degrees=True)
        tcp_rotation = Rotation.from_euler("ZYZ", tcp[3:], degrees=True)
        local_rotation = flange_rotation.inv() * tcp_rotation
        local_offset = flange_rotation.inv().apply(np.array(tcp[:3]) - np.array(flange_pose[:3]))
        deadline = time.monotonic() + 90.0

        def inverse(target):
            if time.monotonic() > deadline:
                raise RuntimeError("옆면 도달 검사가 90초를 넘었습니다. 이동하지 않습니다.")
            native = target.to_list()
            if flange_basis:
                rotation = Rotation.from_euler("ZYZ", native[3:], degrees=True) * local_rotation.inv()
                position = np.array(native[:3]) - rotation.apply(local_offset)
                native = [*map(float, position), *map(float, rotation.as_euler("ZYZ", degrees=True))]
            result = self._call("inverse", Ikin.Request(pos=native, sol_space=solution.sol_space, ref=0), timeout=3)
            if not result or not result.success:
                return None
            values = list(result.conv_posj)
            if not all(math.isfinite(value) for value in values):
                return None
            check = self._call("forward", Fkin.Request(pos=values, ref=0), timeout=3)
            # 일부 드라이버는 IK 실패도 success=True를 반환하므로 FK 왕복으로 검증한다.
            if not check or not check.success or not matches(list(check.conv_posx), native):
                return None
            return values

        self._logger.info("옆면 도달 가능 자세 검사 시작. 아직 이동하지 않습니다.")
        selected = select_side_roll(commands, list(joints.pos), inverse, cancelled, self._logger.info)
        after = self._call("current_joints", GetCurrentPosj.Request(), timeout=3)
        if not after or not after.success or max(abs(a-b) for a,b in zip(joints.pos, after.pos)) > 0.1:
            raise RuntimeError("도달 검사 중 로봇 자세가 바뀌었습니다. 다시 영점을 준비하세요.")
        self._side_validated_solution = int(solution.sol_space)
        return selected

    def move_task_joint(self, target: DsrPos) -> bool:
        """옆면 케이크 바깥 경유점에서 자세를 바꾼다. 그리기 원호에는 사용하지 않는다."""
        joints = self._call("current_joints", GetCurrentPosj.Request())
        if joints is None or not joints.success:
            return False
        solution = self._call(
            "solution", GetSolutionSpace.Request(pos=list(joints.pos))
        )
        if solution is None or not solution.success:
            return False
        validated = getattr(self, "_side_validated_solution", None)
        if validated is not None and int(solution.sol_space) != validated:
            self._logger.error("검사한 관절 해와 현재 해가 달라 자세 전환을 중단합니다.")
            return False
        self._logger.info(
            f"옆면 자세 전환: 현재 팔꿈치 해 {solution.sol_space} 유지, "
            f"목표 TCP={target.to_list()}"
        )
        request = MoveJointx.Request(
            pos=target.to_list(),
            vel=10.0,
            acc=20.0,
            sol=solution.sol_space,
            ref=DR_BASE,
            mode=DR_MV_MOD_ABS,
            sync_type=SYNC,
        )
        moved = self._call_checked("move_jointx", request) and self.verify_pose(
            target.to_list()
        )
        return moved

    def execute_drawing_command(self, command) -> bool:
        """이동 종류만 분기한다. 액션 반복·획 완료 판단은 RE에 그대로 둔다."""
        # 계획에서 접촉 후 옆면에만 높인 각속도·각가속도를 API 요청까지 전달한다.
        angular = dict(
            angular_speed_deg_s=command.angular_vel,
            angular_acc_deg_s2=command.angular_acc,
        )
        if command.motion_type == MotionType.MOVE_SPLINE:
            if not command.spline or not 2 <= len(command.spline) <= 100:
                self._logger.error("스플라인 목표점 수가 유효하지 않습니다.")
                return False
            return self.move_spline(command.spline, command.vel, command.acc, **angular)
        if command.motion_type == MotionType.MOVE_CIRCLE:
            if command.via is None:
                self._logger.error(
                    "원호 경유점이 없습니다. 직선으로 대체하지 않습니다."
                )
                return False
            return self.move_circle(command.via, command.pos, command.vel, command.acc, **angular)
        if command.motion_type != MotionType.MOVE_LINE:
            self._logger.error("지원하지 않는 드로잉 이동 종류입니다.")
            return False
        return self.move_line(
            pos=command.pos, speed_mm_s=command.vel, acc_mm_s2=command.acc,
            **angular,
        )

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
        return self._call_checked("move_joint", req)

    def move_home(self) -> bool:
        """홈 위치로 이동."""
        return self._call_checked("move_home", MoveHome.Request())

    def move_spline(
        self,
        positions: list[DsrPos],
        speed_mm_s: float = 20.0,
        acc_mm_s2: float = DEFAULT_ACC_LINEAR,
        *,
        angular_speed_deg_s: float = DEFAULT_VEL_ANGULAR,
        angular_acc_deg_s2: float = DEFAULT_ACC_ANGULAR,
    ) -> bool:
        """스플라인 경로 이동 (MoveSplineTask).

        여러 웨이포인트를 부드럽게 연결하여 이동한다.
        드로잉 중 연속 경로에 적합하다.
        """
        if not positions:
            return False

        req = MoveSplineTask.Request()
        req.pos_cnt = len(positions)
        req.vel = [speed_mm_s, angular_speed_deg_s]
        req.acc = [acc_mm_s2, angular_acc_deg_s2]
        req.time = 0.0
        req.ref = DR_BASE
        req.mode = DR_MV_MOD_ABS
        # 촘촘한 점은 정속 조건(최소 구간 거리)을 만족하지 못한다.
        # 제어기의 기본 가변 속도 보간을 명시해 불필요한 정속 전환 경고를 피한다.
        req.opt = 0  # SPLINE_VELOCITY_OPTION_DEFAULT
        req.sync_type = SYNC

        for p in positions:
            msg = Float64MultiArray()
            msg.data = p.to_list()
            req.pos.append(msg)

        return self._call_checked("move_spline", req)

    # ------------------------------------------------------------------
    # 그리퍼 제어
    # ------------------------------------------------------------------

    def gripper_open(self, width_mm: float = 100.0) -> bool:
        """그리퍼 열기. RG2 최대 110mm."""
        return self._gripper_command(f"open({width_mm})")

    def gripper_close(self, width_mm: float = 10.0, force_n: float = 20.0) -> bool:
        """그리퍼 닫기. RG2 범위: 3~40N."""
        return self._gripper_command(f"close({width_mm},{force_n})")

    def _gripper_command(self, command: str) -> bool:
        """OnRobot RG2 SetCommand 서비스 호출."""
        req = SetCommand.Request()
        req.command = command
        result = self._call("gripper", req)
        if result is None:
            return False
        if not result.success:
            self._logger.warn(f"그리퍼 명령 실패: {command} → {result.message}")
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
        required = ["move_line", "move_joint", "gripper"]
        for key in required:
            if not self._clients[key].wait_for_service(timeout_sec=timeout):
                self._logger.error(f"{key} 서비스를 찾을 수 없습니다.")
                return False
            self._logger.info(f"{key} 서비스 연결 완료")
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
