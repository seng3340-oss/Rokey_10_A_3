"""Doosan M0609 + 디지털 출력 그리퍼 제어 래퍼.

DSR(Doosan Service Robot) ROS2 서비스 클라이언트를 감싸서
MoveLine, MoveJoint, 그리퍼 제어 등을 단순한 메서드로 제공한다.
그리퍼는 PM 최신 방식인 제어박스 DO 펄스를 기본으로 사용하고,
DO 서비스가 없는 가상환경에서만 기존 /onrobot/sendCommand로 대체한다.
이 모듈은 rclpy에 의존한다.
"""

from __future__ import annotations

from typing import Optional

import threading
import json
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
    GetWorkpieceWeight,
    SetCtrlBoxDigitalOutput,
)
from rcl_interfaces.srv import GetParameters
from onrobot_rg_msgs.srv import GripperPose, SetCommand

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

# ── 그리퍼 디지털 출력 기본값 ──
# PM 최신 디지털 출력 제어 방식: Open=DO2, Close=DO1, 0.5초 펄스.
# 실제 배선이 반대이거나 다른 포트를 쓰는 경우 코드를 고치지 않고
# ROS2 파라미터(gripper_open_do / gripper_close_do / gripper_pulse_sec)만 바꾼다.
# 기본값은 기존 PM 설정을 그대로 유지하기 위한 것이며, 자동으로 다른 포트를
# 시도하지 않는다. 서비스 호출이 실패하면 그 명령은 실패로 끝난다.
GRIPPER_OPEN_DO_DEFAULT = 2
GRIPPER_CLOSE_DO_DEFAULT = 1
GRIPPER_PULSE_SEC_DEFAULT = 0.5
DIGITAL_OUTPUT_MIN_PORT = 1
DIGITAL_OUTPUT_MAX_PORT = 16



def validate_gripper_settings(
    open_do: int,
    close_do: int,
    pulse_sec: float,
) -> tuple[int, int, float]:
    """그리퍼 DO 포트 번호와 펄스 유지 시간을 검사한다.

    Parameters
    ----------
    open_do : Open 신호를 보낼 제어박스 DO 포트 (1~16)
    close_do : Close 신호를 보낼 제어박스 DO 포트 (1~16)
    pulse_sec : ON 상태를 유지할 시간(초). 0보다 커야 한다.

    Returns
    -------
    검사·정규화된 (open_do, close_do, pulse_sec)

    Raises
    ------
    ValueError
        포트 범위를 벗어나거나, Open/Close가 같은 포트이거나,
        펄스 시간이 유한한 양수가 아닐 때. 잘못된 설정으로 로봇을
        움직이지 않도록 노드 시작 단계에서 즉시 실패시킨다.
    """
    for label, value in (("gripper_open_do", open_do), ("gripper_close_do", close_do)):
        # bool은 int의 하위 타입이므로 True/False가 1/0 포트로 해석되지 않게 막는다.
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{label}는 정수여야 합니다: {value!r}")
        if not DIGITAL_OUTPUT_MIN_PORT <= value <= DIGITAL_OUTPUT_MAX_PORT:
            raise ValueError(
                f"{label}는 DO {DIGITAL_OUTPUT_MIN_PORT}~"
                f"{DIGITAL_OUTPUT_MAX_PORT} 범위여야 합니다: {value}"
            )

    if open_do == close_do:
        # 같은 포트를 쓰면 Open/Close를 구분할 수 없어 장비 상태가 불명확해진다.
        raise ValueError(
            f"gripper_open_do와 gripper_close_do가 같습니다: DO{open_do}"
        )

    try:
        pulse = float(pulse_sec)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"gripper_pulse_sec는 숫자여야 합니다: {pulse_sec!r}") from exc
    if not math.isfinite(pulse) or pulse <= 0.0:
        raise ValueError(f"gripper_pulse_sec는 0보다 큰 유한값이어야 합니다: {pulse_sec!r}")

    return open_do, close_do, pulse


class DoosanRobotClient:
    """Doosan M0609 + 디지털 출력 그리퍼 ROS2 서비스 클라이언트.

    모든 이동/그리퍼 명령을 동기 호출로 래핑한다. 그리퍼 기본 백엔드는
    제어박스 DO이며, 가상환경에 DO 서비스가 없을 때만 RG2 서비스로 대체한다.
    """

    # 서비스 이름 상수.
    # 두산 모션 서비스는 /dsr01 네임스페이스의 dsr_controller2 컨트롤러 노드
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
        "move_jointx": ("/dsr01/dsr_controller2/motion/move_jointx", MoveJointx),
        "move_circle": ("/dsr01/dsr_controller2/motion/move_circle", MoveCircle),
        "move_line": ("/dsr01/dsr_controller2/motion/move_line", MoveLine),
        "move_joint": ("/dsr01/dsr_controller2/motion/move_joint", MoveJoint),
        "move_home": ("/dsr01/dsr_controller2/motion/move_home", MoveHome),
        "move_spline": (
            "/dsr01/dsr_controller2/motion/move_spline_task",
            MoveSplineTask,
        ),
        "set_digital_output": (
            "/dsr01/dsr_controller2/io/set_ctrl_box_digital_output",
            SetCtrlBoxDigitalOutput,
        ),
        "get_workpiece_weight": (
            "/dsr01/dsr_controller2/force/get_workpiece_weight", GetWorkpieceWeight,
        ),
        "gripper": ("/onrobot/sendCommand", SetCommand),
        "gripper_pose": ("/onrobot/pose", GripperPose),
    }

    def __init__(
        self,
        node: Node,
        service_timeout: float = 5.0,
        *,
        gripper_open_do: int = GRIPPER_OPEN_DO_DEFAULT,
        gripper_close_do: int = GRIPPER_CLOSE_DO_DEFAULT,
        gripper_pulse_sec: float = GRIPPER_PULSE_SEC_DEFAULT,
        gripper_virtual_fallback: bool = True,
    ):
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

        # 그리퍼 백엔드는 첫 그리퍼 명령 때 한 번만 결정한다.
        # 'digital'이면 제어박스 DO, 'virtual'이면 /onrobot/sendCommand를 쓴다.
        self._gripper_backend: Optional[str] = None
        self._gripper_virtual_fallback = bool(gripper_virtual_fallback)
        self._gripper_open_do = GRIPPER_OPEN_DO_DEFAULT
        self._gripper_close_do = GRIPPER_CLOSE_DO_DEFAULT
        self._gripper_pulse_sec = GRIPPER_PULSE_SEC_DEFAULT
        # OFF 실패 시 출력이 남아 있을 수 있다. 이 플래그가 켜지면 실행 노드는
        # 안전을 위해 후속 자동 홈 복귀도 하지 않는다.
        self.gripper_output_fault = False

        # 노드 생성 시점은 아직 어떤 모터 명령도 보내지 않은 정지 상태다.
        # 이후 운용 중 설정을 바꾸려면 configure_gripper(robot_stopped=True)를
        # 명시해야 하며, 그렇지 않으면 기존 설정을 유지한다.
        self.configure_gripper(
            gripper_open_do,
            gripper_close_do,
            gripper_pulse_sec,
            robot_stopped=True,
        )

        # 그리퍼 현재 조인트 각도(rad). OnRobot 드라이버가 발행하는
        # /onrobot_joint_states의 finger_joint 값을 저장한다.
        # 아직 수신 전이면 None이며, 가상 모드처럼 토픽이 없으면 계속 None이다.
        self._gripper_joint_angle_rad: Optional[float] = None
        self._gripper_joint_sub = node.create_subscription(
            JointState,
            "/onrobot_joint_states",
            self._on_gripper_joint_state,
            10,
            callback_group=self._service_group,
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

    def current_pose(self) -> Optional[list[float]]:
        """현재 활성 TCP를 BASE 기준 [x, y, z, rz1, ry, rz2](mm, ZYZ°)로 읽는다.

        이동 명령은 보내지 않는다. 조회에 실패하면 None을 반환하므로
        호출부는 None일 때 이동을 진행하지 않아야 한다.
        """
        result = self._call("current_pose", GetCurrentPosx.Request(ref=DR_BASE))
        if result is None or not result.success or not result.task_pos_info:
            return None
        return [float(value) for value in result.task_pos_info[0].data[:6]]

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
        # 실물 연결 중 인터넷을 쓸 수 없는 환경에서도 원인을 분석할 수 있게
        # 현재 관절값·TCP·목표를 기존 실행기 로그에 함께 남긴다.
        pose = self._call("current_pose", GetCurrentPosx.Request(ref=DR_BASE))
        current = (
            list(pose.task_pos_info[0].data[:6])
            if pose and pose.success and pose.task_pos_info else None
        )
        diagnostic = dict(
            current_joints_deg=list(joints.pos),
            current_tcp=current,
            target_tcp=target.to_list(),
            solution_space=int(solution.sol_space),
        )
        self._logger.info("SIDE_APPROACH_DIAGNOSTIC " + json.dumps(diagnostic))
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
        if not moved:
            # 자동으로 다른 해를 선택하거나 관절 회전을 재시도하지 않는다.
            after = self._call("current_joints", GetCurrentPosj.Request())
            if after and after.success:
                diagnostic["after_joints_deg"] = list(after.pos)
            self._logger.error("SIDE_APPROACH_FAILED " + json.dumps(diagnostic))
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
        mode: int = DR_MV_MOD_ABS,
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
        req.mode = mode
        req.blend_type = 0
        req.sync_type = SYNC if sync else ASYNC
        return self._call_checked("move_joint", req)

    def get_workpiece_weight(self) -> float | None:
        """작업물 추정 하중을 읽습니다. 실패하면 None을 반환합니다.

        서비스의 실제 단위와 저중량 측정 가능 범위는 실물 장비에서 확인해야 합니다.
        """
        response = self._call('get_workpiece_weight', GetWorkpieceWeight.Request(), timeout=10.0)
        if response is None:
            return None
        weight = float(response.weight)
        if not math.isfinite(weight) or weight < 0.0:
            self._logger.error(f'하중 측정 오류: {weight}')
            return None
        return weight

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
    # 그리퍼 제어 (제어박스 Digital Output)
    # ------------------------------------------------------------------

    def configure_gripper(
        self,
        open_do: int,
        close_do: int,
        pulse_sec: float,
        *,
        robot_stopped: bool = False,
    ) -> bool:
        """그리퍼 Open/Close DO 포트와 펄스 시간을 적용한다.

        실제 장비의 배선이 반대이거나 다른 포트를 쓰는 경우 코드 수정 없이
        ROS2 파라미터만 바꿔 대응하기 위한 진입점이다. 설정 변경 중 로봇이
        움직이면 잘못된 출력으로 사고가 날 수 있으므로, 호출자는 로봇이
        정지한 안전한 상태임을 robot_stopped=True로 명시해야 한다.

        Returns
        -------
        설정 적용 여부. 검증 실패 시 ValueError를 올려 노드 시작을 중단한다.
        """
        if not robot_stopped:
            self._logger.error(
                "그리퍼 포트 설정은 로봇이 정지한 안전한 상태에서만 변경할 수 있습니다."
            )
            return False

        open_do, close_do, pulse_sec = validate_gripper_settings(
            open_do, close_do, pulse_sec
        )
        self._gripper_open_do = open_do
        self._gripper_close_do = close_do
        self._gripper_pulse_sec = pulse_sec
        # 설정이 바뀌면 다음 그리퍼 명령에서 백엔드 가용성을 다시 확인한다.
        self._gripper_backend = None
        self._logger.info(
            f"그리퍼 설정 적용: Open=DO{open_do}, Close=DO{close_do}, "
            f"펄스={pulse_sec:.2f}s"
        )
        return True

    def _ensure_gripper_backend(self) -> Optional[str]:
        """사용할 그리퍼 백엔드를 한 번만 결정한다.

        DO 서비스가 있으면 항상 DO를 사용한다. 가상환경처럼 DO 서비스가
        없을 때만 설정에 따라 /onrobot/sendCommand로 대체한다. 이미 선택한
        백엔드의 호출이 실패해도 다른 포트나 백엔드로 자동 전환하지 않는다.
        """
        if self._gripper_backend is not None:
            return self._gripper_backend

        digital = self._clients.get("set_digital_output")
        if digital is not None and digital.wait_for_service(timeout_sec=self._timeout):
            self._gripper_backend = "digital"
            return self._gripper_backend

        if self._gripper_virtual_fallback:
            virtual = self._clients.get("gripper")
            if virtual is not None and virtual.wait_for_service(
                timeout_sec=self._timeout
            ):
                self._gripper_backend = "virtual"
                self._logger.warn(
                    "제어박스 DO 서비스를 찾지 못해 가상 그리퍼"
                    "(/onrobot/sendCommand)로 대체합니다."
                )
                return self._gripper_backend

        self._logger.error(
            "그리퍼 제어 서비스를 찾지 못했습니다. "
            "set_ctrl_box_digital_output 또는 /onrobot/sendCommand를 확인하세요."
        )
        return None

    # def set_digital_output(self, index: int, value: int) -> bool:
    #     """제어박스 디지털 출력(DO) 상태를 설정한다.

    #     Parameters
    #     ----------
    #     index : 제어박스 DO 포트 번호 (1~16)
    #     value : 출력 값 (1=ON, 0=OFF)

    #     Returns
    #     -------
    #     서비스 호출 성공 여부. 범위를 벗어난 포트/값은 호출하지 않고 실패한다.
    #     """
    #     if not DIGITAL_OUTPUT_MIN_PORT <= index <= DIGITAL_OUTPUT_MAX_PORT:
    #         self._logger.error(f"유효하지 않은 DO 포트입니다: {index}")
    #         return False
    #     if value not in (0, 1):
    #         self._logger.error(f"유효하지 않은 DO 출력값입니다: {value}")
    #         return False

    #     req = SetCtrlBoxDigitalOutput.Request()
    #     req.index = index
    #     req.value = value
    #     return self._call_checked("set_digital_output", req)
    def set_digital_output(self, index: int, value: int) -> bool:
        """제어박스 디지털 출력(DO) 상태를 설정한다.

        Parameters
        ----------
        index : 제어박스 DO 포트 번호 (1~16)
        value : 출력 값 (1=ON, 0=OFF)

        Returns
        -------
        서비스 호출 성공 여부
        """
        req = SetCtrlBoxDigitalOutput.Request()
        req.index = index
        req.value = value
        return self._call_checked("set_digital_output", req)

    def _gripper_pulse(self, port: int, action: str) -> bool:
        """지정 포트를 pulse_sec 동안 ON 했다가 OFF 한다.

        ON/OFF 각각의 서비스 결과를 확인한다. OFF에 실패하면 출력이 남아
        있을 수 있으므로 gripper_output_fault를 세우고 False를 반환한다.
        호출부는 이 False를 받으면 이후 자동 이동을 진행하지 않는다.
        """
        # 새 명령을 시작할 때 이전 실패 플래그를 지운다. 이번 명령이 끝까지
        # 성공해야만 플래그가 꺼진 상태로 남는다.
        self.gripper_output_fault = False

        if not self.set_digital_output(port, 1):
            self.gripper_output_fault = True
            self._logger.error(
                f"그리퍼 {action} 실패: DO{port} ON 명령 실패. "
                "다른 포트로 자동 전환하지 않고 후속 이동을 중단합니다."
            )
            return False

        time.sleep(self._gripper_pulse_sec)

        if not self.set_digital_output(port, 0):
            self.gripper_output_fault = True
            self._logger.error(
                f"그리퍼 {action} 실패: DO{port} OFF 실패. 출력이 남아 있을 수 "
                "있으므로 후속 자동 이동을 중단합니다."
            )
            return False

        return True

    # def gripper_open(self, width_mm: float = 100.0) -> bool:
    #     """그리퍼 열기.

    #     기본은 설정된 Open DO 포트를 0.5초 펄스로 출력하는 PM 최신 방식이다.
    #     가상환경 대체 백엔드에서는 width_mm를 기존 RG2 open 명령으로 전달한다.
    #     """
    #     backend = self._ensure_gripper_backend()
    #     if backend is None:
    #         return False
    #     if backend == "virtual":
    #         return self._gripper_command(f"open({width_mm})")
    #     return self._gripper_pulse(self._gripper_open_do, "열기")

    # def gripper_close(self, width_mm: float = 10.0, force_n: float = 20.0) -> bool:
    #     """그리퍼 닫기.

    #     기본은 설정된 Close DO 포트를 0.5초 펄스로 출력하는 PM 최신 방식이다.
    #     width_mm/force_n은 기존 호출부와 가상 RG2 대체 백엔드 호환을 위해 유지한다.
    #     """
    #     backend = self._ensure_gripper_backend()
    #     if backend is None:
    #         return False
    #     if backend == "virtual":
    #         return self._gripper_command(f"close({width_mm},{force_n})")
    #     return self._gripper_pulse(self._gripper_close_do, "닫기")

    # def _gripper_command(self, command: str) -> bool:
    #     """가상환경 전용 OnRobot RG2 SetCommand 서비스 호출.

    #     실제 장비의 기본 제어 경로가 아니며, DO 서비스가 없는 가상환경에서만
    #     사용한다. 실패해도 DO 포트로 자동 전환하지 않는다.
    #     """
    #     req = SetCommand.Request()
    #     req.command = command
    #     result = self._call("gripper", req)
    #     if result is None:
    #         return False
    #     if not result.success:
    #         self._logger.warn(f"가상 그리퍼 명령 실패: {command} → {result.message}")
    #     return result.success

    def gripper_open(self, width_mm: float = 100.0) -> bool:
        """그리퍼 열기 (제어박스 DO 1번 펄스 출력).

        Parameters
        ----------
        width_mm : 기존 호출부 호환을 위한 열림 폭 인자 (DO 제어에서는 미사용)
        """
        del width_mm
        # 1번 포트를 0.5초간 ON 후 OFF 하여 열림 신호를 전달한다.
        time.sleep(GRIPPER_PULSE_SEC_DEFAULT)
        if not self.set_digital_output(2, 1):
            return False
        time.sleep(GRIPPER_PULSE_SEC_DEFAULT)
        return self.set_digital_output(2, 0)

    def gripper_close(self, width_mm: float = 10.0, force_n: float = 20.0) -> bool:
        """그리퍼 닫기 (제어박스 DO 2번 펄스 출력).

        Parameters
        ----------
        width_mm : 기존 호출부 호환을 위한 파지 폭 인자 (DO 제어에서는 미사용)
        force_n : 기존 호출부 호환을 위한 파지 힘 인자 (DO 제어에서는 미사용)
        """
        del width_mm, force_n
        # 2번 포트를 0.5초간 ON 후 OFF 하여 닫힘 신호를 전달한다.
        time.sleep(GRIPPER_PULSE_SEC_DEFAULT)
        if not self.set_digital_output(1, 1):
            return False
        time.sleep(GRIPPER_PULSE_SEC_DEFAULT)
        return self.set_digital_output(1, 0)

    # ------------------------------------------------------------------
    # 그리퍼 상태 읽기 (OnRobot RG2)
    # 가벼운 물품의 안전예외를 확인하기 위해 사용한다.
    # 무게가 아니라 그리퍼가 물품을 물고 있는지(개폐 상태)를 판단한다.
    # ------------------------------------------------------------------

    def _on_gripper_joint_state(self, msg: JointState) -> None:
        """OnRobot 조인트 상태에서 finger_joint 각도를 저장한다.

        /onrobot_joint_states에는 finger_joint가 포함된다. 이름이 없으면
        첫 번째 position을 사용하고, 비정상 값은 무시한다.
        """
        index = 0
        if "finger_joint" in msg.name:
            index = msg.name.index("finger_joint")
        if index >= len(msg.position):
            return
        value = float(msg.position[index])
        if math.isfinite(value):
            self._gripper_joint_angle_rad = value

    def get_gripper_angle(self) -> Optional[float]:
        """현재 그리퍼 조인트 각도(rad). 수신 전이면 None.

        가벼운 물품의 안전예외를 확인하기 위해 사용한다. 각도 자체는
        그리퍼 개폐 상태를 나타내며 무게나 파지력을 의미하지 않는다.
        """
        return self._gripper_joint_angle_rad

    def get_gripper_angle_deg(self) -> Optional[float]:
        """현재 그리퍼 조인트 각도(deg). 수신 전이면 None."""
        angle = self.get_gripper_angle()
        return None if angle is None else math.degrees(angle)

    def gripper_width_from_angle(self, angle_rad: float) -> Optional[float]:
        """조인트 각도(rad)로 그리퍼 폭(m)을 계산한다.

        가벼운 물품의 안전예외를 확인하기 위해 사용한다. /onrobot/pose는
        알려진 인수 하나로 전체 pose를 계산하는 서비스다. 현재 조인트 각도를
        known.theta로 넣고 pose.x(폭)를 읽는다. 서비스가 없거나 응답이
        비정상이면 None을 반환한다.
        """
        if not math.isfinite(angle_rad):
            return None
        request = GripperPose.Request()
        request.known.theta = float(angle_rad)
        response = self._call("gripper_pose", request, timeout=self._timeout)
        if response is None:
            return None
        width = float(response.pose.x)
        if not math.isfinite(width) or width < 0.0:
            self._logger.error(f"그리퍼 폭 계산 오류: {width}")
            return None
        return width

    def get_gripper_width(self) -> Optional[float]:
        """현재 그리퍼 폭(m). 각도 수신 전이면 None."""
        angle = self.get_gripper_angle()
        if angle is None:
            return None
        return self.gripper_width_from_angle(angle)

    # ------------------------------------------------------------------
    # 상태 확인
    # ------------------------------------------------------------------

    def wait_for_services(self, timeout: float = 10.0) -> bool:
        """필수 서비스 연결 대기.

        Returns
        -------
        모든 서비스 연결 성공 여부
        """
        required = ["move_line", "move_joint"]
        for key in required:
            if not self._clients[key].wait_for_service(timeout_sec=timeout):
                self._logger.error(f"{key} 서비스를 찾을 수 없습니다.")
                return False
            self._logger.info(f"{key} 서비스 연결 완료")

        # 그리퍼는 DO 또는 가상 대체 중 사용 가능한 백엔드를 확인한다.
        # 둘 다 없으면 실제 이동 전에 실패시켜 그리퍼 없는 작업을 막는다.
        if self._ensure_gripper_backend() is None:
            return False
        self._logger.info(f"그리퍼 백엔드: {self._gripper_backend}")
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
        close_wait_sec: float = 0.0,
        close_settle_sec: float = 0.0,
        pre_open_wait_sec: float = 0.0,
    ) -> bool:
        """접근 → 그리퍼 열기 → 하강 → 그리퍼 닫기 → 들어올리기.

        플레이트 오프너도 이 함수를 그대로 재사용한다. 전용 plate_pick을
        두지 않고 대기 시간만 인수로 확장해 기존 호출과 호환한다.

        Parameters
        ----------
        target : 목표 위치 (물체 파지 높이)
        approach : 접근 위치 (목표 위 높이)
        grip_width : 파지 폭 (mm). DO 방식에서는 호환용이며 가상 RG2에서 사용
        grip_force : 파지 힘 (N). DO 방식에서는 호환용이며 가상 RG2에서 사용
        open_width : 그리퍼 열림 폭 (mm). 위와 동일
        close_wait_sec : 닫힘 명령 후 추가 대기(초)
        close_settle_sec : 닫힘 안정화 대기(초)
        pre_open_wait_sec : 접근 후 열림 완료를 기다리는 시간(초)
        """
        if not self.move_line(approach, speed_mm_s=speed_approach):
            return False
        # 그리퍼 명령 실패(특히 OFF 실패) 뒤에는 절대 다음 이동을 보내지 않는다.
        # if not self.gripper_open(open_width):
        #     return False
        # time.sleep(max(0.0, pre_open_wait_sec))
        if not self.move_line(target, speed_mm_s=speed_slow):
            return False
        if not self.gripper_close(grip_width, grip_force):
            return False
        time.sleep(max(0.0, close_wait_sec))
        time.sleep(max(0.0, close_settle_sec))
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
        open_wait_sec: float = 0.0,
    ) -> bool:
        """접근 → 하강 → 그리퍼 열기 → 후퇴.

        플레이트 오프너 반환도 이 함수를 그대로 재사용한다. 전용 plate_place를
        두지 않고 열림 대기 시간만 인수로 확장해 기존 호출과 호환한다.

        Parameters
        ----------
        target : 배치 위치
        approach : 접근/후퇴 위치
        open_wait_sec : 열림 명령 후 추가 대기(초)
        """
        if not self.move_line(approach, speed_mm_s=speed_approach):
            return False
        if not self.move_line(target, speed_mm_s=speed_slow):
            return False
        # 열기 실패 시 후퇴 이동을 보내지 않는다.
        if not self.gripper_open(open_width):
            return False
        time.sleep(max(0.0, open_wait_sec))
        if not self.move_line(approach, speed_mm_s=speed_retract):
            return False
        return True
