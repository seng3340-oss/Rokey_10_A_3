"""팀 거치대 티칭과 실물 검증된 220mm 피벗으로 옆면 펜을 순환한다.

스크립트를 실행하지 않고 기존 DoosanRobotClient의 서비스 그룹과 DO 그리퍼를
사용한다. 모든 작업 좌표는 BASE mm·ZYZ degree이며 티칭값은 플랜지 영점이다.
"""

from dataclasses import dataclass
import math
import uuid
import warnings

import numpy as np
from scipy.spatial.transform import Rotation
from dsr_msgs2.srv import (
    CheckMotion, ConfigCreateTcp, ConfigDeleteTcp, GetCurrentPosj,
    GetCurrentPosx, GetCurrentTcp, GetCurrentToolFlangePosx, GetRobotMode,
    MoveStop, SetCurrentTcp, SetRobotMode,
)

from .bent_pen import SIDE_TCP_NAME, SIDE_TCP_OFFSET_MM
from .drawing_motion import DsrPos
from .tcp_scope import reset_default_tcp


# real_pen_pick_45.py에서 검증한 손끝 지문 중심. grip TCP의 240mm와 다르다.
GRIP_CENTER_MM = (0.0, 0.0, 220.0)


def checked_pose(values):
    """설정·조회값을 유한한 mm/degree 6원소로 정규화한다."""
    if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError("유한한 자세/관절 6값이 필요합니다.")
    return [float(value) for value in values]


def zyz(rotation):
    """수직 자세의 동등한 ZYZ 표현을 경고 없이 계산한다."""
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected.*")
        return rotation.as_euler("ZYZ", degrees=True).tolist()


def flange_to_tcp(pose, offset):
    """플랜지 영점 목표에 TOOL mm 오프셋을 더한다(장착 회전은 0)."""
    rotation = Rotation.from_euler("ZYZ", pose[3:], degrees=True)
    xyz = np.asarray(pose[:3]) + rotation.apply(offset)
    return [*xyz.tolist(), *pose[3:]]


@dataclass
class PickGeometry:
    """파지 상공 관절과 하강→기울임→인출→수직복귀의 플랜지 목표."""

    approach_joints: list[float]
    flange_poses: list[list[float]]


def pick_geometry(rack):
    """현재 YAML 티칭에서 +Y 45도 파지 자세를 계산한다.

    real_pen_pick_45.read_rack_and_poses의 단일 회전 알고리즘을 이식했다.
    실제 실행은 이 플랜지 목표를 220mm TCP로 변환해 중심 고정 보간한다.
    """
    joints = checked_pose(rack["pen_stand_approach_pose"])
    stand = checked_pose(rack["pen_stand_pose"])
    lift = checked_pose(rack["pen_gripper_pose"])
    original = Rotation.from_euler("ZYZ", stand[3:], degrees=True)
    original_axis = original.apply([0., 0., 1.])
    if original_axis[2] > -math.cos(math.radians(2.)):
        raise ValueError("팀 파지 티칭의 +Z가 수직 아래를 향해야 합니다.")
    clearance = lift[2] - stand[2]
    if clearance <= 0.:
        raise ValueError("펜 인출 높이는 파지 높이보다 높아야 합니다.")
    target_axis = np.array([0., math.sqrt(.5), -math.sqrt(.5)])
    axis = np.cross(original_axis, target_axis)
    angle = math.atan2(float(np.linalg.norm(axis)), float(original_axis @ target_axis))
    tilted_rotation = Rotation.from_rotvec(axis / np.linalg.norm(axis) * angle) * original
    center = np.asarray(stand[:3]) + original.apply(GRIP_CENTER_MM)
    tilted = [*(center - tilted_rotation.apply(GRIP_CENTER_MM)).tolist(), *zyz(tilted_rotation)]
    upper = tilted.copy()
    upper[2] += clearance
    upright = stand.copy()
    upright[2] += clearance
    return PickGeometry(joints, [stand, tilted, upper, upright])


class IntegratedPenCycle:
    """주문 단위 펜 집기·반납과 TCP 서비스 변경을 담당한다."""

    # 부모 클라이언트에 없는 TCP 등록/삭제 등만 같은 응답 그룹에 추가한다.
    # 기존 키를 교체하지 않아 다른 작업자의 DoosanRobotClient 병합과 공존한다.
    _SERVICES = {
        "get_tcp": ("tcp/get_current_tcp", GetCurrentTcp),
        "set_tcp": ("tcp/set_current_tcp", SetCurrentTcp),
        "get_mode": ("system/get_robot_mode", GetRobotMode),
        "set_mode": ("system/set_robot_mode", SetRobotMode),
        "create_tcp": ("tcp/config_create_tcp", ConfigCreateTcp),
        "delete_tcp": ("tcp/config_delete_tcp", ConfigDeleteTcp),
        "current_flange": (
            "aux_control/get_current_tool_flange_posx", GetCurrentToolFlangePosx,
        ),
        "current_pose": ("aux_control/get_current_posx", GetCurrentPosx),
        "current_joints": ("aux_control/get_current_posj", GetCurrentPosj),
        "check_motion": ("motion/check_motion", CheckMotion),
        "move_stop": ("motion/move_stop", MoveStop),
    }

    def __init__(self, robot, parameter):
        """클라이언트만 준비한다. 생성 시 서비스·하드웨어 명령은 보내지 않는다."""
        self.robot = robot
        self.parameter = parameter
        for key, (path, kind) in self._SERVICES.items():
            if key not in robot._clients:
                robot._clients[key] = robot._node.create_client(
                    kind, "/dsr01/dsr_controller2/" + path,
                    callback_group=robot._service_group,
                )

    def _call(self, key, **fields):
        """기존 동기 호출을 사용하고 실패 응답을 즉시 예외로 바꾼다."""
        kind = self._SERVICES[key][1]
        response = self.robot._call(key, kind.Request(**fields), timeout=5.)
        if response is None or not response.success:
            raise RuntimeError(f"통합 펜 서비스 실패: {key}")
        return response

    def ensure_default(self, *, stop_running=False):
        """정지 후 영점 TCP를 선택하고 실제 플랜지 일치까지 확인한다."""
        by_path = {
            "/dsr01/dsr_controller2/" + path: key
            for key, (path, _) in self._SERVICES.items()
        }

        def call(kind, path, request, *, timeout=5):
            return self.robot._call(by_path[path], request, timeout=timeout)

        # 통합 모드는 실물에서 검증한 ZYZ 플랜지 기준을 사용한다.
        # TOP/플레이트 티칭에 named nonzero TCP가 섞이지 않게 항상 수치 검증한다.
        outcome = reset_default_tcp(call, stop_running=stop_running)
        self.robot._logger.info(
            "기본 TCP 이미 적용됨: 정지·플랜지 일치 확인 완료"
            if outcome == "already_default"
            else "기본 TCP 선택 해제 및 플랜지 일치 확인 완료"
        )

    def _configure(self, operation):
        """정지 상태의 TCP 변경을 수동 모드에서 하고 원래 모드를 복원한다."""
        if self._call("check_motion").status != 0:
            raise RuntimeError("TCP 변경 전 로봇 정지를 확인하세요.")
        mode = self._call("get_mode").robot_mode
        if mode not in (0, 1):
            raise RuntimeError("TCP 변경은 수동/자동 모드에서만 가능합니다.")
        try:
            if mode != 0:
                self._call("set_mode", robot_mode=0)
            operation()
        finally:
            if mode != 0:
                self._call("set_mode", robot_mode=int(mode))

    def select(self, name):
        """기존 TCP를 선택하고 응답뿐 아니라 선택 이름도 확인한다."""
        self._configure(lambda: self._call("set_tcp", name=name))
        if self._call("get_tcp").info != name:
            raise RuntimeError(f"TCP 적용을 확인하지 못했습니다: {name}")

    def verify_side_tcp(self):
        """등록 이름뿐 아니라 실측 플랜지→펜촉 변환도 검증한다.

        펜던트 교정 오프셋 (1.095,-46.474,271.695)mm와 장착 회전 0을
        확인한다. 이름만 같은 잘못된 TCP로 케이크에 접근하는 것을 막는다.
        """
        flange = checked_pose(self._call("current_flange", ref=0).pos)
        current = self._call("current_pose", ref=0)
        if not current.task_pos_info:
            raise RuntimeError("SIDE TCP 자세 조회 실패")
        tip = checked_pose(current.task_pos_info[0].data[:6])
        flange_rotation = Rotation.from_euler("ZYZ", flange[3:], degrees=True)
        tip_rotation = Rotation.from_euler("ZYZ", tip[3:], degrees=True)
        offset = flange_rotation.inv().apply(np.asarray(tip[:3]) - np.asarray(flange[:3]))
        if (
            np.linalg.norm(offset - np.asarray(SIDE_TCP_OFFSET_MM)) > 1.
            or (flange_rotation.inv() * tip_rotation).magnitude() > math.radians(1.)
        ):
            raise RuntimeError(f"SIDE TCP 장착 변환 불일치: TOOL mm={offset.tolist()}")

    def _rack(self):
        """주문 시작 시 현재 ROS 파라미터를 복사해 한 순환 동안 고정한다."""
        names = (
            "pen_stand_approach_pose", "pen_stand_pose", "pen_gripper_pose",
            "pen_drop_approach_pose", "pen_drop_pose", "pen_drop_after_pose",
        )
        return {name: checked_pose(self.parameter(name)) for name in names}

    @staticmethod
    def _check_cancel(cancelled):
        """취소 후에는 다음 이동이나 그리퍼 명령을 보내지 않는다."""
        if cancelled():
            raise RuntimeError("통합 펜 순환 취소 요청")

    def pick_side(
        self,
        cancelled=lambda: False,
        *,
        reuse_rack_position=False,
        preserve_speed=False,
        extra_lift_mm=0.0,
    ):
        """220mm 피벗으로 파지 후 Pen_Side_45를 선택하고 되감기 관절을 반환한다.

        preserve_speed는 기존 호출과의 호환을 위해 받는다. 연속 주문 여부와
        무관하게 현재 펜 이동 설정을 사용해 느린 고정값이 다시 적용되지 않게 한다.
        """
        self.rack = self._rack()
        geometry = pick_geometry(self.rack)
        if not math.isfinite(extra_lift_mm) or not 0 <= extra_lift_mm <= 50:
            raise ValueError("추가 인출 높이는 0~50mm여야 합니다.")
        # 기울어진 인출과 수직 복귀를 같은 높이로 올려 피벗을 유지한다.
        for pose in geometry.flange_poses[2:]:
            pose[2] += extra_lift_mm
        pivot = "PickPivot_" + uuid.uuid4().hex[:12]
        registered = False
        self._check_cancel(cancelled)
        self.ensure_default()
        try:
            self._configure(lambda: self._call(
                "create_tcp", name=pivot, pos=[*GRIP_CENTER_MM, 0., 0., 0.],
            ))
            registered = True
            self.select(pivot)
            self._check_cancel(cancelled)
            skip_approach = False
            if reuse_rack_position:
                # TOP 반납 위치를 확인한 뒤 제자리 기울임으로 이어간다. 불일치는 중단한다.
                actual = checked_pose(self._call("current_flange", ref=0).pos)
                expected = self.rack["pen_drop_pose"]
                angle = (
                    Rotation.from_euler("ZYZ", actual[3:], degrees=True).inv()
                    * Rotation.from_euler("ZYZ", expected[3:], degrees=True)
                ).magnitude()
                skip_approach = (
                    math.dist(actual[:3], expected[:3]) < 0.5
                    and angle < math.radians(0.5)
                    and actual[2] >= geometry.flange_poses[0][2] - 0.5
                    and math.dist(actual[:2], geometry.flange_poses[0][:2]) < 1.0
                )
            if reuse_rack_position and not skip_approach:
                raise RuntimeError("TOP 반납 위치 불일치: 제자리 재파지를 중단합니다.")
            if skip_approach:
                self.robot._logger.info("TOP 반납 위치 확인: 상공 이동 없이 제자리 45도 재파지")
            elif not self.robot.move_joint(
                geometry.approach_joints,
                speed_deg_s=self.parameter("free_joint_speed_deg_s"),
                acc_deg_s2=self.parameter("free_joint_acc_deg_s2"),
            ):
                raise RuntimeError("45도 펜 거치대 상공 이동 실패")
            self._check_cancel(cancelled)
            # PR23과 동일하게 파지 직전 열기 펄스를 반복하지 않는다.
            # 앞선 도구 반납에서 열린 상태로 내려가 기울인 뒤 닫기만 실행한다.
            self.robot._logger.info("SIDE 펜 파지: 추가 열기 없이 하강 → 기울임 → 닫기")
            for index, flange_pose in enumerate(geometry.flange_poses):
                self._check_cancel(cancelled)
                pose = flange_to_tcp(flange_pose, GRIP_CENTER_MM)
                # 기울임·수직 복귀만 감속하고 하강·인출의 기존 회전 제한은 유지한다.
                rotation = "pen_tilt" if index in (1, 3) else "pen_rotation"
                if not self.robot.move_line(
                    DsrPos(*pose),
                    # 펜 전용 속도를 사용한다. 실제 잉크 경로 속도는 별도다.
                    speed_mm_s=self.parameter("pen_rack_speed_mm_s") if index == 0 else self.parameter("pen_lift_speed_mm_s"),
                    acc_mm_s2=self.parameter("pen_motion_acc_mm_s2"),
                    angular_speed_deg_s=self.parameter(f"{rotation}_speed_deg_s"),
                    angular_acc_deg_s2=self.parameter(f"{rotation}_acc_deg_s2"),
                ) or not self.robot.verify_pose(pose):
                    raise RuntimeError(f"45도 펜 파지 이동/도달 실패: {index}")
                if index == 1:
                    # 중심 고정 기울임이 끝난 뒤에만 닫고, 인출 후에만 수직 복귀한다.
                    self._check_cancel(cancelled)
                    if not self.robot.gripper_close(
                        self.parameter("pen_grip_width"), self.parameter("pen_grip_force"),
                    ):
                        raise RuntimeError("45도 펜 그리퍼 닫기 실패")
        except BaseException:
            self.ensure_default(stop_running=True)
            raise
        finally:
            # 정상/실패 모두 피벗을 해제한 뒤 삭제한다. 선택 해제 실패 시 삭제하지 않는다.
            if registered:
                self.ensure_default()
                self._configure(lambda: self._call("delete_tcp", name=pivot))
        self._check_cancel(cancelled)
        try:
            self.select(SIDE_TCP_NAME)
            self.verify_side_tcp()
            return checked_pose(self._call("current_joints").pos)
        except BaseException:
            self.ensure_default(stop_running=True)
            raise

    def return_side(self, initial_joints, cancelled=lambda: False):
        """J6를 되감고 현재 YAML 거치대에 DO로 반납한 뒤 영점으로 후퇴한다."""
        from .side_pen_return import return_pen

        return_pen(
            self.robot, initial_joints, cancelled,
            rack=getattr(self, "rack", None) or self._rack(),
            open_gripper=lambda: self.robot.gripper_open(self.parameter("gripper_open_width")),
            reset_tcp=self.ensure_default,
            parameter=self.parameter,
        )
