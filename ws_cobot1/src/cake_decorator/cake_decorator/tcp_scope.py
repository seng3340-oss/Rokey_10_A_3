"""옆면 검증이 끝나면 선택된 TCP를 해제하고 기본 플랜지 좌표로 복원한다."""

import math
import time

from scipy.spatial.transform import Rotation
from dsr_msgs2.srv import (
    CheckMotion,
    GetCurrentPosx,
    GetCurrentTcp,
    GetCurrentToolFlangePosx,
    GetRobotMode,
    MoveStop,
    SetCurrentTcp,
    SetRobotMode,
)


def reset_default_tcp(call, *, stop_running=False, flange_euler="ZYZ"):
    """정지 확인 → 수동 모드 → 빈 TCP 선택 → 플랜지 일치 검사를 수행한다.

    call은 실행기의 session.call 또는 같은 인자를 받는 서비스 호출 함수다.
    실패·종료 중에는 정지 명령을 먼저 보낸다. 정지를 확인하지 못하면 TCP를
    바꾸지 않는다. TCP 등록·삭제, 관절 이동, 그리퍼 조작은 수행하지 않는다.
    """
    prefix = "/dsr01/dsr_controller2/"

    def service(kind, path, **fields):
        response = call(kind, prefix + path, kind.Request(**fields), timeout=5)
        if response is None or not response.success:
            raise RuntimeError(f"기본 TCP 복원 서비스 실패: {path}")
        return response

    if stop_running:
        # 중단된 이동의 좌표 기준을 움직이는 도중에 바꾸지 않도록 먼저 정지한다.
        service(MoveStop, "motion/move_stop", stop_mode=2)
    deadline = time.monotonic() + 5.0
    while service(CheckMotion, "motion/check_motion").status != 0:
        if time.monotonic() >= deadline:
            raise RuntimeError("로봇 정지를 확인하지 못해 TCP를 초기화하지 않았습니다.")
        time.sleep(0.1)

    mode = service(GetRobotMode, "system/get_robot_mode").robot_mode
    if mode not in (0, 1):
        raise RuntimeError("기본 TCP 복원에 필요한 수동·자동 모드를 확인하세요.")
    try:
        if mode != 0:
            service(SetRobotMode, "system/set_robot_mode", robot_mode=0)
        # DRFL set_tcp("")는 선택을 해제하여 초기 TCP 0,0,0,0,0,0으로 돌아간다.
        service(SetCurrentTcp, "tcp/set_current_tcp", name="")
        if service(GetCurrentTcp, "tcp/get_current_tcp").info != "":
            raise RuntimeError("TCP 선택 해제를 확인하지 못했습니다.")

        current = service(GetCurrentPosx, "aux_control/get_current_posx", ref=0)
        flange = service(
            GetCurrentToolFlangePosx,
            "aux_control/get_current_tool_flange_posx",
            ref=0,
        ).pos
        if not current.task_pos_info:
            raise RuntimeError("기본 TCP 위치를 조회하지 못했습니다.")
        pose = list(current.task_pos_info[0].data[:6])
        if not all(math.isfinite(value) for value in [*pose, *flange]):
            raise RuntimeError("기본 TCP 또는 플랜지 조회값이 유효하지 않습니다.")
        pose_rotation = Rotation.from_euler("ZYZ", pose[3:], degrees=True)
        # 실물의 ZYZ와 가상 감시값의 ZYX를 분리한다.
        # 현재 TCP API는 두 모드 모두 ZYZ로 비교한다.
        flange_rotation = Rotation.from_euler(flange_euler, flange[3:], degrees=True)
        if (
            math.dist(pose[:3], flange[:3]) > 1.0
            or (flange_rotation.inv() * pose_rotation).magnitude() > math.radians(1.0)
        ):
            raise RuntimeError(
                f"초기화 후 TCP·플랜지 불일치: TCP={pose}, "
                f"플랜지={list(flange)}, 플랜지 각도 해석={flange_euler}"
            )
    finally:
        # TCP 설정만 바꾸고 작업 전의 운전 모드로 돌려놓는다.
        if mode != 0:
            service(SetRobotMode, "system/set_robot_mode", robot_mode=int(mode))
