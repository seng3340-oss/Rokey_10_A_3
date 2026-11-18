"""알려진 상면 높이에서 수직 접촉한 펜촉의 TCP를 계산·적용한다.

상면 높이를 아는 수직 접촉 자세에서 펜촉 오프셋을 추정한다. 실제 펜이 수직이고,
접촉 당시 활성 TCP의 XY가 펜촉의 XY와 같다는 가정이 필요하다.
이 파일에는 로봇 이동이나 그리퍼 제어 명령이 없다.
"""

import json
import math
import time
import warnings
from datetime import datetime

import numpy as np
from scipy.spatial.transform import Rotation


PREFIX = "/dsr01/dsr_controller2/"


def relative_tcp(flange, tip):
    """BASE의 플랜지·펜촉 자세를 플랜지 기준 TCP(mm, ZYZ degree)로 바꾼다."""
    flange_rotation = Rotation.from_euler("ZYZ", flange[3:], degrees=True)
    tip_rotation = Rotation.from_euler("ZYZ", tip[3:], degrees=True)
    offset = flange_rotation.inv().apply(np.array(tip[:3]) - np.array(flange[:3]))
    # ZYZ의 0도 자세는 표현이 여러 개다. scipy가 정한 동등한 표현을 사용한다.
    # 여기서의 표현 중복 경고는 실제 로봇 관절의 특이점 판정이 아니다.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected.*", category=UserWarning)
        orientation = (flange_rotation.inv() * tip_rotation).as_euler("ZYZ", degrees=True)
    return [float(value) for value in (*offset, *orientation)]


def estimate_contact_tcp(flange, current, top_z):
    """알려진 상면 Z와 현재 TCP의 XY로 펜촉을 추정한다. 쓰기·통신은 없다."""
    if not all(math.isfinite(float(v)) for v in (*flange, *current, top_z)):
        raise ValueError("TCP·플랜지·상면 높이는 유한한 값이어야 합니다.")
    if len(flange) != 6 or len(current) != 6 or not 0 <= top_z <= 900:
        raise ValueError("6축 자세와 BASE 기준 0~900mm의 상면 높이가 필요합니다.")
    rotation = Rotation.from_euler("ZYZ", current[3:], degrees=True)
    axis = rotation.apply([0.0, 0.0, 1.0])
    angle = math.degrees(math.acos(float(np.clip(-axis[2], -1.0, 1.0))))
    if angle > 2.0:
        raise ValueError(
            f"현재 TCP +Z축이 수직 아래에서 {angle:.2f}도 기울어 있습니다. "
            "펜을 수직 아래로 세우고 상면 중앙에 다시 맞추세요."
        )
    tip = [current[0], current[1], float(top_z), *current[3:]]
    config = relative_tcp(flange, tip)
    if not 0 < math.dist(config[:3], [0, 0, 0]) <= 600:
        raise ValueError("추정 TCP 오프셋이 0~600mm 범위 밖입니다. 상면 높이를 확인하세요.")
    return config, tip


def apply_contact_tcp(session, top_z, log_path):
    """별도 펜촉 TCP를 등록·적용하고 현재 자세를 검증한다. 물리 이동은 없다."""
    from dsr_msgs2.srv import (
        ConfigCreateTcp, GetCurrentTcp, GetCurrentToolFlangePosx,
        GetCurrentPosj, GetRobotMode, SetCurrentTcp, SetRobotMode,
    )

    def call(kind, path, **fields):
        return session.call(kind, PREFIX + path, kind.Request(**fields))

    current = session.pose()
    flange = list(call(GetCurrentToolFlangePosx,
                       "aux_control/get_current_tool_flange_posx", ref=0).pos)
    config, expected = estimate_contact_tcp(flange, current, top_z)
    previous = call(GetCurrentTcp, "tcp/get_current_tcp").info
    mode = call(GetRobotMode, "system/get_robot_mode").robot_mode
    if mode not in (0, 1):
        raise RuntimeError("측정 모드에서는 TCP를 변경하지 않습니다. 수동 모드로 준비하세요.")
    joints = list(call(GetCurrentPosj, "aux_control/get_current_posj").pos)
    stamp = datetime.now().strftime("%m%d%H%M%S%f")
    name = "CakePen_" + stamp
    backup = previous or "CakePrev_" + stamp
    record = dict(
        assumptions="수직 펜, 활성 TCP의 XY와 실제 펜촉 XY가 동일",
        top_z_mm=top_z, previous_tcp=previous, restore_tcp=backup,
        previous_mode=int(mode), flange_pose=flange, previous_pose=current,
        estimated_tcp=config, expected_pose=expected, configured_tcp=name,
        status="calculated",
    )

    def save():
        log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2))

    save()
    print(f"펜촉 추정 위치 BASE [mm, deg]: {expected}", flush=True)
    print(f"등록할 플랜지 기준 TCP [mm, deg]: {config}", flush=True)
    print(f"펜촉 TCP 이름: {name} / 기록: {log_path}", flush=True)
    activation_attempted = False
    try:
        # 설정 등록을 위해 잠시 수동 모드로 전환하고 종료 시 원래 모드를 복원한다.
        if mode != 0:
            call(SetRobotMode, "system/set_robot_mode", robot_mode=0)
        if not previous:
            # 이름 없는 기본 TCP도 실패 시 복원할 수 있게 수치 설정을 보관한다.
            call(ConfigCreateTcp, "tcp/config_create_tcp", name=backup,
                 pos=relative_tcp(flange, current))
        call(ConfigCreateTcp, "tcp/config_create_tcp", name=name, pos=config)
        activation_attempted = True
        call(SetCurrentTcp, "tcp/set_current_tcp", name=name)
        deadline = time.monotonic() + 2.0
        while True:
            actual = session.pose()
            error = math.dist(actual[:3], expected[:3])
            rotation_error = (
                Rotation.from_euler("ZYZ", expected[3:], degrees=True).inv()
                * Rotation.from_euler("ZYZ", actual[3:], degrees=True)
            ).magnitude()
            if error < 0.5 and math.degrees(rotation_error) < 0.5:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f"적용 후 펜촉 좌표가 예상과 다릅니다: {actual}")
            time.sleep(0.05)
        after = list(call(GetCurrentPosj, "aux_control/get_current_posj").pos)
        if max(abs(a - b) for a, b in zip(joints, after)) > 0.1:
            raise RuntimeError("TCP 설정 중 관절이 움직였습니다. 수동 조작을 멈추고 다시 준비하세요.")
        selected = call(GetCurrentTcp, "tcp/get_current_tcp").info
        if selected != name:
            raise RuntimeError(f"펜촉 TCP가 적용되지 않았습니다: {selected}")
        record["status"] = "applied"
        record["actual_pose"] = actual
        save()
        print("펜촉 추정 TCP 적용 완료. 로봇은 이동시키지 않았습니다.", flush=True)
        print(f"종료 후에도 {name} 설정은 유지됩니다. 이전 TCP 복원 이름: {backup}", flush=True)
        print("웹 제작 요청 전에 펜던트에서 수동 점고정을 확인하세요.", flush=True)
        return actual
    except BaseException:
        # 응답이 끊겼더라도 설정이 적용됐을 수 있으므로 기존 TCP로 복원을 시도한다.
        if activation_attempted:
            try:
                call(SetCurrentTcp, "tcp/set_current_tcp", name=backup)
                record["status"] = "failed_restored"
            except Exception:
                record["status"] = "failed_restore_unconfirmed"
                print(f"기존 TCP 복원 확인 실패. 펜던트에서 {backup}을 확인하세요.", flush=True)
        save()
        raise
    finally:
        if mode != 0:
            call(SetRobotMode, "system/set_robot_mode", robot_mode=int(mode))
