"""45도 파지 옆면 검증 후 손목을 되감고 티칭된 거치대에 펜을 반납한다."""

import math
import time
import warnings

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.exceptions import ParameterNotDeclaredException

from .bent_pen import SIDE_TCP_OFFSET_MM, VERTICAL_GRIPPER_ZYZ
from .drawing_motion import DsrPos


def return_pen(
    robot,
    initial_joints,
    cancelled=lambda: False,
    *,
    rack=None,
    open_gripper=None,
    reset_tcp=None,
    parameter=None,
):
    """이탈 → J6 되감기 → 기울여 반납한다.

    독립 시험의 기본 티칭/서비스는 유지한다. 통합 실행기는 현재 YAML과
    DO 열기, 해제 후 TCP 초기화 콜백을 전달한다. 초기화 뒤의 인출 목표는
    Pen_Side_45 펜촉 좌표가 아니라 같은 물리 위치의 플랜지 좌표를 쓴다.
    """
    from dsr_msgs2.srv import GetCurrentPosj, GetCurrentPosx
    from onrobot_rg_msgs.srv import SetCommand

    # 통합 실행은 현재 이동 설정을 받으며, 독립 시험도 같은 속도 기준을 쓴다.
    # 기울임·수직 복귀만 완만하게 하고 공중 이동·하강·인출의 선속도는 유지한다.
    defaults = {
        "free_joint_speed_deg_s": 60.0,
        "free_joint_acc_deg_s2": 120.0,
        "pen_travel_speed_mm_s": 180.0,
        "pen_lift_speed_mm_s": 150.0,
        "pen_rack_speed_mm_s": 120.0,
        "pen_motion_acc_mm_s2": 400.0,
        "pen_rotation_speed_deg_s": 90.0,
        "pen_rotation_acc_deg_s2": 180.0,
        "pen_tilt_speed_deg_s": 31.5,
        "pen_tilt_acc_deg_s2": 90.0,
    }
    resolved = {}

    def setting(name):
        """새 기울임 설정만 없는 이전 노드는 검증된 기본값으로 반납한다.

        나머지 필수 설정의 누락은 그대로 오류로 처리한다. 이미 읽은 값은 같은
        반납 동안 고정하고, 구버전 호환 안내도 설정마다 한 번만 남긴다.
        """
        if name not in resolved:
            if parameter is None:
                value = defaults[name]
            else:
                try:
                    value = parameter(name)
                except ParameterNotDeclaredException:
                    if name not in ("pen_tilt_speed_deg_s", "pen_tilt_acc_deg_s2"):
                        raise
                    value = defaults[name]
                    robot._logger.warning(
                        f"이전 노드에 {name} 설정이 없어 기본값 {value:g} 적용. "
                        "다음 주문 전에 실행기를 재시작하세요."
                    )
            resolved[name] = value
        return resolved[name]

    def require(success, stage):
        """실패·취소 시 이후 이동과 그리퍼 열기를 수행하지 않는다."""
        if not success:
            raise RuntimeError(f"펜 반납 실패 {stage}")

    def check_cancel():
        if cancelled():
            raise RuntimeError("펜 반납 취소 요청")

    def joints():
        result = robot._call("current_joints", GetCurrentPosj.Request())
        require(result is not None and result.success, "현재 관절 조회")
        return list(result.pos)

    def movej(target, label, *, tilt=False):
        """절대 관절로 되감되 거치대 수직 복귀에만 기울임 속도를 적용한다."""
        check_cancel()
        rotation = "pen_tilt" if tilt else "free_joint"
        require(robot.move_joint(
            target,
            speed_deg_s=setting(f"{rotation}_speed_deg_s"),
            acc_deg_s2=setting(f"{rotation}_acc_deg_s2"),
        ), label)
        actual = joints()
        require(max(abs(a-b) for a,b in zip(target,actual)) < .5, label+" 도달")

    def line(pose, label, speed_name="pen_travel_speed_mm_s"):
        """반납 회전은 기울임 제한을 쓰고 하강·인출·공중 선속도는 유지한다."""
        check_cancel()
        require(robot.move_line(
            DsrPos(*pose),
            speed_mm_s=setting(speed_name),
            acc_mm_s2=setting("pen_motion_acc_mm_s2"),
            angular_speed_deg_s=setting("pen_tilt_speed_deg_s"),
            angular_acc_deg_s2=setting("pen_tilt_acc_deg_s2"),
        ), label)
        require(robot.verify_pose(pose),label+" 도달")

    current = robot._call("current_pose", GetCurrentPosx.Request(ref=0))
    require(current is not None and current.success and bool(current.task_pos_info),"현재 펜촉 조회")
    raised = list(current.task_pos_info[0].data[:6])
    raised[2] += 100.
    line(raised,"케이크 밖 상공으로 추가 이탈", "pen_lift_speed_mm_s")

    # DRL에서 사용한 높은 경유 관절 자세. 먼저 현재 회전수를 유지한 채 이동하고
    # 그 자세에서 J6만 집기 완료 시점의 값으로 되감는다.
    staging = [35.,40.,35.,-20.,120.,joints()[5]]
    movej(staging,"되감기 상공 이동")
    staging[5] = float(initial_joints[5])
    movej(staging,"J6 집기 완료 각도로 되감기")
    drop_approach = (
        list(rack['pen_drop_approach_pose']) if rack is not None
        else [43.58, .53, 69.85, .09, 109.63, 43.31]
    )
    movej(drop_approach, "펜 거치대 상공", tilt=True)

    # 펜은 RG2에 +Y 기울임 상태로 물려 있다. 원래 파지와 같은 45도 기울임을
    # 재현하면 펜 자체가 수직으로 돌아온다. 반납 중심은 DRL의 티칭값을 쓴다.
    original_angles = (
        rack['pen_stand_pose'][3:] if rack is not None
        else VERTICAL_GRIPPER_ZYZ
    )
    drop_pose = (
        list(rack['pen_drop_pose']) if rack is not None
        else [248.86, 244.77, 314.84, 40.25, 180., 40.02]
    )
    after_pose = (
        list(rack['pen_drop_after_pose']) if rack is not None
        else [248.91, 244.76, 548.06, 37.07, -179.99, 36.84]
    )
    clearance = after_pose[2] - drop_pose[2]
    require(clearance > 0., "반납 후 인출 높이 설정")
    original = Rotation.from_euler("ZYZ", original_angles, degrees=True)
    axis = np.array([0.,math.sqrt(.5),-math.sqrt(.5)])
    turn_axis = np.cross(original.apply([0.,0.,1.]),axis)
    turn_axis /= np.linalg.norm(turn_axis)
    # 파지 때와 같은 목표축까지의 정확한 회전각을 써 티칭 미세 오차도 보존한다.
    angle = math.atan2(
        float(np.linalg.norm(np.cross(original.apply([0., 0., 1.]), axis))),
        float(original.apply([0., 0., 1.]) @ axis),
    )
    tilted = Rotation.from_rotvec(turn_axis * angle) * original
    drop_rotation = Rotation.from_euler("ZYZ", drop_pose[3:], degrees=True)
    center = np.array(drop_pose[:3]) + drop_rotation.apply([0., 0., 220.])
    flange_position = center - tilted.apply([0.,0.,220.])
    # 활성 Pen_Side_45 TCP는 플랜지 대비 회전값이 0이다.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore",message="Gimbal lock detected.*")
        angles = tilted.as_euler("ZYZ",degrees=True)
    drop = [*(flange_position+tilted.apply(SIDE_TCP_OFFSET_MM)),*angles]
    above = list(drop)
    above[2] += clearance
    line(above,"펜을 수직으로 맞추는 기울임 자세 상공")
    line(list(drop),"거치대 반납 위치로 하강", "pen_rack_speed_mm_s")
    check_cancel()
    if open_gripper is None:
        response = robot._call("gripper", SetCommand.Request(command="o"))
        require(response is not None and response.success, "RG2 열기")
    else:
        require(open_gripper(), "그리퍼 DO 열기")
    time.sleep(1.)
    if reset_tcp is not None:
        # 해제 직후 TCP를 영점으로 돌린다. 반납 인출도 그 기준으로 변환한다.
        reset_tcp()
        above = [*flange_position, *angles]
        above[2] += clearance
    line(above,"반납 후 수직 인출", "pen_lift_speed_mm_s")
    robot._logger.info("J6 되감기와 펜 반납 완료")
