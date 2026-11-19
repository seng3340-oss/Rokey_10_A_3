"""실물에서 검증한 45도 파지의 펜 방향과 그리퍼 수직 자세를 계산한다."""

import math

from scipy.spatial.transform import Rotation

# 펜촉 위치는 펜던트 자동 교정값이고, 각도는 성공한 +Y 기울임 파지로 정한다.
SIDE_TCP_NAME = "Pen_Side_45"
SIDE_TCP_OFFSET_MM = (1.095, -46.474, 271.695)
VERTICAL_GRIPPER_ZYZ = (123.66, 180.0, 123.43)


def side_gripper_quaternion(x, y, cx, cy):
    """실제 펜이 원주 중심을 향하게 회전하고 RG2 길이축은 수직으로 유지한다."""
    theta = math.atan2(y - cy, x - cx)
    # 성공한 원주 시험은 +Y에서 시작해 반시계로 한 바퀴였다.
    # 파지 완료 시 펜의 수평 방향은 -Y다. 원주 각도에 맞춰 BASE Z만 회전한다.
    yaw = theta - math.pi / 2
    vertical = Rotation.from_euler("ZYZ", VERTICAL_GRIPPER_ZYZ, degrees=True)
    return tuple((Rotation.from_rotvec([0., 0., yaw]) * vertical).as_quat())


def apply_side_gripper_orientations(waypoints, cx, cy):
    """기존 옆면 위치·획 경계를 보존하고 목표·원호 경유점·스플라인 자세를 교체한다."""
    from geometry_msgs.msg import Quaternion

    def orient(pose):
        """원주 위치별 자세로 교체한다. 상면 중심 이탈점은 기준 수직 자세를 쓴다."""
        if math.hypot(pose.position.x - cx, pose.position.y - cy) < 1e-9:
            quat = Rotation.from_euler("ZYZ", VERTICAL_GRIPPER_ZYZ, degrees=True).as_quat()
        else:
            quat = side_gripper_quaternion(pose.position.x, pose.position.y, cx, cy)
        pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))

    for index, waypoint in enumerate(waypoints):
        # 처음 두 점은 상면 위로 이탈하고 옆으로 이동하는 구간이다.
        # DP가 준 영점 자세를 유지해야 XY 이동과 자세 전환이 동시에 걸리지 않는다.
        # 세 번째 공중점에서 RE의 movejx로 옆면 방향을 맞춘다.
        if index < 2:
            continue
        orient(waypoint.pose)
        if waypoint.motion_type == waypoint.CIRCLE:
            orient(waypoint.via_pose)
        for pose in waypoint.spline_poses:
            orient(pose)
