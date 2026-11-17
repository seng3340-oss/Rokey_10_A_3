#!/usr/bin/env python3
"""가상 RG2 펜 모델과 에뮬레이터 TCP를 같은 치수로 설정한다. 실물에는 적용하지 않는다."""

import math
import xml.etree.ElementTree as ET

# RG2 배포 URDF 관절 원점과 inner_finger STL의 끝점에서 구한 값(m).
# 실제 그리퍼를 측정한 치수가 아닌 가상 모델의 형상 기준이다.
FINGER_PIVOT_Z = 0.125797
FINGER_OFFSET_Y = 0.039592
FINGER_OFFSET_Z = 0.038177
FINGER_MESH_TIP_Z = 0.052798572927713394
FINGER_INNER_EDGE_Y = 0.018949951976537704
PEN_RADIUS = 0.004
TCP_NAME = "Virtual_Pen_TCP"


def parse_extension_cm(value):
    """터미널에서 입력한 돌출 길이를 cm에서 m로 바꾸고 유효성을 확인한다."""
    cm = float(value)
    if not math.isfinite(cm) or not 0 < cm <= 20:
        raise ValueError("돌출 길이는 0보다 크고 20cm 이하인 숫자로 입력하세요.")
    return cm / 100.0


def grip_geometry():
    """지름 8mm 펜 양옆에 RG2 손가락 안쪽 면이 닿는 관절각과 끝 높이를 구한다."""
    target = -PEN_RADIUS - (FINGER_INNER_EDGE_Y - 0.017178)
    angle = math.atan2(FINGER_OFFSET_Y, FINGER_OFFSET_Z) + math.asin(
        target / math.hypot(FINGER_OFFSET_Y, FINGER_OFFSET_Z)
    )
    tip = (
        FINGER_PIVOT_Z
        + FINGER_OFFSET_Y * math.sin(angle)
        + FINGER_OFFSET_Z * math.cos(angle)
        + FINGER_MESH_TIP_Z
    )
    return angle, tip


def hardware_params(description):
    """현재 description이 localhost 가상 드라이버인지 검사한다. 쓰기 작업 전 반드시 호출한다."""
    root = ET.fromstring(description)
    params = {
        item.attrib["name"]: item.text
        for item in root.findall(".//ros2_control/hardware/param")
    }
    if params.get("mode") != "virtual" or params.get("host") != "127.0.0.1":
        raise RuntimeError("가상 펜 설정은 virtual / 127.0.0.1에서만 허용됩니다.")
    return root


def align_virtual_kinematics(root):
    """교육장 URDF의 반올림 치수를 로컬 M0609 에뮬레이터 FK와 맞춘다.

    동일 관절각 3개에 대한 ref=BASE FK 비교로 확인한 가상 표시 교정이다.
    원본 벤더 파일·실물 제어 좌표·펜 길이는 바꾸지 않는다. 이미 다른 치수로
    교정된 모델에는 해당 항목을 덮어쓰지 않는다.
    """
    for name, index, original, corrected in (
        ("joint_1", 2, 0.1345, 0.135),
        ("joint_2", 1, 0.0062, 0.00625),
    ):
        origin = root.find(f"joint[@name='{name}']/origin")
        if origin is None:
            continue
        xyz = [float(value) for value in origin.get("xyz", "0 0 0").split()]
        if math.isclose(xyz[index], original, abs_tol=1e-10):
            xyz[index] = corrected
            origin.set("xyz", " ".join(str(value) for value in xyz))
    for i in range(1, 7):
        origin = root.find(f"joint[@name='joint_{i}']/origin")
        if origin is None:
            continue
        rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
        rpy = [
            (
                math.copysign(math.pi / 2, value)
                if math.isclose(abs(value), 1.571, abs_tol=1e-10)
                else value
            )
            for value in rpy
        ]
        origin.set("rpy", " ".join(str(value) for value in rpy))


def add_pen(description, extension_m):
    """강의 URDF를 복사한 문자열에만 펜을 덧붙인다. 외부 벤더 파일은 수정하지 않는다."""
    root = hardware_params(description)
    align_virtual_kinematics(root)
    if not math.isfinite(extension_m) or not 0 < extension_m <= 0.2:
        raise ValueError("펜 돌출 길이 범위 오류")
    _, gripper_tip = grip_geometry()
    # RG2 장착 변환을 합성하면 그리퍼 길이 방향은 플랜지 +Z와 일치한다.
    # 제어 TCP의 회전도 플랜지 기준 0으로 두어 DP의 tool Z 방향과 일치시킨다.
    ET.SubElement(root, "link", name="virtual_gripper_tip")
    joint = ET.SubElement(root, "joint", name="virtual_gripper_tip_joint", type="fixed")
    ET.SubElement(joint, "parent", link="link_6")
    ET.SubElement(joint, "child", link="virtual_gripper_tip")
    ET.SubElement(joint, "origin", xyz=f"0 0 {gripper_tip}", rpy="0 0 0")
    pen = ET.SubElement(root, "link", name="virtual_pen_tip")
    joint = ET.SubElement(root, "joint", name="virtual_pen_tip_joint", type="fixed")
    ET.SubElement(joint, "parent", link="virtual_gripper_tip")
    ET.SubElement(joint, "child", link="virtual_pen_tip")
    ET.SubElement(joint, "origin", xyz=f"0 0 {extension_m}", rpy="0 0 0")
    # 손가락 안쪽에 24mm 들어간 몸통과 마지막 4mm의 가는 펜촉을 표시한다.
    body_length = extension_m + 0.024 - 0.004
    for length, radius, z, color in (
        (body_length, PEN_RADIUS, -0.004 - body_length / 2, "0.82 0.16 0.12 1"),
        (0.004, 0.0008, -0.002, "0.12 0.12 0.12 1"),
    ):
        visual = ET.SubElement(pen, "visual")
        ET.SubElement(visual, "origin", xyz=f"0 0 {z}")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "cylinder", radius=str(radius), length=str(length))
        material = ET.SubElement(visual, "material", name="")
        ET.SubElement(material, "color", rgba=color)
    return ET.tostring(root, encoding="unicode")


def tcp_from_description(description):
    """표시 모델의 두 고정 변환을 합쳐 플랜지→펜촉 TCP(mm, ZYZ deg)를 구한다."""
    root = hardware_params(description)
    distance = 0.0
    for name in ("virtual_gripper_tip_joint", "virtual_pen_tip_joint"):
        joint = root.find(f"joint[@name='{name}']")
        if joint is None:
            raise RuntimeError(
                "가상 펜 모델이 없습니다. 펜 전용 virtual launch를 사용하세요."
            )
        distance += float(joint.find("origin").attrib["xyz"].split()[2])
    return [0.0, 0.0, distance * 1000, 0.0, 0.0, 0.0]


def main():
    """가상 드라이버 준비 후 TCP를 등록·선택하고 펜 폭에 맞춰 가상 손가락을 닫는다."""
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    from dsr_msgs2.srv import (
        ConfigCreateTcp,
        SetCurrentTcp,
        GetCurrentTcp,
        GetRobotMode,
        SetRobotMode,
    )
    from onrobot_rg_msgs.srv import SetCommand

    rclpy.init()
    node = Node("virtual_pen_setup")

    def call(service_type, path, request):
        """초기화 전용 단일 노드에서 서비스를 기다린다. 이동 명령은 보내지 않는다."""
        client = node.create_client(service_type, path)
        try:
            if not client.wait_for_service(timeout_sec=60):
                raise RuntimeError(f"가상 펜 초기화 서비스 대기 시간 초과: {path}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15)
            if not future.done() or future.result() is None:
                raise RuntimeError(f"가상 펜 초기화 응답 없음: {path}")
            response = future.result()
            if hasattr(response, "success") and not response.success:
                raise RuntimeError(f"가상 펜 초기화 실패: {path}")
            return response
        finally:
            node.destroy_client(client)

    try:
        response = call(
            GetParameters,
            "/dsr01/robot_state_publisher/get_parameters",
            GetParameters.Request(names=["robot_description"]),
        )
        # URDF에서 모드를 확인한 뒤에만 컨트롤러 설정을 변경한다.
        pose = tcp_from_description(response.values[0].string_value)
        prefix = "/dsr01/dsr_controller2/tcp/"
        # 제조사 API는 TCP 등록을 수동 모드에서만 허용한다.
        # 가상 대상 확인이 끝난 뒤 잠시 수동으로 바꾸고 원래 모드로 복원한다.
        system = "/dsr01/dsr_controller2/system/"
        previous_mode = call(
            GetRobotMode, system + "get_robot_mode", GetRobotMode.Request()
        ).robot_mode
        try:
            call(
                SetRobotMode,
                system + "set_robot_mode",
                SetRobotMode.Request(robot_mode=0),
            )
            call(
                ConfigCreateTcp,
                prefix + "config_create_tcp",
                ConfigCreateTcp.Request(name=TCP_NAME, pos=pose),
            )
            call(
                SetCurrentTcp,
                prefix + "set_current_tcp",
                SetCurrentTcp.Request(name=TCP_NAME),
            )
        finally:
            call(
                SetRobotMode,
                system + "set_robot_mode",
                SetRobotMode.Request(robot_mode=previous_mode),
            )
        active = call(
            GetCurrentTcp, prefix + "get_current_tcp", GetCurrentTcp.Request()
        )
        if active.info != TCP_NAME:
            raise RuntimeError("가상 펜 TCP 선택 결과가 일치하지 않습니다.")
        angle, _ = grip_geometry()
        call(SetCommand, "/onrobot/sendCommand", SetCommand.Request(command=str(angle)))
        node.get_logger().info(
            f"가상 펜 준비 완료: {TCP_NAME}, 플랜지→펜촉 {pose[2]:.2f}mm. "
            "실물 TCP 설정은 변경하지 않았습니다."
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
