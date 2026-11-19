"""검증된 관절 로그와 필기 경로를 로컬 RViz에서 재생한다."""

import csv
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from simulate_cake_lettering import JOINTS, PEN_TIP, build_models, origin


TOPIC = "/cake_lettering_preview/markers"
FRAME = "cake_preview_base"


def replay(project, output, playback_speed=1.0):
    """검증된 CSV를 로봇 관절 상태로 게시하고 동일한 BASE에 작업대를 표시한다."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Point, TransformStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from rclpy.time import Time
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray

    # 별도 로컬 DDS 그래프에서 읽기 전용 상태를 재생한다. 장비 드라이버는 실행하지 않는다.
    os.environ["ROS_DOMAIN_ID"] = "96"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_STATIC_PEERS", None)
    os.environ.pop("ROS_DISCOVERY_SERVER", None)
    report = json.loads((output / "lettering_validation.json").read_text())
    if report["moving_collision_hits"]:
        raise ValueError("충돌 검사에 실패한 궤적은 재생하지 않습니다.")
    with (output / "lettering_trajectory.csv").open(newline="") as file:
        rows = list(csv.DictReader(file))
    times = np.array([float(row["time_s"]) for row in rows])
    robot, scene, packages = build_models(project)

    # 시각화 전용 URDF의 루트에는 KDL이 읽지 않는 관성 데이터가 필요 없다.
    base = robot.xml.find(f"link[@name='{robot.root}']")
    inertia = base.find("inertial")
    if inertia is not None:
        base.remove(inertia)

    # 빌드 전에도 RViz가 저장소의 원본 메시를 찾도록 로컬 파일 URI를 사용한다.
    for mesh in robot.xml.findall(".//mesh"):
        package, filename = mesh.get("filename").removeprefix("package://").split("/", 1)
        mesh.set("filename", (packages[package] / filename).as_uri())

    # RG2 관절은 검증 모델과 동일한 0 rad로 둔다. 종속 관절은 URDF mimic이 계산한다.
    joint_names = [
        joint.get("name") for joint in robot.xml.findall("joint")
        if joint.get("type") != "fixed" and joint.find("mimic") is None
    ]
    samples = [
        [float(row[f"{name}_rad"]) if name in JOINTS else 0.0 for name in joint_names]
        for row in rows
    ]

    def marker(namespace, identifier, marker_type, rgba):
        """작업대와 잉크의 마커에 BASE와 일치하는 공통 기준 프레임을 할당한다."""
        msg = Marker()
        msg.header.frame_id = FRAME
        msg.ns = namespace
        msg.id = identifier
        msg.type = marker_type
        msg.pose.orientation.w = 1.0
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = rgba
        return msg

    # 작업대 모델은 workcell_root 기준이며, 이 원점과 로봇 BASE가 일치한다.
    scene_markers = []
    scene_frames = scene.frames({})
    for link in scene.xml.findall("link"):
        for entry in link.findall("visual"):
            geometry = entry.find("geometry")[0]
            marker_type = {
                "box": Marker.CUBE,
                "cylinder": Marker.CYLINDER,
                "mesh": Marker.MESH_RESOURCE,
            }.get(geometry.tag)
            if marker_type is None:
                continue
            color = entry.find("material/color")
            rgba = (0.65, 0.65, 0.66, 1.0) if color is None else tuple(map(float, color.get("rgba").split()))
            msg = marker("workcell", len(scene_markers), marker_type, rgba)
            if geometry.tag == "mesh":
                package, filename = geometry.get("filename").removeprefix("package://").split("/", 1)
                msg.mesh_resource = (packages[package] / filename).as_uri()
                scale = tuple(map(float, geometry.get("scale", "1 1 1").split()))
            elif geometry.tag == "box":
                scale = tuple(map(float, geometry.get("size").split()))
            else:
                diameter = 2 * float(geometry.get("radius"))
                scale = (diameter, diameter, float(geometry.get("length")))
            msg.scale.x, msg.scale.y, msg.scale.z = scale
            pose = scene_frames[link.get("name")] @ origin(entry.find("origin"))
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose[:3, 3]
            x, y, z, w = Rotation.from_matrix(pose[:3, :3]).as_quat()
            msg.pose.orientation.x, msg.pose.orientation.y = x, y
            msg.pose.orientation.z, msg.pose.orientation.w = z, w
            scene_markers.append(msg)

    # 펜 마커만 실제 그리퍼 링크의 TF를 따라가도록 고정한다.
    pen = marker("virtual_pen", 0, Marker.CYLINDER, (0.23, 0.13, 0.36, 1.0))
    pen.header.frame_id = "rg2_base_link"
    pen.frame_locked = True
    pen.scale.x = pen.scale.y = 0.006
    pen.scale.z = PEN_TIP - 0.175
    pen.pose.position.z = (PEN_TIP + 0.175) / 2
    ink = marker("lettering", 0, Marker.LINE_LIST, (0.78, 0.08, 0.30, 1.0))
    ink.scale.x = 0.0012

    # 드라이버 설정을 제거한 URDF로 표준 상태 발행기만 실행한다.
    with TemporaryDirectory(prefix="cake-replay-") as directory:
        parameters = Path(directory) / "robot_state_publisher.yaml"
        parameters.write_text(yaml.safe_dump({
            "/dsr01/robot_state_publisher": {"ros__parameters": {
                "robot_description": ET.tostring(robot.xml, encoding="unicode"),
                "publish_frequency": 30.0,
            }},
        }))
        state_publisher = subprocess.Popen([
            "ros2", "run", "robot_state_publisher", "robot_state_publisher",
            "--ros-args", "-r", "__ns:=/dsr01", "--params-file", str(parameters),
        ], start_new_session=True)
        try:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node("cake_lettering_preview")
            try:
                cached = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
                publisher = node.create_publisher(MarkerArray, TOPIC, cached)
                joint_publisher = node.create_publisher(JointState, "/dsr01/joint_states", 10)
                tf_buffer = Buffer()
                tf_listener = TransformListener(tf_buffer, node)
                static_tf = StaticTransformBroadcaster(node)
                transform = TransformStamped()
                transform.header.stamp = node.get_clock().now().to_msg()
                transform.header.frame_id = robot.root
                transform.child_frame_id = FRAME
                transform.transform.rotation.w = 1.0
                static_tf.sendTransform(transform)

                print(f"RViz Fixed Frame: {robot.root} / MarkerArray: {TOPIC}", flush=True)
                print("RViz와 관절 상태 발행기가 연결되면 재생을 시작합니다.", flush=True)
                start = None
                last = -1
                last_marker_time = 0.0
                while rclpy.ok():
                    if state_publisher.poll() is not None:
                        raise RuntimeError("robot_state_publisher가 종료되었습니다.")
                    now = time.monotonic()
                    # 첫 관절값은 먼저 게시하고 TF를 받은 뒤에만 펜 마커와 재생을 시작한다.
                    if start is None and publisher.get_subscription_count():
                        if tf_buffer.can_transform(FRAME, pen.header.frame_id, Time()):
                            start = now
                            print("가상 관절 재생 시작", flush=True)
                    elapsed = 0.0 if start is None else (now - start) * playback_speed
                    index = int(np.searchsorted(times, elapsed, side="right") - 1)

                    joint_state = JointState()
                    joint_state.header.stamp = node.get_clock().now().to_msg()
                    joint_state.name = joint_names
                    joint_state.position = samples[index]
                    joint_publisher.publish(joint_state)
                    if start is not None and index != last:
                        for i in range(max(1, last + 1), index + 1):
                            if rows[i]["phase"] == "DRAWING":
                                for row in rows[i - 1:i + 1]:
                                    x, y, z = (float(row[f"tcp_{axis}_m"]) for axis in "xyz")
                                    # 0.4 mm 렌더링 오프셋으로 잉크와 케이크 표면을 구분한다.
                                    ink.points.append(Point(x=x, y=y, z=z + 0.0004))
                    # 완료 후 새로 연 RViz는 캐시된 마커보다 TF를 늦게 받을 수 있다.
                    # 최종 장면을 1초마다 다시 보내 해당 창에서도 변환을 적용할 수 있게 한다.
                    if start is not None and (index != last or now - last_marker_time >= 1.0):
                        markers = [*scene_markers, pen]
                        if ink.points:
                            markers.append(ink)
                        publisher.publish(MarkerArray(markers=markers))
                        last_marker_time = now
                        if index != last and index == len(rows) - 1:
                            print(f"재생 완료: {times[-1]:.2f}s, 잉크 선분 {len(ink.points) // 2}개", flush=True)
                        last = index
                    # 완료 후에도 최종 관절 TF와 마커를 RViz에 유지한다.
                    rclpy.spin_once(node, timeout_sec=0.03)
            except (ExternalShutdownException, KeyboardInterrupt):
                pass
            finally:
                tf_listener.unregister()
                node.destroy_node()
                rclpy.try_shutdown()
        finally:
            if state_publisher.poll() is None:
                # ros2 run과 자식 ROS 노드를 함께 정리한다.
                os.killpg(state_publisher.pid, signal.SIGTERM)
                state_publisher.wait()
