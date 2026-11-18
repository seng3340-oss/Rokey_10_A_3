"""통합 DP → ROS 직렬화 → 실행기 계획 → 두산 요청까지 새 옆면 형상을 검증한다."""

import math

import pytest
import rclpy
from geometry_msgs.msg import Point32, Quaternion
from rclpy.serialization import serialize_message, deserialize_message
from cake_decorator_interfaces.msg import Stroke2D, Waypoint3D
from cake_decorator_interfaces.srv import GenerateTrajectory
from cake_decorator.drawing_path_node import DrawingPathNode
from cake_decorator.robot_executor_node import RobotExecutorNode
from cake_decorator.drawing_motion import MotionType, plan_drawing_motion
from test_surface_command_contract import RequestRecorder


@pytest.fixture
def node():
    """로봇 서비스 없이 실제 DP 노드만 열고 테스트 후 ROS 자원을 해제한다."""
    rclpy.init()
    value = DrawingPathNode()
    yield value
    value.destroy_node()
    rclpy.try_shutdown()


def convert(node, strokes, surface="SIDE", yaw=0.0):
    """확정 원뿔대와 실기 영점 자세를 갖는 요청을 실제 ROS 형식으로 왕복한다."""
    request = GenerateTrajectory.Request()
    request.canvas_width, request.canvas_height = 1024.0, 768.0
    request.cake_radius, request.cake_bottom_radius = 0.05675, 0.06
    request.cake_height = 0.079
    request.cake_yaw = yaw
    request.cake_pose.position.x, request.cake_pose.position.z = 0.4, 0.15
    request.cake_pose.orientation = Quaternion(y=1.0, w=0.0)
    request.operation_speed_percent = 100
    request.surface_type = surface
    request.strokes_2d = [
        Stroke2D(stroke_id=i, points=[Point32(x=float(x), y=float(y)) for x, y in points])
        for i, points in enumerate(strokes)
    ]
    request = deserialize_message(serialize_message(request), GenerateTrajectory.Request)
    response = node._handle_request(request, GenerateTrajectory.Response())
    assert response.success, response.error_msg
    response = deserialize_message(serialize_message(response), GenerateTrajectory.Response)
    raw, states = RobotExecutorNode._convert_waypoints(response.waypoints_3d)
    plan = plan_drawing_motion(raw, states, surface)
    assert plan.success, plan.error_msg
    return response, plan


def test_midline_remains_circle_through_actual_robot_request(node):
    """반 바퀴 수평선이 중간 반지름58.375mm의 원호 두 개로 실행되어야 한다."""
    response, plan = convert(node, [[(64, 384), (512, 384)]])
    circles = [c for c in plan.commands if c.motion_type == MotionType.MOVE_CIRCLE]
    assert len(circles) == 2
    robot = RequestRecorder()
    for command in plan.commands:
        assert robot.execute_drawing_command(command)
    requests = [req for key, req in robot.requests if key == "move_circle"]
    assert len(requests) == 2
    for request in requests:
        for p in request.pos:
            assert math.hypot(p.data[0] - 400, p.data[1]) == pytest.approx(58.375)
            assert p.data[2] == pytest.approx(110.5)
    # 최초 들어올림과 바깥 이동은 실기 영점 자세를 유지하고 바깥에서만 전환한다.
    for waypoint in response.waypoints_3d[:2]:
        assert waypoint.pose.orientation.y == pytest.approx(1.0)
    assert response.waypoints_3d[2].pose.orientation != response.waypoints_3d[1].pose.orientation


def test_diagonal_letter_keeps_surface_and_pen_angle(node):
    """글자 대각선의 스플라인 목표점과 펜 각도가 새 옆면에서도 유지되어야 한다."""
    response, plan = convert(node, [[(288, 320), (512, 440), (736, 340)]])
    assert any(c.motion_type == MotionType.MOVE_SPLINE for c in plan.commands)
    poses = [p for wp in response.waypoints_3d for p in wp.spline_poses]
    assert poses
    for pose in poses:
        p, q = pose.position, pose.orientation
        h_mm = (p.z - 0.071) * 1000
        assert math.hypot(p.x - 0.4, p.y) * 1000 == pytest.approx(60 - 3.25 * h_mm / 79)
        axis_z = 1 - 2 * (q.x*q.x + q.y*q.y)
        assert math.degrees(math.acos(-axis_z)) == pytest.approx(30.0)


def test_two_letters_travel_outside_frustum(node):
    """글자 사이 이동도 원호를 유지하고 접근 거리10mm만큼 표면 밖이어야 한다."""
    _, plan = convert(node, [[(64,384),(288,384)],[(736,384),(960,384)]])
    assert plan.total_strokes == 2
    clear = [c for c in plan.commands if c.motion_type == MotionType.MOVE_CIRCLE and not c.pen_down]
    assert clear
    for command in clear:
        for p in (command.pos, command.via):
            # 원뿔대 법선 거리: (radial-r(z))/sqrt(1+slope²).
            radius = math.hypot(p.x - 400, p.y)
            radius_at_z = 60 - 3.25 * (p.z - 71) / 79
            assert (radius - radius_at_z) / math.hypot(1, 3.25 / 79) == pytest.approx(10)


def test_top_taught_orientation_is_separate_from_cake_yaw(node):
    """케이크를90도 회전해도 영점 펜 자세는 그대로이고 글자 위치만 회전한다."""
    response, _ = convert(node, [[(512,384),(832,384)]], surface="TOP", yaw=math.pi/2)
    contact = [wp for wp in response.waypoints_3d if wp.pen_down]
    assert contact[-1].pose.position.x == pytest.approx(0.4)
    assert contact[-1].pose.position.y == pytest.approx(0.05675)
    assert all(wp.pose.orientation.y == pytest.approx(1.0) for wp in response.waypoints_3d)


def test_rotated_side_approach_stays_outside_sloped_cake(node):
    """케이크 yaw를 적용한 첫 접근 선분도 상면 위 또는 표면10mm 밖에 있어야 한다."""
    response, _ = convert(node, [[(64,384),(288,384)]], yaw=math.pi/2)
    waypoints = response.waypoints_3d
    contact = next(wp.pose.position for wp in waypoints if wp.pen_down)
    assert contact.x == pytest.approx(0.4)
    assert contact.y == pytest.approx(0.058375)
    # 처음 네 점은 들어올림·바깥 이동·바깥 자세 전환·접근이다.
    positions = [wp.pose.position for wp in waypoints[:4]]
    for start, end in zip(positions, positions[1:]):
        for i in range(101):
            t = i/100
            x = ((1-t)*start.x + t*end.x - 0.4) * 1000
            y = ((1-t)*start.y + t*end.y) * 1000
            z = ((1-t)*start.z + t*end.z) * 1000
            if z <= 150:
                radius_at_z = 60 - 3.25 * (z - 71) / 79
                distance = (math.hypot(x,y) - radius_at_z) / math.hypot(1,3.25/79)
                assert distance >= 10 - 1e-8


@pytest.mark.parametrize("points", [
    [(64, 384), (512, 384)],
    [(512, 320), (512, 440)],
    [(288, 320), (512, 440), (736, 340)],
])
def test_real_path_calls_new_pen_direction_and_points_inward_down(node, monkeypatch, points):
    """실제 DP가 새 함수를 거치며 펜축이 안쪽·아래로 30도를 유지하는지 확인한다."""
    from cake_decorator.drawing_geometry import FrustumCake

    calls = []
    original = FrustumCake.side_pen_direction

    def record(self, arc, height, *args, **kwargs):
        calls.append((arc, height))
        return original(self, arc, height, *args, **kwargs)

    monkeypatch.setattr(FrustumCake, "side_pen_direction", record)
    response, _ = convert(node, [points], yaw=0.4)
    assert calls, "실제 드로잉이 side_pen_direction을 거치지 않았습니다."

    # 최초 두 공중 이동은 영점 자세다. 그 이후 모든 목표·원호·스플라인을 확인한다.
    poses = []
    for waypoint in response.waypoints_3d[2:]:
        poses.append(waypoint.pose)
        poses.extend(waypoint.spline_poses)
        if waypoint.motion_type == Waypoint3D.CIRCLE:
            poses.append(waypoint.via_pose)
    for pose in poses:
        p, q = pose.position, pose.orientation
        axis = (2 * (q.x * q.z + q.w * q.y),
                2 * (q.y * q.z - q.w * q.x),
                1 - 2 * (q.x * q.x + q.y * q.y))
        radius = math.hypot(p.x - 0.4, p.y)
        assert ((p.x - 0.4) * axis[0] + p.y * axis[1]) / radius == pytest.approx(-0.5)
        assert axis[2] == pytest.approx(-math.cos(math.radians(30)))
