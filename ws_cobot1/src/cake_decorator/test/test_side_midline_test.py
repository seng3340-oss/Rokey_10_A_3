"""별도 버튼의 중간 단면·접근 경유점·원호 명령을 장비 없이 검증한다."""

import math
from types import SimpleNamespace
import rclpy
from geometry_msgs.msg import Point32, Quaternion
from cake_decorator_interfaces.msg import Stroke2D, Waypoint3D
from cake_decorator_interfaces.srv import GenerateTrajectory
from cake_decorator.drawing_path_node import DrawingPathNode
from cake_decorator.goal_mapping import build_goal
from cake_decorator.robot_executor_node import RobotExecutorNode
from cake_decorator.drawing_motion import plan_drawing_motion, MotionType


def claim():
    half = 30 / (math.pi * 116.75) * 896 / 2
    return {
        "order_id": "ORD-MIDLINE-TEST",
        "cake": {
            "diameter_mm": 116.75,
            "height_mm": 79.0,
            "board_width_mm": 179.0,
            "board_depth_mm": 179.0,
            "board_thickness_mm": 2.0,
            "reference": "tcp_at_top_center",
            "tool_type": "pen",
            "test_recipe": {
                "kind": "side_midline_frustum",
                "top_diameter_mm": 113.5,
                "bottom_diameter_mm": 120.0,
                "height_mm": 79.0,
                "line_length_mm": 30.0,
            },
        },
        "drawing": {
            "canvas_width": 1024,
            "canvas_height": 768,
            "drawing_surface": "side",
            "strokes": [
                {
                    "stroke_id": 0,
                    "points": [
                        {"x": 512 - half, "y": 384.0},
                        {"x": 512 + half, "y": 384.0},
                    ],
                }
            ],
        },
    }


def test_bridge_marks_only_dedicated_recipe_as_test():
    payload = claim()
    goal = build_goal(payload, [500.0, 0.0, 142.0, 0.0, 180.0, 0.0])
    assert goal.drawing_paths[0].surface == "SIDE_MIDLINE_TEST"
    assert math.isclose(goal.cake_params.radius, 0.058375)
    del payload["cake"]["test_recipe"]
    assert (
        build_goal(payload, [500.0, 0.0, 142.0, 0.0, 180.0, 0.0])
        .drawing_paths[0]
        .surface
        == "SIDE"
    )


def test_midline_is_exact_frustum_cross_section_and_has_outside_rotation():
    goal = build_goal(claim(), [500.0, 0.0, 142.0, 0.0, 180.0, 0.0])
    rclpy.init()
    node = DrawingPathNode()
    try:
        request = GenerateTrajectory.Request()
        request.surface_type = "SIDE_MIDLINE_TEST"
        request.canvas_width, request.canvas_height = 1024.0, 768.0
        request.operation_speed_percent = 100
        request.cake_pose.position = goal.cake_params.center
        request.cake_pose.orientation = goal.cake_params.tool_orientation
        request.cake_radius, request.cake_height = (
            goal.cake_params.radius,
            goal.cake_params.height,
        )
        request.strokes_2d = goal.drawing_paths[0].strokes
        response = node._handle_request(request, GenerateTrajectory.Response())
        assert response.success, response.error_msg
        path = response.waypoints_3d
        assert len(path) == 8
        assert (
            path[0].pose.orientation
            == path[1].pose.orientation
            == goal.cake_params.tool_orientation
        )
        assert path[2].pose.position == path[1].pose.position
        assert path[2].pose.orientation != path[1].pose.orientation
        assert (
            math.hypot(path[2].pose.position.x - 0.5, path[2].pose.position.y) > 0.060
        )
        assert path[2].pose.position.z > 0.142
        contacts = [wp for wp in path if wp.pen_down]
        assert len(contacts) == 2
        for pose in [wp.pose for wp in contacts] + [contacts[-1].via_pose]:
            assert abs(pose.position.z - (0.142 - 0.079 / 2)) < 1e-8
            assert (
                abs(math.hypot(pose.position.x - 0.5, pose.position.y) - 0.058375)
                < 1e-8
            )
        assert contacts[-1].motion_type == Waypoint3D.CIRCLE
        raw, states = RobotExecutorNode._convert_waypoints(path)
        plan = plan_drawing_motion(raw, states, "SIDE_MIDLINE_TEST")
        assert plan.success and plan.total_strokes == 1
        assert [cmd.motion_type for cmd in plan.commands].count(
            MotionType.MOVE_CIRCLE
        ) == 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_midline_dispatch_changes_pose_only_at_outside_waypoint():
    calls = []
    owner = SimpleNamespace(
        _robot=SimpleNamespace(
            move_task_joint=lambda pos: calls.append(("joint", pos)) or True,
            execute_drawing_command=lambda cmd: calls.append(("normal", cmd.pos))
            or True,
        )
    )
    command = SimpleNamespace(pos=object())
    for index in range(8):
        assert RobotExecutorNode._execute_midline_command(owner, command, index)
    assert [kind for kind, _ in calls] == [
        "normal",
        "normal",
        "joint",
        "normal",
        "normal",
        "normal",
        "normal",
        "normal",
    ]


def test_test_surface_rejects_diagonal_or_multiple_strokes():
    import copy

    payload = claim()
    goal = build_goal(payload, [500.0, 0.0, 142.0, 0.0, 180.0, 0.0])
    rclpy.init()
    node = DrawingPathNode()
    try:
        request = GenerateTrajectory.Request()
        request.surface_type = "SIDE_MIDLINE_TEST"
        request.canvas_width, request.canvas_height = 1024.0, 768.0
        request.cake_radius, request.cake_height = 0.058375, 0.079
        request.operation_speed_percent = 100
        request.cake_pose.position = goal.cake_params.center
        request.cake_pose.orientation = goal.cake_params.tool_orientation
        request.strokes_2d = goal.drawing_paths[0].strokes
        request.strokes_2d[0].points[1].y = 400.0
        response = node._handle_request(request, GenerateTrajectory.Response())
        assert not response.success
        request.strokes_2d = [copy.deepcopy(goal.drawing_paths[0].strokes[0])] * 2
        assert not node._handle_request(request, GenerateTrajectory.Response()).success
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_tm_to_executor_keeps_test_branch_and_stops_before_contact_on_failure():
    """실제 TM·DP·RE 함수를 연결하되 로봇 어댑터만 대체해 주문 한 획을 검증한다."""
    from cake_decorator.task_manager_node import TaskManagerNode

    rclpy.init()
    node = DrawingPathNode()
    logger = SimpleNamespace(info=lambda _: None, error=lambda _: None)
    goal = build_goal(claim(), [500.0, 0.0, 142.0, 0.0, 180.0, 0.0])
    try:
        for joint_ok in (True, False):
            commands, results = [], []
            executor = SimpleNamespace(
                get_logger=lambda: logger,
                _convert_waypoints=RobotExecutorNode._convert_waypoints,
                _robot=SimpleNamespace(
                    execute_drawing_command=lambda cmd: commands.append(("normal", cmd))
                    or True,
                    move_task_joint=lambda pos: commands.append(("joint", pos))
                    or joint_ok,
                ),
            )
            executor._execute_midline_command = (
                lambda cmd, i: RobotExecutorNode._execute_midline_command(
                    executor, cmd, i
                )
            )

            def send_draw(draw, _):
                assert draw.drawing_surface == "SIDE_MIDLINE_TEST"
                handle = SimpleNamespace(
                    request=draw,
                    is_cancel_requested=False,
                    publish_feedback=lambda _: None,
                    succeed=lambda: results.append("completed"),
                    abort=lambda: results.append("failed"),
                )
                response = RobotExecutorNode._execute_draw(executor, handle)
                return response.success

            def convert(request):
                response = node._handle_request(request, GenerateTrajectory.Response())
                return SimpleNamespace(result=lambda: response)

            parameters = {
                "drawing_speed": 30.0,
                "contact_force": 0.5,
                "operation_speed_percent": 100,
                "service_timeout": 5.0,
            }
            manager = SimpleNamespace(
                _reference_pose=TaskManagerNode._reference_pose,
                get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
                _check_cancel=lambda _: False,
                get_logger=lambda: logger,
                _gen_traj_client=SimpleNamespace(
                    wait_for_service=lambda **_: True, call_async=convert
                ),
                _wait_future=lambda *_: None,
                _send_draw=send_draw,
                _publish_feedback=lambda *_: None,
            )
            assert TaskManagerNode._execute_drawing(manager, goal, object()) == joint_ok
            assert results == (["completed"] if joint_ok else ["failed"])
            assert [kind for kind, _ in commands[:3]] == ["normal", "normal", "joint"]
            if joint_ok:
                assert len(commands) == 8
                assert commands[5][1].motion_type == MotionType.MOVE_CIRCLE
            else:
                assert len(commands) == 3
                assert not any(
                    cmd.pen_down for kind, cmd in commands if kind == "normal"
                )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_length_limits_survive_point32_ros_serialization():
    """웹에서 허용한 5·50mm도 실제 float32 전송 뒤 DP에서 허용되어야 한다."""
    from rclpy.serialization import serialize_message, deserialize_message

    rclpy.init()
    node = DrawingPathNode()
    try:
        for length in (5.0, 50.0):
            payload = claim()
            half = length / (math.pi * 116.75) * 896 / 2
            payload["cake"]["test_recipe"]["line_length_mm"] = length
            payload["drawing"]["strokes"][0]["points"] = [
                {"x": 512 - half, "y": 384.0},
                {"x": 512 + half, "y": 384.0},
            ]
            goal = build_goal(payload, [500.0, 0.0, 142.0, 0.0, 180.0, 0.0])
            request = GenerateTrajectory.Request()
            request.surface_type = "SIDE_MIDLINE_TEST"
            request.canvas_width, request.canvas_height = 1024.0, 768.0
            request.cake_radius, request.cake_height = 0.058375, 0.079
            request.cake_pose.position = goal.cake_params.center
            request.cake_pose.orientation = goal.cake_params.tool_orientation
            request.operation_speed_percent = 100
            request.strokes_2d = goal.drawing_paths[0].strokes
            request = deserialize_message(
                serialize_message(request), GenerateTrajectory.Request
            )
            response = node._handle_request(request, GenerateTrajectory.Response())
            assert response.success, response.error_msg
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_outside_pose_change_uses_limited_joint_speed_and_arrival_check():
    """자세 전환도 올바른 서비스 요청과 도달 확인을 거쳐야 한다."""
    from cake_decorator.doosan_robot import DoosanRobotClient
    from cake_decorator.drawing_motion import DsrPos

    class JointRecorder(DoosanRobotClient):
        def __init__(self, arrived):
            self.arrived, self.requests, self.targets = arrived, [], []
            self._logger = SimpleNamespace(info=lambda _: None)

        def _call(self, key, request, timeout=180.0):
            if key == "current_joints":
                return SimpleNamespace(
                    success=True, pos=[0.0, 20.0, 70.0, 0.0, 90.0, 0.0]
                )
            return SimpleNamespace(success=True, sol_space=2)

        def _call_checked(self, key, request, timeout=180.0):
            self.requests.append((key, request))
            return True

        def verify_pose(self, target):
            self.targets.append(target)
            return self.arrived

    target = DsrPos(430.0, 10.0, 152.0, 0.0, 150.0, 0.0)
    for arrived in (True, False):
        robot = JointRecorder(arrived)
        assert robot.move_task_joint(target) == arrived
        key, request = robot.requests[0]
        assert key == "move_jointx"
        assert request.vel == 10.0 and request.acc == 20.0
        assert request.sync_type == 0 and request.ref == 0 and request.mode == 0
        assert request.sol == 2 and list(request.pos) == target.to_list()
        assert robot.targets == [target.to_list()]


def test_pen_body_is_outside_and_above_tip_at_thirty_degrees():
    """Tool +Z(펜촉 방향)와 그 반대인 몸통 방향을 혼동해 반전하지 않게 한다."""
    from scipy.spatial.transform import Rotation
    from cake_decorator.drawing_geometry import CylinderCake, pen_quat_side

    cake = CylinderCake(58.375, 79.0, 0.0, 0.0, 0.0)
    for theta in (math.pi - 0.4, math.pi, math.pi + 0.4):
        normal = cake.side_outward_normal(theta * cake.radius)
        tip_direction = Rotation.from_quat(
            pen_quat_side(theta * cake.radius, cake, tilt_deg=60.0)
        ).apply([0.0, 0.0, 1.0])
        body = -tip_direction
        assert abs(sum(a * b for a, b in zip(body, normal)) - 0.5) < 1e-10
        assert abs(body[2] - math.cos(math.radians(30.0))) < 1e-10
        assert tip_direction[2] < 0 and body[2] > 0
