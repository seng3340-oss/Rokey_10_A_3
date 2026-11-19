"""DP 원호가 메시지·RE 계획을 거쳐 MoveCircle 요청으로 보존되는지 확인한다."""

import math
from types import SimpleNamespace
import unittest

import rclpy
from geometry_msgs.msg import Point32
from rclpy.serialization import serialize_message, deserialize_message
from cake_decorator_interfaces.msg import Stroke2D, Waypoint3D
from cake_decorator_interfaces.srv import GenerateTrajectory
from cake_decorator.drawing_path_node import DrawingPathNode
from cake_decorator.robot_executor_node import RobotExecutorNode
from cake_decorator.drawing_motion import MotionType, plan_drawing_motion
from cake_decorator.doosan_robot import DoosanRobotClient


class RequestRecorder(DoosanRobotClient):
    """실제 서비스·장비 없이 요청 메시지만 기록한다."""

    def __init__(self):
        self.requests = []
        self._logger = SimpleNamespace(error=lambda message: None)

    def _call_checked(self, key, request, timeout=180.0):
        self.requests.append((key, request))
        return True


class SurfaceContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = DrawingPathNode()
        from rclpy.parameter import Parameter
        cls.node.set_parameters([Parameter("side_pen_vertical_deg", value=30.0),
                                 Parameter("side_diameter_allowance_mm", value=0.0)])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.try_shutdown()

    def request(self, surface, strokes):
        request = GenerateTrajectory.Request()
        request.canvas_width, request.canvas_height = 1024.0, 768.0
        request.cake_radius, request.cake_height = 0.05, 0.08
        request.cake_bottom_radius = 0.05
        request.cake_pose.position.x = 0.5
        request.cake_pose.position.z = 0.15
        request.operation_speed_percent = 100
        request.surface_type = surface
        request.strokes_2d = [
            Stroke2D(
                stroke_id=i, points=[Point32(x=float(x), y=float(y)) for x, y in points]
            )
            for i, points in enumerate(strokes)
        ]
        response = self.node._handle_request(request, GenerateTrajectory.Response())
        self.assertTrue(response.success, response.error_msg)
        return response

    def test_top_straight_1001_points_are_reduced_to_two_and_keep_one_stroke(self):
        response = self.request("TOP", [[(512 + i * 0.2, 384) for i in range(1001)]])
        self.assertEqual(len(response.waypoints_3d), 4)
        raw, pen = RobotExecutorNode._convert_waypoints(response.waypoints_3d)
        plan = plan_drawing_motion(raw, pen, "TOP")
        self.assertTrue(plan.success, plan.error_msg)
        self.assertEqual(plan.total_strokes, 1)
        self.assertTrue(
            all(cmd.motion_type == MotionType.MOVE_LINE for cmd in plan.commands)
        )

    def test_horizontal_side_reaches_move_circle_service_with_via_and_no_blending(self):
        response = self.request("SIDE", [[(64, 384), (512, 384)]])
        raw, pen = RobotExecutorNode._convert_waypoints(response.waypoints_3d)
        plan = plan_drawing_motion(raw, pen, "SIDE")
        self.assertTrue(plan.success, plan.error_msg)
        self.assertEqual(plan.total_strokes, 1)
        arcs = [
            cmd for cmd in plan.commands if cmd.motion_type == MotionType.MOVE_CIRCLE
        ]
        self.assertEqual(len(arcs), 2)
        robot = RequestRecorder()
        for command in plan.commands:
            self.assertTrue(robot.execute_drawing_command(command))
        circles = [req for key, req in robot.requests if key == "move_circle"]
        self.assertEqual(len(circles), 2)
        for request in circles:
            self.assertEqual(len(request.pos), 2)
            self.assertEqual(request.ref, 0)
            self.assertEqual(request.sync_type, 0)
            self.assertEqual(request.radius, 0.0)
            for point in request.pos:
                self.assertAlmostEqual(
                    math.hypot(point.data[0] - 500, point.data[1]), 50.0
                )
        # 실제 ROS 직렬화에서도 원호 종류와 경유점이 사라지지 않아야 한다.
        message = next(
            wp for wp in response.waypoints_3d if wp.motion_type == Waypoint3D.CIRCLE
        )
        restored = deserialize_message(serialize_message(message), Waypoint3D)
        self.assertEqual(restored.motion_type, Waypoint3D.CIRCLE)
        self.assertEqual(restored.via_pose, message.via_pose)

    def test_two_side_strokes_reposition_on_outer_circle(self):
        response = self.request(
            "SIDE", [[(64, 384), (288, 384)], [(736, 384), (960, 384)]]
        )
        raw, pen = RobotExecutorNode._convert_waypoints(response.waypoints_3d)
        plan = plan_drawing_motion(raw, pen, "SIDE")
        self.assertTrue(plan.success, plan.error_msg)
        self.assertEqual(plan.total_strokes, 2)
        clear_arcs = [
            c
            for c in plan.commands
            if c.motion_type == MotionType.MOVE_SPLINE and not c.pen_down
        ]
        self.assertTrue(clear_arcs)
        for command in clear_arcs:
            self.assertAlmostEqual(math.hypot(command.pos.x - 500, command.pos.y), 60.0)
            for point in command.spline:
                self.assertAlmostEqual(math.hypot(point.x - 500, point.y), 60.0)

    def test_diagonal_side_plan_keeps_every_circle_horizontal(self):
        response = self.request("SIDE", [[(64, 380), (288, 390)]])
        raw, pen = RobotExecutorNode._convert_waypoints(response.waypoints_3d)
        plan = plan_drawing_motion(raw, pen, "SIDE")
        self.assertTrue(plan.success, plan.error_msg)
        previous = None
        for command in plan.commands:
            if command.motion_type == MotionType.MOVE_CIRCLE:
                self.assertAlmostEqual(previous.z, command.via.z)
                self.assertAlmostEqual(command.pos.z, command.via.z)
            previous = command.pos

    def test_diagonal_spline_changes_height_and_keeps_thirty_degree_axis(self):
        response = self.request("SIDE", [[(380, 370), (550, 400), (680, 360)]])
        raw, pen = RobotExecutorNode._convert_waypoints(response.waypoints_3d)
        plan = plan_drawing_motion(raw, pen, "SIDE")
        self.assertTrue(plan.success, plan.error_msg)
        splines = [
            wp for wp in response.waypoints_3d if wp.motion_type == Waypoint3D.SPLINE
        ]
        self.assertTrue(splines)
        robot = RequestRecorder()
        for command in plan.commands:
            self.assertTrue(robot.execute_drawing_command(command))
        requests = [req for key, req in robot.requests if key == "move_spline"]
        self.assertTrue(requests)
        for wp in splines:
            self.assertTrue(2 <= len(wp.spline_poses) <= 100)
            for pose in wp.spline_poses:
                p, q = pose.position, pose.orientation
                self.assertAlmostEqual(math.hypot(p.x - 0.5, p.y), 0.05)
                axis_z = 1 - 2 * (q.x * q.x + q.y * q.y)
                self.assertAlmostEqual(math.degrees(math.acos(-axis_z)), 30.0)
        self.assertGreater(
            max(p.position.z for wp in splines for p in wp.spline_poses)
            - min(p.position.z for wp in splines for p in wp.spline_poses),
            0.005,
        )
