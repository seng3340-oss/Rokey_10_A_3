"""실물 장비 없이 영점 자세 보존과 거짓 이동 성공을 검출한다."""

from types import SimpleNamespace
import math
import rclpy
from geometry_msgs.msg import Point32, Quaternion
from cake_decorator_interfaces.msg import CakeParams, Stroke2D
from cake_decorator_interfaces.srv import GenerateTrajectory
from cake_decorator.task_manager_node import TaskManagerNode
from cake_decorator.drawing_path_node import DrawingPathNode
from cake_decorator.doosan_robot import DoosanRobotClient
from cake_decorator.drawing_motion import DsrPos


def test_tm_preserves_taught_tool_orientation():
    cake = CakeParams()
    cake.center.x, cake.center.z = 0.373, 0.528
    cake.tool_orientation = Quaternion(x=0.002, y=math.sqrt(1 - 0.002**2), z=0.0, w=0.0)
    pose = TaskManagerNode._reference_pose(cake)
    assert pose.orientation == cake.tool_orientation
    assert pose.position == cake.center


def test_top_path_keeps_reference_orientation_including_lift():
    rclpy.init()
    node = DrawingPathNode()
    try:
        request = GenerateTrajectory.Request()
        request.canvas_width, request.canvas_height = 1024.0, 768.0
        request.cake_radius, request.cake_height = 0.05, 0.08
        request.cake_pose.position.x, request.cake_pose.position.z = 0.373, 0.528
        request.cake_pose.orientation = Quaternion(
            x=0.002, y=math.sqrt(1 - 0.002**2), z=0.0, w=0.0
        )
        request.operation_speed_percent = 100
        request.surface_type = "TOP"
        request.strokes_2d = [
            Stroke2D(
                stroke_id=0,
                points=[Point32(x=512.0, y=384.0), Point32(x=600.0, y=384.0)],
            )
        ]
        path, _ = node._convert_strokes(request)
        for point in path:
            assert (
                abs(point.pose.orientation.x - request.cake_pose.orientation.x) < 1e-10
            )
            assert (
                abs(point.pose.orientation.y - request.cake_pose.orientation.y) < 1e-10
            )
        assert node._param("pen_tilt_deg") == 0.0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


class FalseSuccessRobot(DoosanRobotClient):
    """제어기 응답은 성공이지만 목표 미도달인 상황을 재현한다."""

    def __init__(self, arrived):
        self.arrived = arrived
        self.targets = []
        self._logger = SimpleNamespace(warn=lambda _: None, error=lambda _: None)

    def _call(self, key, request, timeout=180.0):
        return SimpleNamespace(success=True)

    def verify_pose(self, target):
        self.targets.append(target)
        return self.arrived


def test_move_line_success_requires_actual_tcp_arrival():
    robot = FalseSuccessRobot(False)
    assert not robot.move_line(DsrPos(373.0, 0.0, 528.0, 0.0, 180.0, 0.0))
    assert len(robot.targets) == 1


def test_move_line_arrival_can_succeed():
    robot = FalseSuccessRobot(True)
    assert robot.move_line(DsrPos(373.0, 0.0, 528.0, 0.0, 180.0, 0.0))
    assert len(robot.targets) == 1


def test_circle_and_spline_success_also_require_arrival():
    """직선만 검사하고 원호·스플라인의 거짓 성공을 놓치지 않게 한다."""
    points = [
        DsrPos(373.0, 5.0, 528.0, 0.0, 180.0, 0.0),
        DsrPos(373.0, 10.0, 528.0, 0.0, 180.0, 0.0),
    ]
    robot = FalseSuccessRobot(False)
    assert not robot.move_circle(points[0], points[1], speed_mm_s=30.0)
    assert not robot.move_spline(points, speed_mm_s=30.0)
    assert robot.targets == [points[1].to_list(), points[1].to_list()]


def test_arrival_accepts_equivalent_zyz_representation():
    """B 부호와 A·C가 다르게 표시돼도 같은 회전이면 도달로 인정한다."""

    class EquivalentPoseRobot(DoosanRobotClient):
        def _call(self, key, request, timeout=180.0):
            return SimpleNamespace(
                success=True,
                task_pos_info=[
                    SimpleNamespace(
                        data=[500.0, 0.0, 152.0, 31.249284744, -150.0, -31.2492752]
                    )
                ],
            )

    robot = EquivalentPoseRobot.__new__(EquivalentPoseRobot)
    assert robot.verify_pose([500.0, 0.0, 152.0, -148.750713893, 150.0, 148.750713893])
