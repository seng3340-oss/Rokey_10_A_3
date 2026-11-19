"""웹 원본 옆면 좌표가 검증한 45도 파지 자세로 변환되는지 검사한다."""

import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation

from cake_decorator.bent_pen import VERTICAL_GRIPPER_ZYZ, side_gripper_quaternion
from cake_decorator.drawing_path_node import DrawingPathNode
from cake_decorator.side_pen_return import return_pen
from cake_decorator.robot_executor_node import RobotExecutorNode
from cake_decorator.drawing_motion import plan_drawing_motion
from cake_decorator_interfaces.msg import Stroke2D
from cake_decorator_interfaces.srv import GenerateTrajectory
from geometry_msgs.msg import Point32


def request(surface):
    """실제 웹의 캔버스·옆면 편집 띠 안에서 수평·대각선을 포함하는 획을 만든다."""
    result=GenerateTrajectory.Request()
    result.order_id="SIDE45-WEB-TEST"
    result.canvas_width=1024.
    result.canvas_height=768.
    result.surface_type=surface
    result.operation_speed_percent=100
    result.cake_radius=.05675
    result.cake_bottom_radius=.06
    result.cake_height=.079
    result.cake_pose.position.x=.399274
    result.cake_pose.position.y=-.054559
    result.cake_pose.position.z=.220
    quat=Rotation.from_euler("ZYZ",VERTICAL_GRIPPER_ZYZ,degrees=True).as_quat()
    result.cake_pose.orientation.x=float(quat[0])
    result.cake_pose.orientation.y=float(quat[1])
    result.cake_pose.orientation.z=float(quat[2])
    result.cake_pose.orientation.w=float(quat[3])
    result.strokes_2d=[
        Stroke2D(stroke_id=0,points=[Point32(x=300.,y=360.),Point32(x=450.,y=360.),Point32(x=550.,y=410.)]),
        Stroke2D(stroke_id=1,points=[Point32(x=650.,y=350.),Point32(x=680.,y=400.)]),
    ]
    return result


def test_web_side_poses_and_stroke_boundaries():
    """서비스의 모든 목표·경유·스플라인에서 RG2 수직과 중심 방향을 유지한다."""
    rclpy.init()
    node=DrawingPathNode()
    try:
        node.set_parameters([Parameter("side_pen_45",value=True)])
        req=request("SIDE")
        response=node._handle_request(req,GenerateTrajectory.Response())
        assert response.success,response.error_msg
        assert sum(w.pen_down and (i==0 or not response.waypoints_3d[i-1].pen_down) for i,w in enumerate(response.waypoints_3d))==2
        checked=0
        for index, waypoint in enumerate(response.waypoints_3d):
            poses=[waypoint.pose,*waypoint.spline_poses]
            if waypoint.motion_type==waypoint.CIRCLE:
                poses.append(waypoint.via_pose)
            for pose in poses:
                q=pose.orientation
                actual=Rotation.from_quat([q.x,q.y,q.z,q.w])
                np.testing.assert_allclose(actual.apply([0,0,1]),[0,0,-1],atol=1e-9)
                dx=pose.position.x-req.cake_pose.position.x
                dy=pose.position.y-req.cake_pose.position.y
                if index >= 2 and math.hypot(dx,dy)>1e-9:
                    expected=Rotation.from_quat(side_gripper_quaternion(pose.position.x,pose.position.y,req.cake_pose.position.x,req.cake_pose.position.y))
                    assert (expected.inv()*actual).magnitude()<1e-9
                checked+=1
        assert checked>10
        # 웹 서비스 응답에서 끝내지 않고 RE가 실제로 보낼 mm·ZYZ 명령까지 검사한다.
        dictionaries, states = RobotExecutorNode._convert_waypoints(response.waypoints_3d)
        plan = plan_drawing_motion(dictionaries, states, "SIDE")
        assert plan.success, plan.error_msg
        assert math.isclose(plan.commands[1].pos.z, 290., abs_tol=1e-9)
        for index, command in enumerate(plan.commands):
            points = [command.pos, *(command.spline or [])]
            if command.via is not None:
                points.append(command.via)
            for point in points:
                dx = point.x / 1000. - req.cake_pose.position.x
                dy = point.y / 1000. - req.cake_pose.position.y
                if index < 2 or math.hypot(dx, dy) < 1e-9:
                    continue
                expected = Rotation.from_quat(side_gripper_quaternion(
                    point.x / 1000., point.y / 1000.,
                    req.cake_pose.position.x, req.cake_pose.position.y,
                ))
                actual = Rotation.from_euler("ZYZ", point.to_list()[3:], degrees=True)
                assert (expected.inv() * actual).magnitude() < 1e-9
        # 실물 MOVE_FAILED_AT_1의 회귀 검사. 첫 XY 이동은 회전을 포함하지 않는다.
        first = Rotation.from_euler("ZYZ", plan.commands[0].pos.to_list()[3:], degrees=True)
        second = Rotation.from_euler("ZYZ", plan.commands[1].pos.to_list()[3:], degrees=True)
        third = Rotation.from_euler("ZYZ", plan.commands[2].pos.to_list()[3:], degrees=True)
        assert (first.inv() * second).magnitude() < 1e-9
        assert (second.inv() * third).magnitude() > math.radians(1.)
        np.testing.assert_allclose(
            plan.commands[1].pos.to_list()[:3],
            plan.commands[2].pos.to_list()[:3],
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_return_rewinds_before_opening_and_failure_keeps_pen():
    """되감기 뒤에만 펜을 반납하고, 이동 실패 후에는 그리퍼를 열지 않는다."""
    events=[]
    robot=Mock()
    state={"joints":[-8.,8.,64.,0.,109.,-300.],"pose":[400.,-50.,260.,0.,180.,0.]}

    def query(key,req):
        if key=="current_joints":
            return SimpleNamespace(success=True,pos=state["joints"])
        if key=="current_pose":
            return SimpleNamespace(success=True,task_pos_info=[SimpleNamespace(data=state["pose"])])
        if key=="gripper":
            events.append("open")
            return SimpleNamespace(success=True)
        raise AssertionError(key)

    def joint(target,**kwargs):
        state["joints"]=list(target)
        events.append(("joints",list(target)))
        return True

    robot._call.side_effect=query
    robot.move_joint.side_effect=joint
    robot.move_line.return_value=True
    robot.verify_pose.return_value=True
    with patch("cake_decorator.side_pen_return.time.sleep"):
        return_pen(robot,[43.59,.54,69.87,.09,109.62,43.31])
    assert events[0][1][5]==-300.
    assert events[1][1][5]==43.31
    assert events[-1]=="open"
    events.clear()
    robot.move_joint.return_value=False
    robot.move_joint.side_effect=None
    with pytest.raises(RuntimeError):
        return_pen(robot,[43.59,.54,69.87,.09,109.62,43.31])
    assert "open" not in events


def test_side_45_full_turn_has_no_quarter_turn_stops():
    """한 바퀴를 90도 원호마다 나누지 않고 하나의 연속 그리기 명령으로 보낸다."""
    from cake_decorator.frustum_path import DrawingFrustum
    from cake_decorator_interfaces.msg import Waypoint3D

    rclpy.init()
    node = DrawingPathNode()
    try:
        node.set_parameters([Parameter("side_pen_45", value=True)])
        cake = DrawingFrustum(r_top=56.75, r_bottom=60., height=79., cx=400., cy=0., cz=141.)
        waypoints, states = [], []
        node._convert_side_mm(
            [(0., 40.), (math.pi * cake.radius, 40.),
             (2 * math.pi * cake.radius, 40.)],
            None, cake, 10., 0., 30., 100., 0., 0., waypoints, states,
        )
        drawing = [w for w in waypoints if w.pen_down]
        assert not any(w.motion_type == Waypoint3D.CIRCLE for w in drawing)
        splines = [w for w in drawing if w.motion_type == Waypoint3D.SPLINE]
        assert len(splines) == 1
        assert 2 <= len(splines[0].spline_poses) <= 100
    finally:
        node.destroy_node()
        rclpy.shutdown()
