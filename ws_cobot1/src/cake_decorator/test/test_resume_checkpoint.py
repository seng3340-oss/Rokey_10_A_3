"""동일 주문 재개에서 완료된 단계·면을 반복하지 않고 안전 확인을 먼저 수행하는지 검사한다."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock
import json
import threading

import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point32
from std_srvs.srv import Trigger
from cake_decorator_interfaces.action import CakeOrder
from cake_decorator_interfaces.msg import DrawingPath, Stroke2D
from cake_decorator_interfaces.srv import GenerateTrajectory

from cake_decorator.task_manager_node import TaskManagerNode
from cake_decorator.robot_executor_node import RobotExecutorNode
from cake_decorator.drawing_motion import TaskState
from cake_decorator.resume_checkpoint import checkpoint_json


@pytest.fixture
def resume_tm():
    """실제 TM 메서드를 사용하며 ROS 서비스·장비 경계만 기록 대역으로 바꾼다."""
    rclpy.init()
    node = TaskManagerNode()
    original = node._gen_traj_client
    node.set_parameters([
        Parameter('plate_sequence_enabled', value=True),
        Parameter('integrated_side_cycle', value=True),
        Parameter('side_reference_pose', value=[395.6, -51.3, 222., 0., 180., 0.]),
    ])
    events = []
    def service(client, phase):
        """안전 확인·준비·출고 호출 순서를 보관한다."""
        name = 'SAFE' if client is node._resume_ready_client else (
            'PREPARE' if client is node._plate_prepare_client else 'FINISH'
        )
        events.append(name)
        return True
    def trajectory(request):
        """실제 경로 생성 요청의 면을 기록하고 장비 없는 응답을 돌려준다."""
        events.append(request.surface_type)
        future = Future()
        future.set_result(GenerateTrajectory.Response(success=True))
        return future
    node._run_plate_service = service
    node._gen_traj_client = SimpleNamespace(wait_for_service=lambda **_: True, call_async=trajectory)
    node._wait_future = lambda *_: None
    node._send_draw = lambda *args: True
    request = CakeOrder.Goal(order_id='RESUME-SAME-ORDER')
    request.cake_params.radius = .05675
    request.cake_params.bottom_radius = .05925
    request.cake_params.height = .079
    request.cake_params.center.z = .511
    request.cake_params.tool_orientation.y = 1.
    for surface in ('TOP', 'SIDE'):
        request.drawing_paths.append(DrawingPath(
            surface=surface, canvas_width=1024, canvas_height=768,
            strokes=[Stroke2D(stroke_id=0, points=[Point32(x=512., y=384.), Point32(x=540., y=390.)])],
        ))
    try:
        yield node, request, events
    finally:
        node._gen_traj_client = original
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('stage,index,surface,expected', [
    ('PREPARING', 0, '', ['SAFE', 'PREPARE', 'TOP', 'SIDE', 'FINISH']),
    ('DRAWING', 0, 'TOP', ['SAFE', 'TOP', 'SIDE', 'FINISH']),
    ('DRAWING', 1, 'SIDE', ['SAFE', 'SIDE', 'FINISH']),
    ('FINISHING', 0, '', ['SAFE', 'FINISH']),
])
def test_resume_skips_completed_stages_and_surfaces(resume_tm, stage, index, surface, expected):
    """준비한 케이크와 이미 완성한 면을 중복 실행하지 않는다."""
    node, request, events = resume_tm
    request.resume_checkpoint = checkpoint_json(request, stage, index, surface, [])
    handle = Mock(request=request, is_cancel_requested=False)
    result = node._execute_order(handle)
    assert result.success, result.error_msg
    assert events == expected
    assert request.order_id == 'RESUME-SAME-ORDER'


def test_failed_safe_check_preserves_checkpoint_and_sends_no_process(resume_tm):
    """안전 확인이 실패하면 운반·그리기를 호출하지 않고 원래 재개 지점을 보존한다."""
    node, request, events = resume_tm
    request.resume_checkpoint = checkpoint_json(request, 'DRAWING', 1, 'SIDE', [])
    node._run_plate_service = lambda *args: events.append('SAFE') or False
    result = node._execute_order(Mock(request=request, is_cancel_requested=False))
    assert not result.success and result.error_code == 'RESUME_NOT_SAFE'
    saved = json.loads(result.resume_checkpoint)
    assert saved['stage'] == 'DRAWING' and saved['path_index'] == 1
    assert events == ['SAFE']


@pytest.mark.parametrize('condition', ['ready', 'wrong_pose', 'wrong_tcp', 'stop', 'closed_gripper', 'moving'])
def test_robot_resume_validation_only_reads_and_rejects_unsafe_state(condition):
    """관리자가 확인했어도 실제 자세·TCP·빈 그리퍼 조건이 틀리면 이동 없이 거부한다."""
    target = [35., 40., 35., -20., 120., 120.]
    params = {'plate_safe_position_1': target, 'gripper_safety_check_enabled': True, 'pen_open_min_width_m': .06}
    joints = target.copy()
    if condition == 'wrong_pose': joints[0] += 2.
    robot = Mock(stop_latched=condition == 'stop', gripper_output_fault=False)
    robot._call.side_effect = [
        SimpleNamespace(success=True, pos=joints),
        SimpleNamespace(success=True, info='wrong' if condition == 'wrong_tcp' else ''),
        SimpleNamespace(success=True, status=1 if condition == 'moving' else 0),
    ]
    robot.get_gripper_width.return_value = .01 if condition == 'closed_gripper' else .08
    node = SimpleNamespace(_state=TaskState.IDLE, _robot=robot, _cancel_requested=False,
                           _plate_lock=threading.Lock(), _param=params.__getitem__)
    response = RobotExecutorNode._check_resume_ready(node, None, Trigger.Response())
    assert response.success == (condition == 'ready')
    robot.move_joint.assert_not_called()
    robot.move_line.assert_not_called()
    robot.gripper_open.assert_not_called()


def test_resume_goal_keeps_original_calibration_even_if_launcher_reference_changed():
    """재개할 때 현재 브릿지 기준점으로 원래 케이크 목표를 덮어쓰지 않는다."""
    from cake_decorator.goal_mapping import build_goal
    from test_goal_mapping import GoalTests
    claim = GoalTests().combined_claim()
    original = build_goal(claim, [399., -53., 511., 0., 180., 0.])
    claim['resume_checkpoint'] = json.loads(checkpoint_json(original, 'DRAWING', 1, 'SIDE', []))
    resumed = build_goal(claim, [500., 50., 400., 0., 180., 0.])
    assert resumed.cake_params.center == original.cake_params.center
    assert resumed.cake_params.tool_orientation == original.cake_params.tool_orientation
    assert resumed.drawing_paths == original.drawing_paths
    assert json.loads(resumed.resume_checkpoint)['path_index'] == 1
    claim['resume_checkpoint']['order_id'] = 'OTHER'
    with pytest.raises(ValueError):
        build_goal(claim, [399., -53., 511., 0., 180., 0.])
