"""오프라인 워크플로 테스트와 별도 읽기 전용 작업셀 점검을 제공한다.

1. ROS 주문: 실제 작업 관리자와 경로 계산 노드를 테스트 이름 공간에 연결한다.
   입력 픽셀 → BASE 기준 웨이포인트 → 모의 DrawOnCake → 주문 결과 순서다.
   현행 CakeOrder에는 판 이송, 펜 파지·반납, 안전자세 이동이 연결되지 않는다.
2. 실행기·드라이버: 로봇과 서비스 응답을 모의로 대체해 실패 명령 뒤의
   이동·그리퍼 명령을 검사한다. Goal 중단이나 후속 명령 생략은 컨트롤러에
   이미 전달된 동작을 정지시키지 않는다. 물리 비상정지와 정지 응답은 검증 범위 밖이다.
3. DART 명령 기록: CAKE_DRL_DIR의 판·펜 원본을 파싱해 티칭 순서를 검사한다.
   DRL은 실행하지 않는다. 첫 이동 전 실제 시작 자세, 활성 TCP/Tool, 도구와
   판의 배치, 그리퍼 배선·피드백을 확인해야 하며, 이 테스트는 이를 가정하지 않는다.

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1로 ROS 플러그인의 로그 설정을 피하고
pytest --log-cli-level=INFO로 실행하면 예상한 실패도 볼 수 있다.
예상한 실패 결과는 INFO, 예상과 다른 결과나 읽기 오류는 ERROR로 기록한다.
실제 작업셀에서는 ROS 환경을 불러온 뒤 이 파일을 --check-workcell로 직접 실행한다.
그 모드는 기존 ROS 도메인과 드라이버를 사용하며 이동·파지·정지 명령을 보내지 않는다.
"""

import argparse
import ast
import json
import logging
import math
import os
from concurrent.futures import Future
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from threading import Event, Thread
from unittest.mock import Mock
from uuid import uuid4

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32
from rclpy.action import ActionClient, ActionServer
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from cake_decorator.drawing_path_node import DrawingPathNode
from cake_decorator.grip_policy import grip_width_error
from cake_decorator.task_manager_node import TaskManagerNode
from cake_decorator_interfaces.action import (
    CakeOrder,
    DrawOnCake,
    GraspTool,
    PickAndPlace,
)
from cake_decorator_interfaces.msg import DrawingPath, Stroke2D


LOGGER = logging.getLogger(__name__)


# 같은 픽셀 입력으로 TOP과 SIDE를 검증한다. 두 점씩 떨어뜨려 획 사이 이동을 확인한다.
TWO_STROKES = (
    ((480, 340), (520, 350)),
    ((610, 420), (640, 460)),
)


@pytest.mark.parametrize("surface", ["TOP", "SIDE"])
def test_two_strokes_complete(workflow, surface):
    """선택한 면의 두 획이 한 번의 드로잉 요청으로 전달되고 주문이 완료된다."""
    order = make_order(make_drawing(surface, TWO_STROKES))
    result = workflow.submit_order(order)

    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.success is True
    assert len(workflow.path.calls) == 1
    assert len(workflow.draw.goals) == 1

    # 첫 번째 전달: 작업 관리자 → 경로 계산 노드. 픽셀 순서와 BASE 기준
    # 윗면 중심 좌표가 그대로 도착해야 경로 노드가 의도한 위치를 계산할 수 있다.
    request, trajectory = workflow.path.calls[0]
    drawing = order.drawing_paths[0]
    cake = order.cake_params
    assert request.order_id == order.order_id
    assert request.surface_type == surface
    assert request.canvas_width == drawing.canvas_width
    assert request.canvas_height == drawing.canvas_height
    assert request.strokes_2d == drawing.strokes
    assert request.cake_pose.position == cake.center
    assert request.cake_pose.orientation == cake.tool_orientation
    assert request.cake_yaw == cake.orientation
    assert request.cake_radius == cake.radius
    assert request.cake_bottom_radius == cake.bottom_radius
    assert request.cake_height == cake.height

    draw_goal = workflow.draw.goals[0]
    assert_draw_goal_matches_trajectory(draw_goal, trajectory)
    assert draw_goal.drawing_surface == surface

    # False는 펜을 든 상태, True는 그리는 상태다. 두 획이면 False에서
    # True로 돌아서는 순간이 두 번 있어야 하며 끝에서는 펜을 들어야 한다.
    pen_states = [waypoint.pen_down for waypoint in draw_goal.waypoints]
    assert pen_states
    assert pen_states[0] is False
    assert pen_states[-1] is False
    assert sum(
        not before and after
        for before, after in zip(pen_states, pen_states[1:])
    ) == 2


def test_two_surfaces_are_drawn_in_order(workflow):
    """한 ROS 주문의 윗면과 옆면이 지정 순서대로 계산·실행된다."""
    order = make_order(
        make_drawing("TOP", TWO_STROKES),
        make_drawing("SIDE", TWO_STROKES),
    )
    result = workflow.submit_order(order)

    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.success is True
    assert [request.surface_type for request, _ in workflow.path.calls] == [
        "TOP",
        "SIDE",
    ]
    assert [goal.drawing_surface for goal in workflow.draw.goals] == [
        "TOP",
        "SIDE",
    ]
    for (_, trajectory), draw_goal in zip(workflow.path.calls, workflow.draw.goals):
        assert_draw_goal_matches_trajectory(draw_goal, trajectory)


def test_path_failure_stops_before_drawing(workflow):
    """한 점짜리 획을 경로 서비스가 처리할 수 없으면 드로잉은 시작하지 않는다."""
    single_point_stroke = [(480, 340)]
    order = make_order(make_drawing("TOP", [single_point_stroke]))
    result = workflow.submit_order(order, expected_error="DRAWING_FAILED")

    assert len(workflow.path.calls) == 1
    _, trajectory = workflow.path.calls[0]
    assert trajectory.success is False
    assert workflow.draw.goals == []
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.success is False
    assert result.result.error_code == "DRAWING_FAILED"


def test_draw_failure_skips_remaining_surface(workflow):
    """윗면 드로잉이 실패하면 다음 옆면의 경로 요청도 시작하지 않는다."""
    # ROS Goal의 두 경로를 직접 구성해 작업 관리자의 중단 순서를 확인한다.
    order = make_order(
        make_drawing("TOP", TWO_STROKES),
        make_drawing("SIDE", TWO_STROKES),
    )
    workflow.draw.should_succeed = False

    result = workflow.submit_order(order, expected_error="DRAWING_FAILED")

    assert len(workflow.path.calls) == 1
    request, trajectory = workflow.path.calls[0]
    assert request.surface_type == "TOP"
    assert trajectory.success is True
    assert [goal.drawing_surface for goal in workflow.draw.goals] == ["TOP"]
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.success is False
    assert result.result.error_code == "DRAWING_FAILED"


def test_next_order_can_start_after_draw_failure(workflow):
    """드로잉 실패가 끝난 뒤 같은 작업 관리자가 다음 주문을 받아 완료한다."""
    workflow.draw.should_succeed = False
    failed = workflow.submit_order(
        make_order(make_drawing("TOP", TWO_STROKES)),
        expected_error="DRAWING_FAILED",
    )
    assert failed.status == GoalStatus.STATUS_ABORTED

    workflow.draw.should_succeed = True
    next_order = make_order(
        make_drawing("TOP", TWO_STROKES),
        order_id="workflow-next",
    )
    completed = workflow.submit_order(next_order)

    assert completed.status == GoalStatus.STATUS_SUCCEEDED
    assert completed.result.success is True
    assert len(workflow.draw.goals) == 2
    assert [request.order_id for request, _ in workflow.path.calls] == [
        "workflow-test",
        "workflow-next",
    ]


def assert_draw_goal_matches_trajectory(draw_goal, trajectory):
    """경로 응답의 위치·자세·펜 상태가 실행 요청까지 그대로 전달됐는지 검사한다."""
    assert trajectory.success is True
    assert draw_goal.waypoints == trajectory.waypoints_3d


# 로봇 호출을 모의로 바꿔 실행기와 드라이버의 중단 조건을 검사한다.
@pytest.fixture
def robot_commands():
    """실제 모듈의 메서드를 호출하되 노드 생성자와 장비 연결은 생략한다.

    실제 인터페이스 타입을 불러오므로 생성된 DSR/RG 메시지가 필요하다.
    반환한 Mock에서 로봇 명령에 실패를 주입하고 전체 호출 순서를 읽는다.
    """
    pytest.importorskip("dsr_msgs2.srv")
    pytest.importorskip("onrobot_rg_msgs.srv")
    from cake_decorator.doosan_robot import DoosanRobotClient
    from cake_decorator.robot_executor_node import RobotExecutorNode

    robot = Mock(spec=DoosanRobotClient)
    robot.move_line.return_value = True
    robot.gripper_open.return_value = True
    robot.gripper_close.return_value = True
    robot.verify_grip.return_value = ""
    executor = Mock(spec=RobotExecutorNode)
    executor._robot = robot
    executor._param.side_effect = RobotExecutorNode._PARAM_DEFAULTS.__getitem__
    return RobotExecutorNode, executor, robot


# 각 행은 성공 경로의 명령 한 건과 그 명령이 실패할 때의 액션 결과 코드다.
# 동일한 MoveLine도 접근, 하강, 이송, 후퇴 중 어느 시점인지에 따라 구별한다.
PEN_GRASP_COMMANDS = (
    ("move_line", "APPROACH_FAILED"),
    ("gripper_open", "TOOL_OPEN_FAILED"),
    ("move_line", "MOVE_TO_TOOL_FAILED"),
    ("gripper_close", "TOOL_GRIP_FAILED"),
    ("verify_grip", "GRIP_INDETERMINATE"),
    ("move_line", "RETRACT_FAILED"),
)
PEN_RELEASE_COMMANDS = (
    ("move_line", "APPROACH_FAILED"),
    ("move_line", "MOVE_TO_TOOL_FAILED"),
    ("gripper_open", "TOOL_RELEASE_FAILED"),
    ("move_line", "RETRACT_FAILED"),
)
PLATE_TRANSFER_COMMANDS = (
    ("move_line", "PICK_APPROACH_FAILED"),
    ("gripper_open", "PICK_OPEN_FAILED"),
    ("move_line", "PICK_DESCEND_FAILED"),
    ("gripper_close", "PICK_GRIP_FAILED"),
    ("verify_grip", "GRIP_INDETERMINATE"),
    ("move_line", "PICK_LIFT_FAILED"),
    ("move_line", "PLACE_MOVE_FAILED"),
    ("move_line", "PLACE_DESCEND_FAILED"),
    ("gripper_open", "PLACE_RELEASE_FAILED"),
    ("move_line", "PLACE_RETRACT_FAILED"),
)


def failure_cases(commands, *goal_args):
    """실패 명령까지의 호출 순서와 오류 코드로 테스트 사례를 만든다."""
    return [
        pytest.param(
            *goal_args,
            tuple(command for command, _ in commands[:index + 1]),
            error_code,
            id="-".join((*goal_args, error_code)),
        )
        for index, (_, error_code) in enumerate(commands)
    ]


def fail_on_last_expected_call(robot, expected_calls, error_code):
    """앞선 호출을 성공시키고 지정된 호출에서 해당 API의 실패 응답을 반환한다."""
    failing_command = expected_calls[-1]
    if failing_command == "verify_grip":
        robot.verify_grip.return_value = error_code
    else:
        prior_calls = expected_calls[:-1].count(failing_command)
        getattr(robot, failing_command).side_effect = [True] * prior_calls + [False]


def assert_action_stopped(goal, result, robot, expected_calls, error_code):
    """액션 실패와 실패 명령까지의 호출 기록을 모두 확인한다.

    호출 목록 전체를 비교해 후퇴·홈 이동이나 같은 명령의 재시도도 검출한다.
    실제 구동 중 모션이 멈췄는지 또는 물체를 유지하는지는 판단할 수 없다.
    """
    observed_calls = tuple(call[0] for call in robot.method_calls)
    # 기대한 호출 순서 자체가 달라졌다면 실패 명령의 위치도 확정할 수 없다.
    if observed_calls[:len(expected_calls)] == expected_calls:
        later_calls = observed_calls[len(expected_calls):]
    else:
        later_calls = "확인 불가: 실패 지점까지의 호출 순서가 다름"

    matched = (
        result.success is False
        and result.error_code == error_code
        and goal.abort.call_count == 1
        and goal.succeed.call_count == 0
        and observed_calls == expected_calls
    )
    LOGGER.log(
        logging.INFO if matched else logging.ERROR,
        "[모의 명령 실패] 기대코드=%s 응답코드=%r 실패지점=%d:%s "
        "성공응답=%s abort횟수=%d 호출기록=%s 후속명령=%s",
        error_code, result.error_code, len(expected_calls), expected_calls[-1],
        result.success, goal.abort.call_count, observed_calls, later_calls,
    )
    assert result.success is False
    assert result.error_code == error_code
    goal.abort.assert_called_once()
    goal.succeed.assert_not_called()
    assert observed_calls == expected_calls


@pytest.mark.parametrize(
    ("action", "expected_calls", "error_code"),
    [
        *failure_cases(PEN_GRASP_COMMANDS, "GRASP"),
        *failure_cases(PEN_RELEASE_COMMANDS, "RELEASE"),
    ],
)
def test_tool_action_stops_after_failed_command(
    robot_commands, action, expected_calls, error_code,
):
    """BASE 기준 펜 접근·하강·파지/개방·후퇴 어느 단계에서든 실패를 전파한다."""
    node_type, node, robot = robot_commands
    request = GraspTool.Goal(tool_id="pen", action=action)
    request.tool_pose.orientation.w = 1.0
    # 현 구현은 접근 방향 벡터의 z만 사용하며 +1이면 BASE Z로 0.05 m를 더한다.
    request.approach_dir.z = 1.0
    goal = Mock(request=request)
    fail_on_last_expected_call(robot, expected_calls, error_code)

    result = node_type._execute_grasp(node, goal)

    assert_action_stopped(goal, result, robot, expected_calls, error_code)


@pytest.mark.parametrize(
    ("expected_calls", "error_code"),
    failure_cases(PLATE_TRANSFER_COMMANDS),
)
def test_transfer_stops_after_failed_command(
    robot_commands, expected_calls, error_code,
):
    """집기·들어올리기·이송·배치·후퇴에서 실패 응답 직후 호출을 끝낸다."""
    node_type, node, robot = robot_commands
    # ROS Pose는 BASE 기준 m와 쿼터니언이다. 접근 높이 0.05 m는 BASE Z에
    # 더한다. 실행기가 DSR 위치를 mm·ZYZ°로 바꾸며 그리퍼 폭은 처음부터 mm다.
    request = PickAndPlace.Goal(approach_height=0.05, grip_width=40.0)
    request.pick_pose.orientation.w = 1.0
    request.place_pose.orientation.w = 1.0
    goal = Mock(request=request)
    fail_on_last_expected_call(robot, expected_calls, error_code)

    result = node_type._execute_pick_place(node, goal)

    assert_action_stopped(goal, result, robot, expected_calls, error_code)


@pytest.mark.parametrize("tool_id", ["opener", "pen"])
@pytest.mark.parametrize("grip_error", ["", "GRIP_ERROR", "GRIP_INDETERMINATE", "GRIP_CHECK_UNAVAILABLE"])
def test_grasp_action_checks_measurement_before_retraction(robot_commands, tool_id, grip_error):
    """실제 액션 콜백의 닫기 → 실측 → 후퇴 순서와 실패 시 조기 중단을 확인한다."""
    node_type, node, robot = robot_commands
    request = GraspTool.Goal(tool_id=tool_id, action="GRASP")
    request.tool_pose.orientation.w = 1.0
    request.approach_dir.z = 1.0
    goal = Mock(request=request)
    robot.verify_grip.return_value = grip_error

    result = node_type._execute_grasp(node, goal)

    expected = ("move_line", "gripper_open", "move_line", "gripper_close", "verify_grip")
    robot.verify_grip.assert_called_once_with(tool_id)
    if grip_error:
        assert_action_stopped(goal, result, robot, expected, grip_error)
    else:
        assert result.success is True
        goal.succeed.assert_called_once()
        assert tuple(call[0] for call in robot.method_calls) == (*expected, "move_line")


@pytest.mark.parametrize(
    ("readings", "expected_error"),
    [
        ([(False, True, False, 20.0), (True, False, True, 10.0)], ""),
        ([(True, False, True, 5.0)], "GRIP_ERROR"),
        ([(True, False, True, 7.5)], "GRIP_INDETERMINATE"),
        ([(True, False, False, 30.0)], "GRIP_CHECK_UNAVAILABLE"),
        ([], "GRIP_CHECK_UNAVAILABLE"),
    ],
    ids=["busy-then-closed", "error", "indeterminate", "open-idle", "timeout"],
)
def test_driver_verifies_fresh_rg2_measurement(robot_commands, readings, expected_error):
    """닫히는 중·열린 채 대기 중인 값은 통과시키지 않고 실측과 센서 상태를 함께 본다."""
    from cake_decorator.doosan_robot import DoosanRobotClient

    rg = pytest.importorskip("onrobot_rg_msgs.srv")
    if not hasattr(rg, "GetGripWidth"):
        pytest.skip("GetGripWidth 서비스 빌드가 필요합니다.")
    _, _, driver = robot_commands
    client = Mock()
    client.wait_for_service.return_value = True
    driver._clients = {"grip_width": client}
    driver._logger = Mock()
    responses = [
        rg.GetGripWidth.Response(
            success=success, busy=busy, grip_detected=detected, width_mm=width,
        )
        for success, busy, detected, width in readings
    ]
    driver._wait_for_response.side_effect = [*responses, None]

    assert DoosanRobotClient.verify_grip(driver, "opener") == expected_error
    assert driver._wait_for_response.call_count == (
        len(responses) + (expected_error == "GRIP_CHECK_UNAVAILABLE")
    )
    driver.gripper_close.assert_not_called()
    driver.move_line.assert_not_called()


@pytest.mark.parametrize(
    ("operation", "failing_command", "expected_calls"),
    [
        ("pick", "gripper_open", ("move_line", "gripper_open")),
        (
            "pick",
            "gripper_close",
            ("move_line", "gripper_open", "move_line", "gripper_close"),
        ),
        ("place", "gripper_open", ("move_line", "move_line", "gripper_open")),
    ],
)
def test_driver_helpers_stop_after_gripper_failure(
    robot_commands, operation, failing_command, expected_calls,
):
    """공통 집기·배치 함수도 실패 직후 후속 이동을 보내지 않는다."""
    from cake_decorator.doosan_robot import DoosanRobotClient

    _, _, robot = robot_commands
    getattr(robot, failing_command).return_value = False
    target, approach = Mock(), Mock()

    if operation == "pick":
        result = DoosanRobotClient.pick(robot, target, approach, grip_width=40.0)
    else:
        result = DoosanRobotClient.place(robot, target, approach)

    assert result is False
    assert tuple(call[0] for call in robot.method_calls) == expected_calls


@pytest.mark.parametrize(
    "phase",
    ["PICKING_CAKE", "PLACING_CAKE", "GRASPING_TOOL", "RETURNING_TOOL", "DELIVERING"],
)
def test_handling_failure_does_not_send_home_move(robot_commands, phase):
    """전체 공정 상태 머신에서도 판·펜의 상태가 불명확하면 홈 이동을 보내지 않는다."""
    from cake_decorator.drawing_motion import TaskState

    node_type, node, robot = robot_commands
    node._state = TaskState[phase]

    node_type._fail(node, "테스트용 실패")

    assert node._state == TaskState.FAILED
    assert robot.method_calls == []


@pytest.mark.parametrize("service", ["move_line", "gripper"])
def test_pending_driver_response_is_treated_as_failure(
    robot_commands, monkeypatch, service,
):
    """응답 시간이 지나도 미완료 Future이면 실행기에서 실패로 처리할 값을 반환한다."""
    from cake_decorator.doosan_robot import DoosanRobotClient

    # ROS나 컨트롤러를 사용하지 않고 대기 기간 안에 응답이 없던 상황을 만든다.
    # Future가 미완료라는 사실로 컨트롤러의 동작 여부를 판정할 수는 없다.
    future = Future()
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    driver = Mock(spec=DoosanRobotClient)
    driver._call = DoosanRobotClient._call.__get__(driver)
    driver._wait_for_response = DoosanRobotClient._wait_for_response.__get__(driver)
    driver._clients = {service: client}
    driver._timeout = 5.0
    driver._node = Mock(executor=None)
    driver._logger = Mock()
    monkeypatch.setattr(rclpy, "spin_until_future_complete", Mock())

    if service == "move_line":
        success = DoosanRobotClient._call_checked(driver, service, Mock())
    else:
        success = DoosanRobotClient._gripper_command(driver, "close(40.0,20.0)")

    LOGGER.log(
        logging.INFO if success is False else logging.ERROR,
        "[모의 응답 시간 초과] 서비스=%s 응답완료=%s 성공응답=%s 요청횟수=%d",
        service, future.done(), success, client.call_async.call_count,
    )
    assert success is False
    client.call_async.assert_called_once()


@pytest.mark.parametrize(
    ("filename", "move_count", "pulse_seconds", "moves_after_release"),
    [
        ("move_cakeplate.drl", 28, (0.5, 1.0), 0),
        ("m0609_pengrip.drl", 12, (1.0, 1.0), 2),
    ],
    ids=["plate-transfer-trace", "pen-grip-and-return-trace"],
)
def test_taught_drl_command_sequence(
    filename, move_count, pulse_seconds, moves_after_release,
):
    """원본 DRL의 닫기·열기 신호와 이동 순서를 명령 기록으로 검사한다."""
    trace = load_drl_trace(filename)
    assert sum(is_drl_movement(entry) for entry in trace) == move_count
    assert [entry["command"] for entry in trace[:5]] == [
        "set_singular_handling", "set_velj", "set_accj", "set_velx", "set_accx",
    ]
    # 첫 이동이 관절 안전/대기 자세로 기록됐는지만 확인한다. 실제 시작점에서
    # 해당 자세로 가는 동안의 충돌이나 케이크판에 작용하는 힘은 계산하지 않는다.
    first_move = next(entry for entry in trace if is_drl_movement(entry))
    assert first_move["command"] == "movej"
    close_index, open_index = assert_gripper_pulses(trace, pulse_seconds)

    # 판: 오프너 파지 → 판 지지·스테이션 배치/회수·출고 → 오프너 반납.
    # 펜: 파지 → 케이크 윗면 중심을 터치하는 티칭 동작 → 거치대 반납 → 후퇴.
    # 펜의 실제 드로잉 경로는 위의 ROS 테스트에서 별도로 다룬다.
    assert any(is_drl_movement(entry) for entry in trace[close_index + 1:open_index])
    assert sum(is_drl_movement(entry) for entry in trace[open_index + 1:]) == (
        moves_after_release
    )
    if filename == "move_cakeplate.drl":
        assert_plate_transfer_stages(trace)
        assert_opener_returns_to_pickup(trace, close_index, open_index)


def load_drl_trace(filename):
    """원본이 설정된 경우에만 파일을 읽고 실행 대신 명령 기록을 반환한다."""
    directory = os.environ.get("CAKE_DRL_DIR")
    if directory is None:
        pytest.skip("원본 DRL 두 파일이 있는 디렉터리를 CAKE_DRL_DIR로 지정하세요.")

    path = Path(directory) / filename
    try:
        return replay_drl(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError, SyntaxError):
        # 파서 오류에는 원문 행 번호가 포함된다. 파일 경로도 남기고 원래 예외를 전파한다.
        LOGGER.exception("[DRL 기록 오류] 파일=%s", path)
        raise


def assert_gripper_pulses(trace, pulse_seconds):
    """닫기·열기 의도의 출력 펄스 폭과 종료 시 출력 값을 검사한다.

    DART 주석의 DO1 닫기·DO2 열기는 배선에 의존한다. ON/대기/OFF에는
    파지 감지, 무게 측정, 그리퍼 완료 피드백이 없으며 RG2 ROS 응답과도 별개다.
    반환하는 두 인덱스로 각 펄스 앞뒤의 이동을 검사할 수 있다.
    """
    pulse_indices = [
        index
        for index, entry in enumerate(trace)
        if entry["command"] == "set_digital_output" and entry["args"][1] == "ON"
    ]
    assert [trace[index]["args"][0] for index in pulse_indices] == [1, 2]
    for channel, index, seconds in zip((1, 2), pulse_indices, pulse_seconds):
        wait, off = trace[index + 1:index + 3]
        assert (wait["command"], wait["args"]) == ("wait", [seconds])
        assert (off["command"], off["args"]) == (
            "set_digital_output",
            [channel, "OFF"],
        )

    outputs = dict(
        entry["args"] for entry in trace
        if entry["command"] == "set_digital_output"
    )
    assert outputs == {1: "OFF", 2: "OFF"}
    return pulse_indices


def same_drl_command(first, second):
    """원문 줄 번호를 제외하고 명령, 인수, 옵션이 같은지 비교한다."""
    return all(first[key] == second[key] for key in ("command", "args", "options"))


def assert_plate_transfer_stages(trace):
    """사용자 주석의 안전자세 1~5와 그 사이에 배치된 이송 명령을 확인한다.

    단계 이름은 티칭 의도를 설명한다. 받침판의 걸림·하중 지지·안착·분리,
    로봇과 주변 구조물 간 여유 공간은 명령 기록으로 확인할 수 없다.
    """
    moves = [entry for entry in trace if is_drl_movement(entry)]
    # 안전자세 1~5는 모두 같은 관절 목표다. 이 자세가 반복되는 위치로
    # 오프너 파지·받침판 들어올리기, 스테이션 배치, 회수, 출고, 반납을 나눈다.
    safe_positions = [
        index
        for index, move in enumerate(moves)
        if same_drl_command(moves[0], move)
    ]
    assert safe_positions == [0, 9, 14, 19, 25]
    _, before_placement, before_pickup, before_delivery, before_return = (
        safe_positions
    )

    # 오프너 홀에 틀 끝단을 걸기 위해 J5만 상대 -7°를 명령한다.
    # 손목 각도 변경이므로 TCP 기준 회전량이나 실제 걸림으로 해석하지 않는다.
    wrist_adjustment = moves[before_placement - 3]
    assert wrist_adjustment["command"] == "movej"
    assert wrist_adjustment["options"]["mod"] == "DR_MV_MOD_REL"
    assert wrist_adjustment["args"] == [[0, 0, 0, 0, -7, 0]]

    # 판을 들어 올릴 때와 출고 후 오프너를 들어 올릴 때 모두 BASE Z +80 mm다.
    plate_lift = moves[before_placement - 1]
    opener_clearance = moves[before_return - 1]
    assert plate_lift["command"] == "movel"
    assert plate_lift["options"]["ref"] == 0
    assert plate_lift["options"]["mod"] == "DR_MV_MOD_REL"
    assert plate_lift["args"] == [[0, 0, 80, 0, 0, 0]]
    assert same_drl_command(plate_lift, opener_clearance)

    # 배치 퇴장 시 마지막 두 자세(ps3 → ps4)를 회수 진입 때 역순으로 쓴다.
    # 두 구간 사이에는 안전자세 3으로 이동하는 명령만 있다. 여기서 드로잉
    # 대기나 완료 신호를 확인하지 않고 곧바로 회수 동작으로 넘어간다.
    placement_exit = moves[before_pickup - 2:before_pickup]
    pickup_entry = moves[before_pickup + 1:before_pickup + 3]
    assert all(
        same_drl_command(place, pickup)
        for place, pickup in zip(reversed(placement_exit), pickup_entry)
    )
    exit_index = trace.index(placement_exit[-1])
    pickup_index = trace.index(pickup_entry[0])
    assert trace[exit_index + 1:pickup_index] == [moves[before_pickup]]

    # 출고 구간은 관절 자세로 틀을 분리할 위치를 만들고 마지막에 직선 상승한다.
    delivery = moves[before_delivery + 1:before_return]
    assert [move["command"] for move in delivery] == ["movej"] * 4 + ["movel"]


def assert_opener_returns_to_pickup(trace, close_index, open_index):
    """받침판 작업 뒤 오프너를 집었던 접근점과 하강 명령으로 되돌아오는지 검사한다."""
    approach, descent = trace[close_index - 2:close_index]
    assert approach["options"]["ref"] == descent["options"]["ref"] == 0
    assert approach["options"]["mod"] == "DR_MV_MOD_ABS"
    assert descent["options"]["mod"] == "DR_MV_MOD_REL"
    assert descent["args"] == [[0, 0, -100, 0, 0, 0]]
    # DO1의 ON 상태를 유지하는 0.5초와 신호를 끈 뒤 가만히 있는 0.5초를
    # 구별한다. 두 대기는 닫힘 완료나 오프너 존재를 확인하지 않는다.
    settle, lift = trace[close_index + 3:close_index + 5]
    assert (settle["command"], settle["args"]) == ("wait", [0.5])
    assert same_drl_command(approach, lift)

    # 위치와 옵션을 함께 비교해 반납에도 같은 BASE 절대 접근과 상대 하강이
    # 쓰이는지 확인한다. DO2는 받침판이 아니라 오프너를 놓는 신호다.
    return_approach, return_descent = trace[open_index - 2:open_index]
    assert same_drl_command(approach, return_approach)
    assert same_drl_command(descent, return_descent)
    # DO2 OFF 직후 프로그램이 끝난다. 보관대에서 후퇴하는 명령은 없다.
    assert [entry["command"] for entry in trace[open_index + 3:]] == ["exit"]


def make_drawing(surface, pixel_strokes):
    """선택한 면과 입력 순서를 보존한 픽셀 획을 하나의 DrawingPath로 묶는다."""
    drawing = DrawingPath(surface=surface, canvas_width=1024, canvas_height=768)
    # 화면 픽셀을 그대로 싣는다. BASE 좌표 변환은 경로 계산 노드의 책임이다.
    for stroke_id, pixels in enumerate(pixel_strokes):
        points = [Point32(x=float(x), y=float(y)) for x, y in pixels]
        drawing.strokes.append(Stroke2D(stroke_id=stroke_id, points=points))

    return drawing


def make_order(*drawings, order_id="workflow-test"):
    """그릴 면들의 순서와 케이크 기준점을 담은 CakeOrder Goal을 만든다."""
    order = CakeOrder.Goal(
        order_id=order_id,
        drawing_paths=list(drawings),
        tool_type="pen",
    )
    # 위치는 모의 BASE 기준이고, 치수는 프로젝트의 원뿔대 규격(mm)을
    # 읽어 ROS 메시지의 m로 변환한다. 케이크 평면 yaw와 펜 자세는 별개다.
    config_path = Path(__file__).resolve().parents[4] / "config" / "test_cake.json"
    cake = json.loads(config_path.read_text(encoding="utf-8"))
    order.cake_params.center.x = 0.4
    order.cake_params.center.z = 0.15
    order.cake_params.radius = cake["top_diameter_mm"] / 2000.0
    order.cake_params.bottom_radius = cake["bottom_diameter_mm"] / 2000.0
    order.cake_params.height = cake["height_mm"] / 1000.0
    order.cake_params.reference_frame = "base"
    order.cake_params.orientation = 0.1
    order.cake_params.tool_orientation.z = math.sin(0.15)
    order.cake_params.tool_orientation.w = math.cos(0.15)
    return order


class RecordingDrawingPath(DrawingPathNode):
    """실제 경로 계산 노드에 서비스 요청·응답 기록만 추가한다."""

    def __init__(self):
        """서비스가 시작되기 전에 요청·응답 목록을 초기화한다."""
        self.calls = []
        super().__init__()

    def _handle_request(self, request, response):
        """실제 변환 결과를 반환하고 테스트가 전달 내용을 검사하도록 기록한다."""
        response = super()._handle_request(request, response)
        self.calls.append((request, response))
        if not response.success:
            LOGGER.info(
                "[경로 생성 실패] 주문=%s 면=%s 원인=%s",
                request.order_id, request.surface_type, response.error_msg,
            )
        return response


class FakeDrawServer:
    """받은 DrawOnCake Goal을 기록하고 테스트가 지정한 성공·실패를 돌려준다."""

    def __init__(self, node):
        """테스트 노드에 드로잉 서버를 등록하고 기본 응답을 성공으로 둔다."""
        self.goals = []
        self.should_succeed = True
        self.server = ActionServer(node, DrawOnCake, "draw_on_cake", self.execute)

    def execute(self, handle):
        """드로잉 요청을 저장하고 액션 상태와 결과 메시지를 함께 종료한다."""
        self.goals.append(handle.request)
        if self.should_succeed:
            handle.succeed()
            return DrawOnCake.Result(success=True)

        error_code = "TEST_DRAW_FAILURE"
        LOGGER.info(
            "[모의 드로잉 실패] 면=%s 오류코드=%s",
            handle.request.drawing_surface, error_code,
        )
        handle.abort()
        return DrawOnCake.Result(success=False, error_code=error_code)


@dataclass
class Workflow:
    """테스트에서 주문을 제출하고 두 하위 노드의 호출 기록을 읽는 데 사용한다."""

    order_client: ActionClient
    path: RecordingDrawingPath
    draw: FakeDrawServer

    def submit_order(self, order, *, expected_error=None):
        """주문 수락과 완료를 기다리고 예상과 다른 최종 결과는 오류로 기록한다."""
        order_id = order.order_id
        previous_paths = len(self.path.calls)
        previous_draws = len(self.draw.goals)
        assert self.order_client.wait_for_server(timeout_sec=5.0), (
            f"주문={order_id}: CakeOrder 서버 준비 시간 초과"
        )
        accepted = self.order_client.send_goal_async(order)
        assert wait_for_future(accepted, timeout_sec=5.0), (
            f"주문={order_id}: Goal 수락 응답 시간 초과"
        )

        handle = accepted.result()
        assert handle.accepted, f"주문={order_id}: CakeOrder Goal 거절"
        finished = handle.get_result_async()
        assert wait_for_future(finished, timeout_sec=8.0), (
            f"주문={order_id}: 최종 결과 응답 시간 초과"
        )
        outcome = finished.result()
        if (
            expected_error is not None
            or not outcome.result.success
            or outcome.status != GoalStatus.STATUS_SUCCEEDED
        ):
            # 실제 경로 계산/드로잉 요청이 어디까지 진행됐는지 주문 결과와 같이 남긴다.
            is_expected = (
                expected_error is not None
                and outcome.status == GoalStatus.STATUS_ABORTED
                and not outcome.result.success
                and outcome.result.error_code == expected_error
            )
            LOGGER.log(
                logging.INFO if is_expected else logging.ERROR,
                "[주문 결과] 주문=%s ROS상태=%s 성공응답=%s 요청면=%s "
                "경로요청=%s 드로잉요청=%s 기대오류=%r 응답오류=%r 설명=%r",
                order_id, outcome.status, outcome.result.success,
                [drawing.surface for drawing in order.drawing_paths],
                [request.surface_type for request, _ in self.path.calls[previous_paths:]],
                [goal.drawing_surface for goal in self.draw.goals[previous_draws:]],
                expected_error, outcome.result.error_code, outcome.result.error_msg,
            )
        return outcome


def wait_for_future(future, timeout_sec):
    """ROS 콜백은 executor에 맡기고 Future 완료 신호를 제한 시간만큼 기다린다."""
    done = Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout=timeout_sec)


@pytest.fixture
def workflow(monkeypatch, tmp_path):
    """테스트마다 ROS 노드를 격리해 연결하고 종료할 때 자원을 정리한다."""
    # 매번 이름 공간을 새로 만들고 절대 이름인 상태 토픽도 안쪽으로 옮긴다.
    # 실제 RobotExecutorNode, DSR 컨트롤러와 RG2 서버는 생성하지 않는다.
    ros_args = [
        "--ros-args",
        "-r", f"__ns:=/workflow_{uuid4().hex}",
        "-r", "/task_status:=task_status",
        "-r", "/cake/order:=cake_order",
    ]
    monkeypatch.delenv("ROS_DISCOVERY_SERVER", raising=False)
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "OFF")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros"))

    # 작업 관리자는 자체 완료 신호로 응답을 기다린다. 테스트에서도
    # 운영 대기 로직을 그대로 두고 다중 스레드 실행기로 응답을 처리한다.
    with ExitStack() as cleanup:
        rclpy.init(args=ros_args)
        cleanup.callback(rclpy.try_shutdown)
        manager = TaskManagerNode()
        cleanup.callback(manager.destroy_node)
        manager.set_parameters([
            Parameter("service_timeout", value=3.0),
            Parameter("action_timeout", value=3.0),
        ])
        path = RecordingDrawingPath()
        cleanup.callback(path.destroy_node)
        test_node = Node("workflow_test")
        cleanup.callback(test_node.destroy_node)

        draw = FakeDrawServer(test_node)
        cleanup.callback(draw.server.destroy)
        # 운영 주소 /cake/order는 위에서 테스트 전용 이름 공간으로 옮겼다.
        order_client = ActionClient(test_node, CakeOrder, "cake_order")
        cleanup.callback(order_client.destroy)

        executor = MultiThreadedExecutor(num_threads=4)
        for node in (manager, path, test_node):
            executor.add_node(node)
        thread = Thread(target=executor.spin, daemon=True)
        thread.start()
        # ExitStack은 역순 정리하므로 노드 삭제 전에 콜백과 스레드를 멈춘다.
        cleanup.callback(thread.join)
        cleanup.callback(executor.shutdown)
        yield Workflow(order_client, path, draw)


# 초기화 명령은 특이점 처리와 기본 속도·가속도를 지정한다. 특이점 회피와
# 무블렌딩(radius=0)은 충돌 검사가 아니다. '안전자세'는 티칭 명칭이다.
# movej/posj는 J1~J6 각도(°), movel/posx는 TCP XYZ(mm)와 ZYZ 각도(°)다.
# ref=0은 BASE, mod는 절대 목표 또는 증분을 뜻한다. 상대 BASE Z 이동은
# 도구 축 이동이 아니며, 선행 자세와 활성 TCP 없이 실제 위치를 확정할 수 없다.
# 아래 파서는 이 값을 변환하거나 제어기에 보내지 않고 텍스트 계약만 검사한다.
SETUP_COMMANDS = {
    "set_singular_handling", "set_velj", "set_accj", "set_velx", "set_accx",
}
DRL_MOTION_COMMANDS = {"movej", "movel"}
DRL_COMMANDS = SETUP_COMMANDS | DRL_MOTION_COMMANDS | {
    "set_digital_output", "wait", "exit",
}
DRL_CONSTANTS = {
    "ON", "OFF", "DR_OFF", "DR_AVOID", "DR_MV_RA_DUPLICATE",
    "DR_MV_MOD_ABS", "DR_MV_MOD_REL", "DR_MV_APP_NONE",
}


def is_drl_movement(entry):
    """기록 한 건이 관절 이동 또는 직선 이동이면 True를 반환한다."""
    return entry["command"] in DRL_MOTION_COMMANDS


def read_drl_argument(node):
    """지원하는 숫자·DRL 상수·posj/posx 좌표를 기록용 값으로 바꾼다."""
    if isinstance(node, ast.Constant) and type(node.value) in (int, float):
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        value = read_drl_argument(node.operand)
        if type(value) in (int, float):
            return -value
    if isinstance(node, ast.Name) and node.id in DRL_CONSTANTS:
        return node.id
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
        if node.func.id in ("posj", "posx") and len(node.args) == 6 and not node.keywords:
            axes = [read_drl_argument(axis) for axis in node.args]
            if all(type(axis) in (int, float) for axis in axes):
                return axes
    raise ValueError(f"{node.lineno}행: 지원하지 않는 DRL 인수")


def read_single_iteration_body(loop, counter):
    """DART의 1회 반복문에서 카운터 증가를 제외한 명령만 제자리에 남긴다."""
    if loop.orelse or ast.unparse(loop.test) != f"{counter} < 1":
        raise ValueError(f"{loop.lineno}행: 지원하지 않는 DRL 반복 조건")

    commands = []
    increment_count = 0
    for statement in loop.body:
        if isinstance(statement, ast.Expr):
            commands.append(statement)
        elif (
            isinstance(statement, ast.Assign)
            and ast.unparse(statement) == f"{counter} = {counter} + 1"
        ):
            increment_count += 1
        else:
            raise ValueError(f"{statement.lineno}행: 지원하지 않는 DRL 반복문 본문")
    if increment_count != 1:
        raise ValueError(f"{loop.lineno}행: DRL 반복문에 카운터 증가가 한 번 필요함")
    # 펜 파일은 증가문 뒤에 출력 초기화가 있다. 판 파일은 exit에서 기록을 끝낸다.
    return commands


def collect_drl_statements(source):
    """최상위 설정과 DART 1회 반복문에서 실행 순서대로 표현식을 모은다."""
    commands = []
    counter = None
    for statement in ast.parse(source).body:
        if isinstance(statement, ast.Expr):
            commands.append(statement)
            continue

        if isinstance(statement, ast.Assign) and len(statement.targets) == 1:
            target = statement.targets[0]
            if (
                isinstance(target, ast.Name)
                and target.id.startswith("gLoop")
                and ast.unparse(statement.value) == "0"
                and counter is None
            ):
                counter = target.id
                continue

        if isinstance(statement, ast.While) and counter:
            commands.extend(read_single_iteration_body(statement, counter))
            counter = None
            continue

        raise ValueError(f"{statement.lineno}행: 지원하지 않는 DRL 구문")

    return commands


def replay_drl(source):
    """원본 명령을 장비 호출 없이 실행 순서대로 기록한다."""
    trace = []
    for statement in collect_drl_statements(source):
        call = statement.value
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
            raise ValueError(f"{statement.lineno}행: DRL 명령이 필요함")
        command = call.func.id
        if command not in DRL_COMMANDS or any(arg.arg is None for arg in call.keywords):
            raise ValueError(f"{statement.lineno}행: 지원하지 않는 DRL 명령 {command}")
        args = [read_drl_argument(arg) for arg in call.args]
        options = {arg.arg: read_drl_argument(arg.value) for arg in call.keywords}
        trace.append({
            "line": statement.lineno,
            "command": command,
            "args": args,
            "options": options,
        })
        if command == "exit":
            break
    return trace


# 도면의 티칭 순서에 따른 기대 하중이다. 3·5에서는 판을 이미 내려놓았다.
PLATE_CHECKPOINTS = {
    1: "EMPTY",
    2: "OPENER_AND_PLATE",
    3: "OPENER_ONLY",
    4: "OPENER_AND_PLATE",
    5: "OPENER_ONLY",
}


PLATE_WEIGHT_TOLERANCE_G = 100.0


class HandlingGuard:
    """모의 이송에서 측정 이상을 기록하고 해당 작업의 진행 허용을 잠근다.

    호출자는 안전자세 도달 후 안정된 무게(g), 또는 그리퍼 닫힘 완료 후
    측정한 턱 간격(mm)을 넘긴다. RG2 드라이버에는 너비와 각도 환산이
    있으므로 연동 시 실제 측정 간격을 우선 사용한다. 닫힘 명령의 목표 너비나
    각도를 실제 측정값으로 사용하지 않는다. 센서 오류·시간 초과·오래된
    표본은 None을 전달한다.
    자세별 기준 무게는 실기 기본값 없이 외부에서 주입한다. False이면
    호출자는 다음 이동·그리퍼 명령을 보내지 않는다.
    실기 정지 인터페이스는 미정이며 콜백은 모의 테스트에서만 연결한다.
    """

    def __init__(self, expected_weights_g, request_stop=None):
        """자세별 정상 하중(g)과 정지 요청 콜백을 받는다."""
        self._expected_weights_g = dict(expected_weights_g)
        if any(
            not math.isfinite(weight) or weight < 0
            for weight in self._expected_weights_g.values()
        ):
            raise ValueError("기준 무게는 음이 아닌 유한값(g)이어야 합니다.")
        self._request_stop = request_stop
        self._failed = False

    def check_plate_weight(self, checkpoint, measured_g):
        """안전자세 1~5의 측정값을 기준 무게(g)와 비교한다. ±100g은 허용한다."""
        if self._failed:
            return False
        load_state = PLATE_CHECKPOINTS.get(checkpoint)
        expected_g = self._expected_weights_g.get(checkpoint)
        context = f"안전자세={checkpoint} 기대하중={load_state} 측정={measured_g!r} g"
        if (
            load_state is None or expected_g is None or measured_g is None
            or not math.isfinite(measured_g) or measured_g < 0
        ):
            return self._stop("WEIGHT_CHECK_UNAVAILABLE", context)

        delta_g = measured_g - expected_g
        if abs(delta_g) <= PLATE_WEIGHT_TOLERANCE_G:
            return True
        # 정상 배치·출고 후 하중 감소는 다음 체크포인트 기준에 반영한다.
        # 판을 들고 있어야 할 때 기준보다 가벼우면 낙하 또는 미파지를 의심한다.
        code = "PLATE_WEIGHT_OUT_OF_RANGE"
        if load_state == "OPENER_AND_PLATE" and delta_g < 0:
            code = "CAKE_PLATE_DROP_SUSPECTED"
            context = f"케이크판 떨어짐 의심. {context}"
        return self._stop(
            code,
            f"{context} 기준={expected_g} g 차이={delta_g:.3f} g "
            f"허용편차={PLATE_WEIGHT_TOLERANCE_G:g} g",
        )

    def check_grip_width(self, tool, measured_mm):
        """오프너·펜을 닫은 뒤 측정한 턱 간격으로 들기 허용 여부를 결정한다."""
        if self._failed:
            return False
        context = f"대상={tool} 닫힘후간격={measured_mm!r} mm"
        code = grip_width_error(measured_mm)
        if not code:
            return True
        labels = {"GRIP_ERROR": "그립 오류", "GRIP_INDETERMINATE": "그립 판정 보류"}
        return self._stop(code, f"{labels.get(code, '그립 측정 불가')}. {context}")

    def _stop(self, code, details):
        """다음 명령을 먼저 차단하고 정지 요청 결과를 오류 원인과 함께 기록한다."""
        self._failed = True
        stop_result = "미연결"
        if self._request_stop is not None:
            stop_result = "미확인"
            try:
                if self._request_stop() is True:
                    stop_result = "수락"
            except Exception:
                LOGGER.exception("[정지 요청 예외] 코드=%s", code)
        LOGGER.error(
            "[이송·파지 중단] 코드=%s %s 정지요청응답=%s 후속명령=차단",
            code, details, stop_result,
        )
        return False


@pytest.fixture
def handling_guard():
    """경계와 실패를 재현할 합성 기준 무게(g)를 사용한다."""
    expected_weights_g = {1: 0.0, 2: 1500.0, 3: 500.0, 4: 1500.0, 5: 500.0}
    robot = Mock()
    robot.request_stop.return_value = True
    return HandlingGuard(expected_weights_g, robot.request_stop), robot


@pytest.mark.parametrize("measured_g", [1400.0, 1600.0])
def test_plate_weight_allows_exactly_100g_deviation(handling_guard, measured_g):
    """양쪽 허용 한계인 -100g·+100g에서는 다음 명령을 허용한다."""
    guard, robot = handling_guard
    assert guard.check_plate_weight(2, measured_g)
    robot.request_stop.assert_not_called()


@pytest.mark.parametrize(
    ("fall_at", "expected_code"),
    [(None, None), (2, "CAKE_PLATE_DROP_SUSPECTED"), (4, "CAKE_PLATE_DROP_SUSPECTED")],
)
def test_plate_weight_is_checked_at_each_safe_position(
    handling_guard, caplog, fall_at, expected_code,
):
    """원본 이송 명령을 기록하며 낙하 시 다음 명령을 기록하지 않는지 확인한다."""
    guard, robot = handling_guard
    trace = load_drl_trace("move_cakeplate.drl")
    first_move = next(entry for entry in trace if is_drl_movement(entry))
    measurements = {1: 0.0, 2: 1500.0, 3: 500.0, 4: 1500.0, 5: 500.0}
    checked = []
    for command in trace:
        robot.record_command(command)
        if same_drl_command(command, first_move):
            checkpoint = len(checked) + 1
            checked.append(checkpoint)
            weight = 1399.0 if checkpoint == fall_at else measurements[checkpoint]
            if not guard.check_plate_weight(checkpoint, weight):
                break

    if fall_at is None:
        assert checked == list(PLATE_CHECKPOINTS)
        robot.request_stop.assert_not_called()
        assert robot.record_command.call_count == len(trace)
    else:
        assert checked == list(range(1, fall_at + 1))
        assert robot.method_calls[-1][0] == "request_stop"
        assert robot.record_command.call_args.args[0] == command
        robot.request_stop.assert_called_once()
        assert expected_code in caplog.text
        assert "케이크판 떨어짐 의심" in caplog.text


@pytest.mark.parametrize(
    ("filename", "tool"),
    [("move_cakeplate.drl", "opener"), ("m0609_pengrip.drl", "pen")],
)
@pytest.mark.parametrize(
    ("width_mm", "expected_code"),
    [
        (10.0, None),
        (5.0, "GRIP_ERROR"),
        (7.5, "GRIP_INDETERMINATE"),
        (None, "GRIP_CHECK_UNAVAILABLE"),
    ],
)
def test_grip_width_blocks_lift_after_closing(
    handling_guard, caplog, monkeypatch, filename, tool, width_mm, expected_code,
):
    """닫기 후 모의 RG2 측정 응답을 판정에 전달하고 다음 이동을 제어한다."""
    rg_services = pytest.importorskip("onrobot_rg_msgs.srv")
    if not hasattr(rg_services, "GetGripWidth"):
        pytest.skip("GetGripWidth 인터페이스를 포함한 onrobot_rg_msgs 빌드가 필요합니다.")
    guard, robot = handling_guard
    node, executor = Mock(), Mock()
    read = Mock(return_value=width_mm)
    if width_mm is None:
        read.side_effect = RuntimeError("RG2가 아직 이동 중입니다.")
    monkeypatch.setitem(check_measured_grip.__globals__, "read_controller_value", read)
    trace = load_drl_trace(filename)
    close_index = next(
        index
        for index, entry in enumerate(trace)
        if entry["command"] == "set_digital_output" and entry["args"] == [1, "ON"]
    )
    # 두 원본 모두 닫기 ON → 대기 → OFF → 대기 뒤에 들기 명령을 보낸다.
    check_index = close_index + 3
    assert trace[check_index]["command"] == "wait"
    assert is_drl_movement(trace[check_index + 1])
    for index, command in enumerate(trace):
        robot.record_command(command)
        if index == check_index and not check_measured_grip(node, executor, tool, 0.2, guard):
            break

    read.assert_called_once_with(
        node, executor, "/onrobot/get_grip_width", rg_services.GetGripWidth, "width_mm", 0.2,
    )
    if expected_code is None:
        robot.request_stop.assert_not_called()
        assert robot.record_command.call_count == len(trace)
    else:
        robot.request_stop.assert_called_once()
        assert robot.method_calls[-1][0] == "request_stop"
        assert robot.record_command.call_count == check_index + 1
        assert expected_code in caplog.text


def test_weight_above_reference_stops_without_claiming_drop(handling_guard, caplog):
    """배치 후 예상보다 무거우면 정지하되 무게만으로 안착 위치를 판정하지 않는다."""
    guard, robot = handling_guard

    assert not guard.check_plate_weight(3, 601.0)
    robot.request_stop.assert_called_once()
    assert "PLATE_WEIGHT_OUT_OF_RANGE" in caplog.text
    assert "CAKE_PLATE_DROP_SUSPECTED" not in caplog.text


@pytest.mark.parametrize("stop_raises", [False, True])
def test_unavailable_weight_latches_failure_even_without_stop_ack(
    handling_guard, caplog, stop_raises,
):
    """비정상 센서값과 정지 응답 실패가 겹쳐도 다음 검사에서 작업을 재개하지 않는다."""
    guard, robot = handling_guard
    if stop_raises:
        robot.request_stop.side_effect = TimeoutError("모의 정지 응답 시간 초과")
    else:
        robot.request_stop.return_value = False

    assert not guard.check_plate_weight(2, float("nan"))
    assert not guard.check_plate_weight(2, 1500.0)
    assert not guard.check_grip_width("pen", 10.0)
    robot.request_stop.assert_called_once()
    assert "WEIGHT_CHECK_UNAVAILABLE" in caplog.text
    assert "정지요청응답=미확인 후속명령=차단" in caplog.text


# 아래는 작업자가 직접 선택하는 읽기 전용 점검이다. 위의 pytest 경로에서는
# 호출되지 않으며, ROS 도메인이나 탐색 범위를 바꾸거나 드라이버를 시작하지 않는다.
# 숫자는 저장소의 dsr_msgs2/srv/GetRobotState.srv 정의를 따른다.
CONTROLLER_STATES = {
    0: "INITIALIZING",
    1: "STANDBY",
    2: "MOVING",
    3: "SAFE_OFF",
    4: "TEACHING",
    5: "SAFE_STOP",
    6: "EMERGENCY_STOP",
    7: "HOMING",
    8: "RECOVERY",
    9: "SAFE_STOP2",
    10: "SAFE_OFF2",
    15: "NOT_READY",
}
CONTROLLER_SYSTEMS = {0: "REAL", 1: "VIRTUAL"}
CONTROLLER_MODES = {0: "MANUAL", 1: "AUTONOMOUS", 2: "MEASURE"}
RG_COMMAND_SERVICE_TYPE = "onrobot_rg_msgs/srv/SetCommand"


def read_controller_value(node, executor, service_name, service_type, field, timeout_sec):
    """조회 서비스의 한 응답 필드를 읽고 실패 사유를 포함한 예외를 반환 경로로 전달한다."""
    client = node.create_client(service_type, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"{service_name}: 서비스 탐색 시간 초과")
        future = client.call_async(service_type.Request())
        rclpy.spin_until_future_complete(
            node, future, executor=executor, timeout_sec=timeout_sec,
        )
        if not future.done():
            raise RuntimeError(f"{service_name}: 응답 시간 초과")
        response = future.result()
        if response is None or not response.success:
            reason = getattr(response, "message", "") or "조회 실패 응답"
            raise RuntimeError(f"{service_name}: {reason}")
        return getattr(response, field)
    finally:
        node.destroy_client(client)


def check_measured_grip(node, executor, tool, timeout_sec, guard):
    """닫힘 확인 후 RG2 실측 간격을 읽어 기존 그립 판정에 전달한다.

    이동 중, 조회 실패, 미연결은 측정 불가로 처리한다. 이 조회와 판정은
    어떤 그리퍼 명령도 보내지 않으며, 실기 호출자는 닫힘 완료를 먼저 확인한다.
    """
    from onrobot_rg_msgs.srv import GetGripWidth

    try:
        width_mm = read_controller_value(
            node, executor, "/onrobot/get_grip_width",
            GetGripWidth, "width_mm", timeout_sec,
        )
    except Exception as exc:
        LOGGER.error("[RG2 간격 조회 실패] 대상=%s 원인=%s", tool, exc)
        width_mm = None

    accepted = guard.check_grip_width(tool, width_mm)
    if accepted:
        LOGGER.info("[RG2 그립 기준 통과] 대상=%s 실측간격=%.1f mm", tool, width_mm)
    return accepted


def read_workpiece_weight_g(node, executor, controller, timeout_sec):
    """기존 Doosan 조회 서비스의 kg 응답을 g로 바꾼다. 오류 응답은 예외로 전달한다."""
    from dsr_msgs2.srv import GetWorkpieceWeight

    weight_kg = read_controller_value(
        node, executor, f"{controller}/force/get_workpiece_weight",
        GetWorkpieceWeight, "weight", timeout_sec,
    )
    # 드라이버가 success를 반환해도 음수는 Doosan의 측정 오류값이다.
    weight_g = weight_kg * 1000.0
    if not math.isfinite(weight_g) or weight_kg < 0:
        raise RuntimeError(f"유효하지 않은 Doosan 무게 응답: {weight_kg!r} kg")
    return weight_g


@pytest.mark.parametrize(
    ("weight_kg", "expected_g"), [(0.0, 0.0), (0.125, 125.0), (-1.0, None)],
)
def test_controller_weight_converts_kg_and_rejects_error(
    monkeypatch, weight_kg, expected_g,
):
    """기존 Doosan 서비스 계약과 kg→g 변환을 모의 응답으로 검증한다."""
    dsr_services = pytest.importorskip("dsr_msgs2.srv")

    read = Mock(return_value=weight_kg)
    monkeypatch.setitem(read_workpiece_weight_g.__globals__, "read_controller_value", read)
    node, executor = Mock(), Mock()
    if expected_g is None:
        with pytest.raises(RuntimeError, match="무게 응답"):
            read_workpiece_weight_g(node, executor, "/mock_controller", 0.2)
    else:
        assert read_workpiece_weight_g(node, executor, "/mock_controller", 0.2) == expected_g
    read.assert_called_once_with(
        node, executor, "/mock_controller/force/get_workpiece_weight",
        dsr_services.GetWorkpieceWeight, "weight", 0.2,
    )


def inspect_workcell(
    node, executor, controller, gripper_service, timeout_sec, *, read_weight=False,
):
    """Doosan 조회와 RG2의 ROS 등록 상태를 기록한다. 점검 기준 미충족 시 False다."""
    from dsr_msgs2.srv import GetCurrentPosj, GetRobotMode, GetRobotState, GetRobotSystem

    # Get 계열 서비스만 요청한다. 코드 경로에 MoveLine, SetCommand 또는
    # 액션 Goal 전송을 포함하지 않아 점검에 실패해도 로봇 동작을 요청하지 않는다.
    checks = (
        ("system/get_robot_system", GetRobotSystem, "robot_system"),
        ("system/get_robot_state", GetRobotState, "robot_state"),
        ("system/get_robot_mode", GetRobotMode, "robot_mode"),
        ("aux_control/get_current_posj", GetCurrentPosj, "pos"),
    )
    readings = {}
    complete = True
    for relative_name, service_type, field in checks:
        service_name = f"{controller}/{relative_name}"
        try:
            readings[field] = read_controller_value(
                node, executor, service_name, service_type, field, timeout_sec,
            )
        except Exception as exc:
            # 한 조회가 실패해도 나머지 항목을 읽어 연결 환경을 진단한다.
            LOGGER.error("[컨트롤러 조회 실패] 서비스=%s 원인=%s", service_name, exc)
            complete = False

    system = readings.get("robot_system")
    state = readings.get("robot_state")
    mode = readings.get("robot_mode")
    joints = readings.get("pos")
    LOGGER.info(
        "[컨트롤러 상태] 시스템=%s(%s) 상태=%s(%s) 운전모드=%s(%s)",
        CONTROLLER_SYSTEMS.get(system, "미확인"), system,
        CONTROLLER_STATES.get(state, "미확인"), state,
        CONTROLLER_MODES.get(mode, "미확인"), mode,
    )
    if joints is not None and len(joints) == 6 and all(map(math.isfinite, joints)):
        LOGGER.info("[현재 관절각] J1~J6(°)=%s", list(joints))
    else:
        LOGGER.error("[현재 관절각] 유효한 6축 응답이 없습니다.")
        complete = False

    # 서비스 이름과 타입은 ROS 그래프에서 확인한다. RG2가 명령을 처리하는지,
    # 물체를 잡았는지, 실제 장비인지 판정할 수 있는 관측은 아니다.
    services = dict(node.get_service_names_and_types())
    gripper_visible = RG_COMMAND_SERVICE_TYPE in services.get(gripper_service, ())
    if gripper_visible:
        LOGGER.info("[그리퍼] ROS 서비스 등록 확인: %s", gripper_service)
    else:
        LOGGER.error("[그리퍼] ROS 서비스를 찾지 못함: %s", gripper_service)

    if system != 0 or state != 1 or mode not in CONTROLLER_MODES:
        LOGGER.error("[점검 기준 미충족] REAL·STANDBY·확인 가능한 운전 모드가 필요합니다.")
        complete = False
    if read_weight:
        # 명시적으로 요청한 스냅샷에서만 읽는다. 이 값으로 기준을 자동 설정하거나
        # 중량 센서를 초기화하지 않으며, 장비 상태도 변경하지 않는다.
        if system != 0 or state != 1:
            LOGGER.error("[작업물 무게] REAL·STANDBY 확인 전에는 조회하지 않습니다.")
            complete = False
        else:
            try:
                weight_g = read_workpiece_weight_g(node, executor, controller, timeout_sec)
                LOGGER.info("[작업물 무게] 측정=%.3f g (단일 스냅샷)", weight_g)
            except Exception as exc:
                LOGGER.error("[작업물 무게 조회 실패] 원인=%s", exc)
                complete = False
    return complete and gripper_visible


def main():
    """명시한 경우에만 기존 ROS 그래프를 조회하며 결과를 종료 코드로 반환한다."""
    parser = argparse.ArgumentParser(description="Doosan/RG2 작업셀 읽기 전용 점검")
    parser.add_argument(
        "--check-workcell", action="store_true", required=True,
        help="기존 ROS 서비스의 연결과 컨트롤러 상태 조회",
    )
    parser.add_argument("--controller", default="/dsr01/dsr_controller2")
    parser.add_argument("--gripper-service", default="/onrobot/sendCommand")
    parser.add_argument(
        "--read-workpiece-weight", action="store_true",
        help="REAL·STANDBY 상태에서 기존 Doosan 무게 서비스의 응답을 g로 한 번 조회",
    )
    parser.add_argument(
        "--check-grip", choices=("opener", "pen"),
        help="닫힘 완료를 직접 확인한 RG2에서 실측 턱 간격을 읽고 그립 판정",
    )
    parser.add_argument(
        "--timeout", type=float, default=2.0,
        help="서비스 탐색과 응답을 각각 기다리는 최대 시간(초)",
    )
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout은 0보다 큰 유한한 수여야 합니다.")
    controller = args.controller.rstrip("/")
    if not controller.startswith("/") or not args.gripper_service.startswith("/"):
        parser.error("컨트롤러와 그리퍼 서비스에는 절대 ROS 이름을 사용하세요.")

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    context = Context()
    try:
        with ExitStack() as cleanup:
            rclpy.init(args=[], context=context)
            cleanup.callback(rclpy.try_shutdown, context=context)
            node = Node(f"workcell_check_{uuid4().hex[:8]}", context=context)
            cleanup.callback(node.destroy_node)
            executor = SingleThreadedExecutor(context=context)
            cleanup.callback(executor.shutdown)
            LOGGER.info(
                "[읽기 전용 점검] ROS 도메인=%d 컨트롤러=%s",
                context.get_domain_id(), controller,
            )
            passed = inspect_workcell(
                node, executor, controller, args.gripper_service, args.timeout,
                read_weight=args.read_workpiece_weight,
            )
            if args.check_grip:
                if passed:
                    # 작업자가 이미 닫힌 상태에서 요청하는 단발 조회다.
                    # 정지 콜백 없이 판정 결과만 로그와 종료 코드로 반환한다.
                    passed = check_measured_grip(
                        node, executor, args.check_grip, args.timeout, HandlingGuard({}),
                    )
                else:
                    LOGGER.error("[RG2 그립 점검 보류] 작업셀 연결·상태 기준 미충족")
    except KeyboardInterrupt:
        LOGGER.warning("작업셀 점검을 취소했습니다.")
        return 130
    except Exception:
        LOGGER.exception("작업셀 점검 중 오류가 발생했습니다.")
        return 1

    LOGGER.info(
        "연결·상태 점검=%s. 이 결과는 작업 승인이나 비상정지 검증과 별개입니다.",
        "통과" if passed else "미충족",
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
