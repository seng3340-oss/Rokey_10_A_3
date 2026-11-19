"""실물 연결 없이 통합 펜 순환의 좌표 기준과 실패 정리를 검증한다."""

import importlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml


def rack_parameters():
    """현재 저장소의 팀 티칭값을 읽어 외부 시험 폴더 의존을 막는다."""
    source = Path(__file__).parents[1] / "config/workspace_config.yaml"
    return yaml.safe_load(source.read_text())["robot_executor_node"]["ros__parameters"]


def test_executor_conflict_is_resolved():
    """작업 파일은 ROS 실행 전에도 유효한 파이썬이어야 한다."""
    source = Path(__file__).parents[1] / "cake_decorator/robot_executor_node.py"
    compile(source.read_text(), str(source), "exec")


def test_pick_geometry_preserves_220mm_pivot():
    """플랜지 티칭을 220mm TCP로 바꿔 회전 중 파지 중심을 고정한다."""
    helper = importlib.import_module("cake_decorator.integrated_pen_cycle")
    rack = rack_parameters()
    geometry = helper.pick_geometry(rack)
    stand, tilted, upper, upright = geometry.flange_poses
    converted = [
        helper.flange_to_tcp(pose, helper.GRIP_CENTER_MM)
        for pose in geometry.flange_poses
    ]
    np.testing.assert_allclose(converted[0][:3], converted[1][:3], atol=1e-9)
    np.testing.assert_allclose(converted[2][:3], converted[3][:3], atol=1e-9)
    np.testing.assert_allclose(stand, rack["pen_stand_pose"])
    clearance = rack["pen_gripper_pose"][2] - stand[2]
    assert upper[2] - tilted[2] == pytest.approx(clearance)
    np.testing.assert_allclose(
        Rotation.from_euler("ZYZ", tilted[3:], degrees=True).apply([0, 0, 1]),
        [0, np.sqrt(0.5), -np.sqrt(0.5)], atol=1e-9,
    )
    np.testing.assert_allclose(upright[:3], np.array(stand[:3]) + [0, 0, clearance])


class ServiceRobot:
    """TCP 변환을 계산하고 명령은 이벤트로 기록하는 로봇 대역이다."""

    def __init__(self):
        self.tcp = ""
        self.mode = 1
        self.offsets = {"": [0.] * 6, "Pen_Side_45": [1.095, -46.474, 271.695, 0., 0., 0.]}
        self.flange = [400., -55., 700., 0., 180., 0.]
        self.joint_values = [0., 20., 70., 0., 90., 12.]
        self.events = []
        self.fail = None
        self._clients = {}
        self._node = SimpleNamespace(create_client=lambda *args, **kwargs: object())
        self._service_group = object()
        self._logger = Mock()
        self.gripper_output_fault = False

    def _call(self, key, request, **kwargs):
        """서비스 요청만 해석하며 네트워크나 ROS executor를 사용하지 않는다."""
        self.events.append((key, request))
        if key == self.fail:
            return SimpleNamespace(success=False)
        result = SimpleNamespace(success=True)
        if key == "get_tcp":
            result.info = self.tcp
        elif key == "get_mode":
            result.robot_mode = self.mode
        elif key == "set_mode":
            self.mode = request.robot_mode
        elif key == "set_tcp":
            assert self.mode == 0
            assert request.name in self.offsets
            self.tcp = request.name
        elif key == "create_tcp":
            self.offsets[request.name] = list(request.pos)
        elif key == "delete_tcp":
            assert request.name != self.tcp
            del self.offsets[request.name]
        elif key == "current_flange":
            result.pos = self.flange.copy()
        elif key == "current_pose":
            from cake_decorator.integrated_pen_cycle import flange_to_tcp
            offset = self.offsets[self.tcp]
            pose = flange_to_tcp(self.flange, offset[:3])
            if any(offset[3:]):
                from cake_decorator.integrated_pen_cycle import zyz
                pose[3:] = zyz(
                    Rotation.from_euler("ZYZ", self.flange[3:], degrees=True)
                    * Rotation.from_euler("ZYZ", offset[3:], degrees=True)
                )
            result.task_pos_info = [SimpleNamespace(data=pose)]
        elif key == "current_joints":
            result.pos = self.joint_values.copy()
        elif key == "check_motion":
            result.status = 0
        return result

    def move_line(self, pose, **kwargs):
        """활성 TCP 목표를 플랜지 좌표로 역산해 다음 조회를 재현한다."""
        target = pose.to_list()
        self.events.append(("line", (self.tcp, target)))
        if self.fail == "line":
            return False
        rotation = Rotation.from_euler("ZYZ", target[3:], degrees=True)
        self.flange = [
            *(np.array(target[:3]) - rotation.apply(self.offsets[self.tcp][:3])),
            *target[3:],
        ]
        return True

    def move_joint(self, joints, **kwargs):
        """관절 도달 검사를 위해 목표 관절을 보관한다."""
        self.events.append(("joint", list(joints)))
        self.joint_values = list(joints)
        return self.fail != "joint"

    def verify_pose(self, pose):
        """이동 실패 주입은 move_line에서 처리한다."""
        return True

    def gripper_open(self, width=100.):
        self.events.append(("open", self.tcp))
        return self.fail != "open"

    def gripper_close(self, width=10., force=20.):
        self.events.append(("close", self.tcp))
        return self.fail != "close"

    def execute_drawing_command(self, command):
        self.events.append(("draw", self.tcp))
        return self.fail != "draw"

    def move_task_joint(self, pose):
        self.events.append(("rotate", self.tcp))
        return self.fail != "draw"

    def current_pose(self):
        from dsr_msgs2.srv import GetCurrentPosx
        return self._call("current_pose", GetCurrentPosx.Request()).task_pos_info[0].data


def test_pick_selects_existing_side_tcp_and_removes_temporary_pivot():
    """집기에는 임시 피벗을 쓰고 완료 후 기존 펜촉 TCP만 선택한다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
    rack = rack_parameters()
    robot = ServiceRobot()
    cycle = IntegratedPenCycle(robot, rack.__getitem__)
    joints = cycle.pick_side()
    assert robot.tcp == "Pen_Side_45"
    assert robot.mode == 1
    assert set(robot.offsets) == {"", "Pen_Side_45"}
    assert joints == robot.joint_values
    lines = [value for kind, value in robot.events if kind == "line"]
    assert len(lines) == 4
    np.testing.assert_allclose(lines[0][1][:3], lines[1][1][:3], atol=1e-9)
    np.testing.assert_allclose(lines[2][1][:3], lines[3][1][:3], atol=1e-9)
    assert all(name.startswith("PickPivot_") for name, _ in lines)


@pytest.mark.parametrize("failure", ["open", "close", "line", "set_tcp"])
def test_pick_failure_resets_without_opening_to_return(failure):
    """집기 실패 뒤에는 반납을 시도하지 않고 TCP와 운전 모드만 복구한다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
    rack = rack_parameters()
    robot = ServiceRobot()
    cycle = IntegratedPenCycle(robot, rack.__getitem__)
    robot.fail = failure
    with pytest.raises(RuntimeError):
        cycle.pick_side()
    assert robot.mode == 1
    if failure != "set_tcp":
        assert robot.tcp == ""
        assert set(robot.offsets) == {"", "Pen_Side_45"}
    assert len([event for event in robot.events if event[0] == "open"]) <= 1


def test_return_uses_current_rack_do_and_zero_after_release(monkeypatch):
    """새 YAML 반납값과 DO 열기, J6 되감기, 해제 뒤 플랜지 후퇴를 확인한다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
    import cake_decorator.side_pen_return as returning
    monkeypatch.setattr(returning.time, "sleep", lambda _: None)
    rack = rack_parameters()
    rack["pen_drop_pose"] = [280., 210., 330., 40.25, 180., 40.02]
    rack["pen_drop_after_pose"] = [280., 210., 570., 37.07, -179.99, 36.84]
    rack["pen_drop_approach_pose"] = [30., 1., 60., 0., 110., 42.]
    robot = ServiceRobot()
    cycle = IntegratedPenCycle(robot, rack.__getitem__)
    robot.tcp = "Pen_Side_45"
    robot.joint_values[5] = 402.
    cycle.return_side([0., 0., 0., 0., 0., 42.])
    assert robot.tcp == ""
    joint_targets = [value for kind, value in robot.events if kind == "joint"]
    assert joint_targets[0][5] == 402.
    assert joint_targets[1][5] == 42.
    assert joint_targets[2] == rack["pen_drop_approach_pose"]
    assert ("open", "Pen_Side_45") in robot.events
    assert not any(kind == "gripper" for kind, _ in robot.events)
    lines = [value for kind, value in robot.events if kind == "line"]
    assert lines[-1][0] == ""
    drop_flange = lines[-1][1].copy()
    drop_flange[2] -= 240.
    center = np.array(drop_flange[:3]) + Rotation.from_euler(
        "ZYZ", drop_flange[3:], degrees=True,
    ).apply([0., 0., 220.])
    expected = np.array(rack["pen_drop_pose"][:3]) + Rotation.from_euler(
        "ZYZ", rack["pen_drop_pose"][3:], degrees=True,
    ).apply([0., 0., 220.])
    np.testing.assert_allclose(center, expected, atol=1e-9)


@pytest.mark.parametrize("offset", [
    [0., 0., 0., 0., 0., 0.],
    [1.095, -46.474, 271.695, 0., 5., 0.],
])
def test_side_tcp_name_with_wrong_transform_cannot_start_drawing(offset):
    """기존 이름이어도 실제 장착 변환이 다르면 케이크로 이동하기 전에 멈춘다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
    robot = ServiceRobot()
    robot.offsets["Pen_Side_45"] = offset
    rack = rack_parameters()
    cycle = IntegratedPenCycle(robot, rack.__getitem__)
    with pytest.raises(RuntimeError, match="TCP.*불일치"):
        cycle.pick_side()
    assert not any(kind == "draw" for kind, _ in robot.events)
    assert robot.tcp == ""


def make_executor(monkeypatch, **parameters):
    """액션·서비스는 실제 Python 메서드를 쓰고 모터/통신 경계만 대체한다."""
    import cake_decorator.robot_executor_node as module
    from cake_decorator.drawing_motion import DsrPos, TaskState
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle

    executor = module.RobotExecutorNode.__new__(module.RobotExecutorNode)
    values = {**module.RobotExecutorNode._PARAM_DEFAULTS, **rack_parameters(), **parameters}
    executor._param = values.__getitem__
    executor._robot = ServiceRobot()
    executor.get_logger = Mock()
    executor._pen_cycle = IntegratedPenCycle(executor._robot, executor._param)
    executor._state = TaskState.IDLE
    executor._cancel_requested = False
    executor._convert_waypoints = lambda *args, **kwargs: ([], [])
    executor._go_to_cake_center = Mock(side_effect=AssertionError("중심 접근 호출"))
    executor._robot.move_home = Mock(side_effect=AssertionError("실패 후 홈 이동"))
    commands = [
        SimpleNamespace(
            pos=DsrPos(401., -55., z, 0., 180., 0.),
            phase=SimpleNamespace(value="APPROACH"),
            motion_type=SimpleNamespace(value="LINE"),
            pen_down=down, stroke_idx=0,
        )
        for z, down in [(600., False), (590., False), (580., False), (570., True)]
    ]
    plan = SimpleNamespace(success=True, commands=commands, total_strokes=1)
    monkeypatch.setattr(module, "plan_drawing_motion", lambda *args: plan)
    monkeypatch.setattr(module.time, "sleep", lambda _: None)
    return executor


def draw_handle(surface):
    """ROS 액션 상태 전이는 Mock으로 기록하되 취소 플래그는 실제 bool이다."""
    handle = Mock()
    handle.request = SimpleNamespace(drawing_surface=surface, waypoints=[], speed=20.)
    handle.is_cancel_requested = False
    handle.is_active = True
    return handle


def test_integrated_side_orders_can_repeat_and_top_does_not_inherit_side_mode(monkeypatch):
    """SIDE 두 주문 후 TOP도 팀 파지/반납과 영점 TCP로 실행할 수 있다."""
    executor = make_executor(monkeypatch, integrated_side_cycle=True, side_pen_45=False)
    for surface in ("SIDE", "SIDE", "TOP"):
        handle = draw_handle(surface)
        result = executor._execute_draw(handle)
        assert result.success, result.error_code
        handle.succeed.assert_called_once()
        assert executor._robot.tcp == ""
        assert not executor._side_pen_returned
    draws = [name for kind, name in executor._robot.events if kind == "draw"]
    assert draws == ["Pen_Side_45"] * 6 + [""] * 4
    assert len([1 for kind, _ in executor._robot.events if kind == "create_tcp"]) == 2
    executor._go_to_cake_center.assert_not_called()


@pytest.mark.parametrize("failure", ["draw", "cancel", "last_cancel", "pick_cancel"])
def test_interrupted_integrated_draw_resets_without_return_or_release(monkeypatch, failure):
    """집기/그리기/마지막 이동 취소를 포함해 실패 시 반납 그리퍼를 열지 않는다."""
    executor = make_executor(monkeypatch, integrated_side_cycle=True)
    handle = draw_handle("SIDE")
    original = executor._robot.execute_drawing_command

    def drawing(command):
        ok = original(command)
        count = len([1 for kind, _ in executor._robot.events if kind == "draw"])
        if failure == "cancel" or (failure == "last_cancel" and count == 3):
            handle.is_cancel_requested = True
        return ok

    executor._robot.execute_drawing_command = drawing
    if failure == "draw":
        executor._robot.fail = "draw"
    elif failure == "pick_cancel":
        original_close = executor._robot.gripper_close

        def close(*args):
            handle.is_cancel_requested = True
            return original_close(*args)

        executor._robot.gripper_close = close
    result = executor._execute_draw(handle)
    assert not result.success
    assert executor._robot.tcp == ""
    assert executor._robot.mode == 1
    assert len([1 for kind, _ in executor._robot.events if kind == "open"]) == 1
    assert len([1 for kind, _ in executor._robot.events if kind == "joint"]) == 1
    handle.succeed.assert_not_called()
    assert not executor._side_pen_returned


def test_top_enforces_zero_from_named_nonzero_tcp_and_uses_first_point(monkeypatch):
    """상면/거치대 플랜지 좌표에 옆면 펜촉 오프셋을 섞지 않는다."""
    executor = make_executor(monkeypatch, integrated_side_cycle=True, side_pen_45=True)
    executor._robot.tcp = "Pen_Side_45"
    handle = draw_handle("TOP")
    assert executor._execute_draw(handle).success
    lines = [value for kind, value in executor._robot.events if kind == "line"]
    assert all(name == "" for name, _ in lines)
    assert all(name == "" for kind, name in executor._robot.events if kind == "draw")
    assert not any(kind == "create_tcp" for kind, _ in executor._robot.events)
    assert not any(kind == "rotate" for kind, _ in executor._robot.events)
    executor._go_to_cake_center.assert_not_called()


def test_top_pick_cancellation_stops_before_lifting_or_drawing(monkeypatch):
    """TOP 집기 중 취소하면 다음 인출 동작 대신 TCP 정리만 수행한다."""
    executor = make_executor(monkeypatch, pen_tool_cycle=True)
    handle = draw_handle("TOP")
    original_close = executor._robot.gripper_close

    def close(*args):
        handle.is_cancel_requested = True
        return original_close(*args)

    executor._robot.gripper_close = close
    result = executor._execute_draw(handle)
    assert not result.success
    assert result.error_code == "CANCELLED"
    handle.canceled.assert_called_once()
    assert len([1 for kind, _ in executor._robot.events if kind == "line"]) == 1
    assert not any(kind == "draw" for kind, _ in executor._robot.events)
    assert executor._robot.tcp == ""


def test_top_draw_failure_does_not_return_or_release_pen(monkeypatch):
    """팀 TOP 집기에 성공해도 드로잉 실패 뒤 자동 반납을 실행하지 않는다."""
    executor = make_executor(monkeypatch, pen_tool_cycle=True)
    executor._robot.fail = "draw"
    handle = draw_handle("TOP")
    result = executor._execute_draw(handle)
    assert not result.success
    assert result.error_code == "MOVE_FAILED_AT_0"
    assert executor._robot.tcp == ""
    # 그리퍼 움찔거림 수정으로 TOP 집기에서 열기 펄스를 보내지 않는다.
    assert len([1 for kind, _ in executor._robot.events if kind == "open"]) == 0
    assert len([1 for kind, _ in executor._robot.events if kind == "joint"]) == 1
    executor._robot.move_home.assert_not_called()


def test_integrated_action_reports_tcp_reset_failure(monkeypatch):
    """본 동작 실패 후 초기화도 실패하면 TCP 오류를 숨기지 않는다."""
    executor = make_executor(monkeypatch, integrated_side_cycle=True)
    handle = draw_handle("SIDE")

    def drawing(command):
        executor._robot.fail = "set_tcp"
        return False

    executor._robot.execute_drawing_command = drawing
    result = executor._execute_draw(handle)
    assert not result.success
    assert result.error_code == "TCP_RESET_FAILED"
    handle.abort.assert_called_once()
    handle.succeed.assert_not_called()


def test_integration_is_opt_in_and_helper_construction_does_not_command_robot():
    """기본 설정은 수동 순환이며 helper 생성은 클라이언트 준비만 한다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
    from cake_decorator.robot_executor_node import RobotExecutorNode
    robot = ServiceRobot()
    original = object()
    robot._clients["get_tcp"] = original
    rack = rack_parameters()
    IntegratedPenCycle(robot, rack.__getitem__)
    assert robot._clients["get_tcp"] is original
    assert robot.events == []
    assert RobotExecutorNode._PARAM_DEFAULTS["integrated_side_cycle"] is False


def test_plate_resets_tcp_before_callback_and_rejects_reset_failure(monkeypatch):
    """플레이트 서비스는 영점 복원 성공 전에는 이동 콜백을 실행하지 않는다."""
    import threading
    executor = make_executor(monkeypatch, integrated_side_cycle=True, plate_sequence_enabled=True)
    executor._plate_lock = threading.Lock()
    executor._robot.tcp = "Pen_Side_45"
    callback = Mock(side_effect=lambda: executor._robot.tcp == "")
    result = executor._plate_run(SimpleNamespace(success=False), "준비", callback)
    assert result.success
    callback.assert_called_once()
    # 기본 TCP면 재설정을 생략하므로, 실패 시험은 실제로 복원이 필요한 상태로 만든다.
    executor._robot.tcp = "Pen_Side_45"
    executor._robot.fail = "set_tcp"
    callback.reset_mock()
    result = executor._plate_run(SimpleNamespace(success=False), "준비", callback)
    assert not result.success
    callback.assert_not_called()


def test_top_return_position_skips_only_redundant_side_joint_approach():
    """TOP 반납 위치가 실측으로 확인된 경우에만 중복 관절 자세를 생략한다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
    rack = rack_parameters()
    robot = ServiceRobot()
    robot.flange = list(rack['pen_drop_pose'])
    cycle = IntegratedPenCycle(robot, rack.__getitem__)
    cycle.pick_side(reuse_rack_position=True)
    assert not any(kind == 'joint' for kind, _ in robot.events)
    lines = [value for kind, value in robot.events if kind == 'line']
    assert len(lines) == 4  # 하강·기울임·인출·수직 복귀는 보존한다.


def test_moved_rack_position_keeps_original_side_joint_approach():
    """반납 위치가 달라졌으면 제자리 재파지를 중단한다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
    rack = rack_parameters()
    robot = ServiceRobot()
    robot.flange = list(rack['pen_drop_pose'])
    robot.flange[0] += 10.
    cycle = IntegratedPenCycle(robot, rack.__getitem__)
    with pytest.raises(RuntimeError, match='위치 불일치'):
        cycle.pick_side(reuse_rack_position=True)
    assert not any(kind == 'joint' for kind, _ in robot.events)


@pytest.mark.parametrize('continuing', [False, True])
def test_side_regrip_preserves_speed_during_two_surface_order(continuing):
    """TOP 이후 재파지에도 펜 전용 인출·거치대 속도를 적용한다."""
    from cake_decorator.integrated_pen_cycle import IntegratedPenCycle
    rack = rack_parameters()
    robot = ServiceRobot()
    robot.move_line = Mock(wraps=robot.move_line)
    cycle = IntegratedPenCycle(robot, rack.__getitem__)
    cycle.pick_side(preserve_speed=continuing)
    speeds = [call.kwargs['speed_mm_s'] for call in robot.move_line.call_args_list]
    assert speeds == [80., 100., 100., 100.]


@pytest.mark.parametrize('has_more', [False, True])
def test_top_intermediate_return_keeps_speed_and_final_return_is_faster(monkeypatch, has_more):
    """중간 반납은 제자리에서 멈추고 최종 반납만 상공으로 후퇴한다."""
    from cake_decorator.drawing_motion import TaskState
    executor = make_executor(monkeypatch, integrated_side_cycle=True)
    executor._state = TaskState.DRAWING
    executor._robot.move_joint = Mock(wraps=executor._robot.move_joint)
    executor._robot.move_line = Mock(wraps=executor._robot.move_line)
    handle = draw_handle('TOP')
    handle.request.has_more_surfaces = has_more
    assert executor._do_returning_tool(handle, handle.request)
    assert executor._robot.move_joint.call_args.kwargs['speed_deg_s'] == 45.
    assert [call.kwargs['speed_mm_s'] for call in executor._robot.move_line.call_args_list] == (
        [80.] if has_more else [80., 100.]
    )


def test_top_to_side_regrips_without_return_lift_or_joint_approach(monkeypatch):
    """실제 TOP 반납 후 SIDE 파지를 연속 호출해 상공 왕복이 재발하지 않게 한다."""
    executor = make_executor(monkeypatch, integrated_side_cycle=True)
    top = draw_handle('TOP')
    top.request.has_more_surfaces = True
    assert executor._execute_draw(top).success
    rack = rack_parameters()
    np.testing.assert_allclose(executor._robot.flange, rack['pen_drop_pose'], atol=1e-8)
    boundary = len(executor._robot.events)
    side = draw_handle('SIDE')
    assert executor._execute_draw(side).success
    events = executor._robot.events[boundary:]
    close_index = next(i for i, (kind, _) in enumerate(events) if kind == 'close')
    assert not any(kind == 'joint' for kind, _ in events[:close_index])
    # 닫기 전 직선 목표는 반납 위치와 기울임 파지뿐이며 상공 후퇴는 없다.
    poses = [value[1] for kind, value in events[:close_index] if kind == 'line']
    assert len(poses) == 2
