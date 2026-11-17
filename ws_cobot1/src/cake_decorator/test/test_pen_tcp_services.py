"""TCP 설정 서비스의 순서·모드 복원을 검사한다. 실제 ROS 서비스는 호출하지 않는다."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

path = Path(__file__).resolve().parents[4] / "scripts/system_launcher.py"
spec = importlib.util.spec_from_file_location("tcp_setup_launcher", path)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


def session(mode=1, fail_create=False):
    instance = launcher.RobotSession.__new__(launcher.RobotSession)
    instance._identity = ("real", "192.168.1.100")
    instance.identity = lambda *args: None
    calls = []

    def call(kind, path, request, timeout=60):
        calls.append((path, request))
        if path.endswith("config_create_tcp") and fail_create:
            raise RuntimeError("등록 실패")
        return SimpleNamespace(success=True, robot_mode=mode)

    instance.call = call
    return instance, calls


def test_registers_only_measured_tcp_and_restores_auto_mode():
    robot, calls = session()
    profile = {
        "tcp_name": "Measured_Pen",
        "tcp_pose_mm_deg": [8.0, -4.0, 273.0, 0.0, 0.0, 0.0],
    }
    robot.apply_pen_tcp(profile)
    assert [path.rsplit("/", 1)[-1] for path, _ in calls] == [
        "get_robot_mode",
        "set_robot_mode",
        "config_create_tcp",
        "set_current_tcp",
        "set_robot_mode",
    ]
    assert calls[1][1].robot_mode == 0 and calls[-1][1].robot_mode == 1
    assert list(calls[2][1].pos) == profile["tcp_pose_mm_deg"]
    assert calls[2][1].name == calls[3][1].name == "Measured_Pen"
    assert not any("/motion/" in path or "gripper" in path for path, _ in calls)


def test_failed_registration_restores_previous_mode_and_does_not_select():
    robot, calls = session(fail_create=True)
    with pytest.raises(RuntimeError, match="등록 실패"):
        robot.apply_pen_tcp(
            {
                "tcp_name": "Measured_Pen",
                "tcp_pose_mm_deg": [0.0, 0.0, 273.0, 0.0, 0.0, 0.0],
            }
        )
    assert calls[-1][1].robot_mode == 1
    assert not any(path.endswith("set_current_tcp") for path, _ in calls)


def test_saved_selection_does_not_overwrite_or_recreate_item():
    robot, calls = session(mode=0)
    robot.apply_pen_tcp({"tcp_name": "Existing_Measured_Pen"}, create=False)
    assert [path.rsplit("/", 1)[-1] for path, _ in calls] == [
        "get_robot_mode",
        "set_current_tcp",
    ]


def test_virtual_target_is_rejected_before_any_configuration_call():
    robot, calls = session()
    robot._identity = ("virtual", "127.0.0.1")
    with pytest.raises(RuntimeError, match="실제 드라이버"):
        robot.apply_pen_tcp(
            {
                "tcp_name": "Measured_Pen",
                "tcp_pose_mm_deg": [0.0, 0.0, 273.0, 0.0, 0.0, 0.0],
            }
        )
    assert not calls


def test_manual_teaching_cancel_restores_mode_without_tcp_changes():
    robot, calls = session()
    with pytest.raises(KeyboardInterrupt):
        with robot.manual_tcp_mode():
            raise KeyboardInterrupt
    assert [path.rsplit("/", 1)[-1] for path, _ in calls] == [
        "get_robot_mode",
        "set_robot_mode",
        "set_robot_mode",
    ]
    assert calls[1][1].robot_mode == 0 and calls[2][1].robot_mode == 1
