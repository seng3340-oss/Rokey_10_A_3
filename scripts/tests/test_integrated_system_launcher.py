"""통합 기동의 표면별 기준점과 하드웨어 없는 검사·실행 배선을 확인한다."""

import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import ModuleType, SimpleNamespace

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "integrated_system_launcher", ROOT / "scripts/system_launcher.py",
)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


@pytest.fixture
def config_files(tmp_path, monkeypatch):
    """실측 SIDE 높이와 TOP 티칭 높이가 다른 파일을 준비한다."""
    monkeypatch.syspath_prepend(str(ROOT / "ws_cobot1/src/cake_decorator"))
    real = tmp_path / "real_robot.json"
    real.write_text(json.dumps({
        "cake_top_z_mm": 222.0,
        "configure_pen_tcp_on_yes": True,
        "side_cake_center_xy_mm": [396.8743225097656, -50.559329986572266],
    }))
    workspace = tmp_path / "workspace.yaml"
    workspace.write_text(yaml.safe_dump({
        "drawing_path_node": {"ros__parameters": {
            "side_diameter_allowance_mm": 9.25,
            "drawing_speed": 30.0,
            "side_drawing_speed": 60.0,
            "travel_speed": 80.0,
        }},
        "robot_executor_node": {"ros__parameters": {
            "side_pen_45": True,
            "cake_center_pose": [401.8, -55.57, 510.39, 179.76, 179.35, 179.02],
        }},
    }))
    return real, workspace


def test_top_and_side_references_keep_distinct_heights(config_files):
    """TOP는 기존 TCP 티칭 높이, SIDE는 물리 상면과 수직 파지 자세를 쓴다."""
    real, workspace = config_files
    top = launcher.load_config_cake_reference(real, workspace)
    side = launcher.load_side_cake_reference(real)
    assert top[:2] == side[:2] == [396.8743225097656, -50.559329986572266]
    assert top[2:] == [510.39, 179.76, 179.35, 179.02]
    assert side[2:] == [222.0, 123.66, 180.0, 123.43]


@pytest.mark.parametrize("xy", [None, [1], [float("nan"), 0], [901, 0]])
def test_side_reference_rejects_invalid_center(config_files, xy):
    """SIDE의 설정 오타를 로봇 조회 없이 거부한다."""
    real, _ = config_files
    config = json.loads(real.read_text())
    config["side_cake_center_xy_mm"] = xy
    real.write_text(json.dumps(config))
    with pytest.raises(RuntimeError):
        launcher.load_side_cake_reference(real)


def test_runtime_enables_surface_specific_executor_and_preserves_yaml(config_files):
    """DP 옆면 설정만 바꾸며 TOP 자세·지름 유격·원본 파일을 보존한다."""
    _, workspace = config_files
    original = workspace.read_text()
    args = SimpleNamespace(integrated_side_cycle=True, side_y_inset_mm=4, mode="real")
    runtime = launcher.load_runtime_parameters(args, workspace)
    dp = runtime["drawing_path_node"]["ros__parameters"]
    executor = runtime["robot_executor_node"]["ros__parameters"]
    assert dp["side_pen_45"] is True
    assert dp["side_y_inset_mm"] == 1.0
    assert dp["side_drawing_speed"] == 40.0
    assert dp["travel_speed"] == 180.0
    assert dp["side_diameter_allowance_mm"] == 9.25
    assert executor["integrated_side_cycle"] is True
    assert executor["side_pen_45"] is False
    assert executor["cake_center_pose"][2] == 510.39
    assert workspace.read_text() == original


def test_standalone_virtual_side_parameters_are_preserved(config_files):
    """독립 가상 시험은 기존 RE 45도 모드와 펜 반납 생략을 유지한다."""
    _, workspace = config_files
    args = SimpleNamespace(integrated_side_cycle=False, side_y_inset_mm=2, mode="virtual")
    runtime = launcher.load_runtime_parameters(args, workspace)
    assert runtime["drawing_path_node"]["ros__parameters"]["side_y_inset_mm"] == 2
    executor = runtime["robot_executor_node"]["ros__parameters"]
    assert executor["side_pen_45"] is True
    assert executor["side_return_pen"] is False
    assert "integrated_side_cycle" not in executor


@pytest.fixture
def isolated_launcher(config_files, tmp_path, monkeypatch):
    """ROS·서비스·프로세스는 전부 가짜로 바꾸고 실제 main의 배선만 실행한다."""
    real, workspace = config_files
    root = tmp_path / "project"
    for name in ("frontend/node_modules/vite/bin/vite.js", "backend/.venv/bin/python"):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    monkeypatch.setattr(launcher, "ROOT", root)
    for name, paths in (
        ("load_real_robot_config", (real,)),
        ("load_config_cake_reference", (real, workspace)),
        ("load_side_cake_reference", (real,)),
    ):
        original = getattr(launcher, name)
        monkeypatch.setattr(
            launcher, name,
            lambda *args, fn=original, defaults=paths: fn(*(args or defaults)),
        )
    load_runtime = launcher.load_runtime_parameters
    monkeypatch.setattr(
        launcher, "load_runtime_parameters", lambda args: load_runtime(args, workspace),
    )
    for name in ("CAKE_PEN_TOOL_CYCLE", "CAKE_PLATE_SEQUENCE", "CAKE_SYSTEM_MODE"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(launcher, "require_free_port", lambda port: None)
    monkeypatch.setattr(launcher.importlib.util, "find_spec", lambda name: object())
    monkeypatch.setattr(launcher.shutil, "which", lambda name: sys.executable)
    monkeypatch.setattr(launcher.subprocess, "check_output", lambda *a, **kw: "v22.12.0")
    monkeypatch.setattr(launcher.time, "sleep", lambda _: None)
    monkeypatch.setattr(launcher.signal, "signal", lambda *args: None)
    monkeypatch.setattr(launcher, "resolve_pending_orders", lambda *args: None)
    service_module = ModuleType("cake_decorator_interfaces.srv")
    service_module.GenerateTrajectory = SimpleNamespace(Request=SimpleNamespace(
        get_fields_and_field_types=lambda: {"strokes_2d": "test"},
    ))
    monkeypatch.setitem(sys.modules, "cake_decorator_interfaces.srv", service_module)
    state = SimpleNamespace(sessions=[], commands={}, closed=[], root=root)

    def forbidden(*args, **kwargs):
        """중앙 이동·YES·현재 TCP 조회·복원 프로세스가 호출되면 실패한다."""
        pytest.fail("통합 기동에서 중앙 접근·YES·TCP 작업을 호출했습니다.")

    class Session:
        """하드웨어 없이 드라이버 신원·준비 검사 호출만 허용한다."""

        def __init__(self):
            state.sessions.append(self)
            self.node = SimpleNamespace(get_node_names_and_namespaces=lambda: [])

        identity = lambda self, *args: None
        wait_controller = lambda self: None
        ready_application = lambda self: None
        close = lambda self: None
        pose = forbidden

    class Processes:
        """명령을 저장하며 실제 자식 프로세스를 생성하지 않는다."""

        def __init__(self, folder, env):
            state.folder = folder
            state.env = env

        def start(self, name, command, cwd=None):
            state.commands[name] = command

        def wait(self, *args):
            pass

        def check(self):
            raise KeyboardInterrupt

        def close(self, **kwargs):
            state.closed.append(kwargs)

    monkeypatch.setattr(launcher, "RobotSession", Session)
    monkeypatch.setattr(launcher, "Processes", Processes)
    for name in (
        "prepare_reference", "prepare_side_pen_reference", "prepare_virtual_side_pen",
        "restore_side_tcp_after_exit",
    ):
        monkeypatch.setattr(launcher, name, forbidden)
    monkeypatch.setattr("builtins.input", forbidden)
    monkeypatch.setattr(sys, "argv", [
        "system_launcher.py", "--mode", "real", "--integrated-side-cycle", "--no-browser",
    ])
    return state


def test_check_validates_configuration_without_robot_session(isolated_launcher, monkeypatch, capsys):
    """--check는 TOP/SIDE 설정만 검증하고 ROS 세션·로그·자식을 만들지 않는다."""
    monkeypatch.setattr(sys, "argv", [*sys.argv, "--check"])
    launcher.main()
    state = isolated_launcher
    assert not state.sessions
    assert not state.commands
    assert not (state.root / "log").exists()
    output = capsys.readouterr().out
    assert "510.39" in output
    assert "222.0" in output
    assert "9.25mm" in output
    assert "준비 검사 통과" in output


def test_integrated_startup_passes_config_references_and_full_process_flags(isolated_launcher):
    """전체 main에서 중앙 이동 없이 RE·TM·브릿지에 정확한 설정이 전달된다."""
    with pytest.raises(KeyboardInterrupt):
        launcher.main()
    state = isolated_launcher
    assert len(state.sessions) == 1
    assert state.env["CAKE_PEN_TOOL_CYCLE"] == "1"
    assert state.env["CAKE_PLATE_SEQUENCE"] == "1"
    record = json.loads((state.folder / "reference.json").read_text())
    assert record["source"] == "config"
    assert record["pose"][2] == 510.39
    assert "start_pose" not in record
    executor_command = state.commands["robot_executor_node"]
    assert "pen_tool_cycle:=true" in executor_command
    assert "plate_sequence_enabled:=true" in executor_command
    config = Path(executor_command[executor_command.index("--params-file") + 1])
    executor = yaml.safe_load(config.read_text())["robot_executor_node"]["ros__parameters"]
    assert executor["integrated_side_cycle"] is True
    assert executor["side_pen_45"] is False
    manager_command = state.commands["task_manager_node"]
    assert "plate_sequence_enabled:=true" in manager_command
    assert "integrated_side_cycle:=true" in manager_command
    side_arg = next(arg for arg in manager_command if arg.startswith("side_reference_pose:="))
    assert json.loads(side_arg.split(":=", 1)[1]) == [
        396.8743225097656, -50.559329986572266, 222.0, 123.66, 180.0, 123.43,
    ]
    assert callable(state.closed[0]["before_driver_close"])


@pytest.mark.parametrize("failed_check", ["identity", "wait_controller"])
def test_integrated_cleanup_requires_verified_driver(isolated_launcher, monkeypatch, failed_check):
    """대상 또는 연결 확인이 실패하면 종료 중 TCP 복원을 예약하지 않는다."""
    def fail(*args):
        raise RuntimeError("드라이버 확인 실패")

    monkeypatch.setattr(launcher.RobotSession, failed_check, fail)
    with pytest.raises(RuntimeError, match="드라이버 확인 실패"):
        launcher.main()
    assert isolated_launcher.closed == [{"before_driver_close": None}]
    assert "backend" not in isolated_launcher.commands


@pytest.mark.parametrize("source, start_pose, drift, rejected", [
    ("config", None, 100.0, False),
    ("tcp", None, 0.11, True),
    ("tcp", [20.0, 30.0, 272.0, 0.0, 180.0, 0.0], 0.49, False),
    ("tcp", [20.0, 30.0, 272.0, 0.0, 180.0, 0.0], 0.5, True),
])
def test_bridge_preserves_config_bypass_and_local_start_pose_guard(
    tmp_path, monkeypatch, source, start_pose, drift, rejected,
):
    """설정 기준점은 TCP를 조회하지 않고 기존 영점·이탈 자세 검사는 유지한다."""
    pose = [396.87, -50.56, 510.39, 179.76, 179.35, 179.02]
    record = {"mode": "real", "pose": pose, "source": source}
    if start_pose is not None:
        record["start_pose"] = start_pose
    path = tmp_path / "reference.json"
    path.write_text(json.dumps(record))
    observed = list(start_pose or pose)
    observed[0] += drift
    state = SimpleNamespace(poses=0, bridge_reference=None)

    class Session:
        """서비스를 호출하지 않고 TCP 조회 횟수와 위치만 제공한다."""

        identity = lambda self, *args: None
        close = lambda self: None

        def pose(self):
            state.poses += 1
            assert source != "config"
            return observed

    def create_bridge(client, reference, **kwargs):
        state.bridge_reference = reference
        return SimpleNamespace(start=lambda: None, close=lambda: None, destroy_node=lambda: None)

    for name, values in (
        ("cake_decorator.backend_client", {"BackendClient": lambda *args: None}),
        ("cake_decorator.ros_bridge_node", {"RosBridgeNode": create_bridge}),
        ("rclpy", {"init": lambda: None, "spin": lambda node: None, "try_shutdown": lambda: None}),
    ):
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setenv("CAKE_BRIDGE_TOKEN", "test")
    monkeypatch.setattr(launcher, "RobotSession", Session)
    args = SimpleNamespace(
        bridge_reference=str(path), mode="real", host="192.0.2.1", backend_port=5000,
    )
    if rejected:
        with pytest.raises(RuntimeError, match="TCP 위치 차이"):
            launcher.bridge_worker(args)
        assert state.bridge_reference is None
    else:
        launcher.bridge_worker(args)
        assert state.bridge_reference == pose
        assert Path(str(path) + ".ready").exists()
    assert state.poses == (0 if source == "config" else 1)


def test_start_real_shell_enables_integrated_workflow_without_prompt(tmp_path):
    """환경 진입점을 가짜로 바꿔 셸 옵션·기능 플래그만 검증한다."""
    script = tmp_path / "start_real.sh"
    shutil.copyfile(ROOT / "scripts/start_real.sh", script)
    (tmp_path / "_system_environment.sh").write_text(
        'python3 - "$@" <<\'PY\'\n'
        'import json, os, sys\n'
        'print(json.dumps({"args": sys.argv[1:], '
        '"pen": os.environ["CAKE_PEN_TOOL_CYCLE"], '
        '"plate": os.environ["CAKE_PLATE_SEQUENCE"]}))\n'
        'PY\n'
    )
    result = subprocess.run(
        ["bash", str(script), "--check", "--host", "192.0.2.1"],
        input="", capture_output=True, text=True, check=True,
    )
    payload = json.loads(result.stdout)
    assert payload["args"] == [
        "real", "--cake-center-from-config", "--integrated-side-cycle",
        "--check", "--host", "192.0.2.1",
    ]
    assert payload["pen"] == payload["plate"] == "1"


def test_top_calibration_does_not_move_verified_side_center(config_files):
    """윗면 십자가 보정은 옆면 TCP 기준 중심과 분리해 전달한다."""
    real, workspace = config_files
    config = json.loads(real.read_text())
    config['top_cake_center_xy_mm'] = [400.5243225097656, -53.00932998657227]
    real.write_text(json.dumps(config))
    assert launcher.load_config_cake_reference(real, workspace)[:2] == config['top_cake_center_xy_mm']
    assert launcher.load_side_cake_reference(real)[:2] == config['side_cake_center_xy_mm']
