"""통합 실행기의 모드 구분과 기존 프로세스 보호를 검사한다."""

import importlib.util
import os
from pathlib import Path
import socket
import sys
import time
import unittest

PATH = Path(__file__).parents[1] / "system_launcher.py"
spec = importlib.util.spec_from_file_location("system_launcher", PATH)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


def test_process_logs_are_teed_before_children_exit(tmp_path, capsys):
    """파일 전용 리다이렉션과 Python 버퍼링으로 오류가 숨는 회귀를 잡는다."""
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "0"
    original_env = env.copy()
    processes = launcher.Processes(tmp_path, env)
    output = ""
    try:
        for name in ("driver", "worker"):
            processes.start(
                name,
                [
                    sys.executable,
                    "-c",
                    "import sys,time; print('준비'); "
                    "print('로봇 오류',file=sys.stderr); time.sleep(30)",
                ],
            )
        deadline = time.monotonic() + 3
        expected = (
            "[driver] 준비\n",
            "[driver] 로봇 오류\n",
            "[worker] 준비\n",
            "[worker] 로봇 오류\n",
        )
        while time.monotonic() < deadline:
            output += capsys.readouterr().out
            if all(line in output for line in expected):
                break
            time.sleep(0.01)
        for line in expected:
            assert line in output
        processes.check()
        for name in ("driver", "worker"):
            assert (tmp_path / f"{name}.log").read_text() == "준비\n로봇 오류\n"
        assert env == original_env
    finally:
        processes.close(grace=0.3, terminate_grace=0.3)


def test_close_drains_shutdown_logs_and_closes_readers(tmp_path, capsys, monkeypatch):
    """종료 핸들러의 마지막 줄과 개행 없는 꼬리도 저장한 뒤 reader를 회수한다."""
    import builtins

    logs = []

    def track_open(*args, **kwargs):
        handle = builtins.open(*args, **kwargs)
        logs.append(handle)
        return handle

    monkeypatch.setattr(launcher, "open", track_open, raising=False)
    processes = launcher.Processes(tmp_path, os.environ.copy())
    ready = tmp_path / "ready"
    script = (
        "import pathlib,signal,sys,time\n"
        "def stop(*_):\n"
        "    print('종료 오류',file=sys.stderr)\n"
        "    sys.stdout.write('마지막 로그')\n"
        "    sys.exit(0)\n"
        "signal.signal(signal.SIGINT,stop)\n"
        "pathlib.Path(sys.argv[1]).touch()\n"
        "while True: time.sleep(.01)\n"
    )
    try:
        processes.start("shutdown", [sys.executable, "-c", script, str(ready)])
        processes.wait(ready.exists, timeout=3)
    finally:
        processes.close(grace=0.5, terminate_grace=0.3)
    assert (tmp_path / "shutdown.log").read_text() == "종료 오류\n마지막 로그"
    output = capsys.readouterr().out
    assert "[shutdown] 종료 오류\n" in output
    assert "[shutdown] 마지막 로그" in output
    assert processes.children[0][1].stdout.closed
    assert all(not reader.is_alive() for reader in processes._readers)
    assert logs and all(log.closed for log in logs)
    processes.close(grace=0.1, terminate_grace=0.1)


def test_exited_child_preserves_raw_log_and_check_failure(tmp_path, capsys):
    """디코딩 불가 바이트가 있어도 tee가 멈추지 않고 기존 종료 검사를 유지한다."""
    import pytest

    processes = launcher.Processes(tmp_path, os.environ.copy())
    try:
        processes.start(
            "failed",
            [
                sys.executable,
                "-c",
                "import os; os.write(1,b'error \\xff\\nlast'); raise SystemExit(7)",
            ],
        )
        processes.children[0][1].wait(timeout=3)
        with pytest.raises(RuntimeError, match="failed 종료"):
            processes.check()
    finally:
        processes.close(grace=0.3, terminate_grace=0.3)
    assert (tmp_path / "failed.log").read_bytes() == b"error \xff\nlast"
    output = capsys.readouterr().out
    assert "[failed] error \ufffd\n" in output
    assert "[failed] last" in output


def test_failed_start_closes_log_file(tmp_path, monkeypatch):
    """자식 실행 실패 시 아직 reader에 넘기지 못한 로그 파일을 닫는다."""
    import builtins
    import pytest

    opened = []

    def track_open(*args, **kwargs):
        handle = builtins.open(*args, **kwargs)
        opened.append(handle)
        return handle

    monkeypatch.setattr(launcher, "open", track_open, raising=False)
    processes = launcher.Processes(tmp_path, os.environ.copy())
    with pytest.raises(FileNotFoundError):
        processes.start("missing", [str(tmp_path / "missing-command")])
    assert opened and all(handle.closed for handle in opened)
    assert processes.children == []


def description(mode, host):
    return f'<robot><ros2_control><hardware><param name="mode">{mode}</param><param name="host">{host}</param></hardware></ros2_control></robot>'


class LauncherTests(unittest.TestCase):
    def test_wrong_hardware_mode_or_host_is_rejected(self):
        for actual_mode, actual_host in (
            ("real", "192.168.1.100"),
            ("virtual", "192.168.1.100"),
        ):
            with self.assertRaises(RuntimeError):
                launcher.validate_identity(
                    description(actual_mode, actual_host), "virtual", "127.0.0.1"
                )
        launcher.validate_identity(
            description("real", "192.168.1.100"), "real", "192.168.1.100"
        )

    def test_virtual_pen_length_rejects_nonfinite_or_invalid_values(self):
        for value in ("0", "-1", "nan", "inf", "21"):
            with self.assertRaises(ValueError):
                launcher.pen_length(value)
        self.assertEqual(launcher.pen_length("4"), 4.0)

    def test_busy_port_is_reported_without_stopping_its_owner(self):
        with socket.socket() as server:
            server.bind(("127.0.0.1", 0))
            server.listen()
            with self.assertRaises(RuntimeError):
                launcher.require_free_port(server.getsockname()[1])
            self.assertGreater(server.fileno(), 0)

    def test_real_reference_only_reads_pose_after_yes(self):
        events = []

        class Session:
            def pose(self):
                events.append("read_pose")
                return [500.0, 0.0, 142.0, 0.0, 180.0, 0.0]

            def prepare_virtual(self):
                raise AssertionError("실제 모드에서 가상 이동 호출")

        def answer(_):
            events.append("YES")
            return "YES"

        pose = launcher.prepare_reference(Session(), "real", answer)
        self.assertEqual(events, ["YES", "read_pose"])
        self.assertEqual(pose[2], 142.0)

    def test_real_reference_decline_never_reads_or_moves_robot(self):
        class Session:
            def pose(self):
                raise AssertionError("거부 후 TCP 조회")

        with self.assertRaises(RuntimeError):
            launcher.prepare_reference(Session(), "real", lambda _: "NO")

    def test_cleanup_only_stops_owned_process_groups(self):
        """실행기 종료가 기존 서버나 다른 작업자의 프로세스를 끄지 않게 한다."""
        import os
        import subprocess
        import sys
        import tempfile
        from time import sleep

        unrelated = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            start_new_session=True,
        )
        try:
            with tempfile.TemporaryDirectory() as folder:
                processes = launcher.Processes(Path(folder), os.environ.copy())
                processes.start(
                    "owned", [sys.executable, "-c", "import time; time.sleep(30)"]
                )
                sleep(0.05)
                processes.close()
                self.assertIsNotNone(processes.children[0][1].poll())
                self.assertIsNone(unrelated.poll())
        finally:
            unrelated.terminate()
            unrelated.wait()

    def test_reuse_driver_does_not_allow_duplicate_application_nodes(self):
        nodes = [("robot_state_publisher", "/dsr01")]
        launcher.validate_existing_nodes(nodes, reuse_driver=True)
        with self.assertRaises(RuntimeError):
            launcher.validate_existing_nodes(nodes, reuse_driver=False)
        with self.assertRaises(RuntimeError):
            launcher.validate_existing_nodes(
                nodes + [("robot_executor_node", "/")], reuse_driver=True
            )

    def test_virtual_entry_cannot_be_overridden_to_real(self):
        """가상용 Bash에서 --mode real을 덧붙여도 실제 모드로 전환하지 않는다."""
        import os
        import subprocess
        import sys

        env = os.environ.copy()
        env["CAKE_SYSTEM_MODE"] = "virtual"
        result = subprocess.run(
            [sys.executable, str(PATH), "--mode", "real", "--check"],
            env=env,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("모드는 바꿀 수 없습니다", result.stderr)

    def test_cleanup_stops_children_after_group_leader_exits(self):
        """ros2 부모가 먼저 끝난 경우에도 그 프로세스 그룹의 자식은 정리한다."""
        import os
        import signal
        import sys
        import tempfile

        parent = (
            "import subprocess,sys,time; "
            "p=subprocess.Popen([sys.executable,'-c',"
            "'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); time.sleep(30)']); "
            "print(p.pid,flush=True); time.sleep(.1)"
        )
        with tempfile.TemporaryDirectory() as folder:
            processes = launcher.Processes(Path(folder), os.environ.copy())
            processes.start("parent", [sys.executable, "-c", parent])
            leader = processes.children[0][1]
            leader.wait(timeout=3)
            try:
                os.killpg(leader.pid, 0)
                processes.close(grace=0.2, terminate_grace=0.3)
                with self.assertRaises(ProcessLookupError):
                    os.killpg(leader.pid, 0)
            finally:
                try:
                    os.killpg(leader.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def test_cleanup_survives_second_ctrl_c(tmp_path):
    """정리 도중 다시 누른 Ctrl+C가 강제 종료 단계까지 끊지 않아야 한다."""
    import signal
    import threading

    processes = launcher.Processes(tmp_path, os.environ.copy())
    ready = tmp_path / "stubborn-ready"
    script = (
        "import signal,time,pathlib,sys; "
        "signal.signal(signal.SIGINT,signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
        "pathlib.Path(sys.argv[1]).touch(); time.sleep(30)"
    )
    processes.start("stubborn", [sys.executable, "-c", script, str(ready)])
    processes.wait(ready.exists, timeout=3)
    previous = signal.getsignal(signal.SIGINT)
    interrupt = threading.Timer(0.05, lambda: os.kill(os.getpid(), signal.SIGINT))
    interrupt.start()
    try:
        processes.close(grace=0.2, terminate_grace=0.2)
        assert processes.children[0][1].poll() is not None
        assert signal.getsignal(signal.SIGINT) == previous
    finally:
        interrupt.join()
        processes._signal_group(processes.children[0][1].pid, signal.SIGKILL)
        processes.close(grace=0.01, terminate_grace=0.01)


def test_real_launch_defaults_to_local_discovery_without_changing_domain():
    """실제 드라이버도 외부 검색 서버가 끊기면 모델 Topic을 못 받는 회귀를 막는다."""
    env = {
        "ROS_DISCOVERY_SERVER": "172.16.0.217:11811",
        "ROS_SUPER_CLIENT": "TRUE",
        "ROS_DOMAIN_ID": "30",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
    }
    launcher.configure_discovery("real", env)
    assert "ROS_DISCOVERY_SERVER" not in env
    assert "ROS_SUPER_CLIENT" not in env
    assert env["ROS_DOMAIN_ID"] == "30"
    assert env["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"


def test_explicit_real_discovery_server_option_preserves_environment():
    """교육장 외부 ROS 노드와 연동할 때만 명시적으로 기존 서버를 재사용한다."""
    env = {
        "CAKE_USE_DISCOVERY_SERVER": "1",
        "ROS_DISCOVERY_SERVER": "host:11811",
        "ROS_SUPER_CLIENT": "TRUE",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
    }
    before = env.copy()
    launcher.configure_discovery("real", env)
    assert env == before


def test_virtual_discovery_stays_local_even_when_real_server_option_is_set():
    """실제 모드용 옵션이 가상 로봇의 검색 격리를 해제하지 않게 한다."""
    env = {"CAKE_USE_DISCOVERY_SERVER": "1", "ROS_DISCOVERY_SERVER": "host:11811"}
    launcher.configure_discovery("virtual", env)
    assert "ROS_DISCOVERY_SERVER" not in env
    assert env["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"


def test_pending_order_decline_preserves_record_and_sends_no_request(tmp_path, monkeypatch):
    """중단 확인을 하지 않으면 기존 주문을 보존하고 새 실행을 거부한다."""
    import sqlite3
    import pytest

    database = tmp_path / "orders.sqlite3"
    with sqlite3.connect(database) as connection:
        connection.execute("CREATE TABLE order_execution(order_id TEXT,state TEXT)")
        connection.execute("INSERT INTO order_execution VALUES ('ORD-TEST','uncertain')")
    def forbidden(*args, **kwargs):
        raise AssertionError("확인 없이 주문 해제 요청")
    monkeypatch.setattr(launcher, "urlopen", forbidden)
    with pytest.raises(RuntimeError, match="보존"):
        launcher.resolve_pending_orders(database, 5000, "test-token", lambda _: "NO")
    with sqlite3.connect(database) as connection:
        assert connection.execute("SELECT state FROM order_execution").fetchone()[0] == "uncertain"


def test_pending_order_reset_uses_authenticated_resolution_api(tmp_path, monkeypatch):
    """RESET은 주문 삭제·재전송 대신 기존 인증된 해제 API만 호출한다."""
    import io
    import json
    import sqlite3

    database = tmp_path / "orders.sqlite3"
    with sqlite3.connect(database) as connection:
        connection.execute("CREATE TABLE order_execution(order_id TEXT,state TEXT)")
        connection.execute("INSERT INTO order_execution VALUES ('ORD-TEST','uncertain')")
    requests = []
    def resolve(request, timeout):
        requests.append(request)
        return io.BytesIO(b'{"order_id":"ORD-TEST","status":"resolved"}')
    monkeypatch.setattr(launcher, "urlopen", resolve)
    launcher.resolve_pending_orders(database, 5000, "test-token", lambda _: "RESET")
    assert len(requests) == 1
    assert requests[0].full_url.endswith("/orders/ORD-TEST/resolve")
    assert requests[0].get_header("Authorization") == "Bearer test-token"
    assert json.loads(requests[0].data)["confirmed_not_running"] is True


def test_real_defaults_are_configured_without_contact_cli_flags():
    """실물 실행은 조정한 상면222mm와 YES 후 TCP 설정을 사용한다."""
    import json
    config = json.loads((PATH.parents[1] / "config/real_robot.json").read_text())
    assert config["cake_top_z_mm"] == 222.0
    assert config["configure_pen_tcp_on_yes"] is True
    # 거치대 위치와 구분한 케이크 중심에 BASE X -1mm, Y -0.8mm 보정을 반영한다.
    assert config["side_cake_center_xy_mm"] == [
        395.6743225097656,
        -51.35932998657227,
    ]


def test_side_tcp_exit_restore_uses_fresh_process(tmp_path, monkeypatch):
    """부모 ROS가 종료돼도 같은 검색 환경에서 별도 초기화 프로세스를 실행한다."""
    from types import SimpleNamespace
    from unittest.mock import Mock

    run = Mock(return_value=SimpleNamespace(returncode=0, stdout="기본 TCP 복원 완료\n"))
    monkeypatch.setattr(launcher.subprocess, "run", run)
    env = {"ROS_DOMAIN_ID": "30", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}
    launcher.restore_side_tcp_after_exit(
        SimpleNamespace(host="192.168.1.100", mode="real"), env, tmp_path,
    )
    command = run.call_args.args[0]
    assert "--reset-side-tcp" in command
    assert "--side-pen-45" not in command
    assert run.call_args.kwargs["env"] == env
    assert (tmp_path / "tcp_restore.log").read_text() == "기본 TCP 복원 완료\n"


def test_virtual_side_policy_isolates_robot_domain():
    """가상 옆면 검증은 실물 도메인30과 외부 주소를 사용할 수 없다."""
    from types import SimpleNamespace
    import pytest

    args = SimpleNamespace(side_pen_45=True, mode="virtual", domain_id=None, host=None)
    launcher.configure_side_test(args)
    assert args.domain_id == 187
    for fields in ({"domain_id": 30}, {"host": "192.168.1.100"}):
        args = SimpleNamespace(side_pen_45=True, mode="virtual", domain_id=None, host=None)
        for key, value in fields.items():
            setattr(args, key, value)
        with pytest.raises(ValueError):
            launcher.configure_side_test(args)


def test_tcp_cleanup_runs_after_app_before_owned_driver(tmp_path):
    """가상 업무 노드를 먼저 끄고 드라이버가 살아 있을 때 TCP를 복원한다."""
    processes = launcher.Processes(tmp_path, os.environ)
    try:
        processes.start("driver", [sys.executable, "-c", "import time; time.sleep(60)"])
        processes.start("drawing", [sys.executable, "-c", "import time; time.sleep(60)"])
        driver = processes.children[0][1]
        drawing = processes.children[1][1]
        called = []

        def restore():
            assert drawing.poll() is not None
            assert driver.poll() is None
            called.append(True)

        processes.close(grace=.2, terminate_grace=.2, before_driver_close=restore)
        assert called == [True]
        assert driver.poll() is not None
    finally:
        processes.close(grace=.1, terminate_grace=.1)


def _write_cake_files(tmp_path, real_config):
    """기준점 테스트용 real_robot.json과 최소 workspace_config.yaml을 만든다."""
    import json

    real = tmp_path / "real_robot.json"
    real.write_text(json.dumps(real_config), encoding="utf-8")
    workspace = tmp_path / "workspace_config.yaml"
    workspace.write_text(
        "robot_executor_node:\n"
        "  ros__parameters:\n"
        "    cake_center_pose: [401.8, -55.57, 510.39, 179.76, 179.35, 179.02]\n",
        encoding="utf-8",
    )
    return real, workspace


def test_config_cake_reference_uses_json_xy_and_taught_contact_pose(tmp_path):
    """XY는 real_robot.json, 접촉 Z·자세는 기존 케이크 중심 하강 자세에서 가져온다."""
    real, workspace = _write_cake_files(tmp_path, {
        "cake_top_z_mm": 222.0,
        "configure_pen_tcp_on_yes": True,
        "side_cake_center_xy_mm": [396.37, -50.56],
    })
    assert launcher.load_config_cake_reference(real, workspace) == [
        396.37, -50.56, 510.39, 179.76, 179.35, 179.02,
    ]


def test_config_cake_reference_requires_center_xy(tmp_path):
    """XY가 없는 단위 테스트용 설정으로 전체 공정을 시작하지 않는다."""
    import pytest

    real, workspace = _write_cake_files(tmp_path, {
        "cake_top_z_mm": 222.0, "configure_pen_tcp_on_yes": True,
    })
    with pytest.raises(RuntimeError, match="side_cake_center_xy_mm"):
        launcher.load_config_cake_reference(real, workspace)


def test_real_robot_config_rejects_out_of_range_top_z(tmp_path):
    """상면 높이 오타(예: 2220)는 로봇을 움직이기 전에 거부한다."""
    import pytest

    real, _ = _write_cake_files(tmp_path, {"cake_top_z_mm": 2220.0})
    with pytest.raises(RuntimeError, match="cake_top_z_mm"):
        launcher.load_real_robot_config(real)


def test_repository_real_robot_config_builds_reference():
    """저장소의 실제 real_robot.json으로 기준점을 만들 수 있어야 한다."""
    reference = launcher.load_config_cake_reference()
    assert len(reference) == 6


def test_paused_order_keeps_hmi_recovery_path_without_cli_reset(tmp_path):
    """복구 대기 주문 때문에 HMI 기동을 막거나 시작 시 실패 종료하지 않는다."""
    import sqlite3
    database = tmp_path / 'paused.sqlite3'
    with sqlite3.connect(database) as connection:
        connection.execute('CREATE TABLE order_execution(order_id TEXT,state TEXT)')
        connection.execute("INSERT INTO order_execution VALUES ('PAUSED-1','paused')")
    def forbidden(_):
        """복구 대기는 HMI 결정 대상이므로 터미널 RESET을 요구하면 실패한다."""
        raise AssertionError('복구 대기 주문에 CLI RESET을 요청했습니다.')
    launcher.resolve_pending_orders(database, 5000, 'test', forbidden)
    with sqlite3.connect(database) as connection:
        assert connection.execute('SELECT state FROM order_execution').fetchone()[0] == 'paused'
