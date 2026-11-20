"""전체 공정의 터미널 시작 관리. 다른 팀 프로세스는 변경하지 않는다."""

import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
# 이번 실행의 로그 파일. main()이 생성 후 기록하며, 중단 사유를 터미널뿐 아니라
# 로그에도 남기기 위해 진입점의 예외 처리에서 사용한다. 생성 전 실패면 None이다.
current_log = None
# 주문 전달 전 제어 주기 검사 시간[초]. 화면 기동 지연과 지연 검출 확률의 절충으로 60초에서
# 30초로 단축했다. 2초 간격으로 조회하므로 2의 배수로 둔다.
VALIDATION_SECONDS = 30


def network_check():
    """정해진 로봇·RG2 주소의 유선 경로와 TCP만 확인한다. 제어 데이터는 보내지 않는다."""
    route = json.loads(subprocess.check_output(
        ["ip", "-j", "route", "get", "192.168.1.100"], text=True,
    ))[0]
    device = route["dev"]
    interface = Path("/sys/class/net") / device
    if (interface / "wireless").exists() or device == "lo":
        raise RuntimeError("로봇 경로가 유선이 아닙니다. 랜선과 유선 IP를 확인하세요.")
    if (interface / "carrier").read_text().strip() != "1":
        raise RuntimeError("유선 랜 링크가 없습니다.")
    for host, port in (("192.168.1.100", 12345), ("192.168.1.1", 502)):
        with socket.create_connection((host, port), timeout=3):
            print(f"TCP 연결 확인: {host}:{port}", flush=True)


def own_driver(supervisor_pid):
    """이번 실행 관리자를 조상으로 가진 드라이버만 반환한다."""
    parents, commands = {}, {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            pid = int(entry.name)
            parents[pid] = int((entry / "stat").read_text().rsplit(")", 1)[1].split()[1])
            commands[pid] = (entry / "cmdline").read_bytes()
        except OSError:
            continue
    found = []
    for pid, command in commands.items():
        if not command.startswith(b"/opt/ros/jazzy/lib/controller_manager/ros2_control_node\0"):
            continue
        seen, ancestor = set(), pid
        while ancestor in parents and ancestor not in seen:
            seen.add(ancestor)
            ancestor = parents[ancestor]
        if supervisor_pid in seen:
            found.append(pid)
    if len(found) != 1:
        raise RuntimeError(f"이번 드라이버를 유일하게 식별하지 못했습니다: {found}")
    return found[0]


def tune_driver(pid, evidence):
    """조회로 식별한 100Hz 제어 타이머와 고부하 SDK 스레드만 배치한다.

    스레드 구성·CPU 구성이 예상과 다르면 추정해서 적용하지 않고 중단한다.
    root 권한은 식별된 제어 스레드 하나의 FIFO 우선순위에만 사용한다.
    """
    tasks = Path(f"/proc/{pid}/task")
    def sample():
        result = {}
        for task in tasks.iterdir():
            try:
                result[int(task.name)] = list(map(int, (task / "schedstat").read_text().split()))
            except OSError:
                pass
        return result
    before = sample()
    sleeping = set()
    begin = time.monotonic()
    for _ in range(20):
        for task in tasks.iterdir():
            try:
                if (task / "wchan").read_text().strip() == "hrtimer_nanosleep":
                    sleeping.add(int(task.name))
            except OSError:
                pass
        time.sleep(0.25)
    duration = time.monotonic() - begin
    after = sample()
    candidates, hot = [], []
    for tid, b in after.items():
        if tid not in before:
            continue
        a = before[tid]
        switches_per_second = (b[2] - a[2]) / duration
        if tid in sleeping and 70 <= switches_per_second <= 150:
            candidates.append(tid)
        if (b[0] - a[0]) / 1e9 > duration * 0.70:
            hot.append(tid)
    if len(candidates) != 1:
        raise RuntimeError(f"스레드 구성을 확인해야 합니다: control={candidates}, SDK={hot}")
    # 이 노트북의 물리 코어를 읽어 제어 코어와 통신 코어를 분리한다.
    physical = {}
    for cpu in sorted(os.sched_getaffinity(pid)):
        topology = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology")
        key = ((topology / "physical_package_id").read_text(),
               (topology / "core_id").read_text())
        physical.setdefault(key, []).append(cpu)
    cores = list(physical.values())
    if len(cores) < 4:
        raise RuntimeError("실제 제어에는 최소 4개 물리 코어가 필요합니다.")
    control = candidates[0]
    original = {str(t): sorted(os.sched_getaffinity(t)) for t in after}
    evidence.write_text(json.dumps({"driver_pid": pid, "control_tid": control,
                                   "sdk_tids": hot, "original_affinity": original}, indent=2))
    others = set(os.sched_getaffinity(pid)) - set(cores[1])
    for tid in after:
        target = others
        if tid == control:
            target = {cores[1][0]}
        elif tid in hot:
            target = {cores[(2 + hot.index(tid)) % len(cores)][0]}
            if target & set(cores[1]):
                target = {cores[0][0]}
        os.sched_setaffinity(tid, target)
    try:
        os.sched_setscheduler(control, os.SCHED_FIFO, os.sched_param(50))
    except PermissionError:
        print("이번 제어 스레드에만 우선순위 50을 설정합니다. 터미널에서 관리자 인증을 해주세요.", flush=True)
        subprocess.run(["sudo", "/usr/bin/chrt", "-f", "-p", "50", str(control)], check=True)
    if os.sched_getscheduler(control) != os.SCHED_FIFO or os.sched_getparam(control).sched_priority != 50:
        raise RuntimeError("실시간 우선순위 적용 확인에 실패했습니다.")
    print(f"이번 드라이버 설정 완료: PID={pid}, 제어 TID={control}, FIFO=50", flush=True)


def validate_loop(logfile):
    """실제 모드·STANDBY 응답과 VALIDATION_SECONDS(30초) 동안의 제어 경고를 확인한다.

    이동 명령은 없다. 2초 간격으로 상태를 조회하며, 정상이어도 일찍 끝내지 않고
    검사 구간 전체의 드라이버 로그에서 제어 주기 지연(Skip dt/Overrun)을 확인한다.
    """
    import rclpy
    from dsr_msgs2.srv import GetRobotState, GetRobotSystem
    rclpy.init()
    node = rclpy.create_node("full_process_start_validation")
    # 서비스 클라이언트는 서비스별로 한 번만 만들어 재사용한다.
    # 조회마다 만들고 지우면 DDS 매칭이 끝나기 전에 요청이 나가 유실되는 경우가 있어
    # (드라이버 로그에 콜백 호출조차 남지 않음) 정상 상태에서도 검사가 중단됐다.
    clients = {}

    def call(kind, name, attempts=3, timeout=5.0):
        """상태 서비스를 조회한다. 응답 없음·실패 응답은 attempts회까지 재시도한다.

        재시도는 일시적인 통신 유실만 흡수한다. 응답이 온 뒤의 상태 판정
        (실제 모드·STANDBY)은 호출자가 그대로 수행하므로 안전 조건은 완화되지 않는다.
        """
        if name not in clients:
            clients[name] = node.create_client(
                kind, f"/dsr01/dsr_controller2/system/{name}"
            )
        client = clients[name]
        reason = "응답 없음"
        for attempt in range(1, attempts + 1):
            if not client.wait_for_service(timeout_sec=timeout):
                reason = "서비스 없음"
            else:
                future = client.call_async(kind.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
                if future.done() and future.result() is not None and future.result().success:
                    return future.result()
                reason = "응답 없음" if not future.done() else "실패 응답"
                if not future.done():
                    # 늦게 도착한 응답이 다음 시도와 섞이지 않게 대기 중 요청을 취소한다.
                    client.remove_pending_request(future)
            if attempt < attempts:
                print(f"상태 조회 재시도 {attempt}/{attempts - 1}: {name} ({reason})", flush=True)
        raise RuntimeError(f"상태 조회 실패: {name} ({reason}, {attempts}회 시도)")

    try:
        if call(GetRobotSystem, "get_robot_system").robot_system != 0:
            raise RuntimeError("실제 로봇 모드가 아닙니다.")
        start = logfile.stat().st_size
        print(
            f"{VALIDATION_SECONDS}초 제어 주기 검사 중입니다. 주문 전달은 아직 차단돼 있습니다.",
            flush=True,
        )
        # 2초 간격 조회 횟수 = 검사 시간 / 2 (30초 → 15회).
        for _ in range(VALIDATION_SECONDS // 2):
            state = call(GetRobotState, "get_robot_state").robot_state
            if state != 1:
                # 1 = STANDBY. 실제 값을 남겨 서보 오프·보호 정지 등을 구분할 수 있게 한다.
                raise RuntimeError(f"로봇이 STANDBY 상태가 아닙니다. (robot_state={state})")
            time.sleep(2)
        with logfile.open("rb") as stream:
            stream.seek(start)
            text = stream.read().decode(errors="replace")
        if "Skip dt=" in text or "Overrun" in text:
            raise RuntimeError("제어 주기 지연이 남아 있습니다. 주문 전달을 허용하지 않습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    """전용 잠금·로그·주문 대기를 관리하며 종료 시 이번 실행기만 정리한다."""
    runtime = ROOT / ".runtime"
    runtime.mkdir(exist_ok=True)
    with (runtime / "full-terminal.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        network_check()
        if "--network-check" in sys.argv:
            return
        if "CAKE_STARTUP_GATE" not in (ROOT / "scripts/system_launcher.py").read_text():
            raise RuntimeError("주문 시작 대기가 있는 실행기가 필요합니다.")
        tag = time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:6]
        logfile = runtime / f"full-{tag}.log"
        global current_log
        current_log = logfile
        gate = runtime / f"control-ready-{tag}"
        env = dict(os.environ, CAKE_STARTUP_GATE=str(gate), PYTHONUNBUFFERED="1")
        process = subprocess.Popen([str(ROOT / "run.sh"), "full", *sys.argv[1:]],
                                   cwd=ROOT, env=env, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        def read_log():
            # 실행기 출력을 로그 파일과 터미널에 동시에 옮긴다.
            # readline()으로 줄 단위로 읽으면 줄바꿈 없는 `read -p` 안내 문구
            # ("... GO 입력")가 버퍼에 남아 화면에 보이지 않으므로,
            # 도착한 바이트를 즉시 전달한다. 빈 바이트는 실행기 출력 종료(EOF)다.
            fd = process.stdout.fileno()
            with logfile.open("wb", buffering=0) as output:
                for chunk in iter(lambda: os.read(fd, 4096), b""):
                    output.write(chunk)
                    sys.stdout.buffer.write(chunk)
                    sys.stdout.buffer.flush()
        reader = threading.Thread(target=read_log, daemon=True)
        reader.start()
        print(f"실행 로그: {logfile}", flush=True)
        try:
            # 현장 시작 확인 GO와 초기 기동을 기다린다. 제한 안에 확인하지 않으면 종료한다.
            end = time.monotonic() + 600
            while True:
                if process.poll() is not None:
                    raise RuntimeError(f"실행기가 종료됐습니다: {process.returncode}")
                if logfile.exists() and "제어 주기 검증 완료 대기:" in logfile.read_text(errors="replace"):
                    break
                if time.monotonic() > end:
                    raise RuntimeError("초기 기동 대기 시간 초과")
                time.sleep(0.5)
            tune_driver(own_driver(process.pid), runtime / f"tuning-{tag}.json")
            validate_loop(logfile)
            gate.touch()
            print("검사 통과. 주문 전달을 허용합니다. UI에서 윗면 제작 요청을 보내세요.", flush=True)
            if process.wait() != 0:
                raise RuntimeError("전체 공정 실행기 오류. 로그를 확인하세요.")
        finally:
            # SIGINT를 한 번 더 보내도 드라이버를 재시작하거나 다른 프로세스를 정리하지 않는다.
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    print("정상 종료 대기 중입니다. 물리적 안전 정지는 펜던트를 사용하세요.", flush=True)
            reader.join(timeout=2)
            gate.unlink(missing_ok=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n전체 공정 실행 종료 요청을 처리했습니다.")
    except Exception as error:
        print(f"실행 중단: {error}", file=sys.stderr)
        # 로그 기록 스레드는 이미 종료됐으므로 덧붙여도 출력과 섞이지 않는다.
        if current_log is not None:
            try:
                with current_log.open("a", encoding="utf-8") as stream:
                    stream.write(f"실행 중단: {error}\n")
            except OSError:
                pass
        sys.exit(1)