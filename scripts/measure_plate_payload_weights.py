#!/usr/bin/env python3
"""플레이트 시퀀스의 Safe Position별 보유 물품 하중 측정 도구.

이 스크립트는 기존 파일을 수정하지 않는다. 원본 PlateSequence의 이동
순서와 좌표를 그대로 사용하되, 하중 판정 메서드만 하위 클래스에서
"기록 전용(비차단)"으로 바꾼다.

측정 대상은 자세가 아니라 같은 Safe 자세에서 그리퍼가 들고 있는 물품이다.
- prepare_start            : 빈 그리퍼 기준값
- prepare_opener           : 따개
- prepare_both             : 따개 + 케이크
- prepare_opener_after_cake: 따개 (케이크 내려놓은 뒤)
- prepare_end              : 빈 그리퍼 (따개 반납 뒤)
- finish_start             : 빈 그리퍼 기준값
- finish_opener            : 따개
- finish_both              : 따개 + 케이크
- finish_opener_after_cake : 따개 (케이크 출고 뒤)
- finish_end               : 빈 그리퍼 (따개 반납 뒤)

Safe Position 1~5는 설정상 이름만 다르고 현재 좌표는 모두 같다. 따라서
이 스크립트는 5개 자세를 비교하지 않고, 같은 자세에서 물품 상태가 바뀔
때의 하중 변화를 3회 반복 측정한다.

안전:
  - 실행 전 로봇이 STANDBY(1)인지 확인한다.
  - 각 회차는 prepare → finish 전체 시퀀스를 수행하므로 실제 로봇이 움직인다.
  - 회차 사이에는 작업자가 오프너와 케이크를 초기 위치로 되돌려야 한다.
  - 원본 코드의 이동/좌표/그리퍼 동작은 변경하지 않는다.
"""

import argparse
from datetime import datetime
from pathlib import Path
import statistics
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml

from dsr_msgs2.srv import GetRobotState


ROOT = Path(__file__).resolve().parent.parent
SOURCE_PACKAGE_ROOT = ROOT / "ws_cobot1/src/cake_decorator"
DEFAULT_CONFIG = (
    ROOT / "ws_cobot1/src/cake_decorator/config/workspace_config.yaml"
)
STATE_SERVICE = "/dsr01/dsr_controller2/system/get_robot_state"
STATE_STANDBY = 1

# 원본 패키지를 소스에서 직접 불러온다. 설치는 --symlink-install 기준이지만,
# 소스 경로를 먼저 두어 리뷰 중인 파일과 실행 파일이 어긋나지 않게 한다.
sys.path.insert(0, str(SOURCE_PACKAGE_ROOT))

from cake_decorator.doosan_robot import DoosanRobotClient  # noqa: E402
from cake_decorator.plate_sequence import PlateSequence  # noqa: E402
from cake_decorator.robot_executor_node import RobotExecutorNode  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="플레이트 시퀀스 보유 물품 하중 반복 측정",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--runs", type=int, default=3, help="전체 반복 횟수")
    parser.add_argument(
        "--timeout", type=float, default=180.0, help="서비스 응답 제한(초)"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "log/weight_measurements",
        help="CSV 저장 디렉터리",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="움직이지 않고 측정 단계와 좌표만 확인",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="실제 로봇 동작을 승인한다. 없으면 계획만 출력한다.",
    )
    parser.add_argument(
        "--no-pause",
        action="store_true",
        help="회차 사이 작업자 확인 입력을 생략한다(연속 시험용).",
    )
    parser.add_argument(
        "--burst-seconds",
        type=float,
        default=3.0,
        help="Safe Position 도착 후 연속 측정 시간(초). 0이면 단일 측정",
    )
    parser.add_argument(
        "--burst-interval",
        type=float,
        default=0.3,
        help="연속 측정 간격(초)",
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=10.0,
        help="로봇 상태 확인 1회 제한 시간(초)",
    )
    parser.add_argument(
        "--state-retries",
        type=int,
        default=3,
        help="로봇 상태 확인 재시도 횟수",
    )
    parser.add_argument(
        "--reads",
        type=int,
        default=0,
        help="단계별 고정 측정 횟수. 0보다 크면 burst-seconds 대신 사용",
    )
    parser.add_argument(
        "--reset-wait",
        type=float,
        default=0.0,
        help="회차 사이 물품 리셋 대기(초). 0이면 Enter 입력을 기다림",
    )
    return parser.parse_args()


def load_params(config_path: Path) -> dict:
    """YAML 좌표와 노드 기본값을 합쳐 PlateSequence용 파라미터를 만든다.

    YAML에 없는 키는 RobotExecutorNode의 기본값으로 채운다. 하중 검사는
    측정이 목적이므로 설정값과 관계없이 활성화한다.
    """
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    yaml_params = (
        document.get("robot_executor_node", {}).get("ros__parameters", {})
    )
    params = dict(RobotExecutorNode._PARAM_DEFAULTS)
    params.update(yaml_params)
    params["plate_weight_check_enabled"] = True
    return params


def call_service(client, request, timeout: float):
    """이미 executor가 도는 상태에서 서비스 응답을 제한 시간까지 기다린다."""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    ready = threading.Event()
    future.add_done_callback(lambda _: ready.set())
    if not ready.wait(timeout):
        return None
    return future.result()


def read_state_with_retry(client, timeout: float, retries: int):
    """연결이 잠시 끊긴 경우를 대비해 로봇 상태를 여러 번 확인한다.

    측정 시작 전 안전 게이트이므로, 모든 시도가 실패하면 None을 반환하고
    호출부가 로봇을 움직이지 않고 종료하게 한다.
    """
    for attempt in range(1, max(1, retries) + 1):
        print(f"로봇 상태 확인 {attempt}/{max(1, retries)}...")
        state = call_service(client, GetRobotState.Request(), timeout)
        if state is not None and state.success:
            return state
        if attempt < max(1, retries):
            time.sleep(2.0)
    return None


class RecordingPlateSequence(PlateSequence):
    """원본 이동 시퀀스를 그대로 쓰고 하중만 기록하는 하위 클래스.

    원본 _weight_at_safe는 기준값과 허용 오차를 비교해 실패 시 False를
    반환한다. 이 클래스는 같은 위치에서 하중을 읽기만 하고 항상 True를
    반환하므로, 측정 중에는 공정이 중단되지 않는다.
    """

    def __init__(
        self,
        robot,
        param,
        run_index: int,
        records: list,
        burst_seconds: float = 0.0,
        burst_interval: float = 0.3,
        fixed_reads: int = 0,
    ):
        super().__init__(robot, param)
        self._run_index = run_index
        self._records = records
        self._baseline_kg = None
        self._phase = None
        self._burst_seconds = max(0.0, float(burst_seconds))
        self._burst_interval = max(0.05, float(burst_interval))
        self._fixed_reads = max(0, int(fixed_reads))

    def _sample(self) -> list[tuple[float, float | None]]:
        """도착 직후부터 지정 시간 동안 반복 측정한 (경과초, 하중) 목록.

        burst_seconds가 0이면 기존처럼 정착 대기 후 1회만 측정한다.
        burst 모드에서는 대기 없이 도착 즉시 첫 측정을 시작한다.
        """
        # 고정 횟수 모드: 회차·단계마다 정확히 N회 측정한다.
        if self._fixed_reads > 0:
            samples = []
            start = time.monotonic()
            for index in range(self._fixed_reads):
                target = start + index * self._burst_interval
                now = time.monotonic()
                if now < target:
                    time.sleep(target - now)
                elapsed = time.monotonic() - start
                samples.append(
                    (elapsed, self._robot.get_workpiece_weight())
                )
            return samples

        if self._burst_seconds <= 0.0:
            time.sleep(float(self._param("plate_weight_wait_sec")))
            return [(0.0, self._robot.get_workpiece_weight())]

        samples = []
        start = time.monotonic()
        while True:
            target = start + len(samples) * self._burst_interval
            now = time.monotonic()
            if now < target:
                time.sleep(target - now)
            elapsed = time.monotonic() - start
            samples.append((elapsed, self._robot.get_workpiece_weight()))
            if time.monotonic() - start >= self._burst_seconds:
                break
        return samples

    @staticmethod
    def _representative(samples: list[tuple[float, float | None]]):
        """버스트 샘플의 대표값으로 중앙값을 사용한다."""
        values = [weight for _, weight in samples if weight is not None]
        return statistics.median(values) if values else None

    def _weight_at_safe(self, stage: str) -> bool:
        samples = self._sample()
        weight = self._representative(samples)

        if stage in ("prepare_start", "finish_start"):
            self._phase = "prepare" if stage == "prepare_start" else "finish"
            self._baseline_kg = weight
            self._record(stage, weight, weight, None, samples)
            return True

        delta = None
        if weight is not None and self._baseline_kg is not None:
            delta = weight - self._baseline_kg
        self._record(stage, weight, self._baseline_kg, delta, samples)
        return True

    def _record(self, stage, weight, baseline, delta, samples):
        """측정값을 목록과 로그에 남긴다. 실패해도 다음 단계로 넘어간다."""
        record = {
            "run": self._run_index,
            "phase": self._phase,
            "stage": stage,
            "weight_kg": weight,
            "baseline_kg": baseline,
            "delta_kg": delta,
            "samples": samples,
        }
        self._records.append(record)

        weight_text = "측정 실패" if weight is None else f"{weight:.6f}"
        baseline_text = "없음" if baseline is None else f"{baseline:.6f}"
        delta_text = "없음" if delta is None else f"{delta:+.6f}"
        sample_text = " ".join(
            f"{elapsed:.2f}:"
            f"{'-' if value is None else f'{value:.6f}'}"
            for elapsed, value in samples
        )
        self._robot._logger.info(
            f"[Payload Weight] run={self._run_index} "
            f"phase={self._phase} stage={stage} "
            f"weight={weight_text} baseline={baseline_text} "
            f"delta={delta_text} samples=[{sample_text}]"
        )


def stats(values: list[float]):
    if not values:
        return None
    return {
        "count": len(values),
        "mean": statistics.mean(values),
        "std": statistics.stdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
        "range": max(values) - min(values),
    }


def print_stage_stats(records: list):
    """단계별 버스트 샘플의 최소·중앙·최대·범위·표준편차를 출력한다.

    burst 모드에서는 한 단계당 여러 샘플이 있으므로, 단일 측정값이 아니라
    도착 후 3초 동안의 실제 흔들림을 그대로 보여 준다.
    """
    print("\n=== 단계별 샘플 통계 (kg) ===")
    print(
        f"{'stage':30s} {'n':>4s} {'min':>11s} {'median':>11s} "
        f"{'max':>11s} {'range':>11s} {'std':>10s}"
    )

    stages = []
    for record in records:
        if record["stage"] not in stages:
            stages.append(record["stage"])

    for stage in stages:
        values = []
        for record in records:
            if record["stage"] != stage:
                continue
            for _, weight in record.get("samples", []):
                if weight is not None:
                    values.append(weight)
        summary = stats(values)
        if summary is None:
            print(f"{stage:30s}    0  측정 성공값 없음")
            continue
        print(
            f"{stage:30s} {summary['count']:4d} "
            f"{summary['min']:11.6f} {statistics.median(values):11.6f} "
            f"{summary['max']:11.6f} {summary['range']:11.6f} "
            f"{summary['std']:10.6f}"
        )


def save_csv(records: list, output_dir: Path) -> Path:
    """3회 측정 원본을 CSV로 남겨 추후 임계값 산출에 사용한다."""
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = output_dir / f"plate_payload_weights_{stamp}.csv"
    with path.open("w", encoding="utf-8") as stream:
        stream.write("run,phase,stage,weight_kg,baseline_kg,delta_kg\n")
        for record in records:
            def value(item):
                return "" if item is None else f"{item:.6f}"

            stream.write(
                f"{record['run']},{record['phase']},{record['stage']},"
                f"{value(record['weight_kg'])},{value(record['baseline_kg'])},"
                f"{value(record['delta_kg'])}\n"
            )
    return path


def save_samples_csv(records: list, output_dir: Path) -> Path:
    """단계별 버스트 원본 샘플을 별도 CSV로 저장한다."""
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = output_dir / f"plate_payload_weight_samples_{stamp}.csv"
    with path.open("w", encoding="utf-8") as stream:
        stream.write("run,phase,stage,sample_index,elapsed_s,weight_kg\n")
        for record in records:
            for index, (elapsed, weight) in enumerate(
                record.get("samples", [])
            ):
                weight_text = "" if weight is None else f"{weight:.6f}"
                stream.write(
                    f"{record['run']},{record['phase']},{record['stage']},"
                    f"{index},{elapsed:.3f},{weight_text}\n"
                )
    return path


def main():
    args = parse_args()
    params = load_params(args.config)

    safe_positions = []
    for index in range(1, 6):
        name = f"plate_safe_position_{index}"
        safe_positions.append((name, params.get(name)))

    print("플레이트 보유 물품 하중 반복 측정")
    print(f"설정 파일: {args.config}")
    print(f"실행 횟수: {args.runs}")
    if args.reads > 0:
        print(
            f"단계별 고정 측정: {args.reads}회, "
            f"간격 {args.burst_interval:.2f}초"
        )
    elif args.burst_seconds > 0:
        print(
            f"버스트 측정: 도착 직후 {args.burst_seconds:.1f}초 동안 "
            f"{args.burst_interval:.2f}초 간격"
        )
    else:
        print("측정 방식: 정착 대기 후 단일 측정")
    if args.runs > 1:
        if args.reset_wait > 0:
            print(f"회차 간 리셋 대기: {args.reset_wait:.0f}초")
        else:
            print("회차 간 리셋: Enter 입력 대기")
    print("Safe Position 좌표(기존 코드 값):")
    for name, joints in safe_positions:
        print(f"  {name}: {joints}")
    print(
        "측정 단계: prepare_start → prepare_opener → prepare_both → "
        "prepare_opener_after_cake → prepare_end → finish_start → "
        "finish_opener → finish_both → finish_opener_after_cake → finish_end"
    )

    if len({tuple(joints or []) for _, joints in safe_positions}) == 1:
        print("  참고: Safe Position 1~5는 같은 좌표입니다(같은 측정 자세).")

    if args.dry_run or not args.yes:
        print("\n로봇을 움직이지 않았습니다. 실제 측정은 --yes가 필요합니다.")
        return 0

    rclpy.init()
    node = Node("plate_payload_weight_measurement")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    records = []
    robot = None
    try:
        state_client = node.create_client(
            GetRobotState,
            STATE_SERVICE,
            callback_group=ReentrantCallbackGroup(),
        )
        robot = DoosanRobotClient(
            node,
            service_timeout=float(params.get("service_timeout", 5.0)),
            gripper_open_do=int(params.get("gripper_open_do", 2)),
            gripper_close_do=int(params.get("gripper_close_do", 1)),
            gripper_pulse_sec=float(params.get("gripper_pulse_sec", 0.5)),
            gripper_virtual_fallback=bool(
                params.get("gripper_virtual_fallback", True)
            ),
        )

        for run in range(1, args.runs + 1):
            print(f"\n=== 실행 {run}/{args.runs} ===")
            state = read_state_with_retry(
                state_client, args.state_timeout, args.state_retries
            )
            if state is None or not state.success:
                print(
                    "로봇 상태를 읽지 못했습니다. 연결/드라이버를 확인한 뒤 "
                    "다시 실행하세요.",
                    file=sys.stderr,
                )
                return 2
            if int(state.robot_state) != STATE_STANDBY:
                print(
                    f"STANDBY(1)가 아닙니다: {state.robot_state}. "
                    "측정을 시작하지 않습니다.",
                    file=sys.stderr,
                )
                return 2

            sequence = RecordingPlateSequence(
                robot,
                params.__getitem__,
                run,
                records,
                burst_seconds=args.burst_seconds,
                burst_interval=args.burst_interval,
                fixed_reads=args.reads,
            )
            print("[prepare] 시작")
            if not sequence._plate_prepare_steps():
                print("prepare 이동 실패. 측정을 중단합니다.", file=sys.stderr)
                return 3
            print("[finish] 시작")
            if not sequence._plate_finish_steps():
                print("finish 이동 실패. 측정을 중단합니다.", file=sys.stderr)
                return 3

            if run < args.runs:
                if args.no_pause:
                    print("다음 회차를 바로 시작합니다.")
                elif args.reset_wait > 0:
                    # 작업자가 케이크를 초기 위치로 되돌릴 시간을 준다.
                    # 대기 후 자동으로 다음 회차를 시작한다.
                    print(
                        "오프너와 케이크를 초기 위치로 되돌리세요. "
                        f"{args.reset_wait:.0f}초 후 다음 회차를 시작합니다."
                    )
                    for remaining in range(int(args.reset_wait), 0, -1):
                        print(
                            f"  남은 시간: {remaining:3d}초",
                            end="\r",
                            flush=True,
                        )
                        time.sleep(1)
                    print(" " * 32, end="\r", flush=True)
                    print("다음 회차 시작")
                else:
                    try:
                        input(
                            "오프너와 케이크를 초기 위치로 되돌린 뒤 "
                            "Enter를 누르세요..."
                        )
                    except EOFError:
                        print(
                            "작업자 확인을 받을 수 없어 중단합니다. "
                            "연속 시험은 --no-pause 또는 --reset-wait를 "
                            "사용하세요.",
                            file=sys.stderr,
                        )
                        return 4
    finally:
        # 스핀 스레드가 끝난 뒤 노드를 파괴해야 종료 시점에 rclpy C++ 객체가
        # std::terminate를 내지 않는다. 실패 경로에서도 동일하게 정리한다.
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        rclpy.try_shutdown()

    print_stage_stats(records)
    path = save_csv(records, args.output_dir)
    print(f"\nCSV 저장: {path}")
    samples_path = save_samples_csv(records, args.output_dir)
    print(f"샘플 CSV 저장: {samples_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
