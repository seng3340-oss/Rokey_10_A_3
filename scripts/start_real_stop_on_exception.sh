#!/usr/bin/env bash
# 강사 전체 공정 검증용: 재시도·관리자 재개 도입 전 소스와 ROS 인터페이스를
# 별도 설치본에서 실행한다. 현재 작업 트리, 개발 DB, 기존 빌드는 수정하지 않는다.
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
baseline_revision="31370d0"
baseline_root="$project_root/log/stop-on-exception-$baseline_revision"
baseline_marker="$baseline_root/.prepared-v1"

node_bin_dir="${CAKE_NODE_BIN_DIR:-$HOME/.local/share/cake-web-tooling/node_modules/.bin}"
if [[ -x "$node_bin_dir/node" ]]; then
  export PATH="$node_bin_dir:$PATH"
fi

# 병렬 기동이 같은 설치본을 만드는 것을 막는다. 로봇 실행 잠금은 원래
# 실행기의 공유 log/system/run.lock으로 따로 검사하며 빌드 잠금과 구분한다.
mkdir -p "$project_root/log"
exec 9>"$project_root/log/stop-on-exception-prepare.lock"
flock 9
if [[ ! -f "$baseline_marker" ]]; then
  if [[ -e "$baseline_root" ]]; then
    echo "미완성 테스트 실행본이 있습니다: $baseline_root" >&2
    echo "기존 내용을 확인한 뒤 별도로 보관하고 다시 실행하세요." >&2
    exit 1
  fi
  echo "예외 즉시 중단 버전($baseline_revision)을 별도로 준비합니다."
  mkdir -p "$baseline_root"
  git -C "$project_root" archive "$baseline_revision" \
    scripts backend frontend config ws_cobot1 | tar -x -C "$baseline_root"
  # 설치된 도구와 벤더만 공유한다. 업무 패키지 소스·빌드·프론트 소스는
  # 기준 커밋의 사본을 사용하므로 개발 중 변경이 실물 테스트에 섞이지 않는다.
  ln -s "$project_root/backend/.venv" "$baseline_root/backend/.venv"
  ln -s "$project_root/frontend/node_modules" "$baseline_root/frontend/node_modules"
  ln -s "$project_root/ws_dsr" "$baseline_root/ws_dsr"
  mkdir -p "$project_root/log/system" "$baseline_root/log"
  ln -s "$project_root/log/system" "$baseline_root/log/system"
  (
    # 상위 터미널의 개발용 오버레이를 빌드에 끌어오지 않는다.
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
    source /opt/ros/jazzy/setup.bash
    source "$project_root/ws_dsr/install/local_setup.bash"
    cd "$baseline_root/ws_cobot1"
    colcon build --packages-select cake_decorator_interfaces cake_decorator \
      --symlink-install >"$baseline_root/build-baseline.log" 2>&1 || {
        cat "$baseline_root/build-baseline.log" >&2
        exit 1
      }
  )
  printf '%s\n' "$baseline_revision" >"$baseline_marker"
fi
flock -u 9
exec 9>&-

# 실측 설정은 실행 직전에 복사한다. 기준 커밋의 제어 로직은 고정하되
# 사용자가 관리하는 현재 좌표·속도 설정은 공정 테스트에도 동일하게 적용한다.
cp "$project_root/config/real_robot.json" "$baseline_root/config/real_robot.json"
cp "$project_root/ws_cobot1/src/cake_decorator/config/workspace_config.yaml" \
  "$baseline_root/ws_cobot1/src/cake_decorator/config/workspace_config.yaml"
export CAKE_DATABASE_PATH="$baseline_root/backend/data/orders.sqlite3"

echo "------------------------------------------------------------"
echo "강사 공정 테스트: $baseline_revision / 예외 발생 시 재시도 없이 중단"
echo "개발 중인 복구·재개 기능은 이 실행에 포함되지 않습니다."
echo "주문 DB: $CAKE_DATABASE_PATH"
echo "------------------------------------------------------------"

# 주소 출력 함수만 개발 실행기에서 읽는다. 모듈 import는 main을 실행하지
# 않으므로 ROS 연결·이동·서버 기동 없이 접속 주소를 안내할 수 있다.
python3 - "$project_root/scripts/system_launcher.py" "$@" <<'PY_ADDRESSES'
import importlib.util
import sys

spec = importlib.util.spec_from_file_location("cake_address_log", sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
port = 8080
arguments = sys.argv[2:]
for index, argument in enumerate(arguments):
    if argument == "--frontend-port" and index + 1 < len(arguments):
        port = int(arguments[index + 1])
    elif argument.startswith("--frontend-port="):
        port = int(argument.split("=", 1)[1])
print("아래 주소는 실행기의 준비 완료 로그 이후 접속하세요.", flush=True)
module.print_web_addresses(port, ready=False)
PY_ADDRESSES

# 기준 커밋의 진입점을 직접 사용해 이 파일로 다시 분기하지 않는다.
# --no-browser는 브라우저 자동 실행만 막고 전체 공정·장비 검사는 유지한다.
exec bash "$baseline_root/scripts/start_real.sh" --no-browser "$@"
