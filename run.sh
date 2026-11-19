#!/usr/bin/env bash
# 프로젝트 폴더의 환경만 준비한다. 기존 팀 프로세스와 설정은 변경하지 않는다.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
export PATH="$project_root/.runtime/node/bin:$PATH"
export PYTHONPATH="$project_root/.runtime/python_deps:${PYTHONPATH:-}"
export CAKE_ROBOT_SETUP="${CAKE_ROBOT_SETUP:-$project_root/ws_dsr/install/local_setup.bash}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
# 실제 작업 노드는 기존 팀 노드와 섞이지 않게 로컬 전용 검색 서버를 쓴다.
export ROS_DISCOVERY_SERVER=127.0.0.1:11930
export ROS_SUPER_CLIENT=TRUE
export CAKE_USE_DISCOVERY_SERVER=1
# 준비 검사는 실행하지 않는다. 실제 시작 때만 필요한 검색 서버를 준비한다.
if [[ "${1:-full}" != "check" && " $* " != *" --check "* && " $* " != *" --help "* ]]; then
  python3 - "$project_root" <<'PY_DISCOVERY'
import errno
from pathlib import Path
import socket
import subprocess
import sys

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
    try:
        probe.bind(("127.0.0.1", 11930))
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        available = False
    else:
        available = True
if available:
    log_path = Path(sys.argv[1]) / ".runtime/discovery.log"
    with log_path.open("ab", buffering=0) as log:
        subprocess.Popen(
            ["/opt/ros/jazzy/bin/fastdds", "discovery", "-i", "0",
             "-l", "127.0.0.1", "-p", "11930"],
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
PY_DISCOVERY
fi
# 운반·펜 순환 활성화는 각 공식 실행기가 담당한다.
mode="${1:-full}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  full) exec bash scripts/start_real_full_process.sh "$@" ;;
  drawing) exec bash scripts/start_real.sh "$@" ;;
  check) exec bash scripts/start_real.sh --check "$@" ;;
  *) echo "사용법: ./run.sh {full|drawing|check} [실행 옵션]" >&2; exit 2 ;;
esac
