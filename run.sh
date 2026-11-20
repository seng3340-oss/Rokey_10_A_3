#!/usr/bin/env bash
# 실제 로봇 실행기의 공통 진입점.
# - 호출자: scripts/start_full.sh(--check), scripts/full_process_terminal.py(실제 시작)
# - 역할: 프로젝트 폴더의 도구·ROS 검색 환경만 준비하고 모드별 공식 실행기로 넘긴다.
#   기존 팀 프로세스와 시스템 설정은 변경하지 않는다.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"

# Node/Python 의존성 경로는 복제 위치와 무관하게 project_environment.sh가 선택한다
# (.runtime/ros-venv 우선, 구버전 .runtime/python_deps 호환).
source "$project_root/scripts/project_environment.sh"
export CAKE_ROBOT_SETUP="${CAKE_ROBOT_SETUP:-$project_root/ws_dsr/install/local_setup.bash}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"

# 실제 작업 노드는 기존 팀 노드와 섞이지 않게 로컬 전용 검색 서버(127.0.0.1:11930)를 쓴다.
export ROS_DISCOVERY_SERVER=127.0.0.1:11930
export ROS_SUPER_CLIENT=TRUE
export CAKE_USE_DISCOVERY_SERVER=1

# 준비 검사·도움말에서는 검색 서버를 띄우지 않는다. 실제 시작 때만 필요하다.
# UDP 11930 포트를 바인드해 보고, 비어 있을 때만 fastdds 검색 서버를 백그라운드로 띄운다.
# 이미 사용 중(EADDRINUSE)이면 기존 서버를 재사용한다.
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
    # 실행기 세션과 분리(start_new_session)해 Ctrl+C로 함께 죽지 않게 한다.
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

# 운반·펜 순환 활성화는 각 공식 실행기가 담당한다. 나머지 인자는 그대로 전달한다.
mode="${1:-full}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  full) exec bash scripts/start_real_full_process.sh "$@" ;;
  drawing) exec bash scripts/start_real.sh "$@" ;;
  check) exec bash scripts/start_real.sh --check "$@" ;;
  *) echo "사용법: ./run.sh {full|drawing|check} [실행 옵션]" >&2; exit 2 ;;
esac
