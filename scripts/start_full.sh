#!/usr/bin/env bash
# 프로젝트의 전체 공정을 준비하고 검증한 뒤 주문 전달을 허용한다.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
if [[ " $* " == *" --help "* || " $* " == *" -h "* ]]; then
  echo "사용법: ./start_full.sh [--check]"
  echo "--check는 준비 상태만 확인합니다. 실제 시작은 현장 확인 후 ./start_full.sh를 사용하세요."
  exit 0
fi
if [[ " ${*} " == *" --check "* ]]; then
  exec ./run.sh full "$@"
fi
source "$project_root/scripts/project_environment.sh"
for setup in /opt/ros/jazzy/setup.bash "${CAKE_ROBOT_SETUP:-$project_root/ws_dsr/install/local_setup.bash}" "$project_root/ws_cobot1/install/local_setup.bash"; do
  [[ -f "$setup" ]] || { echo "설치가 필요합니다: bash scripts/setup_project.sh --install-system" >&2; exit 1; }
done
source /opt/ros/jazzy/setup.bash
source "${CAKE_ROBOT_SETUP:-$project_root/ws_dsr/install/local_setup.bash}"
source "$project_root/ws_cobot1/install/local_setup.bash"
source "$project_root/scripts/project_environment.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_DISCOVERY_SERVER=127.0.0.1:11930
export ROS_SUPER_CLIENT=TRUE
export CAKE_USE_DISCOVERY_SERVER=1
exec python3 scripts/full_process_terminal.py "$@"
