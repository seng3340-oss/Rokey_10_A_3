#!/usr/bin/env bash
# 프로젝트의 전체 공정을 준비하고 검증한 뒤 주문 전달을 허용한다.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
if [[ " ${*} " == *" --check "* ]]; then
  exec ./run.sh full "$@"
fi
source /opt/ros/jazzy/setup.bash
source "$project_root/ws_dsr/install/local_setup.bash"
source "$project_root/ws_cobot1/install/local_setup.bash"
export PATH="$project_root/.runtime/node/bin:$PATH"
export PYTHONPATH="$project_root/.runtime/python_deps:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_DISCOVERY_SERVER=127.0.0.1:11930
export ROS_SUPER_CLIENT=TRUE
export CAKE_USE_DISCOVERY_SERVER=1
exec python3 scripts/full_process_terminal.py "$@"
