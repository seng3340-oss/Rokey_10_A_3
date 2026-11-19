#!/usr/bin/env bash
# 이미 파지한 45도 펜으로 공중 옆면 추종을 시험한다.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" != "--help" && "${1:-}" != "-h" ]]; then
  source /opt/ros/jazzy/setup.bash
  source "${CAKE_ROBOT_SETUP:-$project_root/ws_dsr/install/local_setup.bash}"
  source "$project_root/ws_cobot1/install/local_setup.bash"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
fi
exec python3 "$project_root/scripts/real_side_air_test.py" "$@"
