#!/usr/bin/env bash
# 두 진입점이 같은 ROS 환경을 사용하게 한다. 기존 alias나 .bashrc는 수정하지 않는다.
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="$1"
export CAKE_SYSTEM_MODE="$mode"
shift
if [[ " $* " == *" --help "* || " $* " == *" -h "* ]]; then
  exec python3 "$project_root/scripts/system_launcher.py" --mode "$mode" "$@"
fi
source /opt/ros/jazzy/setup.bash
robot_setup="${CAKE_ROBOT_SETUP:-$project_root/ws_dsr/install/setup.bash}"
if [[ ! -f "$robot_setup" && -z "${CAKE_ROBOT_SETUP:-}" ]]; then
  robot_setup="$HOME/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash"
fi
if [[ ! -f "$robot_setup" || ! -f "$project_root/ws_cobot1/install/local_setup.bash" ]]; then
  echo "ROS 빌드 결과가 없습니다. README의 로봇 환경과 ws_cobot1 빌드 순서를 확인하세요." >&2
  exit 1
fi
source "$robot_setup"
source "$project_root/ws_cobot1/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
if [[ "$mode" == virtual ]]; then
  # 교육장 검색 서버에 연결하지 않고 이 컴퓨터 안의 가상 노드끼리만 통신한다.
  unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
fi
exec python3 "$project_root/scripts/system_launcher.py" --mode "$mode" "$@"
