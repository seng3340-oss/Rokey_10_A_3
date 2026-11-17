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
# 가상·실제 모드의 검색 정책은 Python에서 ROS 초기화 전에 함께 적용한다.
# source한 환경의 외부 검색 서버가 실제 모드에만 남는 불일치를 막는다.
exec python3 "$project_root/scripts/system_launcher.py" --mode "$mode" "$@"
