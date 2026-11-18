#!/usr/bin/env bash
# 실물 주소를 사용하지 않는 케이크 작업셀 가상 배치 전용 실행기다.
set -eo pipefail

# 현재 실행기의 위치를 기준으로 저장소를 찾아 사용자 홈 경로를 고정하지 않는다.
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

# 현재 저장소 설치본을 사용한다. 별도 강의 설치본은 명시적으로 지정한다.
robot_setup="${CAKE_ROBOT_SETUP:-$project_root/ws_dsr/install/local_setup.bash}"
if [[ ! -f "$robot_setup" ]]; then
  bash "$project_root/scripts/setup_project.sh"
fi
source "$robot_setup"

if [[ ! -f "$project_root/ws_cobot1/install/local_setup.bash" ]]; then
  echo "먼저 ws_cobot1의 cake_workcell 패키지를 빌드하세요." >&2
  exit 1
fi
# setup.bash의 과거 underlay 체인 대신 현재 선택한 로봇 환경 위에 패키지만 추가한다.
source "$project_root/ws_cobot1/install/local_setup.bash"
export ROS_DOMAIN_ID=30
# 강의 launch가 동명 컨테이너를 정리하므로 이미 켜진 가상 세션을 덮어쓰지 않는다.
if [[ "$(docker inspect --format '{{.State.Running}}' dsr01_emulator 2>/dev/null)" == "true" ]]; then
  echo "dsr01 가상 컨트롤러가 이미 실행 중입니다. 기존 sodvir/통합 launch를 종료한 뒤 실행하세요." >&2
  exit 1
fi
exec ros2 launch cake_workcell_bringup virtual.launch.py "$@"
