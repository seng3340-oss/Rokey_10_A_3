#!/usr/bin/env bash
# 실물 주소를 사용하지 않는 케이크 작업셀 가상 배치 전용 실행기다.
set -eo pipefail

# 현재 실행기의 위치를 기준으로 저장소를 찾아 사용자 홈 경로를 고정하지 않는다.
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

# 새 워크스페이스를 우선 사용한다. 미빌드 상태에서는 보존된 기존 설치본을 사용한다.
# 이 fallback은 모델/드라이버 재설치 없이 시각화를 확인하기 위한 명시적 임시 경로다.
if [[ -f "$project_root/ws_dsr/install/setup.bash" ]]; then
  source "$project_root/ws_dsr/install/setup.bash"
elif [[ -f "$HOME/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash" ]]; then
  echo "기존 로봇 설치본 사용: ~/ws_cobot_pjt_jihwan/ws_dsr/install"
  source "$HOME/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash"
else
  echo "먼저 ws_dsr의 로봇 실행환경을 빌드하세요." >&2
  exit 1
fi

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
