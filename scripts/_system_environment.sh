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
# setup 파일만 남은 미완성 빌드를 선택하지 않는다. 각 후보의 실제 패키지와
# 실행 파일을 별도 셸에서 검사해 현재 터미널 환경을 섞지 않는다.
robot_environment_ready() (
  candidate="$1"
  [[ -f "$candidate" ]] || exit 1
  source "$candidate" >/dev/null 2>&1 || exit 1
  python3 - "$candidate" <<'PY_READY'
import os
from pathlib import Path
import sys
from ament_index_python.packages import get_package_prefix
root = Path(sys.argv[1]).resolve().parent
try:
    for package in (
        "m0609_rg2_bringup", "dsr_controller2", "dsr_hardware2", "dsr_msgs2",
        "dsr_description2", "dsr_common2", "onrobot_rg_control", "onrobot_rg_msgs",
    ):
        prefix = Path(get_package_prefix(package)).resolve()
        if not prefix.is_relative_to(root):
            raise ValueError("다른 설치본의 패키지가 섞여 있습니다.")
    for package, executable in (
        ("m0609_rg2_bringup", "gripper_joint_state_publisher.py"),
        ("onrobot_rg_control", "OnRobotRGControllerServer"),
    ):
        program = Path(get_package_prefix(package)) / "lib" / package / executable
        if not program.is_file() or not os.access(program, os.X_OK):
            raise ValueError("필수 실행 파일이 준비되지 않았습니다.")
except (LookupError, ValueError):
    sys.exit(1)
PY_READY
)

robot_setup="${CAKE_ROBOT_SETUP:-}"
if [[ -n "$robot_setup" ]]; then
  if ! robot_environment_ready "$robot_setup"; then
    echo "직접 지정한 로봇 설치본이 불완전합니다: $robot_setup" >&2
    exit 1
  fi
else
  for candidate in \
    "$project_root/ws_dsr/install/local_setup.bash" \
    "$HOME/ws_cobot_pjt_jihwan/ws_dsr/install/local_setup.bash"; do
    if robot_environment_ready "$candidate"; then
      robot_setup="$candidate"
      break
    fi
  done
fi
if [[ -z "$robot_setup" || ! -f "$project_root/ws_cobot1/install/local_setup.bash" ]]; then
  echo "완성된 ROS 빌드 결과가 없습니다. README의 로봇 환경 빌드 순서를 확인하세요." >&2
  exit 1
fi
echo "로봇 환경 자동 준비: $robot_setup"
source "$robot_setup"
source "$project_root/ws_cobot1/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
# 가상·실제 모드의 검색 정책은 Python에서 ROS 초기화 전에 함께 적용한다.
# source한 환경의 외부 검색 서버가 실제 모드에만 남는 불일치를 막는다.
exec python3 "$project_root/scripts/system_launcher.py" --mode "$mode" "$@"
