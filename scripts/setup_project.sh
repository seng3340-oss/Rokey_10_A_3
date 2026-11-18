#!/usr/bin/env bash
# 첫 실행 준비. 로봇에 연결하지 않고 이 저장소 위치에서 의존성과 ROS 패키지를 준비한다.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy가 필요합니다. README의 최초 준비를 확인하세요." >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
export PATH="$HOME/.local/share/cake-web-tooling/node_modules/.bin:$PATH"
command -v colcon >/dev/null || { echo "python3-colcon-common-extensions를 설치하세요." >&2; exit 1; }
command -v node >/dev/null || { echo "Node.js 22.12 이상과 npm을 설치하세요." >&2; exit 1; }
node -e 'const [a,b]=process.versions.node.split(".").map(Number);if(a<22||(a===22&&b<12)){throw Error("Node.js 22.12 이상이 필요합니다")}'
python3 -c 'from pymodbus.client import ModbusTcpClient; import scipy, yaml' || {
  echo "sudo apt install python3-pymodbus python3-scipy python3-yaml 로 ROS Python 의존성을 설치하세요." >&2
  exit 1
}
if [[ ! -f "$project_root/frontend/node_modules/vite/bin/vite.js" ]]; then
  (cd "$project_root/frontend" && npm ci)
fi
if [[ ! -x "$project_root/backend/.venv/bin/python" ]]; then
  python3 -m venv "$project_root/backend/.venv"
fi
if ! "$project_root/backend/.venv/bin/python" -c 'import fastapi,uvicorn,pydantic' 2>/dev/null; then
  "$project_root/backend/.venv/bin/python" -m pip install -r "$project_root/backend/requirements.txt"
fi
# symlink-install에서는 원본 실행 권한도 필요하다.
chmod +x "$project_root"/ws_dsr/src/rg2/m0609_rg2_bringup/scripts/*.py
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
(cd "$project_root/ws_dsr" && colcon build --symlink-install --parallel-workers 2 \
  --packages-up-to dsr_controller2 dsr_bringup2 m0609_rg2_bringup onrobot_rg_control)
source "$project_root/ws_dsr/install/local_setup.bash"
(cd "$project_root/ws_cobot1" && colcon build --symlink-install --parallel-workers 2 \
  --packages-up-to cake_decorator cake_workcell_bringup)
echo "프로젝트 준비 완료."
