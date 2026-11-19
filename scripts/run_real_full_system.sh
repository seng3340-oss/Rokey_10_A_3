#!/usr/bin/env bash
# 실제 로봇에서 전체 시스템을 한 번에 띄우는 점검용 실행기.
#
# 실행 순서는 start_real.sh와 같다.
#   드라이버 → 백엔드 → 그림 경로/작업 관리/로봇 실행 노드 → 주문 브리지 → 프론트엔드
#
# 이 스크립트의 목적
#   - 프론트엔드 화면(http://localhost:8080)이 실제 로봇 PC에서 뜨는지 확인한다.
#   - 주문 접수는 하지 않는다. 즉, 웹 주문을 자동으로 만들지 않고 드로잉도 실행하지 않는다.
#   - 그래서 YES 영점 확인 없이 현재 TCP를 기준점으로 기록하는 미리보기 모드로 띄운다.
#     (system_launcher.py의 --preview-no-order)
#
# 주의
#   - 미리보기 기준점은 케이크 상면 중앙이 아닐 수 있다. 이 상태에서 웹으로 제작 요청을
#     보내면 로봇이 엉뚱한 위치로 움직일 수 있으므로 실제 주문을 넣지 않는다.
#   - 실제 제작은 `bash scripts/start_real.sh`로 실행해 YES 영점 확인을 거친다.
#   - 종료는 이 터미널에서 Ctrl+C. 실제 장비의 안전 정지는 티칭 펜던트를 사용한다.
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"

# system_launcher.py는 ~/.local/share/cake-web-tooling 아래의 Node.js를 먼저 찾는다.
# 준비 스크립트(npm ci)도 같은 Node.js를 쓰도록 PATH 맨 앞에 둔다.
node_bin_dir="${CAKE_NODE_BIN_DIR:-$HOME/.local/share/cake-web-tooling/node_modules/.bin}"
if [[ -x "$node_bin_dir/node" ]]; then
  export PATH="$node_bin_dir:$PATH"
fi

# --help는 아무것도 실행하지 않으므로 안내 없이 그대로 넘긴다.
if [[ " $* " == *" --help "* || " $* " == *" -h "* ]]; then
  exec bash "$script_dir/start_real.sh" --preview-no-order "$@"
fi

# --check는 장비를 움직이지 않지만, 실행 전제(의존성·인터페이스)를 그대로 검사한다.
if [[ " $* " == *" --check "* ]]; then
  exec bash "$script_dir/start_real.sh" --preview-no-order "$@"
fi

cat <<'GUIDE'
==================================================================
 실제 로봇 전체 시스템 점검 (주문·드로잉 없음)
==================================================================
 이 실행이 하는 일
  - 드라이버, 백엔드, 그림 경로/작업 관리/로봇 실행 노드, 주문 브리지, 프론트엔드를 켠다.
  - 프론트엔드는 http://localhost:8080 에서 열린다.
  - 주문 접수와 드로잉은 실행하지 않는다. YES 영점 확인도 요구하지 않는다.

 실행 전 확인
  1) 로봇 펜던트가 STANDBY(1)이고 비상정지를 바로 누를 수 있다.
  2) 로봇과 PC의 이더넷이 연결되어 있고 로봇 IP(기본 192.168.1.100)에 ping이 된다.
  3) 프론트엔드 화면만 확인한다. 이 상태에서 웹 제작 요청을 보내지 않는다.
  4) 실제 제작은 Ctrl+C로 종료한 뒤 `bash scripts/start_real.sh`로 YES 확인 후 실행한다.
==================================================================
GUIDE

# 실행에 필요한 산출물이 없으면 먼저 준비 스크립트를 안내한다.
# 필요한 파일을 임의로 만들지 않고, 준비는 setup_project.sh 한 곳에서만 수행한다.
missing=()
[[ -f /opt/ros/jazzy/setup.bash ]] || missing+=("/opt/ros/jazzy (ROS 2 Jazzy)")
[[ -f "$project_root/ws_cobot1/install/local_setup.bash" ]] || missing+=("ws_cobot1 빌드 (colcon build)")
[[ -x "$project_root/backend/.venv/bin/python" ]] || missing+=("backend/.venv")
[[ -f "$project_root/frontend/node_modules/vite/bin/vite.js" ]] || missing+=("frontend/node_modules")
if ((${#missing[@]} > 0)); then
  printf '준비가 필요합니다:\n' >&2
  printf '  - %s\n' "${missing[@]}" >&2
  echo "먼저 bash scripts/setup_project.sh를 실행하세요." >&2
  exit 1
fi

node_program="$(command -v node || true)"
if [[ -z "$node_program" ]]; then
  echo "Node.js 22.12 이상이 필요합니다. 준비 경로를 확인하세요." >&2
  exit 1
fi
node_version="$("$node_program" --version)"
if ! "$node_program" -e '
  const [major, minor] = process.versions.node.split(".").map(Number);
  process.exit(major > 22 || (major === 22 && minor >= 12) ? 0 : 1);
'; then
  echo "Node.js 22.12 이상이 필요합니다. 현재: $node_version ($node_program)" >&2
  exit 1
fi

echo "사용 Node.js: $node_version ($node_program)"
echo "프론트엔드 주소: http://localhost:8080"
echo

# 나머지 기동·정리·로그는 기존 전체 실행기 한 곳에 맡긴다.
# 미리보기 플래그는 YES 확인과 실제 주문 없이 현재 TCP만 기준점으로 기록한다.
exec bash "$script_dir/start_real.sh" --preview-no-order "$@"
