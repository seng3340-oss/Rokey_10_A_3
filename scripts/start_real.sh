#!/usr/bin/env bash
# 실제 로봇 전체 공정: 플레이트 준비 → 펜 집기 → TOP/SIDE → 반납 → 후처리.
# TOP 티칭 자세와 SIDE 실측 기준점은 파일에서 읽으며 기동 때 중앙으로
# 이동하거나 YES를 묻지 않는다. 실제 공정은 웹 주문을 받은 뒤 실행한다.
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 팀 전체 공정 실행기와 같은 Node.js 경로·기능 플래그를 사용한다.
node_bin_dir="${CAKE_NODE_BIN_DIR:-$HOME/.local/share/cake-web-tooling/node_modules/.bin}"
if [[ -x "$node_bin_dir/node" ]]; then
  export PATH="$node_bin_dir:$PATH"
fi
export CAKE_PLATE_SEQUENCE=1
export CAKE_PEN_TOOL_CYCLE=1
source "$script_dir/_system_environment.sh" real \
  --cake-center-from-config --integrated-side-cycle "$@"
