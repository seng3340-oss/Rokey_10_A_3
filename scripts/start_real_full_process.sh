#!/usr/bin/env bash
# 실제 로봇: 웹 주문 1회로 전체 공정을 확인하는 실행기.
#
# 기동 순서 (start_real.sh와 같음)
#   드라이버 → 백엔드 → real_robot.json 기준점 → 그림 경로/로봇 실행/작업 관리 노드
#   → 주문 브리지 → 프론트엔드(http://localhost:8080)
#
# 웹에서 제작 요청 1회마다 task_manager_node가 실행하는 순서
#   1. 플레이트 준비 (/plate_prepare): 오프너 피킹 → 케이크를 스테이지로 이동
#   2. 펜 집기 → 드로잉 → 펜 반납 (pen_tool_cycle)
#   3. 플레이트 후처리 (/plate_finish): 스테이지에서 케이크 회수 → 출고 위치 배달
#
# 켜는 기능
#   CAKE_PLATE_SEQUENCE=1 → robot_executor_node·task_manager_node에
#                           plate_sequence_enabled:=true 전달 (system_launcher.py)
#   CAKE_PEN_TOOL_CYCLE=1 → robot_executor_node에 pen_tool_cycle:=true 전달
#
# 좌표: ws_cobot1/src/cake_decorator/config/workspace_config.yaml
#       (plate_* 플레이트 좌표, pen_* 펜 좌표, cake_center_* 케이크 중심 상공·접촉 자세)
# 드로잉 기준점: XY는 config/real_robot.json의 side_cake_center_xy_mm,
#   접촉 Z·펜 자세는 yaml cake_center_pose. YES 영점 대신 쓴다(--cake-center-from-config).
# 드로잉 이동: 케이크 중심 상공(관절) → 첫 획 시작점 상공(같은 높이)
#   → 수직 하강해 펜 내림 → 그리기.
#   configure_pen_tcp_on_yes는 전체 공정에서 쓰지 않는다(TCP를 바꾸지 않음).
# YES 영점은 start_real.sh 단위 테스트용으로 그대로 남아 있다.
# 옵션(--host, --reuse-driver, --check 등)은 start_real.sh로 그대로 전달된다.
# 종료: 이 터미널에서 Ctrl+C. 실제 장비의 안전 정지는 티칭 펜던트를 사용한다.
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config="$script_dir/../ws_cobot1/src/cake_decorator/config/workspace_config.yaml"
real_config="$script_dir/../config/real_robot.json"

# system_launcher.py가 프론트엔드를 띄울 때 쓰는 Node.js를 PATH 맨 앞에 둔다.
# run_real_full_system.sh와 같은 경로 규칙을 사용한다.
node_bin_dir="${CAKE_NODE_BIN_DIR:-$HOME/.local/share/cake-web-tooling/node_modules/.bin}"
if [[ -x "$node_bin_dir/node" ]]; then
  export PATH="$node_bin_dir:$PATH"
fi

# 두 기능 플래그는 이 스크립트에서만 켠다. 다른 실행기에는 영향이 없다.
export CAKE_PLATE_SEQUENCE=1
export CAKE_PEN_TOOL_CYCLE=1

# --help·--check는 장비를 움직이지 않으므로 확인 절차 없이 바로 넘긴다.
if [[ " $* " == *" --help "* || " $* " == *" -h "* || " $* " == *" --check "* ]]; then
  exec bash "$script_dir/start_real.sh" --cake-center-from-config "$@"
fi

# --calibrate-pen-tcp는 TCP를 새로 등록해 yaml 좌표를 티칭할 때의 TCP와 달라질 수 있다.
if [[ " $* " == *" --calibrate-pen-tcp "* ]]; then
  echo "--calibrate-pen-tcp는 전체 공정 테스트에서 사용할 수 없습니다." >&2
  exit 1
fi

cat <<'GUIDE'
==================================================================
 실제 로봇 전체 공정 테스트
 (플레이트 준비 → 펜 집기 → 드로잉 → 펜 반납 → 플레이트 후처리)
==================================================================
 실행 전 확인
  1) 펜던트에 설정된 TCP가 yaml의 plate_*·pen_* 좌표를 티칭할 때의
     TCP와 같다. (작업 좌표는 현재 TCP 기준이라 다르면 다른 위치로 간다.)
  2) 오프너·케이크가 시작 위치에 있고, 펜이 거치대에 꽂혀 있으며
     그리퍼는 비어 있다.
  3) 플레이트·펜·스테이지·출고 위치 사이 이동 경로에 사람과 장애물이 없다.
  4) 첫 시험은 실제 케이크 대신 종이·박스를 둔다.
  5) 펜던트 비상정지를 바로 누를 수 있다.

 드로잉 기준점 (YES 영점·펜촉 TCP 등록 없음)
  - XY: config/real_robot.json의 side_cake_center_xy_mm
  - 펜이 닿는 Z와 펜 자세: yaml cake_center_pose (기존 케이크 중심 하강 높이)
  - 순서: 케이크 중심 상공 → 첫 획 시작점 상공 → 수직 하강 → 그리기
  - 아래에 출력되는 기준점이 스테이지 위 케이크 중심과 맞는지 확인한다.

 동작
  - 웹 제작 요청 1회마다 위 전체 공정을 한 번 실행한다.
  - 웹에서는 윗면(TOP)만 그린다. 옆면 요청은 펜을 집기 전에 거부한다.
  - 중간 단계가 실패하면 주문은 FAILED로 끝나고 이후 단계는 실행하지 않는다.
    케이크·펜·오프너가 중간 위치에 남을 수 있으므로 수동으로 정리한다.
==================================================================
GUIDE

# 실행 전에 실제로 사용할 좌표 키를 보여 준다. 값 확인은 작업자가 한다.
echo "yaml 펜 좌표 ($config):"
grep -E '^\s*pen_(stand|gripper|drop)' "$config" | sed 's/^ */  /' || true
echo "yaml 플레이트 좌표 키: $(grep -cE '^\s*plate_[a-z0-9_]+:\s*\[' "$config" || true)개"
echo "케이크 기준 좌표 ($real_config):"
sed 's/^/  /' "$real_config"
grep -E '^\s*cake_center_pose:' "$config" | sed 's/^ */  /' || true
echo

read -r -p "위 항목을 모두 확인했으면 GO 입력 (그 외 종료): " answer
if [[ "${answer^^}" != "GO" ]]; then
  echo "실행을 취소했습니다."
  exit 1
fi

# 기동·정리·로그는 기존 전체 실행기 한 곳에 맡긴다.
exec bash "$script_dir/start_real.sh" --cake-center-from-config "$@"
