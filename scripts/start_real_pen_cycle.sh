#!/usr/bin/env bash
# 실제 로봇: 웹 그림 → [펜 집기 → 드로잉 → 펜 반납] 확인용 실행기.
#
# 동작 순서 (웹에서 제작 요청 1회마다)
#   1. 펜 상공(관절) → 그리퍼 열기 → 펜 위치 하강 → 그리퍼 닫기 → 수직 상공
#   2. 케이크 중심 상공(관절) → cake_center_pose 하강 → 상공(관절)
#      → YES 때 기록한 케이크 상면 중심을 기준으로 그림 드로잉
#   3. 반납 상공(관절) → 반납 위치 하강 → 그리퍼 열기 → 상공 후퇴
#
# 펜 좌표: ws_cobot1/src/cake_decorator/config/workspace_config.yaml
#          (robot_executor_node의 pen_* 값)
# 나머지 흐름(드라이버 → 백엔드 → YES 영점 → 업무 노드 → 브릿지 → 프론트)은
# start_real.sh와 같다. 옵션(--host, --reuse-driver 등)도 그대로 전달된다.
# 종료: 이 터미널에서 Ctrl+C. 실제 장비의 안전 정지는 티칭 펜던트를 사용한다.
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config="$script_dir/../ws_cobot1/src/cake_decorator/config/workspace_config.yaml"

# --help·--check는 장비를 움직이지 않으므로 확인 절차 없이 바로 넘긴다.
if [[ " $* " == *" --help "* || " $* " == *" -h "* || " $* " == *" --check "* ]]; then
  export CAKE_PEN_TOOL_CYCLE=1
  exec bash "$script_dir/start_real.sh" "$@"
fi

cat <<'GUIDE'
==================================================================
 실제 로봇 펜 집기 → 드로잉 → 펜 반납 테스트
==================================================================
 실행 전 확인
  1) 펜던트에 설정된 TCP가 yaml 펜 좌표를 티칭할 때의 TCP와 같다.
     (작업 좌표는 현재 TCP 기준이라, 다르면 다른 위치로 이동한다.)
  2) 펜이 거치대에 꽂혀 있고 그리퍼는 비어 있다.
  3) 펜 상공·반납 상공으로 가는 관절 이동 경로에 장애물이 없다.
  4) 첫 시험은 실제 케이크 대신 종이·박스를 둔다.
  5) 펜던트 비상정지를 바로 누를 수 있다.

 영점(YES) 단계
  - 그리퍼가 빈 상태로, 드로잉 때 '펜촉이 닿아야 할 점'에
    현재 TCP가 오도록 케이크 상면 중앙에 맞춘 뒤 YES를 입력한다.
  - 펜촉 TCP라면 그 가상의 펜촉 점을 상면 중앙에 둔다.
    (펜을 실제로 쥐었을 때 그 점에 펜촉이 오는지는 거치대 높이로 결정된다.)
  - --calibrate-pen-tcp 옵션은 TCP를 새로 등록하므로 이 테스트에 쓰지 않는다.

 웹에서는 윗면(TOP)만 그린다. 옆면 요청은 펜을 집기 전에 거부한다.
 웹 제작 요청 1회마다 펜 집기 → 드로잉 → 펜 반납을 실행한다.
 펜 집기·반납 중 실패하면 기존 _fail 처리대로 홈 이동을 시도하므로
 펜던트 비상정지를 준비해 둔다.
 드로잉 중 실패하면 그 자리에서 멈추고 펜은 그리퍼에 남는다.
==================================================================
GUIDE

echo "yaml 펜 좌표 ($config):"
grep -E '^\s*pen_(stand|gripper|drop)' "$config" | sed 's/^ */  /'
echo

if [[ " $* " == *" --calibrate-pen-tcp "* ]]; then
  echo "--calibrate-pen-tcp는 이 테스트에서 사용할 수 없습니다." >&2
  exit 1
fi

read -r -p "위 항목을 모두 확인했으면 GO 입력 (그 외 종료): " answer
if [[ "${answer^^}" != "GO" ]]; then
  echo "실행을 취소했습니다."
  exit 1
fi

# system_launcher.py가 이 값을 보고 robot_executor_node에 pen_tool_cycle:=true를 넘긴다.
export CAKE_PEN_TOOL_CYCLE=1
exec bash "$script_dir/start_real.sh" "$@"
