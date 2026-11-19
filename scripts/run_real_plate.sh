#!/usr/bin/env bash
# 실제 로봇에서 플레이트 준비→후처리를 한 번 실행하는 단일 진입점.
#
# 동작 순서
#   1. ROS·로봇 드라이버·케이크 설치본을 source하고 ROS_DOMAIN_ID를 고정한다.
#   2. 실제 모드 드라이버(m0609_rg2_bringup)를 띄우고 로봇이 STANDBY인지 확인한다.
#   3. robot_executor_node(plate_sequence_enabled=true)와 task_manager_node를 띄운다.
#   4. /cake/order 액션에 주문을 1회 보내 준비→(그림)→후처리를 수행한다.
#   5. 종료 시 이 스크립트가 띄운 프로세스만 정리한다.
#
# 안전
#   - 실제 로봇이 즉시 움직인다. 펜던트 비상정지를 누를 수 있는 위치에서 실행한다.
#   - 중단이 필요하면 Ctrl+C보다 펜던트 비상정지를 먼저 사용한다. Ctrl+C는
#     프로세스만 정리하며, 이미 시작된 서비스 동작을 되돌리지 않는다.
#   - 드로잉 없는 기본 주문은 오프너 피킹·이동·놓기만 수행한다.
#
# 사용 예
#   bash scripts/run_real_plate.sh                  # 확인 프롬프트 후 실행
#   bash scripts/run_real_plate.sh --check          # 이동 없이 환경만 점검
#   bash scripts/run_real_plate.sh --reuse-driver   # 이미 떠 있는 드라이버 재사용
#   bash scripts/run_real_plate.sh --host 192.168.1.100 --skip-ping
#   bash scripts/run_real_plate.sh --pen-cycle      # 펜 집기·반납까지 사용
#   bash scripts/run_real_plate.sh --goal-file my_goal.yaml
#   bash scripts/run_real_plate.sh --keep           # 종료 후 프로세스 유지
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
config="$repo_root/ws_cobot1/src/cake_decorator/config/workspace_config.yaml"
cake_setup="$repo_root/ws_cobot1/install/local_setup.bash"

host="192.168.1.100"
port="12345"
reuse_driver=false
skip_ping=false
keep=false
assume_yes=false
pen_cycle=false
check_only=false
goal_file=""
order_id="plate-$(date +%Y%m%d-%H%M%S)"
startup_timeout=120

usage() {
  # 파일 선두의 주석 블록만 도움말로 출력한다. 코드가 늘어도 범위가 어긋나지 않는다.
  awk 'NR==1 {next} /^set -euo pipefail/ {exit} /^#/ {sub(/^# ?/, ""); print}' \
    "${BASH_SOURCE[0]}"
}

die() {
  echo "오류: $*" >&2
  exit 1
}

# ROS setup 스크립트는 set -u에서 미정의 변수를 참조하므로 source 동안만 해제한다.
load_workspace() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  source "$robot_setup"
  source "$cake_setup"
  set -u
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export RCUTILS_COLORIZED_OUTPUT=1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host) host="${2:-}"; shift 2 ;;
    --host=*) host="${1#*=}"; shift ;;
    --port) port="${2:-}"; shift 2 ;;
    --port=*) port="${1#*=}"; shift ;;
    --goal-file) goal_file="${2:-}"; shift 2 ;;
    --goal-file=*) goal_file="${1#*=}"; shift ;;
    --order-id) order_id="${2:-}"; shift 2 ;;
    --order-id=*) order_id="${1#*=}"; shift ;;
    --timeout) startup_timeout="${2:-}"; shift 2 ;;
    --timeout=*) startup_timeout="${1#*=}"; shift ;;
    --reuse-driver) reuse_driver=true; shift ;;
    --skip-ping) skip_ping=true; shift ;;
    --keep|--no-cleanup) keep=true; shift ;;
    --pen-cycle) pen_cycle=true; shift ;;
    --yes|-y) assume_yes=true; shift ;;
    --check) check_only=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) die "알 수 없는 인자: $1 (--help 참고)" ;;
  esac
done

[[ "$port" =~ ^[0-9]+$ ]] || die "포트가 숫자가 아닙니다: $port"
[[ "$startup_timeout" =~ ^[0-9]+$ ]] || die "timeout이 숫자가 아닙니다: $startup_timeout"
[[ -f "$config" ]] || die "설정 파일이 없습니다: $config"

# 로봇 설치본은 직접 지정, 저장소 내부, 기존 개인 작업 순서로 찾는다.
# 필수 패키지 디렉터리가 없는 setup 파일은 미완성 빌드이므로 건너뛴다.
robot_setup=""
for candidate in \
  "${CAKE_ROBOT_SETUP:-}" \
  "$repo_root/ws_dsr/install/local_setup.bash" \
  "$HOME/ws_cobot_pjt/ws_dsr/install/local_setup.bash"
do
  [[ -n "$candidate" ]] || continue
  [[ -f "$candidate" ]] || continue
  [[ -d "$(dirname "$candidate")/dsr_controller2" ]] || continue
  robot_setup="$candidate"
  break
done

if [[ -z "$robot_setup" ]]; then
  die "로봇 드라이버 설치본을 찾지 못했습니다. CAKE_ROBOT_SETUP=/경로/install/local_setup.bash 를 지정하세요."
fi

if $check_only; then
  echo "저장소     : $repo_root"
  echo "로봇 설치본: $robot_setup"
  echo "케이크 설치: $cake_setup"
  echo "설정 파일  : $config"
  [[ -f "$cake_setup" ]] || die "ws_cobot1 빌드가 없습니다. ws_cobot1에서 colcon build --symlink-install 을 실행하세요."
  # 실제 이동 없이 인터페이스와 액션 타입만 확인한다.
  load_workspace
  ros2 interface show cake_decorator_interfaces/action/CakeOrder >/dev/null \
    || die "CakeOrder 인터페이스를 찾지 못했습니다."
  command -v ros2 >/dev/null || die "ros2 명령이 없습니다."
  echo "점검 통과: ROS 환경과 CakeOrder 인터페이스가 준비되었습니다."
  exit 0
fi

[[ -f "$cake_setup" ]] || die "ws_cobot1 빌드가 없습니다: $cake_setup"
if [[ -n "$goal_file" && ! -f "$goal_file" ]]; then
  die "goal 파일이 없습니다: $goal_file"
fi

cat <<'GUIDE'
==================================================================
 실제 로봇 플레이트 준비 → 후처리 1회 실행
==================================================================
 실행 전 확인
  1) 로봇 제어기와 이 PC가 같은 네트워크에 있고 접속이 가능하다.
  2) 펜던트에 설정된 TCP가 yaml 좌표를 티칭할 때의 TCP와 같다.
  3) 오프너 거치대·케이크·작업대 주변에 사람과 장애물이 없다.
  4) 펜던트 비상정지를 바로 누를 수 있다.
  5) 첫 시험은 실제 케이크 대신 종이·박스를 둔다.

 이 스크립트가 하는 일
  - 드라이버를 실제 모드로 띄우고 로봇이 STANDBY인지 확인한다.
  - 플레이트 시퀀스를 켠 실행 노드와 작업 관리 노드를 띄운다.
  - 주문 1회를 보내 오프너 피킹 → 케이크 이동 → 오프너 놓기를 수행한다.
  - 그림이 없는 기본 주문이므로 드로잉은 건너뛴다.
==================================================================
GUIDE

if ! $assume_yes; then
  read -r -p "위 항목을 모두 확인했으면 GO 입력 (그 외 종료): " answer
  if [[ "${answer^^}" != "GO" ]]; then
    echo "실행을 취소했습니다."
    exit 1
  fi
fi

# shellcheck disable=SC1091
load_workspace

timestamp="$(date +%Y%m%d-%H%M%S)"
log_dir="$repo_root/log/real_plate/$timestamp-$$"
mkdir -p "$log_dir"

pids=()
labels=()

# setsid로 프로세스 그룹을 분리해 두면 ros2 run/launch가 만든 자식까지
# 그룹 단위로 정리할 수 있다. PID가 곧 프로세스 그룹 ID가 된다.
start_bg() {
  local name="$1"
  shift
  setsid "$@" >"$log_dir/$name.log" 2>&1 </dev/null &
  pids+=("$!")
  labels+=("$name")
  echo "[$name] pid=$! log=$log_dir/$name.log"
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if $keep; then
    echo "프로세스를 유지합니다(--keep). 정리하려면 아래 PID 그룹에 SIGINT를 보내세요."
    local i
    for ((i = 0; i < ${#pids[@]}; i++)); do
      echo "  ${labels[$i]}: kill -INT -- -${pids[$i]}"
    done
    exit "$status"
  fi
  local i
  for ((i = ${#pids[@]} - 1; i >= 0; i--)); do
    kill -INT -- "-${pids[$i]}" 2>/dev/null || true
  done
  sleep 2
  for ((i = ${#pids[@]} - 1; i >= 0; i--)); do
    kill -TERM -- "-${pids[$i]}" 2>/dev/null || true
  done
  sleep 1
  for ((i = ${#pids[@]} - 1; i >= 0; i--)); do
    kill -KILL -- "-${pids[$i]}" 2>/dev/null || true
  done
  echo "로그: $log_dir"
  exit "$status"
}
trap cleanup EXIT INT TERM

wait_for_service() {
  local service="$1"
  local limit="$2"
  local i=0
  while ((i < limit)); do
    if timeout 5 ros2 service list 2>/dev/null | grep -qx "$service"; then
      return 0
    fi
    sleep 1
    ((i++))
  done
  return 1
}

wait_for_action() {
  local action="$1"
  local limit="$2"
  local i=0
  while ((i < limit)); do
    if timeout 5 ros2 action list -t 2>/dev/null | grep -q "^$action "; then
      return 0
    fi
    sleep 1
    ((i++))
  done
  return 1
}

# 같은 이름의 노드가 남아 있으면 서비스·액션이 중복되어 엉뚱한 노드가 응답한다.
for node in /robot_executor_node /task_manager_node /drawing_path_node; do
  if timeout 5 ros2 node list 2>/dev/null | grep -qx "$node"; then
    die "$node 가 이미 실행 중입니다. 기존 실행을 종료한 뒤 다시 실행하세요."
  fi
done

if $reuse_driver; then
  wait_for_service /dsr01/dsr_controller2/motion/move_joint 5 \
    || die "--reuse-driver를 지정했지만 드라이버 서비스가 없습니다."
  echo "주의: 기존 드라이버를 재사용합니다. 실제 모드인지 확인하세요."
else
  # 드라이버가 이미 떠 있으면 새로 띄우지 않는다. 중복 드라이버는 같은 이름의
  # 노드·서비스를 만들어 명령이 엉뚱한 인스턴스로 갈 수 있다.
  if wait_for_service /dsr01/dsr_controller2/motion/move_joint 2; then
    die "드라이버가 이미 실행 중입니다. 기존 드라이버를 쓸 때는 --reuse-driver를 지정하세요."
  fi
  if ! $skip_ping && command -v ping >/dev/null; then
    ping -c 1 -W 1 "$host" >/dev/null 2>&1 \
      || die "로봇 제어기에 ping이 닿지 않습니다: $host (케이블·전원·IP 확인, ICMP 차단 시 --skip-ping)"
  fi
  start_bg bringup ros2 launch m0609_rg2_bringup bringup.launch.py \
    mode:=real "host:=$host" "port:=$port"
  wait_for_service /dsr01/dsr_controller2/motion/move_joint "$startup_timeout" \
    || die "드라이버 서비스가 뜨지 않았습니다: $log_dir/bringup.log"
fi

wait_for_service /dsr01/dsr_controller2/system/get_robot_state "$startup_timeout" \
  || die "드라이버 상태 서비스를 찾지 못했습니다."

# get_robot_state가 success=True이고 STANDBY(1)일 때만 자동 명령을 허용한다.
# 연결이 끊기면 success=False가 오므로 여기서 멈추는 것이 안전하다.
# 응답이 없는 드라이버에서 ros2 service call이 무한 대기하지 않도록 회차마다 제한한다.
state_ok=false
state_out=""
for ((i = 0; i < 12; i++)); do
  state_out="$(timeout 5 ros2 service call \
    /dsr01/dsr_controller2/system/get_robot_state \
    dsr_msgs2/srv/GetRobotState '{}' 2>&1 || true)"
  if grep -q 'success=True' <<<"$state_out"; then
    state_ok=true
    break
  fi
  sleep 1
done
if ! $state_ok; then
  echo "$state_out" | tail -5 >&2
  die "로봇 상태를 읽지 못했습니다. 드라이버 로그와 제어기 연결을 확인하세요."
fi
if ! grep -qE 'robot_state=1([^0-9]|$)' <<<"$state_out"; then
  echo "$state_out" | tail -5 >&2
  die "로봇이 STANDBY(1)가 아닙니다. 펜던트에서 상태를 확인하세요."
fi

start_bg drawing_path_node ros2 run cake_decorator drawing_path_node \
  --ros-args --params-file "$config"
start_bg robot_executor_node ros2 run cake_decorator robot_executor_node \
  --ros-args \
  --params-file "$config" \
  -p plate_sequence_enabled:=true \
  -p "pen_tool_cycle:=$pen_cycle"
wait_for_service /plate_prepare "$startup_timeout" \
  || die "plate_prepare 서비스가 뜨지 않았습니다: $log_dir/robot_executor_node.log"
wait_for_service /plate_finish "$startup_timeout" \
  || die "plate_finish 서비스가 뜨지 않았습니다: $log_dir/robot_executor_node.log"

start_bg task_manager_node ros2 run cake_decorator task_manager_node \
  --ros-args \
  -p plate_sequence_enabled:=true \
  -p action_timeout:=1800.0
wait_for_action /cake/order "$startup_timeout" \
  || die "/cake/order 액션이 뜨지 않았습니다: $log_dir/task_manager_node.log"

if [[ -n "$goal_file" ]]; then
  goal="$(cat "$goal_file")"
else
  goal="{order_id: '$order_id', tool_type: 'marker'}"
fi

echo "주문 전송: $goal"
set +e
goal_out="$(ros2 action send_goal \
  /cake/order cake_decorator_interfaces/action/CakeOrder \
  "$goal" --feedback 2>&1)"
goal_rc=$?
set -e
printf '%s\n' "$goal_out" | tail -25

if [[ $goal_rc -ne 0 ]] || ! grep -q 'success: true' <<<"$goal_out"; then
  die "주문이 실패했습니다. 상세 로그: $log_dir"
fi

echo "완료: $order_id"
