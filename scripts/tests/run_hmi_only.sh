#!/usr/bin/env bash
# HMI 화면만 임시로 띄우는 확인용 실행기.
#
# 목적
#   로봇·ROS 노드 없이 백엔드(FastAPI)와 프론트엔드(Vite)만 띄워 /hmi 화면을 확인한다.
#
# 실제 운용과 분리하는 방법
#   - 주문 DB: 실행마다 새 임시 DB(.runtime/hmi-test-*/orders.sqlite3)를 사용하고 종료 시 삭제한다.
#     실제 주문 DB(backend/data/orders.sqlite3)는 읽거나 쓰지 않는다.
#   - 포트: 기본 백엔드 5100, 프론트 8180을 사용한다. 실제 실행기(5000·8080)와 동시에 띄워도 충돌하지 않는다.
#   - 브릿지·로봇이 없으므로 HMI에는 "브릿지 미연결"이 표시되고, 정지 버튼은 로봇에 전달되지 않는다.
#
# 사용법
#   bash scripts/tests/run_hmi_only.sh            # 빈 주문 목록으로 실행
#   bash scripts/tests/run_hmi_only.sh --sample   # 예시 주문 1건(received 상태)을 넣고 실행
#   HMI_BACKEND_PORT=5200 HMI_FRONTEND_PORT=8280 bash scripts/tests/run_hmi_only.sh
#   종료: Ctrl+C (두 서버와 임시 DB를 함께 정리)
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
backend_port="${HMI_BACKEND_PORT:-5100}"
frontend_port="${HMI_FRONTEND_PORT:-8180}"
sample=false
for argument in "$@"; do
  case "$argument" in
    --sample) sample=true ;;
    -h|--help)
      sed -n '2,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *) echo "알 수 없는 옵션: $argument (--sample, --help)" >&2; exit 2 ;;
  esac
done

# ── 실행 도구 확인 ──
# 백엔드는 프로젝트 가상환경, 프론트는 설치된 vite를 실행기와 같은 방식으로 직접 실행한다.
backend_python="$project_root/backend/.venv/bin/python"
vite="$project_root/frontend/node_modules/vite/bin/vite.js"
node_bin="$HOME/.local/share/cake-web-tooling/node_modules/.bin/node"
node="$( [[ -x "$node_bin" ]] && echo "$node_bin" || command -v node || true )"
if [[ ! -x "$backend_python" || ! -f "$vite" || -z "$node" ]]; then
  echo "백엔드 가상환경(backend/.venv)과 프론트 의존성(frontend/node_modules)이 필요합니다." >&2
  echo "설치: bash scripts/setup_project.sh" >&2
  exit 1
fi

# 이미 사용 중인 포트면 다른 서버에 붙지 않도록 시작 전에 중단한다.
for port in "$backend_port" "$frontend_port"; do
  if (exec 3<>"/dev/tcp/127.0.0.1/$port") 2>/dev/null; then
    echo "포트 $port 가 이미 사용 중입니다. HMI_BACKEND_PORT/HMI_FRONTEND_PORT로 바꾸세요." >&2
    exit 1
  fi
done

# ── 임시 실행 폴더·DB ──
mkdir -p "$project_root/.runtime"
work_dir="$(mktemp -d "$project_root/.runtime/hmi-test-XXXXXX")"
backend_pid=""
frontend_pid=""

# 종료 시(Ctrl+C·오류 포함) 이 스크립트가 띄운 두 서버만 정리하고 임시 DB를 지운다.
cleanup() {
  trap - EXIT INT TERM
  for pid in "$frontend_pid" "$backend_pid"; do
    [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  rm -rf "$work_dir"
  echo "HMI 테스트 서버를 종료하고 임시 DB를 삭제했습니다."
}
trap cleanup EXIT INT TERM

# 서버 응답을 최대 20초 기다린다. 프로세스가 먼저 죽으면 바로 실패한다.
wait_http() {
  local url="$1" pid="$2" name="$3"
  for _ in $(seq 1 100); do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "$name 이(가) 시작 중 종료됐습니다. 로그: $work_dir/$name.log" >&2
      exit 1
    fi
    curl -fsS -o /dev/null "$url" 2>/dev/null && return 0
    sleep 0.2
  done
  echo "$name 응답 대기 시간 초과. 로그: $work_dir/$name.log" >&2
  exit 1
}

# ── 백엔드 ──
# 브릿지 토큰은 내부 API 활성화용 임의값이다. 이 실행에서는 브릿지가 접속하지 않는다.
# ROS 환경의 PYTHONPATH가 섞이지 않게 비우고 실행한다.
(
  cd "$project_root/backend"
  env -u PYTHONPATH \
    CAKE_DATABASE_PATH="$work_dir/orders.sqlite3" \
    CAKE_BRIDGE_TOKEN="hmi-test-$RANDOM$RANDOM" \
    "$backend_python" -m uvicorn app.main:app \
      --host 127.0.0.1 --port "$backend_port" --log-level warning
) >"$work_dir/backend.log" 2>&1 &
backend_pid=$!
wait_http "http://127.0.0.1:$backend_port/api/health" "$backend_pid" backend

# ── 예시 주문(선택) ──
# HMI에 주문 카드가 보이도록 임시 DB에만 넣는다. 브릿지가 없어 received 상태로 남는다.
if $sample; then
  curl -fsS -o /dev/null -X POST "http://127.0.0.1:$backend_port/api/drawing" \
    -H "Content-Type: application/json" \
    -d '{
      "canvas_width": 1024,
      "canvas_height": 768,
      "drawing_surface": "top",
      "strokes": [
        {"stroke_id": 0, "points": [
          {"x": 412, "y": 300}, {"x": 452, "y": 262}, {"x": 512, "y": 300},
          {"x": 572, "y": 262}, {"x": 612, "y": 300}, {"x": 512, "y": 430},
          {"x": 412, "y": 300}
        ]}
      ]
    }' && echo "예시 주문 1건을 임시 DB에 추가했습니다(로봇에 전달되지 않음)."
fi

# ── 프론트엔드 ──
# vite 개발 서버의 /api 프록시를 이 스크립트의 백엔드 포트로 돌린다(vite.config.ts의 API_PROXY_TARGET).
(
  cd "$project_root/frontend"
  API_PROXY_TARGET="http://127.0.0.1:$backend_port" \
    "$node" "$vite" --host 127.0.0.1 --port "$frontend_port" --strictPort
) >"$work_dir/frontend.log" 2>&1 &
frontend_pid=$!
wait_http "http://127.0.0.1:$frontend_port/" "$frontend_pid" frontend

echo "HMI 테스트 화면: http://localhost:$frontend_port/hmi"
echo "로봇·브릿지 미연결 상태입니다. 종료: Ctrl+C"
if [[ -n "${DISPLAY:-}" ]] && command -v xdg-open >/dev/null; then
  xdg-open "http://localhost:$frontend_port/hmi" >/dev/null 2>&1 || true
fi

# 둘 중 하나라도 종료되면 나머지를 정리하고 끝낸다.
wait -n "$backend_pid" "$frontend_pid"
