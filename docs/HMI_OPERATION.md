# 사용자 UI–TM–관리자 HMI 운용 연결

현재 작업 브랜치의 운용 경로입니다. 가상 DRCF·RViz·외부 검증 폴더 없이 프로젝트 소스만 사용합니다.

```text
사용자 UI POST /api/drawing → FastAPI 주문 저장 → ros_bridge_node
→ CakeOrder → task_manager_node → GenerateTrajectory / DrawOnCake
→ CakeOrder Feedback + /task_status → 브릿지 → FastAPI → HMI GET 조회
GenerateTrajectory 응답 → TM /hmi/drawing_plan → 브릿지 → /api/orders/{id}/plan
```

## 화면과 정보

- `/`: 고객 그림 작성·확인·접수. 한 요청은 선택한 상단 또는 측면 한 면입니다.
- `/hmi`: 주문·공정·현재 명령·마지막 오류·TM 연결 관측·원본/계획 비교.
- `/workspace` 또는 `/paired`: 고객 UI와 관리자 HMI 나란히 보기.
- 작성 중 그림은 같은 브라우저 출처의 BroadcastChannel 미리보기입니다. 주문이나 로봇 명령이 아닙니다.
- TM 계획은 실제 GenerateTrajectory 응답의 펜 내림 점만 투영합니다. 펜올림 사이를 선으로 잇지 않습니다.
- 실제 TCP·잉크 측정 경로는 아직 연결하지 않았습니다. 진행률/원본을 실측 경로로 재생하는 데모는 운용 HMI에 없습니다.
- `execution_mode`는 브릿지에서 선택한 `real`/`simulation`/`unknown` 설정입니다. 장비 연결/안전 확인을 대신하지 않습니다.
- 브릿지는2초마다 TM 발견·마지막 작업 피드백 나이를 전송합니다. 백엔드는8초 이상 오래된 관측을 최신으로 표시하지 않습니다.
- 선택 주문의 heartbeat는 활성 주문 ID가 일치할 때만 최신으로 표시합니다. 종료 주문의 원본·계획·상태는 DB에 남습니다.
- HMI 로그는 브라우저가3초 조회에서 관측한 변화이며 영구 감사 로그가 아닙니다.
- 물리 E-stop 신호는 미연결로 확인 불가 상태를 유지합니다. HMI에는 로봇 이동·정지 API가 없습니다.

## 웹 서버 실행

아래 명령은 프로젝트 루트에서 실행합니다. 실제 운영 DB와 테스트 DB를 구분합니다.

```bash
PROJECT_ROOT=$(pwd)
cd frontend
npm ci
npm run build
cd ../backend
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
export CAKE_FRONTEND_DIST="$PROJECT_ROOT/frontend/dist"
export CAKE_DATABASE_PATH="$PROJECT_ROOT/backend/data/orders.sqlite3"
# 브릿지 터미널과 같은 충분히 긴 ASCII 토큰을 별도로 설정합니다.
export CAKE_BRIDGE_TOKEN='REPLACE_WITH_YOUR_LOCAL_SECRET'
.venv/bin/python -m uvicorn app.main:app --host 127.0.0.1 --port 5000
```

노트북 화면은 `http://127.0.0.1:5000/workspace`입니다. 태블릿 접속을 허용할 때는 서버 바인딩과 내부 네트워크 주소를 운영자가 설정합니다. 내부 토큰은 프론트엔드 코드에 넣지 않습니다.
`.env.example`은 예시이며 자동으로 로드되지 않습니다. 개발 모드에서는 기존 Vite의 API 프록시를 사용할 수 있습니다.

## ROS 연결

운영자가 설치·교정한 실제 드라이버, 작업 노드(DP/RE/TM)를 사용합니다. 이 안내는 드라이버를 자동 실행하거나 IP/TCP/속도/힘 설정을 변경하지 않습니다.
실제 업무 노드를 띄우기 전에 설치본과 실제 운영 ROS 도메인을 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
source <운영_ROS_작업공간>/install/setup.bash
# 업무 노드가 기존 launch에서 실행 중이면 중복 실행하지 않습니다.
ros2 run cake_decorator drawing_path_node
ros2 run cake_decorator robot_executor_node
ros2 run cake_decorator task_manager_node
```

각 노드는 별도 터미널에서 실행합니다. RE는 주문을 받으면 실제 장비 명령을 보낼 수 있는 기존 실행부입니다.
웹 서버와 동일 토큰을 설정한 브릿지 터미널에서:

```bash
export CAKE_BRIDGE_TOKEN='REPLACE_WITH_YOUR_LOCAL_SECRET'
ros2 run cake_decorator ros_bridge_node \
  --backend-url http://127.0.0.1:5000 --execution-mode real
```

브릿지는 교정된 활성 펜촉 TCP가 수평 케이크 윗면 중심에 닿았음을 운영자가 확인하고 `YES`를 입력한 뒤 현재 BASE TCP를 읽습니다. 그 이후 고객 주문을 TM으로 전달하면 실제 작업이 시작될 수 있습니다. 케이크를 옮겼다면 기준점을 다시 기록합니다.
이 문서는 가상 환경의 고정 좌표·TCP 길이·펜 기울기를 실물 설정으로 사용하지 않습니다. 현재 로컬 TM은 드로잉 전용이며 픽업/반출 공정을 새로 추가하지 않았습니다.

## API

- `GET /api/orders`, `GET /api/orders/{id}`: 원본과 작업 상태.
- `GET /api/orders/{id}/monitor`: 주문별 모드 설정·명령·마지막 오류·소요 시간.
- `GET /api/monitor`: TM 연결 관측. `robot_connected=null`, `safety_status=unknown` 유지.
- `GET /api/orders/{id}/plan`: TM 계획 또는 미수신404. `measurement_available=false`.
- `POST /internal/bridge/heartbeat`: 인증된 표시용 관측. 주문 인수/실행 없음.
- `POST /internal/bridge/orders/{id}/plan`: 토큰·주문·면·해상도를 대조한 계획 저장.
- 기존 claim/events/resolve 계약은 유지합니다. 계획 전송 실패는 Goal 재전송이나 작업 실패를 일으키지 않습니다.

`/api/health`의 `ros_connected`는 최신 브릿지가 TM 액션 서버를 발견했다는 뜻입니다. 실제 로봇 연결 신호가 아닙니다.
기존 DB에는 표시용 테이블만 추가하며 원본 주문·스테이징 파일·인수 잠금을 삭제하지 않습니다.

## 장비 없는 검증

백엔드와 프론트엔드:

```bash
cd backend
.venv/bin/python -m pip install -r requirements-dev.txt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
cd ../frontend
npm test
npm run build
```

ROS 통합 검사는 **실제 드라이버가 없는 별도 도메인231**과 임시 DB에서만 명시적으로 실행합니다. 원본 TM·DP·브릿지는 실제 코드이며 DrawOnCake 실행부만 명령을 보내지 않는 테스트 대역입니다.
테스트 실행 Python에 백엔드 검사 의존성과 ROS 작업공간을 준비한 뒤 프로젝트 루트에서:

```bash
export ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 CAKE_RUN_ROS_TESTS=1
python3 -m pytest -q ws_cobot1/src/cake_decorator/test/test_hmi_pipeline.py
```

테스트 서버는 검사와 같은 Python으로 실행하고, 최대30초 실제 경과 시간으로 준비를 기다립니다. 프로세스 조기 종료는 원인 로그와 함께 즉시 실패합니다.
기존 장비 없는 브릿지 검사도 해당 테스트용 Python을 지정해 별도로 실행할 수 있습니다.
