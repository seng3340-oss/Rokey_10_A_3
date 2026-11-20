# 사용자 UI–TM–관리자 HMI 운용 연결

## 실제 전체 공정 실행

`bash scripts/start_real.sh`만 실행하면 최신 예외 재시도·HMI 재개 기능과
케이크 무게 검사를 사용합니다. 아래의 복구·재개 설명은 이 기본 실행에 해당합니다.

```bash
bash scripts/start_real.sh --check  # 장비 기동·이동 없는 준비 검사
bash scripts/start_real.sh          # 최신 전체 공정·예외 재시도·HMI 재개
```

재시도 도입 전 `31370d0` 버전은 `bash scripts/start_real.sh --stop-on-exception`으로
선택할 수 있습니다. 이 경우 `log/stop-on-exception-31370d0/`의 독립 소스·ROS
설치본·주문 DB를 사용하고, 현재 실측 설정 파일만 실행 직전에 복사합니다.
이전의 `--recovery-dev` 옵션도 최신 기본 실행과 같은 동작으로 유지합니다.
두 실행기는 같은 로봇 실행 잠금을 사용하므로 동시에 실행할 수 없습니다.
브라우저는 자동 실행하지 않으며 HMI·태블릿 주소를 구분해 출력합니다.

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
- HMI 로그는 서버에 저장하며 스크롤로 이전 기록을 추가 조회합니다.
- 물리 E-stop 신호는 미연결로 확인 불가 상태를 유지합니다. HMI의 소프트웨어 정지는 별도 상태로 유지하며 복구 버튼으로 해제하지 않습니다.

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
이 문서는 가상 환경의 고정 좌표·TCP 길이·펜 기울기를 실물 설정으로 사용하지 않습니다. 최신 TM은 설정에 따라 기존 플레이트 준비·회수 공정을 실행하며, HMI가 운반 명령을 새로 만들지는 않습니다.

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

## 최신 main 통합 (2026-11-19)

- 원뿔대는 윗지름 113.5mm·밑지름 120mm·높이 79mm를 별도 전달합니다.
- TM은 기존 플레이트 준비·TOP/SIDE 순서·회수·취소 후 정리 동작을 유지합니다.
- 플레이트 공정이 켜지면 표시 구간은 준비 0~10%, 드로잉 10~90%, 회수 90~100%입니다.
  시간이나 면적의 측정 비율이 아니라 공정별 표시 구간입니다.
- 계획 표시는 실제 DP 요청의 기준점과 케이크 회전을 사용하며 SIDE 교정 기준도 반영합니다.
  원호 경유점·스플라인 표본을 잇는 계획 제어점 표시이며 실측 TCP나 보간 곡선이 아닙니다.
  DP의 접촉·지름 보정 때문에 원본 그림과 미소한 차이가 날 수 있습니다.
- HMI는 읽기 전용이며 이동·정지 버튼은 활성화하지 않습니다.
- 실제 운용 DB나 로봇에 연결하지 않고 별도 테스트 도메인에서 검증합니다.

## 테스트 재현

[장비 없는 백엔드·화면·ROS 통합 검사 절차](TESTING_HMI.md)를 참고하세요.

## 세 가지 예외 재시도와 제작 실패 후 재설정

원격 main의 `fc7413f`(펜·따개 재파지)를 케이크 재집기와 함께 사용한다.
실제 폭을 읽어 도구가 없음을 판정한 경우에만 펜·따개를 한 번 더 집는다.
그리퍼를 연 뒤 `tool_regrasp_wait_sec`(기본 5초)를 기다려 작업자가 도구를
거치대에 놓을 시간을 준다. TOP/SIDE 펜과 준비/회수의 따개 파지에 모두 적용한다.
폭 측정 실패나 파지 전 열림 부족은 도구 없음으로 추정하지 않으며, 정지·취소·
그리퍼 출력 이상 중에는 재파지하지 않는다.

두 번째도 도구가 없으면 빈 그리퍼를 열고 `plate_safe_position_1`에서 대기한다.
도구 없음 판정이므로 빈손으로 거치대 반납을 추가하지 않는다. 이 최종 실패도
아래의 주문 접수 잠금·태블릿 경고·HMI 관리자 재설정 흐름에 동일하게 연결된다.

케이크 하중 검사가 활성화된 경우, 입고함 집기와 스테이션 회수에서 케이크 없음
판정을 받으면 따개를 유지한 채 같은 출발지의 기존 집기 경로를 한 번 더 실행한다.
재검사는 최초 따개 기준 무게를 유지한다. 성공하면 안착·출고부터 계속하며, 두 번째도
실패하면 따개를 절반 속도로 반납하고 `plate_safe_position_1`로 이동한 뒤 대기한다.
이동·그리퍼·정지 차단 중에는 후속 명령을 강행하지 않는다. 하중 통신 실패는 케이크
없음으로 추정해 재집기하지 않고 측정 실패로 처리한다.

현재 `plate_weight_check_enabled`는 `true`다. 입고함·스테이션 집기 후 모두 검사하며,
따개 단독 대비 증가량이 0.075kg을 초과해야 케이크 보유로 통과한다. 실행 중인
노드에는 설정 파일 변경이 즉시 적용되지 않으므로 공정을 종료한 뒤 실행기를
다시 시작한다. virtual 실행은 기존과 같이 하중 검사를 끈다. 펜·따개 폭 검사는
현재 설정의 `gripper_safety_check_enabled`를 따른다.

예외 처리와 안전 복귀를 마치면 서버는 주문을 `paused`(복구 대기)로 보관하며 새
주문 접수·일반 인수를 차단한다. 중단 단계·그리기 면 순번·원래 기준 자세를 저장하므로
이미 완료한 케이크 준비나 윗면을 재실행하지 않는다. 태블릿 새로고침·다른 태블릿에서도
경고 화면을 유지한다. 임의 획 중간의 이동 실패나 안착 후 센서 장애처럼 재개 지점을
검증하지 못한 오류는 자동 재개를 제공하지 않는다.

관리자는 현장에서 빈 그리퍼·기본 TCP·`plate_safe_position_1`로 복귀시킨 뒤 HMI에
표시된 중단 위치를 확인하고 다음 중 하나를 선택한다.

- **재개하기**: 같은 주문 ID를 중단 단계부터 1회 인수하도록 예약한다. 준비 단계라면
  준비부터, 펜 파지라면 해당 면부터, 스테이션 회수라면 회수·출고부터 이어간다.
  재개 전 실행 노드는 관절별 안전 자세 오차 1도 이내·기본 TCP·열린 그리퍼를
  읽기만 해 확인한다. 확인 실패 시 이동 없이 다시 복구 대기로 돌아간다.
- **초기 자세 세팅**: 관리자가 초기 자세와 도구·케이크 정리를 마쳤음을 확인하고
  현재 주문을 최종 `failed`로 종료한 뒤 다음 주문을 받는다. 로봇을 자동 이동시키지 않는다.

재개 중에도 다른 주문은 받지 않으며, 태블릿은 새 주문 POST 없이 같은 주문의 진행
화면을 연결한다. 재개까지 성공하면 기존 완료·픽업 화면으로 이어진다. 재개 예약 직후
서버가 인수 전에 종료됐다면 자동 재전송하지 않고 새 관리자 확인을 받는다. 실제 실행
결과가 미확정인 `claimed`/`running`/`uncertain`은 기존 수동 확인 절차를 먼저 따른다.
HMI 소프트웨어 정지 잠금은 이 두 버튼으로 해제하지 않는다.

HMI 로그는 서버의 제작 이벤트와 관리자 재설정 기록을 저장한다. 최신 30건부터
표시하며 제한 높이의 로그 영역을 아래로 스크롤하면 이전 30건씩 읽는다. 새 기록은
ID 커서로 이어 읽고, 진행률만 바뀐 반복 피드백은 로그로 추가하지 않는다. 이 기능
도입 전에 서버에 저장하지 않았던 중간 이벤트를 과거 로그로 복원하지는 않는다.

- `GET /api/workcell/recovery`: 복구 대기·중단 위치·재개/초기 확인 가능 여부.
- `POST /api/workcell/resume`: 현재 `revision`과 현장 확인으로 같은 주문의 재개를 예약.
- `POST /api/workcell/initialize`: 현장 초기화 확인 후 주문 최종 실패·다음 접수 허용.
- `GET /api/workcell/logs`: `before_id`/`after_id` 중 하나와 `limit`(최대 100)으로 로그 조회.

실행 중인 서버·ROS 노드는 이전 코드를 보관하므로 새 기능 적용은 전체 실행기를
정상 종료한 뒤 다시 시작한다. 구현·검사는 장비 없는 별도 도메인과 임시 DB로 수행한다.


재개 계약을 위해 `CakeOrder` Goal·Result에 `resume_checkpoint`가 추가됐다.
전체 실행기를 종료한 뒤 `cake_decorator_interfaces`와 `cake_decorator`를 함께 빌드하고
모든 관련 ROS 노드를 재시작해야 한다. 구버전 인터페이스는 실행기 준비 검사에서 거부한다.
