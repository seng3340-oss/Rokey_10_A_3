# 플라스틱 케이크 1차 테스트 연결

웹 원본 좌표 → FastAPI·SQLite → ROS 브릿지 → TM의 `CakeOrder` Action 구조입니다.
실제 TM·경로 계산·로봇 실행 노드는 아직 틀만 있습니다. 브릿지 구현만으로 실제 필기가 시작되지는 않습니다.
통신 검증은 테스트 전용 가짜 TM으로 수행하며, 그 결과를 실제 그리기 성공으로 표시하지 않습니다.

## 치수와 기준점

공통 설정은 `config/test_cake.json`입니다. 프론트와 백엔드가 함께 읽습니다.

| 항목 | 실측값 |
|---|---|
| 케이크 윗면 지름 | 113.5mm |
| 케이크 아랫면 지름 | 120mm |
| 케이크 높이 | 79mm |
| 케이크판 | 179 × 179 × 2mm |

옆면은 수직 원통이 아닌 원뿔대입니다. 윗면에서 아래로 d만큼 내려갔을 때
`r(d) = 56.75 + (60 - 56.75) × d / 79` mm입니다.
DP는 경로 계산 시 이 반지름 변화와 표면 법선을 고려해야 합니다. 현재 DP 구현은 없습니다.
웹의 옆면 편집은 평균 둘레와 경사 높이의 비율을 사용한 띠이며, 정밀한 원뿔대 전개도는 아닙니다.

실행 시작 전에 활성 TCP가 실제 펜촉으로 교정되어 있어야 합니다.
사용자가 펜촉을 수평 케이크의 윗면 중심에 가볍게 닿게 한 뒤 브릿지 터미널에서 `YES`를 입력합니다.
브릿지는 현재 TCP를 BASE 기준으로 **조회만** 하고 그 위치를 `CakeParams.center`로 기록합니다.
이미 상면이므로 높이·판 두께를 더하지 않습니다. 기록한 TCP 방향도 `tool_orientation`으로 전달합니다.

- `reference_frame="base"`, 중심 위치와 치수는 m, `tool_orientation`은 쿼터니언입니다.
- `radius`는 윗면, `bottom_radius`는 아랫면 반지름입니다.
- 영점은 한 번 실행한 브릿지 세션 동안 고정됩니다. 케이크가 움직였다면 재실행해 다시 기록합니다.
- TCP를 조회했다고 실제 접촉이 감지된 것은 아닙니다. 접촉·펜 교정은 사용자가 확인합니다.
- 테이블 가운데 네 금속 구멍 중심의 XY는 기존 모델 기준 BASE (500, 0)mm입니다.
  이는 배치 참고이며 실행 좌표를 덮어쓰지 않습니다.
- RViz의 지그·케이크 배치는 임시입니다. 상면 높이는 로봇에서 기록한 영점을 사용합니다.

## 단일 주문과 중복 처리

1. 새 주문은 `received`로 저장합니다. 활성 주문이 있으면 다른 주문은 HTTP 409로 거부합니다.
2. 프론트는 `Idempotency-Key`를 보내며, 응답 유실 후 같은 그림을 재시도할 때 키를 재사용합니다.
3. 같은 키·같은 그림은 기존 번호를 반환합니다. 같은 키·다른 그림은 거부합니다.
4. 브릿지는 주문을 원자적으로 `claimed`로 인수하고, `dispatching` 저장 후 Goal을 한 번 전송합니다.
5. TM 수락은 `running`, 확정 결과는 `completed` 또는 `failed`, 거절은 `rejected`입니다.
6. 시간 초과·전송 불명확은 `uncertain`으로 남기고 새 주문도 막습니다. Goal을 자동 재전송하지 않습니다.

브릿지의 인수 응답이 유실돼도 해당 주문은 다시 인수되지 않습니다.
프로세스가 `claimed`/`dispatching`/`running` 상태에서 중단돼도 자동 재할당하지 않습니다.
완료·실패·거절 또는 운영자의 수동 확인이 끝난 뒤에만 다음 주문을 받습니다.

DB 원본과 번호는 보존합니다. 이전 버전 주문은 실행 이력을 알 수 없으므로 처음 이관할 때
`uncertain`으로 둡니다. 이전 주문이 남아 있으면 접수가 막히는 것이 의도된 동작입니다.
`Idempotency-Key`를 보내지 않는 외부 호출도 단일 활성 주문 제한을 받지만,
종료 후의 늦은 재시도를 같은 주문으로 구분할 수는 없습니다.

## 실행

첫 번째 터미널에서 백엔드를 실행합니다. 토큰은 브릿지 터미널과 같은 값으로 설정합니다.
아래 문자열은 예시이므로 두 터미널에서 팀이 정한 값을 사용합니다.

```bash
cd ~/ws_cobot_pjt/backend
export CAKE_BRIDGE_TOKEN='cake-local-test-change-me'
.venv/bin/uvicorn app.main:app --host 127.0.0.1 --port 5000
```

다른 터미널에서 기존 방식으로 프론트를 켭니다.

```bash
cd ~/ws_cobot_pjt/frontend
npm run dev
```

로봇은 별도 터미널에서 기존 `sodreal` 또는 `sodvir` 방식으로 연결합니다.
브릿지는 드라이버나 RViz를 자동 실행하지 않습니다.

브릿지 터미널에서는 수업 로봇 환경과 새 패키지 환경을 함께 불러옵니다.
현재 노트북은 보존한 `ws_cobot_pjt_jihwan/ws_dsr/install`을 사용합니다.
새 `ws_cobot_pjt/ws_dsr`를 빌드했다면 아래 두 번째 source를 새 설치 경로로 바꿉니다.
기존 sod 별칭은 아직 없는 새 설치 경로를 가리키므로 이 안내에서는 직접 source합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash
export ROS_DOMAIN_ID=30
cd ~/ws_cobot_pjt/ws_cobot1
colcon build --symlink-install --packages-up-to cake_decorator
source install/local_setup.bash
export CAKE_BRIDGE_TOKEN='cake-local-test-change-me'
ros2 run cake_decorator ros_bridge_node
```

`YES` 후 조회한 TCP가 표시됩니다. TM Action 서버가 준비되기 전에는 주문을 인수하지 않습니다.
기본 TM 주소는 `/cake/order`, 타입은 `cake_decorator_interfaces/action/CakeOrder`입니다.
TM 개발자는 동일한 Action 주소를 제공해야 합니다. 주소를 바꾸려면 다음 옵션을 사용합니다.

```bash
ros2 run cake_decorator ros_bridge_node --action-name /팀에서정한주소
```

TCP 조회 기본 서비스는 `/dsr01/dsr_controller2/aux_control/get_current_posx`입니다.
`--pose-service`, `--backend-url`, `--result-timeout`으로 연결과 결과 대기 시간을 바꿀 수 있습니다.
기본 결과 대기는 1800초입니다. 결과 대기 종료는 로봇 정지 명령이 아닙니다.

## 내부 HTTP 계약

`Authorization: Bearer <CAKE_BRIDGE_TOKEN>`이 필요합니다.
토큰이 설정되지 않으면 내부 API만 HTTP 503으로 비활성화되고 그림 저장은 사용할 수 있습니다.
프론트의 `/api` 프록시에 `/internal`은 추가하지 않습니다. HMI API도 추가하지 않았습니다.

| API | 용도 |
|---|---|
| POST `/internal/bridge/claim` | 원자적 인수. 주문·원본 drawing·실측 cake·claim_token 반환. 없으면 204 |
| POST `/internal/bridge/orders/{id}/events` | 인수 토큰과 이벤트 ID로 전송·수락·진행·결과 기록 |
| POST `/internal/bridge/orders/{id}/resolve` | 실행이 끝났음을 사람이 확인한 불명확 주문의 잠금 해제 |

이벤트는 `event_id`, `claim_token`, `kind`를 필수로 받습니다.
선택 필드는 `phase`, `progress_pct`, `message`, `error_code`, `elapsed_time_sec`입니다.
같은 이벤트 ID와 같은 본문으로 재시도하면 중복 적용하지 않습니다.
브릿지는 Action Feedback과 활성 주문의 `/task_status`를 백엔드로 전달합니다.
주문 목록·상세에서 상태·진행·메시지를 확인할 수 있으며 HMI는 이를 아직 사용하지 않습니다.

수동 해제는 장비 정지 기능이 아닙니다. 작업이 실행 중이지 않음을 현장에서 확인한 뒤에만 사용합니다.
주문 번호는 `/api/orders`에서 확인합니다.

```bash
curl -X POST http://127.0.0.1:5000/internal/bridge/orders/주문번호/resolve \
  -H "Authorization: Bearer $CAKE_BRIDGE_TOKEN" \
  -H 'Content-Type: application/json' \
  -d '{"confirmed_not_running":true,"reason":"장비와 주문이 실행 중이 아님을 현장 확인"}'
```

## 형상 재생성과 검증

실측 치수를 바꿨다면 다음 명령으로 RViz용 새 원뿔대와 판 형상을 갱신합니다.
원본 A안 STL은 보존합니다. 이 명령은 URDF의 케이크·판 형상만 갱신합니다.

```bash
cd ~/ws_cobot_pjt
python3 scripts/generate_test_cake_model.py
```

단위·원점 변환, 백엔드 경쟁 접수·재시도·마이그레이션, 실제 HTTP↔가짜 ROS TM 통신을 테스트합니다.
통합 테스트는 별도 도메인에서 실행하며 실제 로봇 브링업을 시작하지 않습니다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 backend/.venv/bin/python -m pytest -q backend/tests
source /opt/ros/jazzy/setup.bash
source ws_cobot1/install/local_setup.bash
ROS_DOMAIN_ID=173 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q ws_cobot1/src/cake_decorator/test
```

`Stroke2D.is_closed`는 제거했습니다. 사용자가 원을 닫아 그렸으면 그 좌표를 따라가고,
별도의 자동 끝점 연결은 하지 않습니다. 타입 변경 후 관련 워크스페이스를 다시 빌드해야 합니다.
