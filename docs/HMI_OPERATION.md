# 실제 주문과 전체 공정 HMI

`./start_full.sh`로 전체 공정을 시작하면 브릿지가 TM 발견 상태를 2초마다 보고합니다. `/hmi`는 `/api/hmi`를 요청 완료 후 1초 간격으로 조회합니다. 8초 이상 갱신되지 않은 TM 관측은 연결됨으로 표시하지 않습니다. API 오류 시 마지막 데이터에 오래된 표시를 붙입니다.

HMI는 실제 주문과 주문별 전체10단계, 진행률, 시작/완료/실패 이력을 표시합니다. 실행 노드의 단계 관측은 주문 ID로 연결되며, 빈 ID의 수동 서비스 호출을 다른 주문에 붙이지 않습니다. 기록 없는 단계는 완료로 추정하지 않습니다. 단계 배분 진행률은 물리 측정값이 아닙니다.

관측 흐름: RE `/cake/process_events` → ROS 브릿지의 현재 주문 대조 → 인증된 기존 이벤트 API → SQLite `hmi_events` → `/api/hmi` → HMI. TM은 주문 ID가 있는 `RunPlateSequence` 서비스와 `DrawOnCake.order_id`로 실행을 식별합니다. 기존 Trigger 플레이트 서비스는 유지됩니다. 로봇 이동 좌표·속도·명령 순서는 바꾸지 않았습니다.

브라우저는 조회만 합니다. 비상정지와 물리 로봇 연결 신호는 연결하지 않았고 `unknown`/`null`로 표시합니다. TM 발견이나 API 성공을 안전 확인으로 사용하지 않습니다. 브릿지 토큰·인수 토큰은 HMI 응답에 포함하지 않습니다. 기존 DB는 테이블 추가만 하며 주문과 잠금을 보존합니다. 예전 이벤트의 원문이 없는 기간은 복원하지 않습니다.

검사:
```bash
cd ~/Rokey_10_A_3
export PATH="$PWD/.runtime/node/bin:$PATH"
cd frontend && npm test && npm run build
```

ROS 통합 검사는 명시적으로 장비와 분리된 도메인231에서만 실행합니다. TM·DP·RE·브릿지는 실제 코드이며 DoosanRobotClient만 명령을 전송하지 않는 대역입니다. 실제 드라이버를 시작하지 않습니다.

```bash
cd ~/Rokey_10_A_3
source /opt/ros/jazzy/setup.bash
source ws_dsr/install/local_setup.bash
source ws_cobot1/install/local_setup.bash
export ROS_DOMAIN_ID=231 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export CAKE_RUN_HMI_ROS_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD/backend/.venv/lib/python3.12/site-packages:$PWD/backend:$PYTHONPATH:/usr/lib/python3/dist-packages"
backend/.venv/bin/python -m pytest -q ws_cobot1/src/cake_decorator/test/test_hmi_full_pipeline.py
```

실행 범위의 ROS 오류 로그(`/rosout`, ERROR 이상)도 브릿지 관측으로 전달합니다. 대상은 TM·DP·RE와 실제 드라이버 노드로 제한하며, 로봇 제어나 오류 자동 해제를 하지 않습니다.
