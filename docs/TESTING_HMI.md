# 최신 main 기반 HMI 테스트

Ubuntu 24.04·ROS 2 Jazzy·Python 3.12·Node.js 22.12 이상을 기준으로 합니다.
아래 명령은 저장소 루트에서 실행합니다. 실제 장비 드라이버와 운용 DB는 실행하지 않습니다.

## 1. 백엔드와 화면 검사

```bash
python3 -m venv --system-site-packages /tmp/cake-hmi-test-venv
/tmp/cake-hmi-test-venv/bin/python -m pip install -r backend/requirements-dev.txt
PYTHONPATH=backend:ws_cobot1/src/cake_decorator \
  /tmp/cake-hmi-test-venv/bin/python -m pytest backend/tests -q

cd frontend
npm ci
npm test
npm run build
cd ..
```

백엔드 검사는 임시 DB로 주문·인수 토큰·상태·계획 저장·연결 정보 노후화를 확인합니다.
화면 검사는 공정 이름, 조회 계약, 입력·계획 표시와 타입/운용 빌드를 확인합니다.

## 2. ROS 빌드와 장비 없는 통합 검사

`dsr_msgs2`, `onrobot_rg_msgs` 등 프로젝트의 벤더 ROS 메시지가 설치돼 있어야 합니다.
다음 예시는 이 노트북의 벤더 설치본을 읽습니다. 다른 PC에서는 해당 경로를 설치된 벤더 작업공간으로 바꿉니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/rokey/install/local_setup.bash

colcon --log-base /tmp/cake-hmi-test-log build \
  --base-paths ws_cobot1/src/cake_decorator_interfaces ws_cobot1/src/cake_decorator \
  --build-base /tmp/cake-hmi-test-build \
  --install-base /tmp/cake-hmi-test-install \
  --packages-select cake_decorator_interfaces cake_decorator
source /tmp/cake-hmi-test-install/local_setup.bash

export ROS_DOMAIN_ID=231
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/cake-hmi-test-ros-log
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/ws_cobot1/src/cake_decorator:$PYTHONPATH"
export CAKE_TEST_BACKEND_PYTHON=/tmp/cake-hmi-test-venv/bin/python
export CAKE_RUN_ROS_TESTS=1
export CAKE_HMI_TEST_ARTIFACTS=/tmp/cake-hmi-test-results

"$CAKE_TEST_BACKEND_PYTHON" -m pytest ws_cobot1/src/cake_decorator/test -q
```

231번 도메인은 테스트 전용으로 비어 있어야 하며 이 도메인에 실제 로봇 드라이버를 띄우지 않습니다.
테스트가 기대하는 도메인 번호는 231입니다. 실제 RE 실행 명령은 위 절차에 포함하지 않습니다.
`CAKE_RUN_ROS_TESTS=1`을 생략하면 실제 ROS/HTTP HMI 파이프라인 검사는 건너뜁니다.
`CAKE_TEST_BACKEND_PYTHON`을 지정해야 브릿지 HTTP 통합 검사도 활성화됩니다.

확인하는 내용:

- 원뿔대 치수·TOP/SIDE 픽셀 변환·곡선 표본·하단 10mm 제한
- 플레이트 준비 → TOP/SIDE 그리기 → 회수 순서와 기존 취소/정리 계약
- HMI 공정별 진행률, 오류 전달, 다른 주문·늦은 응답·NaN 배제
- 케이크 yaw·SIDE 전용 기준점·펜올림 구간·스플라인 표시
- HMI 표시 채널 장애가 기존 로봇 공정을 실패시키지 않는지
- 임시 HTTP 서버·실제 TM/DP·모의 DrawOnCake를 거친 성공/실패 결과와 계획 조회

## 3. 화면만 확인

같은 PC의 두 터미널에서 실행합니다. 새 임시 DB만 사용하며 ROS 브릿지는 실행하지 않습니다.

```bash
# 터미널 A: 저장소 루트
CAKE_DATABASE_PATH=/tmp/cake-hmi-preview.sqlite3 \
  /tmp/cake-hmi-test-venv/bin/python -m uvicorn app.main:app \
  --app-dir backend --host 127.0.0.1 --port 5000

# 터미널 B
cd frontend
npm run dev
```

브라우저에서 `http://localhost:8080/workspace` 또는 `/hmi`를 엽니다.
주문이 없고 TM 연결은 미관측으로 표시되는 것이 정상입니다. 고객 입력 중 그림은 같은 브라우저의
작성 화면과 BroadcastChannel로 공유되며, 실측·실행 계획으로 표시하지 않습니다.
서버와 브릿지 토큰을 설정하지 않은 이 미리보기에서는 주문이 실제 TM으로 전달되지 않습니다.
모의 ROS 파이프라인 결과는 `/tmp/cake-hmi-test-results/ros-pipeline-results.json`에서 확인합니다.

## 수행 결과

2026-11-19 최신 main `1cc52b5` 기준 통합 사본에서:

- 백엔드 48개 통과
- ROS 182개 및 하위 검사 20개 통과, 외부 참고 프로젝트 부재로 1개 건너뜀
- 표시 채널 장애 처리 최종 보완 후 통합 공정 5개 재검사 통과
- 프론트엔드 테스트 파일 5개와 타입 검사·빌드 통과
- ROS 인터페이스/업무 패키지 2개 빌드 성공

실물 운전·장비 접촉 검증은 포함하지 않습니다. 화면 빌드에는 기존 Kiosk 묶음 크기 경고가 있으며 빌드는 성공합니다.
