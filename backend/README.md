# 그림 접수·TM 연결·HMI 서버

현재 기준은 지름100mm·높이80mm 원통, 원본 픽셀 입력, 케이크 윗면 중심입니다.
[운용 HMI 실행 안내](../docs/HMI_OPERATION.md)와 [좌표 계약](../docs/CYLINDER_PIXEL_CONTRACT.md)을 참고하세요.

FastAPI가 고객 그림을 검증·저장하고, 인증된 ROS 브릿지가 주문을 TM에 전달합니다.
HMI는 주문·TM 상태·계획 경로·연결 관측을 읽습니다. 웹 서버 자체는 ROS 드라이버나 장비를 시작하지 않습니다.
접수201은 저장 성공이며 로봇 제작 완료가 아닙니다.

## 실행

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
export CAKE_BRIDGE_TOKEN='REPLACE_WITH_YOUR_LOCAL_SECRET'
# 프론트엔드 빌드 결과도 함께 제공할 경우 절대 경로를 지정합니다.
export CAKE_FRONTEND_DIST='/absolute/project/path/frontend/dist'
.venv/bin/python -m uvicorn app.main:app --host 127.0.0.1 --port 5000
```

화면은 `/`, `/hmi`, `/workspace`이며 API 문서는 `/docs`입니다. 개발 Vite의 `/api` 프록시도 유지합니다.
DB는 기본 `backend/data/orders.sqlite3`이며 `CAKE_DATABASE_PATH`로 변경합니다.
토큰·DB·WAL/SHM 파일은 Git에 올리지 않습니다. 기존 DB에는 표시용 테이블을 추가하며 기존 주문은 보존합니다.

## 입력과 상태

`POST /api/drawing`: canvas_width, canvas_height, drawing_surface(top/side), strokes(stroke_id, points(x,y)).
캔버스1~8192px,1~100획, 획당2~5000점, 전체20,000점 이하, 본문2MiB 이하입니다.
숫자/유한수/캔버스 경계/연속된 획 ID를 검사합니다. 실제 작업 표면·경로 생성은 DP가 담당합니다.

활성 주문이 있으면409로 새 접수를 제한합니다. 같은 Idempotency-Key와 본문은 같은 주문을 반환합니다.
인수 후 전송 여부가 불명확한 주문은 자동 재전송하지 않습니다. 기존 미확정 기록은 uncertain으로 보존합니다.

## 읽기·내부 연결

- `/api/orders`, `/api/orders/{id}`: 원본·공정·진행률·오류.
- `/api/orders/{id}/monitor`: 주문별 실행 설정·현재 명령·오류·시간.
- `/api/monitor`: 브릿지/TM 연결 관측과 최신성. 물리 안전/장비 연결과 구분합니다.
- `/api/orders/{id}/plan`: TM이 받은 실제 DP 결과의 표시용 계획. 실측 경로가 아닙니다.
- `/api/health`: 웹 상태와 최신 TM 서버 발견 여부. 실제 장비 연결은 확인하지 않습니다.
- `/internal/bridge/*`: 토큰 인증된 인수·상태·계획·heartbeat·수동 확인. 공개 화면은 GET만 사용합니다.

8초 이상 갱신되지 않은 관측은 최신 연결로 표시하지 않습니다. 물리 E-stop과 실제 TCP/잉크 측정은 별도 인터페이스가 필요합니다.

## 검사

```bash
.venv/bin/python -m pip install -r requirements-dev.txt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

각 검사는 임시 DB를 사용하며 장비를 시작하지 않습니다. ROS 프로토콜까지의 검사는 운용 안내의 장비 없는 도메인231 절차를 사용합니다.
