# 그림 접수 서버

FastAPI로 PM 형식의 그림을 검증하고 SQLite에 저장합니다.
접수 성공은 서버 저장 성공입니다. ROS 브리지·HMI API·로봇 제어는 포함하지 않습니다.

## 실행

Python 3.12 기준입니다.

```bash
cd ~/ws_cobot_pjt/backend
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/uvicorn app.main:app --host 127.0.0.1 --port 5000
```

별도 터미널에서 frontend의 `npm run dev`를 실행합니다.
태블릿은 노트북의 Wi-Fi 주소 `http://노트북-IP:8080`에 접속합니다.
Vite가 `/api`를 노트북 내부 `127.0.0.1:5000`으로 전달합니다.
이 서버는 로봇 브링업이나 모션을 실행하지 않습니다.

API 문서: http://127.0.0.1:5000/docs

## 파일

- `app/main.py`: 앱 생성, 요청 크기 제한, 오류 응답, 서버 확인
- `app/schemas.py`: PM 요청/응답과 좌표 검증
- `app/drawings.py`: 그림 접수 API
- `app/orders.py`: 주문 번호 생성, 저장, 목록/상세 조회
- `app/database.py`: SQLite 연결과 초기 테이블
- `tests/test_drawings.py`: 잘못된 입력, 영구 저장, 동시 주문, 조회 검사

FastAPI는 schemas 검증에 성공한 요청만 drawings 함수에 전달합니다.
그 후 orders가 database를 통해 저장하고 접수 응답을 반환합니다.

## API

`POST /api/drawing`은 PM의 canvas_width, canvas_height, drawing_surface, strokes를 그대로 받습니다.
좌표는 원본 픽셀 값이며 로봇 좌표로 변환하지 않습니다. 한 요청은 한 면입니다.

```json
{
  "canvas_width": 1024,
  "canvas_height": 768,
  "drawing_surface": "top",
  "strokes": [
    {"stroke_id": 0, "points": [{"x": 500.5, "y": 350.0}, {"x": 520.0, "y": 360.0}]}
  ]
}
```

저장 성공은 HTTP 201과 `{"order_id":"ORD-20261114-001","status":"received"}`로 응답합니다.
이것은 프론트와 맞춘 서버 접수 응답이며 로봇 접수·제작 완료와 다릅니다.

- `GET /api/orders?limit=20&offset=0`: 최신 순 목록. items, total, limit, offset 반환. 좌표 배열 제외.
- `GET /api/orders/{order_id}`: 원본 drawing과 상세. robot_dispatched는 false. 없는 주문은 404.
- `GET /api/health`: 서버 상태. ros_connected는 false.

HMI·브리지 API와 주문 상태 변경 API는 만들지 않았습니다.
주문 목록·상세는 향후 연결용이며 현재 정적 HMI가 사용하지 않습니다.
두 면을 한 주문으로 묶는 형식은 팀에서 확정해야 합니다.

## 저장·검증

기본 DB는 backend/data/orders.sqlite3이며 앱 시작 시 준비합니다.
CAKE_DATABASE_PATH 환경변수로 다른 경로를 설정할 수 있습니다.
주문과 날짜별 일련번호는 같은 트랜잭션으로 저장하여 동시 요청·재시작 시 번호 중복을 막습니다.
DB를 삭제하면 주문과 번호 기록도 지워집니다. DB와 WAL 파일은 Git에 올리지 않습니다.

제한: 캔버스 1~8192px, 그림 1~100획, 획당 2~5000점, 전체 최대 20,000점, 본문 최대 2MiB.
엄격한 숫자 타입·유한수·캔버스 경계·0부터 연속된 획 ID를 검사합니다.
실제 원형/곡면 작업 영역, mm 치수, TCP 자세와 로봇 도달 가능성은 아직 검사하지 않습니다.
검증 오류 422, 본문 초과 413, 저장소 오류 503으로 응답합니다.

재전송은 별도 주문으로 생성됩니다. 응답 타임아웃 후에는 재요청 전 주문 조회로 접수 여부를 확인해야 합니다.
권한 분리와 요청 idempotency 키는 팀 규격 확정 후 추가합니다.

## 테스트

```bash
.venv/bin/python -m pip install -r requirements-dev.txt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

ROS를 source한 셸의 pytest 플러그인이 섞이지 않도록 자동 로드를 끕니다.
테스트별 임시 DB를 사용하므로 개발 주문을 변경하지 않습니다.
