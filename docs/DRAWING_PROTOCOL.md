# PM 그림 전달 계약 (2026-11-14)

현재 파일은 PM의 요청 규격을 기록한 문서다. React 프론트와 FastAPI 서버는 이 형식으로 그림 접수·저장까지 연결했다. ROS 발행은 아직 구현하지 않았다.
현재 접수 응답은 order_id와 status=received이며 서버 저장 성공을 뜻한다. HMI는 정적 화면으로 API를 만들지 않았다.

## 태블릿 → FastAPI

HTTP POST `/api/drawing`, Content-Type: application/json. 로그인·사용자 구분 없음.

```json
{
  "canvas_width": 1024,
  "canvas_height": 768,
  "drawing_surface": "top",
  "strokes": [
    {"stroke_id": 0, "points": [{"x": 102.5, "y": 340.2}, {"x": 110.0, "y": 345.5}]}
  ]
}
```

`canvas_width/height`: 정수 픽셀 해상도. `drawing_surface`: top 또는 side.
`stroke_id`: 정수, 획 순서대로 0부터 자동 부여. x/y: 실수 픽셀 좌표.
좌표 원점은 캔버스 좌측 상단, +X 오른쪽, +Y 아래다. CSS 화면 크기와 무관하게 내부 해상도로 환산한다.
위 JSON은 필드 예시이며 시안의 top 원 경계와는 별개다.

## FastAPI → 작업 관리 노드

ROS 2 Topic `/drawing/order`, 커스텀 메시지 DrawingOrder.
향후 백엔드가 생성한 order_id와 원시 픽셀 좌표를 전달하는 계약이다. 현재 Topic 발행이나 브리지는 구현하지 않는다.
예시 order_id: ORD-20261114-001. ID 생성은 동시 요청·재시작에도 중복되지 않아야 한다.

계약상 메시지 정의 (패키지명 cake_interfaces는 제안이며 PM과 확정 필요):

```text
# Point2D.msg
float64 x
float64 y

# Stroke.msg
uint32 stroke_id
cake_interfaces/Point2D[] points

# DrawingOrder.msg
string order_id
uint32 canvas_width
uint32 canvas_height
string drawing_surface
cake_interfaces/Stroke[] strokes
```

## 화면 시안의 작업 영역

1024×768 캔버스를 사용한다. top 원 중심은 (512,384), 반경320px다.
side는 x=64..960, y=210..558의 펼친 띠 편집 영역이다.
서버 전달값은 영역 내부 상대값이 아니라 전체 캔버스 기준 픽셀 좌표다.
면을 바꿔도 각 그림은 임시 보존하며 확인한 면 하나만 JSON으로 내보낸다.
캔버스·영역 정의 및 실제 케이크 치수는 좌표 계산 측과 공유해야 한다.
이 편집 영역의 크기를 실제 옆면 전체 둘레 또는 물리적 작업 범위로 간주하지 않는다.

## 구현 전에 확정할 항목

- 픽셀 → 로봇 BASE/TCP 좌표 변환의 담당 위치. 최신 Topic에는 로봇 경로가 아니라 원시 좌표가 담긴다.
- 고정 케이크 치수·중심·높이와 각 면의 유효 영역, 옆면의 시작각·방향·그릴 수 있는 각도.
- 원형 윗면 좌표와 펼친 옆면 좌표의 실제 mm 매핑. 기존 run3는 윗면 참고 구현이다.
- HTTP 성공 응답, 검증 오류 형식, 최대 획·점·본문 크기.
- 주문 재시도·중복 방지, ROS 연결 끊김 시 저장/재발행, 작업 관리 수신 확인.
- Topic 발행 성공을 주문 실행 접수나 제작 완료로 표시하지 않는다.
- 상태·오류·완료를 돌려줄 Topic, 작업 ID, 진행률·취소 규격. 현재 PM 규격은 단방향 주문 전달만 정의한다.
- ROS QoS 및 늦게 연결된 노드에 과거 주문이 재실행되지 않는 정책.
- 인증 없는 고객 주문 API와 운영자 제어 API의 권한 분리.

데모 화면의 예시 상태는 실제 로봇 상태가 아니다. 로봇 실행 통합은 별도 검증 후 연결한다.
