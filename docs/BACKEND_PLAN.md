# 현재 백엔드 구조

FastAPI로 그림 접수·검증·SQLite 저장·주문 조회를 구현했습니다.
사용자 요청에 따라 ROS 브리지와 HMI API는 이번 범위에서 제외했습니다.

```text
backend/
├── app/
│   ├── main.py          # 앱 생성, 본문 제한, 오류 응답, health
│   ├── schemas.py       # PM 요청과 응답 검증
│   ├── drawings.py      # POST /api/drawing
│   ├── orders.py        # 주문 번호 생성, 저장, 목록·상세
│   └── database.py      # SQLite 연결과 초기 테이블
├── tests/
├── requirements.txt
├── requirements-dev.txt
└── README.md
```

## 현재 흐름

태블릿 → POST /api/drawing → 입력 검증 → SQLite 저장 → 주문 ID/received 응답.
최근 주문과 원본 그림은 GET /api/orders 및 GET /api/orders/{order_id}로 조회합니다.
주문은 서버 접수 상태에 머물며 robot_dispatched는 false입니다.
재시작 후에도 주문·날짜별 번호를 보존하고 동시 생성 시 같은 DB 트랜잭션으로 중복 번호를 방지합니다.

## 현재 범위 밖

HMI는 팀에서 표시할 데이터를 정할 때까지 정적 예시 화면입니다.
HMI API·폴링·ROS 브리지·Topic 발행·로봇 실행·제작 완료 추정은 하지 않습니다.
로그 조회와 주문 상태 변경도 아직 제공하지 않습니다.
PM의 /drawing/order Topic 계약은 이후 ROS 연결을 위한 명세로만 유지합니다.

## 후속 합의

- 두 면을 한 주문으로 제작하는 형식. 현재 한 요청에 top 또는 side 하나만 받습니다.
- 재시도 중복 방지 키와 주문 그룹, 운영자 권한.
- 로봇 상태/작업 접수/오류/완료 회신 규격과 좌표 변환의 담당 노드.
- 원형/곡면 편집 영역과 실제 케이크 mm 치수·옆면 각도의 매핑.

백엔드는 원본 픽셀 값을 보존합니다. 현재 입력 검증은 캔버스 경계까지이며,
실제 로봇 작업 범위와 잉크 품질이 검증된 것은 아닙니다.
실행·제한값·응답 예시는 ../backend/README.md를 참고합니다.
