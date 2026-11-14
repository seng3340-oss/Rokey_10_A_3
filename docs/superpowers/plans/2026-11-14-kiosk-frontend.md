# Kiosk Frontend Implementation Plan

Goal: 승인된 시안을 React·TypeScript 앱으로 이전하고 Flask 주문 API 경계를 준비한다.
Spec: docs/DESIGN.md, docs/DRAWING_PROTOCOL.md 및 승인된 design-preview 화면.

- React 화면 컴포넌트와 캔버스/Three.js 엔진을 분리한다. 입력 점을 생략하지 않는다.
- 60초 무조작 복귀와 드래그 방향, 불투명 입력 배경을 유지한다.
- PM의 한 요청 한 면 JSON 계약을 유지하고 데이터 없는 면은 요청을 차단한다.
- POST /api/drawing 어댑터를 분리한다. 오류·미연결은 접수 성공으로 처리하지 않는다.
- HMI는 예시·미연결 정적 화면으로 제공한다. HMI API나 폴링은 구현하지 않는다.
- 타입 검사·프로덕션 빌드·브라우저 시나리오를 검증한다.
- FastAPI 그림 검증·SQLite 저장·주문 조회를 구현한다. ROS 브리지와 HMI API는 제외한다.
