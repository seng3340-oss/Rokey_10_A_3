# 인터페이스 합의 기록

2026-11-15 사용자가 추천안 세 가지를 승인했습니다. 이번 확정 범위는 아래 세 항목입니다.

| ID | 확정 내용 | 적용 위치 |
|---|---|---|
| D01 | 이름·기능 분리는 PM의 상세 정의 기준. CakeOrder / PickAndPlace / GraspTool / DrawOnCake / GenerateTrajectory 사용 | 기존 상세 IDL 유지, 보고서 시퀀스 이름 정리 |
| D02 | 원본 픽셀 좌표와 캔버스 크기를 ROS까지 함께 전달 | DrawingPath.msg, GenerateTrajectory.srv에 canvas_width/height uint32 추가 |
| D03 | 웹 stroke_id를 ROS에도 보존하고 자동 폐곡선 연결 금지 | Stroke2D.msg에 uint32 stroke_id 추가, is_closed=false 규칙 |

픽셀 원점은 왼쪽 위, +X 오른쪽, +Y 아래를 유지합니다. 좌표점을 임의로 정규화하지 않습니다.
타입 정의만으로 값 검증이 수행되지는 않습니다. 다음 노드 구현에서 해상도 양수·획 번호 순서·폐경로 금지를 검사해야 합니다.

## 아직 결정하지 않은 것

- 두 면을 같은 주문으로 제출하는 웹 API와 주문 그룹 구성
- cake_params와 TM 고정 YAML의 우선순위, center.z/cake_pose 기준점
- 전역/waypoint 속도 우선순위와 중복 pen_states 필드의 처리
- 실행 중 취소·파지 중 복구·상태 문자열 집합·오류 회신·WebSocket 규격

PM 원본 시퀀스·상태도는 SYSTEM_DESIGN.md에 보존합니다.
보고서는 확정된 상세 Action 이름으로 목표 흐름을 다시 표현하지만,
추가 상태 정책이나 로봇 동작을 확정·구현한 것은 아닙니다.
현재 HMI는 정적이며 웹 주문은 DB 저장까지만 연결됩니다.

## 2026-11-16 실물 테스트 갱신

D03의 자동 연결 금지는 유지하되 불필요한 is_closed 필드는 제거했습니다.
케이크는 실측 원뿔대(윗면113.5/아랫면120/높이79mm), 판은179×179×2mm입니다.
center는 사용자가 접촉 확인한 활성 TCP의 BASE 기준 윗면 중심으로 확정합니다.
백엔드가 주문 접수 시 치수를 사본으로 보존하며 브릿지가 현재 세션의 영점·TCP 자세를 추가합니다.
단일 활성 주문, 요청 키 기반 중복 접수 방지, 내부 HTTP 인수·결과 기록과 CakeOrder 전송을 구현했습니다.
TM·DP·RE·HMI 연동은 여전히 미구현입니다.
