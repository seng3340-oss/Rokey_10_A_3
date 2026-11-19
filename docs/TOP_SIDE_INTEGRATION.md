
### 웹 주문 본문 확장

기존 한 면 필드(canvas_width, canvas_height, drawing_surface, strokes)를 유지한다.
두 면이면 TOP을 기본 본문으로 보내고 `additional_paths: [SIDE의 동일한 필드]`를 추가한다.
추가 면은 최대 하나이며 중첩하지 않는다. SQLite에는 같은 주문의 drawing_json에 함께 저장한다.
백엔드와 브릿지 모두 두 면 합계 100획/20,000점 제한과 각 면의 픽셀 범위를 검사한다.

그리기 Action 취소는 자식 실행기에 전달하고 정리 결과를 기다린다. 결과가 불명확하면 새 주문을 막는다.
운반의 Trigger 서비스는 취소를 지원하지 않으므로 PREPARING/FINISHING 단계에서는 ROS 주문 취소를 거절한다.
웹/ROS 취소는 물리적 비상정지 기능이 아니다.
