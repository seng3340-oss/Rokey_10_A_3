"""제작 순서를 관리할 노드의 자리. 아직 구현하지 않는다."""

# 예정 역할: CakeOrder ActionServer로 제작 요청을 받고 주문·준비 상태를 검사한다.
# GenerateTrajectory 서비스 클라이언트로 DP에 원본 좌표·해상도·케이크 정보를 보낸다.
# 계산된 경로를 DrawOnCake ActionClient로 RE에 전달하고 실제 결과를 확인한다.
# PickAndPlace·GraspTool은 운반·펜 자동 파지를 구현할 때 연결한다.
# 현재 단계·진행률을 CakeOrder Feedback과 /task_status Topic으로 전달한다.
# Action 서버이면서 다른 Action·Service의 클라이언트 역할도 맡는다.
# 콜백 안에서 응답 처리를 막지 않도록 비동기 호출과 콜백 그룹을 설계한다.
# 동시 주문 실행 제한·실패·취소·파지 중 복구 규칙은 구현 전에 확정한다.
# 순서 관리만 담당하며 좌표 계산과 DRL API 호출은 각각 DP·RE에 맡긴다.

# 현재 원통형·원본 픽셀 계약: CakeOrder.cake_params를 DP 요청에 매핑한다.
# 원통형 radius/height를 cake_radius/cake_height로 전달한다.
# cake_pose.position은 BASE 윗면 중심, cake_pose.orientation은 케이크 프레임 방향이다.
# tool_orientation은 별도 필드로 전달한다. 펜 방향을 케이크 프레임 회전으로 쓰지 않는다.
