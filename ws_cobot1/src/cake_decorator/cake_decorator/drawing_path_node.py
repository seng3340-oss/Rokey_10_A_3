"""원본 좌표를 로봇 경로로 바꿀 서비스 노드의 자리. 아직 구현하지 않는다."""

# 예정 역할: GenerateTrajectory 서비스 서버로 TM의 계산 요청을 받는다.
# 입력: canvas_width/height, strokes_2d, surface_type, cake_pose, 반지름·높이.
# drawing_geometry.py를 호출해 중복점 정리·필요점 선별·실제 크기 변환을 수행한다.
# 출력: Waypoint3D 배열, 펜 상태, 예상 시간, 계산 성공 여부와 오류 설명.
# 원본 획 번호와 순서를 유지하고 서로 다른 획 사이를 그리는 선으로 연결하지 않는다.
# ROS 요청·응답 처리는 이곳에, ROS 없이 검증할 계산 함수는 drawing_geometry.py에 둔다.
# 로봇 이동이나 실제 펜 접촉을 확인한 것처럼 성공을 반환하지 않는다.
