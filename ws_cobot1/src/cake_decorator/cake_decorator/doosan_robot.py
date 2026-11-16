"""두산 ROS·DRL API 연결을 한곳에 모을 자리. 아직 구현하지 않는다."""

# 예정 역할: ROS 노드와 DR_init을 연결하고 DSR_ROBOT2 API를 초기화한다.
# movej·movel·moveb 등 검증한 API와 TCP·Tool 조회·정지 처리를 감싼다.
# ROS 위치 m·쿼터니언을 두산 API의 mm·ZYZ degree로 변환할 책임을 둔다.
# virtual/real 연결 대상과 활성 TCP·Tool을 확인한 뒤에만 동작을 허용한다.
# RG2 제어는 기존 OnRobot 드라이버를 별도로 연결한다. 두산 API에 있다고 가정하지 않는다.
# 실행 노드가 생성되기 전에 API를 import하거나 파일 import만으로 연결·이동하지 않는다.
# 일반 정지 명령과 장비의 비상정지를 같은 기능으로 취급하지 않는다.
