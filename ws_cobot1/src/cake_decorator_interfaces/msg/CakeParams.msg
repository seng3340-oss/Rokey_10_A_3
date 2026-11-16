# 치수는 m다. 원통형 케이크의 반지름 하나를 전달한다.
float64 radius
float64 height
float64 board_width
float64 board_depth
float64 board_thickness
# center는 BASE 기준 윗면 중심이다. 사용자가 접촉 확인한 현재 활성 TCP에서 읽는다.
geometry_msgs/Point center
# 케이크 평면 회전 rad. 최초 테스트는 BASE X/Y와 나란한 수평 상면을 가정한다.
float64 orientation
string reference_frame
# 영점에서 기록한 펜촉 TCP 자세. 케이크 상면 법선과 혼동하지 않는다.
geometry_msgs/Quaternion tool_orientation
