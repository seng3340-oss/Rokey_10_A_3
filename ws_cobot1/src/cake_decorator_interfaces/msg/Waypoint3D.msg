# 목표 위치는 m, 자세는 쿼터니언이다. 두산 mm·ZYZ 각도로의 변환은 실행 측 책임이다.
geometry_msgs/Pose pose
# PM 상세 정의 기준 속도 단위는 mm/s다.
float64 speed
# 표면 접촉/비접촉 구분이다. 이 값만으로 실제 잉크 접촉이 검증되는 것은 아니다.
bool pen_down
# 목표 접촉력 N. 현재 높이 고정 드로잉에서는 사용하지 않는다.
float64 contact_force

# 기본값 LINE=0은 기존 직선 명령과 호환된다. CIRCLE은 현재점→via_pose→pose 원호다.
uint8 LINE=0
uint8 CIRCLE=1
uint8 motion_type
geometry_msgs/Pose via_pose
# 높이가 함께 변하는 옆면 곡선의 연속 스플라인 목표점(명령당 2~100개).
uint8 SPLINE=2
geometry_msgs/Pose[] spline_poses
