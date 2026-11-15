# 웹이 생성한 0부터 시작하는 획 번호를 그대로 보존한다.
uint32 stroke_id
# 원본 2D 픽셀 값은 x/y에 담으며 사용하지 않는 z는 0으로 둔다.
geometry_msgs/Point32[] points
# 현재 계약은 false 고정: 마지막 점에서 첫 점으로 연결선을 자동 추가하지 않는다.
# 사용자가 원을 닫아 그렸다면 이미 저장된 좌표대로만 따라간다.
bool is_closed
