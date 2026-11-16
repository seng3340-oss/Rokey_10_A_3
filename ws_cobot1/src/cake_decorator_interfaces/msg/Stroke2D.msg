# 웹의 0부터 시작하는 획 번호를 보존한다. 획 사이를 자동 연결하지 않는다.
uint32 stroke_id
# 원본 픽셀 x/y와 z=0. 마지막 점과 첫 점을 자동 연결하는 필드는 두지 않는다.
geometry_msgs/Point32[] points
