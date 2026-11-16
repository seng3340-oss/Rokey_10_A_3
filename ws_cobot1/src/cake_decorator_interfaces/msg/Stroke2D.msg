# 웹에서 받은 원본 픽셀 획이다. 획 번호는 0부터 순서대로 유지한다.
uint32 stroke_id
# Point32의 x/y는 px, z는 0이다. 점 선별·mm 변환은 DP가 담당한다.
# 마지막 점과 첫 점을 자동 연결하지 않고 실제 입력 좌표만 따른다.
geometry_msgs/Point32[] points
