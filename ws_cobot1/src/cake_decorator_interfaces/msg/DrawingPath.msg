# 원본 필기 해상도를 함께 전달해 픽셀 위치를 케이크 크기에 정확히 매핑한다.
uint32 canvas_width
uint32 canvas_height
# 상세 정의 기준 TOP/SIDE를 사용한다. 웹 top/side는 연결 경계에서 대문자로 변환한다.
string surface
# 면 안에서 stroke_id와 배열 순서가 일치하도록 원본 획을 보존한다.
cake_decorator_interfaces/Stroke2D[] strokes
