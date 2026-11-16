# 프론트엔드에서 전달하는 획 데이터 (mm 단위)
# original_mm → simplified_mm → target_mm 순으로 가공됨
string stroke_id                      # 획 식별자 (예: "cake-boundary")
string color                          # 획 색상 (예: "black")
cake_decorator_interfaces/PointMM[] original_mm    # 원본 좌표 [x, y] mm
cake_decorator_interfaces/PointMM[] simplified_mm  # 간소화 좌표 [x, y] mm
cake_decorator_interfaces/PointMM3D[] target_mm    # 최종 좌표 [x, y, z] mm