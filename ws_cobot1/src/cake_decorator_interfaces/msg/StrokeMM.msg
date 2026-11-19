# 획 데이터 (mm 단위 좌표)
# original_mm: 원본 픽셀 좌표 (브릿지 통과 후 mm 변환)
# simplified_mm: 간소화된 경로 포인트
# target_mm: 목표 3D 좌표 (드로잉 경로)
string stroke_id        # 획 식별자
string color            # 획 색상 (예: "black")
# 원본 픽셀 좌표 또는 mm 변환 좌표 리스트
cake_decorator_interfaces/PointMM[] original_mm
cake_decorator_interfaces/PointMM[] simplified_mm
cake_decorator_interfaces/PointMM3D[] target_mm