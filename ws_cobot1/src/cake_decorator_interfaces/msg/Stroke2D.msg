# 프론트엔드에서 전달하는 획 데이터 (캔버스 픽셀 좌표)
# 웹 → 백엔드 → 브릿지 → TM → DP 노드 순으로 원본 그대로 전달됨
string stroke_id                                    # 획 식별자
string color                                        # 획 색상 (예: "black")
cake_decorator_interfaces/Point2D[] points          # 캔버스 픽셀 좌표 리스트