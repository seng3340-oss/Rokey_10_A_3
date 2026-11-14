"""PM이 정한 픽셀 좌표 계약과 서버 응답을 한곳에서 정의한다."""

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


# 수치를 문자열·불리언에서 자동 변환하지 않는다. NaN/무한대도 좌표로 받지 않는다.
Pixel = Annotated[float, Field(strict=True, ge=0, allow_inf_nan=False)]
CanvasSize = Annotated[int, Field(strict=True, gt=0, le=8192)]
StrokeId = Annotated[int, Field(strict=True, ge=0, le=4_294_967_295)]


class ContractModel(BaseModel):
    """오타나 예상하지 못한 필드를 조용히 버리지 않도록 공통 설정한다."""

    model_config = ConfigDict(extra="forbid")


class Point2D(ContractModel):
    """캔버스 왼쪽 위를 원점으로 하는 픽셀 좌표다. 로봇 mm 좌표가 아니다."""

    x: Pixel
    y: Pixel


class Stroke(ContractModel):
    """한 번 이어서 그린 획이다. 단일 점은 이동 경로가 없어 받지 않는다."""

    stroke_id: StrokeId
    points: Annotated[list[Point2D], Field(min_length=2, max_length=5000)]


class DrawingRequest(ContractModel):
    """태블릿의 POST /api/drawing 본문. 한 요청은 한 면만 포함한다."""

    canvas_width: CanvasSize
    canvas_height: CanvasSize
    drawing_surface: Literal["top", "side"]
    strokes: Annotated[list[Stroke], Field(min_length=1, max_length=100)]

    @model_validator(mode="after")
    def validate_stroke_coordinates(self):
        """개별 필드 검사 후 획 순서·전체 점 개수·캔버스 경계를 검사한다."""
        total_points = sum(len(stroke.points) for stroke in self.strokes)
        if total_points > 20_000:
            raise ValueError("한 그림의 좌표점은 20,000개 이하여야 합니다.")

        for expected_id, stroke in enumerate(self.strokes):
            # 중복 획 ID나 누락된 번호가 제작 순서를 모호하게 만들지 않게 한다.
            if stroke.stroke_id != expected_id:
                raise ValueError("stroke_id는 0부터 순서대로 이어져야 합니다.")

            for point in stroke.points:
                if point.x > self.canvas_width or point.y > self.canvas_height:
                    raise ValueError("좌표점이 캔버스 경계를 벗어났습니다.")

        # 원형 윗면/옆면의 실제 유효 작업 영역과 mm 변환은 ROS 통합 단계에서 검사한다.
        return self


class DrawingReceipt(ContractModel):
    """DB 저장 성공을 알리는 응답이다. 로봇 실행이나 제작 완료를 뜻하지 않는다."""

    order_id: str
    status: Literal["received"] = "received"


class OrderSummary(DrawingReceipt):
    """목록에는 큰 좌표 배열을 제외하고 주문을 식별하는 정보만 싣는다."""

    drawing_surface: Literal["top", "side"]
    created_at: str
    robot_dispatched: Literal[False] = False


class OrderDetail(OrderSummary):
    """주문 상세에는 검증 후 저장한 원본 픽셀 데이터도 반환한다."""

    drawing: DrawingRequest


class OrderPage(ContractModel):
    """무제한 전체 조회를 피하도록 목록의 범위와 총 개수를 함께 반환한다."""

    items: list[OrderSummary]
    total: int
    limit: int
    offset: int
