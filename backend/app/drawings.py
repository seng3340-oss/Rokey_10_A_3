"""프론트에서 그림을 받는 API. 검증과 저장이 끝나야 접수 응답을 반환한다."""

from fastapi import APIRouter, Request

from .orders import save_order
from .schemas import DrawingReceipt, DrawingRequest


router = APIRouter(prefix="/api", tags=["그림 접수"])


@router.post("/drawing", status_code=201, response_model=DrawingReceipt)
def receive_drawing(drawing: DrawingRequest, request: Request):
    """Pydantic 검증을 통과한 그림을 저장하며 로봇으로 전송하지는 않는다."""
    order_id = save_order(request.app.state.database, drawing)
    return DrawingReceipt(order_id=order_id)
