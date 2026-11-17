"""원뿔대의 중간 수평 단면만 기존 원호 실행기로 검증하는 별도 주문이다."""

import json
import math
from pathlib import Path
from typing import Annotated

from fastapi import APIRouter, HTTPException, Request
from pydantic import Field

from .cake import load_cake_settings
from .orders import save_order
from .schemas import ContractModel, DrawingReceipt, DrawingRequest

router = APIRouter(prefix="/api/tests", tags=["드로잉 검증"])
CONFIG_PATH = Path(__file__).resolve().parents[2] / "config/side_midline_test.json"


class SideMidlineRequest(ContractModel):
    """길이는 펼친 옆면의 호 길이 mm다. 임의 그림·실행 좌표는 받지 않는다."""

    line_length_mm: Annotated[
        float, Field(strict=True, ge=5, le=50, allow_inf_nan=False)
    ]


def build_side_midline_test(line_length_mm):
    """중간 단면 지름과 양 끝 픽셀을 만든다. 위치는 실행 시 TCP 영점이다.

    원뿔대 반지름은 높이에 따라 선형으로 변한다. 딱 중간 높이의 수평선에
    한정하면 평균 지름의 원과 정확히 일치하므로 일반 형상 코드를 바꾸지 않는다.
    이 등가 반지름을 다른 높이의 선이나 대각선에 사용해서는 안 된다.
    """
    recipe = json.loads(CONFIG_PATH.read_text())
    if (
        not recipe["min_line_length_mm"]
        <= line_length_mm
        <= recipe["max_line_length_mm"]
    ):
        raise ValueError("테스트 선 길이는 5~50mm여야 합니다.")
    diameter = (recipe["top_diameter_mm"] + recipe["bottom_diameter_mm"]) / 2
    # 옆면 편집 띠의 중앙 X는 BASE -X 방향(로봇을 향한 쪽)이다.
    half_width_px = line_length_mm / (math.pi * diameter) * 896 / 2
    drawing = DrawingRequest.model_validate(
        {
            "canvas_width": 1024,
            "canvas_height": 768,
            "drawing_surface": "side",
            "strokes": [
                {
                    "stroke_id": 0,
                    "points": [
                        {"x": 512.0 - half_width_px, "y": 384.0},
                        {"x": 512.0 + half_width_px, "y": 384.0},
                    ],
                }
            ],
        }
    )
    cake = load_cake_settings()
    cake.update(diameter_mm=diameter, height_mm=recipe["height_mm"])
    # DB에는 실측값을 함께 남기고 브릿지가 일반 옆면 주문과 구별하게 한다.
    cake["test_recipe"] = {
        "kind": "side_midline_frustum",
        "top_diameter_mm": recipe["top_diameter_mm"],
        "bottom_diameter_mm": recipe["bottom_diameter_mm"],
        "height_mm": recipe["height_mm"],
        "line_length_mm": line_length_mm,
    }
    return drawing, cake


@router.post("/side-midline", status_code=201, response_model=DrawingReceipt)
def receive_side_midline(body: SideMidlineRequest, request: Request):
    """기존 단일 활성 주문·중복 방지 규칙으로 테스트 한 획만 접수한다."""
    key = request.headers.get("Idempotency-Key")
    if key is not None and (not key.strip() or len(key) > 128):
        raise HTTPException(
            422, "요청 키는 비어 있지 않은 128자 이하 문자열이어야 합니다."
        )
    drawing, cake = build_side_midline_test(body.line_length_mm)
    order_id = save_order(request.app.state.database, drawing, key, cake_override=cake)
    return DrawingReceipt(order_id=order_id)
