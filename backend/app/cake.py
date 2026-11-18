"""확정된 원뿔대 케이크 치수를 읽는다. 실행 위치는 이 파일에서 정하지 않는다."""

import json
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, model_validator


class CakeSettings(BaseModel):
    """치수는 mm다. 윗면·아랫면 지름을 사용하며 실행 위치는 따로 기록한다."""

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    top_diameter_mm: float = Field(gt=0)
    bottom_diameter_mm: float = Field(gt=0)
    height_mm: float = Field(gt=0)
    board_width_mm: float = Field(gt=0)
    board_depth_mm: float = Field(gt=0)
    board_thickness_mm: float = Field(gt=0)
    tool_type: str
    reference: str

    @model_validator(mode="after")
    def check_shape(self):
        """윗면 중심 기준과 위·아랫면이 판 안에 들어가는지 검사한다."""
        if self.reference != "tcp_at_top_center":
            raise ValueError("실행 기준은 접촉 TCP의 윗면 중심이어야 합니다.")
        if max(self.top_diameter_mm, self.bottom_diameter_mm) > min(
            self.board_width_mm, self.board_depth_mm
        ):
            raise ValueError("케이크판이 케이크보다 작습니다.")
        return self


def load_cake_settings():
    """웹 3D와 공유하는 JSON을 읽고 주문 생성 시점의 치수 사본을 제공한다."""
    path = Path(__file__).resolve().parents[2] / "config/test_cake.json"
    return CakeSettings.model_validate(json.loads(path.read_text())).model_dump()
