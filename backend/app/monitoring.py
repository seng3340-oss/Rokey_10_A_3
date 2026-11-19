"""HMI의 읽기 전용 주문·단계·오류 조회와 인증된 브릿지 관측을 제공한다."""

from datetime import datetime, timezone
import json
from typing import Literal
from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import Field
from .bridge import authorize, now
from .schemas import ContractModel

router = APIRouter(tags=["HMI 모니터링"])


class RuntimeReport(ContractModel):
    """로봇 안전 신호가 아닌 브릿지의 TM 발견 관측이다."""

    last_error_message: str = Field(default="", max_length=2048)
    last_error_source: str = Field(default="", max_length=128)
    last_error_at: str = Field(default="", max_length=64)
    task_server_ready: bool
    active_order_id: str | None = Field(default=None, max_length=128)
    execution_mode: Literal["real", "simulation", "unknown"] = "unknown"
    process_mode: Literal["full", "pen", "drawing", "unknown"] = "unknown"


def runtime_snapshot(connection):
    """오래된 heartbeat를 연결됨으로 표시하지 않는다. 물리 E-stop은 별도 신호다."""
    row = connection.execute(
        "SELECT payload,updated_at FROM hmi_runtime WHERE id=1"
    ).fetchone()
    if not row:
        return {
            "fresh": False,
            "task_server_ready": False,
            "updated_at": None,
            "age_sec": None,
            "execution_mode": "unknown",
            "process_mode": "unknown",
            "active_order_id": None,
            "robot_connected": None,
            "safety_status": "unknown",
        }
    age = max(
        0,
        (
            datetime.now(timezone.utc) - datetime.fromisoformat(row["updated_at"])
        ).total_seconds(),
    )
    payload = json.loads(row["payload"])
    return {
        **payload,
        "fresh": age <= 8,
        "updated_at": row["updated_at"],
        "age_sec": age,
        "task_server_ready": age <= 8 and payload["task_server_ready"],
        "robot_connected": None,
        "safety_status": "unknown",
    }


@router.post("/internal/bridge/heartbeat", dependencies=[Depends(authorize)])
def heartbeat(report: RuntimeReport, request: Request):
    """표시용 관측만 갱신한다. 주문 인수나 동작을 유발하지 않는다."""
    with request.app.state.database.connection() as c:
        c.execute(
            "INSERT INTO hmi_runtime VALUES (1,?,?) ON CONFLICT(id) DO UPDATE SET payload=excluded.payload,updated_at=excluded.updated_at",
            (report.model_dump_json(), now()),
        )
        c.commit()
    return {"status": "ok"}


@router.get("/api/hmi")
def snapshot(request: Request, order_id: str | None = None):
    """한 읽기 스냅샷의 목록·선택 주문·최근 영구 이벤트를 반환한다. 토큰은 제외한다."""
    with request.app.state.database.connection() as c:
        c.execute("BEGIN")
        rows = c.execute(
            "SELECT o.order_id,o.drawing_surface,o.created_at,e.state,e.phase,e.progress_pct,e.message,e.result_json,e.updated_at FROM orders o JOIN order_execution e USING(order_id) ORDER BY o.id DESC LIMIT 100"
        ).fetchall()
        orders = []
        for row in rows:
            item = dict(row)
            result = json.loads(item.pop("result_json") or "{}")
            item["error_code"] = result.get("error_code", "")
            item["elapsed_time_sec"] = result.get("elapsed_time_sec", 0)
            orders.append(item)
        selected_id = order_id or (orders[0]["order_id"] if orders else None)
        selected = next((o for o in orders if o["order_id"] == selected_id), None)
        if order_id and selected is None:
            # 목록의 최근100건 바깥 주문도 ID로 직접 조회할 수 있다.
            row = c.execute(
                "SELECT o.order_id,o.drawing_surface,o.created_at,e.state,e.phase,e.progress_pct,e.message,e.result_json,e.updated_at FROM orders o JOIN order_execution e USING(order_id) WHERE o.order_id=?",
                (order_id,),
            ).fetchone()
            if not row:
                raise HTTPException(404, "주문이 없습니다.")
            selected = dict(row)
            result = json.loads(selected.pop("result_json") or "{}")
            selected.update(
                error_code=result.get("error_code", ""),
                elapsed_time_sec=result.get("elapsed_time_sec", 0),
            )
        events = [
            dict(row)
            for row in c.execute(
                "SELECT id,order_id,kind,phase,stage_key,stage_state,progress_pct,message,error_code,created_at FROM hmi_events WHERE order_id=? ORDER BY id DESC LIMIT 300",
                (selected_id,),
            ).fetchall()
        ]
        # 현재 실행 설정으로 과거 주문의 공정 종류를 덮어쓰지 않는다.
        for item in orders + ([selected] if selected else []):
            mode = c.execute(
                "SELECT process_mode FROM hmi_order_modes WHERE order_id=?",
                (item["order_id"],),
            ).fetchone()
            item["process_mode"] = mode["process_mode"] if mode else "unknown"
        if selected and selected["state"] == "running":
            stage = next((event for event in events if event["stage_key"]), None)
            if stage:
                selected.update(
                    phase=stage["phase"],
                    progress_pct=stage["progress_pct"],
                    message=stage["message"],
                )
        runtime = runtime_snapshot(c)
    return {
        "orders": orders,
        "selected": selected,
        "events": events,
        "runtime": runtime,
        "server_time": now(),
    }
