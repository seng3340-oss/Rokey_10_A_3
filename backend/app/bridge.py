"""인증된 ROS 브릿지만 주문을 인수하고 실행 상태를 기록하는 내부 HTTP API다."""

from datetime import datetime, timezone
import hashlib
import json
import secrets
from typing import Annotated, Literal
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException, Request, Response
from pydantic import Field

from .schemas import ContractModel

router = APIRouter(prefix="/internal/bridge", tags=["ROS 브릿지 내부 연결"])
TERMINAL = {"completed", "failed", "rejected", "resolved"}


class BridgeEvent(ContractModel):
    """한 이벤트를 같은 ID로 재시도해도 상태를 두 번 바꾸지 않는다."""

    event_id: Annotated[str, Field(min_length=1, max_length=128)]
    claim_token: Annotated[str, Field(min_length=1, max_length=128)]
    kind: Literal[
        "dispatching",
        "accepted",
        "feedback",
        "completed",
        "failed",
        "rejected",
        "uncertain",
    ]
    # 운전 모드는 브릿지 실행 설정이며 실제 장비 연결이나 안전 확인을 뜻하지 않는다.
    execution_mode: Literal["unknown", "real", "simulation"] = "unknown"
    current_command: Annotated[str, Field(max_length=128)] = ""
    phase: Annotated[str, Field(max_length=128)] = ""
    progress_pct: Annotated[float, Field(ge=0, le=100, allow_inf_nan=False)] = 0.0
    message: Annotated[str, Field(max_length=2048)] = ""
    error_code: Annotated[str, Field(max_length=256)] = ""
    elapsed_time_sec: Annotated[float, Field(ge=0, allow_inf_nan=False)] = 0.0


class ManualResolution(ContractModel):
    """장비·주문을 직접 확인한 뒤 불명확 상태만 해제한다. 실행 중 정지 API가 아니다."""

    confirmed_not_running: Literal[True]
    reason: Annotated[str, Field(min_length=1, max_length=2048)]


def authorize(request: Request):
    """브라우저 공개 API와 분리한다. 키 미설정 시 내부 연결만 비활성화한다."""
    token = request.app.state.bridge_token
    if not token:
        raise HTTPException(503, "CAKE_BRIDGE_TOKEN을 서버에 설정해야 합니다.")
    if not secrets.compare_digest(
        request.headers.get("Authorization", "").encode(), f"Bearer {token}".encode()
    ):
        raise HTTPException(401, "브릿지 인증이 필요합니다.")


def now():
    """갱신 시각은 UTC ISO 문자열로 보존한다."""
    return datetime.now(timezone.utc).isoformat()


@router.post("/claim", dependencies=[Depends(authorize)])
def claim_order(request: Request):
    """received 주문 하나를 원자적으로 인수한다. 인수한 주문은 자동 재할당하지 않는다."""
    with request.app.state.database.connection() as connection:
        connection.execute("BEGIN IMMEDIATE")
        row = connection.execute(
            "SELECT o.*,e.cake_json FROM orders o JOIN order_execution e USING(order_id) "
            "WHERE e.state='received' ORDER BY o.id LIMIT 1"
        ).fetchone()
        if row is None:
            return Response(status_code=204)
        token = str(uuid4())
        connection.execute(
            "UPDATE order_execution SET state='claimed',claim_token=?,updated_at=? WHERE order_id=?",
            (token, now(), row["order_id"]),
        )
        connection.commit()
    return {
        "order_id": row["order_id"],
        "claim_token": token,
        "drawing": json.loads(row["drawing_json"]),
        "cake": json.loads(row["cake_json"]),
    }


@router.post("/orders/{order_id}/events", dependencies=[Depends(authorize)])
def record_event(order_id: str, event: BridgeEvent, request: Request):
    """허용 순서만 저장하고 불명확한 전송은 활성 잠금을 유지한다."""
    digest = hashlib.sha256(event.model_dump_json().encode()).hexdigest()
    with request.app.state.database.connection() as connection:
        connection.execute("BEGIN IMMEDIATE")
        row = connection.execute(
            "SELECT * FROM order_execution WHERE order_id=?", (order_id,)
        ).fetchone()
        if row is None:
            raise HTTPException(404, "주문이 없습니다.")
        if not secrets.compare_digest(
            (row["claim_token"] or "").encode(), event.claim_token.encode()
        ):
            raise HTTPException(409, "주문 인수 토큰이 다릅니다.")
        old = connection.execute(
            "SELECT * FROM bridge_events WHERE event_id=?", (event.event_id,)
        ).fetchone()
        if old:
            if old["order_id"] != order_id or old["payload_hash"] != digest:
                raise HTTPException(409, "같은 이벤트 ID의 내용이 다릅니다.")
            return {"order_id": order_id, "status": row["state"]}
        allowed = {
            "claimed": {"dispatching", "uncertain"},
            "dispatching": {"accepted", "rejected", "uncertain"},
            "running": {"feedback", "completed", "failed", "uncertain"},
        }
        if event.kind not in allowed.get(row["state"], set()):
            raise HTTPException(409, "현재 상태에서 해당 이벤트를 적용할 수 없습니다.")
        state = {"accepted": "running", "feedback": "running"}.get(
            event.kind, event.kind
        )
        connection.execute(
            "UPDATE order_execution SET state=?,phase=?,progress_pct=?,message=?,result_json=?,updated_at=? "
            "WHERE order_id=?",
            (
                state,
                event.phase,
                event.progress_pct,
                event.message,
                event.model_dump_json(),
                now(),
                order_id,
            ),
        )
        connection.execute(
            "INSERT INTO bridge_events VALUES (?,?,?)",
            (event.event_id, order_id, digest),
        )
        connection.commit()
    return {"order_id": order_id, "status": state}


@router.post("/orders/{order_id}/resolve", dependencies=[Depends(authorize)])
def resolve_order(order_id: str, resolution: ManualResolution, request: Request):
    """재시작·이전 DB의 잠금은 작업이 끝났음을 사람이 확인한 경우에만 해제한다."""
    with request.app.state.database.connection() as connection:
        connection.execute("BEGIN IMMEDIATE")
        row = connection.execute(
            "SELECT state FROM order_execution WHERE order_id=?", (order_id,)
        ).fetchone()
        if row is None:
            raise HTTPException(404, "주문이 없습니다.")
        if row["state"] not in {"uncertain", "claimed", "dispatching", "running"}:
            raise HTTPException(409, "수동 확인이 필요한 주문이 아닙니다.")
        connection.execute(
            "UPDATE order_execution SET state='resolved',message=?,updated_at=? WHERE order_id=?",
            (resolution.reason, now(), order_id),
        )
        connection.commit()
    return {"order_id": order_id, "status": "resolved"}
