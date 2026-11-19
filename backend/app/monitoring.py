"""TM 연결 메타데이터와 DP 계획의 표시 전용 계약. 로봇 제어 API는 제공하지 않는다."""

from datetime import datetime, timezone
import json
import secrets
import time
from typing import Annotated, Literal

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import Field
from .bridge import authorize, now
from .schemas import ContractModel, DrawingRequest

router = APIRouter(tags=["HMI 모니터링"])
ExecutionMode = Literal["unknown", "real", "simulation"]
FRESH_SECONDS = 8.0


class RuntimeReport(ContractModel):
    """인증된 브릿지가 관측한 TM 상태다. 로봇 연결·E-stop 신호는 포함하지 않는다."""

    execution_mode: ExecutionMode = "unknown"
    task_server_ready: bool
    active_order_id: Annotated[str, Field(max_length=128)] | None = None
    task_feedback_age_sec: Annotated[float, Field(ge=0, allow_inf_nan=False)] | None = (
        None
    )


class PlanData(DrawingRequest):
    """실제 GenerateTrajectory 결과를 원본 캔버스로 투영한 계획. 실측 경로가 아니다."""

    version: Literal[1]
    source: Literal["task_manager_node/GenerateTrajectory"]
    order_id: Annotated[str, Field(min_length=1, max_length=128)]
    waypoint_count: Annotated[int, Field(strict=True, ge=2, le=30000)]


class PlanReport(ContractModel):
    """인수 토큰을 가진 브릿지만 해당 주문의 계획을 저장할 수 있다."""

    claim_token: Annotated[str, Field(min_length=1, max_length=128)]
    execution_mode: ExecutionMode = "unknown"
    plan: PlanData


def runtime_snapshot(database):
    """백엔드 수신 시각으로 노후화를 계산해 PC 간 시계 차이를 피한다."""
    with database.connection() as connection:
        row = connection.execute("SELECT * FROM hmi_runtime WHERE id=1").fetchone()
    if row is None:
        return {
            "observed": False,
            "bridge_fresh": False,
            "task_server_ready": False,
            "task_feedback_fresh": False,
            "execution_mode": "unknown",
            "active_order_id": None,
            "feedback_age_sec": None,
            "updated_at": None,
            "robot_connected": None,
            "safety_status": "unknown",
        }
    report = json.loads(row["report_json"])
    age = max(0.0, time.time() - row["received_at"])
    fresh = age <= FRESH_SECONDS
    feedback_age = report["task_feedback_age_sec"]
    if feedback_age is not None:
        feedback_age += age
    return {
        "observed": True,
        "bridge_fresh": fresh,
        "task_server_ready": fresh and report["task_server_ready"],
        "task_feedback_fresh": fresh
        and feedback_age is not None
        and feedback_age <= FRESH_SECONDS,
        "execution_mode": report["execution_mode"],
        "active_order_id": report["active_order_id"],
        "feedback_age_sec": feedback_age,
        "updated_at": row["updated_at"],
        "robot_connected": None,
        "safety_status": "unknown",
    }


@router.post("/internal/bridge/heartbeat", dependencies=[Depends(authorize)])
def heartbeat(report: RuntimeReport, request: Request):
    """표시용 연결 기록만 갱신한다. 주문 인수·Goal 전송을 일으키지 않는다."""
    with request.app.state.database.connection() as connection:
        connection.execute(
            "INSERT INTO hmi_runtime VALUES (1,?,?,?) "
            "ON CONFLICT(id) DO UPDATE SET report_json=excluded.report_json, "
            "received_at=excluded.received_at, updated_at=excluded.updated_at",
            (report.model_dump_json(), time.time(), now()),
        )
        connection.commit()
    return {"status": "recorded"}


@router.get("/api/monitor")
def read_runtime(request: Request):
    """브라우저는 연결 관측값만 읽는다. 명령·정지 요청은 제공하지 않는다."""
    return runtime_snapshot(request.app.state.database)


@router.post(
    "/internal/bridge/orders/{order_id}/plan", dependencies=[Depends(authorize)]
)
def record_plan(order_id: str, report: PlanReport, request: Request):
    """주문·면·해상도·인수 토큰을 대조해 다른 주문의 계획 혼입을 막는다."""
    with request.app.state.database.connection() as connection:
        connection.execute("BEGIN IMMEDIATE")
        row = connection.execute(
            "SELECT o.drawing_json,e.claim_token,e.state FROM orders o "
            "JOIN order_execution e USING(order_id) WHERE order_id=?",
            (order_id,),
        ).fetchone()
        if row is None:
            raise HTTPException(404, "주문이 없습니다.")
        if not secrets.compare_digest(
            (row["claim_token"] or "").encode(), report.claim_token.encode()
        ):
            raise HTTPException(409, "주문 인수 토큰이 다릅니다.")
        if row["state"] != "running":
            raise HTTPException(409, "실행 중인 주문의 계획만 기록합니다.")
        original = json.loads(row["drawing_json"])
        plan = report.plan
        if (
            plan.order_id != order_id
            or plan.drawing_surface != original["drawing_surface"]
            or plan.canvas_width != original["canvas_width"]
            or plan.canvas_height != original["canvas_height"]
        ):
            raise HTTPException(409, "계획의 주문·면·해상도가 원본과 다릅니다.")
        connection.execute(
            "INSERT INTO hmi_plans VALUES (?,?,?,?) "
            "ON CONFLICT(order_id) DO UPDATE SET plan_json=excluded.plan_json, "
            "execution_mode=excluded.execution_mode, updated_at=excluded.updated_at",
            (order_id, plan.model_dump_json(), report.execution_mode, now()),
        )
        connection.commit()
    return {"order_id": order_id, "status": "recorded"}


@router.get("/api/orders/{order_id}/plan")
def read_plan(order_id: str, request: Request):
    """TM이 보낸 계획만 제공한다. 원본 그림이나 진행률로 실행 궤적을 만들지 않는다."""
    with request.app.state.database.connection() as connection:
        row = connection.execute(
            "SELECT * FROM hmi_plans WHERE order_id=?", (order_id,)
        ).fetchone()
    if row is None:
        raise HTTPException(404, "TM 계획 경로 미수신")
    return {
        **json.loads(row["plan_json"]),
        "execution_mode": row["execution_mode"],
        "updated_at": row["updated_at"],
        "measurement_available": False,
    }
