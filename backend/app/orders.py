"""주문 생성·보존과 읽기 전용 목록/상세 API를 제공한다."""

from datetime import datetime
import json
import hashlib
import sqlite3
from typing import Annotated
from zoneinfo import ZoneInfo

from fastapi import APIRouter, HTTPException, Query, Request

from .database import Database
from .cake import load_cake_settings
from .schemas import DrawingRequest, OrderDetail, OrderPage, OrderSummary

router = APIRouter(prefix="/api/orders", tags=["주문 조회"])


def save_order(
    database: Database, drawing: DrawingRequest, request_key: str | None = None
) -> str:
    """날짜별 주문 번호와 그림 저장을 하나의 트랜잭션으로 묶는다."""
    payload_hash = hashlib.sha256(drawing.model_dump_json().encode()).hexdigest()
    now = datetime.now(ZoneInfo("Asia/Seoul"))
    order_date = now.strftime("%Y%m%d")

    with database.connection() as connection:
        # 동시 요청이 같은 번호를 읽지 않도록 번호 조회보다 먼저 쓰기 잠금을 얻는다.
        connection.execute("BEGIN IMMEDIATE")
        # 동일 요청은 활성 슬롯 검사보다 먼저 조회해 응답 유실 후에도 같은 번호를 돌려준다.
        if request_key:
            previous = connection.execute(
                "SELECT order_id, payload_hash FROM order_execution WHERE request_key=?",
                (request_key,),
            ).fetchone()
            if previous:
                if previous["payload_hash"] != payload_hash:
                    raise HTTPException(
                        409, "같은 요청 키로 다른 그림을 제출할 수 없습니다."
                    )
                return previous["order_id"]
        active = connection.execute(
            "SELECT order_id FROM order_execution WHERE state NOT IN "
            "('completed','failed','rejected','resolved') LIMIT 1"
        ).fetchone()
        if active:
            raise HTTPException(
                409,
                f"현재 주문 {active['order_id']} 처리 중입니다. 새 주문을 받을 수 없습니다.",
            )
        connection.execute(
            """
            INSERT INTO daily_sequences(order_date, last_number) VALUES (?, 1)
            ON CONFLICT(order_date) DO UPDATE SET last_number = last_number + 1
            """,
            (order_date,),
        )
        row = connection.execute(
            "SELECT last_number FROM daily_sequences WHERE order_date = ?",
            (order_date,),
        ).fetchone()
        order_id = f"ORD-{order_date}-{row['last_number']:03d}"

        # 좌표 단순화나 로봇 좌표 변환 없이 검증한 입력을 그대로 저장한다.
        connection.execute(
            """
            INSERT INTO orders(order_id, drawing_surface, drawing_json, created_at)
            VALUES (?, ?, ?, ?)
            """,
            (
                order_id,
                drawing.drawing_surface,
                drawing.model_dump_json(),
                now.isoformat(),
            ),
        )
        # 저장과 활성 슬롯 확보를 같은 트랜잭션으로 묶어 동시 접수를 하나로 제한한다.
        connection.execute(
            "INSERT INTO order_execution "
            "(order_id,state,request_key,payload_hash,cake_json,updated_at) VALUES (?,?,?,?,?,?)",
            (
                order_id,
                "received",
                request_key,
                payload_hash,
                json.dumps(load_cake_settings()),
                now.isoformat(),
            ),
        )
        connection.commit()

    return order_id


def summary_from_row(row: sqlite3.Row) -> OrderSummary:
    """내부 DB 번호나 전체 그림을 목록 응답에 노출하지 않는다."""
    return OrderSummary(
        order_id=row["order_id"],
        drawing_surface=row["drawing_surface"],
        created_at=row["created_at"],
        status=row["execution_state"],
        phase=row["phase"],
        progress_pct=row["progress_pct"],
        message=row["message"],
        error_code=json.loads(row["result_json"] or "{}").get("error_code", ""),
        robot_dispatched=(
            None
            if row["execution_state"] in ("dispatching", "uncertain")
            else row["execution_state"] in ("running", "completed", "failed")
        ),
    )


@router.get("", response_model=OrderPage)
def list_orders(
    request: Request,
    limit: Annotated[int, Query(ge=1, le=100)] = 20,
    offset: Annotated[int, Query(ge=0)] = 0,
):
    """최근 주문부터 조회한다. 총 개수와 목록은 같은 읽기 스냅샷을 사용한다."""
    with request.app.state.database.connection() as connection:
        connection.execute("BEGIN")
        total = connection.execute("SELECT COUNT(*) FROM orders").fetchone()[0]
        rows = connection.execute(
            "SELECT o.order_id,o.drawing_surface,o.created_at,e.state AS execution_state,e.phase,e.progress_pct,e.message,e.result_json FROM orders o "
            "JOIN order_execution e USING(order_id) ORDER BY o.id DESC LIMIT ? OFFSET ?",
            (limit, offset),
        ).fetchall()

    return OrderPage(
        items=[summary_from_row(row) for row in rows],
        total=total,
        limit=limit,
        offset=offset,
    )


@router.get("/{order_id}", response_model=OrderDetail)
def read_order(order_id: str, request: Request):
    """저장된 주문과 원본 그림을 조회하며 존재하지 않는 주문은 404로 구분한다."""
    with request.app.state.database.connection() as connection:
        row = connection.execute(
            "SELECT o.*, e.state AS execution_state,e.phase,e.progress_pct,e.message,e.result_json FROM orders o "
            "JOIN order_execution e USING(order_id) WHERE o.order_id = ?",
            (order_id,),
        ).fetchone()

    if row is None:
        raise HTTPException(status_code=404, detail="해당 주문을 찾을 수 없습니다.")

    return OrderDetail(
        **summary_from_row(row).model_dump(),
        drawing=json.loads(row["drawing_json"]),
    )


@router.get("/{order_id}/monitor")
def read_order_monitor(order_id: str, request: Request):
    """저장된 마지막 이벤트만 읽는다. 안전 신호·실행 모드·실측 궤적은 추정하지 않는다.

    기존 주문 응답을 유지하는 선택적 확장이다. 내부 토큰과 원본 이벤트 본문은
    반환하지 않으며 DB에 주문 생성·인수·변경을 일으키지 않는다.
    """
    with request.app.state.database.connection() as connection:
        row = connection.execute(
            "SELECT updated_at,result_json FROM order_execution WHERE order_id=?",
            (order_id,),
        ).fetchone()
    if row is None:
        raise HTTPException(404, "해당 주문을 찾을 수 없습니다.")
    event = json.loads(row["result_json"] or "{}")
    return {
        "updated_at": row["updated_at"],
        "error_code": event.get("error_code", ""),
        "elapsed_time_sec": event.get("elapsed_time_sec", 0.0),
        "execution_mode": event.get("execution_mode", "unknown"),
        "current_command": event.get("current_command", ""),
        "safety_status": "unknown",
        "trajectory_available": False,
    }
