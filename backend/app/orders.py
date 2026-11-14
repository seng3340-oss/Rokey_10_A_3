"""주문 생성·보존과 읽기 전용 목록/상세 API를 제공한다."""

from datetime import datetime
import json
import sqlite3
from typing import Annotated
from zoneinfo import ZoneInfo

from fastapi import APIRouter, HTTPException, Query, Request

from .database import Database
from .schemas import DrawingRequest, OrderDetail, OrderPage, OrderSummary


router = APIRouter(prefix="/api/orders", tags=["주문 조회"])


def save_order(database: Database, drawing: DrawingRequest) -> str:
    """날짜별 주문 번호와 그림 저장을 하나의 트랜잭션으로 묶는다."""
    now = datetime.now(ZoneInfo("Asia/Seoul"))
    order_date = now.strftime("%Y%m%d")

    with database.connection() as connection:
        # 동시 요청이 같은 번호를 읽지 않도록 번호 조회보다 먼저 쓰기 잠금을 얻는다.
        connection.execute("BEGIN IMMEDIATE")
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
            (order_id, drawing.drawing_surface, drawing.model_dump_json(), now.isoformat()),
        )
        connection.commit()

    return order_id


def summary_from_row(row: sqlite3.Row) -> OrderSummary:
    """내부 DB 번호나 전체 그림을 목록 응답에 노출하지 않는다."""
    return OrderSummary(
        order_id=row["order_id"],
        drawing_surface=row["drawing_surface"],
        created_at=row["created_at"],
        status=row["status"],
        robot_dispatched=False,
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
            "SELECT order_id, drawing_surface, created_at, status "
            "FROM orders ORDER BY id DESC LIMIT ? OFFSET ?",
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
            "SELECT * FROM orders WHERE order_id = ?",
            (order_id,),
        ).fetchone()

    if row is None:
        raise HTTPException(status_code=404, detail="해당 주문을 찾을 수 없습니다.")

    return OrderDetail(
        **summary_from_row(row).model_dump(),
        drawing=json.loads(row["drawing_json"]),
    )
