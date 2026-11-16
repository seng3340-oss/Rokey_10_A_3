"""작은 MVP를 위한 SQLite 연결과 초기 테이블을 관리한다."""

from contextlib import contextmanager
from pathlib import Path
import sqlite3


class Database:
    """연결을 요청 간 공유하지 않고 필요한 동안에만 열어 스레드 충돌을 피한다."""

    def __init__(self, path: Path):
        self.path = Path(path)

    @contextmanager
    def connection(self):
        """실패한 쓰기는 rollback하고 연결은 성공·실패에 관계없이 닫는다."""
        connection = sqlite3.connect(self.path, timeout=10)
        connection.row_factory = sqlite3.Row
        try:
            yield connection
        except BaseException:
            connection.rollback()
            raise
        finally:
            connection.close()

    def initialize(self):
        """앱 시작 시에만 테이블을 준비하며 재시작 시 기존 주문은 보존한다."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.connection() as connection:
            # WAL은 한쪽의 조회가 짧은 주문 저장을 불필요하게 막는 상황을 줄인다.
            connection.execute("PRAGMA journal_mode=WAL")
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS daily_sequences (
                    order_date TEXT PRIMARY KEY,
                    last_number INTEGER NOT NULL
                );
                CREATE TABLE IF NOT EXISTS orders (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    order_id TEXT NOT NULL UNIQUE,
                    drawing_surface TEXT NOT NULL CHECK(drawing_surface IN ('top', 'side')),
                    drawing_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    status TEXT NOT NULL DEFAULT 'received' CHECK(status = 'received')
                );
                """
            )
            # 원본 orders 테이블은 그대로 보존하고 실행·중복 처리만 별도 기록한다.
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS order_execution (
                    order_id TEXT PRIMARY KEY REFERENCES orders(order_id),
                    state TEXT NOT NULL CHECK(state IN (
                        'received','claimed','dispatching','running',
                        'completed','failed','rejected','uncertain','resolved'
                    )),
                    request_key TEXT UNIQUE,
                    payload_hash TEXT NOT NULL,
                    cake_json TEXT NOT NULL,
                    claim_token TEXT,
                    phase TEXT NOT NULL DEFAULT '',
                    progress_pct REAL NOT NULL DEFAULT 0,
                    message TEXT NOT NULL DEFAULT '',
                    result_json TEXT,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS bridge_events (
                    event_id TEXT PRIMARY KEY,
                    order_id TEXT NOT NULL,
                    payload_hash TEXT NOT NULL
                );
                """
            )
            # 기존 주문은 실행 이력을 알 수 없으므로 재전송하지 않고 확인 대기로 이관한다.
            connection.execute(
                """INSERT OR IGNORE INTO order_execution
                (order_id, state, payload_hash, cake_json, updated_at)
                SELECT order_id, 'uncertain', '', '{}', created_at FROM orders"""
            )
            connection.commit()
