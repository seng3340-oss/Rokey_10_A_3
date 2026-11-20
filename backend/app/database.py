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
                        'completed','failed','rejected','uncertain','resolved','paused'
                    )),
                    request_key TEXT UNIQUE,
                    payload_hash TEXT NOT NULL,
                    cake_json TEXT NOT NULL,
                    claim_token TEXT,
                    resume_checkpoint TEXT NOT NULL DEFAULT '',
                    phase TEXT NOT NULL DEFAULT '',
                    progress_pct REAL NOT NULL DEFAULT 0,
                    message TEXT NOT NULL DEFAULT '',
                    result_json TEXT,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS hmi_runtime (
                    id INTEGER PRIMARY KEY CHECK(id=1),
                    report_json TEXT NOT NULL,
                    received_at REAL NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS hmi_plans (
                    order_id TEXT PRIMARY KEY REFERENCES orders(order_id),
                    plan_json TEXT NOT NULL,
                    execution_mode TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS bridge_events (
                    event_id TEXT PRIMARY KEY,
                    order_id TEXT NOT NULL,
                    payload_hash TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS workcell_recovery (
                    id INTEGER PRIMARY KEY CHECK(id=1),
                    revision INTEGER NOT NULL DEFAULT 0,
                    requires_reset INTEGER NOT NULL DEFAULT 0 CHECK(requires_reset IN (0,1)),
                    order_id TEXT NOT NULL DEFAULT '',
                    error_code TEXT NOT NULL DEFAULT '',
                    message TEXT NOT NULL DEFAULT '',
                    updated_at TEXT NOT NULL DEFAULT '',
                    reset_at TEXT,
                    checkpoint_json TEXT NOT NULL DEFAULT '',
                    resume_requested INTEGER NOT NULL DEFAULT 0
                );
                INSERT OR IGNORE INTO workcell_recovery(id) VALUES (1);
                CREATE TABLE IF NOT EXISTS workcell_logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    created_at TEXT NOT NULL,
                    order_id TEXT NOT NULL,
                    severity TEXT NOT NULL CHECK(severity IN ('info','warning','error')),
                    message TEXT NOT NULL
                );
                """
            )
            # 기존 SQLite CHECK 제약에 paused를 추가한다. 주문 원본·실행 이력은
            # 같은 트랜잭션에서 복사하고 실패 시 전체 롤백해 기존 테이블을 보존한다.
            connection.execute('BEGIN IMMEDIATE')
            schema = connection.execute(
                "SELECT sql FROM sqlite_master WHERE name='order_execution'"
            ).fetchone()['sql']
            if "'paused'" not in schema:
                upgraded = schema.replace(
                    'order_execution', 'order_execution_upgrade', 1
                ).replace("'resolved'", "'resolved','paused'")
                connection.execute(upgraded)
                columns = [row['name'] for row in connection.execute('PRAGMA table_info(order_execution)')]
                names = ','.join(columns)
                connection.execute(
                    f'INSERT INTO order_execution_upgrade ({names}) SELECT {names} FROM order_execution'
                )
                connection.execute('DROP TABLE order_execution')
                connection.execute('ALTER TABLE order_execution_upgrade RENAME TO order_execution')
            # 새 열은 과거 DB에서도 기본값으로 시작해 이전 실패를 임의로 재개하지 않는다.
            for table, column, declaration in (
                ('order_execution', 'resume_checkpoint', "TEXT NOT NULL DEFAULT ''"),
                ('workcell_recovery', 'checkpoint_json', "TEXT NOT NULL DEFAULT ''"),
                ('workcell_recovery', 'resume_requested', 'INTEGER NOT NULL DEFAULT 0'),
            ):
                known = {row['name'] for row in connection.execute(f'PRAGMA table_info({table})')}
                if column not in known:
                    connection.execute(f'ALTER TABLE {table} ADD COLUMN {column} {declaration}')
            # 재개 예약만 남고 인수 전 서버가 종료됐으면 새 현장 확인을 받는다.
            # claimed/running은 실제 실행 여부가 불명확하므로 이 자동 전환 대상이 아니다.
            queued_resume = connection.execute(
                "SELECT order_id FROM order_execution WHERE state='received' AND resume_checkpoint != ''"
            ).fetchone()
            if queued_resume:
                connection.execute(
                    "UPDATE order_execution SET state='paused',claim_token=NULL "
                    'WHERE order_id=?', (queued_resume['order_id'],),
                )
                connection.execute(
                    'UPDATE workcell_recovery SET resume_requested=0,revision=revision+1 '
                    'WHERE id=1 AND order_id=?', (queued_resume['order_id'],),
                )
            # 기존 주문은 실행 이력을 알 수 없으므로 재전송하지 않고 확인 대기로 이관한다.
            connection.execute(
                """INSERT OR IGNORE INTO order_execution
                (order_id, state, payload_hash, cake_json, updated_at)
                SELECT order_id, 'uncertain', '', '{}', created_at FROM orders"""
            )
            connection.commit()
