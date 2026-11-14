"""FastAPI 생성·요청 크기 제한·공통 오류 처리만 담당한다."""

from contextlib import asynccontextmanager
import logging
import os
from pathlib import Path
import sqlite3

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.types import ASGIApp, Receive, Scope, Send

from .database import Database
from .drawings import router as drawing_router
from .orders import router as order_router


logger = logging.getLogger(__name__)
DEFAULT_DATABASE = Path(__file__).resolve().parents[1] / "data" / "orders.sqlite3"


class BodyLimitMiddleware:
    """JSON 파싱 전에 실제 수신 크기를 검사하여 대형 좌표 요청을 제한한다."""

    def __init__(self, app: ASGIApp, max_bytes: int):
        self.app = app
        self.max_bytes = max_bytes

    async def __call__(self, scope: Scope, receive: Receive, send: Send):
        # lifespan 등 HTTP가 아닌 ASGI 이벤트는 그대로 통과시킨다.
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        body = bytearray()
        while True:
            message = await receive()
            if message["type"] == "http.disconnect":
                return

            chunk = message.get("body", b"")
            # 초과한 청크는 누적 버퍼에 복사하기 전에 거부한다.
            if len(body) + len(chunk) > self.max_bytes:
                response = JSONResponse(
                    status_code=413,
                    content={"detail": "요청 본문이 허용 크기를 초과했습니다."},
                )
                await response(scope, receive, send)
                return

            body.extend(chunk)
            if not message.get("more_body", False):
                break

        # 한 번 읽은 본문을 라우터에 다시 전달한다. 이후 disconnect는 원래 수신기를 사용한다.
        delivered = False

        async def replay():
            nonlocal delivered
            if not delivered:
                delivered = True
                return {"type": "http.request", "body": bytes(body), "more_body": False}
            return await receive()

        await self.app(scope, replay, send)


def create_app(
    database_path: Path | None = None,
    max_body_bytes: int = 2 * 1024 * 1024,
) -> FastAPI:
    """테스트마다 별도 DB를 주입할 수 있도록 앱 생성과 DB 준비를 분리한다."""
    path = database_path or Path(os.environ.get("CAKE_DATABASE_PATH", DEFAULT_DATABASE))
    database = Database(path)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        # import만으로 파일을 만들거나 장비 연결을 시작하지 않는다.
        database.initialize()
        yield

    app = FastAPI(
        title="케이크 드로잉 주문 API",
        version="0.1.0",
        description="그림 접수·저장·조회 전용. HMI/ROS 브리지와 로봇 실행은 미연결 상태입니다.",
        lifespan=lifespan,
    )
    app.state.database = database
    app.add_middleware(BodyLimitMiddleware, max_bytes=max_body_bytes)
    app.include_router(drawing_router)
    app.include_router(order_router)

    @app.exception_handler(RequestValidationError)
    async def validation_error(request: Request, error: RequestValidationError):
        """NaN 같은 입력 자체를 오류 JSON에 재삽입하지 않고 위치·사유만 반환한다."""
        return JSONResponse(
            status_code=422,
            content={
                "detail": [
                    {"loc": list(item["loc"]), "msg": item["msg"], "type": item["type"]}
                    for item in error.errors()
                ]
            },
        )

    @app.exception_handler(sqlite3.Error)
    async def database_error(request: Request, error: sqlite3.Error):
        """DB 저장 실패를 접수 성공으로 응답하지 않으며 내부 오류는 서버 로그에 남긴다."""
        logger.exception("주문 DB 처리 실패", exc_info=error)
        return JSONResponse(
            status_code=503,
            content={"detail": "주문 저장소를 사용할 수 없습니다. 운영자에게 확인해주세요."},
        )

    @app.get("/api/health", tags=["서버 확인"])
    def health():
        """웹 서버 상태만 반환한다. ROS나 로봇이 연결된 것으로 오해하지 않게 한다."""
        return {"status": "ok", "ros_connected": False}

    return app


# Uvicorn은 이 앱을 로드하고 lifespan 진입 시에 DB를 준비한다.
app = create_app()
