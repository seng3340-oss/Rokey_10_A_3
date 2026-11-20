"""HMI 소프트웨어 정지 요청과 브릿지 롱폴링·적용 보고 계약을 검증한다(해제 API 없음)."""

from concurrent.futures import ThreadPoolExecutor
import time

from fastapi.testclient import TestClient

from app.main import create_app

HEADERS = {"Authorization": "Bearer test-secret"}
DRAWING = {
    "canvas_width": 1024,
    "canvas_height": 768,
    "drawing_surface": "top",
    "strokes": [
        {"stroke_id": 0, "points": [{"x": 500.5, "y": 350.0}, {"x": 520.0, "y": 360.0}]}
    ],
}


def test_stop_increments_sequence_and_requires_bridge_report(tmp_path):
    """정지마다 순번이 오르고, 브릿지가 보고하기 전에는 applied가 False다."""
    with TestClient(create_app(tmp_path / "o.sqlite3", bridge_token="test-secret")) as client:
        initial = client.get("/api/robot/stop").json()
        assert initial["seq"] == 0 and initial["stopped"] is False
        assert initial["bridge_listening"] is False
        # 정지 해제 API는 제공하지 않는다.
        assert client.post("/api/robot/stop/release").status_code in (404, 405)

        stopped = client.post("/api/robot/stop").json()
        assert stopped["seq"] == 1 and stopped["stopped"] is True
        assert stopped["applied"] is False

        # 지난 순번 보고는 반영하지 않는다.
        stale = {"seq": 0, "applied": True, "message": "old"}
        assert client.post(
            "/internal/bridge/stop/applied", json=stale, headers=HEADERS
        ).json() == {"current": False}
        report = {"seq": 1, "applied": True, "message": "정지 적용"}
        assert client.post(
            "/internal/bridge/stop/applied", json=report, headers=HEADERS
        ).json() == {"current": True}
        assert client.get("/api/robot/stop").json()["applied"] is True

        # 정지 중 다시 누르면 순번이 올라 브릿지가 정지를 한 번 더 보내고,
        # 새 순번은 다시 브릿지 보고가 필요하다. 정지 상태는 유지된다.
        again = client.post("/api/robot/stop").json()
        assert again["seq"] == 2 and again["stopped"] is True
        assert again["applied"] is False


def test_bridge_endpoints_require_token(tmp_path):
    """정지 롱폴링·보고는 인증된 브릿지만 사용할 수 있다."""
    with TestClient(create_app(tmp_path / "o.sqlite3", bridge_token="test-secret")) as client:
        body = {"after": -1, "timeout_sec": 1}
        assert client.post("/internal/bridge/stop/wait", json=body).status_code == 401
        report = {"seq": 0, "applied": True}
        assert client.post("/internal/bridge/stop/applied", json=report).status_code == 401


def test_long_poll_returns_immediately_on_stop(tmp_path):
    """브릿지가 대기 중일 때 정지 요청이 오면 타임아웃을 기다리지 않고 응답한다."""
    with TestClient(create_app(tmp_path / "o.sqlite3", bridge_token="test-secret")) as client:
        first = client.post(
            "/internal/bridge/stop/wait",
            json={"after": -1, "timeout_sec": 1},
            headers=HEADERS,
        ).json()
        assert first["seq"] == 0
        with ThreadPoolExecutor(max_workers=1) as pool:
            started = time.monotonic()
            waiting = pool.submit(
                client.post,
                "/internal/bridge/stop/wait",
                json={"after": 0, "timeout_sec": 10},
                headers=HEADERS,
            )
            time.sleep(0.3)
            client.post("/api/robot/stop")
            state = waiting.result(timeout=5).json()
            assert time.monotonic() - started < 5
        assert state["seq"] == 1 and state["stopped"] is True
        assert client.get("/api/robot/stop").json()["bridge_listening"] is True


def test_claim_is_blocked_after_stop(tmp_path):
    """정지 후에는 접수된 주문이 있어도 브릿지에 내주지 않는다."""
    with TestClient(create_app(tmp_path / "o.sqlite3", bridge_token="test-secret")) as client:
        client.post("/api/drawing", json=DRAWING)
        client.post("/api/robot/stop")
        assert client.post("/internal/bridge/claim", headers=HEADERS).status_code == 204
