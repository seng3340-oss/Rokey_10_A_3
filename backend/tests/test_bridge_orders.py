"""단일 주문·중복 접수·브릿지 인수의 실제 HTTP/SQLite 동작을 검증한다."""

from concurrent.futures import ThreadPoolExecutor

from fastapi.testclient import TestClient

from app.main import create_app

DRAWING = {
    "canvas_width": 1024,
    "canvas_height": 768,
    "drawing_surface": "top",
    "strokes": [
        {"stroke_id": 0, "points": [{"x": 500.5, "y": 350.0}, {"x": 520.0, "y": 360.0}]}
    ],
}


def test_only_one_active_order_even_with_concurrent_requests(tmp_path):
    """단일 활성 주문 제한이 프로세스 내부 플래그가 아니라 DB 트랜잭션으로 작동한다."""
    with TestClient(create_app(tmp_path / "orders.sqlite3")) as client:
        with ThreadPoolExecutor(max_workers=6) as pool:
            responses = list(
                pool.map(lambda _: client.post("/api/drawing", json=DRAWING), range(12))
            )
        assert sorted(r.status_code for r in responses) == [201] + [409] * 11
        assert client.get("/api/orders").json()["total"] == 1


def test_retried_request_keeps_same_order_and_rejects_changed_body(tmp_path):
    """응답 유실 후 같은 키를 재사용하면 주문이 늘지 않는다."""
    with TestClient(create_app(tmp_path / "orders.sqlite3")) as client:
        headers = {"Idempotency-Key": "test-request-1"}
        first = client.post("/api/drawing", json=DRAWING, headers=headers)
        repeated = client.post("/api/drawing", json=DRAWING, headers=headers)
        assert first.status_code == repeated.status_code == 201
        assert first.json() == repeated.json()
        changed = {**DRAWING, "drawing_surface": "side"}
        assert (
            client.post("/api/drawing", json=changed, headers=headers).status_code
            == 409
        )


def test_claim_and_results_are_authorized_persistent_and_idempotent(tmp_path):
    """인수는 한 번만 되고 확정 결과 후에만 다음 주문을 받는다."""
    path = tmp_path / "orders.sqlite3"
    with TestClient(create_app(path, bridge_token="test-secret")) as client:
        order_id = client.post("/api/drawing", json=DRAWING).json()["order_id"]
        assert client.post("/internal/bridge/claim").status_code == 401
        headers = {"Authorization": "Bearer test-secret"}
        claim = client.post("/internal/bridge/claim", headers=headers).json()
        assert claim["order_id"] == order_id
        assert claim["drawing"] == DRAWING
        assert claim["cake"]["diameter_mm"] == 100.0
        assert client.post("/internal/bridge/claim", headers=headers).status_code == 204
        for kind in ("dispatching", "accepted", "completed"):
            event = {
                "event_id": f"event-{kind}",
                "claim_token": claim["claim_token"],
                "kind": kind,
            }
            response = client.post(
                f"/internal/bridge/orders/{order_id}/events",
                headers=headers,
                json=event,
            )
            assert response.status_code == 200
            assert (
                client.post(
                    f"/internal/bridge/orders/{order_id}/events",
                    headers=headers,
                    json=event,
                ).status_code
                == 200
            )
        assert client.post("/api/drawing", json=DRAWING).status_code == 201
    with TestClient(create_app(path, bridge_token="test-secret")) as client:
        assert client.post("/api/drawing", json=DRAWING).status_code == 409
        assert client.get(f"/api/orders/{order_id}").json()["status"] == "completed"


def test_unknown_dispatch_blocks_new_orders_and_requires_manual_confirmation(tmp_path):
    """전송 결과를 모르면 자동 재할당·완료하지 않는다."""
    path = tmp_path / "orders.sqlite3"
    headers = {"Authorization": "Bearer test-secret"}
    with TestClient(create_app(path, bridge_token="test-secret")) as client:
        order_id = client.post("/api/drawing", json=DRAWING).json()["order_id"]
        claim = client.post("/internal/bridge/claim", headers=headers).json()

        def send(kind, token=None):
            return client.post(
                f"/internal/bridge/orders/{order_id}/events",
                headers=headers,
                json={
                    "event_id": f"unknown-{kind}",
                    "claim_token": token or claim["claim_token"],
                    "kind": kind,
                },
            )

        assert send("completed").status_code == 409
        assert send("dispatching", "wrong-token").status_code == 409
        assert send("dispatching").status_code == 200
        assert send("uncertain").status_code == 200
        assert client.get(f"/api/orders/{order_id}").json()["robot_dispatched"] is None
    with TestClient(create_app(path, bridge_token="test-secret")) as client:
        assert client.post("/api/drawing", json=DRAWING).status_code == 409
        assert client.post("/internal/bridge/claim", headers=headers).status_code == 204
        resolve = f"/internal/bridge/orders/{order_id}/resolve"
        assert (
            client.post(
                resolve,
                headers=headers,
                json={"confirmed_not_running": False, "reason": "확인 안 함"},
            ).status_code
            == 422
        )
        assert (
            client.post(
                resolve,
                headers=headers,
                json={
                    "confirmed_not_running": True,
                    "reason": "장비 정지와 주문 미실행을 현장에서 확인",
                },
            ).status_code
            == 200
        )
        assert client.post("/api/drawing", json=DRAWING).status_code == 201


def test_legacy_orders_are_preserved_but_never_automatically_dispatched(tmp_path):
    """이전 버전 DB를 열어도 원본을 지우거나 과거 그림을 로봇에 보내지 않는다."""
    import json
    import sqlite3

    path = tmp_path / "legacy.sqlite3"
    with sqlite3.connect(path) as db:
        db.execute(
            "CREATE TABLE orders (id INTEGER PRIMARY KEY AUTOINCREMENT, order_id TEXT UNIQUE, drawing_surface TEXT, drawing_json TEXT, created_at TEXT, status TEXT DEFAULT 'received' CHECK(status='received'))"
        )
        db.execute(
            "INSERT INTO orders(order_id,drawing_surface,drawing_json,created_at) VALUES (?,?,?,?)",
            ("OLD-1", "top", json.dumps(DRAWING), "2026-11-15"),
        )
    with TestClient(create_app(path, bridge_token="test-secret")) as client:
        detail = client.get("/api/orders/OLD-1").json()
        assert detail["drawing"] == DRAWING
        assert detail["status"] == "uncertain"
        assert (
            client.post(
                "/internal/bridge/claim",
                headers={"Authorization": "Bearer test-secret"},
            ).status_code
            == 204
        )
        assert client.post("/api/drawing", json=DRAWING).status_code == 409


def test_idempotency_key_survives_restart_and_finished_orders(tmp_path):
    """완료 후 뒤늦은 원래 요청 재시도가 새 작업을 만들지 않는다."""
    path = tmp_path / "orders.sqlite3"
    headers = {"Authorization": "Bearer test-secret"}
    request_headers = {"Idempotency-Key": "stable-key"}
    with TestClient(create_app(path, bridge_token="test-secret")) as client:
        order = client.post(
            "/api/drawing", json=DRAWING, headers=request_headers
        ).json()
        claim = client.post("/internal/bridge/claim", headers=headers).json()
        for kind in ("dispatching", "rejected"):
            assert (
                client.post(
                    f"/internal/bridge/orders/{order['order_id']}/events",
                    headers=headers,
                    json={
                        "event_id": kind,
                        "claim_token": claim["claim_token"],
                        "kind": kind,
                    },
                ).status_code
                == 200
            )
    with TestClient(create_app(path, bridge_token="test-secret")) as client:
        repeated = client.post("/api/drawing", json=DRAWING, headers=request_headers)
        assert repeated.json() == order
        assert client.get("/api/orders").json()["total"] == 1


def test_bridge_is_disabled_without_token_and_hmi_stays_static(tmp_path, monkeypatch):
    """설정되지 않은 내부 연결과 HMI 운영 API를 임의로 열지 않는다."""
    monkeypatch.delenv("CAKE_BRIDGE_TOKEN", raising=False)
    with TestClient(create_app(tmp_path / "orders.sqlite3")) as client:
        assert client.post("/internal/bridge/claim").status_code == 503
        assert client.get("/api/hmi/state").status_code == 404
