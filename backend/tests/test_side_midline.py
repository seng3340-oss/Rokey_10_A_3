"""일반 주문과 분리한 중간 높이 옆면 테스트 주문을 검증한다."""

import json
import math
import sqlite3
import pytest
from fastapi.testclient import TestClient
from app.main import create_app
from app.cake import load_cake_settings


def test_test_order_uses_midsection_only_and_keeps_normal_config(tmp_path):
    path = tmp_path / "orders.sqlite3"
    original = load_cake_settings()
    with TestClient(create_app(path, bridge_token="test")) as client:
        response = client.post(
            "/api/tests/side-midline",
            json={"line_length_mm": 30.0},
            headers={"Idempotency-Key": "side-test"},
        )
        assert response.status_code == 201, response.text
        claim = client.post(
            "/internal/bridge/claim", headers={"Authorization": "Bearer test"}
        ).json()
        assert claim["cake"]["diameter_mm"] == 116.75
        assert claim["cake"]["height_mm"] == 79.0
        assert claim["cake"]["test_recipe"]["top_diameter_mm"] == 113.5
        assert claim["cake"]["test_recipe"]["bottom_diameter_mm"] == 120.0
        assert claim["cake"]["test_recipe"]["kind"] == "side_midline_frustum"
        drawing = claim["drawing"]
        assert drawing["drawing_surface"] == "side"
        points = drawing["strokes"][0]["points"]
        assert len(points) == 2 and all(p["y"] == 384.0 for p in points)
        assert math.isclose(
            (points[1]["x"] - points[0]["x"]) / 896 * math.pi * 116.75, 30.0
        )
        assert load_cake_settings() == original
    with sqlite3.connect(path) as db:
        assert db.execute("SELECT COUNT(*) FROM orders").fetchone()[0] == 1


def test_retries_are_idempotent_and_changed_length_is_rejected(tmp_path):
    with TestClient(create_app(tmp_path / "orders.sqlite3")) as client:
        headers = {"Idempotency-Key": "side-test"}
        first = client.post(
            "/api/tests/side-midline", json={"line_length_mm": 30.0}, headers=headers
        )
        repeat = client.post(
            "/api/tests/side-midline", json={"line_length_mm": 30.0}, headers=headers
        )
        assert first.status_code == repeat.status_code == 201
        assert first.json() == repeat.json()
        assert (
            client.post(
                "/api/tests/side-midline",
                json={"line_length_mm": 40.0},
                headers=headers,
            ).status_code
            == 409
        )
        assert (
            client.post(
                "/api/tests/side-midline",
                json={"line_length_mm": 30.0},
                headers={"Idempotency-Key": "other"},
            ).status_code
            == 409
        )


@pytest.mark.parametrize(
    "body",
    [
        {"line_length_mm": 0.0},
        {"line_length_mm": 51.0},
        {"line_length_mm": -1.0},
        {"line_length_mm": True},
        {"line_length_mm": "30"},
        {"line_length_mm": 30.0, "strokes": []},
    ],
)
def test_invalid_recipe_never_creates_an_order(tmp_path, body):
    with TestClient(create_app(tmp_path / "orders.sqlite3")) as client:
        assert client.post("/api/tests/side-midline", json=body).status_code == 422
        assert client.get("/api/orders").json()["total"] == 0


def test_test_and_normal_orders_share_one_active_slot(tmp_path):
    normal = {
        "canvas_width": 1024,
        "canvas_height": 768,
        "drawing_surface": "top",
        "strokes": [
            {
                "stroke_id": 0,
                "points": [{"x": 500.0, "y": 384.0}, {"x": 520.0, "y": 384.0}],
            }
        ],
    }
    with TestClient(create_app(tmp_path / "normal-first.sqlite3")) as client:
        assert client.post("/api/drawing", json=normal).status_code == 201
        assert (
            client.post(
                "/api/tests/side-midline", json={"line_length_mm": 30.0}
            ).status_code
            == 409
        )
    with TestClient(create_app(tmp_path / "test-first.sqlite3")) as client:
        assert (
            client.post(
                "/api/tests/side-midline", json={"line_length_mm": 30.0}
            ).status_code
            == 201
        )
        assert client.post("/api/drawing", json=normal).status_code == 409
