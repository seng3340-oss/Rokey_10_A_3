"""실장비 없이 운용용 TM→브릿지→HTTP→HMI 계약과 권한·노후화를 검사한다."""

import json
import math
from pathlib import Path
from queue import Queue, Empty
import sys
from types import SimpleNamespace as NS
import pytest
from fastapi.testclient import TestClient
from app.main import create_app
from test_hmi_monitor import load_methods

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ws_cobot1/src/cake_decorator"))
from cake_decorator.hmi_plan import build_hmi_plan
from cake_decorator.drawing_geometry import pixel_to_mm_side, side_drawing_rect

AUTH = {"Authorization": "Bearer test-only"}
DRAWING = {
    "canvas_width": 1024,
    "canvas_height": 768,
    "drawing_surface": "top",
    "strokes": [
        {"stroke_id": 0, "points": [{"x": 500.0, "y": 350.0}, {"x": 520.0, "y": 360.0}]}
    ],
}


def started(client, drawing=DRAWING):
    """임시 DB의 단일 주문만 인수한다. ROS Goal은 전송하지 않는다."""
    oid = client.post("/api/drawing", json=drawing).json()["order_id"]
    claim = client.post("/internal/bridge/claim", headers=AUTH).json()
    for index, kind in enumerate(["dispatching", "accepted"]):
        response = client.post(
            f"/internal/bridge/orders/{oid}/events",
            headers=AUTH,
            json={
                "event_id": str(index),
                "claim_token": claim["claim_token"],
                "kind": kind,
                "execution_mode": "real",
            },
        )
        assert response.status_code == 200
    return oid, claim


def test_heartbeat_is_authenticated_read_only_and_expires(tmp_path):
    with TestClient(
        create_app(tmp_path / "db.sqlite3", bridge_token="test-only")
    ) as client:
        assert client.get("/api/monitor").json()["observed"] is False
        report = {
            "execution_mode": "real",
            "task_server_ready": True,
            "active_order_id": None,
            "task_feedback_age_sec": None,
        }
        assert client.post("/internal/bridge/heartbeat", json=report).status_code == 401
        assert (
            client.post(
                "/internal/bridge/heartbeat", json=report, headers=AUTH
            ).status_code
            == 200
        )
        snapshot = client.get("/api/monitor").json()
        assert snapshot["bridge_fresh"] and snapshot["task_server_ready"]
        assert (
            snapshot["robot_connected"] is None
            and snapshot["safety_status"] == "unknown"
        )
        assert client.get("/api/orders").json()["total"] == 0
        assert client.post("/api/monitor", json={}).status_code == 405
        with client.app.state.database.connection() as connection:
            connection.execute("UPDATE hmi_runtime SET received_at=received_at-20")
            connection.commit()
        stale = client.get("/api/monitor").json()
        assert not stale["bridge_fresh"] and not stale["task_server_ready"]


@pytest.mark.parametrize("surface", ["top", "side"])
def test_actual_plan_serializer_bridge_queue_and_public_plan(tmp_path, surface):
    drawing = {**DRAWING, "drawing_surface": surface}
    cake = NS(center=NS(x=0.399, y=-0.049, z=0.219), radius=0.05, height=0.08)
    original = drawing["strokes"][0]["points"]
    points = []
    for p in original:
        if surface == "top":
            x = 0.399 + (p["x"] - 512) * 0.1 / 640
            y = -0.049 + (384 - p["y"]) * 0.1 / 640
            z = 0.219
        else:
            arc, height = pixel_to_mm_side(
                p["x"],
                p["y"],
                1024,
                768,
                0.05,
                0.08,
                drawing_rect=side_drawing_rect(1024, 768, 0.05, 0.08),
            )
            x = 0.399 + 0.05 * math.cos(arc / 0.05)
            y = -0.049 + 0.05 * math.sin(arc / 0.05)
            z = 0.139 + height
        points.append(NS(pose=NS(position=NS(x=x, y=y, z=z)), pen_down=True))
    with TestClient(
        create_app(tmp_path / "db.sqlite3", bridge_token="test-only")
    ) as client:
        oid, claim = started(client, drawing)
        plan = build_hmi_plan(
            oid,
            NS(canvas_width=1024, canvas_height=768, surface=surface.upper()),
            cake,
            points,
        )
        assert plan["strokes"][0]["points"][0]["x"] == pytest.approx(original[0]["x"])
        bridge_type = load_methods(
            "ros_bridge_node.py",
            "RosBridgeNode",
            {"_on_plan", "_flush_plan"},
            {"json": json, "Empty": Empty, "Full": __import__("queue").Full},
        )
        bridge = bridge_type()
        bridge.active_order_id = oid
        bridge.plans = Queue(maxsize=1)
        bridge.get_logger = lambda: NS(warning=lambda _: None)

        def send(claim, plan):
            response = client.post(
                f"/internal/bridge/orders/{oid}/plan",
                headers=AUTH,
                json={
                    "claim_token": claim["claim_token"],
                    "execution_mode": "real",
                    "plan": plan,
                },
            )
            assert response.status_code == 200, response.text

        bridge.backend = NS(plan=send)
        bridge._on_plan(NS(data=json.dumps(plan)))
        bridge._flush_plan(claim)
        public = client.get(f"/api/orders/{oid}/plan").json()
        assert (
            public["execution_mode"] == "real"
            and public["measurement_available"] is False
        )
        assert public["strokes"] == plan["strokes"] and "claim_token" not in public
        assert (
            client.get(f"/api/orders/{oid}/monitor").json()["execution_mode"] == "real"
        )
        assert client.post(f"/api/orders/{oid}/plan", json=plan).status_code == 405
        bad = {**plan, "drawing_surface": "side" if surface == "top" else "top"}
        assert (
            client.post(
                f"/internal/bridge/orders/{oid}/plan",
                headers=AUTH,
                json={"claim_token": claim["claim_token"], "plan": bad},
            ).status_code
            == 409
        )
        assert (
            client.post(
                f"/internal/bridge/orders/{oid}/plan",
                headers=AUTH,
                json={"claim_token": "wrong", "plan": plan},
            ).status_code
            == 409
        )
        assert client.get(f"/api/orders/{oid}").json()["status"] == "running"
        # 늦은 다른 주문 메시지와 표시 서버 장애는 주문 상태를 바꾸지 않는다.
        bridge._on_plan(NS(data=json.dumps({**plan, "order_id": "old-order"})))
        assert bridge.plans.empty()
        bridge.plans.put(plan)
        bridge.backend = NS(
            plan=lambda *args: (_ for _ in ()).throw(RuntimeError("offline"))
        )
        bridge._flush_plan(claim)
        assert client.get(f"/api/orders/{oid}").json()["status"] == "running"


def test_static_workspace_uses_no_virtual_execution_endpoints(tmp_path):
    dist = tmp_path / "dist"
    dist.mkdir()
    (dist / "index.html").write_text("<html>production-ui</html>")
    with TestClient(create_app(tmp_path / "db.sqlite3", frontend_dist=dist)) as client:
        for path in ["/", "/hmi", "/workspace", "/paired"]:
            assert client.get(path).status_code == 200
        assert client.get("/rviz").status_code == 404
        assert client.post("/api/rviz-camera", json={}).status_code == 404
        assert client.get("/api/orders").json()["total"] == 0
