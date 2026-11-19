"""영구 이력·인증·통신 단절 표시를 실제 HTTP/SQLite 계약으로 검사한다."""

from datetime import datetime, timedelta, timezone
from fastapi.testclient import TestClient
from app.main import create_app

DRAWING = dict(
    canvas_width=1024,
    canvas_height=768,
    drawing_surface="top",
    strokes=[
        dict(stroke_id=0, points=[dict(x=500.0, y=380.0), dict(x=520.0, y=380.0)])
    ],
)
AUTH = {"Authorization": "Bearer test-secret"}


def start(client):
    oid = client.post("/api/drawing", json=DRAWING).json()["order_id"]
    claim = client.post("/internal/bridge/claim", headers=AUTH).json()

    def event(kind, eid, **kwargs):
        body = dict(
            event_id=eid,
            claim_token=claim["claim_token"],
            kind=kind,
            process_mode="full",
            **kwargs,
        )
        return client.post(
            f"/internal/bridge/orders/{oid}/events", headers=AUTH, json=body
        )

    assert event("dispatching", "dispatch").status_code == 200
    assert event("accepted", "accepted").status_code == 200
    return oid, event, claim


def test_empty_and_heartbeat_authentication_and_stale(tmp_path):
    with TestClient(create_app(tmp_path / "db", bridge_token="test-secret")) as c:
        snap = c.get("/api/hmi").json()
        assert snap["orders"] == [] and not snap["runtime"]["fresh"]
        body = dict(task_server_ready=True, execution_mode="real", process_mode="full")
        assert c.post("/internal/bridge/heartbeat", json=body).status_code == 401
        assert (
            c.post("/internal/bridge/heartbeat", headers=AUTH, json=body).status_code
            == 200
        )
        runtime = c.get("/api/hmi").json()["runtime"]
        assert runtime["fresh"] and runtime["task_server_ready"]
        assert (
            runtime["robot_connected"] is None and runtime["safety_status"] == "unknown"
        )
        with c.app.state.database.connection() as connection:
            connection.execute(
                "UPDATE hmi_runtime SET updated_at=?",
                ((datetime.now(timezone.utc) - timedelta(seconds=20)).isoformat(),),
            )
            connection.commit()
        runtime = c.get("/api/hmi").json()["runtime"]
        assert not runtime["fresh"] and not runtime["task_server_ready"]


def test_stage_history_is_idempotent_and_survives_restart_without_tokens(tmp_path):
    path = tmp_path / "db"
    with TestClient(create_app(path, bridge_token="test-secret")) as c:
        oid, event, claim = start(c)
        fields = dict(
            stage_key="opener_pick",
            stage_state="running",
            phase="OPENER_PICK",
            message="오프너 집기",
            progress_pct=0.0,
        )
        assert event("feedback", "stage1", **fields).status_code == 200
        assert event("feedback", "stage1", **fields).status_code == 200
        assert (
            event(
                "feedback",
                "stage2",
                **{**fields, "stage_state": "completed", "progress_pct": 8.0},
            ).status_code
            == 200
        )
        snap = c.get("/api/hmi", params={"order_id": oid}).json()
        assert len([e for e in snap["events"] if e["stage_key"]]) == 2
        assert snap["selected"]["progress_pct"] == 8.0
        assert snap["selected"]["process_mode"] == "full"
        text = c.get("/api/hmi").text
        assert claim["claim_token"] not in text and "test-secret" not in text
        assert c.get("/api/hmi", params={"order_id": "' OR 1=1 --"}).status_code == 404
    with TestClient(create_app(path, bridge_token="test-secret")) as c:
        snap = c.get("/api/hmi").json()
        assert snap["selected"]["order_id"] == oid and len(snap["events"]) == 4
        assert snap["selected"]["state"] == "running"


def test_stage_error_and_terminal_error_are_preserved(tmp_path):
    with TestClient(create_app(tmp_path / "db", bridge_token="test-secret")) as c:
        oid, event, _ = start(c)
        assert (
            event(
                "feedback",
                "stage-error",
                stage_key="cake_to_station",
                stage_state="failed",
                error_code="CAKE_TO_STATION_FAILED",
                message="스테이션 진입 이동 실패",
            ).status_code
            == 200
        )
        assert (
            event(
                "failed",
                "result",
                error_code="PLATE_PREPARE_FAILED",
                message="준비 실패",
                elapsed_time_sec=2.0,
            ).status_code
            == 200
        )
        snap = c.get("/api/hmi").json()
        assert snap["selected"]["state"] == "failed"
        assert snap["selected"]["error_code"] == "PLATE_PREPARE_FAILED"
        assert any(
            e["stage_state"] == "failed" and e["message"] == "스테이션 진입 이동 실패"
            for e in snap["events"]
        )
        count = len(snap["events"])
        assert (
            event(
                "feedback", "late", stage_key="drawing", stage_state="completed"
            ).status_code
            == 409
        )
        assert len(c.get("/api/hmi").json()["events"]) == count


def test_historical_order_mode_does_not_follow_current_runtime(tmp_path):
    with TestClient(create_app(tmp_path / "db", bridge_token="test-secret")) as c:
        oid, event, _ = start(c)
        event("completed", "done", progress_pct=100.0)
        c.post(
            "/internal/bridge/heartbeat",
            headers=AUTH,
            json=dict(
                task_server_ready=True,
                process_mode="drawing",
                execution_mode="simulation",
            ),
        )
        snap = c.get("/api/hmi", params={"order_id": oid}).json()
        assert snap["selected"]["process_mode"] == "full"
        assert snap["runtime"]["process_mode"] == "drawing"
        assert snap["events"][0]["kind"] == "completed"


def test_ros_error_observation_is_read_only(tmp_path):
    with TestClient(create_app(tmp_path / "db", bridge_token="test-secret")) as c:
        report = dict(
            task_server_ready=True,
            execution_mode="real",
            process_mode="full",
            last_error_message="드라이버 통신 오류",
            last_error_source="dsr_hw_interface2",
            last_error_at="2026-11-19T12:00:00+00:00",
        )
        assert (
            c.post("/internal/bridge/heartbeat", headers=AUTH, json=report).status_code
            == 200
        )
        data = c.get("/api/hmi").json()
        assert data["runtime"]["last_error_message"] == "드라이버 통신 오류"
        assert data["orders"] == []
        assert data["runtime"]["robot_connected"] is None
