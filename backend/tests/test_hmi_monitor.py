"""장비 연결 없이 실제 TM·브릿지 콜백 → FastAPI → HMI 조회 계약을 검사한다.

ROS 생성자·executor·액션 전송은 실행하지 않는다. 원본 메서드의 AST만 로드해
테스트 더블의 메시지/시계를 주입하므로 네트워크나 로봇 서비스가 필요 없다.
"""
import ast
from enum import Enum
import math
import time
from pathlib import Path
from queue import Empty, Full, Queue
from types import SimpleNamespace as NS

from fastapi.testclient import TestClient
import pytest

from app.main import create_app

ROOT = Path(__file__).resolve().parents[2]
NODES = ROOT / "ws_cobot1/src/cake_decorator/cake_decorator"
DRAWING = {
    "canvas_width": 1024, "canvas_height": 768, "drawing_surface": "top",
    "strokes": [{"stroke_id": 0, "points": [{"x": 500., "y": 350.}, {"x": 520., "y": 360.}]}],
}


def load_methods(filename, class_name, names, namespace):
    """원본 메서드만 읽어 ROS 초기화와 모듈 import를 피한다."""
    tree = ast.parse((NODES / filename).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    exec(compile(module, filename, "exec"), namespace)
    return namespace[class_name]


@pytest.mark.parametrize("surface", ["top", "side"])
def test_tm_bridge_api_hmi_contract(tmp_path, surface):
    """면별 원본·진행률·오류를 저장하며 안전·실행 모드는 unknown으로 유지한다."""
    phase = Enum("Phase", {"DRAWING": "DRAWING"})
    tm_type = load_methods("task_manager_node.py", "TaskManagerNode", {
        "_on_draw_feedback", "_publish_feedback", "_publish_status", "_drawing_progress_range",
    }, {"math": math, "Phase": phase, "CakeOrder": NS(Feedback=NS),
        "TaskStatus": NS, "TimeMsg": NS})
    bridge_type = load_methods(
        "ros_bridge_node.py",
        "RosBridgeNode",
        {
            "_on_status",
            "_on_feedback",
            "_flush_feedback",
            "_flush_plan",
        },
        {"SimpleNamespace": NS, "Empty": Empty, "Full": Full, "time": time},
    )
    drawing = {**DRAWING, "drawing_surface": surface}
    with TestClient(create_app(tmp_path / "test.sqlite3", bridge_token="test-only")) as client:
        order_id = client.post("/api/drawing", json=drawing).json()["order_id"]
        auth = {"Authorization": "Bearer test-only"}
        claim = client.post("/internal/bridge/claim", headers=auth).json()
        count = 0

        def event(claim, kind, **values):
            """실제 내부 HTTP 계약에 테스트 이벤트만 전달한다. ROS 전송은 없다."""
            nonlocal count
            count += 1
            response = client.post(f"/internal/bridge/orders/{order_id}/events", headers=auth,
                json={"event_id": str(count), "claim_token": claim["claim_token"], "kind": kind, **values})
            assert response.status_code == 200, response.text

        event(claim, "dispatching")
        event(claim, "accepted")
        bridge = bridge_type()
        bridge.active_order_id = order_id
        bridge.feedback = Queue(maxsize=1)
        bridge.backend = NS(event=event)
        tm = tm_type()
        tm._current_phase = phase.DRAWING
        tm._current_progress_pct = 0.0
        tm._last_order_id = order_id
        tm._current_order_id = order_id
        tm.get_parameter = lambda _: NS(value=False)
        tm.get_clock = lambda: NS(now=lambda: NS(seconds_nanoseconds=lambda: (123, 0)))
        statuses, action_feedback = [], []
        tm._status_pub = NS(publish=lambda status: (statuses.append(status), bridge._on_status(status)))
        goal = NS(request=NS(order_id=order_id), publish_feedback=action_feedback.append)
        feedback = NS(feedback=NS(progress_pct=42., phase="DRAWING", current_stroke_idx=1, total_strokes=3))
        tm._on_draw_feedback(feedback, goal)
        assert action_feedback[-1].progress_pct == statuses[-1].progress_pct == 42.
        bridge._flush_feedback(claim)
        detail = client.get(f"/api/orders/{order_id}").json()
        assert detail["progress_pct"] == 42.
        assert detail["phase"] == "DRAWING"
        assert detail["drawing"] == drawing
        assert client.get("/api/orders").json()["items"][0]["progress_pct"] == 42.
        # 플레이트 준비·출고 구간을 남기고 여러 면의 진행률을 환산한다.
        tm.get_parameter = lambda _: NS(value=True)
        tm._on_draw_feedback(feedback, goal, 1, 2)
        assert statuses[-1].progress_pct == pytest.approx(66.8)
        before = len(statuses)
        feedback.feedback.progress_pct = float("nan")
        tm._on_draw_feedback(feedback, goal)
        goal.request.order_id = "old-order"
        feedback.feedback.progress_pct = 10.
        tm._on_draw_feedback(feedback, goal)
        assert len(statuses) == before
        event(claim, "failed", message="test failure", error_code="DRAWING_FAILED")
        before_state = client.get(f"/api/orders/{order_id}").json()
        meta = client.get(f"/api/orders/{order_id}/monitor").json()
        assert meta["error_code"] == "DRAWING_FAILED"
        assert before_state["error_code"] == "DRAWING_FAILED"
        assert client.get("/api/orders").json()["items"][0]["error_code"] == "DRAWING_FAILED"
        assert "claim_token" not in before_state
        assert meta["execution_mode"] == meta["safety_status"] == "unknown"
        assert meta["trajectory_available"] is False
        assert "claim_token" not in meta and meta["updated_at"]
        # 읽기 API는 상태를 바꾸지 않으며 없는 주문과 쓰기 요청을 거부한다.
        assert client.get(f"/api/orders/{order_id}").json() == before_state
        assert client.get("/api/orders/missing/monitor").status_code == 404
        assert client.post(f"/api/orders/{order_id}/monitor").status_code == 405
        assert client.get("/api/health").json()["ros_status_monitored"] is False
