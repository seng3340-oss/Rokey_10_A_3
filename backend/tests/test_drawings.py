"""서버 접수와 로봇 실행을 혼동하지 않도록 API·저장 동작을 검증한다."""

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json

from fastapi.testclient import TestClient
import pytest

from app.main import create_app


@pytest.fixture
def database_path(tmp_path):
    """테스트별 임시 DB 경로를 제공한다."""
    # 실제 개발 DB 대신 각 테스트 전용 파일을 사용한다.
    return tmp_path / "orders.sqlite3"


@pytest.fixture
def client(database_path):
    """앱의 시작·종료 절차까지 포함해 요청을 시험한다."""
    with TestClient(create_app(database_path=database_path)) as client:
        yield client


@pytest.fixture
def drawing():
    """PM 계약에 맞는 최소한의 유효한 그림을 제공한다."""
    # 프론트가 보내는 PM 계약과 동일한 원본 픽셀 데이터다.
    return {
        "canvas_width": 1024,
        "canvas_height": 768,
        "drawing_surface": "top",
        "strokes": [
            {
                "stroke_id": 0,
                "points": [{"x": 500.5, "y": 350.25}, {"x": 520.0, "y": 360.0}],
            }
        ],
    }


@pytest.mark.parametrize("surface", ["top", "side"])
def test_accept_and_read_original_drawing(client, drawing, surface):
    """윗면과 옆면 모두 원본이 저장되고 로봇 미전송 상태로 조회되는지 확인한다."""
    drawing["drawing_surface"] = surface
    response = client.post("/api/drawing", json=drawing)
    assert response.status_code == 201
    receipt = response.json()
    assert receipt["status"] == "received"
    assert receipt["order_id"].startswith("ORD-")
    detail = client.get(f"/api/orders/{receipt['order_id']}").json()
    assert detail["drawing"] == drawing
    assert detail["status"] == "received"
    assert detail["robot_dispatched"] is False
    assert client.get("/api/orders").json()["total"] == 1


@pytest.mark.parametrize(
    "change",
    [
        {"canvas_width": True},
        {"canvas_width": 0},
        {"canvas_height": "768"},
        {"drawing_surface": "bottom"},
        {"strokes": []},
        {"unknown_field": 1},
    ],
)
def test_reject_invalid_fields_without_saving(client, drawing, change):
    """필드 검증 실패가 DB에 불완전한 주문을 만들지 않는지 확인한다."""
    drawing.update(change)
    assert client.post("/api/drawing", json=drawing).status_code == 422
    assert client.get("/api/orders").json()["total"] == 0


@pytest.mark.parametrize("x", [-1, 1025, "500", True])
def test_reject_invalid_points(client, drawing, x):
    """음수·경계 초과·숫자 문자열·불리언 좌표를 거부하는지 확인한다."""
    drawing["strokes"][0]["points"][0]["x"] = x
    assert client.post("/api/drawing", json=drawing).status_code == 422


def test_reject_nonfinite_points(client, drawing):
    """유한하지 않은 좌표의 오류 응답도 올바른 JSON인지 확인한다."""
    # JSON 파서가 NaN을 허용하더라도 검증 응답은 유효한 JSON이어야 한다.
    body = json.dumps(drawing).replace("500.5", "NaN")
    response = client.post(
        "/api/drawing",
        content=body,
        headers={"Content-Type": "application/json"},
    )
    assert response.status_code == 422
    assert response.json()["detail"]


def test_reject_short_stroke_and_wrong_id(client, drawing):
    """경로 없는 획과 잘못된 획 순서가 접수되지 않는지 확인한다."""
    short = deepcopy(drawing)
    short["strokes"][0]["points"].pop()
    assert client.post("/api/drawing", json=short).status_code == 422
    drawing["strokes"][0]["stroke_id"] = 1
    assert client.post("/api/drawing", json=drawing).status_code == 422


def test_reject_total_point_limit(client, drawing):
    """개별 획 한도를 지켜도 전체 점 한도를 넘으면 거부하는지 확인한다."""
    points = [{"x": 500.0, "y": 350.0}] * 5000
    drawing["strokes"] = [{"stroke_id": i, "points": points} for i in range(5)]
    assert client.post("/api/drawing", json=drawing).status_code == 422


def test_orders_survive_restart_and_number_does_not_repeat(database_path, drawing):
    """서버 재시작이 주문을 지우거나 번호를 재사용하지 않는지 확인한다."""
    with TestClient(create_app(database_path=database_path)) as first:
        first_id = first.post("/api/drawing", json=drawing).json()["order_id"]
    with TestClient(create_app(database_path=database_path)) as restarted:
        assert restarted.get(f"/api/orders/{first_id}").status_code == 200
        second_id = restarted.post("/api/drawing", json=drawing).json()["order_id"]
        assert first_id != second_id


def test_concurrent_submissions_have_distinct_ids(client, drawing):
    """동시 요청의 주문 번호가 겹치지 않고 모두 저장되는지 확인한다."""
    # 여러 요청이 동시에 들어와도 일련번호와 주문 저장은 한 트랜잭션으로 처리한다.
    with ThreadPoolExecutor(max_workers=6) as executor:
        responses = list(
            executor.map(
                lambda _: client.post("/api/drawing", json=drawing),
                range(12),
            )
        )
    assert all(response.status_code == 201 for response in responses)
    assert len({response.json()["order_id"] for response in responses}) == 12
    assert client.get("/api/orders").json()["total"] == 12


def test_unknown_order_and_pagination(client, drawing):
    """존재하지 않는 주문과 잘못된 조회 범위를 구분하는지 확인한다."""
    assert client.get("/api/orders/ORD-MISSING").status_code == 404
    for _ in range(3):
        client.post("/api/drawing", json=drawing)
    page = client.get("/api/orders?limit=1&offset=1").json()
    assert page["total"] == 3
    assert len(page["items"]) == 1
    assert "drawing" not in page["items"][0]
    assert client.get("/api/orders?limit=0").status_code == 422


def test_body_limit_applies_before_json_parsing(database_path):
    """본문 크기 초과가 JSON 파싱 전에 차단되는지 확인한다."""
    with TestClient(create_app(database_path=database_path, max_body_bytes=512)) as client:
        response = client.post("/api/drawing", content=b"x" * 513)
        assert response.status_code == 413
        assert client.get("/api/orders").json()["total"] == 0


def test_hmi_and_bridge_apis_are_not_created(client):
    """이번 범위에서 제외한 HMI·브리지 API가 생기지 않는지 확인한다."""
    assert client.get("/api/hmi/state").status_code == 404
    assert client.get("/api/bridge").status_code == 404
    assert client.get("/api/health").json()["ros_connected"] is False
