"""현재 중심 기준 50mm 십자가를 웹 주문과 같은 경로로 제출한다.

--preview는 계산만 한다. 일반 실행은 TOP 주문 하나를 제출하므로 실행 중인
start_real.sh의 케이크 운반·펜 집기·그리기·반납·회수 공정이 진행된다.
"""

import argparse
import json
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]


def make_cross_payload(top_diameter_mm):
    """640px 원 지름에 실제 윗지름을 대응시켜 정확히 50mm인 두 획을 만든다."""
    if top_diameter_mm <= 50:
        raise ValueError("50mm 십자가를 넣으려면 케이크 윗지름이 50mm보다 커야 합니다.")
    half_px = 25.0 * 640.0 / top_diameter_mm
    segments = [
        [(512.0 - half_px, 384.0), (512.0 + half_px, 384.0)],
        [(512.0, 384.0 + half_px), (512.0, 384.0 - half_px)],
    ]
    return {
        "canvas_width": 1024,
        "canvas_height": 768,
        "drawing_surface": "top",
        "strokes": [
            {"stroke_id": i, "points": [{"x": x, "y": y} for x, y in points]}
            for i, points in enumerate(segments)
        ],
    }


def main():
    """미리보기는 통신하지 않으며, 실행 요청은 자동 재전송하지 않는다."""
    parser = argparse.ArgumentParser(description="윗면 중심 확인용 가로·세로 각50mm 십자가")
    parser.add_argument("--preview", action="store_true", help="좌표만 출력하고 주문하지 않음")
    parser.add_argument("--backend-url", default="http://127.0.0.1:5000")
    args = parser.parse_args()
    cake = json.loads((ROOT / "config/test_cake.json").read_text())
    payload = make_cross_payload(cake["top_diameter_mm"])
    print("가로50mm·세로50mm, 교차점에서 각 끝까지25mm인 TOP 주문입니다.")
    print("XY/Z는 실행 중인 start_real.sh가 읽은 설정을 사용합니다.")
    if args.preview:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return
    request = Request(
        args.backend_url.rstrip("/") + "/api/drawing",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "Idempotency-Key": str(uuid4())},
        method="POST",
    )
    try:
        with urlopen(request, timeout=15) as response:
            receipt = json.load(response)
        print(f"주문 접수: {receipt['order_id']} — 실제 완료 여부는 실행기에서 확인하세요.")
    except HTTPError as error:
        raise SystemExit(f"주문 거부 HTTP {error.code}: {error.read().decode()}") from error
    except (URLError, TimeoutError) as error:
        raise SystemExit("응답을 확인하지 못했습니다. 접수됐을 수 있으므로 주문 상태를 확인한 뒤 재실행하세요.") from error


if __name__ == "__main__":
    main()
