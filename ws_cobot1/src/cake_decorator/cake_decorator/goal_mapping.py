"""백엔드 원본 데이터를 ROS Goal로 포장한다. 점 선별·로봇 이동은 수행하지 않는다."""

import math

from geometry_msgs.msg import Point32, Quaternion
from cake_decorator_interfaces.action import CakeOrder
from cake_decorator_interfaces.msg import CakeParams, DrawingPath, Stroke2D


def validate_reference(pose):
    """BASE mm·ZYZ degree 6값을 검사한다. 실제 접촉 여부는 사용자가 확인한다."""
    if len(pose) != 6 or not all(math.isfinite(float(value)) for value in pose):
        raise ValueError("유효한 TCP 영점 6값이 필요합니다.")
    return [float(value) for value in pose]


def build_goal(claim, reference):
    """측정한 TCP 위치를 상면 중심으로 사용하고 원본 픽셀·획 순서를 유지한다."""
    x, y, z, a, b, c = validate_reference(reference)
    cake = claim["cake"]
    if cake["reference"] != "tcp_at_top_center":
        raise ValueError("지원하지 않는 케이크 원점입니다.")
    if "diameter_mm" not in cake:
        raise ValueError(
            "이전 경사 케이크 주문입니다. 새 원통형 설정으로 다시 접수해야 합니다."
        )
    dimensions = [
        cake[key]
        for key in (
            "diameter_mm",
            "height_mm",
            "board_width_mm",
            "board_depth_mm",
            "board_thickness_mm",
        )
    ]
    if not all(math.isfinite(value) and value > 0 for value in dimensions):
        raise ValueError("실측 치수가 유효하지 않습니다.")
    diameter, height, width, depth, thickness = dimensions
    if diameter > min(width, depth):
        raise ValueError("케이크와 판의 크기가 맞지 않습니다.")
    params = CakeParams(
        radius=diameter / 2000,
        height=height / 1000,
        board_width=width / 1000,
        board_depth=depth / 1000,
        board_thickness=thickness / 1000,
        reference_frame="base",
        orientation=0.0,
    )
    # 이미 상면에 닿은 TCP이므로 케이크 높이나 판 두께를 더하지 않는다.
    params.center.x, params.center.y, params.center.z = x / 1000, y / 1000, z / 1000
    # 두산 자세 Rz(a)Ry(b)Rz(c)를 ROS 쿼터니언으로 변환한다.
    a, b, c = (math.radians(value) for value in (a, b, c))
    params.tool_orientation = Quaternion(
        x=-math.sin(b / 2) * math.sin((a - c) / 2),
        y=math.sin(b / 2) * math.cos((a - c) / 2),
        z=math.cos(b / 2) * math.sin((a + c) / 2),
        w=math.cos(b / 2) * math.cos((a + c) / 2),
    )
    drawing = claim["drawing"]
    if drawing["drawing_surface"] not in ("top", "side"):
        raise ValueError("그릴 면이 유효하지 않습니다.")
    if not all(
        isinstance(drawing[key], int) and 0 < drawing[key] <= 8192
        for key in ("canvas_width", "canvas_height")
    ):
        raise ValueError("캔버스 해상도가 유효하지 않습니다.")
    path = DrawingPath(
        canvas_width=drawing["canvas_width"],
        canvas_height=drawing["canvas_height"],
        surface=drawing["drawing_surface"].upper(),
    )
    if not 1 <= len(drawing["strokes"]) <= 100:
        raise ValueError("획 개수가 유효하지 않습니다.")
    total = 0
    for expected_id, raw in enumerate(drawing["strokes"]):
        if raw["stroke_id"] != expected_id or not 2 <= len(raw["points"]) <= 5000:
            raise ValueError("획 순서 또는 점 개수가 유효하지 않습니다.")
        stroke = Stroke2D(stroke_id=expected_id)
        for point in raw["points"]:
            px, py = point["x"], point["y"]
            if not (
                math.isfinite(px)
                and math.isfinite(py)
                and 0 <= px <= path.canvas_width
                and 0 <= py <= path.canvas_height
            ):
                raise ValueError("원본 좌표가 캔버스 밖입니다.")
            stroke.points.append(Point32(x=float(px), y=float(py), z=0.0))
        total += len(stroke.points)
        path.strokes.append(stroke)
    if total > 20_000:
        raise ValueError("총 좌표점 한도를 초과했습니다.")
    return CakeOrder.Goal(
        order_id=claim["order_id"],
        drawing_paths=[path],
        cake_params=params,
        tool_type=cake["tool_type"],
    )
