"""GenerateTrajectory 결과의 표시 전용 사본. 원본/로봇 명령을 변경하지 않는다."""

import math
from .drawing_geometry import side_drawing_rect


def build_hmi_plan(order_id, drawing, cake, waypoints):
    """BASE m 좌표를 입력 캔버스로 역투영한다. 펜올림 구간은 서로 잇지 않는다."""
    width, height = int(drawing.canvas_width), int(drawing.canvas_height)
    radius, cake_height = float(cake.radius), float(cake.height)
    if width <= 0 or height <= 0 or radius <= 0 or cake_height <= 0:
        raise ValueError("계획 표시 치수가 유효하지 않습니다.")
    surface = drawing.surface.lower()
    if surface not in ("top", "side"):
        raise ValueError("지원하지 않는 계획 면입니다.")
    if len(waypoints) > 30000:
        raise ValueError("계획 표시의 최대 점 수를 넘었습니다.")
    groups, group = [], []
    left, top, strip_width, strip_height = side_drawing_rect(
        width, height, radius, cake_height
    )
    for waypoint in waypoints:
        if not waypoint.pen_down:
            if group:
                groups.append(group)
                group = []
            continue
        p = waypoint.pose.position
        if surface == "top":
            scale = 2 * radius / (min(width, height) * (640 / 768))
            x = width / 2 + (p.x - cake.center.x) / scale
            y = height / 2 - (p.y - cake.center.y) / scale
        else:
            angle = math.atan2(p.y - cake.center.y, p.x - cake.center.x) % (2 * math.pi)
            x = left + angle / (2 * math.pi) * strip_width
            y = (
                top
                + strip_height
                - (p.z - (cake.center.z - cake_height)) / cake_height * strip_height
            )
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("계획 표시 좌표가 유한하지 않습니다.")
        # ROS float32 반올림의 미소 경계 오차만 허용한다. 실제 범위 초과는 숨기지 않는다.
        if x < -0.001 or x > width + 0.001 or y < -0.001 or y > height + 0.001:
            raise ValueError("계획 좌표가 원본 캔버스 밖입니다.")
        group.append(
            {"x": max(0.0, min(float(width), x)), "y": max(0.0, min(float(height), y))}
        )
    if group:
        groups.append(group)
    if not groups or any(len(points) < 2 for points in groups):
        raise ValueError("표시 가능한 계획 획이 없습니다.")
    return {
        "version": 1,
        "source": "task_manager_node/GenerateTrajectory",
        "order_id": order_id,
        "drawing_surface": surface,
        "canvas_width": width,
        "canvas_height": height,
        "waypoint_count": len(waypoints),
        "strokes": [
            {"stroke_id": index, "points": points}
            for index, points in enumerate(groups)
        ],
    }
