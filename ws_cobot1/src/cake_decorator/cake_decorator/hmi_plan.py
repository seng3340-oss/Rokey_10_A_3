"""최신 DP 응답을 원본 캔버스에 투영하는 표시 전용 어댑터. 로봇 명령은 바꾸지 않는다."""

import math


def build_hmi_plan(order_id, drawing, cake, waypoints, *, reference_pose=None):
    """실제 요청의 기준점·원뿔대·yaw로 계획 제어점을 역투영한다.

    TOP/SIDE의 교정 기준이 다를 수 있어 TM이 DP에 보낸 자세를 우선 사용한다.
    펜올림은 획을 나누고 원호의 경유점·스플라인 표본도 표시한다. 곡선의 제어점을
    잇는 미리보기이며 제어기 보간 결과나 실측 TCP 경로를 뜻하지 않는다.
    """
    width, height = int(drawing.canvas_width), int(drawing.canvas_height)
    radius = float(cake.radius)
    bottom_radius = float(cake.bottom_radius)
    cake_height = float(cake.height)
    yaw = float(cake.orientation)
    center = reference_pose.position if reference_pose is not None else cake.center
    values = (radius, bottom_radius, cake_height, yaw, center.x, center.y, center.z)
    if (
        width <= 0 or height <= 0
        or not all(math.isfinite(value) for value in values)
        or min(radius, bottom_radius, cake_height) <= 0
    ):
        raise ValueError("계획 표시 치수·기준점이 유효하지 않습니다.")
    surface = drawing.surface.lower()
    if surface not in ("top", "side"):
        raise ValueError("지원하지 않는 계획 면입니다.")
    if not 2 <= len(waypoints) <= 30000:
        raise ValueError("계획 표시의 명령 수가 허용 범위를 벗어났습니다.")

    # 웹·DP가 사용하는 사면 길이/평균 둘레와 동일한 편집 띠다. 단위는 m다.
    strip_width = width * (896.0 / 1024.0)
    strip_height = (
        strip_width * math.hypot(cake_height, bottom_radius - radius)
        / (math.pi * (radius + bottom_radius))
    )
    left, top = (width - strip_width) / 2.0, (height - strip_height) / 2.0
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    groups, group = [], []
    previous_position = None
    count = 0

    for waypoint in waypoints:
        if not waypoint.pen_down:
            if len(group) >= 2:
                groups.append(group)
            group = []
            previous_position = waypoint.pose.position
            continue

        # CIRCLE은 직전 목표→경유점→목표, SPLINE은 표본→목표를 표시한다.
        # 각 목록은 새로 구성하므로 원본 ROS 메시지와 실행 순서는 수정되지 않는다.
        motion = int(getattr(waypoint, "motion_type", 0))
        positions = []
        if motion == 1 and previous_position is not None:
            positions.append(waypoint.via_pose.position)
        elif motion == 2:
            positions.extend(pose.position for pose in waypoint.spline_poses)
        positions.append(waypoint.pose.position)

        for position in positions:
            count += 1
            if count > 20000:
                raise ValueError("계획 표시의 최대 제어점 수를 넘었습니다.")
            if not all(math.isfinite(v) for v in (position.x, position.y, position.z)):
                raise ValueError("계획 표시 좌표가 유한하지 않습니다.")
            dx, dy = position.x - center.x, position.y - center.y
            # BASE 평면에서 케이크 로컬 평면으로 역회전한다. TCP 자세는 사용하지 않는다.
            local_x = dx * cos_yaw + dy * sin_yaw
            local_y = -dx * sin_yaw + dy * cos_yaw
            if surface == "top":
                scale = 2.0 * radius / (height * 640.0 / 768.0)
                x, y = width / 2.0 + local_x / scale, height / 2.0 - local_y / scale
            else:
                angle = math.atan2(local_y, local_x) % (2.0 * math.pi)
                # 끝 경계의 float 오차로 같은 획이 왼쪽 경계로 튀는 것을 막는다.
                if group and angle < 1e-6 and group[-1]["x"] > width / 2.0:
                    angle = 2.0 * math.pi
                x = left + angle * strip_width / (2.0 * math.pi)
                y = top + strip_height * (center.z - position.z) / cake_height
                # 전개도의 이음새를 가로지르는 선은 화면 전체를 잇지 않는다.
                if group and abs(x - group[-1]["x"]) > strip_width / 2.0:
                    if len(group) >= 2:
                        groups.append(group)
                    group = []
            # 직렬화 반올림만 허용한다. 잘못된 기준점을 조용히 정상 경로로 보이지 않는다.
            if x < -0.001 or x > width + 0.001 or y < -0.001 or y > height + 0.001:
                raise ValueError("계획 좌표가 원본 캔버스 밖입니다.")
            point = {
                "x": max(0.0, min(float(width), x)),
                "y": max(0.0, min(float(height), y)),
            }
            if not group or point != group[-1]:
                group.append(point)
        previous_position = waypoint.pose.position
    if len(group) >= 2:
        groups.append(group)
    if not groups or len(groups) > 100:
        raise ValueError("표시 가능한 계획 획 수가 허용 범위를 벗어났습니다.")
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
