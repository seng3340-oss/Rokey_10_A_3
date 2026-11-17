"""GenerateTrajectory 서비스 서버 노드 — 원통형(cylinder) 케이크.

TM이 프론트엔드에서 받은 캔버스 픽셀 좌표를 그대로 전달하면,
DP(이 노드) 내부에서 픽셀→mm 변환 후 원통형 좌표 계산을 수행하여
로봇 절대 좌표 3D 경로로 응답한다.

전체 흐름:
  웹(픽셀) → 백엔드 → 브릿지·TM → 이 노드(DP)
  DP 내부: 픽셀→mm → 원통형 3D 계산 → Waypoint3D[]
  → robot_executor_node → 로봇 이동

좌표계 약속:
  - srv 입력: Stroke2D (캔버스 픽셀, 왼쪽 위 = (0,0))
  - srv cake_pose: TCP로 기록한 윗면 중심 (m)
  - 내부: 바닥 중심 기준으로 변환 (bottom_z = top_z - height)
"""

from __future__ import annotations

import math

from cake_decorator_interfaces.msg import Waypoint3D as Waypoint3DMsg
from cake_decorator_interfaces.srv import GenerateTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
import rclpy
from rclpy.node import Node

from .path_simplification import simplify_stroke_mm
from .surface_path import (
    side_surface_leg,
    continuous_side_leg,
    SplineSurfaceMove,
    split_spline_samples,
)

from .drawing_geometry import (
    CylinderCake,
    pen_quat_side,
    pen_quat_top,
    pixel_to_mm_side,
    pixel_to_mm_top,
)


class DrawingPathNode(Node):
    """캔버스 픽셀 좌표 → 로봇 절대 좌표 3D 경로 변환 서비스 노드."""

    def __init__(self):
        super().__init__("drawing_path_node")

        # ── 변환 설정 파라미터 (yaml에서 로드) ──
        self.declare_parameter("drawing_speed", 30.0)  # mm/s, 테스트2 종이 검증 기준
        self.declare_parameter("travel_speed", 80.0)  # mm/s (펜 올린 상태)
        self.declare_parameter("approach_height", 10.0)  # mm (펜 접근 높이)
        self.declare_parameter("drawing_z_offset", 0.0)  # mm (표면 접촉 보정)
        self.declare_parameter("default_contact_force", 0.5)  # N
        self.declare_parameter("simplify_tolerance_mm", 0.15)
        self.declare_parameter("duplicate_tolerance_mm", 0.01)
        self.declare_parameter("side_path_error_mm", 0.1)
        self.declare_parameter("side_diagonal_mode", "spline")
        self.declare_parameter("side_pen_vertical_deg", 30.0)
        self.declare_parameter("pen_tilt_deg", 30.0)  # 펜 기울기 (도)
        # TOP: 수직에서 기울어지는 각도 (0=수직, 30=30도 기울임)
        # SIDE: 법선에서 아래로 기울어지는 각도

        self.srv = self.create_service(
            GenerateTrajectory,
            "generate_trajectory",
            self._handle_request,
        )
        self.get_logger().info("drawing_path_node 서비스 서버 시작 (pixel→cylinder)")

    def _param(self, name: str):
        return self.get_parameter(name).value

    # ==================================================================
    # 서비스 콜백
    # ==================================================================

    def _handle_request(self, request, response):
        self.get_logger().info(
            f"경로 변환 요청: order={request.order_id}, "
            f"surface={request.surface_type}, "
            f"strokes={len(request.strokes_2d)}, "
            f"speed={request.operation_speed_percent}%, "
            f"canvas=({request.canvas_width}x{request.canvas_height}), "
            f"radius={request.cake_radius}"
        )

        try:
            waypoints, pen_states = self._convert_strokes(request)

            if not waypoints:
                raise ValueError(
                    f"변환할 웨이포인트가 없습니다. (strokes={len(request.strokes_2d)})"
                )
            response.success = True
            response.waypoints_3d = waypoints
            response.pen_states = pen_states
            response.estimated_duration_sec = self._estimate_duration(waypoints)
            response.error_msg = ""

            self.get_logger().info(
                f"경로 변환 완료: waypoints={len(waypoints)}, "
                f"예상 시간={response.estimated_duration_sec:.1f}s"
            )
        except Exception as e:
            self.get_logger().error(f"경로 변환 실패: {e}")
            response.success = False
            response.error_msg = str(e)
            response.estimated_duration_sec = 0.0

        return response

    # ==================================================================
    # 스트로크 변환 (핵심 로직)
    # ==================================================================

    def _convert_strokes(self, request):
        """캔버스 픽셀 좌표 → 로봇 절대 좌표 Waypoint3D 변환.

        1. 윗면 중심(TCP) → 바닥 중심으로 변환
        2. CylinderCake 생성
        3. 각 스트로크의 픽셀 좌표 → mm → 3D 절대 좌표
        """
        # ── 윗면 중심 → 바닥 중심 변환 (m → mm) ──
        top_x = request.cake_pose.position.x * 1000.0
        top_y = request.cake_pose.position.y * 1000.0
        top_z = request.cake_pose.position.z * 1000.0
        cake_height_mm = request.cake_height * 1000.0
        cake_radius_mm = request.cake_radius * 1000.0

        bottom_z = top_z - cake_height_mm  # 바닥 중심 Z

        # ── CylinderCake 생성 (바닥 중심 기준, mm) ──
        cake = CylinderCake(
            radius=cake_radius_mm,
            height=cake_height_mm,
            cx=top_x,  # X, Y는 윗면·바닥 동일
            cy=top_y,
            cz=bottom_z,  # 바닥 중심 Z
        )

        # ── 캔버스·속도 파라미터 ──
        canvas_w = request.canvas_width
        canvas_h = request.canvas_height

        speed_ratio = max(1, min(100, request.operation_speed_percent)) / 100.0
        draw_spd = self._param("drawing_speed") * speed_ratio
        travel_spd = self._param("travel_speed") * speed_ratio
        approach_h = self._param("approach_height")
        z_offset = self._param("drawing_z_offset")
        force = self._param("default_contact_force")
        tilt_deg = self._param("pen_tilt_deg")

        surface = request.surface_type.upper()
        if surface not in ("TOP", "SIDE"):
            raise ValueError(f"지원하지 않는 surface_type입니다: {request.surface_type}")
        if not all(
            math.isfinite(v) and v > 0
            for v in (canvas_w, canvas_h, cake.radius, cake.height, approach_h)
        ):
            raise ValueError("해상도·케이크 치수·접근 거리는 양수여야 합니다.")
        if surface == "SIDE" and z_offset < 0:
            raise ValueError(
                "옆면 안쪽으로 들어가는 음수 접촉 오프셋은 허용하지 않습니다."
            )

        waypoints: list[Waypoint3DMsg] = []
        pen_states: list[bool] = []

        previous_side_end = None
        total_input = total_selected = 0
        for stroke in request.strokes_2d:
            pts_px = [(pt.x, pt.y) for pt in stroke.points]
            if len(pts_px) < 2:
                continue

            converter = pixel_to_mm_top if surface == "TOP" else pixel_to_mm_side
            args = (
                (canvas_w, canvas_h, cake.radius)
                if surface == "TOP"
                else (canvas_w, canvas_h, cake.radius, cake.height)
            )
            raw_mm = [converter(x, y, *args) for x, y in pts_px]
            selected = simplify_stroke_mm(
                raw_mm,
                self._param("simplify_tolerance_mm"),
                self._param("duplicate_tolerance_mm"),
            )
            total_input += len(raw_mm)
            total_selected += len(selected.points)
            if surface == "TOP":
                self._convert_top_mm(
                    selected.points,
                    cake,
                    approach_h,
                    z_offset,
                    draw_spd,
                    travel_spd,
                    force,
                    tilt_deg,
                    waypoints,
                    pen_states,
                )
            else:
                previous_side_end = self._convert_side_mm(
                    selected.points,
                    previous_side_end,
                    cake,
                    approach_h,
                    z_offset,
                    draw_spd,
                    travel_spd,
                    force,
                    tilt_deg,
                    waypoints,
                    pen_states,
                )
            if len(waypoints) > 50000:
                raise ValueError("표면 경로 명령 한도를 초과했습니다.")
        self.get_logger().info(
            f'점 선별: 원본 {total_input} → {total_selected}, 허용 오차 {self._param("simplify_tolerance_mm")}mm'
        )

        # 웨이포인트의 펜 상태를 기준으로 응답을 만든다. 분리된 배열과의 불일치는
        # 첫 획 누락으로 이어질 수 있으므로 여기서 즉시 검출한다.
        waypoint_states = [waypoint.pen_down for waypoint in waypoints]
        if pen_states != waypoint_states:
            raise ValueError("웨이포인트와 펜 상태 배열이 일치하지 않습니다.")
        return waypoints, waypoint_states

    def _convert_top_mm(
        self,
        points,
        cake,
        clearance,
        offset,
        draw_speed,
        travel_speed,
        force,
        tilt,
        waypoints,
        states,
    ):
        """선별된 상면 점을 변환한다. 획 시작·끝과 펜 상태 전환은 바꾸지 않는다."""
        quat = pen_quat_top(tilt_deg=tilt)
        x, y, z = cake.top_to_abs(*points[0])
        waypoints.append(self._wp(x, y, z + clearance, quat, travel_speed, force))
        states.append(False)
        waypoints.append(self._wp(x, y, z + offset, quat, draw_speed * 0.5, force, pen_down=True))
        states.append(True)
        for point in points[1:]:
            x, y, z = cake.top_to_abs(*point)
            waypoints.append(self._wp(x, y, z + offset, quat, draw_speed, force, pen_down=True))
            states.append(True)
        waypoints.append(self._wp(x, y, z + clearance, quat, travel_speed, force))
        states.append(False)

    def _side_leg(
        self, start, end, cake, offset, speed, force, tilt, pen_down, waypoints, states
    ):
        """원호의 경유점까지 명시해 RE가 옆면 구간을 movel로 단축하지 않게 한다."""
        mode = self._param("side_diagonal_mode")
        if mode not in ("spline", "arc_vertical"):
            raise ValueError("옆면 대각선 모드가 유효하지 않습니다.")
        diagonal = abs(end[0] - start[0]) > 1e-9 and abs(end[1] - start[1]) > 1e-9
        moves = (
            continuous_side_leg(cake, start, end, offset)
            if pen_down and diagonal and mode == "spline"
            else side_surface_leg(
                cake, start, end, offset, self._param("side_path_error_mm")
            )
        )
        for move in moves:
            quat = pen_quat_side(move.arc_mm, cake, tilt_deg=tilt)
            target = self._wp(*move.end, quat, speed, force, pen_down=pen_down)
            if isinstance(move, SplineSurfaceMove):
                target.motion_type = Waypoint3DMsg.SPLINE
                target.spline_poses = [
                    self._wp(
                        *sample, pen_quat_side(arc, cake, tilt_deg=tilt), speed, force
                    ).pose
                    for sample, arc in zip(move.samples, move.sample_arcs)
                ]
            elif move.via is not None:
                target.motion_type = Waypoint3DMsg.CIRCLE
                via_quat = pen_quat_side(move.via_arc_mm, cake, tilt_deg=tilt)
                target.via_pose = self._wp(*move.via, via_quat, speed, force).pose
            waypoints.append(target)
            states.append(pen_down)
            if len(waypoints) > 50000:
                raise ValueError("표면 명령 수 한도 초과")

    def _convert_side_mm(
        self,
        points,
        previous,
        cake,
        clearance,
        offset,
        draw_speed,
        travel_speed,
        force,
        tilt,
        waypoints,
        states,
    ):
        """첫 접근은 상면 위, 획 사이는 바깥 반지름의 원호를 따라 이동한다."""
        first, last = points[0], points[-1]
        vertical_angle = self._param("side_pen_vertical_deg")
        if not math.isfinite(vertical_angle) or not 0 < vertical_angle < 90:
            raise ValueError("옆면 펜의 수직 기준 각도는 0~90도 사이여야 합니다.")
        # 기존 자세 함수는 수평 법선 기준이다. 수직 아래에서 30도면 해당 함수에는 60도.
        tilt = 90.0 - vertical_angle
        if not all(0 <= h <= cake.height for _, h in points):
            raise ValueError("옆면 입력이 편집 띠 높이 밖입니다.")
        quat = pen_quat_side(first[0], cake, tilt_deg=tilt)
        if previous is None:
            # 시작 TCP가 상면 중심인 계약을 따른다. 먼저 위로 빠져나와 원통 밖으로 간다.
            # 옆면 자세를 상면 위부터 유지해 수직 자세의 손목 특이점을 반복 통과하지 않는다.
            top_quat = quat
            waypoints.append(
                self._wp(
                    cake.cx,
                    cake.cy,
                    cake.cz + cake.height + clearance,
                    top_quat,
                    travel_speed,
                    force,
                )
            )
            states.append(False)
            x, y, _ = cake.side_abs_with_offset(first[0], cake.height, clearance)
            waypoints.append(
                self._wp(
                    x, y, cake.cz + cake.height + clearance, quat, travel_speed, force
                )
            )
            states.append(False)
            x, y, z = cake.side_abs_with_offset(*first, clearance)
            waypoints.append(self._wp(x, y, z, quat, travel_speed, force))
            states.append(False)
        else:
            self._side_leg(
                previous,
                first,
                cake,
                clearance,
                travel_speed,
                force,
                tilt,
                False,
                waypoints,
                states,
            )
        x, y, z = cake.side_abs_with_offset(*first, offset)
        waypoints.append(self._wp(x, y, z, quat, draw_speed * 0.5, force, pen_down=True))
        states.append(True)
        is_horizontal = all(abs(point[1] - first[1]) < 1e-8 for point in points)
        is_vertical = all(abs(point[0] - first[0]) < 1e-8 for point in points)
        if self._param("side_diagonal_mode") == "spline" and not (
            is_horizontal or is_vertical
        ):
            # 한 획의 연속 곡선을 모아 보내 RDP 점마다 정지하지 않게 한다.
            samples = []
            for start, end in zip(points, points[1:]):
                if math.dist(start, end) < 1e-9:
                    continue
                for move in continuous_side_leg(cake, start, end, offset):
                    samples.extend(zip(move.samples, move.sample_arcs))
            if len(samples) > 20000:
                raise ValueError("옆면 연속 표본 한도 초과")
            for chunk in split_spline_samples(samples):
                if len(chunk) < 2:
                    raise ValueError("스플라인 마지막 묶음은 두 점 이상이어야 합니다.")
                poses = [
                    self._wp(
                        *sample,
                        pen_quat_side(arc, cake, tilt_deg=tilt),
                        draw_speed,
                        force,
                    ).pose
                    for sample, arc in chunk
                ]
                target = self._wp(
                    *chunk[-1][0],
                    pen_quat_side(chunk[-1][1], cake, tilt_deg=tilt),
                    draw_speed,
                    force,
                    pen_down=True,
                )
                target.motion_type = Waypoint3DMsg.SPLINE
                target.spline_poses = poses
                waypoints.append(target)
                states.append(True)
        else:
            for start, end in zip(points, points[1:]):
                self._side_leg(
                    start,
                    end,
                    cake,
                    offset,
                    draw_speed,
                    force,
                    tilt,
                    True,
                    waypoints,
                    states,
                )
        x, y, z = cake.side_abs_with_offset(*last, clearance)
        quat = pen_quat_side(last[0], cake, tilt_deg=tilt)
        waypoints.append(self._wp(x, y, z, quat, travel_speed, force))
        states.append(False)
        return last

    # ------------------------------------------------------------------
    # Waypoint 생성
    # ------------------------------------------------------------------

    @staticmethod
    def _wp(
        x_mm: float,
        y_mm: float,
        z_mm: float,
        quat: tuple[float, float, float, float],
        speed: float,
        force: float,
        pen_down: bool = False,
    ) -> Waypoint3DMsg:
        """Waypoint3D 메시지 생성. mm → m 변환 포함."""
        msg = Waypoint3DMsg()
        msg.pose = Pose(
            position=Point(x=x_mm / 1000.0, y=y_mm / 1000.0, z=z_mm / 1000.0),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )
        msg.speed = speed
        msg.pen_down = pen_down
        msg.contact_force = force
        return msg

    # ------------------------------------------------------------------
    # 시간 추정
    # ------------------------------------------------------------------

    @staticmethod
    def _estimate_duration(waypoints: list[Waypoint3DMsg]) -> float:
        if len(waypoints) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(waypoints)):
            p = waypoints[i - 1].pose.position
            c = waypoints[i].pose.position
            dist = math.sqrt(
                (c.x - p.x) ** 2 + (c.y - p.y) ** 2 + (c.z - p.z) ** 2,
            )
            speed_m_s = waypoints[i].speed / 1000.0
            if speed_m_s > 0:
                total += dist / speed_m_s
        return total


def main(args=None):
    rclpy.init(args=args)
    node = DrawingPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
