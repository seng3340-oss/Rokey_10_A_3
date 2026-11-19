"""GenerateTrajectory 서비스 서버 노드 — 원뿔대(frustum) 케이크.

TM이 프론트엔드에서 받은 캔버스 픽셀 좌표를 그대로 전달하면,
DP(이 노드) 내부에서 픽셀→mm 변환 후 원뿔대 좌표 계산을 수행하여
로봇 절대 좌표 3D 경로로 응답한다.

전체 흐름:
  웹(픽셀) → 백엔드 → 브릿지·TM → 이 노드(DP)
  DP 내부: 픽셀→mm → 원뿔대 3D 계산 → Waypoint3D[]
  → robot_executor_node → 로봇 이동

좌표계 약속:
  - srv 입력: Stroke2D (캔버스 픽셀, 왼쪽 위 = (0,0))
  - srv cake_pose: TCP로 기록한 윗면 중심 (m)
  - 내부: 바닥 중심 기준으로 변환 (bottom_z = top_z - height)
"""

from __future__ import annotations

import math
from dataclasses import replace

from cake_decorator_interfaces.msg import Waypoint3D as Waypoint3DMsg
from cake_decorator_interfaces.srv import GenerateTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
import rclpy
from rclpy.node import Node

from .frustum_path import DrawingFrustum, pixel_to_path_mm
from .path_simplification import simplify_stroke_mm
from .surface_path import (
    side_surface_leg,
    side_clearance_leg,
    continuous_side_leg,
    SplineSurfaceMove,
    split_spline_samples,
)

from .drawing_geometry import (
    pen_quat_side,
    pen_quat_top,
    pixel_to_mm_top,
)


class DrawingPathNode(Node):
    """캔버스 픽셀 좌표 → 로봇 절대 좌표 3D 경로 변환 서비스 노드."""

    def __init__(self):
        super().__init__("drawing_path_node")

        # ── 변환 설정 파라미터 (yaml에서 로드) ──
        self.declare_parameter("drawing_speed", 30.0)  # mm/s, 테스트2 종이 검증 기준
        self.declare_parameter("side_drawing_speed", 60.0)  # 옆면 접촉 후 그리기 mm/s
        self.declare_parameter("travel_speed", 80.0)  # mm/s (펜 올린 상태)
        self.declare_parameter("approach_height", 10.0)  # mm (펜 접근 높이)
        self.declare_parameter("side_diameter_allowance_mm", 3.5)  # 옆면 경로 지름 보정(mm)
        self.declare_parameter("drawing_z_offset", 0.0)  # mm (표면 접촉 보정)
        self.declare_parameter("default_contact_force", 0.5)  # N
        self.declare_parameter("simplify_tolerance_mm", 0.15)
        self.declare_parameter("duplicate_tolerance_mm", 0.01)
        self.declare_parameter("side_path_error_mm", 0.1)
        self.declare_parameter("side_diagonal_mode", "spline")
        self.declare_parameter("side_pen_vertical_deg", 17.0)
        # 양수이면 BASE +Y/-Y 가장자리를 각각 같은 거리만큼 안으로 줄인다.
        self.declare_parameter("side_y_inset_mm", 0.0)
        self.declare_parameter("integrated_side_cycle", False)
        self.declare_parameter("side_pen_45", False)  # 45도 실물 파지 웹 검증 모드
        self.declare_parameter("pen_tilt_deg", 0.0)  # 영점 자세가 없는 TOP 요청의 기본값
        # TOP: 수직에서 기울어지는 각도 (0=수직, 30=30도 기울임)
        # SIDE: 법선에서 아래로 기울어지는 각도

        self.srv = self.create_service(
            GenerateTrajectory,
            "generate_trajectory",
            self._handle_request,
        )
        self.get_logger().info("drawing_path_node 서비스 서버 시작 (pixel→frustum)")

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
        2. DrawingFrustum 생성
        3. 각 스트로크의 픽셀 좌표 → mm → 3D 절대 좌표
        """
        # ── 윗면 중심 → 바닥 중심 변환 (m → mm) ──
        top_x = request.cake_pose.position.x * 1000.0
        top_y = request.cake_pose.position.y * 1000.0
        top_z = request.cake_pose.position.z * 1000.0
        cake_height_mm = request.cake_height * 1000.0
        cake_radius_mm = request.cake_radius * 1000.0

        if not math.isfinite(request.cake_yaw):
            raise ValueError("케이크 회전각은 유한 값이어야 합니다.")
        bottom_z = top_z - cake_height_mm  # 바닥 중심 Z

        # ── 실제 반지름과 고정 경로 호 좌표를 연결한다 (바닥 중심 기준, mm). ──
        cake = DrawingFrustum(
            r_top=cake_radius_mm,
            r_bottom=request.cake_bottom_radius * 1000.0,
            yaw=request.cake_yaw,
            height=cake_height_mm,
            cx=top_x,  # X, Y는 윗면·바닥 동일
            cy=top_y,
            cz=bottom_z,  # 바닥 중심 Z
        )

        # ── 캔버스·속도 파라미터 ──
        canvas_w = request.canvas_width
        canvas_h = request.canvas_height

        speed_ratio = max(1, min(100, request.operation_speed_percent)) / 100.0
        side_speed = self._param("side_drawing_speed")
        if not math.isfinite(side_speed) or side_speed <= 0:
            raise ValueError("옆면 그리기 속도는 양수여야 합니다.")
        draw_spd = self._param("drawing_speed") * speed_ratio
        travel_spd = self._param("travel_speed") * speed_ratio
        approach_h = self._param("approach_height")
        z_offset = self._param("drawing_z_offset")
        force = self._param("default_contact_force")
        tilt_deg = self._param("pen_tilt_deg")
        # 실제 주문은 YES 시점의 펜 자세를 고정해 쓴다. 이전 클라이언트처럼
        # 쿼터니언이 모두 0이면 기본 수직 자세로 대체한다. 옆면은 별도 계산한다.
        reference = request.cake_pose.orientation
        top_quat = (reference.x, reference.y, reference.z, reference.w)
        norm = math.sqrt(sum(value * value for value in top_quat))
        if not math.isfinite(norm):
            raise ValueError("영점 펜 자세가 유효하지 않습니다.")
        top_quat = (
            tuple(value / norm for value in top_quat)
            if norm > 1e-8 else pen_quat_top(tilt_deg=tilt_deg)
        )

        surface = request.surface_type.upper()
        if surface not in ("TOP", "SIDE"):
            raise ValueError(f"지원하지 않는 surface_type입니다: {request.surface_type}")
        if not all(
            math.isfinite(v) and v > 0
            for v in (canvas_w, canvas_h, cake.r_top, cake.r_bottom, cake.height, approach_h)
        ):
            raise ValueError("해상도·케이크 치수·접근 거리는 양수여야 합니다.")
        if surface == "SIDE" and z_offset < 0:
            raise ValueError(
                "옆면 안쪽으로 들어가는 음수 접촉 오프셋은 허용하지 않습니다."
            )

        if surface == "SIDE":
            allowance = self._param("side_diameter_allowance_mm")
            if not math.isfinite(allowance) or not 0 <= allowance <= 20:
                raise ValueError("옆면 지름 보정은 0~20mm여야 합니다.")
            y_inset = self._param("side_y_inset_mm")
            if not math.isfinite(y_inset) or not 0 <= y_inset <= 5:
                raise ValueError("옆면 Y 양쪽 보정은 각각 0~5mm여야 합니다.")
            cake = replace(
                cake,
                radial_offset_mm=allowance / 2.0,
                side_y_inset_mm=y_inset,
            )
            self.get_logger().info(f"옆면 BASE Y 양쪽 안쪽 보정: 각각 {y_inset:.2f}mm")
            self.get_logger().info(f"옆면 펜촉 반지름 보정: +{allowance / 2.0:.2f}mm")

        waypoints: list[Waypoint3DMsg] = []
        pen_states: list[bool] = []

        previous_side_end = None
        total_input = total_selected = 0
        for stroke in request.strokes_2d:
            pts_px = [(pt.x, pt.y) for pt in stroke.points]
            if len(pts_px) < 2:
                continue

            converter = pixel_to_mm_top if surface == "TOP" else pixel_to_path_mm
            args = (
                (canvas_w, canvas_h, cake.r_top)
                if surface == "TOP"
                else (canvas_w, canvas_h, cake)
            )
            if not all(
                math.isfinite(x) and math.isfinite(y)
                and 0 <= x <= canvas_w and 0 <= y <= canvas_h
                for x, y in pts_px
            ):
                raise ValueError("원본 픽셀이 캔버스 범위를 벗어났습니다.")
            raw_mm = [converter(x, y, *args) for x, y in pts_px]
            if surface == "TOP":
                from .drawing_geometry import validate_top_drawing_margin
                validate_top_drawing_margin(raw_mm, cake.r_top)
            if surface == "SIDE":
                from .drawing_geometry import validate_side_drawing_height

                validate_side_drawing_height(raw_mm)
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
                    reference_quat=top_quat,
                )
            else:
                previous_side_end = self._convert_side_mm(
                    selected.points,
                    previous_side_end,
                    cake,
                    approach_h,
                    z_offset,
                    side_speed * speed_ratio,
                    travel_spd,
                    force,
                    tilt_deg,
                    waypoints,
                    pen_states,
                    reference_quat=top_quat,
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
        # 이미 기울어진 펜을 잡고 시작하는 독립 시험은 TOP을 허용하지 않는다.
        if self._param("side_pen_45") and surface == "TOP" and not self._param("integrated_side_cycle"):
            raise ValueError("독립 45도 파지 시험에서는 옆면만 그릴 수 있습니다.")
        # TOP은 기존 수직 펜 경로를 유지하며 SIDE에만 검증된 45도 자세를 적용한다.
        if self._param("side_pen_45") and surface == "SIDE":
            from .bent_pen import apply_side_gripper_orientations

            apply_side_gripper_orientations(waypoints, cake.cx / 1000., cake.cy / 1000.)
            # 거치대에서 접근할 때의 자세 전환은 공중 원주 시험과 같이 상면 위에서 한다.
            # 370mm 바깥 목표는 긴 펜 TCP를 적용한 가상 제어기에서 도달 불가였다.
            # 실제 상면220mm에서 70mm 위인290mm로 이동·공중 회전을 검증했다.
            safe_z = (cake.cz + cake.height + 70.) / 1000.
            for waypoint in waypoints[:3]:
                waypoint.pose.position.z = max(waypoint.pose.position.z, safe_z)
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
        reference_quat=None,
    ):
        """선별된 상면 점을 변환한다. 획 시작·끝과 펜 상태 전환은 바꾸지 않는다."""
        quat = reference_quat if reference_quat is not None else pen_quat_top(tilt_deg=tilt)
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
        self, start, end, cake, offset, speed, force, tilt, pen_down, waypoints, states,
        reference_quat=None,
    ):
        """원호의 경유점까지 명시해 RE가 옆면 구간을 movel로 단축하지 않게 한다."""
        mode = self._param("side_diagonal_mode")
        if mode not in ("spline", "arc_vertical"):
            raise ValueError("옆면 대각선 모드가 유효하지 않습니다.")
        diagonal = abs(end[0] - start[0]) > 1e-9 and abs(end[1] - start[1]) > 1e-9
        if not pen_down:
            # 획 사이에는 현재 높이에서 케이크 바깥으로 빠져 다음 획 높이까지 곡선 이동한다.
            moves = side_clearance_leg(cake, start, end, offset)
        elif diagonal and mode == "spline":
            moves = continuous_side_leg(cake, start, end, offset)
        else:
            moves = side_surface_leg(
                cake, start, end, offset, self._param("side_path_error_mm")
            )
        # 높이와 케이크 객체를 각각 전달해 새 펜 방향 계산을 사용한다.
        for move in moves:
            quat = pen_quat_side(move.arc_mm, move.end[2] - cake.cz, cake, tilt_deg=tilt, reference_quat=reference_quat)
            target = self._wp(*move.end, quat, speed, force, pen_down=pen_down)
            if isinstance(move, SplineSurfaceMove):
                target.motion_type = Waypoint3DMsg.SPLINE
                target.spline_poses = [
                    self._wp(
                        *sample, pen_quat_side(arc, sample[2] - cake.cz, cake, tilt_deg=tilt, reference_quat=reference_quat), speed, force
                    ).pose
                    for sample, arc in zip(move.samples, move.sample_arcs)
                ]
            elif move.via is not None:
                target.motion_type = Waypoint3DMsg.CIRCLE
                via_quat = pen_quat_side(move.via_arc_mm, move.via[2] - cake.cz, cake, tilt_deg=tilt, reference_quat=reference_quat)
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
        reference_quat=None,
    ):
        """첫 접근은 상면 위, 획 사이는 바깥 반지름의 원호를 따라 이동한다."""
        first, last = points[0], points[-1]
        vertical_angle = self._param("side_pen_vertical_deg")
        if not math.isfinite(vertical_angle) or not 0 < vertical_angle < 90:
            raise ValueError("옆면 펜의 수직 기준 각도는 0~90도 사이여야 합니다.")
        # 기존 자세 함수는 수평 법선 기준이다. 수직 아래에서 17도면 해당 함수에는 73도.
        # 경사 법선은 이미 아래로 기울어져 있다. 그 각도를 빼야 PR #16의
        # 수직 기준 펜 각도가 원뿔대에서도 그대로 유지된다.
        tilt = 90.0 - vertical_angle - math.degrees(cake.slant_angle)
        if not all(0 <= h <= cake.height for _, h in points):
            raise ValueError("옆면 입력이 편집 띠 높이 밖입니다.")
        quat = pen_quat_side(first[0], first[1], cake, tilt_deg=tilt, reference_quat=reference_quat)
        if previous is None:
            # 시작 TCP가 상면 중심인 계약을 따른다. 먼저 위로 빠져나와 원통 밖으로 간다.
            # 옆면 자세를 상면 위부터 유지해 수직 자세의 손목 특이점을 반복 통과하지 않는다.
            # 영점 자세로 먼저 들어 올리고 케이크 바깥으로 이동한다.
            # 펜 자세 변경은 그 다음 바깥 경유점에서 수행한다.
            top_quat = reference_quat if reference_quat is not None else quat
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
            waypoints.append(self._wp(x, y, cake.cz + cake.height + clearance, top_quat, travel_speed, force))
            states.append(False)
            waypoints.append(self._wp(x, y, cake.cz + cake.height + clearance, quat, travel_speed, force))
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
                reference_quat=reference_quat,
            )
        x, y, z = cake.side_abs_with_offset(*first, offset)
        # 접촉 진입은 기존 속도를 넘기지 않는다. 빨라지는 구간은 접촉 후의 그림이다.
        speed_ratio = draw_speed / self._param("side_drawing_speed")
        contact_speed = self._param("drawing_speed") * speed_ratio * 0.5
        waypoints.append(self._wp(x, y, z, quat, contact_speed, force, pen_down=True))
        states.append(True)
        is_horizontal = all(abs(point[1] - first[1]) < 1e-8 for point in points)
        is_vertical = all(abs(point[0] - first[0]) < 1e-8 for point in points)
        side_45 = self._param("side_pen_45")
        if side_45 or (
            self._param("side_diagonal_mode") == "spline"
            and not (is_horizontal or is_vertical)
        ):
            # 한 획의 연속 곡선을 모아 보내 RDP 점마다 정지하지 않게 한다.
            # 45도 펜은 수평·수직도 같은 획으로 묶어 90도 원호 경계 정지를 없앤다.
            # 표본은 최대 4도/1mm로 생성해 100점 제한 때문에 생기는 명령 경계를 줄인다.
            # 큰 반지름에서는 현의 처짐이 0.05mm 이내가 되도록 각도를 더 좁힌다.
            max_radius = max(cake.r_top, cake.r_bottom) + cake.radial_offset_mm + clearance
            angle_step = min(
                4.0,
                math.degrees(2 * math.acos(1 - min(0.05 / max_radius, 1.0))),
            ) if side_45 else 1.0
            samples = []
            for start, end in zip(points, points[1:]):
                if math.dist(start, end) < 1e-9:
                    continue
                for move in continuous_side_leg(
                    cake, start, end, offset,
                    max_angle_deg=angle_step,
                    max_height_mm=1.0 if side_45 else 0.5,
                ):
                    samples.extend(zip(move.samples, move.sample_arcs))
            if len(samples) > 20000:
                raise ValueError("옆면 연속 표본 한도 초과")
            for chunk in split_spline_samples(samples):
                if len(chunk) < 2:
                    raise ValueError("스플라인 마지막 묶음은 두 점 이상이어야 합니다.")
                poses = [
                    self._wp(
                        *sample,
                        pen_quat_side(arc, sample[2] - cake.cz, cake, tilt_deg=tilt, reference_quat=reference_quat),
                        draw_speed,
                        force,
                    ).pose
                    for sample, arc in chunk
                ]
                target = self._wp(
                    *chunk[-1][0],
                    pen_quat_side(chunk[-1][1], chunk[-1][0][2] - cake.cz, cake, tilt_deg=tilt, reference_quat=reference_quat),
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
                    reference_quat=reference_quat,
                )
        x, y, z = cake.side_abs_with_offset(*last, clearance)
        quat = pen_quat_side(last[0], last[1], cake, tilt_deg=tilt, reference_quat=reference_quat)
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
