"""GenerateTrajectory 서비스 서버 노드 — 원뿔대(frustum) 케이크 지원.

TM(task_manager)이 프론트엔드 JSON과 동일한 형식으로 요청하면
mm 단위 2D/3D 스트로크를 로봇 절대 좌표 3D 경로로 변환해 응답한다.
실제 로봇 이동은 수행하지 않는다.

케이크 형상::

       ╭───────╮  ← r_top (윗면)
      ╱         ╲
     ╱           ╲  height
    ╱             ╲
   ╰───────────────╯  ← r_bottom (아랫면)

데이터 흐름:
  프론트엔드 JSON → task_manager_node → (srv Request) → 이 노드

  TOP 면: (x, y) mm → 케이크 윗면 절대좌표
  SIDE 면: (x_arc, y_height) mm → 원뿔대 표면 3D + 법선 방향 자세
    높이 y에서의 반지름: r(y) = r_bottom + (r_top - r_bottom) × y / H
    θ = x_arc / r(y), 위치 = (r·cosθ, r·sinθ, y)
"""

from __future__ import annotations

import math

from cake_decorator_interfaces.msg import Waypoint3D as Waypoint3DMsg
from cake_decorator_interfaces.srv import GenerateTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
import rclpy
from rclpy.node import Node

from .drawing_geometry import FrustumCake, pen_quat_side, pen_quat_top


class DrawingPathNode(Node):
    """mm 스트로크 → 로봇 절대 좌표 3D 경로 변환 서비스 노드."""

    def __init__(self):
        super().__init__('drawing_path_node')

        # ── 변환 설정 파라미터 (yaml에서 로드) ──
        self.declare_parameter('drawing_speed', 20.0)         # mm/s
        self.declare_parameter('travel_speed', 80.0)          # mm/s
        self.declare_parameter('approach_height', 10.0)       # mm
        self.declare_parameter('drawing_z_offset', 0.0)       # mm
        self.declare_parameter('default_contact_force', 0.5)  # N

        self.srv = self.create_service(
            GenerateTrajectory,
            'generate_trajectory',
            self._handle_request,
        )
        self.get_logger().info('drawing_path_node 서비스 서버 시작 (frustum 지원)')

    def _param(self, name: str):
        return self.get_parameter(name).value

    # ==================================================================
    # 서비스 콜백
    # ==================================================================

    def _handle_request(self, request, response):
        """GenerateTrajectory 서비스 콜백."""
        self.get_logger().info(
            f'경로 변환 요청: order={request.order_id}, '
            f'surface={request.surface_type}, '
            f'strokes={len(request.strokes)}, '
            f'speed={request.operation_speed_percent}%, '
            f'r_top={request.cake_radius_top}, '
            f'r_bottom={request.cake_radius_bottom}'
        )

        try:
            waypoints, pen_states = self._convert_strokes(request)

            response.success = True
            response.waypoints_3d = waypoints
            response.pen_states = pen_states
            response.estimated_duration_sec = self._estimate_duration(waypoints)
            response.error_msg = ''

            self.get_logger().info(
                f'경로 변환 완료: waypoints={len(waypoints)}, '
                f'예상 시간={response.estimated_duration_sec:.1f}s'
            )
        except Exception as e:
            self.get_logger().error(f'경로 변환 실패: {e}')
            response.success = False
            response.error_msg = str(e)
            response.estimated_duration_sec = 0.0

        return response

    # ==================================================================
    # 스트로크 변환 (핵심 로직)
    # ==================================================================

    def _convert_strokes(self, request):
        """모든 스트로크를 로봇 절대 좌표 Waypoint3D로 변환."""
        # FrustumCake 생성 (m → mm 변환)
        cake = FrustumCake(
            r_top=request.cake_radius_top * 1000.0,
            r_bottom=request.cake_radius_bottom * 1000.0,
            height=request.cake_height * 1000.0,
            cx=request.cake_pose.position.x * 1000.0,
            cy=request.cake_pose.position.y * 1000.0,
            cz=request.cake_pose.position.z * 1000.0,
        )

        # 속도 계산: operation_speed_percent 반영
        speed_ratio = max(1, min(100, request.operation_speed_percent)) / 100.0
        draw_spd = self._param('drawing_speed') * speed_ratio
        travel_spd = self._param('travel_speed') * speed_ratio
        approach_h = self._param('approach_height')
        z_offset = self._param('drawing_z_offset')
        force = self._param('default_contact_force')

        surface = request.surface_type.upper()

        waypoints: list[Waypoint3DMsg] = []
        pen_states: list[bool] = []

        for stroke in request.strokes:
            pts = self._extract_points(stroke)
            if not pts:
                continue

            if surface == 'TOP':
                self._convert_stroke_top(
                    pts, cake, approach_h, z_offset,
                    draw_spd, travel_spd, force,
                    waypoints, pen_states,
                )
            else:  # SIDE
                self._convert_stroke_side(
                    pts, cake, approach_h, z_offset,
                    draw_spd, travel_spd, force,
                    waypoints, pen_states,
                )

        return waypoints, pen_states

    # ------------------------------------------------------------------
    # TOP 면 변환
    # ------------------------------------------------------------------

    def _convert_stroke_top(
        self, pts, cake: FrustumCake,
        approach_h, z_offset, draw_spd, travel_spd, force,
        waypoints, pen_states,
    ):
        """TOP 면: (x, y) mm → 윗면 절대좌표. 펜은 아래를 향함.

        프론트엔드의 (x, y)는 윗면 중심 기준 오프셋.
        Z 방향으로 접근·접촉·이탈.
        """
        quat = pen_quat_top()

        # ── 1. 펜 올린 상태로 첫 점 위 이동 ──
        fx, fy, fz = cake.top_to_abs(pts[0][0], pts[0][1])
        waypoints.append(self._wp(fx, fy, fz + approach_h, quat, travel_spd, force))
        pen_states.append(False)

        # ── 2. 펜 내리기 (첫 점 접촉) ──
        waypoints.append(self._wp(fx, fy, fz + z_offset, quat, draw_spd * 0.5, force))
        pen_states.append(True)

        # ── 3. 획 그리기 ──
        for pt in pts[1:]:
            ax, ay, az = cake.top_to_abs(pt[0], pt[1])
            waypoints.append(self._wp(ax, ay, az + z_offset, quat, draw_spd, force))
            pen_states.append(True)

        # ── 4. 펜 올리기 ──
        lx, ly, lz = cake.top_to_abs(pts[-1][0], pts[-1][1])
        waypoints.append(self._wp(lx, ly, lz + approach_h, quat, travel_spd, force))
        pen_states.append(False)

    # ------------------------------------------------------------------
    # SIDE 면 변환 (원뿔대 지오메트리)
    # ------------------------------------------------------------------

    def _convert_stroke_side(
        self, pts, cake: FrustumCake,
        approach_h, z_offset, draw_spd, travel_spd, force,
        waypoints, pen_states,
    ):
        """SIDE 면: (x_arc, y_height) mm → 원뿔대 표면 3D.

        x = 원주 방향 호 길이 (mm), y = 바닥부터 높이 (mm).
        접근·접촉·이탈 모두 표면 법선 방향으로 수행.
        펜 자세는 점마다 다름 (원주 위치 + 경사각 반영).
        """
        # ── 1. 펜 올린 상태로 첫 점 법선 방향 접근 ──
        x0, y0 = pts[0][0], pts[0][1]
        quat0 = pen_quat_side(x0, y0, cake)
        ax0, ay0, az0 = cake.side_abs_with_offset(x0, y0, approach_h)
        waypoints.append(self._wp(ax0, ay0, az0, quat0, travel_spd, force))
        pen_states.append(False)

        # ── 2. 펜 내리기 (표면 접촉) ──
        cx0, cy0, cz0 = cake.side_abs_with_offset(x0, y0, z_offset)
        waypoints.append(self._wp(cx0, cy0, cz0, quat0, draw_spd * 0.5, force))
        pen_states.append(True)

        # ── 3. 획 그리기 ──
        for pt in pts[1:]:
            xi, yi = pt[0], pt[1]
            qi = pen_quat_side(xi, yi, cake)
            cxi, cyi, czi = cake.side_abs_with_offset(xi, yi, z_offset)
            waypoints.append(self._wp(cxi, cyi, czi, qi, draw_spd, force))
            pen_states.append(True)

        # ── 4. 펜 올리기 ──
        xn, yn = pts[-1][0], pts[-1][1]
        quatn = pen_quat_side(xn, yn, cake)
        axn, ayn, azn = cake.side_abs_with_offset(xn, yn, approach_h)
        waypoints.append(self._wp(axn, ayn, azn, quatn, travel_spd, force))
        pen_states.append(False)

    # ------------------------------------------------------------------
    # 좌표 추출·변환 헬퍼
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_points(stroke) -> list[tuple[float, float]]:
        """스트로크에서 (x, y) mm 튜플 리스트를 추출한다.

        우선순위: target_mm > simplified_mm > original_mm
        target_mm의 z는 SIDE 면에서 y_height로 사용하거나
        TOP 면에서 무시될 수 있으므로, (x, y)만 추출한다.
        (z 정보가 필요한 경우 target_mm에서 별도 처리)
        """
        if stroke.target_mm:
            return [(pt.x, pt.y) for pt in stroke.target_mm]
        if stroke.simplified_mm:
            return [(pt.x, pt.y) for pt in stroke.simplified_mm]
        if stroke.original_mm:
            return [(pt.x, pt.y) for pt in stroke.original_mm]
        return []

    # ------------------------------------------------------------------
    # Waypoint 생성
    # ------------------------------------------------------------------

    @staticmethod
    def _wp(
        x_mm: float, y_mm: float, z_mm: float,
        quat: tuple[float, float, float, float],
        speed: float,
        force: float,
    ) -> Waypoint3DMsg:
        """Waypoint3D 메시지 생성. mm → m 변환 포함."""
        msg = Waypoint3DMsg()
        msg.pose = Pose(
            position=Point(
                x=x_mm / 1000.0, y=y_mm / 1000.0, z=z_mm / 1000.0,
            ),
            orientation=Quaternion(
                x=quat[0], y=quat[1], z=quat[2], w=quat[3],
            ),
        )
        msg.speed = speed
        msg.pen_down = False  # pen_states 배열로 별도 관리
        msg.contact_force = force
        return msg

    # ------------------------------------------------------------------
    # 시간 추정
    # ------------------------------------------------------------------

    @staticmethod
    def _estimate_duration(waypoints: list[Waypoint3DMsg]) -> float:
        """총 소요 시간 추정 (초). 거리 / 속도 합산."""
        if len(waypoints) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(waypoints)):
            p = waypoints[i - 1].pose.position
            c = waypoints[i].pose.position
            dist = math.sqrt(
                (c.x - p.x) ** 2 + (c.y - p.y) ** 2 + (c.z - p.z) ** 2,
            )
            speed_m_s = waypoints[i].speed / 1000.0  # mm/s → m/s
            if speed_m_s > 0:
                total += dist / speed_m_s

        return total


# ------------------------------------------------------------------
# 엔트리포인트
# ------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = DrawingPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()