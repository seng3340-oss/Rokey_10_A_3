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

from .drawing_geometry import (
    CylinderCake,
    pen_quat_side,
    pen_quat_top,
    pixel_to_mm_side,
    side_drawing_rect,
    pixel_to_mm_top,
)


class DrawingPathNode(Node):
    """캔버스 픽셀 좌표 → 로봇 절대 좌표 3D 경로 변환 서비스 노드."""

    def __init__(self):
        super().__init__('drawing_path_node')

        # ── 변환 설정 파라미터 (yaml에서 로드) ──
        self.declare_parameter('drawing_speed', 20.0)         # mm/s
        self.declare_parameter('travel_speed', 80.0)          # mm/s (펜 올린 상태)
        self.declare_parameter('approach_height', 10.0)       # mm (펜 접근 높이)
        self.declare_parameter('drawing_z_offset', 0.0)       # mm (표면 접촉 보정)
        self.declare_parameter('default_contact_force', 0.5)  # N
        self.declare_parameter('pen_tilt_deg', 30.0)          # 펜 기울기 (도)
        # TOP: 수직에서 기울어지는 각도 (0=수직, 30=30도 기울임)
        # SIDE: 법선에서 아래로 기울어지는 각도

        self.srv = self.create_service(
            GenerateTrajectory,
            'generate_trajectory',
            self._handle_request,
        )
        self.get_logger().info('drawing_path_node 서비스 서버 시작 (pixel→cylinder)')

    def _param(self, name: str):
        return self.get_parameter(name).value

    # ==================================================================
    # 서비스 콜백
    # ==================================================================

    def _handle_request(self, request, response):
        self.get_logger().info(
            f'경로 변환 요청: order={request.order_id}, '
            f'surface={request.surface_type}, '
            f'strokes={len(request.strokes_2d)}, '
            f'speed={request.operation_speed_percent}%, '
            f'canvas=({request.canvas_width}x{request.canvas_height}), '
            f'radius={request.cake_radius}'
        )

        try:
            waypoints, pen_states = self._convert_strokes(request)

            if not waypoints:
                # 획이 없거나 점이 하나뿐인 입력은 웨이포인트가 하나도 안 나온다.
                # 성공으로 넘기면 실행 단계에서 '빈 웨이포인트' 오류로 원인이
                # 묻히므로 여기서 획 수를 밝혀 실패시킨다.
                raise ValueError(
                    f'변환할 웨이포인트가 없습니다. (strokes='
                    f'{len(request.strokes_2d)})'
                )

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
        """캔버스 픽셀 좌표 → 로봇 절대 좌표 Waypoint3D 변환.

        1. 윗면 중심(TCP) → 바닥 중심으로 변환
        2. CylinderCake 생성
        3. 요청에 담긴 모든 획을 순서대로 픽셀 → mm → 3D 절대 좌표로 변환

        입력은 캔버스 픽셀(왼쪽 위 = (0,0)) 원본 그대로 받으며,
        한 요청의 여러 획은 각각 접근-하강-그리기-이탈 구간을 포함해
        획 사이 이동이 자동으로 만들어진다. 반환값은 웨이포인트 목록과
        길이·값이 같은 펜 상태 목록이다.
        """
        # ── 윗면 중심 → 바닥 중심 변환 (m → mm) ──
        top_x = request.cake_pose.position.x * 1000.0
        top_y = request.cake_pose.position.y * 1000.0
        top_z = request.cake_pose.position.z * 1000.0
        cake_height_mm = request.cake_height * 1000.0
        cake_radius_mm = request.cake_radius * 1000.0

        bottom_z = top_z - cake_height_mm   # 바닥 중심 Z

        # ── CylinderCake 생성 (바닥 중심 기준, mm) ──
        cake = CylinderCake(
            radius=cake_radius_mm,
            height=cake_height_mm,
            cx=top_x,       # X, Y는 윗면·바닥 동일
            cy=top_y,
            cz=bottom_z,    # 바닥 중심 Z
        )

        # ── 캔버스·속도 파라미터 ──
        canvas_w = request.canvas_width
        canvas_h = request.canvas_height

        speed_ratio = max(1, min(100, request.operation_speed_percent)) / 100.0
        draw_spd = self._param('drawing_speed') * speed_ratio
        travel_spd = self._param('travel_speed') * speed_ratio
        approach_h = self._param('approach_height')
        z_offset = self._param('drawing_z_offset')
        force = self._param('default_contact_force')
        tilt_deg = self._param('pen_tilt_deg')

        # 면 종류는 TOP/SIDE 두 가지뿐이다. 알 수 없는 값을 받으면 SIDE로
        # 조용히 처리되면 옆면 경로가 대신 그려지므로 여기서 바로 실패시킨다.
        surface = request.surface_type.upper()
        if surface not in ('TOP', 'SIDE'):
            raise ValueError(
                f'surface_type은 TOP 또는 SIDE여야 합니다: {request.surface_type}'
            )

        waypoints: list[Waypoint3DMsg] = []

        # 요청에 담긴 모든 획을 순서대로 변환한다. 첫 획에서 멈추지 않도록
        # 그릴 수 없는 획만 건너뛰고 나머지는 전부 웨이포인트로 추가한다.
        for stroke in request.strokes_2d:
            pts_px = [(pt.x, pt.y) for pt in stroke.points]
            # 점 두 개 미만은 이동 경로가 없어 그릴 수 없다. 백엔드·웹도 같은
            # 규칙(2점 이상)으로 받으므로, 남으면 '찍고 지나가는' 동작이 되어
            # 로봇만 불필요하게 움직이므로 건너뛴다.
            if len(pts_px) < 2:
                continue

            if surface == 'TOP':
                self._convert_stroke_top(
                    pts_px, cake, canvas_w, canvas_h,
                    approach_h, z_offset, draw_spd, travel_spd, force,
                    tilt_deg, waypoints,
                )
            else:  # SIDE
                self._convert_stroke_side(
                    pts_px, cake, canvas_w, canvas_h,
                    approach_h, z_offset, draw_spd, travel_spd, force,
                    tilt_deg, waypoints,
                )

        # 응답 계약: pen_states는 waypoints_3d[i].pen_down과 길이·값이 같아야 한다.
        # 펜 상태는 이미 각 웨이포인트에 기록돼 있으므로 여기서 다시 뽑아
        # 두 표현이 서로 어긋나는 상황을 만들지 않는다.
        pen_states = [wp.pen_down for wp in waypoints]

        return waypoints, pen_states

    # ------------------------------------------------------------------
    # TOP 면 변환
    # ------------------------------------------------------------------

    def _convert_stroke_top(
        self, pts_px, cake: CylinderCake,
        canvas_w, canvas_h,
        approach_h, z_offset, draw_spd, travel_spd, force,
        tilt_deg, waypoints,
    ):
        """TOP 면 한 획분의 웨이포인트를 추가한다.

        접근(펜 들기) → 하강(펜 내림) → 이어 그린 점들 → 이탈(펜 들기) 순서로
        추가되므로 여러 획이 연결될 때도 획 사이에 펜이 반드시 올라간다.
        """
        quat = pen_quat_top(tilt_deg=tilt_deg)

        # 웹의 가운데 점선 원은 1024×768 캔버스에서 지름 640px다.
        # 입력 픽셀은 보존하고 이 단계에서만 케이크 반지름에 맞춰 환산한다.
        diameter_px = min(canvas_w, canvas_h) * (640 / 768)

        def to_top(pixel):
            """점선 원 안의 픽셀을 BASE 기준 윗면 mm 좌표로 옮긴다."""
            x_mm, y_mm = pixel_to_mm_top(
                *pixel, canvas_w, canvas_h, cake.radius,
                drawing_diameter_px=diameter_px,
            )
            if math.hypot(x_mm, y_mm) > cake.radius + 1e-6:
                raise ValueError("획에 케이크 윗면을 벗어난 점이 있습니다.")
            return cake.top_to_abs(x_mm, y_mm)

        fx, fy, fz = to_top(pts_px[0])
        waypoints.append(self._wp(
            fx, fy, fz + approach_h, quat, travel_spd, force, pen_down=False,
        ))
        waypoints.append(self._wp(
            fx, fy, fz + z_offset, quat, draw_spd * 0.5, force, pen_down=True,
        ))
        for pixel in pts_px[1:]:
            x, y, z = to_top(pixel)
            waypoints.append(self._wp(
                x, y, z + z_offset, quat, draw_spd, force, pen_down=True,
            ))
        lx, ly, lz = to_top(pts_px[-1])
        waypoints.append(self._wp(
            lx, ly, lz + approach_h, quat, travel_spd, force, pen_down=False,
        ))

    # ------------------------------------------------------------------
    # SIDE 면 변환
    # ------------------------------------------------------------------

    def _convert_stroke_side(
        self, pts_px, cake: CylinderCake,
        canvas_w, canvas_h,
        approach_h, z_offset, draw_spd, travel_spd, force,
        tilt_deg, waypoints,
    ):
        """SIDE 면 한 획분의 웨이포인트를 추가한다.

        캔버스 픽셀 → mm(호, 높이) → 원통 표면 3D로 변환하며,
        TOP과 같이 접근-하강-그리기-이탈 순서로 펜 상태를 지정한다.
        """

        # ── 1. 첫 점 법선 방향 접근 (펜 들기) ──
        arc0, h0 = pixel_to_mm_side(
            pts_px[0][0], pts_px[0][1],
            canvas_w, canvas_h, cake.radius, cake.height,
            drawing_rect=side_drawing_rect(canvas_w, canvas_h, cake.radius, cake.height),
        )
        quat0 = pen_quat_side(arc0, cake, tilt_deg=tilt_deg)
        ax0, ay0, az0 = cake.side_abs_with_offset(arc0, h0, approach_h)
        waypoints.append(
            self._wp(ax0, ay0, az0, quat0, travel_spd, force, pen_down=False),
        )

        # ── 2. 펜 내리기 ──
        cx0, cy0, cz0 = cake.side_abs_with_offset(arc0, h0, z_offset)
        waypoints.append(
            self._wp(cx0, cy0, cz0, quat0, draw_spd * 0.5, force, pen_down=True),
        )

        # ── 3. 획 그리기 ──
        for px_x, px_y in pts_px[1:]:
            arc_i, h_i = pixel_to_mm_side(
                px_x, px_y, canvas_w, canvas_h, cake.radius, cake.height,
                drawing_rect=side_drawing_rect(canvas_w, canvas_h, cake.radius, cake.height),
            )
            qi = pen_quat_side(arc_i, cake, tilt_deg=tilt_deg)
            cxi, cyi, czi = cake.side_abs_with_offset(arc_i, h_i, z_offset)
            waypoints.append(
                self._wp(cxi, cyi, czi, qi, draw_spd, force, pen_down=True),
            )

        # ── 4. 펜 올리기 ──
        arcn, hn = pixel_to_mm_side(
            pts_px[-1][0], pts_px[-1][1],
            canvas_w, canvas_h, cake.radius, cake.height,
            drawing_rect=side_drawing_rect(canvas_w, canvas_h, cake.radius, cake.height),
        )
        quatn = pen_quat_side(arcn, cake, tilt_deg=tilt_deg)
        axn, ayn, azn = cake.side_abs_with_offset(arcn, hn, approach_h)
        waypoints.append(
            self._wp(axn, ayn, azn, quatn, travel_spd, force, pen_down=False),
        )

    # ------------------------------------------------------------------
    # Waypoint 생성
    # ------------------------------------------------------------------

    @staticmethod
    def _wp(
        x_mm: float, y_mm: float, z_mm: float,
        quat: tuple[float, float, float, float],
        speed: float,
        force: float,
        pen_down: bool,
    ) -> Waypoint3DMsg:
        """Waypoint3D 메시지 생성. mm → m 변환 포함.

        pen_down은 이 지점에서 펜이 케이크에 닿아 있는지를 나타낸다.
        실행 노드(robot_executor)는 이 값만 읽어 그리기/이동을 구분하므로
        생략할 수 없고, 응답 pen_states와 반드시 같은 값이어야 한다.
        """
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
