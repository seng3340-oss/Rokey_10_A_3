"""가상 로봇 대상 2D→3D 변환 + 드로잉 실행 데모.

동작 순서:
  1. GenerateTrajectory 서비스(drawing_path_node)로 캔버스 픽셀 스트로크를
     원통형 케이크 3D 웨이포인트로 변환한다. 여러 획을 한 요청에 담아
     획 사이 펜 들기/내림이 자동으로 만들어지는지 확인한다.
  2. 돌려받은 웨이포인트로 DrawOnCake 액션(robot_executor_node)을 전송한다.
  3. 실행 노드는 웨이포인트를 MoveLine 명령으로 바꿔 가상 로봇(/dsr01/dsr_controller2/motion/move_line)
     에 보내므로, RViz에서 로봇 팔이 획을 따라가는 것을 확인할 수 있다.

좌표 규약:
  - 입력은 캔버스 픽셀이며 왼쪽 위가 (0,0)이다(원본 좌표 그대로 전달).
  - 윗면 원 중심=(512, 384), 캔버스는 1024×768px.
  - cake_pose는 TCP로 기록한 윗면 중심이다. 바닥 중심이 아니다.

입력 치수와 케이크 위치는 아래 상수로 바꾼다. 치수 단위는 미터(m)다.
실행 예시(가상 로봇 launch 후):
  python3 scripts/virtual_draw_demo.py
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cake_decorator_interfaces.action import DrawOnCake
from cake_decorator_interfaces.msg import Stroke2D, Waypoint3D
from cake_decorator_interfaces.srv import GenerateTrajectory
from geometry_msgs.msg import Point, Point32, Pose, Quaternion

# ── 캔버스 (픽셀, 왼쪽 위 = (0,0)) ──
# 프론트엔드 원본 해상도와 같다. 윗면 원의 지름은 640px(반지름 320px)다.
CANVAS_W = 1024
CANVAS_H = 768
CANVAS_CX = 512.0
CANVAS_CY = 384.0
# 픽셀→mm 배율: 윗면 반지름 50mm가 원 반지름 320px에 대응한다.
# drawing_path_node의 pixel_to_mm_top은 캔버스 높이(768px)를 지름(100mm)에
# 맞추므로 여기서도 같은 배율을 써야 데모 좌표와 실제 변환이 일치한다.
# 단위는 mm/px다.
PX_TO_MM = (2.0 * 50.0) / CANVAS_H

# ── 케이크 치수 (m) ──
# GenerateTrajectory 요청은 m 단위이고, drawing_path_node 내부에서 mm로 바꾼다.
# 치수는 가상 작업셀의 test_cake.stl(지름 100mm·높이 80mm)에 맞춘다.
CAKE_RADIUS = 0.05      # 원통 반지름 50mm
CAKE_HEIGHT = 0.08      # 높이 80mm

# 케이크 윗면 중심의 BASE 절대 좌표 (m).
# 윗면 중심을 기록한 값이며 높이를 다시 더하지 않는다.
# cake_workcell_bringup/config/workcell.yaml의 fixture(415, -85, 0)에
# 케이크 메시 원점 오프셋(85, 85, 62)mm를 더하면 바닥 중심 (0.5, 0, 0.062),
# 여기에 원통 높이 80mm를 더한 윗면이 z=0.142m이다. 펜촉이 면에 닿게 하려면
# 이 값을 써야 한다(0.11이면 판 안쪽 32mm 아래로 그린다).
CAKE_POSE = Pose(
    position=Point(x=0.50, y=0.00, z=0.142),
    # 케이크는 로봇 베이스와 축을 맞춘 수평 배치를 가정한다.
    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
)

# 작업 속도 퍼센트. 100이면 drawing_path_node 기본 속도(드로잉 20mm/s, 이동 50mm/s).
OPERATION_SPEED_PERCENT = 100


def _mm_to_px(x_mm: float, y_mm: float) -> tuple[float, float]:
    """윗면 중심 기준 mm 오프셋 → 캔버스 픽셀.

    화면 y는 아래로 증가하고 BASE y는 위로 증가하므로 y를 반전한다.
    """
    px = CANVAS_CX + x_mm / PX_TO_MM
    py = CANVAS_CY - y_mm / PX_TO_MM
    return (px, py)


def _make_stroke(stroke_id: int, points_mm: list[tuple[float, float]]) -> Stroke2D:
    """(x, y) mm 오프셋 목록을 픽셀 획(Stroke2D) 메시지로 만든다.

    점 순서가 곧 그리는 순서이며, z는 사용하지 않아 항상 0이다.
    """
    stroke = Stroke2D(stroke_id=stroke_id)
    for x_mm, y_mm in points_mm:
        px, py = _mm_to_px(x_mm, y_mm)
        stroke.points.append(Point32(x=px, y=py, z=0.0))
    return stroke


def _letter_r_strokes() -> list[Stroke2D]:
    """윗면에 그릴 'R' 글자 3획 (단위 mm, 윗면 중심 기준).

    획 사이 이동은 drawing_path_node가 자동으로 펜 들기 처리하므로
    3개의 획으로 나눠 전달한다. 글자 크기는 지름 100mm 윗면 안에 들어오게 잡았다.
    """
    return [
        # 세로 기둥: 아래에서 위로
        _make_stroke(0, [
            (-18.0, -18.0),
            (-18.0, 18.0),
        ]),
        # 윗돌기(오른쪽 위 활): 위에서 시작해 시계 방향으로 돌아 내려온다
        _make_stroke(1, [
            (-18.0, 18.0),
            (6.0, 18.0),
            (14.0, 12.0),
            (14.0, 4.0),
            (6.0, -2.0),
            (-18.0, -2.0),
        ]),
        # 오른쪽 다리: 중간에서 오른쪽 아래로
        _make_stroke(2, [
            (-2.0, -2.0),
            (16.0, -18.0),
        ]),
    ]


class VirtualDrawDemo(Node):
    """변환 서비스 호출 + 드로잉 액션 전송을 순서대로 수행하는 데모 노드."""

    def __init__(self):
        super().__init__('virtual_draw_demo')

        # 2D→3D 변환 서비스 클라이언트
        self._path_client = self.create_client(
            GenerateTrajectory,
            'generate_trajectory',
        )
        # 로봇 실행 액션 클라이언트
        self._draw_client = ActionClient(
            self,
            DrawOnCake,
            'draw_on_cake',
        )

    # ------------------------------------------------------------------
    # 1단계: 2D → 3D 변환
    # ------------------------------------------------------------------

    def convert(self) -> list[Waypoint3D]:
        """여러 획의 캔버스 픽셀을 3D 웨이포인트로 변환한다.

        Returns
        -------
        drawing_path_node가 돌려준 Waypoint3D 목록 (m, 쿼터니언).
        웨이포인트의 pen_down에 펜 상태가 그대로 실려 있다.
        실패하면 예외를 던진다.
        """
        if not self._path_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('generate_trajectory 서비스를 찾을 수 없습니다.')

        strokes = _letter_r_strokes()
        request = GenerateTrajectory.Request()
        request.order_id = 'virtual-draw-demo'
        request.operation_speed_percent = OPERATION_SPEED_PERCENT
        request.surface_type = 'TOP'
        request.cake_pose = CAKE_POSE
        request.cake_radius = CAKE_RADIUS
        # 이 데모는 기존 원통 검증이므로 두 반지름을 같게 지정한다.
        request.cake_bottom_radius = CAKE_RADIUS
        request.cake_height = CAKE_HEIGHT
        request.canvas_width = float(CANVAS_W)
        request.canvas_height = float(CANVAS_H)
        # 획 전체를 한 요청에 담는다. 첫 획만 넘기면 첫 획만 그려진다.
        request.strokes_2d = strokes

        future = self._path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()

        if response is None:
            raise RuntimeError('generate_trajectory 응답이 없습니다.')
        if not response.success:
            raise RuntimeError(f'경로 변환 실패: {response.error_msg}')

        # 펜이 내려간 구간 수가 요청한 획 수와 같은지 먼저 확인한다.
        stroke_count = sum(
            1
            for i, pen in enumerate(response.pen_states)
            if pen and (i == 0 or not response.pen_states[i - 1])
        )
        self.get_logger().info(
            f'변환 완료: 요청 획={len(strokes)}, 획 구간={stroke_count}, '
            f'waypoints={len(response.waypoints_3d)}, '
            f'예상시간={response.estimated_duration_sec:.1f}s'
        )
        if stroke_count != len(strokes):
            raise RuntimeError(
                f'획 {len(strokes)}개를 요청했지만 {stroke_count}개만 만들어졌다.'
            )

        # 앞뒤 좌표 몇 개만 출력해 실제 3D 값을 눈으로 확인한다.
        wps = list(response.waypoints_3d)
        for idx in sorted({0, len(wps) // 2, len(wps) - 1}):
            p = wps[idx].pose.position
            self.get_logger().info(
                f'  waypoint[{idx}] = ({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) m '
                f'pen_down={wps[idx].pen_down}'
            )
        return wps

    # ------------------------------------------------------------------
    # 2단계: 드로잉 액션 전송
    # ------------------------------------------------------------------

    def draw(self, waypoints: list[Waypoint3D]) -> bool:
        """DrawOnCake 액션을 전송하고 완료를 기다린다.

        Returns
        -------
        액션이 성공으로 끝나면 True.
        """
        if not self._draw_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('draw_on_cake 액션 서버를 찾을 수 없습니다.')

        goal = DrawOnCake.Goal()
        goal.waypoints = waypoints
        goal.drawing_surface = 'TOP'
        goal.contact_force = 0.5
        goal.speed = 20.0

        self.get_logger().info(f'DrawOnCake 목표 전송: waypoints={len(waypoints)}')

        future = self._draw_client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('DrawOnCake 목표가 거절되었습니다.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)
        result = result_future.result()

        if result is None:
            raise RuntimeError('DrawOnCake 결과가 없습니다.')

        status = result.status
        res = result.result
        self.get_logger().info(
            f'액션 종료: status={status}, success={res.success}, '
            f'completed_strokes={res.completed_strokes}, '
            f'error_code={res.error_code}'
        )
        return bool(res.success)

    def _on_feedback(self, feedback_msg):
        """실행 진행 피드백을 로그로 남긴다."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  진행: {fb.progress_pct:5.1f}% '
            f'(획 {fb.current_stroke_idx}/{fb.total_strokes}, {fb.phase})'
        )


def main() -> int:
    """엔트리포인트. 변환 → 실행 순서로 수행하고 종료 코드를 반환한다."""
    rclpy.init()
    node = VirtualDrawDemo()
    try:
        waypoints = node.convert()
        ok = node.draw(waypoints)
        return 0 if ok else 1
    except Exception as exc:  # noqa: BLE001 — 데모 스크립트는 모든 실패를 보고하고 끝낸다
        node.get_logger().error(f'데모 실패: {exc}')
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
