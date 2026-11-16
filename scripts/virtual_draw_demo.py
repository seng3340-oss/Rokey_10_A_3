"""가상 로봇 대상 2D→3D 변환 + 드로잉 실행 데모.

동작 순서:
  1. GenerateTrajectory 서비스(drawing_path_node)로 2D 스트로크를 원뿔대 표면
     3D 웨이포인트로 변환한다. 윗면 반지름·아랫면 반지름·높이를 이 요청에 담는다.
  2. 돌려받은 웨이포인트로 DrawOnCake 액션(robot_executor_node)을 전송한다.
  3. 실행 노드는 웨이포인트를 MoveLine 명령으로 바꿔 가상 로봇(/dsr01/motion/move_line)
     에 보내므로, RViz에서 로봇 팔이 획을 따라가는 것을 확인할 수 있다.

입력 치수와 케이크 위치는 아래 상수로 바꾼다. 단위는 모두 미터(m)다.
실행 예시(가상 로봇 launch 후):
  python3 scripts/virtual_draw_demo.py
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cake_decorator_interfaces.action import DrawOnCake
from cake_decorator_interfaces.msg import PointMM, PointMM3D, StrokeMM, Waypoint3D
from cake_decorator_interfaces.srv import GenerateTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion

# ── 케이크 치수 (m) ──
# GenerateTrajectory 요청은 m 단위이고, drawing_path_node 내부에서 mm로 바꾼다.
CAKE_RADIUS_TOP = 0.05      # 윗면 반지름 50mm
CAKE_RADIUS_BOTTOM = 0.06   # 아랫면 반지름 60mm
CAKE_HEIGHT = 0.04          # 높이 40mm

# 케이크 바닥면 중심의 BASE 절대 좌표 (m).
# z는 가상 작업대 높이를 임시로 잡은 값이며, 윗면은 z + 높이가 된다.
CAKE_POSE = Pose(
    position=Point(x=0.50, y=0.00, z=0.11),
    # 케이크는 로봇 베이스와 축을 맞춘 수평 배치를 가정한다.
    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
)

# 작업 속도 퍼센트. 100이면 drawing_path_node 기본 속도(드로잉 20mm/s, 이동 80mm/s).
OPERATION_SPEED_PERCENT = 100


def _make_stroke(stroke_id: str, points: list[tuple[float, float]]) -> StrokeMM:
    """(x, y) mm 좌표 목록을 획(StrokeMM) 메시지로 만든다.

    TOP 면에서 (x, y)는 윗면 중심 기준 오프셋 mm다.
    target_mm·simplified_mm·original_mm에 같은 값을 넣어 어느 필드를 읽어도
    동일한 경로가 나오도록 한다.
    """
    stroke = StrokeMM()
    stroke.stroke_id = stroke_id
    stroke.color = 'black'
    for x, y in points:
        pt = PointMM(x=x, y=y)
        stroke.original_mm.append(pt)
        stroke.simplified_mm.append(pt)
        stroke.target_mm.append(PointMM3D(x=x, y=y, z=0.0))
    return stroke


def _letter_r_strokes() -> list[StrokeMM]:
    """윗면에 그릴 'R' 글자 3획 (단위 mm, 윗면 중심 기준).

    획 사이 이동은 drawing_path_node가 자동으로 펜 들기 처리하므로
    3개의 획으로 나눠 전달한다. 글자 크기는 지름 100mm 윗면 안에 들어오게 잡았다.
    """
    return [
        # 세로 기둥: 아래에서 위로
        _make_stroke('r-stem', [
            (-18.0, -18.0),
            (-18.0, 18.0),
        ]),
        # 윗돌기(오른쪽 위 활): 위에서 시작해 시계 방향으로 돌아 내려온다
        _make_stroke('r-bowl', [
            (-18.0, 18.0),
            (6.0, 18.0),
            (14.0, 12.0),
            (14.0, 4.0),
            (6.0, -2.0),
            (-18.0, -2.0),
        ]),
        # 오른쪽 다리: 중간에서 오른쪽 아래로
        _make_stroke('r-leg', [
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
        """2D 스트로크를 3D 웨이포인트로 변환한다.

        Returns
        -------
        drawing_path_node가 돌려준 Waypoint3D 목록 (m, 쿼터니언).
        실패하면 예외를 던진다.
        """
        if not self._path_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('generate_trajectory 서비스를 찾을 수 없습니다.')

        request = GenerateTrajectory.Request()
        request.order_id = 'virtual-draw-demo'
        request.operation_speed_percent = OPERATION_SPEED_PERCENT
        request.surface_type = 'TOP'
        request.cake_pose = CAKE_POSE
        request.cake_radius_top = CAKE_RADIUS_TOP
        request.cake_radius_bottom = CAKE_RADIUS_BOTTOM
        request.cake_height = CAKE_HEIGHT
        request.strokes = _letter_r_strokes()

        future = self._path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()

        if response is None:
            raise RuntimeError('generate_trajectory 응답이 없습니다.')
        if not response.success:
            raise RuntimeError(f'경로 변환 실패: {response.error_msg}')

        self.get_logger().info(
            f'변환 완료: waypoints={len(response.waypoints_3d)}, '
            f'pen_states={list(response.pen_states)}, '
            f'예상시간={response.estimated_duration_sec:.1f}s'
        )

        # 앞뒤 좌표 몇 개만 출력해 실제 3D 값을 눈으로 확인한다.
        wps = list(response.waypoints_3d)
        for idx in sorted({0, len(wps) // 2, len(wps) - 1}):
            p = wps[idx].pose.position
            self.get_logger().info(
                f'  waypoint[{idx}] = ({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) m'
            )

        # 액션 목표에는 응답의 pen_states를 그대로 실어 보낸다.
        # (DrawOnCake 목표에는 펜 상태 배열이 따로 없으므로 waypoint에 복사)
        for wp, pen in zip(wps, response.pen_states):
            wp.pen_down = bool(pen)
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
