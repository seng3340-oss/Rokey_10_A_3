"""원통형 케이크의 윗면에 R을 그리는 로컬 가상 ROS 데모."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from geometry_msgs.msg import Point, Point32, Pose, Quaternion
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cake_decorator_interfaces.action import DrawOnCake
from cake_decorator_interfaces.msg import Stroke2D
from cake_decorator_interfaces.srv import GenerateTrajectory


CANVAS_WIDTH = 1024
CANVAS_HEIGHT = 768
TOP_DIAMETER_PX = 640
CAKE_TOP_CENTER = Point(x=0.50, y=0.00, z=0.142)
DRAWING_SPEED = 20.0

# 케이크 중심(오른쪽 +X, 위쪽 +Y)을 기준으로 한 R의 세 획, 단위 mm.
LETTER_R = (
    ((-18.0, -18.0), (-18.0, 18.0)),
    ((-18.0, 18.0), (6.0, 18.0), (14.0, 12.0), (14.0, 4.0),
     (6.0, -2.0), (-18.0, -2.0)),
    ((-2.0, -2.0), (16.0, -18.0)),
)


def _letter_r_strokes(diameter_mm: float) -> list[Stroke2D]:
    """R의 mm 오프셋을 웹과 동일한 원본 캔버스 픽셀로 바꾼다."""
    pixels_per_mm = TOP_DIAMETER_PX / diameter_mm
    return [
        Stroke2D(
            stroke_id=stroke_id,
            points=[
                Point32(
                    x=CANVAS_WIDTH / 2 + x_mm * pixels_per_mm,
                    y=CANVAS_HEIGHT / 2 - y_mm * pixels_per_mm,
                    z=0.0,
                )
                for x_mm, y_mm in stroke
            ],
        )
        for stroke_id, stroke in enumerate(LETTER_R)
    ]


def _await(node, future, timeout, operation):
    """독립 데모에서 요청을 처리하고 응답이 없으면 다음 단계로 넘어가지 않는다."""
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"{operation} 응답이 없습니다.")
    return future.result()


def main() -> int:
    """로컬 가상 도메인에서 변환 서비스와 드로잉 액션을 순서대로 호출한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=97)
    args, ros_args = parser.parse_known_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    for key in ("ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT"):
        os.environ.pop(key, None)

    config_path = Path(__file__).resolve().parents[1] / "config/test_cake.json"
    dimensions = json.loads(config_path.read_text())
    diameter_mm = dimensions["top_diameter_mm"]
    request = GenerateTrajectory.Request(
        order_id="virtual-draw-demo",
        operation_speed_percent=100,
        surface_type="TOP",
        cake_pose=Pose(position=CAKE_TOP_CENTER, orientation=Quaternion(w=1.0)),
        cake_radius=diameter_mm / 2000,
        cake_bottom_radius=dimensions["bottom_diameter_mm"] / 2000,
        cake_height=dimensions["height_mm"] / 1000,
        canvas_width=float(CANVAS_WIDTH),
        canvas_height=float(CANVAS_HEIGHT),
        strokes_2d=_letter_r_strokes(diameter_mm),
    )

    rclpy.init(args=ros_args)
    node = Node("virtual_draw_demo")
    # 전용 이름을 사용해 일반 드로잉 서버를 대상으로 실행하는 일을 피한다.
    path_client = node.create_client(GenerateTrajectory, "/cake_sim/generate_trajectory")
    draw_client = ActionClient(node, DrawOnCake, "/cake_sim/draw_on_cake")
    try:
        if not path_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("가상 generate_trajectory 서비스를 찾을 수 없습니다.")
        response = _await(node, path_client.call_async(request), 30.0, "경로 변환")
        if not response.success:
            raise RuntimeError(f"경로 변환 실패: {response.error_msg}")
        node.get_logger().info(f"변환 완료: {len(response.waypoints_3d)}개 웨이포인트")

        if not draw_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("가상 draw_on_cake 액션 서버를 찾을 수 없습니다.")
        goal = DrawOnCake.Goal(
            waypoints=response.waypoints_3d,
            drawing_surface="TOP",
            contact_force=0.5,
            speed=DRAWING_SPEED,
        )
        handle = _await(node, draw_client.send_goal_async(goal), 30.0, "가상 드로잉 접수")
        if not handle.accepted:
            raise RuntimeError("가상 드로잉이 거절되었습니다.")
        result = _await(node, handle.get_result_async(), 300.0, "가상 드로잉").result
        if not result.success:
            raise RuntimeError(f"가상 드로잉 실패: {result.error_code}")
        node.get_logger().info(f"가상 드로잉 완료: {result.completed_strokes}획")
        return 0
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        draw_client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
