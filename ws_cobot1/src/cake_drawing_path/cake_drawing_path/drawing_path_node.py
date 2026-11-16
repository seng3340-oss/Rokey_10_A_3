"""GenerateTrajectory 서비스 서버 노드.

TM(task_manager)이 요청하면 2D 스트로크를 3D 경로로 변환해 응답한다.
실제 로봇 이동은 수행하지 않는다.
"""

from cake_decorator_interfaces.msg import Waypoint3D as Waypoint3DMsg
from cake_decorator_interfaces.srv import GenerateTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
import rclpy
from rclpy.node import Node

from .converter import (
    CakePose,
    ConvertConfig,
    Point2D,
    Stroke2D,
    generate_trajectory,
)


class DrawingPathNode(Node):
    """2D → 3D 경로 변환 서비스 노드."""

    def __init__(self):
        super().__init__('drawing_path_node')

        # 파라미터 선언
        self.declare_parameter('drawing_speed', 20.0)
        self.declare_parameter('travel_speed', 50.0)
        self.declare_parameter('approach_height', 0.01)
        self.declare_parameter('drawing_z_offset', 0.0)

        self.declare_parameter('side_margin_ratio', 0.05)

        # 서비스 서버 등록
        self.srv = self.create_service(
            GenerateTrajectory,
            'generate_trajectory',
            self.handle_generate_trajectory,
        )
        self.get_logger().info('drawing_path_node 서비스 서버 시작')

    def _load_config(self) -> ConvertConfig:
        """매 요청마다 파라미터를 읽어 동적 변경을 반영한다."""
        return ConvertConfig(
            drawing_speed=self.get_parameter('drawing_speed').value,
            travel_speed=self.get_parameter('travel_speed').value,
            approach_height=self.get_parameter('approach_height').value,
            drawing_z_offset=self.get_parameter('drawing_z_offset').value,
            side_margin_ratio=self.get_parameter('side_margin_ratio').value,
        )

    def handle_generate_trajectory(self, request, response):
        """GenerateTrajectory 서비스 콜백."""
        self.get_logger().info(
            f'경로 변환 요청: surface={request.surface_type}, '
            f'strokes={len(request.strokes_2d)}, '
            f'canvas={request.canvas_width}x{request.canvas_height}'
        )

        # ROS 메시지 → 순수 Python 데이터 변환
        strokes = []
        for s in request.strokes_2d:
            points = [Point2D(x=p.x, y=p.y) for p in s.points]

            strokes.append(Stroke2D(
                stroke_id=s.stroke_id,
                points=points,
                is_closed=s.is_closed,
            ))

        cake_pose = CakePose(
            x=request.cake_pose.position.x,
            y=request.cake_pose.position.y,
            z=request.cake_pose.position.z,
        )

        config = self._load_config()

        # 변환 실행
        result = generate_trajectory(
            canvas_width=request.canvas_width,
            canvas_height=request.canvas_height,
            strokes=strokes,
            surface_type=request.surface_type,
            cake=cake_pose,
            cake_radius=request.cake_radius,
            cake_height=request.cake_height,
            config=config,
        )

        # 순수 Python 결과 → ROS 메시지 변환
        response.success = result.success
        response.error_msg = result.error_msg
        response.estimated_duration_sec = result.estimated_duration_sec

        for wp in result.waypoints:

            msg = Waypoint3DMsg()
            msg.pose = Pose(
                position=Point(
                    x=wp.pose.x,
                    y=wp.pose.y,
                    z=wp.pose.z,
                ),
                orientation=Quaternion(
                    x=wp.pose.qx,
                    y=wp.pose.qy,
                    z=wp.pose.qz,
                    w=wp.pose.qw,
                ),
            )
            msg.speed = wp.speed
            msg.pen_down = wp.pen_down
            msg.contact_force = wp.contact_force
            response.waypoints_3d.append(msg)

        response.pen_states = result.pen_states

        if result.success:
            self.get_logger().info(
                f'경로 변환 완료: waypoints={len(result.waypoints)}, '
                f'예상 시간={result.estimated_duration_sec:.1f}s'
            )
        else:
            self.get_logger().warn(f'경로 변환 실패: {result.error_msg}')

        return response



def main(args=None):
    """노드 엔트리포인트."""
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