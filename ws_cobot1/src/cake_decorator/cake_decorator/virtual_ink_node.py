"""관측 virtual_pen_tip TF로 추정한 접촉만 RViz에 빨간 잉크로 표시한다.

실제 물리 접촉 센서가 아닌 TF 표면거리 모델이다. 로봇 이동, 서비스 제어,
백엔드 또는 HMI 연결은 수행하지 않는다. setup/launch 등록 없이도 다음처럼
실행할 수 있다: python3 -m cake_decorator.virtual_ink_node

/cake/ink_reference는 base_link 기준의 회전하지 않은 케이크 top center다.
위치만 사용하며 새 기준 수신 시 잉크·파형 통계를 초기화한다.
"""

import json

from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from cake_decorator.contact_geometry import CakeSurface, ContactTracker


class VirtualInkNode(Node):
    """base_link→virtual_pen_tip 최신 TF를 읽기만 하는 표시 전용 노드."""

    def __init__(self):
        """설정·TF listener·표시 토픽을 준비하고 관측/출력 타이머를 시작한다."""
        super().__init__('virtual_ink_node')
        defaults = {
            'center_x': 0.5, 'center_y': 0.0, 'top_z': 0.142,
            'cake_radius': 0.05, 'cake_height': 0.08,
            'contact_tolerance_m': 0.0005, 'max_tf_age_sec': 0.25,
            'max_points': 100000, 'max_link_distance_m': 0.01,
        }
        values = {
            name: self.declare_parameter(name, default).value
            for name, default in defaults.items()
        }
        cake = CakeSurface(
            center_x=values['center_x'], center_y=values['center_y'],
            top_z=values['top_z'], radius=values['cake_radius'],
            height=values['cake_height'],
            contact_tolerance_m=values['contact_tolerance_m'],
        )
        self.tracker = ContactTracker(
            cake=cake, max_tf_age_sec=values['max_tf_age_sec'],
            max_points=values['max_points'],
            max_link_distance_m=values['max_link_distance_m'],
        )
        self._buffer = Buffer(node=self)
        self._listener = TransformListener(self._buffer, self)
        # 늦게 열린 RViz도 마지막 전체 잉크를 받도록 내구성을 지정한다.
        ink_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._ink_pub = self.create_publisher(MarkerArray, '/cake/ink', ink_qos)
        self._telemetry_pub = self.create_publisher(String, '/cake/ink_telemetry', 10)
        self._tip_pub = self.create_publisher(PoseStamped, '/cake/observed_tip', 10)
        self._clear_sub = self.create_subscription(
            Empty, '/cake/ink_clear', self._clear, 10,
        )
        self._reference_sub = self.create_subscription(
            PoseStamped, '/cake/ink_reference', self._reference, 10,
        )
        self._dirty = True
        self._generation = 0
        # TF는 50Hz로 읽고 무거운 전체 MarkerArray는 최대 10Hz로 합쳐 발행한다.
        self._observe_timer = self.create_timer(0.02, self._observe)
        self._publish_timer = self.create_timer(0.1, self._publish)
        self.get_logger().info(
            '가상 잉크: 실제 물리 접촉 센서가 아닌 TF 표면거리 모델입니다. '
            'base_link → virtual_pen_tip 관측/표시만 수행합니다.'
        )

    def _clear(self, _message):
        """파형 시작용 Empty 수신 시 잉크와 통계를 지우고 빈 마커를 즉시 발행한다."""
        self.tracker.clear()
        self._generation += 1
        self._dirty = True
        self._publish()

    def _reference(self, message):
        """base_link의 유한한 top center만 적용하며 잘못된 기준은 그대로 거부한다."""
        if message.header.frame_id != 'base_link':
            self.get_logger().warning('ink_reference frame_id는 base_link여야 합니다.')
            return
        position = message.pose.position
        try:
            self.tracker.set_reference((position.x, position.y, position.z))
        except ValueError as error:
            self.get_logger().warning(f'ink_reference 거부: {error}')
            return
        self._generation += 1
        self._dirty = True
        self._publish()

    def _observe(self):
        """Time(0) 최신 TF의 원래 stamp를 검증하고 새 관측 위치만 발행한다."""
        try:
            transform = self._buffer.lookup_transform(
                'base_link', 'virtual_pen_tip', Time(),
            )
        except TransformException:
            self.tracker.missing()
            return
        stamp = transform.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        translation = transform.transform.translation
        observed_before = self.tracker.observed_count
        stored_before = self.tracker.stored_point_count
        self.tracker.observe(
            (translation.x, translation.y, translation.z), stamp_sec, now,
        )
        if self.tracker.observed_count != observed_before:
            # 표시 투영점이 아니라 원래 TF 위치와 회전을 그대로 내보낸다.
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.header.stamp = stamp
            pose.pose.position = Point(
                x=translation.x, y=translation.y, z=translation.z,
            )
            pose.pose.orientation = transform.transform.rotation
            self._tip_pub.publish(pose)
        if self.tracker.stored_point_count != stored_before:
            self._dirty = True

    def _marker(self, marker_type, identifier, stamp):
        """고정 namespace/id의 빨간 마커를 만들어 다음 발행 시 전체를 교체한다."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = stamp
        marker.ns = 'virtual_contact_ink'
        marker.id = identifier
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.a = 1.0
        # 접촉 범위가 과장되지 않도록 선폭은 0.6mm, 점 지름은 0.8mm로 둔다.
        size = 0.0006 if marker_type == Marker.LINE_LIST else 0.0008
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        return marker

    def _publish(self):
        """변경된 전체 마커와 파형별 관측 통계를 최대 10Hz로 발행한다."""
        stamp = self.get_clock().now().to_msg()
        if self._dirty:
            lines = self._marker(Marker.LINE_LIST, 0, stamp)
            dots = self._marker(Marker.SPHERE_LIST, 1, stamp)
            lines.points = [
                Point(x=point[0], y=point[1], z=point[2])
                for segment in self.tracker.segments() for point in segment
            ]
            # 선으로 연결되지 않은 단독 접촉도 남도록 각 접촉점에 점을 그린다.
            dots.points = [
                Point(x=sample.contact.rendered[0],
                      y=sample.contact.rendered[1],
                      z=sample.contact.rendered[2])
                for sample in self.tracker.points
            ]
            # 빈 형상의 해석에 의존하지 않고 이 노드 소유의 id만 명시적으로 지운다.
            for marker in (lines, dots):
                if not marker.points:
                    marker.action = Marker.DELETE
            self._ink_pub.publish(MarkerArray(markers=[lines, dots]))
            self._dirty = False
        telemetry = self.tracker.telemetry()
        telemetry.update({
            'frame_id': 'base_link', 'tip_frame': 'virtual_pen_tip',
            'generation': self._generation,
            'published_at_sec': stamp.sec + stamp.nanosec * 1e-9,
        })
        self._telemetry_pub.publish(String(data=json.dumps(telemetry, allow_nan=False)))


def main(args=None):
    """ROS를 초기화해 표시 노드를 실행하고 종료 시 자원을 해제한다."""
    rclpy.init(args=args)
    node = None
    try:
        node = VirtualInkNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
