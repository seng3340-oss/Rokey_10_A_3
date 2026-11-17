"""ROS 없이 TF 표면거리 모델의 접촉·연결·메모리 계약을 검사한다."""

import json
import math
import time

import pytest

from cake_decorator.contact_geometry import CakeSurface, ContactTracker


@pytest.mark.parametrize('delta', [-0.0005, -0.0004, 0.0, 0.0004, 0.0005])
def test_top_signed_distance_and_projection(delta):
    """상면 안팎의 오차를 허용하되 렌더링 점은 표면 바로 위에 둔다."""
    hit = CakeSurface().contact((0.51, 0.0, 0.142 + delta))
    assert hit.surface == 'top'
    assert hit.signed_distance_m == pytest.approx(delta)
    assert hit.projected == pytest.approx((0.51, 0.0, 0.142))
    assert 0 < math.dist(hit.projected, hit.rendered) <= 0.0003


@pytest.mark.parametrize('delta', [-0.0005, -0.0004, 0.0, 0.0004, 0.0005])
def test_side_signed_distance_and_projection(delta):
    """측면은 중심에서 바깥쪽을 양의 부호로 사용한다."""
    hit = CakeSurface().contact((0.5, 0.05 + delta, 0.1))
    assert hit.surface == 'side'
    assert hit.signed_distance_m == pytest.approx(delta)
    assert hit.projected == pytest.approx((0.5, 0.05, 0.1))
    assert 0 < math.dist(hit.projected, hit.rendered) <= 0.0003


@pytest.mark.parametrize('point', [
    (0.5, 0.0, 0.15), (0.5, 0.0, 0.14),
    (0.56, 0.0, 0.1), (0.548, 0.0, 0.1),
    (0.5, 0.0, 0.062), (0.55, 0.0, 0.06),
    (float('nan'), 0.0, 0.142),
])
def test_far_deep_inside_bottom_and_invalid_are_rejected(point):
    """바닥면과 내부 등 허용 범위 밖의 위치에서는 잉크를 만들지 않는다."""
    assert CakeSurface().contact(point) is None


def test_finite_faces_do_not_extend_beyond_cake():
    """원통 밖 상면 연장선과 케이크 위쪽 측면 연장선을 배제한다."""
    cake = CakeSurface()
    assert cake.contact((0.551, 0, 0.142)) is None
    assert cake.contact((0.55, 0, 0.143)) is None


def test_fresh_samples_connect_but_duplicate_stamp_is_not_observed_twice():
    """같은 최신 TF를 반복 조회해도 점·관측 수를 늘리지 않는다."""
    tracker = ContactTracker()
    tracker.observe((0.5, 0, 0.142), stamp=1.0, now=1.0)
    tracker.observe((0.501, 0, 0.142), stamp=1.01, now=1.02)
    tracker.observe((0.502, 0, 0.142), stamp=1.01, now=1.03)
    assert tracker.observed_count == 2
    assert len(tracker.points) == 2
    assert len(tracker.segments()) == 1


@pytest.mark.parametrize('gap', ['missing', 'stale', 'stamp_gap', 'future', 'reverse'])
def test_tf_gaps_never_connect(gap):
    """관측 중단이나 시간 역전 뒤 가까운 점으로 돌아와도 선을 잇지 않는다."""
    tracker = ContactTracker()
    tracker.observe((0.5, 0, 0.142), stamp=1.0, now=1.0)
    if gap == 'missing':
        tracker.missing()
    elif gap == 'stale':
        tracker.observe((0.5, 0, 0.142), stamp=1.0, now=1.3)
    elif gap == 'future':
        tracker.observe((0.5, 0, 0.142), stamp=2.0, now=1.01)
    elif gap == 'reverse':
        tracker.observe((0.5, 0, 0.142), stamp=0.9, now=1.01)
    stamp = {'stamp_gap': 1.4, 'stale': 1.31}.get(gap, 1.04)
    tracker.observe((0.501, 0, 0.142), stamp=stamp, now=stamp)
    assert len(tracker.points) == 2
    assert tracker.segments() == []


def test_noncontact_and_surface_transition_and_long_jump_break_lines():
    """들림·상면/측면 전환·과도한 점 간격은 별개의 잉크 구간이다."""
    tracker = ContactTracker()
    for index, point in enumerate([
        (0.549, 0, 0.142), (0.55, 0, 0.141),
        (0.55, 0, 0.12), (0.56, 0, 0.12), (0.55, 0, 0.119),
    ]):
        tracker.observe(point, stamp=1 + index * 0.01, now=1 + index * 0.01)
    assert len(tracker.points) == 4
    assert tracker.segments() == []
    assert tracker.observed_count == 5


def test_points_and_segments_are_bounded_and_clear_keeps_stamp_watermark():
    """오래된 점은 제거하며 clear 직전 TF를 새 파형으로 재사용하지 않는다."""
    tracker = ContactTracker(max_points=3)
    for index in range(8):
        stamp = 1 + index * 0.01
        tracker.observe((0.5 + index * 0.001, 0, 0.142), stamp, stamp)
    assert len(tracker.points) == 3
    assert len(tracker.segments()) == 2
    assert tracker.contact_count == 8
    tracker.clear()
    tracker.observe((0.507, 0, 0.142), 1.07, 1.08)
    assert len(tracker.points) == 0
    assert tracker.observed_count == 0
    tracker.observe((0.508, 0, 0.142), 1.09, 1.09)
    assert len(tracker.points) == 1
    assert tracker.segments() == []


def test_reference_update_clears_old_ink_and_changes_geometry():
    """새 top center는 이전 좌표계의 잉크를 초기화한다."""
    tracker = ContactTracker()
    tracker.observe((0.5, 0, 0.142), 1, 1)
    tracker.set_reference((0.6, 0.1, 0.2))
    assert not tracker.points
    tracker.observe((0.6, 0.1, 0.2), 2, 2)
    assert tracker.points[-1].contact.projected == pytest.approx((0.6, 0.1, 0.2))


def test_stationary_new_stamps_count_observations_without_duplicate_ink():
    """정지 중에도 관측 통계는 증가하지만 같은 위치에 점을 계속 쌓지 않는다."""
    tracker = ContactTracker()
    tracker.observe((0.5, 0, 0.142), 1, 1)
    tracker.observe((0.5, 0, 0.142), 1.01, 1.01)
    assert tracker.observed_count == 2
    assert tracker.contact_count == 2
    assert len(tracker.points) == 1


def test_contact_tolerance_boundary_does_not_admit_meaningfully_farther_points():
    """경계의 부동소수 오차만 허용하고 1nm 밖의 점은 거부한다."""
    cake = CakeSurface()
    assert cake.contact((0.5, 0, 0.142 + 0.000500001)) is None
    assert cake.contact((0.55 + 0.000000001 + 0.0005, 0, 0.1)) is None


@pytest.mark.parametrize('kwargs', [
    {'radius': 0}, {'height': -1}, {'top_z': float('nan')},
    {'render_offset_m': 0.00031}, {'contact_tolerance_m': -1},
])
def test_invalid_geometry_configuration_is_rejected(kwargs):
    """잘못된 단위·수치는 잘못 그리는 대신 초기화 오류로 드러낸다."""
    with pytest.raises(ValueError):
        CakeSurface(**kwargs)


def test_stationary_tf_does_not_hide_a_real_missing_gap():
    """정지 중 관측 후에도 명시적 누락은 같은 위치에서 새 구간을 시작한다."""
    tracker = ContactTracker()
    for index in range(40):
        stamp = 1 + index * 0.01
        tracker.observe((0.5, 0, 0.142), stamp, stamp)
    tracker.observe((0.501, 0, 0.142), 1.4, 1.4)
    assert len(tracker.segments()) == 1
    tracker.missing()
    tracker.observe((0.501, 0, 0.142), 1.41, 1.41)
    tracker.observe((0.502, 0, 0.142), 1.42, 1.42)
    assert len(tracker.segments()) == 2


def test_telemetry_counts_unique_observations_and_contact_errors():
    """파형 수집용 JSON은 유효 관측·접촉·오차와 모델의 한계를 담는다."""
    tracker = ContactTracker()
    tracker.observe((0.5, 0, 0.1424), 1, 1)
    tracker.observe((0.501, 0, 0.1418), 1.01, 1.01)
    tracker.observe((0.501, 0, 0.15), 1.02, 1.02)
    tracker.observe((0.501, 0, 0.15), 1.02, 1.03)
    data = json.loads(json.dumps(tracker.telemetry(), allow_nan=False))
    assert data['observed_count'] == 3
    assert data['contact_count'] == 2
    assert data['state'] == 'no_contact'
    assert data['contact_error_mean_m'] == pytest.approx(0.0003)
    assert data['contact_error_max_m'] == pytest.approx(0.0004)
    assert data['ink_length_m'] == pytest.approx(0.001)
    assert data['model'] == 'tf_surface_distance_not_physical_sensor'


@pytest.mark.parametrize('kwargs', [
    {'max_points': 0}, {'max_points': 1.5}, {'max_tf_age_sec': 0},
    {'max_link_distance_m': float('nan')},
])
def test_invalid_tracker_configuration_is_rejected(kwargs):
    """시간·메모리 제한이 무효이면 초기화 단계에서 거부한다."""
    with pytest.raises(ValueError):
        ContactTracker(**kwargs)


def test_ros_node_markers_observation_and_clear():
    """ROS 사용 가능 시 실제 Buffer·메시지·QoS로 표시 계약을 검사한다.

    로봇/서비스는 호출하지 않는다. 실행 시 별도 ROS_DOMAIN_ID를 지정하면
    팀원의 실행 중 그래프와 분리해 확인할 수 있다.
    """
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Empty, String
    from visualization_msgs.msg import Marker, MarkerArray
    from cake_decorator.virtual_ink_node import VirtualInkNode

    # 테스트 과정에서 파일 로그를 만들지 않는다.
    rclpy.init(args=['--ros-args', '--disable-external-lib-logs'])
    node = None
    peer = None
    try:
        node = VirtualInkNode()
        peer = Node('virtual_ink_test_observer')
        received = {'ink': [], 'telemetry': [], 'tip': []}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [
            peer.create_subscription(MarkerArray, '/cake/ink', received['ink'].append, qos),
            peer.create_subscription(
                String, '/cake/ink_telemetry', received['telemetry'].append, 10,
            ),
            peer.create_subscription(PoseStamped, '/cake/observed_tip', received['tip'].append, 10),
        ]
        deadline = time.monotonic() + 5
        while node._tip_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(peer, timeout_sec=0.02)
        assert node._ink_pub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'virtual_pen_tip'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.translation.x = 0.5
        transform.transform.translation.z = 0.142
        transform.transform.rotation.w = 1.0
        node._buffer.set_transform(transform, 'test_observed_tf')
        node._observe()
        node._observe()
        node._publish()
        deadline = time.monotonic() + 5
        while not all(received.values()) and time.monotonic() < deadline:
            rclpy.spin_once(peer, timeout_sec=0.02)
        assert all(received.values())
        markers = received['ink'][-1].markers
        assert [marker.type for marker in markers] == [Marker.LINE_LIST, Marker.SPHERE_LIST]
        assert len(markers[0].points) == 0
        assert len(markers[1].points) == 1
        assert markers[1].color.r == 1.0 and markers[1].color.g == 0.0
        assert markers[1].header.frame_id == 'base_link'
        assert received['tip'][-1].pose.position.z == pytest.approx(0.142)
        assert received['tip'][-1].header.stamp == transform.header.stamp
        assert json.loads(received['telemetry'][-1].data)['observed_count'] == 1

        # 정지 상태의 새 stamp도 관측하지만 전체 잉크 재생성은 필요하지 않다.
        transform.header.stamp = node.get_clock().now().to_msg()
        node._buffer.set_transform(transform, 'test_observed_tf')
        node._observe()
        assert node.tracker.observed_count == 2
        assert not node._dirty

        reference = PoseStamped()
        reference.pose.position.x = 0.6
        reference.pose.position.z = 0.2
        node._reference(reference)
        assert node.tracker.cake.center_x == 0.5  # frame가 없으면 거부한다.
        reference.header.frame_id = 'base_link'
        node._reference(reference)
        assert node.tracker.cake.center_x == 0.6
        assert not node.tracker.points
        node._clear(Empty())
        received['ink'].clear()
        deadline = time.monotonic() + 5
        while not received['ink'] and time.monotonic() < deadline:
            rclpy.spin_once(peer, timeout_sec=0.02)
        assert received['ink']
        assert all(not marker.points for marker in received['ink'][-1].markers)
        assert all(marker.action == Marker.DELETE for marker in received['ink'][-1].markers)
        assert subscriptions  # 참조를 유지하여 테스트 중 구독이 살아 있게 한다.
    finally:
        if peer is not None:
            peer.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
