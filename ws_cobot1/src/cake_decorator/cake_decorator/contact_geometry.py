"""실제 접촉 센서가 아닌, 관측 TF와 원통 표면거리로 잉크를 추정하는 순수 모델.

모든 좌표·거리는 base_link 기준 m, 시간은 같은 ROS clock 기준 초이다.
ROS 또는 로봇 제어 의존성이 없으며 상면과 측면만 잉크 대상으로 취급한다.
"""

from collections import Counter, deque
from dataclasses import dataclass, replace
from itertools import islice
import math


@dataclass(frozen=True)
class Contact:
    """관측점의 표면 종류, 바깥쪽 양의 거리, 표면 투영점과 표시점을 보관한다."""

    surface: str
    signed_distance_m: float
    projected: tuple[float, float, float]
    rendered: tuple[float, float, float]


@dataclass(frozen=True)
class CakeSurface:
    """회전하지 않은 원통의 상단 중심·치수(m)와 접촉/표시 허용값이다."""

    center_x: float = 0.5
    center_y: float = 0.0
    top_z: float = 0.142
    radius: float = 0.05
    height: float = 0.08
    contact_tolerance_m: float = 0.0005
    render_offset_m: float = 0.0002

    def __post_init__(self):
        """생성 시 비유한 좌표, 음수 치수와 과도한 표시 오프셋을 거부한다."""
        values = (
            self.center_x, self.center_y, self.top_z, self.radius, self.height,
            self.contact_tolerance_m, self.render_offset_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('케이크 설정은 유한한 수여야 합니다.')
        if self.radius <= 0 or self.height <= 0 or self.contact_tolerance_m < 0:
            raise ValueError('치수는 양수, 접촉 허용값은 0 이상이어야 합니다.')
        if not 0 <= self.render_offset_m <= 0.0003:
            raise ValueError('표시 오프셋은 0~0.0003m여야 합니다.')

    def contact(self, point):
        """유한한 상면/측면 영역에서만 ±허용거리 이내의 접촉을 반환한다.

        모서리에서 두 면이 모두 가능하면 절대거리가 작은 면을 택하며 동률은
        상면 우선이다. 내부 깊은 점과 바닥면은 접촉으로 처리하지 않는다.
        """
        if len(point) != 3 or not all(math.isfinite(value) for value in point):
            return None
        x, y, z = point
        dx, dy = x - self.center_x, y - self.center_y
        radial = math.hypot(dx, dy)
        candidates = []
        # m 단위 뺄셈의 반올림으로 정확히 ±0.5mm인 점이 거부되지 않게 한다.
        # 1pm 여유는 모델 허용거리와는 별개이며 실제 1nm 초과점은 거부한다.
        epsilon = 1e-12
        tolerance = self.contact_tolerance_m + epsilon
        top_distance = z - self.top_z
        if radial <= self.radius + epsilon and abs(top_distance) <= tolerance:
            projected = (x, y, self.top_z)
            candidates.append(Contact(
                'top', top_distance, projected,
                (x, y, self.top_z + self.render_offset_m),
            ))
        side_distance = radial - self.radius
        if (
            self.top_z - self.height - epsilon <= z <= self.top_z + epsilon
            and radial > 0
            and abs(side_distance) <= tolerance
        ):
            nx, ny = dx / radial, dy / radial
            projected = (
                self.center_x + nx * self.radius,
                self.center_y + ny * self.radius, z,
            )
            candidates.append(Contact(
                'side', side_distance, projected,
                (projected[0] + nx * self.render_offset_m,
                 projected[1] + ny * self.render_offset_m, z),
            ))
        return min(candidates, key=lambda item: abs(item.signed_distance_m), default=None)


@dataclass(frozen=True)
class InkPoint:
    """저장된 접촉 하나와 직전 저장점으로 이어도 되는지 나타내는 계약이다."""

    contact: Contact
    stamp: float
    linked: bool


class ContactTracker:
    """고유하고 신선한 TF만 관측하며, 끊긴 구간을 연결하지 않는 유계 잉크 저장소."""

    def __init__(
        self, cake=None, max_tf_age_sec=0.25,
        max_points=100000, max_link_distance_m=0.01,
    ):
        """표면 모델과 시간·거리 제한을 받아 파형별 통계를 초기화한다."""
        if (
            not math.isfinite(max_tf_age_sec) or max_tf_age_sec <= 0
            or not math.isfinite(max_link_distance_m) or max_link_distance_m <= 0
            or isinstance(max_points, bool) or not isinstance(max_points, int)
            or max_points < 1
        ):
            raise ValueError('시간·연결 거리는 양수, max_points는 양의 정수여야 합니다.')
        self.cake = cake if cake is not None else CakeSurface()
        self.max_tf_age_sec = max_tf_age_sec
        self.max_link_distance_m = max_link_distance_m
        self.points = deque(maxlen=max_points)
        self._last_stamp = None
        self._last_now = None
        self.clear()

    def clear(self):
        """잉크와 파형 통계를 비우되 이전 TF를 재사용하지 않도록 stamp는 유지한다."""
        self.points.clear()
        self._previous = None
        self.state = 'cleared'
        self.observed_count = 0
        self.contact_count = 0
        self.stored_point_count = 0
        self.reasons = Counter()
        self.last_tip = None
        self.last_contact = None
        self.tf_age_sec = None
        self.contact_error_sum_m = 0.0
        self.contact_error_max_m = 0.0
        self.ink_length_m = 0.0

    def set_reference(self, center):
        """검증된 새 상단 중심(m)을 적용하고 이전 기준의 잉크를 지운다."""
        self.cake = replace(
            self.cake, center_x=center[0], center_y=center[1], top_z=center[2],
        )
        self.clear()

    def missing(self):
        """TF 조회 실패 시 기존 잉크는 남기고 다음 샘플과의 연결만 끊는다."""
        self._reject('missing_tf')
        self.tf_age_sec = None

    def _reject(self, reason):
        """비접촉/유효하지 않은 관측 사유를 기록하고 연결 앵커를 해제한다."""
        self.state = reason
        self.reasons[reason] += 1
        self._previous = None
        self.last_contact = None

    def observe(self, point, stamp, now):
        """최신 TF 위치·stamp·현재 시각을 받아 새 접촉이면 Contact를 반환한다.

        동일 stamp는 위치가 달라도 재관측하지 않는다. 정지 상태의 새 stamp는
        통계에만 반영하고, 연결이 끊겼다면 같은 위치라도 새 구간 시작점을 남긴다.
        """
        if not math.isfinite(stamp) or not math.isfinite(now):
            self._reject('invalid_time')
            self.tf_age_sec = None
            return None
        # /clock 역행은 새 시간축으로 취급하지만 이전 잉크에 선을 잇지는 않는다.
        if self._last_now is not None and now < self._last_now:
            self._previous = None
            self._last_stamp = None
        self._last_now = now
        self.tf_age_sec = now - stamp
        if not 0 <= self.tf_age_sec <= self.max_tf_age_sec:
            self._reject('future_tf' if self.tf_age_sec < 0 else 'stale_tf')
            return None
        if self._last_stamp is not None and stamp <= self._last_stamp:
            if stamp < self._last_stamp:
                self._reject('out_of_order_tf')
            else:
                self.reasons['duplicate_tf'] += 1
            return None
        if self._last_stamp is not None and stamp - self._last_stamp > self.max_tf_age_sec:
            self._previous = None
        self._last_stamp = stamp
        if len(point) != 3 or not all(math.isfinite(value) for value in point):
            self._reject('invalid_point')
            return None
        self.observed_count += 1
        self.last_tip = tuple(point)
        contact = self.cake.contact(point)
        if contact is None:
            self._reject('no_contact')
            return None
        self.state = 'contact'
        self.last_contact = contact
        self.contact_count += 1
        error = abs(contact.signed_distance_m)
        self.contact_error_sum_m += error
        self.contact_error_max_m = max(self.contact_error_max_m, error)
        previous = self._previous
        linked = (
            previous is not None
            and previous.contact.surface == contact.surface
            and stamp - previous.stamp <= self.max_tf_age_sec
            and math.dist(previous.contact.projected, contact.projected)
            <= self.max_link_distance_m
        )
        sample = InkPoint(contact, stamp, linked)
        if linked and math.dist(previous.contact.projected, contact.projected) < 1e-12:
            # 정지 중 앵커의 시각을 갱신하여 정상 재개를 시간 누락으로 오인하지 않는다.
            self._previous = sample
            return contact
        if linked:
            self.ink_length_m += math.dist(previous.contact.projected, contact.projected)
        self.points.append(sample)
        self.stored_point_count += 1
        self._previous = sample
        return contact

    def segments(self):
        """현재 보관 중인 점만 사용해 LINE_LIST용 (시작, 끝) 쌍을 반환한다."""
        result = []
        previous = None
        for sample in self.points:
            if previous is not None and sample.linked:
                result.append((previous.contact.rendered, sample.contact.rendered))
            previous = sample
        return result

    def telemetry(self):
        """파형 수집기가 JSON으로 직렬화할 수 있는 SI 단위 관측 통계를 반환한다."""
        contact = self.last_contact
        return {
            'model': 'tf_surface_distance_not_physical_sensor',
            'state': self.state,
            'observed_count': self.observed_count,
            'contact_count': self.contact_count,
            'stored_point_count': self.stored_point_count,
            'retained_point_count': len(self.points),
            'segment_count': sum(p.linked for p in islice(self.points, 1, None)),
            'tf_stamp_sec': self._last_stamp,
            'tf_age_sec': self.tf_age_sec,
            'observed_tip_m': self.last_tip,
            'surface': contact.surface if contact else None,
            'signed_distance_m': contact.signed_distance_m if contact else None,
            'contact_error_mean_m': (
                self.contact_error_sum_m / self.contact_count if self.contact_count else None
            ),
            'contact_error_max_m': self.contact_error_max_m,
            'ink_length_m': self.ink_length_m,
            'rejection_counts': dict(self.reasons),
        }
