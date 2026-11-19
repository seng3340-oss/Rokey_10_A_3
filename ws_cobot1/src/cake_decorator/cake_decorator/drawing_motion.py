# 예정 역할: 시작점 위 접근 → 펜 내림 → 획 실행 → 펜 올림 → 다음 획 이동.
# 실제 DRL 호출은 doosan_robot.py에 위임한다.
# 점 선별·픽셀 변환을 다시 하지 않고 DP가 만든 경로를 따른다.
# 이전 run3에서 검증한 동작을 분리해 가져오되 전체 테스트 서버를 복사하지 않는다.
# 순응·힘제어, TCP 전환·속도 우선순위는 확정·검증한 정책만 적용한다.
# 펜 내림 명령이나 TCP 궤적을 실제 잉크가 그려졌다는 관측으로 취급하지 않는다.
"""접근·내림·그리기·올림 동작 로직.

ROS 의존성 없이 순수 Python + math만 사용한다.
Doosan M0609의 좌표계(mm, ZYZ 오일러각)로 변환하는 함수와
동작 시퀀스 계획 로직을 포함한다.

단위 규약
---------
- ROS 쪽 (Waypoint3D): m, 쿼터니언, mm/s
- Doosan DSR 쪽: mm, ZYZ 오일러각(°), mm/s
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

# ---------------------------------------------------------------------------
# 상수
# ---------------------------------------------------------------------------

# Doosan M0609 작업 영역 근사 범위 (mm)
WORKSPACE_X_MIN = -900.0
WORKSPACE_X_MAX = 900.0
WORKSPACE_Y_MIN = -900.0
WORKSPACE_Y_MAX = 900.0
WORKSPACE_Z_MIN = 0.0
WORKSPACE_Z_MAX = 900.0
WORKSPACE_RADIUS_MAX = 900.0  # BASE에서 최대 도달 거리 (mm)

# 기본 가감속 값
DEFAULT_ACC_LINEAR = 100.0  # mm/s²
# 윗면의 짧은 직선에서 감속·재가속에 걸리는 시간을 줄인다.
# 접촉·이탈·공중 이동과 옆면에는 기본 가속도를 유지한다.
TOP_DRAW_ACC_LINEAR = 300.0  # mm/s²
DEFAULT_ACC_ANGULAR = 30.0  # °/s²
DEFAULT_VEL_ANGULAR = 30.0  # °/s
# 원주를 돌면서 펜 방향도 회전하므로 선속도와 각속도 제한을 함께 조정한다.
# 접촉 진입·공중 이동에는 적용하지 않는다.
SIDE_DRAW_ACC_LINEAR = 300.0  # mm/s²
SIDE_DRAW_VEL_ANGULAR = 60.0  # °/s
SIDE_DRAW_ACC_ANGULAR = 120.0  # °/s²


# ---------------------------------------------------------------------------
# 열거형
# ---------------------------------------------------------------------------


class MotionType(Enum):
    """동작 유형."""

    MOVE_LINE = "movel"
    MOVE_CIRCLE = "movec"
    MOVE_SPLINE = "movesx"
    MOVE_JOINT = "movej"


class Phase(Enum):
    """드로잉 동작 단계."""

    APPROACH = "APPROACH"
    PEN_DOWN = "PEN_DOWN"
    DRAWING = "DRAWING"
    PEN_UP = "PEN_UP"
    REPOSITIONING = "REPOSITIONING"


# ---------------------------------------------------------------------------
# 전체 작업 상태 머신 (Task State Machine)
# ---------------------------------------------------------------------------
# 흐름도:
#   IDLE → RECEIVED → VALIDATING → PICKING_CAKE → PLACING_CAKE
#   → GRASPING_TOOL → DRAWING → RETURNING_TOOL → DELIVERING
#   → COMPLETED / FAILED
#   (어느 단계에서든 취소 → CANCELED, 실패 → FAILED)
# ---------------------------------------------------------------------------


class TaskState(Enum):
    """케이크 레터링 전체 작업 상태."""

    IDLE = "IDLE"
    RECEIVED = "RECEIVED"
    VALIDATING = "VALIDATING"
    PICKING_CAKE = "PICKING_CAKE"
    PLACING_CAKE = "PLACING_CAKE"
    GRASPING_TOOL = "GRASPING_TOOL"
    DRAWING = "DRAWING"
    RETURNING_TOOL = "RETURNING_TOOL"
    DELIVERING = "DELIVERING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


# 정상 흐름 순서 (FAILED/CANCELED 제외)
_NORMAL_FLOW = [
    TaskState.IDLE,
    TaskState.RECEIVED,
    TaskState.VALIDATING,
    TaskState.PICKING_CAKE,
    TaskState.PLACING_CAKE,
    TaskState.GRASPING_TOOL,
    TaskState.DRAWING,
    TaskState.RETURNING_TOOL,
    TaskState.DELIVERING,
    TaskState.COMPLETED,
]

# 활성 상태 (IDLE, 종료 상태 제외)
_ACTIVE_STATES = set(_NORMAL_FLOW[1:-1])  # RECEIVED ~ DELIVERING

# 유효한 상태 전이 맵
VALID_TRANSITIONS: dict[TaskState, set[TaskState]] = {}
for _i, _s in enumerate(_NORMAL_FLOW[:-1]):
    _next = {_NORMAL_FLOW[_i + 1]}
    # 활성 상태에서는 FAILED, CANCELED로도 전이 가능
    if _s in _ACTIVE_STATES:
        _next |= {TaskState.FAILED, TaskState.CANCELED}
    VALID_TRANSITIONS[_s] = _next
VALID_TRANSITIONS[TaskState.COMPLETED] = set()
VALID_TRANSITIONS[TaskState.FAILED] = {TaskState.IDLE}
VALID_TRANSITIONS[TaskState.CANCELED] = {TaskState.IDLE}

# 각 상태별 내부 세부 단계 (흐름도 참조)
TASK_STATE_SUBSTEPS: dict[TaskState, list[str]] = {
    TaskState.VALIDATING: [
        "경로데이터 검증",
        "로봇상태 확인",
        "교정치 확인",
        "재료/도구 확인",
    ],
    TaskState.PICKING_CAKE: [
        "작업위치이동",
        "로봇상태 확인",
        "그리퍼열기/닫기",
        "로봇이동 확인",
        "들어올리기",
    ],
    TaskState.PLACING_CAKE: [
        "작업대이동",
        "로봇상태 확인",
        "케이크배치",
        "배치 완료 확인",
        "그리퍼해제",
    ],
    TaskState.GRASPING_TOOL: [
        "TCP전환",
        "TCP전환 완료 확인",
        "거치대이동",
        "로봇동작 확인",
        "원위치",
    ],
    TaskState.DRAWING: [
        "레이어변환",
        "웨이포인트로변환",
        "명령도로잉",
        "웨이포인트로변환",
        "명령도로잉",
    ],
    TaskState.RETURNING_TOOL: [
        "거치대복귀",
        "로봇동작 확인",
        "펜반납",
        "반납 완료 확인",
        "TCP복원",
    ],
    TaskState.DELIVERING: [
        "케이크집기",
        "로봇이동 확인",
        "출고위치이동",
        "로봇동작 확인",
        "케이크내려놓기",
    ],
}


def can_transition(current: TaskState, target: TaskState) -> bool:
    """현재 상태에서 대상 상태로 전이 가능한지 확인한다."""
    return target in VALID_TRANSITIONS.get(current, set())


def get_next_state(current: TaskState) -> TaskState | None:
    """정상 흐름에서 다음 상태를 반환한다.

    FAILED, CANCELED, COMPLETED는 정상 흐름이 아니므로 제외.
    """
    try:
        idx = _NORMAL_FLOW.index(current)
        if idx < len(_NORMAL_FLOW) - 1:
            return _NORMAL_FLOW[idx + 1]
    except ValueError:
        pass
    return None


def get_substeps(state: TaskState) -> list[str]:
    """해당 상태의 내부 세부 단계 목록을 반환한다."""
    return TASK_STATE_SUBSTEPS.get(state, [])


# ---------------------------------------------------------------------------
# 데이터 클래스
# ---------------------------------------------------------------------------


@dataclass
class DsrPos:
    """Doosan 직교 좌표 (mm, ZYZ °)."""

    x: float  # mm
    y: float  # mm
    z: float  # mm
    rz1: float  # ° (Z축 1차 회전)
    ry: float  # ° (Y축 회전)
    rz2: float  # ° (Z축 2차 회전)

    def to_list(self) -> list[float]:
        """DSR API에 전달할 6요소 리스트."""
        return [self.x, self.y, self.z, self.rz1, self.ry, self.rz2]

    def with_z_offset(self, offset_mm: float) -> DsrPos:
        """Z축 오프셋을 적용한 새 좌표를 반환한다."""
        return DsrPos(self.x, self.y, self.z + offset_mm, self.rz1, self.ry, self.rz2)


@dataclass
class MotionCommand:
    """로봇에 보낼 단일 동작 명령."""

    motion_type: MotionType
    pos: DsrPos
    vel: float  # mm/s (선형 속도)
    acc: float  # mm/s² (선형 가속도)
    phase: Phase
    pen_down: bool
    stroke_idx: int = 0
    sync: bool = True  # True=동기, False=비동기
    spline: list[DsrPos] | None = None
    via: DsrPos | None = None  # 원호 경유점. 직선에서는 사용하지 않는다.
    angular_vel: float = DEFAULT_VEL_ANGULAR  # °/s, 도구 자세 회전 제한
    angular_acc: float = DEFAULT_ACC_ANGULAR  # °/s²


@dataclass
class MotionPlan:
    """전체 동작 계획."""

    commands: list[MotionCommand] = field(default_factory=list)
    total_strokes: int = 0
    error_msg: str = ""

    @property
    def success(self) -> bool:
        """에러 없으면 성공."""
        return len(self.commands) > 0 and not self.error_msg


# ---------------------------------------------------------------------------
# 좌표 변환
# ---------------------------------------------------------------------------


def quaternion_to_zyz(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float]:
    """쿼터니언 → ZYZ 오일러각 (°).

    Doosan M0609는 ZYZ 고유(intrinsic) 오일러각을 사용한다.
    회전 행렬을 거쳐 ZYZ로 분해한다.

    Returns
    -------
    (rz1, ry, rz2) : ZYZ 오일러각, 단위 degree
    """
    # 쿼터니언 → 회전 행렬 (필요한 요소만 계산)
    r00 = 1 - 2 * (qy * qy + qz * qz)
    r01 = 2 * (qx * qy - qz * qw)
    r02 = 2 * (qx * qz + qy * qw)
    r12 = 2 * (qy * qz - qx * qw)
    r20 = 2 * (qx * qz - qy * qw)
    r21 = 2 * (qy * qz + qx * qw)
    r22 = 1 - 2 * (qx * qx + qy * qy)

    eps = 1e-7

    if abs(r22) < 1.0 - eps:
        ry = math.acos(max(-1.0, min(1.0, r22)))
        rz1 = math.atan2(r12, r02)
        rz2 = math.atan2(r21, -r20)
    elif r22 > 0:
        # ry ≈ 0 (짐벌락)
        ry = 0.0
        rz1 = math.atan2(-r01, r00)
        rz2 = 0.0
    else:
        # ry ≈ π (짐벌락)
        ry = math.pi
        # B=π이면 r00=-cos(A-C), r01=-sin(A-C)다.
        # C=0 표현으로 합칠 때 A-C를 보존해야 한다. r01 부호를 그대로 쓰면
        # 수직 그리퍼의 수평 방향이 반전되어 실제 펜이 케이크 중심을 벗어난다.
        rz1 = math.atan2(-r01, -r00)
        rz2 = 0.0

    return (math.degrees(rz1), math.degrees(ry), math.degrees(rz2))


def m_to_mm(val: float) -> float:
    """미터 → 밀리미터."""
    return val * 1000.0


def waypoint_to_dsr_pos(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> DsrPos:
    """Waypoint3D 좌표(m, 쿼터니언) → Doosan 좌표(mm, ZYZ°)."""
    rz1, ry, rz2 = quaternion_to_zyz(qx, qy, qz, qw)
    return DsrPos(
        x=m_to_mm(x),
        y=m_to_mm(y),
        z=m_to_mm(z),
        rz1=rz1,
        ry=ry,
        rz2=rz2,
    )


def pose_msg_to_dsr(pose, z_offset_m: float = 0.0) -> DsrPos:
    """geometry_msgs/Pose → DsrPos 변환 헬퍼.

    Parameters
    ----------
    pose : geometry_msgs.msg.Pose (position + orientation)
    z_offset_m : Z축 오프셋 (m). 접근 높이 등에 사용.
    """
    return waypoint_to_dsr_pos(
        pose.position.x,
        pose.position.y,
        pose.position.z + z_offset_m,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


# ---------------------------------------------------------------------------
# 작업 영역 검증
# ---------------------------------------------------------------------------


def validate_workspace(pos: DsrPos) -> str:
    """Doosan M0609 작업 영역 내인지 검증한다.

    Returns
    -------
    빈 문자열이면 유효, 아니면 에러 메시지.
    """
    dist_xy = math.sqrt(pos.x**2 + pos.y**2)
    if dist_xy > WORKSPACE_RADIUS_MAX:
        return f"XY 도달 범위 초과: {dist_xy:.1f}mm > {WORKSPACE_RADIUS_MAX}mm"
    if pos.z < WORKSPACE_Z_MIN:
        return f"Z 하한 초과: {pos.z:.1f}mm < {WORKSPACE_Z_MIN}mm"
    if pos.z > WORKSPACE_Z_MAX:
        return f"Z 상한 초과: {pos.z:.1f}mm > {WORKSPACE_Z_MAX}mm"
    return ""


# ---------------------------------------------------------------------------
# 속도·가속도 계산
# ---------------------------------------------------------------------------


def calculate_vel_acc(
    speed_mm_s: float,
    acc_linear: float = DEFAULT_ACC_LINEAR,
) -> tuple[list[float], list[float]]:
    """DSR API용 속도/가속도 배열을 반환한다.

    Returns
    -------
    (vel, acc) : 각각 [mm/s, °/s], [mm/s², °/s²]
    """
    return [speed_mm_s, DEFAULT_VEL_ANGULAR], [acc_linear, DEFAULT_ACC_ANGULAR]


# ---------------------------------------------------------------------------
# 동작 시퀀스 계획
# ---------------------------------------------------------------------------


def plan_drawing_motion(
    waypoints: list[dict],
    pen_states: list[bool],
    drawing_surface: str = "TOP",
) -> MotionPlan:
    """웨이포인트 목록을 MotionCommand 시퀀스로 변환한다.

    Parameters
    ----------
    waypoints : Waypoint3D 정보 딕셔너리 목록
        각 항목: {x, y, z, qx, qy, qz, qw, speed, pen_down, contact_force}
    pen_states : 각 웨이포인트의 펜 상태
    drawing_surface : 'TOP' 또는 'SIDE'

    Returns
    -------
    MotionPlan
    """
    if not waypoints:
        return MotionPlan(error_msg="빈 웨이포인트 목록입니다.")
    if len(waypoints) != len(pen_states):
        return MotionPlan(error_msg="웨이포인트와 펜 상태 목록의 길이가 다릅니다.")

    commands: list[MotionCommand] = []
    stroke_idx = 0
    prev_pen = False

    for i, (wp, pen) in enumerate(zip(waypoints, pen_states)):
        # 좌표 변환
        dsr_pos = waypoint_to_dsr_pos(
            wp["x"],
            wp["y"],
            wp["z"],
            wp["qx"],
            wp["qy"],
            wp["qz"],
            wp["qw"],
        )

        # 작업 영역 검증
        err = validate_workspace(dsr_pos)
        if err:
            return MotionPlan(error_msg=f"웨이포인트 {i}: {err}")

        # 원호 경유점 전달은 획의 완료·중단 판단과 독립적으로 처리한다.
        kind = wp.get("motion_type", 0)
        if kind not in (0, 1, 2):
            return MotionPlan(error_msg=f"웨이포인트 {i}: 알 수 없는 이동 종류")
        via = None
        spline = None
        if kind == 2:
            if i == 0 or not 2 <= len(wp.get("spline", [])) <= 100:
                return MotionPlan(
                    error_msg="스플라인은 시작점과 2~100개 목표점이 필요합니다."
                )
            spline = []
            for sample in wp["spline"]:
                point = waypoint_to_dsr_pos(
                    sample["x"],
                    sample["y"],
                    sample["z"],
                    sample["qx"],
                    sample["qy"],
                    sample["qz"],
                    sample["qw"],
                )
                err = validate_workspace(point)
                if err:
                    return MotionPlan(error_msg=f"스플라인 표본: {err}")
                spline.append(point)
            if math.dist(spline[-1].to_list()[:3], dsr_pos.to_list()[:3]) > 1e-6:
                return MotionPlan(error_msg="스플라인 마지막 점이 목표점과 다릅니다.")
        if kind == 1:
            if i == 0 or "via" not in wp:
                return MotionPlan(error_msg="원호는 시작점과 경유점이 필요합니다.")
            v = wp["via"]
            via = waypoint_to_dsr_pos(
                v["x"], v["y"], v["z"], v["qx"], v["qy"], v["qz"], v["qw"]
            )
            err = validate_workspace(via)
            if err:
                return MotionPlan(error_msg=f"원호 경유점 {i}: {err}")
            previous = commands[-1].pos
            if max(abs(previous.z - via.z), abs(previous.z - dsr_pos.z)) > 1e-5:
                return MotionPlan(error_msg="옆면 원호의 시작·경유·끝 높이가 다릅니다.")
            determinant = (via.x - previous.x) * (dsr_pos.y - previous.y) - (
                via.y - previous.y
            ) * (dsr_pos.x - previous.x)
            if abs(determinant) < 1e-10:
                return MotionPlan(error_msg="원호의 세 점이 중복되거나 일직선입니다.")

        # 단계 결정
        phase = _determine_phase(prev_pen, pen, i, len(waypoints))

        # 이미 펜이 닿은 윗면의 직선에만 빠른 가감속을 적용한다.
        # 목표 좌표와 선속도, 각 목표점의 도달 확인 방식은 유지한다.
        acceleration = DEFAULT_ACC_LINEAR
        if drawing_surface == "TOP" and prev_pen and pen and kind == 0:
            acceleration = TOP_DRAW_ACC_LINEAR

        # 옆면은 원주 방향으로 펜 자세가 바뀌어 각속도 제한도 실행기에 전달한다.
        angular_vel = DEFAULT_VEL_ANGULAR
        angular_acc = DEFAULT_ACC_ANGULAR
        if drawing_surface == "SIDE" and prev_pen and pen:
            acceleration = SIDE_DRAW_ACC_LINEAR
            angular_vel = SIDE_DRAW_VEL_ANGULAR
            angular_acc = SIDE_DRAW_ACC_ANGULAR

        # 획 완료 카운트: pen_down → pen_up 전환 시
        if prev_pen and not pen:
            stroke_idx += 1

        commands.append(
            MotionCommand(
                motion_type={
                    0: MotionType.MOVE_LINE,
                    1: MotionType.MOVE_CIRCLE,
                    2: MotionType.MOVE_SPLINE,
                }[kind],
                spline=spline,
                via=via,
                pos=dsr_pos,
                vel=wp["speed"],
                acc=acceleration,
                angular_vel=angular_vel,
                angular_acc=angular_acc,
                phase=phase,
                pen_down=pen,
                stroke_idx=stroke_idx,
            )
        )
        prev_pen = pen

    return MotionPlan(
        commands=commands,
        total_strokes=stroke_idx + (1 if prev_pen else 0),
    )


def _determine_phase(
    prev_pen: bool,
    cur_pen: bool,
    idx: int,
    total: int,
) -> Phase:
    """이전/현재 펜 상태로 동작 단계를 결정한다."""
    if not cur_pen and not prev_pen:
        return Phase.REPOSITIONING
    if not prev_pen and cur_pen:
        return Phase.PEN_DOWN
    if prev_pen and not cur_pen:
        return Phase.PEN_UP
    # cur_pen == True
    return Phase.DRAWING
