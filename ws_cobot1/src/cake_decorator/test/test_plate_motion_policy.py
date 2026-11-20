"""실제 서비스 없이 플레이트 단계별 속도·순서·중단 계약을 검증한다."""

from types import SimpleNamespace
from unittest.mock import patch

import pytest

from cake_decorator.doosan_robot import DoosanRobotClient
from cake_decorator.plate_sequence import PlateSequence


# 기대값은 정책 구현과 독립적으로 지정한다: 이름, 서비스, 속도, 가속도, 모드.
PREPARE = [
    ("safe_position_1", "move_joint", 60, 120, 0),
    ("move_cake_ps_l2", "move_line", 150, 300, 0),
    ("move_cake_plate_ps1", "move_joint", 30, 30, 0),
    ("move_cake_plate_ps2", "move_joint", 30, 30, 1),
    ("move_cake_plate_ps3", "move_joint", 30, 30, 0),
    ("move_cake_plate_ps4", "move_line", 25, 50, 1),
    ("safe_position_2", "move_joint", 30, 30, 0),
    ("cake_station_ps1", "move_joint", 30, 30, 0),
    ("cake_station_ps2", "move_joint", 15, 15, 0),
    ("cake_station_ps3", "move_joint", 30, 30, 0),
    ("cake_station_ps4", "move_joint", 60, 120, 0),
    ("safe_position_3", "move_joint", 60, 120, 0),
]
FINISH = [
    ("safe_position_1", "move_joint", 60, 120, 0),
    ("move_cake_ps_l2", "move_line", 150, 300, 0),
    ("pickup_cake_station_ps2", "move_joint", 30, 30, 0),
    ("pickup_cake_station_ps3", "move_joint", 15, 15, 0),
    ("pickup_cake_station_ps4", "move_joint", 15, 15, 0),
    ("safe_position_4", "move_joint", 30, 30, 0),
    ("deliver_cake_station_ps1", "move_joint", 30, 30, 0),
    ("deliver_cake_station_ps2", "move_joint", 15, 15, 0),
    ("deliver_cake_station_ps3", "move_joint", 30, 30, 0),
    ("deliver_cake_station_ps4", "move_joint", 60, 120, 0),
    ("deliver_cake_station_ps5", "move_line", 150, 300, 1),
    ("safe_position_5", "move_joint", 60, 120, 0),
    ("safe_position_5", "move_joint", 60, 120, 0),
]
MOTION_PARAMS = {
    "free_joint_speed_deg_s": 60.0,
    "free_joint_acc_deg_s2": 120.0,
    "free_line_speed_mm_s": 150.0,
    "free_line_acc_mm_s2": 300.0,
    "cake_lift_speed_mm_s": 25.0,
    "cake_lift_acc_mm_s2": 50.0,
    "cake_seat_joint_speed_deg_s": 15.0,
    "cake_seat_joint_acc_deg_s2": 15.0,
}


class RecordingRobot(DoosanRobotClient):
    """실제 이동 요청 생성은 실행하되 ROS 서비스와 그리퍼 IO는 기록만 한다."""

    def __init__(self, fail_at=None):
        self.moves = []
        self.grips = []
        self.events = []
        self.logs = []
        self.fail_at = fail_at
        self._logger = SimpleNamespace(info=self.logs.append)

    def _call_checked(self, key, request):
        """이동 요청을 저장하고 지정된 순번의 서비스 실패를 재현한다."""
        self.moves.append((key, request))
        self.events.append(("move", key, tuple(request.pos)))
        return len(self.moves) != self.fail_at

    def gripper_open(self, width_mm=100.0):
        """실제 DO 출력 없이 열림 폭과 순서만 보존한다."""
        self.grips.append(("open", width_mm))
        self.events.append(("open", width_mm))
        return True

    def gripper_close(self, width_mm=10.0, force_n=20.0):
        """실제 DO 출력 없이 파지 폭·힘과 순서만 보존한다."""
        self.grips.append(("close", width_mm, force_n))
        self.events.append(("close", width_mm, force_n))
        return True


def make_sequence(overrides=None, fail_at=None):
    """단계마다 식별 가능한 좌표와 독립적인 속도 설정을 준비한다."""
    names = sorted({row[0] for row in PREPARE + FINISH})
    # 삭제 대상도 입력에 남겨 호출 시 순서 검사에서 반드시 검출되게 한다.
    names.append("pickup_cake_station_ps1")
    params = {
        "plate_" + name: [float(index + 1)] * 6
        for index, name in enumerate(names)
    }
    params.update(MOTION_PARAMS)
    params.update({
        "plate_pick_opener1": [426.73, 252.72, 379.41, 30.09, 179.36, 28.13],
        "plate_pick_opener2": [0, 0, -100, 0, 0, 0],
        "opener_grip_width_mm": 12.0,
        "opener_grip_force_n": 22.0,
        "opener_open_width_mm": 98.0,
        "opener_pre_open_wait_sec": 0.7,
        "opener_close_wait_sec": 0.8,
        "opener_close_settle_sec": 0.9,
        "opener_open_wait_sec": 1.1,
    })
    params.update(overrides or {})
    robot = RecordingRobot(fail_at)
    return PlateSequence(robot, params.__getitem__), robot, params


def named_moves(robot, params):
    """오프너 내부 이동을 제외하고 플레이트 좌표별 서비스 요청을 반환한다."""
    names = {
        tuple(value): key.removeprefix("plate_")
        for key, value in params.items()
        if key.startswith("plate_") and "opener" not in key
    }
    return [
        (names[tuple(req.pos)], kind, req)
        for kind, req in robot.moves
        if tuple(req.pos) in names
    ]


@pytest.mark.parametrize("phase, expected", [("prepare", PREPARE), ("finish", FINISH)])
def test_stage_speeds_order_modes_and_logs(phase, expected):
    """취급 구간 경계·저속 단계·삭제된 경유점의 재삽입 오류를 검출한다."""
    sequence, robot, params = make_sequence()
    with patch("cake_decorator.doosan_robot.time.sleep"):
        assert getattr(sequence, "_plate_" + phase + "_steps")()
    actual = named_moves(robot, params)
    assert [name for name, _, _ in actual] == [row[0] for row in expected]
    for (name, kind, req), (_, want_kind, speed, acc, mode) in zip(actual, expected):
        assert kind == want_kind
        assert req.mode == mode
        assert req.sync_type == 0
        if kind == "move_joint":
            assert (req.vel, req.acc) == (speed, acc), name
        else:
            assert (req.vel[0], req.acc[0], req.ref) == (speed, acc, 0), name
        assert any(
            name in message and f"speed={speed:g}" in message
            for message in robot.logs
        ), name


@pytest.mark.parametrize("phase", ["prepare", "finish"])
@pytest.mark.parametrize("free_speed, rack_speed", [(150, 90), (71, 71), (90, 90), (240, 90)])
def test_opener_speeds_waits_and_grip_contract(phase, free_speed, rack_speed):
    """오프너의 자유 이동·랙 속도 상한과 기존 대기·그리퍼 계약을 검사한다."""
    sequence, robot, params = make_sequence({"free_line_speed_mm_s": free_speed})
    with patch("cake_decorator.doosan_robot.time.sleep") as sleep:
        assert getattr(sequence, "_plate_" + phase + "_steps")()
    plate_moves = {id(req) for _, _, req in named_moves(robot, params)}
    opener = [(kind, req) for kind, req in robot.moves if id(req) not in plate_moves]
    # 공통 pick()은 랙 진입과 파지 후 후퇴에 같은 속도를 사용한다.
    assert [req.vel[0] for _, req in opener] == [
        free_speed, rack_speed, rack_speed, free_speed, rack_speed, free_speed,
    ]
    assert all(kind == "move_line" and req.acc[0] == 100 for kind, req in opener)
    assert all((req.ref, req.mode, req.sync_type) == (0, 0, 0) for _, req in opener)
    approach = params["plate_pick_opener1"]
    target = [426.73, 252.72, 279.41, 30.09, 179.36, 28.13]
    for (_, req), pose in zip(opener, [approach, target, approach] * 2):
        assert list(req.pos) == pytest.approx(pose)
    # 움찔거림 수정으로 pick은 닫기만 하고 place에서만 열기 펄스를 보낸다.
    assert robot.grips == [("close", 12, 22), ("open", 98)]
    assert [call.args[0] for call in sleep.call_args_list] == [0.8, 0.9, 1.1]
    for stage in ("pick_opener", "place_opener"):
        retract_speed = rack_speed if stage == "pick_opener" else free_speed
        assert (
            f"plate {stage}: approach={free_speed:g} slow={rack_speed:g} "
            f"retract={retract_speed:g}mm/s"
        ) in robot.logs


@pytest.mark.parametrize("phase, expected", [("prepare", PREPARE), ("finish", FINISH)])
def test_complete_motion_gripper_and_wait_order_is_preserved(phase, expected):
    """이동 사이의 파지·해제·대기 위치까지 전체 실행 순서를 고정한다."""
    sequence, robot, params = make_sequence()
    with patch(
        "cake_decorator.doosan_robot.time.sleep",
        side_effect=lambda seconds: robot.events.append(("wait", seconds)),
    ):
        assert getattr(sequence, "_plate_" + phase + "_steps")()

    approach = tuple(params["plate_pick_opener1"])
    target = (426.73, 252.72, 279.41, 30.09, 179.36, 28.13)
    plate_events = [
        ("move", kind, tuple(params["plate_" + name]))
        for name, kind, _, _, _ in expected
    ]
    # 완료 시퀀스의 마지막 안전 자세는 오프너 반납·후퇴 뒤에 실행한다.
    before_place = plate_events[1:] if phase == "prepare" else plate_events[1:-1]
    after_place = [] if phase == "prepare" else plate_events[-1:]
    assert robot.events == [
        plate_events[0],
        ("move", "move_line", approach),
        ("move", "move_line", target),
        ("close", 12, 22),
        ("wait", 0.8),
        ("wait", 0.9),
        ("move", "move_line", approach),
        *before_place,
        ("move", "move_line", approach),
        ("move", "move_line", target),
        ("open", 98),
        ("wait", 1.1),
        ("move", "move_line", approach),
        *after_place,
    ]


def test_parameter_overrides_leave_normal_cake_transport_unchanged():
    """여덟 파라미터의 변경이 해당 동작에만 적용되는지 확인한다."""
    overrides = dict(zip(MOTION_PARAMS, [41, 42, 71, 142, 21, 42, 11, 12]))
    sequence, robot, params = make_sequence(overrides)
    with patch("cake_decorator.doosan_robot.time.sleep"):
        assert sequence._plate_prepare_steps()
        assert sequence._plate_finish_steps()
    expected = {
        "safe_position_1": (41, 42),
        "move_cake_ps_l2": (71, 142),
        "move_cake_plate_ps1": (30, 30),
        "move_cake_plate_ps4": (21, 42),
        "cake_station_ps2": (11, 12),
        "pickup_cake_station_ps3": (11, 12),
        "pickup_cake_station_ps4": (11, 12),
        "safe_position_4": (30, 30),
        "deliver_cake_station_ps2": (11, 12),
        "deliver_cake_station_ps5": (71, 142),
    }
    for name, kind, req in named_moves(robot, params):
        if name in expected:
            actual = (req.vel, req.acc) if kind == "move_joint" else (req.vel[0], req.acc[0])
            assert actual == expected[name], name


@pytest.mark.parametrize("phase, total", [("prepare", 18), ("finish", 19)])
def test_any_failed_move_stops_sequence(phase, total):
    """오프너 내부를 포함해 어느 이동이 실패해도 다음 요청을 보내지 않는다."""
    for fail_at in range(1, total + 1):
        sequence, robot, _ = make_sequence(fail_at=fail_at)
        with patch("cake_decorator.doosan_robot.time.sleep"):
            assert not getattr(sequence, "_plate_" + phase + "_steps")(), fail_at
        assert len(robot.moves) == fail_at
