"""실제 서비스 요청 대역으로 케이크 재집기의 횟수·경로·실패 경계를 검증한다."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from cake_decorator.safety_exceptions import PlateWeightError
from test_plate_motion_policy import make_sequence, named_moves


def weight_sequence(samples):
    """하중 검사만 켜고 시간·장비 IO 없이 공정 전체를 실행할 대역을 만든다."""
    sequence, robot, params = make_sequence({
        'plate_weight_check_enabled': True,
        'plate_weight_wait_sec': 0.0,
        'plate_weight_reads': 1,
        'plate_weight_read_interval': 0.0,
        'plate_opener_weight_kg': 0.05,
        'plate_cake_weight_kg': 0.20,
        'plate_weight_tolerance_percent': 10.0,
        'plate_cake_min_delta_kg': 0.075,
    })
    robot.stop_latched = False
    robot.gripper_output_fault = False
    robot.get_workpiece_weight = Mock(side_effect=samples)
    robot._logger = SimpleNamespace(
        info=robot.logs.append,
        warning=robot.logs.append,
        error=robot.logs.append,
    )
    sequence._logger = robot._logger
    # 좌표 조회 대역에는 하중 스칼라 설정을 섞지 않는다. 실행기는 원본 전체를 읽는다.
    coordinates = {
        name: value for name, value in params.items() if isinstance(value, list)
    }
    return sequence, robot, coordinates


@pytest.mark.parametrize('phase', ['prepare', 'finish'])
@pytest.mark.parametrize('missing_first', [False, True])
def test_complete_phase_retries_only_pick_and_continues(phase, missing_first):
    """양쪽 출발지에서 최초 성공은 1회, 재시도 성공은 2회 집고 후속 공정은 1회다."""
    samples = [0.10, 0.15, *([0.20] if missing_first else []), 0.30, 0.15, 0.10]
    sequence, robot, params = weight_sequence(samples)
    with patch('cake_decorator.doosan_robot.time.sleep'):
        assert getattr(sequence, f'_plate_{phase}_steps')()
    moves = named_moves(robot, params)
    names = [name for name, _, _ in moves]
    stage = f'{phase}_both'
    pick_names = [row[0] for row in sequence._CAKE_PICK_STEPS[stage]]
    expected_picks = 2 if missing_first else 1
    for name in pick_names:
        assert names.count(name) == expected_picks, name
    # 출발지·상대 이동·운동 제한까지 첫 집기와 재시도가 동일해야 한다.
    if missing_first:
        start = names.index(pick_names[0])
        count = len(pick_names)
        first = moves[start:start + count]
        retry = moves[start + count:start + count * 2]
        for (_, kind, original), (_, retry_kind, repeated) in zip(first, retry):
            assert kind == retry_kind
            assert original == repeated
        assert any('재시도 1/1 성공' in message for message in robot.logs)
    destination = 'cake_station_ps2' if phase == 'prepare' else 'deliver_cake_station_ps2'
    assert names.count(destination) == 1
    assert robot.get_workpiece_weight.call_count == len(samples)
    assert sequence._weight_checker.index == 4
    assert robot.grips == [('close', 12, 22), ('open', 98)]


@pytest.mark.parametrize('phase', ['prepare', 'finish'])
def test_second_missing_result_is_final_failure_and_returns_safely(phase):
    """두 번째도 케이크가 없으면 세 번째 집기 없이 감속 반납·안전 자세로 끝난다."""
    sequence, robot, params = weight_sequence([0.10, 0.15, 0.20, 0.20])
    with patch('cake_decorator.doosan_robot.time.sleep'):
        with pytest.raises(PlateWeightError, match='재시도 1/1 후에도 케이크 없음') as failure:
            getattr(sequence, f'_plate_{phase}_steps')()
    assert failure.value.code == 'CAKE_WEIGHT_FAILED'
    assert failure.value.resume_allowed is True
    assert '안전 자세에서 대기중' in str(failure.value)
    names = [name for name, _, _ in named_moves(robot, params)]
    assert names.count('move_cake_ps_l2') == 2
    assert 'cake_station_ps2' not in names
    assert 'deliver_cake_station_ps2' not in names
    assert robot.get_workpiece_weight.call_count == 4
    # 따개 반납 접근·하강·후퇴는 모두 75mm/s, 마지막 관절은 30도/s다.
    assert [request.vel[0] for _, request in robot.moves[-4:-1]] == [75.0] * 3
    assert robot.moves[-1][1].vel == 30.0
    assert list(robot.moves[-1][1].pos) == params['plate_safe_position_1']
    assert any('최종 실패' in message for message in robot.logs)


@pytest.mark.parametrize('phase', ['prepare', 'finish'])
def test_weight_service_failure_does_not_trigger_another_scoop(phase):
    """하중 응답 부재는 케이크 없음으로 추정하지 않고 기존 실패 복구로 보낸다."""
    sequence, robot, params = weight_sequence([0.10, 0.15, None])
    with patch('cake_decorator.doosan_robot.time.sleep'):
        with pytest.raises(PlateWeightError, match='하중 측정 실패') as failure:
            getattr(sequence, f'_plate_{phase}_steps')()
    assert failure.value.resume_allowed is False
    names = [name for name, _, _ in named_moves(robot, params)]
    assert names.count('move_cake_ps_l2') == 1
    assert not any('재시도 1/1 시작' in message for message in robot.logs)


@pytest.mark.parametrize('stage', ['prepare_both', 'finish_both'])
def test_failed_retry_motion_sends_no_return_or_further_motion(stage):
    """재집기 이동 실패 후 위치를 알 수 없으므로 반납·안전 복귀를 강행하지 않는다."""
    for failure_index in range(1, 7 if stage == 'prepare_both' else 6):
        sequence, robot, _ = weight_sequence([0.10, 0.15, 0.20])
        phase = stage.removesuffix('_both')
        with patch('cake_decorator.doosan_robot.time.sleep'):
            sequence._weight_at_safe(f'{phase}_start')
            sequence._weight_at_safe(f'{phase}_opener')
            sequence._opener_held = True
            robot.fail_at = failure_index
            with pytest.raises(PlateWeightError, match='재시도 1/1 이동 실패'):
                sequence._weight_at_safe(stage)
        assert len(robot.moves) == failure_index
        assert not robot.grips
        assert robot.get_workpiece_weight.call_count == 3


@pytest.mark.parametrize('fault', ['stop_latched', 'gripper_output_fault'])
def test_stop_during_retry_blocks_next_command(fault):
    """재집기 첫 이동 후 정지·출력 이상이 발생하면 다음 이동과 측정을 차단한다."""
    sequence, robot, _ = weight_sequence([0.10, 0.15, 0.20])
    original = robot._call_checked

    def block_after_first(key, request):
        """첫 서비스 요청만 기록한 뒤 다음 단계부터 중단 상태를 만든다."""
        moved = original(key, request)
        setattr(robot, fault, True)
        return moved

    robot._call_checked = block_after_first
    with patch('cake_decorator.doosan_robot.time.sleep'):
        sequence._weight_at_safe('prepare_start')
        sequence._weight_at_safe('prepare_opener')
        sequence._opener_held = True
        with pytest.raises(PlateWeightError, match='명령 차단'):
            sequence._weight_at_safe('prepare_both')
    assert len(robot.moves) == 1
    assert not robot.grips
