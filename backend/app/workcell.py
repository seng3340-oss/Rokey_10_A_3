"""제작 실패 후 주문 잠금·관리자 확인과 영속 로그 조회를 관리한다.

관리자는 중단 단계 재개 또는 초기 자세 확인 후 주문 폐기를 선택한다. 자동 초기 이동은 없다.
기존 HMI 정지와 별개로 SQLite에 저장하므로 웹 서버 재시작으로 잠금이 풀리지 않는다.
"""

from datetime import datetime, timezone
import json
import math
from typing import Annotated, Literal

from fastapi import APIRouter, HTTPException, Query, Request
from pydantic import Field, model_validator

from .schemas import ContractModel

router = APIRouter(prefix='/api/workcell', tags=['관리자 재설정·로그'])


class ResetConfirmation(ContractModel):
    """HMI에 표시된 실패 세대와 현장 정리 확인. 오래된 화면의 해제를 거부한다."""

    revision: Annotated[int, Field(strict=True, ge=1)]
    confirmed_reset: Literal[True]


def recovery_required(connection):
    """호출자가 잡은 DB 트랜잭션 안에서 주문 접수·인수 차단 여부를 읽는다."""
    row = connection.execute(
        'SELECT requires_reset FROM workcell_recovery WHERE id=1'
    ).fetchone()
    return bool(row['requires_reset'])


class RecoveryCheckpoint(ContractModel):
    """ROS와 공유하는 v1 재개 계약. 임의 획 중간 재개는 허용하지 않는다.

    top_pose는 BASE 위치 m 3값과 quaternion 4값, side_pose는 BASE mm·ZYZ deg
    6값이다. path_index는 주문의 TOP→SIDE 실행 순서에서 0부터 센 표면 번호다.
    """

    version: Literal[1]
    order_id: str
    stage: Literal['PREPARING', 'DRAWING', 'FINISHING']
    path_index: Annotated[int, Field(strict=True, ge=0, le=1)]
    surface: Literal['', 'TOP', 'SIDE']
    top_pose: Annotated[list[float], Field(min_length=7, max_length=7)]
    side_pose: Annotated[list[float], Field(max_length=6)]

    @model_validator(mode='after')
    def validate_stage(self):
        """비정상 좌표와 운반/그리기 단계가 섞인 체크포인트를 거부한다."""
        if not all(
            math.isfinite(value) for value in [*self.top_pose, *self.side_pose]
        ):
            raise ValueError('재개 기준 자세는 유한한 값이어야 합니다.')
        if len(self.side_pose) not in (0, 6):
            raise ValueError('옆면 기준 자세는 비어 있거나 6값이어야 합니다.')
        if self.stage == 'DRAWING' and self.surface not in ('TOP', 'SIDE'):
            raise ValueError('재개할 면이 없습니다.')
        if self.stage != 'DRAWING' and (self.path_index != 0 or self.surface):
            raise ValueError('운반 단계의 재개 정보가 잘못됐습니다.')
        return self


def recovery_snapshot(connection, stopped=False):
    """중단 위치·재개 가능 여부를 반환한다. 조회만으로 로봇 상태를 추정하지 않는다."""
    row = connection.execute(
        'SELECT revision,requires_reset,order_id,error_code,message,updated_at,reset_at,'
        'checkpoint_json,resume_requested FROM workcell_recovery WHERE id=1'
    ).fetchone()
    state = dict(row)
    state['requires_reset'] = bool(state['requires_reset'])
    state['resume_requested'] = bool(state['resume_requested'])
    checkpoint = json.loads(state.pop('checkpoint_json') or 'null')
    order = connection.execute(
        'SELECT state,phase FROM order_execution WHERE order_id=?', (state['order_id'],)
    ).fetchone()
    others = connection.execute(
        "SELECT order_id FROM order_execution WHERE order_id != ? AND state NOT IN "
        "('completed','failed','rejected','resolved') LIMIT 1", (state['order_id'],)
    ).fetchone()
    state['can_reset'] = bool(
        state['requires_reset']
        and not stopped
        and not state['resume_requested']
        and not others
        and order
        and order['state'] in ('paused', 'failed', 'resolved')
    )
    state['can_resume'] = state['can_reset'] and checkpoint is not None
    state['stopped_stage'] = (
        checkpoint['stage'] if checkpoint else (order['phase'] if order else '')
    )
    if checkpoint and state['error_code'] == 'RESUME_NOT_SAFE':
        state['stopped_label'] = '재개 전 안전 자세·빈 그리퍼·기본 TCP 확인'
    elif checkpoint and checkpoint['stage'] == 'DRAWING':
        surface = '윗면' if checkpoint['surface'] == 'TOP' else '옆면'
        state['stopped_label'] = f'{surface} 펜 파지'
    elif checkpoint:
        location = (
            '입고 준비' if checkpoint['stage'] == 'PREPARING'
            else '스테이션 회수·출고'
        )
        tool = (
            '따개 파지' if state['error_code'] == 'OPENER_MISSING'
            else '케이크 집기·무게 검사'
        )
        state['stopped_label'] = f'{location} / {tool}'
    else:
        state['stopped_label'] = '중단 단계 확인 필요 (자동 재개 불가)'
    return state


@router.get('/recovery')
def read_recovery(request: Request):
    """HMI·태블릿이 공유하는 복구 대기와 재개 요청 상태를 읽는다."""
    with request.app.state.database.connection() as connection:
        connection.execute('BEGIN')
        return recovery_snapshot(connection, request.app.state.robot_stop.stopped)


def checked_decision(connection, body, request):
    """HMI에 보인 실패 세대가 지금도 대기 중인지 같은 쓰기 트랜잭션에서 확인한다."""
    state = recovery_snapshot(connection, request.app.state.robot_stop.stopped)
    if not state['requires_reset'] or state['revision'] != body.revision:
        raise HTTPException(409, '복구 대상이 변경됐습니다. 상태를 다시 확인해주세요.')
    if not state['can_reset']:
        raise HTTPException(
            409, '진행·미확정 주문 또는 소프트웨어 정지를 먼저 확인해주세요.'
        )
    return state


@router.post('/resume')
def resume_order(body: ResetConfirmation, request: Request):
    """동일 주문을 저장된 단계부터 1회 인수하도록 예약한다. 새 주문은 계속 차단한다."""
    with request.app.state.database.connection() as connection:
        connection.execute('BEGIN IMMEDIATE')
        current = recovery_snapshot(connection, request.app.state.robot_stop.stopped)
        # 응답 유실 후 동일 버튼 요청이 재전송돼도 두 번째 인수를 만들지 않는다.
        if current['revision'] == body.revision and current['resume_requested']:
            return current
        state = checked_decision(connection, body, request)
        if not state['can_resume']:
            raise HTTPException(
                409, '검증된 재개 지점이 없습니다. 초기 자세 세팅을 선택하세요.'
            )
        checkpoint = connection.execute(
            'SELECT checkpoint_json FROM workcell_recovery WHERE id=1'
        ).fetchone()['checkpoint_json']
        timestamp = datetime.now(timezone.utc).isoformat()
        connection.execute(
            "UPDATE order_execution SET state='received',claim_token=NULL,resume_checkpoint=?,"
            "message='관리자 복구 확인, 중단 단계 재개 대기',updated_at=? WHERE order_id=?",
            (checkpoint, timestamp, state['order_id']),
        )
        connection.execute(
            'UPDATE workcell_recovery SET resume_requested=1 WHERE id=1'
        )
        connection.execute(
            'INSERT INTO workcell_logs(created_at,order_id,severity,message) VALUES (?,?,?,?)',
            (
                timestamp, state['order_id'], 'info',
                f"관리자 재개 요청 · {state['stopped_label']}",
            ),
        )
        connection.commit()
        return recovery_snapshot(connection, request.app.state.robot_stop.stopped)


@router.post('/initialize')
@router.post('/reset', include_in_schema=False)
def complete_reset(body: ResetConfirmation, request: Request):
    """관리자의 초기 자세 정리 확인 뒤 현재 주문을 최종 실패로 닫고 다음 접수를 연다.

    자동 로봇 이동·정지 해제는 하지 않는다. 이전 /reset도 같은 명시적 폐기 확인이며
    HMI는 initialize를 사용한다. 재개가 진행 중이면 주문을 중간 폐기할 수 없다.
    """
    with request.app.state.database.connection() as connection:
        connection.execute('BEGIN IMMEDIATE')
        state = checked_decision(connection, body, request)
        timestamp = datetime.now(timezone.utc).isoformat()
        connection.execute(
            "UPDATE order_execution SET state='failed',phase='FAILED',resume_checkpoint='',"
            "updated_at=? WHERE order_id=?", (timestamp, state['order_id']),
        )
        connection.execute(
            "UPDATE workcell_recovery SET requires_reset=0,reset_at=?,checkpoint_json='',"
            'resume_requested=0 WHERE id=1', (timestamp,),
        )
        connection.execute(
            'INSERT INTO workcell_logs(created_at,order_id,severity,message) VALUES (?,?,?,?)',
            (
                timestamp, state['order_id'], 'warning',
                '관리자 초기 자세 세팅 확인 · 주문 최종 실패 · 다음 접수 허용',
            ),
        )
        connection.commit()
        return recovery_snapshot(connection, request.app.state.robot_stop.stopped)


@router.get('/logs')
def read_logs(
    request: Request,
    limit: Annotated[int, Query(ge=1, le=100)] = 30,
    before_id: Annotated[int | None, Query(ge=1)] = None,
    after_id: Annotated[int | None, Query(ge=0)] = None,
):
    """고정 ID 커서로 이전/새 로그를 조회한다. 새 로그 추가 중에도 페이지가 밀리지 않는다.

    최초·이전 조회는 최신순, after_id는 누락 없이 순서대로 따라잡도록 오래된 순이다.
    브라우저가 관측하지 못한 중간 제작 이벤트도 서버 기록에서 읽을 수 있다.
    """
    if before_id is not None and after_id is not None:
        raise HTTPException(422, '이전/이후 커서는 하나만 지정하세요.')
    condition, parameters, order = '', [], 'DESC'
    if before_id is not None:
        condition, parameters = 'WHERE id < ?', [before_id]
    elif after_id is not None:
        condition, parameters, order = 'WHERE id > ?', [after_id], 'ASC'
    with request.app.state.database.connection() as connection:
        rows = connection.execute(
            'SELECT * FROM workcell_logs ' + condition + f' ORDER BY id {order} LIMIT ?',
            [*parameters, limit + 1],
        ).fetchall()
    return {
        'items': [dict(row) for row in rows[:limit]],
        'has_more': len(rows) > limit,
    }
