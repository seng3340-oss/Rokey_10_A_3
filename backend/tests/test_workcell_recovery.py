"""실패 잠금·관리자 확인·로그 페이지의 영속성과 동시 요청 계약을 검사한다."""

from concurrent.futures import ThreadPoolExecutor

from fastapi.testclient import TestClient
import pytest

from app.main import create_app
from test_bridge_orders import DRAWING

HEADERS = {'Authorization': 'Bearer test-secret'}


def fail_order(client, code='CAKE_WEIGHT_FAILED', checkpoint=None):
    """인증된 브릿지 흐름으로 실패를 확정하고 재전송 검사에 쓸 원본 이벤트를 반환한다."""
    order = client.post('/api/drawing', json=DRAWING).json()['order_id']
    claim = client.post('/internal/bridge/claim', headers=HEADERS).json()
    event = {}
    for kind in ('dispatching', 'accepted', 'failed'):
        event = {
            'event_id': f'{order}-{kind}',
            'claim_token': claim['claim_token'],
            'kind': kind,
            'phase': 'FAILED' if kind == 'failed' else 'PREPARING',
            'error_code': code if kind == 'failed' else '',
            'message': '입고함 집기 재시도 1/1 후에도 케이크 없음. 안전 자세에서 대기중'
            if kind == 'failed' else '케이크 준비',
        }
        if kind == 'failed' and checkpoint is not None:
            event['resume_checkpoint'] = {**checkpoint, 'order_id': order}
        response = client.post(
            f'/internal/bridge/orders/{order}/events', headers=HEADERS, json=event,
        )
        assert response.status_code == 200
    return order, event


@pytest.mark.parametrize('code', ['CAKE_WEIGHT_FAILED', 'PEN_GRASP_FAILED', 'OPENER_MISSING'])
def test_failure_gate_survives_restart_and_only_explicit_reset_opens_orders(tmp_path, code):
    """예외 종류와 무관하게 실패 확정 뒤 재시작·다른 클라이언트로 접수를 우회할 수 없다."""
    path = tmp_path / 'orders.sqlite3'
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        order, failure = fail_order(client, code)
        locked = client.get('/api/workcell/recovery').json()
        assert locked['requires_reset'] and locked['can_reset']
        assert locked['order_id'] == order and locked['error_code'] == code
        assert client.post('/api/drawing', json=DRAWING).status_code == 409
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        assert client.get('/api/workcell/recovery').json() == locked
        with ThreadPoolExecutor(max_workers=4) as pool:
            blocked = list(pool.map(lambda _: client.post('/api/drawing', json=DRAWING), range(8)))
        assert all(response.status_code == 409 for response in blocked)
        assert client.post('/internal/bridge/claim', headers=HEADERS).status_code == 204
        assert client.post('/api/workcell/reset', json={
            'revision': locked['revision'], 'confirmed_reset': False,
        }).status_code == 422
        assert client.post('/api/workcell/reset', json={
            'revision': locked['revision'] + 1, 'confirmed_reset': True,
        }).status_code == 409
        reset = client.post('/api/workcell/reset', json={
            'revision': locked['revision'], 'confirmed_reset': True,
        })
        assert reset.status_code == 200
        assert not reset.json()['requires_reset']
        assert reset.json()['reset_at']
        assert client.get(f'/api/orders/{order}').json()['status'] == 'failed'
        # 네트워크가 늦어 같은 실패가 다시 와도 해제한 잠금을 다시 걸지 않는다.
        assert client.post(
            f'/internal/bridge/orders/{order}/events', headers=HEADERS, json=failure,
        ).status_code == 200
        assert not client.get('/api/workcell/recovery').json()['requires_reset']
        assert client.post('/api/drawing', json=DRAWING).status_code == 201


def test_old_reset_cannot_release_a_new_failure_or_software_stop(tmp_path):
    """이전 주문 화면의 늦은 클릭과 소프트웨어 정지 해제를 구분한다."""
    with TestClient(create_app(tmp_path / 'orders.sqlite3', bridge_token='test-secret')) as client:
        fail_order(client)
        first = client.get('/api/workcell/recovery').json()
        assert client.post('/api/workcell/reset', json={
            'revision': first['revision'], 'confirmed_reset': True,
        }).status_code == 200
        fail_order(client)
        second = client.get('/api/workcell/recovery').json()
        assert second['revision'] > first['revision']
        assert client.post('/api/workcell/reset', json={
            'revision': first['revision'], 'confirmed_reset': True,
        }).status_code == 409
        client.post('/api/robot/stop')
        assert not client.get('/api/workcell/recovery').json()['can_reset']
        assert client.post('/api/workcell/reset', json={
            'revision': second['revision'], 'confirmed_reset': True,
        }).status_code == 409
        assert client.get('/api/robot/stop').json()['stopped']
        assert client.post('/api/drawing', json=DRAWING).status_code == 409


def test_cursor_logs_are_persistent_deduplicated_and_keep_insertion_boundaries(tmp_path):
    """같은 이벤트는 한 번만 저장하고 새 로그 추가 중에도 이전 페이지가 밀리지 않는다."""
    path = tmp_path / 'orders.sqlite3'
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        order, event = fail_order(client)
        first = client.get('/api/workcell/logs?limit=2').json()
        assert first['has_more']
        assert len(first['items']) == 2
        oldest = first['items'][-1]['id']
        newest = first['items'][0]['id']
        client.post(f'/internal/bridge/orders/{order}/events', headers=HEADERS, json=event)
        recovery = client.get('/api/workcell/recovery').json()
        client.post('/api/workcell/reset', json={
            'revision': recovery['revision'], 'confirmed_reset': True,
        })
        older = client.get(f'/api/workcell/logs?before_id={oldest}&limit=2').json()
        assert not older['has_more']
        assert all(item['id'] < oldest for item in older['items'])
        newer = client.get(f'/api/workcell/logs?after_id={newest}&limit=2').json()
        assert len(newer['items']) == 1
        assert '초기 자세 세팅 확인' in newer['items'][0]['message']
        all_rows = client.get('/api/workcell/logs?limit=100').json()['items']
        assert len(all_rows) == 4
        assert sum(item['severity'] == 'error' for item in all_rows) == 1
        assert client.get('/api/workcell/logs?before_id=2&after_id=1').status_code == 422
        assert client.get('/api/workcell/logs?limit=101').status_code == 422
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        assert client.get('/api/workcell/logs?limit=100').json()['items'] == all_rows


@pytest.mark.parametrize('stage,index,surface,code', [
    ('PREPARING', 0, '', 'CAKE_WEIGHT_FAILED'),
    ('DRAWING', 0, 'TOP', 'PEN_GRASP_FAILED'),
    ('FINISHING', 0, '', 'OPENER_MISSING'),
])
def test_resume_claim_keeps_same_order_checkpoint_and_blocks_new_orders(tmp_path, stage, index, surface, code):
    """관리자 재개는 같은 주문 한 건만 인수시키고 완료 전에는 새 주문을 받지 않는다."""
    checkpoint = {
        'version': 1, 'order_id': '', 'stage': stage,
        'path_index': index, 'surface': surface,
        'top_pose': [0.399, -0.053, 0.511, 0.0, 1.0, 0.0, 0.0], 'side_pose': [],
    }
    path = tmp_path / 'resume.sqlite3'
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        order, old_event = fail_order(client, code, checkpoint)
        paused = client.get(f'/api/orders/{order}').json()
        assert paused['status'] == 'paused'
        locked = client.get('/api/workcell/recovery').json()
        assert locked['can_resume'] and locked['stopped_stage'] == stage
    # 서버 재시작 후에도 같은 주문·같은 기준 자세로 재개한다.
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        body = {'revision': locked['revision'], 'confirmed_reset': True}
        resumed = client.post('/api/workcell/resume', json=body)
        assert resumed.status_code == 200 and resumed.json()['resume_requested']
        assert client.post('/api/workcell/resume', json=body).status_code == 200
        assert client.post('/api/workcell/initialize', json=body).status_code == 409
        claim = client.post('/internal/bridge/claim', headers=HEADERS).json()
        assert claim['order_id'] == order
        assert claim['resume_checkpoint'] == {**checkpoint, 'order_id': order}
        assert claim['claim_token'] != old_event['claim_token']
        assert client.post('/internal/bridge/claim', headers=HEADERS).status_code == 204
        assert client.post('/api/drawing', json=DRAWING).status_code == 409
        # 전 시도의 늦은 결과는 새 시도의 상태를 덮지 못한다.
        assert client.post(f'/internal/bridge/orders/{order}/events', headers=HEADERS, json=old_event).status_code == 409
        for kind in ('dispatching', 'accepted', 'completed'):
            response = client.post(f'/internal/bridge/orders/{order}/events', headers=HEADERS, json={
                'event_id': f'resume-{kind}', 'claim_token': claim['claim_token'], 'kind': kind,
            })
            assert response.status_code == 200
        assert client.get('/api/orders').json()['total'] == 1
        assert client.get(f'/api/orders/{order}').json()['status'] == 'completed'
        assert not client.get('/api/workcell/recovery').json()['requires_reset']
        assert client.post('/api/drawing', json=DRAWING).status_code == 201


def test_non_checkpoint_failure_cannot_be_resumed(tmp_path):
    """중간 잉크 경로 등 안전한 재개 지점이 없는 실패는 초기 확인 후 종료만 허용한다."""
    with TestClient(create_app(tmp_path / 'no-checkpoint.sqlite3', bridge_token='test-secret')) as client:
        order, _ = fail_order(client, 'MOVE_FAILED_AT_6')
        locked = client.get('/api/workcell/recovery').json()
        assert not locked['can_resume']
        body = {'revision': locked['revision'], 'confirmed_reset': True}
        assert client.post('/api/workcell/resume', json=body).status_code == 409
        assert client.post('/api/workcell/initialize', json=body).status_code == 200
        assert client.get(f'/api/orders/{order}').json()['status'] == 'failed'


def test_existing_execution_table_migrates_without_losing_orders(tmp_path):
    """paused가 없는 구버전 CHECK 제약과 데이터가 새 접수·복구 계약으로 이관된다."""
    import sqlite3
    path = tmp_path / 'legacy-execution.sqlite3'
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        order = client.post('/api/drawing', json=DRAWING).json()['order_id']
    with sqlite3.connect(path) as connection:
        schema = connection.execute("SELECT sql FROM sqlite_master WHERE name='order_execution'").fetchone()[0]
        schema = schema.replace('order_execution', 'legacy_execution', 1).replace(",'paused'", '')
        schema = '\n'.join(line for line in schema.splitlines() if 'resume_checkpoint' not in line)
        connection.execute(schema)
        columns = [row[1] for row in connection.execute('PRAGMA table_info(legacy_execution)')]
        names = ','.join(columns)
        connection.execute(f'INSERT INTO legacy_execution({names}) SELECT {names} FROM order_execution')
        connection.execute('DROP TABLE order_execution')
        connection.execute('ALTER TABLE legacy_execution RENAME TO order_execution')
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        assert client.get(f'/api/orders/{order}').json()['drawing'] == DRAWING
        claim = client.post('/internal/bridge/claim', headers=HEADERS).json()
        for kind in ('dispatching', 'accepted', 'failed'):
            assert client.post(f'/internal/bridge/orders/{order}/events', headers=HEADERS, json={
                'event_id': f'legacy-{kind}', 'claim_token': claim['claim_token'], 'kind': kind,
            }).status_code == 200
        assert client.get(f'/api/orders/{order}').json()['status'] == 'paused'


def test_restart_before_resume_claim_requires_fresh_operator_confirmation(tmp_path):
    """재개 예약만 한 뒤 서버를 재시작하면 자동으로 로봇 작업을 보내지 않는다."""
    path = tmp_path / 'queued-resume.sqlite3'
    checkpoint = {
        'version': 1, 'order_id': '', 'stage': 'PREPARING', 'path_index': 0, 'surface': '',
        'top_pose': [0.399, -0.053, .511, 0., 1., 0., 0.], 'side_pose': [],
    }
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        order, _ = fail_order(client, checkpoint=checkpoint)
        old = client.get('/api/workcell/recovery').json()
        assert client.post('/api/workcell/resume', json={
            'revision': old['revision'], 'confirmed_reset': True,
        }).status_code == 200
    with TestClient(create_app(path, bridge_token='test-secret')) as client:
        state = client.get('/api/workcell/recovery').json()
        assert state['requires_reset'] and state['can_resume']
        assert not state['resume_requested']
        assert state['revision'] > old['revision']
        assert client.get(f'/api/orders/{order}').json()['status'] == 'paused'
        assert client.post('/internal/bridge/claim', headers=HEADERS).status_code == 204
