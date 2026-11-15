"""연결 구조와 핵심 데이터 필드를 세로 A4 세 쪽으로 정리한다."""

from html import escape
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def table(rows):
    """필드명·의미를 두 열로 표시한다."""
    return '<table><thead><tr><th>필드</th><th>의미</th></tr></thead><tbody>' + ''.join(
        f'<tr><td>{escape(name)}</td><td>{escape(description)}</td></tr>'
        for name, description in rows
    ) + '</tbody></table>'


def page(number, title, content):
    """같은 여백과 글자 크기로 한 페이지를 구성한다."""
    return f'<section class="page"><header><small>케이크 레터링 프로젝트</small><h1>{title}</h1></header><main>{content}</main><footer>2026.11.15 · 구조와 데이터 필드<span>{number} / 3</span></footer></section>'


# 첫 쪽은 누가 어떤 일을 맡는지만 설명한다.
structure = '''<h2>전체 연결 구조</h2>
<div class="flow">고객 웹 · 태블릿<div>↓ 그림 좌표 / HTTP JSON</div>FastAPI 백엔드<div>↓ 제작 주문 / CakeOrder Action</div>작업 관리 노드 · task_manager_node<div>↔ 경로 계산 요청 / GenerateTrajectory Service</div>경로 계산 노드 · drawing_path_node</div>
<p>작업 관리 노드는 계산된 경로를 <b>로봇 실행 노드</b>로 보냅니다.</p>
<div class="flow">로봇 실행 노드 · robot_executor_node<div>↓ 이동 · TCP · 그리퍼 명령</div>두산 드라이버 / RG2 드라이버<div>↓</div>M0609 로봇팔 + RG2 그리퍼</div>
<h2>각 부분의 역할</h2>''' + table([
    ('고객 웹', '그림 입력, 윗면·옆면 선택, 3D 미리보기'),
    ('백엔드', '입력 검증, 주문 번호 생성, SQLite 저장, ROS 주문 전달'),
    ('작업 관리 노드', '주문을 받아 준비 → 그리기 → 정리 순서 관리'),
    ('경로 계산 노드', '화면의 픽셀 좌표를 케이크 위의 로봇 경로로 변환'),
    ('로봇 실행 노드', '경로를 따라 이동하고 펜·그리퍼·TCP 제어'),
    ('드라이버', 'ROS 명령을 실제 로봇·그리퍼 장비에 전달'),
]) + '<p class="note">현재 웹 → 백엔드 저장까지 연결되어 있습니다. ROS 주문 전달·실행 노드는 구현 전이며, HMI는 정적 화면입니다.</p>'


# 둘째 쪽은 웹 입력과 ROS 주문에서 사용하는 필드를 정리한다.
order = '<h2>1. 웹 → 백엔드</h2><p><b>POST /api/drawing</b> · JSON · 한 요청에 한 면의 그림</p>' + table([
    ('canvas_width / canvas_height', '캔버스 가로·세로 해상도 (px)'),
    ('drawing_surface', '그리는 면: top(윗면) / side(옆면)'),
    ('strokes[]', '손을 떼기 전까지 그린 획들의 배열'),
    ('strokes[].stroke_id', '획 순서 번호. 0부터 시작'),
    ('strokes[].points[]', '해당 획의 좌표점 배열'),
    ('points[].x / y', '화면 왼쪽 위를 원점으로 한 픽셀 좌표'),
]) + '<h2>2. 백엔드 → 작업 관리 노드</h2><p><b>CakeOrder Action</b> · 제작 주문</p>' + table([
    ('order_id', '백엔드가 생성한 주문 번호'),
    ('drawing_paths[]', '그릴 면과 원본 획 데이터'),
    ('cake_params', '케이크 반지름·높이·중심·방향'),
    ('tool_type', '사용할 도구 종류'),
]) + '<h2>drawing_paths[] 안의 데이터</h2>' + table([
    ('canvas_width / canvas_height', '웹에서 사용한 캔버스 해상도 (px)'),
    ('surface', '그릴 면: TOP / SIDE'),
    ('strokes[].stroke_id', '웹에서 전달한 획 번호'),
    ('strokes[].points[]', '원본 x/y 픽셀 좌표. ROS Point32의 z는 0'),
    ('strokes[].is_closed', 'false 사용. 획의 끝과 시작을 자동으로 잇지 않음'),
]) + '<p class="note">캔버스 크기와 원본 좌표를 함께 전달해야 실제 케이크 크기에 맞게 변환할 수 있습니다.</p>'


# 셋째 쪽은 좌표 계산 결과와 상태 통신에 필요한 필드만 담는다.
execution = '<h2>3. 작업 관리 ↔ 경로 계산 노드</h2><p><b>GenerateTrajectory Service</b></p>' + table([
    ('요청: canvas_width / height', '원본 캔버스 해상도 (px)'),
    ('요청: strokes_2d / surface_type', '원본 획과 그릴 면 (TOP / SIDE)'),
    ('요청: cake_pose', '케이크 위치와 방향'),
    ('요청: cake_radius / height', '케이크 반지름과 높이 (m)'),
    ('응답: success / error_msg', '계산 성공 여부와 오류 설명'),
    ('응답: waypoints_3d[]', '계산된 로봇 목표점 배열'),
    ('응답: pen_states[]', '목표점별 펜 내림·올림 상태'),
    ('응답: estimated_duration_sec', '예상 작업 시간 (초)'),
]) + '<h2>4. 로봇이 따라갈 목표점 · Waypoint3D</h2>' + table([
    ('pose', '위치 (m) + 자세 (쿼터니언)'),
    ('speed', '이동 속도 (mm/s)'),
    ('pen_down', 'true: 그리기 / false: 펜을 뗀 이동'),
    ('contact_force', '목표 접촉력 (N). 높이 고정 방식에서는 사용 안 함'),
]) + '<h2>5. 작업 상태 · TaskStatus</h2><p><b>/task_status Topic</b> · 작업 관리 노드 → 백엔드</p>' + table([
    ('order_id / phase', '주문 번호 / 현재 작업 단계'),
    ('progress_pct', '진행률 (0~100%)'),
    ('current_command / message', '현재 실행 명령 / 설명 또는 오류 안내'),
    ('timestamp', '상태가 기록된 시각'),
]) + '<p class="note">위치 기준점, 두 면의 주문 묶음, 속도 우선순위, 취소·복구 방식은 추가 합의가 필요합니다. 상태 화면 연결도 아직 구현 전입니다.</p>'

style = (ROOT / 'report.css').read_text()
html = '<!doctype html><html lang="ko"><meta charset="utf-8"><title>케이크 시스템 구조와 데이터 필드</title><style>' + style + '</style><body>'
html += page(1, '시스템 구조', structure)
html += page(2, '그림과 주문 데이터', order)
html += page(3, '로봇 경로와 작업 상태', execution)
html += '</body></html>'
(ROOT / 'system-contract.html').write_text(html)
