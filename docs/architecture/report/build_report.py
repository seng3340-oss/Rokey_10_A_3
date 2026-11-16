"""팀원이 구조·통신·필드 순서로 읽는 세로 A4 안내서 네 쪽을 생성한다."""

from html import escape
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def table(headers, rows, widths=None):
    """내용은 이스케이프하고 열 너비를 지정해 긴 타입명이 표를 밀지 않게 한다."""
    columns = ""
    if widths:
        columns = "<colgroup>" + "".join(
            f'<col style="width:{width}%">' for width in widths
        ) + "</colgroup>"
    heading = "".join(f"<th>{escape(item)}</th>" for item in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{escape(item)}</td>" for item in row) + "</tr>"
        for row in rows
    )
    return f"<table>{columns}<thead><tr>{heading}</tr></thead><tbody>{body}</tbody></table>"


def page(number, title, subtitle, content):
    """페이지마다 제목·읽는 목적·본문·쪽 번호를 같은 위치에 배치한다."""
    return f'''<section class="page"><header>
    <small>케이크 레터링 프로젝트 · 팀 공유용</small><h1>{title}</h1>
    <p>{subtitle}</p></header><main>{content}</main>
    <footer><span>2026.11.16 · 시스템 구조와 ROS 통신 · v0.3</span>
    <span>{number} / 4</span></footer></section>'''


# 화살표는 요청 방향이다. 경로 계산은 반드시 TM으로 결과를 돌려준다.
# 데이터가 DP에서 RE로 직접 전달되는 것으로 오해하지 않도록 분기해서 표현한다.
architecture = '''<div class="architecture">
<div class="node web"><b>고객 웹 · 태블릿</b><span>그림 입력 / 3D 확인</span></div>
<div class="arrow">↓ HTTP · POST /api/drawing</div>
<div class="node"><b>FastAPI 백엔드</b><span>검증 / 주문 번호 / SQLite 저장</span></div>
<div class="arrow planned">↓ 내부 HTTP · 주문 인수 / 상태 기록</div>
<div class="ros"><label>ROS 2 · 브릿지 구현 / 업무 노드 3개 구현 전</label>
<div class="node"><b>브릿지 · BR</b><span>ros_bridge_node · 원본 좌표 전달 / 상태 수신</span></div>
<div class="arrow">↓ CakeOrder Action</div>
<div class="branch"><div class="node"><b>작업 관리 · TM</b><span>task_manager_node</span></div>
<div class="side-arrow">⇄<small>GenerateTrajectory<br>Service</small></div>
<div class="node"><b>경로 계산 · DP</b><span>drawing_path_node</span></div></div>
<div class="arrow">↓ PickAndPlace / GraspTool / DrawOnCake · Action</div>
<div class="node executor"><b>로봇 실행 · RE</b><span>robot_executor_node</span></div></div>
<div class="arrow">↓ 이동·TCP 명령 / RG2 제어 명령</div>
<div class="driver-row"><div class="node"><b>두산 드라이버 → M0609</b></div>
<div class="node"><b>OnRobot 드라이버 → RG2</b></div></div></div>
<p class="caption">상태는 반대 방향으로 돌아옵니다. RE → TM은 Action의 진행·결과,
TM → 백엔드는 CakeOrder의 진행·결과와 /task_status Topic을 사용합니다.</p>'''

roles = table(["구성", "맡는 일"], [
    ("백엔드", "입력 검증·원본 저장, 단일 주문·중복 요청·상태 기록"),
    ("BR · 브릿지", "주문 인수 → CakeOrder 전달, 진행·결과를 백엔드에 기록"),
    ("TM · 작업 관리", "주문 수신, 고정 설정 확인, 준비 → 그리기 → 정리 순서 관리"),
    ("DP · 경로 계산", "픽셀 좌표를 윗면·옆면의 3D 목표점으로 변환. 로봇 이동은 하지 않음"),
    ("RE · 로봇 실행", "TM의 작업을 실행. 이동·그리퍼·TCP 호출을 한곳에서 관리"),
    ("드라이버", "기존 두산·OnRobot 패키지. 장비와 실제 통신하는 부분"),
], [24, 76])
structure = architecture + '<h2>누가 무엇을 담당하나</h2>' + roles + '''
<div class="note"><b>현재 상태</b> 웹 → 백엔드 저장은 동작합니다. 브릿지와 내부 HTTP는 구현했습니다.
TM·DP·RE는 아직 구현 전이며 실제 동작 연결은 안 됐습니다. HMI는 정적 화면입니다.</div>'''


# 통신 방식과 실제 타입을 같은 쪽에 배치해 코드 담당자가 연결점을 찾게 한다.
communication = '''<div class="types">
<div><b>Action · 오래 걸리는 작업</b><p>목표(Goal)를 보내고, 진행(Feedback)을 받다가 결과(Result)를 받습니다. 제작·운반·그리기에 사용합니다.</p></div>
<div><b>Service · 한 번 요청·응답</b><p>입력(Request)을 보내 계산·설정 결과(Response)를 받습니다. 경로 계산·TCP 설정에 사용합니다.</p></div>
<div><b>Topic · 상태 알림</b><p>발행자가 상태를 내보내면 구독자가 받습니다. 개별 요청에 대한 응답은 아닙니다.</p></div></div>
<h2>연결별 인터페이스</h2>
<p class="caption">화살표는 요청 또는 발행 방향입니다. Action·Service 응답은 반대 방향입니다.</p>''' + table(
    ["종류 / 타입", "보내는 쪽 → 받는 쪽", "용도"], [
        ("Action\nCakeOrder", "브릿지 BR → TM", "전체 제작 주문. 진행 단계·최종 결과 회신"),
        ("Service\nGenerateTrajectory", "TM → DP", "그림 좌표를 3D 경로로 계산"),
        ("Action\nPickAndPlace", "TM → RE", "케이크 픽업·작업대 배치·출고"),
        ("Action\nGraspTool", "TM → RE", "펜 파지 또는 반납"),
        ("Action\nDrawOnCake", "TM → RE", "계산된 경로로 그리기"),
        ("Topic\nTaskStatus", "TM → 브릿지 BR", "/task_status로 작업 상태 알림"),
        ("Service\nSwitchTCP", "실행 측 보조 기능\n호출·제공 노드 미정", "활성 TCP 변경. 타입만 정의"),
        ("Service\nSetCompliance", "실행 측 보조 기능\n호출·제공 노드 미정", "순응제어 설정. 현재 미사용"),
    ], [29, 35, 36]
) + '''<h2>주문 하나가 처리되는 순서</h2>
<div class="steps">① 웹 그림 전송 → ② 백엔드 저장·주문 전달 → ③ TM이 DP에 경로 계산 요청<br>
→ ④ DP가 TM에 경로 반환 → ⑤ TM이 RE에 그리기 요청 → ⑥ 진행·결과 회신</div>
<p class="caption">케이크 운반·펜 자동 파지·출고는 확장할 공정입니다. 고정 케이크 MVP에서는 드로잉 연결을 먼저 검증합니다.</p>
<div class="note"><b>타입명 ≠ 통신 주소</b> CakeOrder 등은 데이터 형식의 이름입니다.
Action·Service의 실제 ROS 이름과 네임스페이스는 노드 구현 시 정합니다.
백엔드↔브릿지는 내부 HTTP입니다. 기본 제작 Action 주소는 /cake/order이며 설정으로 변경합니다.</div>'''


# 메시지는 독립 통신 채널이 아니라 Action·Service·Topic에 담기는 데이터라는 점을 명시한다.
data = '''<h2>웹에서 보내는 원본</h2><p><b>POST /api/drawing</b> · JSON · 현재 한 요청에 한 면</p>''' + table(
    ["필드", "의미"], [
        ("canvas_width / canvas_height", "캔버스 가로·세로 해상도 (px)"),
        ("drawing_surface", "top: 윗면 / side: 옆면"),
        ("strokes[] → stroke_id", "획 배열과 0부터 시작하는 획 번호"),
        ("strokes[] → points[] → x / y", "획 안의 원본 픽셀 좌표. 왼쪽 위가 원점"),
    ], [47, 53]
) + '''<h2>ROS 메시지 6종</h2>
<p class="caption">Message(.msg)는 데이터 묶음입니다. Topic에 발행하거나 Action·Service의 필드로 넣습니다.</p>''' + table(
    ["메시지 / 쓰이는 곳", "들어 있는 필드와 의미"], [
        ("DrawingPath\nCakeOrder.drawing_paths[]", "canvas_width, canvas_height: 해상도\nsurface: TOP / SIDE\nstrokes[]: Stroke2D 획 목록"),
        ("Stroke2D\nDrawingPath.strokes[]", "stroke_id: 획 번호\npoints[]: 원본 x/y 픽셀 좌표, z=0. 자동 끝점 연결 없음"),
        ("CakeParams\nCakeOrder.cake_params", "radius / bottom_radius: 윗면·밑면 반지름 (m)\nheight, board_width/depth/thickness: 실측 치수 (m)\ncenter: BASE 윗면 중심, reference_frame: base\ntool_orientation: 접촉 TCP 자세, orientation: 평면 방향(rad)"),
        ("Waypoint3D\n경로 계산 응답·그리기 Goal", "pose: 위치 (m)·자세 (쿼터니언)\nspeed: 속도 (mm/s), pen_down: 펜 내림 여부\ncontact_force: 접촉력 (N), 높이 고정 시 미사용"),
        ("WorkspaceCoords\n고정 작업 위치 묶음", "cake_pickup_pose, workstation_pose, tool_holder_pose, delivery_pose\ncake_center, cake_radius, cake_height, cake_orientation\napproach_height, drawing_z_offset"),
        ("TaskStatus\n/task_status Topic", "order_id: 주문, phase: 단계, progress_pct: 진행률\ncurrent_command: 명령, message: 설명\ntimestamp: 기록 시각"),
    ], [35, 65]
) + '''<div class="note"><b>좌표 규칙</b> 화면은 px, ROS Pose는 m·쿼터니언입니다.
RE에서 두산 API의 mm·ZYZ 각도(degree)로 변환합니다. center는 사용자가 접촉 확인한 TCP의 윗면 중심입니다.</div>'''


# 상세 필드도 요청·진행·결과를 구분해 처음 보는 팀원이 방향을 혼동하지 않게 한다.
contracts = '''<h2>Action 4종 · Goal / Feedback / Result</h2>''' + table(
    ["타입", "요청 · 진행 · 결과 데이터"], [
        ("CakeOrder", "목표: order_id, drawing_paths[], cake_params, tool_type\n진행: current_phase, progress_pct, message\n결과: success, error_code, error_msg, elapsed_time_sec"),
        ("PickAndPlace", "목표: pick_pose, place_pose, approach_height, grip_width\n진행: phase, progress_pct\n결과: success, actual_place_pose, error_code"),
        ("GraspTool", "목표: tool_id, action(GRASP / RELEASE), tool_pose, approach_dir\n진행: phase, progress_pct\n결과: success, error_code"),
        ("DrawOnCake", "목표: waypoints[], drawing_surface, contact_force, speed\n진행: current_stroke_idx, total_strokes, progress_pct, phase\n결과: success, completed_strokes, error_code"),
    ], [26, 74]
) + '''<h2>Service 3종 · Request / Response</h2>''' + table(
    ["타입", "요청 · 응답 데이터"], [
        ("GenerateTrajectory", "요청: canvas_width, canvas_height, strokes_2d[], surface_type, cake_pose, cake_radius, cake_bottom_radius, cake_height, tool_orientation\n응답: success, waypoints_3d[], pen_states[], estimated_duration_sec, error_msg"),
        ("SwitchTCP", "요청: tcp_name\n응답: success, current_tcp, error_msg"),
        ("SetCompliance\n현재 미사용", "요청: enable, axis[6], stiffness[6]\n응답: success, error_code"),
    ], [26, 74]
) + '''<h2>필드를 읽는 기준</h2>
<p><b>success</b>는 해당 요청의 성공 여부, <b>error_code / error_msg</b>는 오류 코드·설명입니다.
<b>phase / current_phase</b>는 단계, <b>progress_pct</b>는 0~100% 진행률입니다.
<b>pose</b>는 위치와 자세, <b>[]</b>는 배열입니다.<br>
<b>TCP</b>는 펜촉 등 도구의 작업 기준점, <b>grip_width</b>는 그리퍼 폭(mm)입니다.</p>
<div class="note"><b>구현 파일</b> ws_cobot1/src/cake_decorator_interfaces/<br>
msg/ 6개 · srv/ 3개 · action/ 4개. 브릿지만 구현했고 TM·DP·RE는 아직 없습니다.<br>
<b>추가 합의</b> 두 면 주문 묶음, 속도 우선순위,
실행 중 취소·복구, 상태 문자열·HMI 연결 규격.</div>'''


# HTML은 CSS까지 포함하므로 팀원 PC에서 파일만 열어도 동일하게 볼 수 있다.
style = (ROOT / "report.css").read_text()
sections = [
    ("한눈에 보는 시스템", "웹에서 그린 좌표가 로봇에 도착하기까지", structure),
    ("누가 누구와 통신하나", "Topic · Service · Action과 프로젝트 연결표", communication),
    ("어떤 데이터를 주고받나", "웹 원본과 ROS 메시지 필드", data),
    ("요청과 응답에 담기는 값", "생성된 Action·Service 파일 기준", contracts),
]
html = (
    '<!doctype html><html lang="ko"><meta charset="utf-8">'
    '<title>케이크 시스템 구조와 ROS 통신</title>'
    f"<style>{style}</style><body>"
)
html += "".join(
    page(index, title, subtitle, body)
    for index, (title, subtitle, body) in enumerate(sections, start=1)
)
html += "</body></html>"
(ROOT / "system-contract.html").write_text(html)
