# 확정 원통형 케이크와 원본 픽셀 통신 규격

2026-11-16 확정: 케이크는 3D 프린트 원통형이며 지름100mm·높이80mm입니다.
플라스틱 원뿔대의 이전 치수와 mm 입력형 DP 인터페이스는 더 이상 사용하지 않습니다.
받침판은 변경 요청이 없어 기존179×179×2mm를 유지합니다.

## 역할과 기준

```text
웹 → 백엔드 원본 저장 → 브릿지 → TM → DP(점 선별·px→mm→3D) → TM → RE
```

- 웹·백엔드·브릿지·TM은 원본 획 좌표와 캔버스 해상도를 보존합니다.
- Stroke2D: uint32 stroke_id, Point32[] points. x/y는 원본 px이며 z는 0입니다.
- 폐곡선 여부 필드는 없으며 입력하지 않은 끝점 연결을 하지 않습니다.
- 원통 반지름은 하나만 사용합니다. 위·아랫면 반지름을 따로 전달하지 않습니다.
- 케이크 위치는 BASE 기준 **윗면 중심**입니다. 바닥 중심이 아닙니다.
- 실물의 윗면 중심은 기존처럼 사용자가 접촉 확인한 활성 펜촉 TCP에서 기록합니다.
- 케이크 높이나 판 두께를 윗면 중심 Z에 다시 더하지 않습니다.
- ROS 위치·치수는 m, 자세는 쿼터니언, 속도는 mm/s입니다.

## 설정 파일

`config/test_cake.json`은 웹·백엔드가 공유하는 원통 지름과 판 치수입니다.
`ws_cobot1/src/cake_decorator/config/workspace_config.yaml`은 원래의 노드별
`ros__parameters` 구조이며 DP의 기본 반지름0.05m·높이0.08m와 변환 설정을 기록합니다.
치수 변경 시 두 설정을 함께 맞춥니다. 실제 위치는 요청으로 전달하며 미측정 좌표는 0으로 채우지 않습니다.
YAML 접근 거리는 m입니다. 접근 높이0.01은10mm이며 새 원뿔대 코드의 mm 설정과 혼용하지 않습니다.

RViz 케이크·판 형상은 다음 명령으로 공유 JSON에서 재생성합니다.

```bash
python3 scripts/generate_test_cake_model.py
```

지그 위치는 기존 임시 배치를 유지합니다. 판 아래가 지그 기준 Z=60mm인 현재 배치에서
케이크 상면은 BASE Z=142mm입니다. 실제 로봇의 영점으로 이 가상 높이를 사용하지 않습니다.

## TM → DP: GenerateTrajectory.srv

| 요청 필드 | 의미 |
|---|---|
| canvas_width / canvas_height | 원본 캔버스 해상도(px) |
| strokes_2d | Stroke2D 배열. 순서·원본 픽셀 그대로 |
| surface_type | TOP 또는 SIDE |
| cake_pose | BASE 윗면 중심(m)·케이크 프레임 방향 |
| cake_radius | 원통 반지름(m), 기본0.05 |
| cake_height | 원통 높이(m), 기본0.08 |
| order_id | 요청을 보낸 주문 번호 |
| operation_speed_percent | 기본 경로 속도 대비 백분율 |

응답은 success, waypoints_3d, pen_states, estimated_duration_sec, error_msg를 유지합니다.
`CakeOrder → DrawingPath → Stroke2D` 입력도 같은 원본 픽셀 규칙을 사용합니다.
mm 스트로크는 이 서비스에서 사용하지 않습니다.

## 화면 좌표 규칙

웹 원본 캔버스는1024×768px입니다. 실제 그림 영역은 캔버스 전체와 다릅니다.

- 윗면 원 중심=(512,384), 반지름320px.
- 윗면 변환 예: local_x=(pixel_x-512)/320×50mm,
  local_y=-(pixel_y-384)/320×50mm. 화면 Y 아래 방향과 BASE Y 방향을 구분합니다.
- 옆면 띠는 x=64..960, 너비896px, 높이896×80/(π×100)≈228.16565px.
- 옆면 상단 y≈269.91717, 하단 y≈498.08283.
- u=(pixel_x-64)/896, v=(pixel_y-269.91717)/228.16565.
  θ=2πu, z=윗면 중심Z-v×80mm를 기본으로 두고 케이크 프레임 방향을 적용합니다.
- 유효 영역·점 선별·펜 접근/이탈·자세·도달 가능성 검사는 DP·실행 측에서 검증합니다.

이 규칙은 전달할 픽셀을 미리 정규화하라는 뜻이 아닙니다. 변환은 DP에서 수행합니다.

## 윗면 가상 검증 범위

윗면 서비스는 1024×768px 캔버스의 가운데 640px 지름 원을 기준으로 합니다.
펜 접촉 여부를 응답 배열과 개별 웨이포인트에 동일하게 넣습니다.
옆면의 화면 띠와 컨트롤러 추종은 이번 윗면 실행 시험에 포함하지 않았습니다.

## 재빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws_cobot_pjt/ws_cobot1
colcon build --symlink-install --packages-select cake_decorator_interfaces cake_decorator cake_workcell_description cake_workcell_bringup
source install/local_setup.bash
ros2 interface show cake_decorator_interfaces/srv/GenerateTrajectory
```

빌드는 타입과 설치를 확인합니다. 실제 장비의 접촉과 컨트롤러 추종은 별도 검증 범위입니다.
