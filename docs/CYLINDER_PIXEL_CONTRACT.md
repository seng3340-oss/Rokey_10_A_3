# 케이크 형상과 원본 픽셀 통신 규격

현재 확정 형상은 원뿔대(윗지름113.5mm·밑지름120mm·높이79mm)입니다.
ROS 인터페이스와 DP는 윗면·아랫면 반지름을 각각 받는 원뿔대도 표현하며,
두 반지름이 같을 때 원통으로 처리합니다. 받침판은179×179×2mm입니다.

## 역할과 기준

```text
웹 → 백엔드 원본 저장 → 브릿지 → TM → DP(점 선별·px→mm→3D) → TM → RE
```

- 웹·백엔드·브릿지·TM은 원본 획 좌표와 캔버스 해상도를 보존합니다.
- Stroke2D: uint32 stroke_id, Point32[] points. x/y는 px, z는 0입니다.
- 폐곡선 여부 필드는 없으며 입력하지 않은 끝점 연결을 하지 않습니다.
- 케이크는 윗면 반지름과 아랫면 반지름을 별도로 전달합니다. 현재 설정은 두 값이 다릅니다.
- 케이크 위치는 BASE 기준 **윗면 중심**입니다. 바닥 중심이 아닙니다.
- 실물의 윗면 중심은 기존처럼 사용자가 접촉 확인한 활성 펜촉 TCP에서 기록합니다.
- 케이크 높이나 판 두께를 윗면 중심 Z에 다시 더하지 않습니다.
- ROS 위치·치수는 m, 자세는 쿼터니언, 속도는 mm/s입니다.

## 설정 파일

`config/test_cake.json`은 웹·백엔드가 공유하는 윗면·아랫면 지름과 높이·판 치수입니다.
`CakeParams`는 이를 윗면·아랫면 반지름으로 전달하며, 기존 원통 주문은 아랫면 반지름을 윗면 값으로 채웁니다.
`ws_cobot1/src/cake_decorator/config/workspace_config.yaml`은 원래의 노드별
`ros__parameters` 구조이며 윗면 반지름0.05675m·밑면 반지름0.06m·높이0.079m와 변환 설정을 기록합니다.
치수 변경 시 두 설정을 함께 맞춥니다. 실제 위치는 요청으로 전달하며 미측정 좌표는 0으로 채우지 않습니다.
DP의 approach_height는 mm(기본10)이며 RE의 approach_height는 m입니다.

RViz 케이크·판 형상은 다음 명령으로 공유 JSON에서 재생성합니다.

```bash
python3 scripts/generate_test_cake_model.py
```

지그 위치는 기존 임시 배치를 유지합니다. 판 아래가 지그 기준 Z=60mm인 현재 배치에서
legacy 케이크 상면은 지그 바닥 기준 Z=141mm입니다. 실제 로봇의 영점으로 이 가상 높이를 사용하지 않습니다.

## TM → DP: GenerateTrajectory.srv

| 요청 필드 | 의미 |
|---|---|
| canvas_width / canvas_height | 원본 캔버스 해상도(px) |
| order_id / operation_speed_percent | 주문 식별자 / 작업 속도 비율 |
| strokes_2d | Stroke2D 배열. 순서·원본 픽셀 그대로 |
| surface_type | TOP 또는 SIDE |
| cake_pose | BASE 윗면 중심(m)·영점에서 기록한 펜 TCP 자세 |
| cake_yaw | 케이크 평면 회전(rad), 펜 자세와 별개 |
| cake_radius / cake_bottom_radius | 윗면 / 아랫면 반지름(m) |
| cake_height | 원뿔대 높이(m) |

응답은 success, waypoints_3d, pen_states, estimated_duration_sec, error_msg를 유지합니다.
`CakeOrder → DrawingPath → Stroke2D` 입력도 같은 원본 픽셀 규칙을 사용합니다.
서비스 요청에는 주문 추적용 order_id와 operation_speed_percent도 포함합니다.

## DP 담당자가 맞춰야 할 화면 좌표 규칙

웹 원본 캔버스는1024×768px입니다. 실제 그림 영역은 캔버스 전체와 다릅니다.

- 윗면 원 중심=(512,384), 반지름320px.
- 윗면 변환 예: local_x=(pixel_x-512)/320×56.75mm,
  local_y=-(pixel_y-384)/320×56.75mm. 화면 Y 아래 방향과 BASE Y 방향을 구분합니다.
- 옆면 띠는 너비의 87.5%를 사용하고 캔버스 세로 중앙에 둡니다.
  원뿔대의 사면 길이 `S=sqrt(H²+(r_bottom-r_top)²)`를 사용해
  띠 높이를 `너비×S/(π×(r_top+r_bottom))`로 계산합니다.
- `u`는 띠 안의 가로 비율, `v`는 세로 비율입니다.
  `θ=2πu`, `height=H×(1-v)`로 대응합니다.
  팀원 형상 API는 `arc=θ×r(height)`를 사용하고, 실행 경로 어댑터는
  `arc=θ×(r_top+r_bottom)/2`를 사용합니다. 높이가 변해도 각도를 보존하기 위한 내부 변환입니다.
  최종3D 좌표에는 언제나 해당 높이의 실제 반지름을 적용합니다.
- 유효 영역·점 선별·펜 접근/이탈·자세·도달 가능성 검사는 DP·실행 측에서 검증합니다.

이 규칙은 전달할 픽셀을 미리 정규화하라는 뜻이 아닙니다. 변환은 DP에서 수행합니다.

## 이번 수정과 담당 범위

ROS 인터페이스는 윗면·아랫면 반지름을 모두 전달합니다. TM은 획의 원본 픽셀을
보존해 DP에 보내고, DP는 캔버스의 TOP/SIDE 편집 영역을 해당 치수에 맞춰 변환합니다.
SIDE 경로는 각 높이의 원뿔대 반지름과 표면 법선을 사용합니다.

빌드 목록에서 참조하던 Stroke2D를 복원하고, mm 입력 전용 PointMM/PointMM3D/StrokeMM과
비어 있던 dsr_msgs2.srv/onrobot_rg_msgs.srv는 제거했습니다.
기존 백엔드 주문 사본은 자동 변경하지 않습니다. 이전 경사 케이크 주문을 재실행하지 말고 새 설정으로 접수합니다.

## 재빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws_cobot_pjt/ws_cobot1
colcon build --symlink-install --packages-select cake_decorator_interfaces cake_decorator cake_workcell_description cake_workcell_bringup
source install/local_setup.bash
ros2 interface show cake_decorator_interfaces/srv/GenerateTrajectory
```

빌드는 타입·설치 확인입니다. DP 업데이트 후 TOP/SIDE의 실제 계산·가상 이동 통합 검증이 추가로 필요합니다.

## HMI 계획 표시

HMI는 DP 응답의 표시용 사본을 읽습니다. 원뿔대의 사면 길이·케이크 평면 회전과
실제 요청의 TOP/SIDE 기준점을 반영하며, 펜을 올린 구간은 잇지 않습니다.
원호의 경유점과 스플라인 표본은 계획 제어점으로 표시하며 실측 궤적이 아닙니다.
HMI는 이동·정지 명령을 보내지 않습니다.
