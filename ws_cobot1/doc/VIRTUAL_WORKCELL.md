# 케이크 드로잉 가상 작업셀

기존 테이블에 금속 스테이션, 일체형 선반, 대각선 틀, 오른쪽 가이드, 펜 홀더와 케이크따개를 배치한 RViz 장면입니다.

## 실행

저장소 루트에서:

```bash
cd ~/ws_cobot_pjt
bash scripts/start_virtual_workcell.sh
```

가상 DRCF Docker, M0609+RG2 드라이버, 테이블/작업셀 시각화, RViz가 실행됩니다.
이미 sodvir 또는 같은 dsr01 가상 launch가 켜져 있다면 먼저 종료하세요.
실행기는 기존 sod/sodvir/sodreal 별칭을 바꾸지 않으며 ROS_DOMAIN_ID=30을 사용합니다.
종료는 실행한 터미널에서 Ctrl+C입니다. RViz만 닫으면 컨트롤러가 남을 수 있습니다.

새 ws_dsr는 아직 빌드되지 않아 현재 실행기는 보존된
~/ws_cobot_pjt_jihwan/ws_dsr/install을 fallback으로 사용합니다.
새 경로에 ws_dsr/install/setup.bash가 준비되면 그쪽을 우선 사용합니다.
이전 설치본 경로를 삭제하기 전에는 새 ws_dsr 재빌드가 필요합니다.

## 패키지

```text
ws_cobot1/src/
├── cake_workcell_description/
│   ├── meshes/                 # 상판·매립 링과 배치별 STL
│   ├── urdf/workcell.urdf.xacro # 테이블과 배치 선택
│   ├── urdf/station_layout.xacro # 승인한 배치의 프레임과 메시
│   ├── SOURCES.md              # Slack 원문·원본 치수·단위·제한
│   └── tests/test_workcell.py   # 프레임/단위/체크섬 검사
└── cake_workcell_bringup/
    ├── launch/virtual.launch.py       # 작업대+가상 로봇+RViz
    ├── launch/course_virtual.launch.py# 강의 launch 재사용, virtual/localhost 고정
    ├── config/workcell.yaml           # 이전 고정구 위치 (mm/degree)
    └── rviz/workcell.rviz             # 로봇과 환경을 각각 표시
```

## 기본 배치

좌표는 로봇 BASE 기준입니다. 테이블 상면은 Z=0 mm입니다.

| 부품 또는 프레임 | 기준과 치수 |
|---|---|
| 스테이션 `station_base` | (399, −49, 0) mm. 오른쪽 위 개구부 40 × 40 mm |
| 스테이션 틀 `cake_tray_surface` | 틀 윗면 (399, −49, 139) mm |
| 스테이션의 원뿔대 케이크 | 밑면 지름 120 mm, 윗면 지름 113.5 mm, 높이 80 mm. 밑면 Z=139 mm, 윗면 중심 (399, −49, 219) mm |
| 일체형 선반 `cake_shared_shelf` | 바닥 중심 (755, 50, 0) mm. X 150 × Y 550 × Z 150 mm |
| 드로잉 전 틀 `cake_input_tray` | 바닥 중심 (755, 200, 150) mm. yaw +45° |
| 반출구 틀 `cake_output_tray` | 바닥 중심 (755, −100, 150) mm. yaw +45° |
| 펜 홀더 `cake_pen_holder` | (275, 225, 0) mm. 위치 확인용 임시 형상 |
| 케이크따개 `cake_opener` | (425, 225, 0) mm. 제공한 STL 사용 |

150 mm 정사각형 틀을 45° 회전하면 선반 양쪽으로 약 31.1 mm 돌출됩니다.
두 틀 사이 여유는 약 87.9 mm이고 각각 오른쪽에 `>`형 가이드가 있습니다.
가이드의 체결 구조, 로봇 도달 범위 및 충돌 검증은 별도 작업입니다.

## 기존 드로잉 데모 배치

`scripts/virtual_draw_demo.py`는 이전 장면의 케이크 윗면 중심 (500, 0, 142) mm를 사용합니다.
해당 데모를 실행하려면 작업셀을 아래처럼 띄웁니다.

```bash
bash scripts/start_virtual_workcell.sh layout:=legacy
```

`fixture_x`, `fixture_y`, `fixture_z`, `fixture_yaw`는 이 이전 장면에서 사용합니다.
런치 인자의 단위는 m/rad입니다.

## 최초 빌드 또는 재빌드

```bash
source /opt/ros/jazzy/setup.bash
# 현재 컴퓨터의 기존 로봇 설치본. 새 ws_dsr 빌드 후에는 그 설치본을 source합니다.
source ~/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash
cd ~/ws_cobot_pjt/ws_cobot1
colcon build --symlink-install --packages-select cake_workcell_description cake_workcell_bringup
```

모델은 package:// 경로로 자산을 참조하고 기존 테스트 폴더에 의존하지 않습니다.
로봇은 /dsr01/robot_description, 환경은 /workcell_description으로 표시하며
base_link → workcell_root 정적 TF로 연결해 프레임 이름 충돌을 피합니다.

## 검증 범위

- 모델 테스트는 Xacro 결과, 16개 매립 홀, 메시 경로와 단위, 두 배치의 기준 프레임을 확인합니다.
- 새 배치는 RViz 시각 형상입니다. 드로잉 좌표, MoveIt 충돌 장면과 물리 접촉은 별도로 설정합니다.
- 가상 컨트롤러와 그리퍼는 강의 브링업의 가상 모드로 실행됩니다.
