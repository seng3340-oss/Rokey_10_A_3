# 테이블2 + A안 고정구 가상 배치

강승묵님이 Slack에 공유한 A안의 타공판·프레임·케이크판·케이크를
기존 test_custom_mini_cake2의 테이블 위에 올린 RViz 장면입니다.
위치는 임시값이며 로봇 이동이나 그리기 동작은 실행하지 않습니다.

## 실행

저장소 루트에서:

```bash
cd ~/ws_cobot_pjt
bash scripts/start_virtual_workcell.sh
```

가상 DRCF Docker, M0609+RG2 드라이버, 테이블/A안 시각화, RViz가 실행됩니다.
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
│   ├── meshes/                 # 기존 상판·매립 링, 팀원 A안 STL 4개
│   ├── urdf/workcell.urdf.xacro # 독립 작업 환경 모델, A안 mount 위치 인자
│   ├── SOURCES.md              # Slack 원문·원본 치수·단위·제한
│   └── tests/test_workcell.py   # 프레임/단위/체크섬 검사
└── cake_workcell_bringup/
    ├── launch/virtual.launch.py       # 작업대+가상 로봇+RViz
    ├── launch/course_virtual.launch.py# 강의 launch 재사용, virtual/localhost 고정
    ├── config/workcell.yaml           # 임시 설치 위치 (mm/degree)
    └── rviz/workcell.rviz             # 로봇과 환경을 각각 표시
```

## 위치 조정

cake_workcell_bringup/config/workcell.yaml의 fixture 값을 바꿉니다.
현재 원점은 BASE (415,-85,0)mm이고 A안 케이크 중심은 (500,0)mm입니다.
타공판은 X=375..625, Y=-125..125mm 범위에 있으며 밑면이 상판 Z=0에 닿습니다.
A안 메시에는 자체 위치가 포함돼 있으므로 원점을 케이크 중심과 혼동하지 마세요.
A안 원본 케이크는 약 지름100mm·높이80mm이며 이전 테스트의 지름150mm를 강제로 적용하지 않았습니다.
케이크 상면은 현재 BASE Z=150mm입니다. 모델 높이를 실제 TCP 영점으로 사용하지 않습니다.

빠르게 시험할 때는 launch 인자로 덮어쓸 수 있습니다. 이 인자는 **m/rad** 단위입니다.

```bash
bash scripts/start_virtual_workcell.sh fixture_x:=0.45 fixture_y:=-0.085 fixture_yaw:=0.0
```

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

## 검증 및 한계

- 두 패키지 빌드, 단위·위치 인자·16개 매립 구멍·메시 체크섬 테스트 4개 통과
- 실제 가상 launch에서 joint_state_broadcaster/dsr_controller2 active 확인
- RViz에서 기존 캐비닛 테이블·바퀴·로봇·A안 조립체 표시 확인
- 타공판 최종 배치, 로봇 도달 가능성, TCP·힘/접촉, 고정구 삽입 동작은 미검증
- RViz 표시만 추가하며 A안의 collision 형상·MoveIt 충돌 장면·물리 접촉 시뮬레이션은 추가하지 않음
- 가상 컨트롤러와 그리퍼는 강의 브링업의 기존 동작을 사용하며 실물 로봇은 연결하지 않음
