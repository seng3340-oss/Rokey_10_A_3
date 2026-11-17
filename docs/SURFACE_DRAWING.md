# 원통 케이크 옆면 드로잉 검증

현재 원통은 지름 100mm, 높이 80mm다. 가상 작업대의 상면 중심은 BASE `(500, 0, 142)mm`다. 실제 로봇에서는 이 고정값 대신 TM이 기록한 펜촉 접촉 위치를 사용한다.

## 좌표 선별과 이동

- `path_simplification.py`: 테스트 케이크2와 같은 RDP 방식이다. 픽셀을 실제 mm로 바꾼 뒤 0.01mm 이내 중복을 제거하고 0.15mm 허용 오차로 점을 선별한다. 원본 점 순서와 획 시작·끝을 유지한다.
- `surface_path.py`: 옆면 가로선은 실제 원호(`movec`), 세로선은 직선(`movel`)으로 옮긴다. 높이와 원주 위치가 함께 바뀌는 선은 원통 위에 촘촘한 목표점을 만들고 `movesx`로 묶는다. 한 요청은 최대 100점이다.
- `drawing_path_node.py`: 위 규칙으로 `Waypoint3D`를 생성한다. 옆면 펜의 +Z 방향은 케이크 중심을 향하면서 수직 아래 방향에서 30° 기울어진다. 획 사이에는 원통 바깥 반지름으로 물러나 원호로 이동한다.
- `drawing_motion.py` → `doosan_robot.py`: 원호의 경유점과 스플라인 점들을 두산 서비스 요청까지 보존한다. 실패한 원호를 관통 직선으로 대체하지 않는다.

수평 법선 기준으로는 아래쪽 60°다. `side_pen_vertical_deg`의 기본값 30은 **수직 기준**이다. 지정한 TCP 축과 실제 펜 길이·장착 방향이 일치해야 한다.

스플라인의 목표점은 원통 위에 있지만 제어기 보간 중의 전체 곡선까지 수학적으로 표면 일치를 보장하지는 않는다. 따라서 가상 로봇의 실제 관측 TF로 경로와 반경 오차를 별도 검사한다. 한 획 완료 후 정지 판단 로직은 이번 작업 범위에 포함하지 않았다.

## 가상 모델 FK 교정

수업용 URDF의 반올림 치수를 그대로 쓰면 에뮬레이터 FK와 표시 펜촉 사이에 고정 Z 약 -0.5mm와 작은 XY 편차가 생긴다. 동일 관절각 3개를 읽기 전용 `fkin(ref=BASE)`과 비교해 확인했다.

`virtual_pen.py`는 가상 모드의 **렌더링 URDF 사본**에서 알려진 원본값만 정리한다. `joint_1` Z 134.5→135mm, `joint_2` Y 6.20→6.25mm, 로봇 관절 원점의 ±1.571rad→±π/2를 적용한다. 이미 다른 치수인 항목은 덮어쓰지 않는다. 벤더 소스, 실물 BASE, 명령 좌표, 케이크 위치와 펜 길이는 변경하지 않는다. 다른 에뮬레이터/모델을 쓰면 FK를 다시 비교해야 한다.

## 빨간 잉크

`virtual_ink_node`는 `base_link → virtual_pen_tip` TF를 관측한다. 상면 또는 옆면에서 표면거리 ±0.5mm 이내일 때만 `/cake/ink` 빨간 마커를 남긴다. 공중 이동, 오래된 TF, 누락된 TF를 그림으로 채우지 않는다. 표시선이 모델 표면과 겹쳐 깜박이지 않도록 0.2mm 밖에 표시한다.

이는 **가상 기하 접촉 모델**이다. 실제 볼펜 잉크·마찰·힘 센서를 물리적으로 시뮬레이션한 결과는 아니다. `/cake/observed_tip`은 표시용 투영점이 아닌 원래 관측 TCP 자세이며, `/cake/ink_telemetry`에서 접촉 거리와 누락 통계를 확인할 수 있다.

## 실행

메시지에 원호·스플라인 필드를 추가했으므로 패키지를 빌드하고, 이 메시지를 쓰는 기존 노드를 모두 다시 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash
cd ~/ws_cobot_pjt/ws_cobot1
colcon build --symlink-install --packages-select cake_decorator_interfaces cake_decorator cake_workcell_bringup
source install/local_setup.bash
```

다음 두 터미널 모두 위 환경을 source한다. 교육장 검색 서버가 없는 로컬 가상 검증에서는 **각 터미널에만** 다음을 적용한다. 기존 alias나 `.bashrc`는 변경하지 않는다.

```bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_DOMAIN_ID=83
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

터미널 1 — 가상 작업대, 펜, 잉크, RViz:

```bash
ros2 launch cake_workcell_bringup virtual.launch.py pen_extension_cm:=4
```

터미널 2 — 파형 9개 실행:

```bash
cd ~/ws_cobot_pjt
ros2 run cake_decorator surface_wave_test --execute-virtual
```

이 검증기는 하드웨어 설정이 `mode=virtual`, `host=127.0.0.1`인지 확인한 후에만 움직인다. 진폭 1·3·6mm × 파장 4·8·16mm를 사용한다. 각 파형 사이에는 원통 밖에서 상승한 뒤 홈 자세로 복귀해 다음 시험을 독립적으로 시작한다. 중심 접근을 반복하는 이전 검증기에서는 손목 6축 회전 누적과 한계 오류가 발생했다. 이 검증기의 독립 시험 통과가 여러 주문의 연속 운전까지 보장하지는 않는다. 알려진 획 완료 오류와 분리해 DP → 실행 계획 → 두산 가상 서비스 경로를 검사하며, 전체 주문 Action의 정상 완료를 검증하는 도구는 아니다.

결과는 `results/side_wave_verification.json`에 저장된다. 접촉점 수, 원본 파형 대비 최대/RMS 거리, 반경 오차와 각도 오차를 포함한다. 더 가까이 보려면 별도 RViz를 다음 설정으로 실행한다.

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix cake_workcell_bringup)/share/cake_workcell_bringup/rviz/side_wave.rviz"
```

공식 보간 동작 참고: [두산 movesx 설명](https://doosanrobotics.github.io/doosan-robotics-api-manual/GL013305/mode/auto/motion_primitives_advanced/movesx.html).

비교 그림은 `python3 scripts/plot_surface_waves.py`로 생성한다. 회색은 원본 파형, 빨간색은 실제 관측 TF다. 그래프 출력에 NumPy·Matplotlib이 필요하다.

검증기는 서비스 반환 후에도 **명령 이후의 신선한 TF**가 목표점 0.05mm 이내에 도달했는지 확인한다. 접촉 접근 구간은 파형 형상 오차 계산에서 제외한다. 관측점→원본 거리뿐 아니라 원본→관측점 거리도 검사해 끝단이 빠진 기록은 성공 처리하지 않는다.

일부 항목만 다시 검증하려면 기존 결과와 잉크를 유지할 수 있다. 번호는 진폭 행, 파장 열 순서의 0~8이다.

```bash
ros2 run cake_decorator surface_wave_test --execute-virtual \
  --cases 0,1,2 --resume-results --keep-ink
```

가상 실행 결과와 비교 그림은 [검증 결과](../results/README.md)에 정리했다.
