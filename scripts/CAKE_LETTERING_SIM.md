# ‘사랑해’ M0609 가상 필기

ROS 2 Jazzy의 Xacro와 이 저장소의 M0609·RG2 모델을 사용한다. 오프라인 검사에 Docker, DRCF, DART Studio 또는 장비 네트워크는 필요하지 않다. RViz 재생은 검증 CSV를 관절 상태로 게시하며, 작업대와 필기 결과는 마커로 표시한다. 작업대 위치는 현재 Xacro 기본 배치다.

## 실행

저장소 루트에서 실행한다. Python NumPy·SciPy·Matplotlib과 시스템 C++ 컴파일러, pkg-config, FCL·Assimp 개발 라이브러리가 필요하다. 출력 폴더는 지정한 위치에 생성한다.

```bash
cd ~/ws_cobot_a3/Rokey_10_A_3
source /opt/ros/jazzy/setup.bash
python3 scripts/simulate_cake_lettering.py --output /tmp/cake-lettering
```

- `lettering_preview.png`는 위에서 본 펜촉 경로, `lettering_trajectory.csv`는 시간·단계·TCP·관절 로그다.
- `lettering_validation.json`에는 치수·가장자리 여유·속도·역기구학 오차·충돌 검사 결과가 기록된다. 움직이는 링크의 충돌을 검출하면 명령이 오류로 종료된다.

## RViz 재생

첫 터미널에서 검사에 사용한 출력 폴더를 지정한다.

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/simulate_cake_lettering.py --output /tmp/cake-lettering --replay
```

두 번째 터미널에서 로컬 전용 RViz를 시작한다.

```bash
source /opt/ros/jazzy/setup.bash
env -u ROS_STATIC_PEERS -u ROS_DISCOVERY_SERVER ROS_DOMAIN_ID=96 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  rviz2 -d scripts/cake_lettering_preview.rviz
```

RViz 설정은 **Fixed Frame** `base_link`, **RobotModel** `/dsr01/robot_description`, **MarkerArray** `/cake_lettering_preview/markers`를 사용한다. 표준 상태 발행기가 `/dsr01/joint_states`를 읽어 `/tf`를 계산하며, 정적 TF `base_link → cake_preview_base`가 작업대 및 필기 좌표를 연결한다. 재생은 로컬 ROS 도메인 96에서 RViz가 구독하고 펜의 TF가 준비되면 시작한다. 터미널에서 Ctrl+C로 종료한다. 다시 재생하려면 재생 명령을 다시 실행한다.

## 좌표와 검사 범위

- BASE 기준 케이크 바닥은 Z=62mm, 윗면 중심은 (500, 0, 141)mm다. 캔버스 오른쪽은 BASE +X, 아래쪽은 −Y다. 펜촉 중심선의 전체 가로와 세로는 70×40mm다.
- 펜은 RG2 베이스의 로컬 +Z로 240mm 떨어진 가상 TCP이며, 필기 중 +Z 방향은 수직 아래다. RG2 관절은 0rad로 고정한다. 이 오프셋은 모델 검증용 가정이며 실물 펜촉 교정값은 `docs/BRIDGE_TEST.md`의 접촉 영점 절차로 결정한다.
- 매 획은 Z=151mm에서 접근해 Z=141mm로 수직 하강한다. 획 뒤에는 Z=151mm로 수직 상승한 뒤 획이나 글자 사이를 이동한다. 필기 및 수직 이동은 20mm/s, 수평 이송은 40mm/s, 선형 가속도는 100mm/s²로 제한한다. 5차 보간으로 각 선분의 양 끝에서 정지한다.
- 첫 획 위에 정지한 상태부터 마지막 리프트까지 검사한다. 1mm 이하·40ms 이하 TCP 간격으로 샘플링하고 모든 샘플에서 URDF 관절 위치, 샘플 구간 평균 관절 속도와 링크 충돌을 검사한다. 팔의 직접 연결 링크, 서로 고정된 RG2 부품과 펜의 장착부에서 의도된 내부 겹침을 제외한다. 케이크에는 펜촉의 점 접촉만 허용하며 펜 몸체를 메시 장면과 비교한다.
- 고정 베이스와 상판·앞판의 초기 모델 겹침은 JSON에 기록된다. 시뮬레이션 범위는 첫 획 위에서 시작하는 경로와 샘플 위치의 기구학이다. 홈에서의 진입·복귀, 샘플 사이의 연속 충돌, 컨트롤러 추종, 접촉 압력·케이크 변형은 별도 검증이 필요하다.
