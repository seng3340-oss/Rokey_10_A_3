# 로봇 레터링 케이크

태블릿에 그린 그림을 두산 M0609와 RG2 그리퍼로 케이크에 그리는 프로젝트입니다.
고객은 그림을 그리고, 운영자는 HMI에서 제작 상황을 확인하는 서비스를 목표로 합니다.
우선 고정된 케이크의 윗면·옆면 드로잉을 만들고, 이후 케이크 운반과 출고를 추가합니다.

현재 브랜치는 드로잉 통합 문제를 함께 확인하기 위한 작업본입니다.
원뿔대 케이크의 레터링 좌표와 이동 방식은 [표면 드로잉 안내](docs/SURFACE_DRAWING.md)를 참고하세요.

## 지금 되는 것

- 고객 화면에서 윗면·옆면 그림을 그리고 3D로 전체 모습 확인
- 전체 확인 화면에서 60초 동안 조작이 없으면 그림을 유지한 채 수정 화면으로 복귀
- FastAPI에서 그림 검증, 주문 ID 생성, SQLite 저장·조회와 단일 활성 주문 제한
- 원본 좌표·실측 치수·TCP 영점을 CakeOrder로 보내는 ROS 브릿지
- 가상 M0609·RG2와 테이블·타공판·케이크 고정구를 RViz에 함께 표시

HMI는 아직 정적 예시 화면입니다. 웹 → 백엔드 → 브릿지 → TM·DP·RE 연결 코드가 있으며,
전체 실행기로 준비한 뒤 웹에서 제작 요청을 누르면 선택한 모드의 로봇이 움직입니다.
옆면의 미세 계단과 손목 회전 한계는 추가 검증이 필요합니다.

## 전체 실행

```bash
cd ~/ws_cobot_pjt
bash scripts/start_virtual.sh  # 가상: 펜 돌출 길이 입력, 기본 4cm
# 또는
bash scripts/start_real.sh     # 실제: 상면 중앙에 펜촉을 두고 YES 입력
```

프론트·백엔드·로봇·업무 노드·브릿지를 함께 켭니다. 종료는 같은 터미널의 Ctrl+C입니다.
이미 `sodreal`을 켰다면 `bash scripts/start_real.sh --reuse-driver`를 사용합니다.
[전체 실행 순서와 옵션](docs/START_SYSTEM.md)에 포트 변경과 준비 검사를 정리했습니다.

## 폴더

```text
ws_cobot_pjt/
├── frontend/                  # React·TypeScript 고객 화면과 정적 HMI
├── backend/                   # FastAPI 그림 접수·SQLite 저장·주문 조회
├── ws_cobot1/
│   ├── src/
│   │   ├── cake_decorator/            # 주문 브릿지·업무 노드 개발 위치
│   │   ├── cake_decorator_interfaces/ # 메시지·서비스·액션 타입
│   │   ├── cake_workcell_description/  # 테이블·A안 고정구 모델
│   │   └── cake_workcell_bringup/      # 가상 로봇과 작업 환경 통합 launch
│   └── doc/                   # 가상 환경 실행·설정 문서
├── ws_dsr/src/                # 두산·OnRobot RG2 드라이버와 강의 브링업
├── config/                    # 웹·백엔드 공통 원뿔대 케이크 치수
├── scripts/                   # 실행·실측 형상 생성 스크립트
├── docs/                      # 설계·데이터 규격·환경 설명
├── docker/                    # Docker 설정을 둘 자리
└── DartPlatform/              # Dart Platform 설치를 둘 자리
```

기존 기어 삽입 예제는 최상위에 그대로 보관합니다.
`node_modules`, `.venv`, `build`, `install`, `log`와 주문 DB는 Git에 올리지 않습니다.

## 웹 실행

Python 3.12와 Node.js 22.12 이상을 사용합니다. 터미널 두 개를 엽니다.

**터미널 1 — 백엔드**

```bash
git clone https://github.com/seng3340-oss/Rokey_10_A_3.git
cd Rokey_10_A_3

# 교육장 ROS 환경에서 최초 한 번 준비
source /opt/ros/jazzy/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
  python3-venv python3-pymodbus python3-scipy python3-yaml
rosdep install --from-paths ws_dsr/src ws_cobot1/src --ignore-src -r -y
```

아래 실행기는 웹 의존성이나 ROS 빌드가 없으면 자동으로 준비합니다.
최초 설치·빌드는 인터넷 연결이 필요할 수 있으므로 로봇 LAN으로 전환하기 전에
`bash scripts/setup_project.sh`로 미리 준비해두면 편합니다.
다른 PC의 `build`, `install`, `.venv`, `node_modules`는 복사하지 않습니다.
폴더 이름이나 사용자 이름은 달라도 됩니다.

## 실제 로봇으로 실행

```bash
bash scripts/start_real.sh
```

로봇 기본 주소는 `192.168.1.100`, 모델은 `m0609`, ROS_DOMAIN_ID 기본값은30입니다.
기존 미완료 주문이 있으면 작업이 실제 중단됐는지 확인한 뒤 RESET을 입력합니다.
펜을 수직으로 세워 펜촉을 케이크 상면 중앙에 가볍게 맞추고 YES를 입력합니다.
YES 뒤에는 펜촉 TCP를 계산·적용하고 웹·ROS 업무 노드를 준비합니다.
웹에서 주문을 제출하면 실제 로봇이 움직입니다. 종료는 실행 터미널에서 Ctrl+C입니다.

펜촉 계산은 실제 상면 높이와 현재 TCP의 XY가 펜촉 XY와 같다는 가정을 사용합니다.
정밀 교정을 대신하지 않으므로 펜을 다시 잡았거나 배치를 바꿨다면 실제 펜촉을 확인합니다.
설정한 펜촉 TCP는 종료 후에도 유지되며 이전 TCP 복원 이름을 터미널에 표시합니다.

## 로봇 없이 가상 실행

```bash
bash scripts/start_virtual.sh
```

펜 돌출 길이는 기본4cm이며 실행 시 입력합니다. 가상 로봇·RViz·웹·백엔드·업무 노드를 함께 켭니다.
실물용 TCP 설정은 가상 모드에 적용하지 않습니다.
기본 station 모델과 가상 자동 영점의 legacy 배치는 아직 별도로 확인해야 합니다.

## 화면 접속

- 고객 화면: `http://localhost:8080`
- 태블릿: 같은 Wi-Fi에서 `http://노트북의-Wi-Fi-IP:8080`
- HMI: 같은 주소의 `/hmi`

로봇용 유선 LAN과 태블릿용 Wi-Fi를 함께 사용할 수 있습니다.
자세한 설정은 [네트워크 안내](docs/NETWORK.md)를 참고하세요.

## 45도 펜 옆면 이동만 가상 검증

```bash
bash scripts/start_virtual_side_web_test.sh
```

RViz에서 케이크 중앙 도착을 확인하고 터미널에 YES를 입력합니다.
웹 `http://localhost:8081`에서 옆면을 선택해 그림을 제출합니다.
실물과 분리한 도메인187·별도 주문 DB를 사용하며 펜 반납과 그리퍼 조작은 생략합니다.
다른 그림을 시험하려면 Ctrl+C 후 같은 명령을 다시 실행합니다.

## 자주 바꾸는 설정

| 파일 | 내용 |
|---|---|
| `config/real_robot.json` | 실제 상면 BASE Z=220mm, YES 후 펜촉 TCP 설정 여부 |
| `config/test_cake.json` | 실제 케이크 윗지름113.5mm·밑지름120mm·높이79mm, 판179×179×2mm |
| `ws_cobot1/src/cake_decorator/config/workspace_config.yaml` | 옆면 각도17도, 옆면 기준속도60mm/s, 접촉 지름 보정2mm |

지름 보정2mm는 실제 케이크를 키우는 값이 아니라 옆면 펜촉 목표를 반지름 방향으로1mm 바깥에 두는 값입니다.
그리기 전 도달 가능 자세를 대표점으로 검사합니다. 이 검사는 연속 경로의 충돌·접촉 품질을 보장하지 않습니다.

## 개발할 때

프론트만 실행하려면 `cd frontend && npm run dev`, 백엔드만 실행하려면 아래 명령을 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash
source ~/ws_cobot_pjt/ws_cobot1/install/local_setup.bash
export ROS_DOMAIN_ID=30
ros2 launch cake_workcell_bringup virtual.launch.py
```

실행하면 `그리퍼 끝에서 펜촉까지 돌출 길이(cm)`를 묻습니다. Enter를 누르면
기본 4cm를 사용합니다. 입력한 길이는 펜 모델과 `Virtual_Pen_TCP`에 함께 반영되며,
`가상 펜 준비 완료`가 출력된 뒤 그리기 노드를 사용합니다.
길이를 미리 지정하려면 `pen_extension_cm:=4`를 launch 명령 뒤에 붙입니다.
이 설정은 가상 모드 전용입니다. `sodreal`과 실물의 교정된 TCP는 변경하지 않습니다.
실물에서는 펜촉을 케이크 상면 중심에 맞추고 브릿지에서 YES를 눌러 작업 기준점을 기록합니다.

현재는 보존한 `ws_cobot_pjt_jihwan`의 로봇 설치본을 사용합니다.
새 `ws_cobot_pjt/ws_dsr`를 빌드한 뒤에는 두 번째 source 경로를 새 설치본으로 바꿉니다.
`.sh`는 새 설치본이 있으면 그쪽을 우선 사용합니다.

다른 PC에서 처음 실행할 때는 로봇 환경을 준비하고, 작업 환경 패키지도 빌드해야 합니다.
[가상 환경 빌드·위치 조정 방법](ws_cobot1/doc/VIRTUAL_WORKCELL.md)을 참고합니다.

### 현재 모델 크기와 위치

- 테이블: 가로 1200 × 세로 650mm, 상판 높이 960mm
- 확정 원뿔대 케이크: 윗지름113.5 / 밑지름120 / 높이79mm
- 케이크판: 179 × 179 × 두께 2mm 사각판
- 타공판 조립체: 임시 배치. 현재 케이크 중심은 로봇 BASE 기준 X=500, Y=0mm

원뿔대 치수는 웹과 백엔드가 함께 읽는 `config/test_cake.json`에서 관리합니다.
RViz 형상 갱신은 `python3 scripts/generate_test_cake_model.py`로 수행합니다.
고정구 위치는 `ws_cobot1/src/cake_workcell_bringup/config/workcell.yaml`의 임시값입니다.
실행 원점은 이 배치가 아니라 브릿지 시작 시 접촉 확인한 현재 TCP의 윗면 중심입니다.

RViz에 모델이 보이는 것과 실제 충돌·접촉 검증은 별개입니다.
현재 타공판 위치와 모델 높이는 실물 설치 좌표나 펜촉 영점으로 확정한 값이 아닙니다.

## 실제 로봇에 연결할 때

교육장에서 노트북과 로봇을 유선으로 연결합니다.
현재 강의 설정은 M0609, 로봇 주소 `192.168.1.100`, 포트 `12345`, 도메인 ID `30`입니다.
가상 브링업을 종료한 뒤 별도 터미널에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch m0609_rg2_bringup bringup.launch.py \
  mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```

기존 `sodreal` 별칭이 있다면 마지막 launch 명령 대신 사용해도 됩니다.
새 `ws_dsr`를 빌드했다면 source 경로는 `~/ws_cobot_pjt/ws_dsr/install/setup.bash`를 사용합니다.

이 단계는 **실제 로봇 연결**입니다. 이후 다른 터미널에서도 같은 ROS 환경과 도메인 ID를 설정하고,
실행하려는 로봇 동작 Python 파일이나 ROS 노드를 별도로 켭니다.
연결 전용 launch를 켠 것만으로 케이크 드로잉이 시작되지는 않습니다.
주문 전달 브릿지와 TM·DP·RE 코드가 있으며, 현재는 고정 원뿔대의 드로잉을 검증 중입니다. 케이크 운반까지의 전체 자동 제작은 별도 검증이 필요합니다.

실제 필기 검증 전에는 설치 위치, 활성 TCP·Tool, 펜촉 접촉 높이를 현장에서 확인합니다.
Real은 기존 강의 브링업을 사용하며, 임시 위치의 테이블·케이크 모델은 추가하지 않았습니다.

## 관련 문서

- [프론트 실행·구성](frontend/README.md)
- [백엔드 실행·API·테스트](backend/README.md)
- [그림 데이터 규격](docs/DRAWING_PROTOCOL.md)
- [로봇 실행환경 이관·빌드](docs/ROBOT_ENVIRONMENT.md)
- [태블릿 네트워크 연결](docs/NETWORK.md)

- [PM 시퀀스·상태 흐름도](docs/architecture/SYSTEM_DESIGN.md)
- [ROS 인터페이스 초안과 확인할 항목](docs/architecture/INTERFACES.md)
- [시스템 설계·통신 규격 보고서 PDF](docs/architecture/report/케이크_시스템설계_통신규격_v0.3.pdf)
- [확정한 인터페이스 결정](docs/architecture/DECISIONS.md)
- [인터페이스 패키지 빌드](ws_cobot1/src/cake_decorator_interfaces/README.md)

## 원뿔대 케이크와 주문 브릿지

[단일 주문·브릿지 실행 방법](docs/BRIDGE_TEST.md)을 참고합니다.
TM은 `/cake/order` Action을 제공해야 하며 브릿지는 TM이 준비된 뒤에만 주문을 인수합니다.

현재 원뿔대·원본 픽셀 계약은 [확정 통신 규격](docs/CYLINDER_PIXEL_CONTRACT.md)을 기준으로 합니다.

직사각형 옆면 입력을 원뿔대 표면으로 옮기는 규칙과 회귀 검사는 [표면 드로잉 안내](docs/SURFACE_DRAWING.md)에 정리했습니다.

### 윗면·옆면 통합 실물 공정

```bash
cd ~/ws_cobot_pjt
bash scripts/start_real.sh
```

이미 `sodreal`을 켰다면 `--reuse-driver`를 붙인다. 케이크 중심을 찍거나 YES를 입력하는 과정 없이 웹 주문을 기다린다.
두 면을 그리면 하나의 주문으로 접수해 케이크 배치 → 윗면 → 펜 반납·45도 재파지 → 옆면 → J6 되감기·펜 반납 → 케이크 회수 순서로 실행한다.
한 면만 그린 주문은 해당 면만 실행한다. 중간 실패 시 이후 공정을 실행하지 않는다.

윗면은 기존 `cake_center_pose`의 기본 TCP 티칭 높이 510.39mm를 유지한다.
옆면은 `real_robot.json`의 실제 펜촉 기준 상면 높이 222mm를 사용한다. 두 값은 TCP 기준이 다르다.
전체 사이클은 아직 실물 통합 시험이 필요하며, 기존 옆면 검증 스크립트는 최종 확인 전까지 남겨 둔다.
