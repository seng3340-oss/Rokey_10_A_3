# 로봇 레터링 케이크

태블릿에서 그린 그림을 두산 M0609와 RG2로 케이크에 그리는 프로젝트입니다.
고객 화면에서 윗면·옆면을 선택해 그림을 제출하면 백엔드, ROS 브릿지, 작업 관리·경로 계산·로봇 실행 노드를 거쳐 동작합니다.
현재는 고정된 케이크에 그리는 단계입니다. HMI는 정적 예시 화면이며, 케이크 운반·출고는 별도 작업입니다.

## 처음 준비하기

Ubuntu 24.04와 ROS 2 Jazzy, Node.js 22.12 이상·npm을 사용합니다.
가상 모드에는 Docker Engine과 강의 DRCF 에뮬레이터도 필요합니다.
Node.js가 이미 설치돼 있다면 `node --version`으로 버전을 확인하세요.

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
cd backend
.venv/bin/python -m uvicorn app.main:app --host 0.0.0.0 --port 5000
```

ROS 코드나 인터페이스를 업데이트한 뒤에는 저장소 루트에서 `bash scripts/setup_project.sh`로 다시 빌드합니다.
실제 연결 없이 준비 상태만 검사하려면 `bash scripts/start_real.sh --check`를 사용합니다.

```text
frontend/       고객 그림 화면·3D 미리보기·정적 HMI
backend/        FastAPI·주문 저장·단일 활성 주문 제한
config/         실제 로봇·케이크 설정
scripts/        설치 준비·가상/실제 전체 실행
ws_cobot1/src/  브릿지·TM·DP·RE·통신 타입·가상 작업셀
ws_dsr/src/     강의 두산·RG2 드라이버
```

실행 로그는 `log/system/실행시각-모드-PID/`에 저장합니다. 주문 DB·빌드 결과·로그는 Git에 올리지 않습니다.
[전체 실행 옵션](docs/START_SYSTEM.md) · [펜촉 설정과 옆면 동작](docs/PEN_TCP.md) · [통신 규격](docs/DRAWING_PROTOCOL.md)
