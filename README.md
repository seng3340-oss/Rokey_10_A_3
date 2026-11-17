# 로봇 레터링 케이크

태블릿에 그린 그림을 두산 M0609와 RG2 그리퍼로 케이크에 그리는 프로젝트입니다.
고객은 그림을 그리고, 운영자는 HMI에서 제작 상황을 확인하는 서비스를 목표로 합니다.
우선 고정된 케이크의 윗면·옆면 드로잉을 만들고, 이후 케이크 운반과 출고를 추가합니다.

## 지금 되는 것

- 고객 화면에서 윗면·옆면 그림을 그리고 3D로 전체 모습 확인
- 전체 확인 화면에서 60초 동안 조작이 없으면 그림을 유지한 채 수정 화면으로 복귀
- FastAPI에서 그림 검증, 주문 ID 생성, SQLite 저장·조회와 단일 활성 주문 제한
- 원본 좌표·실측 치수·TCP 영점을 CakeOrder로 보내는 ROS 브릿지
- 가상 M0609·RG2와 테이블·타공판·케이크 고정구를 RViz에 함께 표시

HMI는 아직 정적 예시 화면입니다. 브릿지는 구현했지만 TM·경로 계산·로봇 실행 노드는 아직 틀만 있습니다.
**실행 노드까지 연결되기 전에는 웹 접수만으로 로봇이 움직이지 않습니다.**
기존 테스트에서 검증한 드로잉은 윗면이며, 옆면 경로와 접촉은 추가 검증이 필요합니다.

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
├── config/                    # 웹·백엔드 공통 원통형 케이크 치수
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
cd ~/ws_cobot_pjt/backend
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/uvicorn app.main:app --host 127.0.0.1 --port 5000
```

**터미널 2 — 프론트**

```bash
cd ~/ws_cobot_pjt/frontend
npm ci
npm run dev
```

의존성 설치는 처음 실행하거나 변경됐을 때만 하면 됩니다.
이 개발 노트북에서 `node -v`가 18로 나오면 프론트 실행 전에 다음을 입력합니다.

```bash
# 이 노트북에 별도로 설치해 둔 Node.js 22 사용
export PATH="$HOME/.local/share/cake-web-tooling/node_modules/.bin:$PATH"
```

| 화면 | 노트북에서 접속 | 같은 Wi-Fi의 다른 기기에서 접속 |
|---|---|---|
| 고객용 | `http://localhost:8080` | `http://노트북-Wi-Fi-IP:8080` |
| HMI | `http://localhost:8080/hmi` | `http://노트북-Wi-Fi-IP:8080/hmi` |
| API 문서 | `http://127.0.0.1:5000/docs` | 노트북에서 확인 |

태블릿과 HMI용 데스크톱을 노트북과 같은 Wi-Fi에 연결합니다.
프론트 실행 시 터미널에 나오는 `Network` 주소를 사용하면 됩니다.
Wi-Fi가 바뀌면 IP도 달라지므로 이전 주소를 그대로 쓰지 않습니다.
웹 통신은 Wi-Fi, 실제 로봇 연결은 유선 LAN을 사용합니다.

그림은 이미지가 아닌 획별 픽셀 좌표로 `POST /api/drawing`에 전송됩니다.
서버는 `backend/data/orders.sqlite3`에 저장하고 주문 ID를 반환합니다.
한 요청에는 현재 선택한 면 하나만 포함됩니다. HMI는 이 주문을 조회하지 않습니다.

## 로봇 없이 가상 시뮬 켜기

ROS 2 Jazzy, Docker와 두산 DRCF 에뮬레이터가 준비된 환경에서 실행합니다.
이미 `sodvir`나 같은 `dsr01` 가상 환경이 켜져 있으면 먼저 종료합니다.

```bash
cd ~/ws_cobot_pjt
bash scripts/start_virtual_workcell.sh
```

이 스크립트가 ROS·로봇 패키지 환경을 불러오고 `ROS_DOMAIN_ID=30`을 설정한 뒤,
가상 컨트롤러와 금속 스테이션·일체형 선반·케이크 틀이 배치된 RViz를 켭니다.
기존 `scripts/virtual_draw_demo.py`에 맞는 이전 배치는 `layout:=legacy`로 엽니다.
배치와 좌표는 [가상 작업셀 안내](ws_cobot1/doc/VIRTUAL_WORKCELL.md)를 참고하세요.
실제 로봇에는 연결하지 않으며, 로봇 이동 명령도 보내지 않습니다.
종료할 때는 실행한 터미널에서 `Ctrl+C`를 누릅니다.

수업처럼 직접 실행하고 싶으면 아래 명령을 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash
source ~/ws_cobot_pjt/ws_cobot1/install/local_setup.bash
export ROS_DOMAIN_ID=30
ros2 launch cake_workcell_bringup virtual.launch.py
```

현재는 보존한 `ws_cobot_pjt_jihwan`의 로봇 설치본을 사용합니다.
새 `ws_cobot_pjt/ws_dsr`를 빌드한 뒤에는 두 번째 source 경로를 새 설치본으로 바꿉니다.
`.sh`는 새 설치본이 있으면 그쪽을 우선 사용합니다.

다른 PC에서 처음 실행할 때는 로봇 환경을 준비하고, 작업 환경 패키지도 빌드해야 합니다.
[가상 환경 빌드·위치 조정 방법](ws_cobot1/doc/VIRTUAL_WORKCELL.md)을 참고합니다.

### 현재 모델 크기와 위치

- 테이블: 가로 1200 × 세로 650mm, 상판 높이 960mm
- 확정 3D 프린트 원통형 케이크: 지름100 / 높이80mm
- 케이크판: 179 × 179 × 두께 2mm 사각판
- 타공판 조립체: 임시 배치. 현재 케이크 중심은 로봇 BASE 기준 X=500, Y=0mm

원통형 치수는 웹과 백엔드가 함께 읽는 `config/test_cake.json`에서 관리합니다.
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
현재 저장소에는 주문 전달 브릿지가 있으며 자동 제작하는 TM·DP·RE는 아직 구현 전입니다.

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

## 원통형 케이크와 주문 브릿지

[단일 주문·브릿지 실행 방법](docs/BRIDGE_TEST.md)을 참고합니다.
TM은 `/cake/order` Action을 제공해야 하며 브릿지는 TM이 준비된 뒤에만 주문을 인수합니다.

현재 원통형·원본 픽셀 계약은 [확정 통신 규격](docs/CYLINDER_PIXEL_CONTRACT.md)을 기준으로 합니다.
