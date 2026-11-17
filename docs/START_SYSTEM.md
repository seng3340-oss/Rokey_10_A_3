# 한 터미널에서 전체 실행

프론트, FastAPI, 로봇 드라이버, TM·DP·RE, 주문 브릿지를 함께 켠다. 기존 `sodvir`, `sodreal`과 `.bashrc`는 변경하지 않는다.
ROS 환경과 웹 의존성은 기존 README 순서대로 설치·빌드되어 있어야 한다.

## 가상 로봇

```bash
cd ~/ws_cobot_pjt
bash scripts/start_virtual.sh
```

1. 그리퍼 끝부터 펜촉까지 돌출 길이를 cm로 입력한다. Enter만 누르면 **4cm**다.
2. 가상 로봇·작업대·케이크·펜·RViz를 켜고, 가상 펜 TCP 준비가 끝날 때까지 기다린다.
3. 가상 로봇을 케이크 상면 중앙으로 이동해 영점을 기록한다.
4. 웹과 업무 노드가 준비되면 브라우저가 열린다. 직접 그린 뒤 제작 요청을 누른다.

현재 모델은 지름100mm·높이80mm 원통이다. 중심은 작업대 설정에서 계산한다.
가상 모드는 이 PC 내부 통신만 사용하고 외부 ROS 검색 서버 설정을 이번 실행에서만 해제한다.
주문 DB는 실행마다 `log/system/실행시각-virtual-PID/orders.sqlite3`에 따로 저장해 이전 가상 주문이 재실행되지 않게 한다.

## 실제 로봇

```bash
cd ~/ws_cobot_pjt
bash scripts/start_real.sh
```

기본 연결은 강의와 같은 `mode:=real host:=192.168.1.100 port:=12345 model:=m0609`이다.
연결이 준비되면 케이크 상면 중앙에 교정된 펜촉 TCP를 가볍게 닿게 두고 **YES**를 입력한다.
그 직후 조회한 BASE TCP를 영점으로 기록한다. 실제 모드의 실행기는 홈 이동·펜 길이 변경·TCP 변경·그리퍼 조작 명령을 보내지 않는다.
웹에서 제작 요청을 누르면 실제 드로잉 명령이 실행된다.

이미 `sodreal`을 켰다면 중복 브링업 대신 아래 명령을 쓴다.

```bash
bash scripts/start_real.sh --reuse-driver
```

실제 RG2 드라이버에는 `pymodbus`가 필요하다. 해당 ROS Python 환경에 없으면 실행 전에 중단하고 누락을 안내한다.
실제 주문 DB는 기존 `backend/data/orders.sqlite3`를 사용한다. 미완료 주문은 자동 삭제·해제·재전송하지 않는다.
남아 있는 주문 때문에 중단되면 [브릿지 안내](BRIDGE_TEST.md)의 확인 절차를 따른다.

## 주소와 종료

- 고객 화면: `http://localhost:8080`
- 같은 Wi-Fi 태블릿: `http://노트북-Wi-Fi-IP:8080`
- HMI: 같은 주소의 `/hmi` — 현재는 정적 예시 화면
- 종료: 실행한 터미널에서 **Ctrl+C**

이 실행기가 켠 프로세스만 종료한다. `--reuse-driver`로 재사용한 드라이버는 유지한다.
Ctrl+C는 소프트웨어 종료이며 실제 장비의 안전 정지는 티칭 펜던트를 사용한다.

이미 웹이나 백엔드가 켜져 있으면 강제로 종료하지 않고 중복 포트를 알려준다. 기존 서버를 종료하거나 다른 포트로 실행한다.

```bash
bash scripts/start_virtual.sh --frontend-port 8081 --backend-port 5001
```

## 선택 옵션

```bash
# 입력 생략, 기본 4cm 대신 6cm
bash scripts/start_virtual.sh --pen-extension-cm 6

# 도메인 변경. 기본은 현재 ROS_DOMAIN_ID, 미설정 시 교육장 30
bash scripts/start_virtual.sh --domain-id 83

# 실제 로봇 주소 변경
bash scripts/start_real.sh --host 192.168.1.100

# 실행 없이 의존성·포트 검사
bash scripts/start_virtual.sh --check
bash scripts/start_real.sh --check

# 브라우저 자동 열기 생략
bash scripts/start_virtual.sh --no-browser
```

ROS 설치 경로를 직접 고르려면 `CAKE_ROBOT_SETUP=/경로/install/setup.bash`를 지정한다.
기본은 새 `ws_dsr/install/setup.bash`, 없으면 보존한 `~/ws_cobot_pjt_jihwan/ws_dsr/install/setup.bash`를 사용한다.
`CAKE_DATABASE_PATH`를 지정하면 선택한 DB를 사용한다. 모드별 DB를 섞지 않는 것이 좋다.

실행한 터미널에 `[driver]`, `[task_manager_node]`, `[robot_executor_node]`처럼 출처를 붙여 로그를 실시간으로 표시한다.
같은 내용은 `log/system/실행시각-모드-PID/`의 프로세스별 `.log` 파일에도 저장한다.
현재 HMI는 정적 화면이므로 오류 확인에는 실행 터미널과 저장된 로그를 사용한다.

가상 모드에서는 첫 비접촉 접근에만 관절 이동을 사용한다. 영점의 수직 자세에서 옆면 자세로 바로 직선 이동할 때 발생한 손목 특이점을 피하기 위한 처리다.
현재 자세로 현재점·첫 접근점 중 높은 지점보다 100mm 위로 이동한다.
그 높이에서 BASE 쪽으로 물러나 옆면 자세로 바꾼 뒤 첫 접근점으로 관절 이동한다.
각 단계의 실제 TCP 위치·자세를 조회해 목표에 도달했을 때만 다음 단계로 진행한다.
드라이버가 `virtual/127.0.0.1`인지 다시 확인한다.
실제 모드의 접근과 획 실행 방식은 이 옵션으로 바뀌지 않는다.

그리기 기준 속도는 테스트2 종이 검증과 같은 30mm/s, 펜을 뗀 이동은 80mm/s다.
주문 가동속도 50%이면 각각 15mm/s·40mm/s로 적용된다. 접촉 진입은 그리기 속도의 절반을 유지한다.
짧은 구간과 자세 변화에서는 가감속·관절 제한 때문에 실제 속도가 더 낮을 수 있다.

Ctrl+C 뒤에는 정상 종료를 3초 기다리고, 남은 소유 프로세스 그룹에 TERM·KILL을 차례로 보낸다.
이 정리 중 다시 누른 Ctrl+C는 무시하므로 종료 완료 문구까지 기다린다.
이 실행기는 현재 그리기 로직을 연결하는 도구다. 옆면의 미세 계단과 손목 회전 한계 문제를 해결하는 기능은 포함하지 않는다.

## 이번 확인 범위

가상 모드에서 4cm 펜 설정, 초기 영점 배치, 전체 노드·웹 기동, 프론트의 백엔드 프록시, HMI 페이지 표시와 Ctrl+C 후 종료를 확인했다. 자동으로 그림을 제출하지 않았다.
실행기·첫 접근·경로 변환·통신 규격 검사에 추가 Ctrl+C와 속도 비율 적용 검사도 포함한다. 실제 모드는 자동 이동 없이 YES 뒤에만 TCP를 읽는 분기를 검사했다. 실제 장비 연결 시험은 수행하지 않았다.

주문 후 첫 옆면 접근에서 멈췄던 목표점을 가상 로봇에서 다시 실행해 도달을 확인했다.
관측한 펜촉 궤적과 도달 결과는 `results/initial_approach_verification.json`에 남겼다.
이번 확인은 첫 접근까지이며 해당 주문 전체의 옆면 그리기를 재검증한 결과는 아니다.

상승 경유점을 추가한 접근은 `results/raised_side_approach_verification.json`에서 확인할 수 있다.
기준 상면 Z=142mm에서 Z=252mm까지 상승한 뒤, X=350mm의 높은 지점에서 자세를 바꾸고 첫 접근점으로 이동했다.
관측한 펜촉 표본 243개에서 원통 XY 내부의 최저 Z는 약 142mm였고 첫 접근점 도달을 확인했다.
이는 펜촉 경로 관측이며 로봇 링크·펜 몸체 전체의 충돌 검사를 수행한 결과는 아니다.
