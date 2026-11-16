# 케이크 ROS 업무 코드 틀

팀원이 나눠 개발할 파일 위치와 역할을 먼저 정리했습니다.
**브릿지와 백엔드 HTTP 연결, Goal 변환을 구현했습니다. TM·DP·RE와 로봇 제어는 아직 역할 주석만 있습니다.**
`ros2 run cake_decorator ros_bridge_node`를 등록했습니다.
실측 치수·영점·토큰 설정·단일 주문 정책과 실행 순서는 [브릿지 안내](../../../docs/BRIDGE_TEST.md)를 참고합니다.
`application.launch.py`와 업무 좌표 YAML은 여전히 주석으로 된 자리입니다.

## 데이터 흐름

```text
고객 웹 → FastAPI 검증·원본 저장 → 브릿지 → 작업 관리 노드
                                           │
                                           ├─ 경로 계산 요청 → 경로 계산 노드
                                           │                  └─ 점 선별·실제 좌표 계산
                                           │← 계산된 경로 ──────────┘
                                           └─ 실행 요청 → 로봇 실행 노드
                                                            └─ 두산·RG2 드라이버
```

원본 획 좌표와 해상도를 ROS까지 전달합니다.
점 선별·픽셀 좌표 변환은 Python 서버가 아닌 경로 계산 노드 쪽에서 처리합니다.
계산 함수는 일반 Python 모듈로 분리해 ROS·로봇 없이도 검증할 수 있게 만듭니다.
백엔드는 입력값·좌표 범위·요청 크기 등을 검사하고 기존 저장 역할을 유지합니다.

## 파일별 담당

```text
cake_decorator/
├── package.xml / setup.py / setup.cfg  # ROS Python 패키지 등록
├── resource/cake_decorator            # ament 인덱스용 빈 마커 파일
├── cake_decorator/
│   ├── __init__.py
│   ├── ros_bridge_node.py             # 주문 전달·진행·결과 수신
│   ├── task_manager_node.py           # 제작 순서·상태 관리
│   ├── drawing_path_node.py           # 경로 계산 Service
│   ├── robot_executor_node.py         # 동작 실행 Action
│   ├── backend_client.py              # 인증된 내부 HTTP 통신
│   ├── goal_mapping.py                # 원본 픽셀·실측 치수·TCP → Goal
│   ├── drawing_geometry.py            # 중복점 정리·필요점 선별·좌표 변환
│   ├── drawing_motion.py              # 접근·그리기·획 사이 이탈
│   └── doosan_robot.py                # DRL 호출·단위 변환·장비 연결
├── config/workspace_config.yaml       # 고정 좌표 설정 자리. 값 없음
├── launch/application.launch.py       # 업무 노드 launch 자리. 실행 불가
└── test/.gitkeep                      # 향후 계산·공정 테스트를 둘 자리
```

| 노드 | 제공할 기능 | 호출·구독할 기능 |
|---|---|---|
| 브릿지 BR | 백엔드 주문·상태 연결 | CakeOrder Action Client, TaskStatus 구독 |
| 작업 관리 TM | CakeOrder Action Server, TaskStatus 발행 | GenerateTrajectory Client, 동작 Action Client |
| 경로 계산 DP | GenerateTrajectory Service Server | drawing_geometry의 일반 Python 함수 |
| 로봇 실행 RE | DrawOnCake, PickAndPlace, GraspTool Action Server | drawing_motion과 장비 연결 모듈 |

브릿지의 기본 제작 Action 주소는 `/cake/order`이며 옵션으로 바꿀 수 있습니다. 나머지 주소는 아직 미정입니다.
타입 정의는 기존 [cake_decorator_interfaces](../cake_decorator_interfaces/README.md)를 사용합니다.
TCP·순응제어 서비스의 제공 위치와 실제 적용 정책은 이 틀에서 추가 확정하지 않습니다.

## 수업 코드에서 참고할 부분

- `count_up_client/server.py`: Action 목표 전송·수락·진행·결과 처리
- `add_client/server.py`: 경로 계산 요청·응답 처리
- `param_demo.py`: 작업 설정 읽기
- `calculator.launch.py`, `setup.py`: 노드 실행과 실행 파일 등록

TM은 Action 서버와 다른 노드의 클라이언트를 함께 맡습니다.
긴 작업 중 응답 처리가 막히지 않도록 비동기 호출·콜백 그룹·실행 구조를 고려합니다.

## 개발 순서

1. 테스트 주문으로 TM·DP·모의 RE 사이 통신을 확인합니다. 모의 실행을 실제 성공과 구분합니다.
2. 기존 윗면 검증의 점 선별·좌표 변환을 `drawing_geometry.py`로 분리합니다.
3. 접근·획 실행·펜 올림을 연결하고 virtual 브링업에서 검증합니다.
4. 백엔드↔브릿지 통신 방식을 정한 뒤 웹 주문을 연결합니다.
5. 실제 로봇 검증 후 옆면·펜 파지·케이크 운반을 추가합니다.

두 면 주문 묶음, 케이크 Z 기준·설정 우선순위, 획 경계와 완료 획 집계,
속도 우선순위, 취소·복구 정책은 구현 전에 맞춰야 합니다.
HMI는 기존 정적 화면을 유지합니다. 이 틀에는 HMI API나 운영 명령을 추가하지 않았습니다.

## 패키지 빌드만 확인하기

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws_cobot_pjt/ws_cobot1
colcon build --symlink-install --packages-up-to cake_decorator
source install/local_setup.bash
ros2 pkg prefix cake_decorator
```

빌드 성공은 파일 설치 확인이며 노드 실행·좌표 계산·로봇 동작 검증을 의미하지 않습니다.
브릿지만 실행할 수 있으며 TM·DP·RE나 이 패키지의 launch는 아직 실행하지 않습니다.
기존 frontend·backend·가상 작업대·ws_dsr 파일은 이번 틀 추가로 변경하지 않았습니다.
