# 케이크 ROS 인터페이스 초안

소은님이 2026-11-14 19:23 KST에 공유한 시스템 설계 PDF의 **상세 정의 파일**을 옮겼습니다.
메시지 6개, 서비스 3개, 액션 4개입니다. 통신 타입만 생성하며 노드나 로봇 명령은 실행하지 않습니다.

출처: https://app.slack.com/archives/C0CGDTN7BPE/p1794651834002869
문서 내 서로 다른 표기와 연결 전 확인 사항은
[인터페이스 작업 기준](../../../docs/architecture/INTERFACES.md)에 정리했습니다.

## 구성

```text
cake_decorator_interfaces/
├── msg/
│   ├── TaskStatus.msg        # /task_status로 전달할 공정 상태
│   ├── CakeParams.msg        # 케이크 중심·반지름·높이·방향
│   ├── WorkspaceCoords.msg   # 사전 지정 작업 위치
│   ├── DrawingPath.msg       # TOP/SIDE별 원본 획 배열
│   ├── Stroke2D.msg          # Point32 픽셀 좌표·폐경로 여부
│   └── Waypoint3D.msg        # Pose·속도·접촉 상태·접촉력
├── srv/
│   ├── GenerateTrajectory.srv # 2D→3D 계산 요청/응답 형식
│   ├── SwitchTCP.srv          # TCP 적용 요청/응답 형식
│   └── SetCompliance.srv      # 원문 보존용 미사용 형식
└── action/
    ├── CakeOrder.action       # 전체 제작
    ├── PickAndPlace.action    # 픽업·배치
    ├── GraspTool.action       # 도구 파지·반납
    └── DrawOnCake.action      # 경로 실행
```

## 빌드·확인

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws_cobot_pjt/ws_cobot1
colcon build --symlink-install --packages-select cake_decorator_interfaces
source install/local_setup.bash
ros2 interface show cake_decorator_interfaces/action/CakeOrder
ros2 interface show cake_decorator_interfaces/srv/GenerateTrajectory
colcon test --packages-select cake_decorator_interfaces --event-handlers console_direct+
colcon test-result --test-result-base build/cake_decorator_interfaces --verbose
```

패키지명은 상세 IDL 참조가 일관되게 쓰는 cake_decorator_interfaces로 정했습니다.
PDF 폴더 트리의 cake_decoration_interfaces는 참조 이름과 달라 확인이 필요합니다.
사용자 승인으로 상세 정의 이름을 기준으로 통일했습니다. ExecuteTask/MoveRobot/ConvertPath 별칭은 추가하지 않습니다.

## 단위와 한계

- Stroke2D Point32의 x/y는 픽셀, z는 2D 연결 시 0으로 둘 예정입니다.
- Pose와 케이크 치수는 m, orientation은 rad, Pose 자세는 쿼터니언입니다.
- speed/grip_width는 문서 기준 mm/s와 mm, contact_force는 N입니다.
- TOP/SIDE 등 문자열 후보는 주석일 뿐 IDL이 열거형 검증을 하지 않습니다.
- 기본 생성된 Pose는 측정·검증한 목표가 아닙니다. 위치와 쿼터니언 유효성을 실행 전에 검사해야 합니다.
- 사용자 승인: canvas_width/height를 DrawingPath와 GenerateTrajectory에 추가하고 stroke_id를 Stroke2D에 보존합니다.
- 사용자 승인: is_closed는 false로 사용하여 그리지 않은 자동 연결선을 만들지 않습니다.
- 웹 top/side는 ROS 경계에서 TOP/SIDE로 바꿉니다. 실제 변환·요청 전달 코드는 아직 연결하지 않았습니다.
- SetCompliance 형식이 생성되더라도 순응제어가 활성화되는 것은 아닙니다.
- 웹 API, HMI, ROS 브리지, Action/Service 서버, 실제 이동은 변경하지 않습니다.
