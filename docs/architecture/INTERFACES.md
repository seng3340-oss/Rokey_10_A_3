# ROS 인터페이스 작업 기준

기준 자료는 소은님이 2026-11-14 19:23 KST에 공유한
「ROS2 케이크 데코레이션 자동화 시스템 아키텍처 설계 (1).pdf」입니다.
[원문과 스레드](https://app.slack.com/archives/C0CGDTN7BPE/p1794651834002869)

원문은 작성자가 초안이라고 표시했으며 확인한 스레드에 형식을 수정하는 합의는 없었습니다.
이번 작업은 **문서의 상세 정의를 빌드 가능한 타입으로 옮기는 단계**입니다.
2026-11-15 사용자가 승인한 세 항목은 [합의 기록](DECISIONS.md)에 반영했습니다. 나머지 모호한 필드를 임의로 보완하거나 실제 명령을 보내지 않습니다.

## 이번에 구현한 범위

| 종류 | 생성한 타입 | 원문 기준 |
|---|---|---|
| Message | TaskStatus, CakeParams, WorkspaceCoords, DrawingPath, Stroke2D, Waypoint3D | PDF 7~8쪽 |
| Service | GenerateTrajectory, SwitchTCP, SetCompliance | PDF 6~7쪽 |
| Action | CakeOrder, PickAndPlace, GraspTool, DrawOnCake | PDF 5~6쪽 |

패키지는 cake_decorator_interfaces입니다. 노드 없이 타입만 빌드·직렬화 검사합니다.
SetCompliance는 원문에서 미사용으로 표시된 형식을 보존했을 뿐 실행하지 않습니다.
원문의 센서 유무 서술을 실제 M0609 하드웨어 사실로 확정하지 않습니다.

## 이름이 다른 부분

| 위치 | 표/시퀀스/개발 순서 | 상세 정의 | 이번 처리 |
|---|---|---|---|
| 전체 제작 | ExecuteTask와 CakeOrder 혼용 | CakeOrder.action | CakeOrder만 생성 |
| 로봇 실행 | MoveRobot | PickAndPlace/GraspTool/DrawOnCake | 상세 Action 3개만 생성 |
| 경로 계산 | ConvertPath | GenerateTrajectory.srv | GenerateTrajectory만 생성 |
| 패키지 | cake_decoration_interfaces | cake_decorator_interfaces 참조 | IDL 참조 이름으로 생성 |
| 상태 표기 | RETURNING_TOOL | RELEASING_TOOL | 원문 보존, 상태 집합은 미확정 |

두 인터페이스 체계를 동시에 제공하면 호출 노드마다 서로 다른 계약을 쓰게 됩니다.
따라서 별칭·임시 범용 Action은 추가하지 않았고, 상세 정의 이름으로 통일하는 안은 사용자 승인을 받았습니다. 노드 구현도 같은 이름을 사용합니다.

## 웹 → ROS 연결 전에 필요한 결정

1. **캔버스 크기/좌표 기준 — 확정**: DrawingPath와 GenerateTrajectory에 canvas_width/height를 추가합니다.
   원본 px와 해상도를 함께 보내며 연결 경계에서 임의 정규화를 하지 않습니다.
2. **획 정보 — 확정**: Stroke2D에 stroke_id를 보존하고 is_closed=false를 사용합니다.
   사용자가 그리지 않은 마지막점→첫점 연결선을 추가하지 않습니다.
3. **면 표기/복수 면**: 웹 top/side → ROS TOP/SIDE 변환과 한 주문의 drawing_paths 조립 규칙이 필요합니다.
   현재 웹 한 요청을 두 면 주문으로 임의 확장하지 않습니다.
4. **케이크 설정 주체**: CakeOrder는 cake_params를 받지만 원문 설명은 TM의 고정 YAML을 기준으로 합니다.
   어느 값을 우선할지, center.z와 cake_pose 원점이 바닥/상면/중심 중 무엇인지 확정해야 합니다.
5. **속도·펜 상태**: DrawOnCake.speed와 Waypoint3D.speed의 우선순위,
   GenerateTrajectory.pen_states와 Waypoint3D.pen_down의 길이·값 일치 규칙이 필요합니다.
6. **접촉·폐경로·그리퍼**: contact_force는 현재 높이 고정 방식에서 미사용입니다.
   RG2 서비스는 기존 OnRobot 드라이버와 별도로 연결해야 하며 두산 드라이버에 모두 있다고 가정하지 않습니다.
7. **타입과 좌표계**: ROS Pose는 m/쿼터니언, 두산 API는 mm/ZYZ degree입니다.
   정확한 BASE TF와 변환을 실행 모듈에서 관리하고 위치·쿼터니언 유효성을 실행 전에 검사합니다.
8. **취소·복구**: 시퀀스/상태도는 설계 기준이며 실제 중단 확인을 구현한 상태가 아닙니다.
   취소 접수만으로 CANCELED/홈 복귀를 확정하지 않고 물체·펜 파지 상태에 따른 절차가 필요합니다.
9. **연결 상태와 오류**: TaskStatus에는 별도 error_code 필드가 없고 일부 Result에만 있습니다.
   WebSocket 채널·오류 매핑·재접속·중복 주문 규칙은 아직 구현하지 않습니다.

## 단계별 진행

- 이번: 원문 다이어그램 보존 + 상세 인터페이스 생성 + 메시지 직렬화 테스트.
- 다음: 노드 담당자와 위 차이점을 맞춘 뒤 최소 통신 노드를 연결.
- 이후: 윗면 run3 계산 로직, 가상 실행, 옆면 경로 순으로 검증.

기존 웹 주문은 DB 저장까지만 동작하며 HMI는 정적입니다. 이 패키지 빌드로 로봇이 움직이지 않습니다.

검토한 PDF SHA-256: `b169757311c3d902b832c3f700c664354ca22fc6c63f6842f1e6fb1f89b0c2d1`
