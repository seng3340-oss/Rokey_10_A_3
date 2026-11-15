# 케이크 자동화 시스템 설계 기준

PM이 공유한 전체 시퀀스와 작업 상태 흐름도를 보관합니다.
출처는 [소은님 시스템 설계 초안](https://app.slack.com/archives/C0CGDTN7BPE/p1794651834002869)과
프로젝트 대화에 전달된 다이어그램입니다. 아래는 **목표 설계이며 현재 실행 흐름이 아닙니다.**

원문의 이름·단계는 보존했습니다. 상세 정의와 다른 Action/Service 이름,
진행 중 취소와 자동 복귀 등 구현 전 확인 항목은 [인터페이스 작업 기준](INTERFACES.md)에 정리했습니다.
현재 구현은 웹 주문 저장·가상 배치와 인터페이스 타입 생성까지입니다.

## 전체 시퀀스

```mermaid
sequenceDiagram
    title ROS2 케이크 데코레이션 자동화 시스템 아키텍처 설계
    participant TAB as 태블릿 (고객용)
    participant BE as Python 백엔드
    participant HMI as HMI (운영자용)
    participant TM as task_manager_node
    participant DP as drawing_path_node
    participant RE as robot_executor_node
    participant DRV as 두산 드라이버

    Note over TM: 기동 시 workspace_config.yaml에서<br/>고정 좌표 로드 (ROS2 파라미터)
    rect rgb(230, 245, 255)
        Note over TAB,BE: Phase 1 — 주문 수신 (로그인 없음)
        TAB->>BE: 스트로크 데이터 전송 (strokes, drawing_surface, canvas_size)
        BE->>BE: order_id 자동 생성 (ORD-YYYYMMDD-NNN)
        BE->>TM: CakeOrder.action Goal (스트로크 원본 + order_id)
        TM-->>BE: ExecuteTask Feedback (단계 시작)
        BE-->>TAB: WebSocket (간소화 상태: 작업 중)
        BE-->>HMI: WebSocket (전체 상태 상세)
    end
    rect rgb(255, 245, 230)
        Note over TM,RE: Phase 2 — 케이크 픽업 (고정 좌표)
        TM->>TM: 고정 좌표에서 cake_pickup_pose 조회
        TM->>RE: MoveRobot Goal (PICK_CAKE, workspace_coords)
        RE->>DRV: movej(픽업 접근 위치)
        RE->>DRV: movel(픽업 위치)
        RE->>DRV: gripper_close(width=100mm)
        RE->>DRV: movel(들어올리기)
        RE-->>TM: MoveRobot Result (success)
    end
    rect rgb(230, 255, 230)
        Note over TM,RE: Phase 3 — 작업대 배치 (고정 좌표)
        TM->>TM: 고정 좌표에서 workstation_pose 조회
        TM->>RE: MoveRobot Goal (PLACE_CAKE, workspace_coords)
        RE->>DRV: movej(작업대 접근)
        RE->>DRV: movel(배치 위치)
        RE->>DRV: gripper_open()
        RE->>DRV: movel(후퇴)
        RE-->>TM: MoveRobot Result (success)
    end
    rect rgb(255, 230, 255)
        Note over TM,RE: Phase 4 — 펜 파지 (고정 좌표)
        TM->>TM: 고정 좌표에서 tool_holder_pose 조회
        TM->>RE: MoveRobot Goal (GRASP_TOOL, workspace_coords)
        RE->>DRV: SwitchTCP(드로잉용)
        RE->>DRV: movej(거치대 접근)
        RE->>DRV: movel(펜 위치)
        RE->>DRV: gripper_close(펜 파지)
        RE-->>TM: MoveRobot Result (success)
    end
    rect rgb(230, 240, 255)
        Note over TM,DP: Phase 5 — 드로잉 (윗면/옆면)
        TM->>DP: ConvertPath(strokes, cake_center, radius, height)
        DP->>DP: 2D→3D 궤적 변환 (평면/곡면) + 보간
        DP-->>TM: waypoints (Pose[])
        TM->>RE: MoveRobot Goal (DRAW, waypoints)
        RE->>DRV: movel(드로잉 시작점 접근)
        RE->>DRV: movel(표면 접촉)
        loop 각 waypoint
            RE->>DRV: movel(waypoint_i)
        end
        RE->>DRV: movel(펜 들어올리기)
        RE-->>TM: MoveRobot Result (success)
        TM-->>BE: ExecuteTask Feedback (드로잉 완료)
        BE-->>TAB: WebSocket (진행률 업데이트)
        BE-->>HMI: WebSocket (드로잉 상세 상태)
    end
    rect rgb(255, 240, 230)
        Note over TM,RE: Phase 6 — 펜 반납 (고정 좌표)
        TM->>RE: MoveRobot Goal (RETURN_TOOL, workspace_coords)
        RE->>DRV: movej(거치대 접근)
        RE->>DRV: movel(펜 반납 위치)
        RE->>DRV: gripper_open()
        RE->>DRV: SwitchTCP(운반용)
        RE-->>TM: MoveRobot Result (success)
    end
    rect rgb(230, 255, 245)
        Note over TM,RE: Phase 7 — 케이크 출고 (고정 좌표)
        TM->>TM: 고정 좌표에서 delivery_pose 조회
        TM->>RE: MoveRobot Goal (DELIVER_CAKE, workspace_coords)
        RE->>DRV: movel(케이크 집기)
        RE->>DRV: gripper_close()
        RE->>DRV: movej(출고 위치)
        RE->>DRV: gripper_open()
        RE->>DRV: movej(홈 포지션)
        RE-->>TM: MoveRobot Result (success)
        TM-->>BE: ExecuteTask Result (success)
        BE-->>TAB: WebSocket (작업 완료 알림)
        BE-->>HMI: WebSocket (완료 상태 + 전체 로그)
    end
```

## 작업 상태 흐름

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> RECEIVED : ExecuteTask Goal 수신
    RECEIVED --> VALIDATING : 데이터 검증 시작
    state VALIDATING {
        [*] --> 경로데이터검증
        경로데이터검증 --> 고정좌표확인 : 데이터 유효
        고정좌표확인 --> [*] : 좌표 로드 완료
    }
    VALIDATING --> PICKING_CAKE : 검증 통과
    VALIDATING --> FAILED : 경로 데이터 부적합 / 좌표 로드 실패
    state PICKING_CAKE {
        [*] --> 픽업위치이동
        픽업위치이동 --> 그리퍼잠금 : 위치 도달
        그리퍼잠금 --> 들어올리기 : 파지 완료
        들어올리기 --> [*]
    }
    PICKING_CAKE --> PLACING_CAKE : 픽업 성공
    PICKING_CAKE --> FAILED : 동작 실패
    state PLACING_CAKE {
        [*] --> 작업대이동
        작업대이동 --> 케이크배치 : 위치 도달
        케이크배치 --> 그리퍼해제 : 배치 완료
        그리퍼해제 --> [*]
    }
    PLACING_CAKE --> GRASPING_TOOL : 배치 성공
    PLACING_CAKE --> FAILED : 동작 실패
    state GRASPING_TOOL {
        [*] --> TCP전환
        TCP전환 --> 거치대이동 : TCP 전환 완료
        거치대이동 --> 펜파지 : 위치 도달
        펜파지 --> [*]
    }
    GRASPING_TOOL --> DRAWING : 펜 파지 성공
    GRASPING_TOOL --> FAILED : 동작 실패
    state DRAWING {
        [*] --> 궤적변환
        궤적변환 --> 윗면드로잉 : 윗면 경로 존재
        궤적변환 --> 옆면드로잉 : 옆면 경로 존재
        윗면드로잉 --> 옆면드로잉 : 옆면 경로 존재
        윗면드로잉 --> [*] : 옆면 없음
        옆면드로잉 --> [*]
    }
    DRAWING --> RETURNING_TOOL : 드로잉 완료
    DRAWING --> FAILED : 드로잉 실패
    state RETURNING_TOOL {
        [*] --> 거치대복귀
        거치대복귀 --> 펜반납 : 위치 도달
        펜반납 --> TCP복원 : 반납 완료
        TCP복원 --> [*]
    }
    RETURNING_TOOL --> DELIVERING : 펜 반납 성공
    RETURNING_TOOL --> FAILED : 동작 실패
    state DELIVERING {
        [*] --> 케이크집기
        케이크집기 --> 출고위치이동 : 파지 완료
        출고위치이동 --> 케이크내려놓기 : 위치 도달
        케이크내려놓기 --> [*]
    }
    DELIVERING --> COMPLETED : 출고 성공
    DELIVERING --> FAILED : 동작 실패
    COMPLETED --> IDLE : 초기화
    FAILED --> IDLE : 복구 / 재시작
    IDLE --> CANCELED : 작업 취소
    RECEIVED --> CANCELED : 작업 취소
    VALIDATING --> CANCELED : 작업 취소
    CANCELED --> IDLE : 초기화
```

도식의 자동 홈 복귀·취소 전이는 구현 코드가 아닙니다. 실행 중 취소 확인과 파지 상태에 따른 복구는 별도 설계가 필요합니다.
두산/RG2 호출이 한 DRV로 표현됐지만 실제 실행 코드는 두산·OnRobot 드라이버를 구분해야 합니다.
