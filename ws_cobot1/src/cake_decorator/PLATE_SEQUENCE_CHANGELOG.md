# 플레이트 시퀀스 변경 내역 (2026-11-18)

## 변경 목적
`aaa/position.py`의 실물 티칭 이동을 기존 Git 프로젝트의 함수로 재사용하며 드로잉 전후에 배치합니다. 원본 참고 파일은 실행 시 import하지 않습니다.

## 파일별 변경
- `config/workspace_config.yaml`: 원본 28개 이동 명령의 좌표/주석을 `plate_` 파라미터로 추가. 상대 이동을 주석으로 명시. `plate_sequence_enabled: false` 기본 유지.
- `cake_decorator/doosan_robot.py`: `move_joint(mode=0)` 인수 확장. 기존 `pick`/`place`를 재사용하며 RG2 명령 실패 확인 및 그리퍼 대기시간 인수 추가. 기존 호출은 기본 0초로 호환.
- `cake_decorator/robot_executor_node.py`: `plate_prepare`, `plate_finish` Trigger 서비스. 오프너 픽업은 `pick()`, 반환은 `place()` 사용. 중간 동작은 기존 `move_joint`/`move_line` 사용.
- `cake_decorator/task_manager_node.py`: 활성화 시 준비 → 기존 드로잉 → 후처리. 기본값은 드로잉만 실행.
- `package.xml`: `std_srvs` 실행 의존성 추가.

## 실행 전 필수 확인
1. 실제 오프너의 RG2 파지 폭·파지력·열림 폭, TCP 및 툴 설정 확인.
2. `pick()`/`place()`의 추가 사전 그리퍼 열림·복귀 동작이 실물과 간섭하지 않는지 확인.
3. 원본 `position.py`의 안전 자세 명칭은 충돌 검증을 의미하지 않습니다.
4. 원본 `set_singular_handling(DR_AVOID)`는 아직 이식하지 않았습니다. 별도 검증이 필요합니다.
5. 원본 이동 속도(관절 60/100, 직선 250/1000 등)는 **요청에 따라 적용하지 않고** Git 기본값을 사용합니다.
6. 두 노드의 `plate_sequence_enabled`를 모두 true로 변경해야 전체 시퀀스가 연결됩니다. 실물 검증 전 활성화 금지.
7. 서비스 실패/시간 초과 시 로봇을 자동으로 원위치 복귀시키지 않습니다. 실제 정지 여부를 별도 확인해야 합니다.
8. 서비스 Trigger는 진행 중 취소 신호를 세부 동작으로 전달하지 않습니다. 실물 운용 전 취소/정지 경로 보완이 필요합니다.
9. ROS2 Jazzy/Doosan 드라이버/실물 하드웨어 환경에서 실행 테스트는 수행하지 않았습니다.

## 시퀀스
준비: safe_position_1 → opener pick → move_cake_ps_L2 → move_cake_plate_ps1~ps5 → safe_position_2 → cake_station_ps1~ps3 → safe_position_3.

기존 드로잉 액션.

후처리: pickup_cake_station_ps1~ps4 → safe_position_4 → deliver_cake_station_ps1~ps5 → safe_position_5 → opener place.

## 의도적인 원본 대비 변경
- `move_cake_ps_L`: `pick()`의 접근 위치 복귀와 중복되어 별도 호출하지 않음.
- 오프너 반환은 `place()`가 접근 위치까지 추가 후퇴함. 원본은 해제 후 종료.
- ~~DO1/DO2 펄스 대신 기존 RG2 `open`/`close` 서비스 사용.~~ (2026-11-19 변경: 아래 최신 그리퍼 절 참조. 제어박스 DO 펄스를 기본으로 사용한다.)
- `pick_opener2`는 BASE 기준 상대 Z=-100mm를 절대 파지 위치로 환산하여 `pick()`에 전달.
- 원본 좌표 주석은 YAML에 보존하고, 실제 실행 시 오프너 반납 좌표는 픽업 좌표와 공통 사용.


## 2026-11-18 사용자 수정 티칭 파일 반영

- 기준: `move cakeplate_update.txt` (좌표 28개, 원본 주석).
- 기존 잘못된 `safe_position_2`/`cake_station_ps1~ps3` 매핑을 수정하고 `cake_station_ps4` 추가.
- 수정 파일에는 `move_cake_plate_ps5`가 없으므로 제거. `safe_position_2`는 실제 공통 안전 관절 좌표.
- Git의 기본 속도·가속도 사용. 원본 속도/특이점 설정은 YAML 주석으로 기록.
- 기존 RG2 그리퍼 서비스와 open/close 대기시간 유지. 추후 오프너 그리퍼 명령 변경 가능성 주석.
- 오프너 반납 완료 후 `safe_position_5` 이동 추가.
- 기존 `plate_sequence_enabled` 비활성 기본값 유지.
- 검증: YAML 파싱, 원본 28개 좌표 대조, Python 구문 검사. 실물 구동 검증 아님.

## 2026-11-19 그리퍼 디지털 출력 전환 및 포트 설정

### 플레이트 전용 함수 정리
- 플레이트 오프너 파지·반환은 기존 `pick()`·`place()`를 그대로 재사용한다.
- 중복 구현이던 `plate_pick()`, `plate_place()`, `plate_gripper_open/close()`를 제거했다.
- `pick()`에는 `close_wait_sec`, `close_settle_sec`, `pre_open_wait_sec`,
  `place()`에는 `open_wait_sec`만 추가했다. 기본값은 0초라 기존 호출과 호환된다.
- `plate_sequence.py`는 좌표 이동 후 기존 `pick()`/`place()`를 호출한다.
- PM의 기존 드로잉 호출 경로와 Task Manager의 준비 → 드로잉 → 후처리 순서는 유지한다.

### 그리퍼 제어 방식 (실제 장비)
- `/onrobot/sendCommand` 대신 제어박스 디지털 출력 서비스
  `/dsr01/dsr_controller2/io/set_ctrl_box_digital_output`을 사용한다.
- 기본값: **DO2 = Open, DO1 = Close, 펄스 유지 0.5초**.
- 각 명령은 지정 포트를 ON → 0.5초 유지 → OFF 순서로 보내고,
  ON과 OFF의 서비스 결과를 모두 확인한다.
- ON 또는 OFF가 실패하면 오류를 기록하고 `gripper_output_fault`를 세운다.
  이 상태에서는 `_fail()`이 홈 복귀 자동 이동도 보내지 않는다.
- 서비스 호출이 실패해도 다른 포트로 자동 전환하지 않는다.

### 포트 전환 방법
`ws_cobot1/src/cake_decorator/config/workspace_config.yaml`의
`robot_executor_node.ros__parameters`에서 아래 값만 바꾼다.

```yaml
gripper_open_do: 2
gripper_close_do: 1
gripper_pulse_sec: 0.5
```

배선이 반대인 장비는 다음처럼 두 포트만 교환한다.

```yaml
gripper_open_do: 1
gripper_close_do: 2
gripper_pulse_sec: 0.5
```

검증 규칙:
- 포트는 정수 1~16이어야 한다.
- Open과 Close 포트가 같으면 오류로 처리한다.
- 펄스 시간은 0보다 큰 유한값이어야 한다.
- 검증 실패 시 `robot_executor_node`가 시작 단계에서 중단되어 로봇을 움직이지 않는다.
- 포트 설정은 로봇이 정지한 상태에서만 적용한다. 실행 중 변경은 거부된다
  (`configure_gripper(..., robot_stopped=True)` 명시 필요).

### 가상환경과 실제 장비의 차이
- 실제 장비: DO 서비스가 있으므로 항상 DO2/DO1 펄스를 사용한다.
- 가상환경: `set_ctrl_box_digital_output` 서비스가 없을 때만
  `gripper_virtual_fallback: true` 설정에 따라 `/onrobot/sendCommand`의
  `open(width)`/`close(width,force)`를 대체 사용한다.
- 대체 백엔드는 시작 시 한 번만 선택하며, DO 호출 실패 후 자동으로
  가상 백엔드로 넘어가지 않는다.
- 가상 RG2에서는 `opener_*_width_mm`, `opener_grip_force_n`이 실제 명령 인수로
  쓰이지만, 실제 DO 방식에서는 장비 배선·PM 설정이 동작을 결정하므로 호환용이다.

### 검증 범위
- 단위 테스트: 기본 DO2/DO1, 포트 교환, 범위·중복 검증, ON/OFF 실패 시 중단,
  `pick()`/`place()` 그리퍼 실패 시 후속 이동 없음, 플레이트 시퀀스의 함수 재사용.
- 실제 장비의 포트 전환·그리퍼 구동은 실물 확인 전 자동 실행하지 않는다.
