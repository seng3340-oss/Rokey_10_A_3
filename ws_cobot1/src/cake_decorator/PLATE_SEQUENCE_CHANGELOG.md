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
- DO1/DO2 펄스 대신 기존 RG2 `open`/`close` 서비스 사용. 원본 대기시간을 작업별 설정으로 보존.
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
