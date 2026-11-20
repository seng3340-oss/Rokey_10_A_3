# 옆면 그리기 확정 설정

2026-11-19 실물 시험 기준값에 2026-11-20 요청한 중심·유격 보정을 반영했다.
상면·옆면 중심을 각각 BASE X -1mm, Y -0.8mm 옮겼으며 전체 지름 유격은 0.3mm 줄였다.
이번 보정값의 실물 운전은 수행하지 않았다.

| 항목 | 확정 값 | 설정 위치 |
|---|---|---|
| 옆면용 케이크 윗면 중심 BASE XYZ | 395.6743225097656, -51.35932998657227, 222.0 mm | `config/real_robot.json` |
| Y 양쪽 안쪽 보정 | 각각 1mm, 전체 Y 폭 2mm 감소 | `scripts/start_real_side_web_test.sh`의 `--side-y-inset-mm 1` |
| 옆면 지름 유격 | 6.2mm, 반지름 방향으로 3.1mm 추가 | `ws_cobot1/src/cake_decorator/config/workspace_config.yaml`의 `side_diameter_allowance_mm` |
| 옆면 TCP 이름 | `Pen_Side_45` | 펜던트 등록값 및 `bent_pen.py` |
| 펜촉 TCP XYZ / ABC | 1.095, -46.474, 271.695 mm / 0, 0, 0도 | 펜던트 등록값 및 `bent_pen.py`의 검증 기준 |
| 펜 파지 회전 중심 | 0, 0, 220mm | `scripts/real_pen_pick_45.py`의 `GRIP_CENTER_MM` |

Y 보정은 중심에 대해 sin(theta)로 부드럽게 적용한다. X/Z는 유지하고 Y 양끝만 각각 1mm 줄인다.
지름 유격과 함께 적용되므로 같은 높이에서 X 양끝은 원래 표면보다 3.1mm, Y 양끝은 2.1mm 바깥이다.
펜 파지 중심 TCP와 펜촉 TCP는 용도가 다르다. 220mm는 회전할 때 지문 중앙을 고정하기 위한 값이고 펜촉 길이가 아니다.
TCP의 XYZ는 로봇 BASE 기준 케이크 좌표가 아니라 플랜지 기준 펜촉 오프셋이다.
ABC가 0이어도 펜은 물리적으로 45도로 잡혀 있다. TCP 축을 플랜지 축과 같게 정의한 것이다.

## 어떤 파일을 바꾸는가

### config/real_robot.json — 실제 케이크 위치

- `side_cake_center_xy_mm`: 옆면 시험에서 접근할 케이크 상면 중심 BASE X/Y(mm).
- `top_cake_center_xy_mm`: 상면용 TCP 기준 중심 BASE X/Y(mm). 현재 [399.5243225097656, -53.80932998657227]이며 옆면과의 기존 보정 차이를 유지한다.
- `cake_top_z_mm`: 같은 중심의 BASE Z(mm). 테이블 기준 케이크 높이가 아니다.
- `configure_pen_tcp_on_yes`: 일반 실물 실행의 수직 접촉 TCP 추정 기능. 45도 옆면 시험은 이 기능을 건너뛰고 `Pen_Side_45`를 적용한다.

옆면 시험은 이 위치로 이동하고 실제 도달한 펜촉 좌표를 해당 실행의 중심으로 기록한다.
현재 두 실물 시험 스크립트는 YES 입력 없이 진행한다.

### config/test_cake.json — 케이크 자체 치수

- `top_diameter_mm`: 윗지름 113.5mm.
- `bottom_diameter_mm`: 밑지름 120mm.
- `height_mm`: 케이크 높이 79mm. BASE Z=222mm와 다르다. 현재 바닥 높이는 222−79=143mm다.
- `board_width_mm`, `board_depth_mm`, `board_thickness_mm`: 받침판 179×179×2mm.
- `tool_type`: 도구 종류 `pen`.
- `reference`: `tcp_at_top_center`, 위치 기준이 케이크 상면 중앙이라는 뜻.

백엔드 주문 치수와 웹 모델이 공유한다. 케이크 위치를 옮기려고 이 치수를 변경하지 않는다.

### workspace_config.yaml — ROS 경로·동작 설정

- `side_diameter_allowance_mm`: 실제 케이크 크기는 그대로 두고 옆면 펜촉 경로만 바깥으로 넓힌다. 지름 기준이므로 반으로 나눠 반지름에 적용한다.
- `side_seam_overlap_mm`: 웹 옆면 띠의 왼쪽 끝과 오른쪽 끝이 비슷한 높이(3mm 이내)로 이어진 링일 때 이음매를 지나 더 그리는 길이(mm). 현재 3mm, 0이면 끈다. 한쪽 끝만 이음매에 닿은 획은 연장하지 않는다. 웹은 띠 좌우 경계에서 약 12px(≈5mm) 안의 시작·끝점을 경계에 맞춰 보낸다.
- `drawing_z_offset`: 접촉 경로 보정(mm). 윗면은 높이, 옆면은 표면 바깥 법선 방향 보정이다. 현재 0.
- DP의 `approach_height`: 접근·이탈 거리 10mm. 케이크 중심 높이가 아니다.
- RE의 같은 이름 `approach_height`: 별도 실행기 설정 0.05m(50mm). DP의 10mm와 단위·역할이 다르므로 함께 바꾸지 않는다.
- `side_pen_vertical_deg`: 기존 옆면 방식의 17도 자세 설정. 45도 시험에서는 최종 자세를 별도 계산으로 교체하므로 이 값을 45로 바꾸지 않는다.
- `pen_tilt_deg`: 윗면 기준 자세가 없을 때 쓰는 기본 기울기. 현재 0도.
- `cake_radius`, `cake_bottom_radius`, `cake_height`, `cake_orientation`: 기존 문서용 치수(m)/방향(rad). 현재 경로 계산은 주문 요청에 들어온 치수·중심·회전각을 사용한다. 여기서 중심을 수정하지 않는다.

`side_y_inset_mm`는 실행기가 명령줄 값을 임시 YAML에 넣는다. 원본 YAML에 없더라도 정상이다.
`side_pen_45.yaml`은 실행별 설정 사본이므로 직접 편집하지 않는다.

### config/local/pen_tcp*.json — 이전 교정 시험 기록

현재 옆면 실행 경로에서는 이 파일들을 읽지 않는다.
- `tcp_pose_mm_deg`: 당시 추정한 플랜지 기준 TCP XYZ/ABC.
- `reference_point_base_mm`: 당시 교정할 때 사용한 BASE 기준 고정점. 현재 케이크 중심이 아니다.
- `samples`, `max_residual_mm`, `rms_residual_mm`, `condition_number`: 교정 표본과 계산 품질 기록.
- `tcp_name`, `max_error_mm`, `quality`: 당시 TCP 이름, 허용 오차, 검증 상태.

현재 `quality`는 `manual_trial_unverified`다. 확정된 `Pen_Side_45` 값으로 혼동하지 않는다.

### cake_workcell_bringup/config/workcell.yaml — 기존 가상 배치

`fixture.x_mm/y_mm/z_mm/yaw_deg`는 legacy 가상 고정구의 원점이다. 케이크 중심이 아니다.
45도 옆면 가상 시험은 `real_robot.json`의 중심에 맞춰 모델을 배치하므로 이 값으로 실물 중심을 조정하지 않는다.

## 실행과 기록

```bash
bash scripts/start_real_pen_pick_45.sh
bash scripts/start_real_side_web_test.sh
```

위 두 명령은 실제 로봇 이동을 시작한다. 웹의 그리기 버튼을 누르면 제출한 그림을 실행한다.
옆면 작업 중에만 펜촉 TCP를 적용하고, 작업 종료 시 기본 0,0,0,0,0,0 TCP로 복원한다.
통신 단절·강제 종료 시에는 복원 여부를 확인해야 한다.

`log/system/<실행시각>/reference.json`은 해당 실행에서 기록한 중심과 시작 위치의 결과물이다.
예전 로그 좌표를 다음 실행의 설정으로 자동 사용하지 않는다.
