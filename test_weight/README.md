# 플레이트 하중 측정 실험 (test_weight)

- 작성일: 2026-11-19
- 목적: 플레이트 시퀀스의 Safe Position에서 보유 물품(따개/케이크) 하중 변화를 반복 측정해 판정 기준을 검토한다.
- 환경: ROS2 Jazzy, Doosan M0609, OnRobot RG2, `ROS_DOMAIN_ID=30`
- 측정: 3사이클 × 단계별 5회, 단위 g(소수점 제외)
- 좌표: `workspace_config.yaml`의 `plate_safe_position_1..5` (현재 5개 좌표 모두 동일)
- 샘플 간격: 0.3초 요청이었으나 `get_workpiece_weight` 서비스 응답 지연으로 실제 약 0.7초
- 주의: 보정된 절대 저울값이 아니라 로봇의 작업물 추정 하중이다.

## 측정 단계와 보유 물품

| 단계 | 보유 물품 |
|---|---|
| prepare_start | 빈 그리퍼(기준) |
| prepare_opener | 따개 |
| prepare_both | 따개 + 케이크 |
| prepare_opener_after_cake | 따개(케이크 내려놓은 뒤) |
| prepare_end | 빈 그리퍼(따개 반납 뒤) |
| finish_start | 빈 그리퍼(기준) |
| finish_opener | 따개 |
| finish_both | 따개 + 케이크 |
| finish_opener_after_cake | 따개(케이크 출고 뒤) |
| finish_end | 빈 그리퍼(따개 반납 뒤) |

## 1회차 원시 측정값 (g)

| 단계 | S1 (g) | S2 (g) | S3 (g) | S4 (g) | S5 (g) |
|---|---:|---:|---:|---:|---:|
| prepare_start | 1188 | 1200 | 1220 | 1233 | 1224 |
| prepare_opener | 1267 | 1196 | 1203 | 1219 | 1236 |
| prepare_both | 1341 | 1394 | 1399 | 1390 | 1364 |
| prepare_opener_after_cake | 1193 | 1194 | 1199 | 1206 | 1191 |
| prepare_end | 1281 | 1233 | 1239 | 1204 | 1198 |
| finish_start | 1204 | 1213 | 1235 | 1249 | 1222 |
| finish_opener | 1305 | 1200 | 1207 | 1210 | 1228 |
| finish_both | 1391 | 1375 | 1364 | 1353 | 1335 |
| finish_opener_after_cake | 1278 | 1285 | 1225 | 1221 | 1246 |
| finish_end | 1249 | 1180 | 1192 | 1205 | 1220 |

## 2회차 원시 측정값 (g)

| 단계 | S1 (g) | S2 (g) | S3 (g) | S4 (g) | S5 (g) |
|---|---:|---:|---:|---:|---:|
| prepare_start | 1208 | 1222 | 1242 | 1228 | 1216 |
| prepare_opener | 1235 | 1175 | 1192 | 1204 | 1217 |
| prepare_both | 1354 | 1357 | 1388 | 1422 | 1409 |
| prepare_opener_after_cake | 1193 | 1222 | 1226 | 1130 | 1187 |
| prepare_end | 1254 | 1212 | 1225 | 1214 | 1188 |
| finish_start | 1192 | 1192 | 1217 | 1238 | 1230 |
| finish_opener | 1256 | 1188 | 1213 | 1180 | 1187 |
| finish_both | 1411 | 1365 | 1359 | 1355 | 1358 |
| finish_opener_after_cake | 1278 | 1241 | 1195 | 1204 | 1212 |
| finish_end | 1247 | 1176 | 1190 | 1208 | 1219 |

## 3회차 원시 측정값 (g)

| 단계 | S1 (g) | S2 (g) | S3 (g) | S4 (g) | S5 (g) |
|---|---:|---:|---:|---:|---:|
| prepare_start | 1195 | 1200 | 1222 | 1230 | 1201 |
| prepare_opener | 1283 | 1199 | 1199 | 1209 | 1229 |
| prepare_both | 1358 | 1356 | 1399 | 1403 | 1405 |
| prepare_opener_after_cake | 1246 | 1229 | 1215 | 1224 | 1246 |
| prepare_end | 1232 | 1188 | 1219 | 1216 | 1200 |
| finish_start | 1200 | 1212 | 1224 | 1239 | 1234 |
| finish_opener | 1269 | 1204 | 1190 | 1206 | 1226 |
| finish_both | 1387 | 1351 | 1372 | 1342 | 1337 |
| finish_opener_after_cake | 1305 | 1251 | 1202 | 1215 | 1237 |
| finish_end | 1225 | 1170 | 1170 | 1188 | 1209 |

## 단계별 요약 (g)

| 단계 | 1회차 중앙값 (g) | 2회차 중앙값 (g) | 3회차 중앙값 (g) | 전체 중앙값 (g) | 최소 (g) | 최대 (g) | 범위 (g) |
|---|---:|---:|---:|---:|---:|---:|---:|
| prepare_start | 1220 | 1222 | 1201 | 1220 | 1188 | 1242 | 55 |
| prepare_opener | 1219 | 1204 | 1209 | 1209 | 1175 | 1283 | 108 |
| prepare_both | 1390 | 1388 | 1399 | 1390 | 1341 | 1422 | 82 |
| prepare_opener_after_cake | 1194 | 1193 | 1229 | 1206 | 1130 | 1246 | 116 |
| prepare_end | 1233 | 1214 | 1216 | 1216 | 1188 | 1281 | 93 |
| finish_start | 1222 | 1217 | 1224 | 1222 | 1192 | 1249 | 57 |
| finish_opener | 1210 | 1188 | 1206 | 1207 | 1180 | 1305 | 125 |
| finish_both | 1364 | 1359 | 1351 | 1359 | 1335 | 1411 | 77 |
| finish_opener_after_cake | 1246 | 1212 | 1237 | 1237 | 1195 | 1305 | 110 |
| finish_end | 1205 | 1208 | 1188 | 1205 | 1170 | 1249 | 79 |

## 케이크 신호 (중앙값 차이, g)

| 회차 | 공정 | both - opener (g) | both - opener_after_cake (g) | both - start (g) |
|---|---|---:|---:|---:|
| 1 | prepare | +171.4 | +195.9 | +170.0 |
| 1 | finish | +153.4 | +117.4 | +141.6 |
| 2 | prepare | +184.1 | +194.7 | +165.2 |
| 2 | finish | +171.2 | +147.6 | +142.3 |
| 3 | prepare | +190.5 | +170.3 | +198.3 |
| 3 | finish | +144.6 | +113.4 | +126.6 |

## 관찰

- `both - opener` 기준 케이크 기여분: 최소 +144.6g, 최대 +190.5g
- `both - opener_after_cake` 기준: 최소 +113.4g, 최대 +195.9g
- `both - start` 기준: 최소 +126.6g, 최대 +198.3g
- 따개 단독은 빈 그리퍼와 구분되지 않는다(회차 간 노이즈가 따개 무게와 비슷하거나 큼).
- 케이크는 `both` 상태에서 뚜렷하게 증가하지만, 절대 무게는 설정값 200g과 다르며 보정이 필요하다.
- 같은 단계에서도 첫 샘플이 튀는 경우가 있어, 첫 샘플 제외 또는 추가 정착 대기를 검토해야 한다.
- 서비스 응답 지연으로 실제 샘플 간격은 약 0.7초였다. 0.3초 간격을 보장하려면 별도 인터페이스가 필요하다.

## 실행 방법

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source ~/Rokey_10_A_3/ws_cobot1/install/setup.bash
export ROS_DOMAIN_ID=30

# 이동 없이 확인
python3 test_weight/measure_plate_payload_weights.py --dry-run

# 3사이클 × 단계별 5회 (회차 사이 60초 리셋 대기)
python3 test_weight/measure_plate_payload_weights.py \
  --yes --runs 3 --reads 5 --reset-wait 60
```

## 파일

- `measure_plate_payload_weights.py`: 원본 코드 수정 없이 `PlateSequence`를 상속해 하중만 기록하는 측정 도구
- `data/plate_payload_weight_samples_20261119-183328.csv`: 3사이클 × 단계별 5회 원시 샘플
- `data/plate_payload_weights_20261119-183328.csv`: 단계별 대표값(중앙값)과 기준값

## 특이사항

- 실험 중 로봇이 `SAFE_OFF(3)`로 전환되거나 RT 제어 스트림이 끊기는 현상이 있었다. 이 경우 모션이 실행되지 않으므로 펜던트에서 STANDBY 복귀 또는 드라이버 재기동이 필요하다.
- 드라이버를 중복 기동하면 RT 스트림이 꼬일 수 있으므로, 실행 전 `ros2 launch m0609_rg2_bringup` 프로세스가 하나인지 확인한다.
- 이 실험은 힘/토크 기반 작업물 추정 하중이며, 정밀 저울이 아니다. 실패 기준 확정 전 실제 무게(저울)와 파지 성공 여부를 함께 확인해야 한다.
