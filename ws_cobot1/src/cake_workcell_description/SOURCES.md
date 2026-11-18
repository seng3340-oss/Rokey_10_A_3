# 모델 출처와 단위

현재 기준은 **지름100mm·높이80mm 원통, 원본 픽셀 입력, 윗면 중심**입니다.
[최신 통신·설정 규격](../../../docs/CYLINDER_PIXEL_CONTRACT.md)을 우선합니다. 아래 원뿔대·이전 치수 설명은 과거 테스트 기록입니다.


- A안 모델: 강승묵님, 2026-11-15 13:15 KST, a그룹-3조-실무프로젝트의 ros2_pkg.zip.
- 메시 링크: https://app.slack.com/archives/C0CGDTN7BPE/p1794716120726689
- ZIP SHA-256: 31929ba8db6ee898a856b377e09cfe83e48335ab1a4c28ed10283823a3db7104
- 원본 패키지명 cake_holder_ab_description, package.xml의 라이선스 표기는 MIT다.
- A_pegboard/A_frame/A_board/A_cake STL은 바이트를 수정하지 않고 복사했다. B안은 이번 통합에 포함하지 않았다.
- 기존 작업대와 mounting_ring: test_custom_mini_cake2/assets 모델. 원본 크기·좌표를 유지했다.
- A안 STL은 mm, 기존 테이블 STL은 m다. URDF에서 A안에만 0.001 스케일을 적용한다.
- A안 타공판은 250×250×20mm, 케이크판은 150×150×10mm, 케이크는 약 지름100×높이80mm다.
- A안 로컬 케이크 중심은 (85,85)mm, 상면 Z=150mm다. 타공판 밑면 Z=0을 상판에 맞춘다.
- 기존 테이블: 1200×650mm, BASE 기준 X=-275..925/Y=-325..325mm, 바닥 Z=-960mm.
- 상판 두께·구멍/바퀴 세부 치수 등 기존 임시값은 실제 측정값으로 취급하지 않는다.
- 이 모델은 RViz 표시용이다. A안 원본에는 collision 요소가 없으며 MoveIt 장면이나 물리 시뮬레이터에 등록하지 않았다.

## 스테이션 케이크

3D 프린트 규격(밑면 지름 120 mm, 윗면 지름 113.5 mm, 높이 80 mm)의 원뿔대를 `station_cake_frustum.stl`로 표시합니다.
저장소 루트에서 `python3 scripts/generate_test_cake_model.py --station`으로 재생성합니다.
STL 단위는 m이며, 밑면 중심을 스테이션 틀의 윗면 중심에 맞춥니다.

## 실측 플라스틱 테스트 케이크

기존 A안 원본 메시와 체크섬은 보존합니다. 현재 케이크 표시는 새 test_cake.stl, 판은 box 형상을 사용합니다.
config/test_cake.json과 scripts/generate_test_cake_model.py로 재생성합니다.
윗면113.5/아랫면120/높이79mm 원뿔대, 판179×179×2mm이며 위치는 기존 지그의 임시 배치를 유지합니다.
RViz 위치와 실제 로봇 TCP 영점은 자동 동기화되지 않습니다. 충돌 형상 정의만으로 실제 접촉이 검증되지는 않습니다.
