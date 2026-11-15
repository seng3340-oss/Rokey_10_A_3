# 모델 출처와 단위

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
