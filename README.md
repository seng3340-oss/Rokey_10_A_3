# 로봇 레터링 케이크 이벤트

사용자가 태블릿에 그린 그림을 두산 M0609 로봇으로 케이크에 그려주는 행사 서비스입니다.
첫 목표는 고정된 위치의 정해진 크기 케이크 윗면과 옆면에 그림을 그리는 것입니다.
이후 케이크 운반·출고로 확장합니다. 기존 run3 검증은 윗면에 한정되며 옆면은 추가 검증이 필요합니다.

React 고객 화면·정적 HMI와 FastAPI 그림 접수·저장·주문 조회 서버를 구성했습니다. ROS 주문 연동은 아직 하지 않습니다.

- `frontend/`: 고객용 키오스크와 관리자 HMI
- `backend/`: FastAPI 그림 검증·주문 저장·조회
- `ws_cobot1/`: 프로젝트 ROS 2 패키지와 실행 문서
- `ws_dsr/`: 이관한 두산 로봇·RG2 실행환경 소스 (재빌드 필요)
- `docs/`: 설계와 환경 구성
- `docker/`, `DartPlatform/`: 실행환경 구성 및 시뮬레이터 설치 위치

[화면·기술 설계](docs/DESIGN.md) · [태블릿 연결](docs/NETWORK.md)

기존 `m0609_insert_gear/`, `python_insert_gear/` 예제는 유지합니다.
의존성과 빌드 산출물은 Git에 올리지 않으며 빈 폴더는 `.gitkeep`으로 유지합니다.

[로봇 실행환경 이관 및 빌드](docs/ROBOT_ENVIRONMENT.md)

[프론트 실행 방법](frontend/README.md) · [백엔드 구조](docs/BACKEND_PLAN.md)

[백엔드 실행 방법](backend/README.md)
