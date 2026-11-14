# 케이크 드로잉 프론트

React·TypeScript·Vite와 Three.js를 사용합니다. 고객 `/`, 관리자 `/hmi` 화면을 제공합니다.
승인한 시안의 그리기, 윗면/옆면 3D, 제출 전 전체 확인, 60초 무조작 복귀를 옮겼습니다.

## 실행

Node.js 22.12 이상이 필요합니다. `.nvmrc`를 사용하는 경우 `nvm install && nvm use` 후 실행합니다.

```bash
cd ~/ws_cobot_pjt/frontend
npm ci
npm run dev
```

노트북: http://localhost:8080
같은 Wi-Fi 태블릿: http://노트북-Wi-Fi-IP:8080
HMI: http://노트북-Wi-Fi-IP:8080/hmi
8080에서 이전 HTML 서버가 실행 중이면 종료 후 시작합니다. 임시 시안과 React 앱을 동시에 같은 포트에 실행할 수 없습니다.

이 개발 컴퓨터에서는 전역 Node 18을 변경하지 않고 Node 22를 별도로 설치했습니다.
필요하면 현재 셸에서 `export PATH="$HOME/.local/share/cake-web-tooling/node_modules/.bin:$PATH"` 후 npm 명령을 실행합니다.

## FastAPI 연결

개발 중 브라우저는 같은 주소의 `/api`를 호출하며 Vite가 127.0.0.1:5000으로 중계합니다.
변경이 필요하면 `.env.example`을 `.env.local`로 복사하고 API_PROXY_TARGET을 설정합니다.
FastAPI 서버가 실행되지 않았으면 요청은 실패로 표시되고 그린 그림은 유지됩니다.
단순 HTTP 성공만으로 접수 완료를 표시하지 않고 order_id/status 응답을 검사합니다.
기존 드로잉 요청과 접수 응답 처리는 `src/api/drawing.ts`에 있습니다.
HMI 데이터와 연결 계약은 미확정이며, HMI API 모듈과 폴링은 제공하지 않습니다.

## 빌드

```bash
npm test
npm run build
```

배포 시 `dist`와 `/api`를 FastAPI 또는 리버스 프록시가 동일 출처에서 제공해야 합니다.
`npm run preview`는 빌드 확인용이며 FastAPI 프록시를 제공하지 않습니다.

## 파일 역할

- `src/components/`: 고객·HMI·전체 확인 화면
- `src/drawing/controller.js`: 캔버스 입력, 점 저장, 상태·타이머 처리
- `src/drawing/cake3d.js`: Three.js 모형·회전·텍스처 처리와 해제
- `src/api/drawing.ts`: 기존 드로잉 요청·접수 응답 모델과 통신
- `src/styles/`: 고객/HMI 별도 스타일

그림 입력 좌표는 생략하지 않습니다. 2D 선을 즉시 표시하며 3D만 100ms 간격으로 갱신합니다.
두 면을 미리 보더라도 PM 규격에 따라 한 요청에는 선택한 한 면만 보냅니다.
현재 저장은 브라우저 메모리여서 새로고침하면 그림이 사라집니다.
ROS 실행과 run3 경로 엔진은 아직 연결하지 않았습니다.

## HMI 표시 정책

`/hmi`는 기존 운영 콘솔 배치를 유지한 미연결 정적 예시 화면입니다.
주문 ID·상태·로그 시각은 화면 구성 확인용 고정 예시이며 실제 장비 상태가 아닙니다.
로봇 상태와 마지막 갱신 시각은 조회하지 않습니다. 새로고침 버튼은 비활성화되어 있고,
자동 갱신·폴링·HMI API 호출·장비 제어는 없습니다. 데이터 계약 확정 전에는 연결 기능을 추가하지 않습니다.

## 코드 읽기와 설정

직접 관리하는 소스에는 역할과 처리 흐름을 설명하는 한글 주석을 작성합니다.
입력 좌표 환산, 두 면의 텍스처 변환, 선택 면 제출, 타이머와 WebGL 자원 해제의 이유를 설명하고,
긴 JSX·조건식·객체는 의미 단위로 줄을 나눕니다.
외부 벤더 코드, lockfile, 자동 생성 파일과 `dist/`에는 주석을 추가하지 않습니다.

주석을 지원하지 않는 JSON 설정은 유효한 형식을 유지하며 목적은 다음과 같습니다.

- `package.json`: 개발·테스트·빌드 명령과 기존 React·Three.js·Vite 의존성을 정의합니다.
- `tsconfig.json`: ES2022·DOM 환경, 엄격한 TypeScript 검사와 React JSX 변환을 사용합니다.
  기존 JS 드로잉 모듈은 `allowJs`로 포함하고 `checkJs: false`를 유지합니다.
  타입 검사는 파일을 출력하지 않으며 배포 파일은 Vite가 생성합니다.
- `vite.config.ts`: 개발 중 `/api`를 `API_PROXY_TARGET`으로 중계합니다.
  기본 대상은 `http://127.0.0.1:5000`이며 HMI는 이 경로를 호출하지 않습니다.
