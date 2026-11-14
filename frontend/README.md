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

## Flask 연결

개발 중 브라우저는 같은 주소의 `/api`를 호출하며 Vite가 127.0.0.1:5000으로 중계합니다.
변경이 필요하면 `.env.example`을 `.env.local`로 복사하고 API_PROXY_TARGET을 설정합니다.
Flask가 아직 없으면 요청은 실패로 표시되고 그린 그림은 유지됩니다.
단순 HTTP 성공만으로 접수 완료를 표시하지 않고 order_id/status 응답을 검사합니다.
응답/HMI GET은 `src/api/`에 정의한 제안 계약이며 백엔드 구현 시 맞춰야 합니다.

## 빌드

```bash
npm test
npm run build
```

배포 시 `dist`와 `/api`를 Flask 또는 리버스 프록시가 동일 출처에서 제공해야 합니다.
`npm run preview`는 빌드 확인용이며 Flask 프록시를 제공하지 않습니다.

## 파일 역할

- `src/components/`: 고객·HMI·전체 확인 화면
- `src/drawing/controller.js`: 캔버스 입력, 점 저장, 상태·타이머 처리
- `src/drawing/cake3d.js`: Three.js 모형·회전·텍스처 처리와 해제
- `src/api/`: TypeScript 요청·응답 모델과 통신
- `src/styles/`: 고객/HMI 별도 스타일

그림 입력 좌표는 생략하지 않습니다. 2D 선을 즉시 표시하며 3D만 100ms 간격으로 갱신합니다.
두 면을 미리 보더라도 PM 규격에 따라 한 요청에는 선택한 한 면만 보냅니다.
현재 저장은 브라우저 메모리여서 새로고침하면 그림이 사라집니다.
ROS 실행과 run3 경로 엔진은 아직 연결하지 않았습니다. HMI는 실제 API 없이는 미연결 상태를 표시합니다.
