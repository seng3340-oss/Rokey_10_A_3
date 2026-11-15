/**
 * Slack A안 STL의 치수(mm). 화면 표기·3D 형상·옆면 편집 영역의 공통 기준이다.
 * A_cake는 약 지름100×높이80, A_board는 150×150×10이다.
 * 원본 경로·단위는 ws_cobot1/src/cake_workcell_description/SOURCES.md를 참고한다.
 * 시각화 비율이며 실제 TCP 영점·로봇 작업 좌표를 자동 설정하는 값은 아니다.
 */
export const CAKE_DIMENSIONS = Object.freeze({
  diameterMm: 100,
  heightMm: 80,
  boardWidthMm: 150,
  boardDepthMm: 150,
  boardThicknessMm: 10,
});

// 전송 계약의 캔버스 해상도는 유지한다. 편집 영역만 물리적 종횡비에 맞춘다.
const canvasWidth = 1024;
const canvasHeight = 768;
const sideWidth = 896;
const circumferenceMm = Math.PI * CAKE_DIMENSIONS.diameterMm;
const sideHeight = (sideWidth * CAKE_DIMENSIONS.heightMm) / circumferenceMm;

/** 옆면은 원통의 전체 둘레를 펼친 띠다. 로봇이 전 둘레에 접근 가능하다는 뜻은 아니다. */
export const DRAWING_LAYOUT = Object.freeze({
  width: canvasWidth,
  height: canvasHeight,
  top: {
    x: 192,
    y: 64,
    width: 640,
    height: 640,
    centerX: 512,
    centerY: 384,
    radius: 320,
  },
  side: {
    x: (canvasWidth - sideWidth) / 2,
    y: (canvasHeight - sideHeight) / 2,
    width: sideWidth,
    height: sideHeight,
  },
});

// Three.js에서는 케이크 반지름을 1로 두되 모든 높이와 받침 크기에 같은 축척을 적용한다.
const mmPerUnit = CAKE_DIMENSIONS.diameterMm / 2;
const boardThickness = CAKE_DIMENSIONS.boardThicknessMm / mmPerUnit;
const cakeHeight = CAKE_DIMENSIONS.heightMm / mmPerUnit;
export const MODEL_SIZE = Object.freeze({
  cakeRadius: 1,
  cakeHeight,
  boardWidth: CAKE_DIMENSIONS.boardWidthMm / mmPerUnit,
  boardDepth: CAKE_DIMENSIONS.boardDepthMm / mmPerUnit,
  boardThickness,
  cakeBottom: boardThickness,
  cakeTop: boardThickness + cakeHeight,
  // 받침 아래부터 케이크 상면까지의 중심을 바라봐 위아래 잘림을 줄인다.
  focalHeight: (boardThickness + cakeHeight) / 2,
});
