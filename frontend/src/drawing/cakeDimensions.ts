import measuredCake from "../../../config/test_cake.json" with { type: "json" };

/** 확정 원통형 케이크와 판 치수. 실행 위치·TCP 영점을 지정하는 값은 아니다. */
export const CAKE_DIMENSIONS = Object.freeze({
  diameterMm: measuredCake.diameter_mm,
  heightMm: measuredCake.height_mm,
  boardWidthMm: measuredCake.board_width_mm,
  boardDepthMm: measuredCake.board_depth_mm,
  boardThicknessMm: measuredCake.board_thickness_mm,
});

// 전송 계약의 캔버스 해상도는 유지한다. 편집 영역만 물리적 종횡비에 맞춘다.
const canvasWidth = 1024;
const canvasHeight = 768;
const sideWidth = 896;
// 원통의 전체 둘레와 높이를 그대로 펼친 띠 비율이다.
const circumferenceMm = Math.PI * CAKE_DIMENSIONS.diameterMm;
const sideHeight = (sideWidth * CAKE_DIMENSIONS.heightMm) / circumferenceMm;

/** 옆면은 원통의 전체 둘레를 펼친 편집 띠다. 로봇이 전 둘레에 접근 가능하다는 뜻은 아니다. */
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
