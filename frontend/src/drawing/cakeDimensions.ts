import measuredCake from "../../../config/test_cake.json" with { type: "json" };

/** 확정 원뿔대 케이크와 판 치수. 실행 위치·TCP 영점을 지정하는 값은 아니다. */
export const CAKE_DIMENSIONS = Object.freeze({
  topDiameterMm: measuredCake.top_diameter_mm,
  bottomDiameterMm: measuredCake.bottom_diameter_mm,
  heightMm: measuredCake.height_mm,
  // 스테이지와 굵은 펜 몸체의 간섭 때문에 옆면 바닥 10mm는 사용하지 않는다.
  sideBottomClearanceMm: 10,
  // 윗면 가장자리 5mm는 펜 입력을 제한한다. 좌표 축척은 그대로 둔다.
  topEdgeClearanceMm: 5,
  boardWidthMm: measuredCake.board_width_mm,
  boardDepthMm: measuredCake.board_depth_mm,
  boardThicknessMm: measuredCake.board_thickness_mm,
});

// 전송 계약의 캔버스 해상도는 유지한다. 편집 영역만 물리적 종횡비에 맞춘다.
const canvasWidth = 1024;
const canvasHeight = 768;
const sideWidth = 896;
// DP와 같은 평균 둘레·사면 길이를 사용해 같은 픽셀이 같은 높이·각도로 변환된다.
const circumferenceMm =
  (Math.PI *
    (CAKE_DIMENSIONS.topDiameterMm + CAKE_DIMENSIONS.bottomDiameterMm)) /
  2;
const slantHeightMm = Math.hypot(
  CAKE_DIMENSIONS.heightMm,
  (CAKE_DIMENSIONS.bottomDiameterMm - CAKE_DIMENSIONS.topDiameterMm) / 2,
);
const sideHeight = (sideWidth * slantHeightMm) / circumferenceMm;

/** 옆면은 원뿔대의 각도·높이를 대응시킨 편집 띠다. 로봇이 전 둘레에 접근 가능하다는 뜻은 아니다. */
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
    drawableRadius: 320 - (5 * 640) / CAKE_DIMENSIONS.topDiameterMm,
  },
  side: {
    x: (canvasWidth - sideWidth) / 2,
    y: (canvasHeight - sideHeight) / 2,
    width: sideWidth,
    height: sideHeight,
    // 전체 케이크를 계속 표시하고 입력 하한만 올린다. 그림을 위로 압축하지 않는다.
    drawableBottomY:
      (canvasHeight - sideHeight) / 2 +
      sideHeight * (1 - CAKE_DIMENSIONS.sideBottomClearanceMm / CAKE_DIMENSIONS.heightMm),
  },
});

// Three.js에서는 케이크 반지름을 1로 두되 모든 높이와 받침 크기에 같은 축척을 적용한다.
const mmPerUnit = CAKE_DIMENSIONS.topDiameterMm / 2;
const boardThickness = CAKE_DIMENSIONS.boardThicknessMm / mmPerUnit;
const cakeHeight = CAKE_DIMENSIONS.heightMm / mmPerUnit;
export const MODEL_SIZE = Object.freeze({
  cakeRadius: 1,
  cakeBottomRadius: CAKE_DIMENSIONS.bottomDiameterMm / (2 * mmPerUnit),
  cakeHeight,
  boardWidth: CAKE_DIMENSIONS.boardWidthMm / mmPerUnit,
  boardDepth: CAKE_DIMENSIONS.boardDepthMm / mmPerUnit,
  boardThickness,
  cakeBottom: boardThickness,
  cakeTop: boardThickness + cakeHeight,
  // 받침 아래부터 케이크 상면까지의 중심을 바라봐 위아래 잘림을 줄인다.
  focalHeight: (boardThickness + cakeHeight) / 2,
});
