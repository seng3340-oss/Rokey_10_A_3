import test from "node:test";
import assert from "node:assert/strict";
import {
  CAKE_DIMENSIONS,
  DRAWING_LAYOUT,
  MODEL_SIZE,
} from "../src/drawing/cakeDimensions.ts";

// 실측 치수와 경사 옆면의 확대 비율을 함께 검증한다.
test("실측 원뿔대와 얇은 케이크판 크기를 사용한다", () => {
  assert.equal(CAKE_DIMENSIONS.diameterMm, 113.5);
  assert.equal(CAKE_DIMENSIONS.bottomDiameterMm, 120);
  assert.equal(CAKE_DIMENSIONS.heightMm, 79);
  assert.equal(CAKE_DIMENSIONS.boardWidthMm, 179);
  assert.equal(CAKE_DIMENSIONS.boardDepthMm, 179);
  assert.equal(CAKE_DIMENSIONS.boardThicknessMm, 2);
  assert.ok(
    Math.abs(
      MODEL_SIZE.cakeBottomRadius / MODEL_SIZE.cakeRadius - 120 / 113.5,
    ) < 1e-12,
  );
});

test("옆면 편집 영역은 평균 둘레와 경사 높이 비율을 사용한다", () => {
  const side = DRAWING_LAYOUT.side;
  const slant = Math.hypot(79, (120 - 113.5) / 2);
  assert.ok(
    Math.abs(side.width / side.height - (Math.PI * (113.5 + 120)) / 2 / slant) <
      1e-10,
  );
  assert.equal(side.x + side.width / 2, 512);
  assert.equal(side.y + side.height / 2, 384);
});

test("케이크 밑면은 받침판 위에 놓인다", () => {
  assert.equal(MODEL_SIZE.cakeBottom, MODEL_SIZE.boardThickness);
  assert.equal(
    MODEL_SIZE.cakeTop,
    MODEL_SIZE.cakeBottom + MODEL_SIZE.cakeHeight,
  );
});
