import test from "node:test";
import assert from "node:assert/strict";
import {
  CAKE_DIMENSIONS,
  DRAWING_LAYOUT,
  MODEL_SIZE,
} from "../src/drawing/cakeDimensions.ts";

// 확정 치수와 원통 옆면의 확대 비율을 함께 검증한다.
test("확정 원통형 케이크와 얇은 케이크판 크기를 사용한다", () => {
  assert.equal(CAKE_DIMENSIONS.diameterMm, 100);
  assert.equal(CAKE_DIMENSIONS.heightMm, 80);
  assert.equal(CAKE_DIMENSIONS.boardWidthMm, 179);
  assert.equal(CAKE_DIMENSIONS.boardDepthMm, 179);
  assert.equal(CAKE_DIMENSIONS.boardThicknessMm, 2);
  assert.equal(MODEL_SIZE.cakeHeight / (2 * MODEL_SIZE.cakeRadius), 0.8);
});

test("옆면 편집 영역은 원통 둘레와 높이 비율을 사용한다", () => {
  const side = DRAWING_LAYOUT.side;
  assert.ok(Math.abs(side.width / side.height - (Math.PI * 100) / 80) < 1e-10);
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
