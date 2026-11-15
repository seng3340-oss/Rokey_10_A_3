import test from "node:test";
import assert from "node:assert/strict";
import {
  CAKE_DIMENSIONS,
  DRAWING_LAYOUT,
  MODEL_SIZE,
} from "../src/drawing/cakeDimensions.ts";

// A안 원본 STL의 mm 치수와 실제 미리보기 비율이 일치해야 한다.
test("A안 케이크와 사각 받침판 치수를 사용한다", () => {
  assert.equal(CAKE_DIMENSIONS.diameterMm, 100);
  assert.equal(CAKE_DIMENSIONS.heightMm, 80);
  assert.equal(CAKE_DIMENSIONS.boardWidthMm, 150);
  assert.equal(CAKE_DIMENSIONS.boardDepthMm, 150);
  assert.equal(CAKE_DIMENSIONS.boardThicknessMm, 10);
  assert.equal(MODEL_SIZE.cakeHeight / (2 * MODEL_SIZE.cakeRadius), 0.8);
  assert.equal(MODEL_SIZE.boardWidth / (2 * MODEL_SIZE.cakeRadius), 1.5);
});

test("옆면 편집 영역이 실제 둘레 대 높이 비율을 보존한다", () => {
  const side = DRAWING_LAYOUT.side;
  assert.ok(Math.abs(side.width / side.height - (Math.PI * 100) / 80) < 1e-10);
  assert.equal(side.x + side.width / 2, DRAWING_LAYOUT.width / 2);
  assert.equal(side.y + side.height / 2, DRAWING_LAYOUT.height / 2);
  assert.ok(side.x >= 0 && side.y >= 0);
  assert.ok(side.x + side.width <= DRAWING_LAYOUT.width);
  assert.ok(side.y + side.height <= DRAWING_LAYOUT.height);
});

test("받침판 윗면과 케이크 밑면이 맞닿는다", () => {
  assert.equal(MODEL_SIZE.cakeBottom, MODEL_SIZE.boardThickness);
  assert.equal(
    MODEL_SIZE.cakeTop,
    MODEL_SIZE.cakeBottom + MODEL_SIZE.cakeHeight,
  );
});
