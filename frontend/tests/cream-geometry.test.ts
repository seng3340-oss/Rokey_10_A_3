import test from "node:test";
import assert from "node:assert/strict";
import {
  createCreamLayer,
  creamSurfacePoint,
} from "../src/drawing/creamGeometry.js";
import { DRAWING_LAYOUT, MODEL_SIZE } from "../src/drawing/cakeDimensions.ts";

test("크림 표시 좌표는 윗면 중심·옆면 이음매와 원뿔대 높이에 맞는다", () => {
  const center = creamSurfacePoint({ x: 512, y: 384 }, "top");
  assert.equal(center.x, 0);
  assert.equal(center.z, 0);
  assert.ok(center.y > MODEL_SIZE.cakeTop);
  const region = DRAWING_LAYOUT.side;
  const first = creamSurfacePoint({ x: region.x, y: region.y }, "side");
  const last = creamSurfacePoint(
    { x: region.x + region.width, y: region.y },
    "side",
  );
  assert.ok(first.distanceTo(last) < 1e-10);
  assert.equal(first.y, MODEL_SIZE.cakeTop);
  const bottom = creamSurfacePoint(
    { x: region.x, y: region.y + region.height },
    "side",
  );
  assert.equal(bottom.y, MODEL_SIZE.cakeBottom);
  assert.ok(Math.hypot(bottom.x, bottom.z) > Math.hypot(first.x, first.z));
});

test("입체 획은 원본을 보존하며 동일 획 재사용·수정·삭제 때 GPU 자원을 정리한다", () => {
  const layer = createCreamLayer();
  const drawing = {
    top: [
      [
        { x: 420, y: 350 },
        { x: 500, y: 300 },
        { x: 590, y: 380 },
      ],
    ],
    side: [
      [
        { x: 180, y: 350 },
        { x: 400, y: 370 },
        { x: 700, y: 340 },
      ],
    ],
  };
  const original = JSON.stringify(drawing);
  try {
    layer.update(drawing);
    assert.equal(layer.group.children.length, 2);
    const [top, side] = layer.group.children;
    const positions = top.geometry.getAttribute("position");
    const heights = Array.from({ length: positions.count }, (_, index) =>
      positions.getY(index),
    );
    assert.ok(Math.max(...heights) - Math.min(...heights) > 0.04);
    for (const mesh of layer.group.children) {
      assert.ok(
        Array.from(mesh.geometry.getAttribute("position").array).every(
          Number.isFinite,
        ),
      );
      assert.ok(
        Array.from(mesh.geometry.getAttribute("normal").array).every(
          Number.isFinite,
        ),
      );
    }
    layer.update(structuredClone(drawing));
    assert.equal(layer.group.children[0], top);
    assert.equal(layer.group.children[1], side);
    let disposed = 0;
    top.geometry.addEventListener("dispose", () => {
      disposed += 1;
    });
    layer.update({ top: [], side: drawing.side });
    assert.equal(disposed, 1);
    assert.equal(layer.group.children[0], side);
    layer.update({ top: [], side: [] });
    assert.equal(layer.group.children.length, 0);
    assert.equal(JSON.stringify(drawing), original);
  } finally {
    layer.dispose();
  }
});
