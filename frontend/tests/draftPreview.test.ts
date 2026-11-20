import test from "node:test";
import assert from "node:assert/strict";
import { parseDraft } from "../src/api/draftPreview.ts";
const preview = () => ({
  kind: "draft-preview",
  surface: "top",
  drawings: ["top", "side"].map((drawing_surface) => ({
    canvas_width: 1024,
    canvas_height: 768,
    drawing_surface,
    strokes: [
      {
        stroke_id: 0,
        points: [
          { x: 500, y: 350 },
          { x: 550, y: 380 },
        ],
      },
    ],
  })),
});
test("작성 중 두 면을 주문과 분리해 검증한다", () => {
  assert.ok(parseDraft(preview()));
  assert.equal(parseDraft({ kind: "request-draft" }), null);
});
test("잘못된 면·좌표·크기의 초안을 거부한다", () => {
  const data = preview();
  data.drawings[0].drawing_surface = "side";
  assert.equal(parseDraft(data), null);
  const invalid = preview();
  invalid.drawings[0].strokes[0].points[0].x = Infinity;
  assert.equal(parseDraft(invalid), null);
  const large = preview();
  large.drawings[0].strokes[0].points = Array(20001).fill({ x: 500, y: 350 });
  assert.equal(parseDraft(large), null);
});
