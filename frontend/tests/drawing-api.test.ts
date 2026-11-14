import test from "node:test";
import assert from "node:assert/strict";
import { submitDrawing, type DrawingPayload } from "../src/api/drawing.ts";
const drawing: DrawingPayload = {
  canvas_width: 1024,
  canvas_height: 768,
  drawing_surface: "side",
  strokes: [
    {
      stroke_id: 0,
      points: [
        { x: 100, y: 300 },
        { x: 120, y: 320 },
      ],
    },
  ],
};

test("PM 형식 그대로 전송하고 주문 접수 응답을 구분한다", async (t) => {
  let sent: unknown;
  t.mock.method(globalThis, "fetch", async (url: string, init: RequestInit) => {
    assert.equal(url, "/api/drawing");
    assert.equal(init.method, "POST");
    sent = JSON.parse(init.body as string);
    return Response.json(
      { order_id: "ORD-TEST-001", status: "received" },
      { status: 201 },
    );
  });
  assert.deepEqual(await submitDrawing(drawing), {
    order_id: "ORD-TEST-001",
    status: "received",
  });
  assert.deepEqual(sent, drawing);
});
test("빈 그림은 서버에 요청하지 않는다", async (t) => {
  const fetch = t.mock.method(globalThis, "fetch", async () => {
    throw new Error("호출되면 실패");
  });
  await assert.rejects(
    submitDrawing({ ...drawing, strokes: [] }),
    /그림이 없습니다/,
  );
  assert.equal(fetch.mock.callCount(), 0);
});
test("서버 오류를 접수 완료로 표시하지 않는다", async (t) => {
  t.mock.method(
    globalThis,
    "fetch",
    async () => new Response("", { status: 503 }),
  );
  await assert.rejects(submitDrawing(drawing), /확인하지 못했습니다/);
});
test("order_id 또는 received 없는 성공 응답도 접수로 간주하지 않는다", async (t) => {
  t.mock.method(globalThis, "fetch", async () =>
    Response.json({ status: "completed" }),
  );
  await assert.rejects(submitDrawing(drawing), /주문 접수 응답/);
});
