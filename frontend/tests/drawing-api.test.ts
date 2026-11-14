import test from "node:test";
import assert from "node:assert/strict";
import { submitDrawing, type DrawingPayload } from "../src/api/drawing.ts";
// 옆면과 두 입력 점을 포함해 면 선택과 좌표가 전송 과정에서 보존되는지 확인한다.
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

// 실제 서버 없이 요청 본문을 관찰하고, 서버 접수와 장비 제작 완료를 혼동하지 않는지 검사한다.
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
  // 화면용 가공 필드가 끼거나 입력 점이 누락되지 않아야 한다.
  assert.deepEqual(sent, drawing);
});
// 빈 입력은 네트워크에 도달하기 전에 거절되어야 한다.
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
// 서버 장애를 정상 접수로 처리하면 고객이 재확인 기회를 잃으므로 오류를 유지한다.
test("서버 오류를 접수 완료로 표시하지 않는다", async (t) => {
  t.mock.method(
    globalThis,
    "fetch",
    async () => new Response("", { status: 503 }),
  );
  await assert.rejects(submitDrawing(drawing), /확인하지 못했습니다/);
});
// 성공 상태 코드여도 주문 식별자와 접수 상태가 없으면 기존 응답 계약 위반이다.
test("order_id 또는 received 없는 성공 응답도 접수로 간주하지 않는다", async (t) => {
  t.mock.method(globalThis, "fetch", async () =>
    Response.json({ status: "completed" }),
  );
  await assert.rejects(submitDrawing(drawing), /주문 접수 응답/);
});
