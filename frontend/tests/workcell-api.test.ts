import test from "node:test";
import assert from "node:assert/strict";
import {
  mergeWorkcellLogs,
  parseWorkcellLogs,
  parseWorkcellRecovery,
  sendWorkcellDecision,
} from "../src/api/hmi.ts";

// 물리 안전 상태와 구분해 실패 세대와 주문 접수 잠금만 검사한다.
const locked = {
  revision: 2,
  requires_reset: true,
  can_reset: true,
  can_resume: true,
  resume_requested: false,
  stopped_stage: "DRAWING",
  stopped_label: "옆면 펜 파지",
  order_id: "ORD-TEST",
  error_code: "CAKE_WEIGHT_FAILED",
  message: "재시도 후에도 케이크 없음",
  updated_at: "2026-11-20T09:00:00Z",
  reset_at: null,
};

test("잘못된 잠금 응답을 주문 가능으로 해석하지 않는다", () => {
  assert.equal(parseWorkcellRecovery(locked).requires_reset, true);
  for (const invalid of [
    { ...locked, requires_reset: "false" },
    { ...locked, revision: 0 },
    { ...locked, order_id: "" },
    { ...locked, requires_reset: false, can_reset: true },
  ])
    assert.throws(() => parseWorkcellRecovery(invalid));
});

test("관리자 재개는 표시된 실패 세대로 동일 주문 예약만 요청한다", async (t) => {
  let request: { url: unknown; options: RequestInit | undefined } | null = null;
  t.mock.method(
    globalThis,
    "fetch",
    async (url: unknown, options?: RequestInit) => {
      request = { url, options };
      return Response.json({
        ...locked,
        requires_reset: true,
        can_reset: false,
        can_resume: false,
        resume_requested: true,
        reset_at: null,
      });
    },
  );
  const controller = new AbortController();
  assert.equal(
    (await sendWorkcellDecision("resume", 2, controller.signal))
      .resume_requested,
    true,
  );
  assert.equal(request!.url, "/api/workcell/resume");
  assert.equal(request!.options!.method, "POST");
  assert.deepEqual(JSON.parse(request!.options!.body as string), {
    revision: 2,
    confirmed_reset: true,
  });
  assert.equal(request!.options!.signal, controller.signal);
  t.mock.method(globalThis, "fetch", async () =>
    Response.json({ detail: "실패 세대 변경" }, { status: 409 }),
  );
  await assert.rejects(
    sendWorkcellDecision("resume", 2, controller.signal),
    /실패 세대 변경/,
  );
});

test("스크롤 페이지와 새 로그가 동시에 와도 ID 중복 없이 최신순으로 합친다", () => {
  const row = (id: number) => ({
    id,
    created_at: "2026-11-20T09:00:00Z",
    order_id: "ORD-TEST",
    severity: "error" as const,
    message: "재시도 실패",
  });
  const initial = parseWorkcellLogs({
    items: [row(5), row(4)],
    has_more: true,
  });
  const older = mergeWorkcellLogs(initial.items, [row(4), row(3), row(2)]);
  const combined = mergeWorkcellLogs(older, [row(6), row(7)]);
  assert.deepEqual(
    combined.map((item) => item.id),
    [7, 6, 5, 4, 3, 2],
  );
  for (const invalid of [
    { items: [row(-1)], has_more: false },
    { items: [{ ...row(1), created_at: "invalid" }], has_more: false },
    { items: [row(1)], has_more: "false" },
  ])
    assert.throws(() => parseWorkcellLogs(invalid));
});
