import assert from "node:assert/strict";
import { test } from "node:test";
import { isStale, latestStage, stages, fetchHmi } from "../src/api/hmi.ts";
import type { ProcessEvent } from "../src/api/hmi.ts";
test("전체10단계에 오프너 재파지·최종 반납·안전위치가 포함된다", () => {
  assert.equal(stages.length, 10);
  assert.deepEqual(
    stages.slice(-3).map((s) => s[0]),
    ["cake_to_outlet", "opener_return_after", "safe_position"],
  );
});
test("늦게 조회한 완료 이력을 최신 ID로 선택하고 없는 단계는 추정하지 않는다", () => {
  const events = [
    { id: 4, stage_key: "drawing", stage_state: "running" },
    { id: 6, stage_key: "drawing", stage_state: "completed" },
  ] as ProcessEvent[];
  assert.equal(latestStage(events, "drawing")?.stage_state, "completed");
  assert.equal(latestStage(events, "pen_return"), undefined);
});
test("조회 실패나5초 경과는 이전 데이터를 최신으로 표시하지 않는다", () => {
  assert.equal(isStale(1000, 2000, ""), false);
  assert.equal(isStale(1000, 7000, ""), true);
  assert.equal(isStale(1000, 2000, "연결 실패"), true);
});
test("실패한 HTTP 응답은 정상 주문 응답으로 처리하지 않는다", async () => {
  const previous = globalThis.fetch;
  globalThis.fetch = async () => new Response("unavailable", { status: 503 });
  try {
    await assert.rejects(
      fetchHmi("", new AbortController().signal),
      /HTTP 503/,
    );
  } finally {
    globalThis.fetch = previous;
  }
});
