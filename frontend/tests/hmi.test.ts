import assert from "node:assert/strict";
import test from "node:test";
import {
  parseDetail,
  parsePlan,
  parseRuntime,
  isOrderFeedbackFresh,
  parseOrder,
  parsePage,
  readJson,
  collectOrderChanges,
} from "../src/api/hmi.ts";

const order = {
  order_id: "ORD-test",
  status: "running",
  drawing_surface: "top",
  created_at: "2026-11-19T11:00:00+09:00",
  phase: "DRAWING",
  progress_pct: 42,
  message: "획 진행",
};
/** HMI는 알 수 없는 상태나 잘못된 진행률을 정상으로 표시하지 않는다. */
test("order contract rejects unknown states and nonfinite progress", () => {
  assert.equal(parseOrder(order).progress_pct, 42);
  for (const value of [NaN, Infinity, -1, 101, "42"]) {
    assert.throws(() => parseOrder({ ...order, progress_pct: value }));
  }
  assert.throws(() => parseOrder({ ...order, status: "safe" }));
  assert.throws(() => parsePage({ items: [order], total: 0 }));
});
/** 원본의 픽셀 경계를 검사해 깨진 그림이 실행 경로처럼 보이지 않게 한다. */
test("drawing keeps original coordinates and validates boundaries", () => {
  const drawing = {
    canvas_width: 1024,
    canvas_height: 768,
    drawing_surface: "top",
    strokes: [
      {
        stroke_id: 0,
        points: [
          { x: 100, y: 200 },
          { x: 300, y: 400 },
        ],
      },
    ],
  };
  assert.deepEqual(parseDetail({ ...order, drawing }).drawing, drawing);
  assert.throws(() =>
    parseDetail({ ...order, drawing: { ...drawing, drawing_surface: "side" } }),
  );
  assert.throws(() =>
    parseDetail({ ...order, drawing: { ...drawing, canvas_width: 0 } }),
  );
});
/** 네트워크 경로에는 GET만 전달되고 서버 오류가 화면 오류로 전달된다. */
test("HMI transport only sends GET and surfaces errors", async () => {
  const original = globalThis.fetch;
  const calls: RequestInit[] = [];
  globalThis.fetch = async (_url, options) => {
    calls.push(options!);
    return new Response("{}", { status: 503 });
  };
  try {
    await assert.rejects(
      readJson("/api/orders", new AbortController().signal),
      /503/,
    );
    assert.equal(calls[0].method, "GET");
    assert.equal(calls[0].body, undefined);
    assert.equal(calls[0].headers, undefined);
  } finally {
    globalThis.fetch = original;
  }
});

/** 다른 주문을 보고 있어도 새 주문의 접수·진행·완료·오류 변화가 로그에 남아야 한다. */
test("collects all order transitions and unselected failures without duplicates", () => {
  let previous = new Map<string, string>();
  for (const status of ["received", "running", "completed"]) {
    const state = collectOrderChanges(previous, [
      parseOrder({ ...order, status }),
    ]);
    assert.equal(state.changes.length, 1);
    previous = state.signatures;
    assert.equal(
      collectOrderChanges(previous, [parseOrder({ ...order, status })]).changes
        .length,
      0,
    );
  }
  const state = collectOrderChanges(previous, [
    parseOrder(order),
    parseOrder({
      ...order,
      order_id: "another-order",
      status: "failed",
      error_code: "DEMO_ERROR",
    }),
  ]);
  const failure = state.changes.find((change) =>
    change.message.includes("another-order"),
  );
  assert.equal(failure?.severity, "error");
  assert.match(failure!.message, /DEMO_ERROR/);
  assert.throws(() => parseOrder({ ...order, error_code: 123 }));
});

/** 수신 경로는 주문·면·해상도가 일치할 때만 원본에 겹친다. */
test("TM 계획은 주문·면·생산자와 실측 아님 표시를 검증한다", () => {
  const detail = parseDetail({
    ...order,
    drawing: {
      canvas_width: 1024,
      canvas_height: 768,
      drawing_surface: "top",
      strokes: [
        {
          stroke_id: 0,
          points: [
            { x: 500, y: 350 },
            { x: 520, y: 360 },
          ],
        },
      ],
    },
  });
  const plan = {
    ...detail.drawing,
    version: 1,
    source: "task_manager_node/GenerateTrajectory",
    order_id: order.order_id,
    waypoint_count: 4,
    execution_mode: "real",
    updated_at: "now",
    measurement_available: false,
  };
  assert.equal(parsePlan(plan, detail).execution_mode, "real");
  for (const change of [
    { order_id: "wrong" },
    { drawing_surface: "side" },
    { source: "replay" },
    { measurement_available: true },
    { execution_mode: "virtual" },
    { canvas_width: 200 },
  ])
    assert.throws(() => parsePlan({ ...plan, ...change }, detail));
});
test("연결 관측은 실물 안전을 추정하지 않는다", () => {
  const status = {
    observed: true,
    bridge_fresh: true,
    task_server_ready: true,
    task_feedback_fresh: false,
    execution_mode: "real",
    active_order_id: null,
    feedback_age_sec: null,
    updated_at: "now",
    robot_connected: null,
    safety_status: "unknown",
  };
  assert.equal(parseRuntime(status).task_server_ready, true);
  assert.throws(() => parseRuntime({ ...status, robot_connected: true }));
  assert.throws(() => parseRuntime({ ...status, feedback_age_sec: NaN }));
});

test("선택한 다른 주문에 heartbeat를 잘못 붙이지 않는다", () => {
  const snapshot = parseRuntime({
    observed: true,
    bridge_fresh: true,
    task_server_ready: true,
    task_feedback_fresh: true,
    execution_mode: "real",
    active_order_id: "new-order",
    feedback_age_sec: 1,
    updated_at: "now",
    robot_connected: null,
    safety_status: "unknown",
  });
  assert.equal(isOrderFeedbackFresh(snapshot, "old-order"), false);
  assert.equal(isOrderFeedbackFresh(snapshot, "new-order"), true);
  assert.equal(
    isOrderFeedbackFresh({ ...snapshot, bridge_fresh: false }, "new-order"),
    false,
  );
});

// 최신 공정의 표시 이름과 미정 공정의 원문 보존을 함께 확인한다.
import { phaseLabel } from "../src/api/hmi.ts";
test("최신 플레이트 공정과 미정 상태를 구분한다", () => {
  assert.equal(phaseLabel("PREPARING"), "플레이트 준비");
  assert.equal(phaseLabel("FINISHING"), "회수·출고");
  assert.equal(phaseLabel("CUSTOM_PHASE"), "CUSTOM_PHASE");
});
