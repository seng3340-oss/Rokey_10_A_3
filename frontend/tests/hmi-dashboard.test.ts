import assert from "node:assert/strict";
import test from "node:test";
import {
  dashboardOrder,
  dashboardSurface,
  dashboardTimestamp,
  dashboardToday,
} from "../src/api/hmiDashboard.ts";
import { parseDetail, parseOrder, parsePage } from "../src/api/hmi.ts";

/** 접수 시각·순서가 실제 공정 상태와 다를 때도 자동 선택이 올바른지 검사한다. */
const summary = (
  id: string,
  status: string,
  created = "2026-11-20T10:00:00+09:00",
) =>
  parseOrder({
    order_id: id,
    status,
    created_at: created,
    drawing_surface: "top",
    phase: "DRAWING",
    progress_pct: 60,
    message: "DrawingPath[2/2] SIDE",
  });

test("최신 접수나 완료 기록보다 진행 중인 주문을 우선하고 수동 선택은 보존한다", () => {
  const orders = [
    summary("new", "received"),
    summary("done", "completed"),
    summary("active", "running"),
  ];
  assert.equal(dashboardOrder(orders), "active");
  assert.equal(dashboardOrder(orders, "done"), "done");
  assert.equal(
    dashboardOrder([
      summary("recent", "completed"),
      summary("paused", "paused"),
    ]),
    "paused",
  );
  assert.equal(
    dashboardOrder([summary("new", "completed"), summary("old", "failed")]),
    "new",
  );
  assert.equal(dashboardOrder([]), "");
});

test("오늘 총계는 목록 크기와 분리하고 이전 서버의 잘린 목록은 하한으로 표시한다", () => {
  const page = { items: [summary("one", "running")], total: 200 };
  const now = new Date("2026-11-20T23:00:00+09:00");
  assert.equal(dashboardToday({ ...page, today_total: 140 }, now), "140");
  assert.equal(dashboardToday(page, now), "1+");
  assert.equal(
    dashboardToday(
      {
        items: [summary("old", "completed", "2026-11-19T10:00:00+09:00")],
        total: 1,
      },
      now,
    ),
    "0",
  );
  assert.throws(() => parsePage({ ...page, today_total: 201 }));
  assert.throws(() => parsePage({ ...page, today_total: -1 }));
});

test("주문 날짜는 한국 시간의 자정 경계를 따르며 잘못된 날짜를 추정하지 않는다", () => {
  assert.equal(
    dashboardTimestamp("2026-11-19T15:00:00Z"),
    "2026.11.20 00:00:00",
  );
  assert.equal(dashboardTimestamp("invalid"), "—");
});

test("양면 주문은 실제 경로 순번으로만 현재 면을 구분한다", () => {
  const drawing = {
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
  };
  const order = parseDetail({
    ...summary("active", "running"),
    drawing: {
      ...drawing,
      additional_paths: [{ ...drawing, drawing_surface: "side" }],
    },
  });
  assert.equal(dashboardSurface(order), "side");
  assert.equal(dashboardSurface({ ...order, message: "드로잉 준비" }), null);
  assert.equal(dashboardSurface(null), null);
});
