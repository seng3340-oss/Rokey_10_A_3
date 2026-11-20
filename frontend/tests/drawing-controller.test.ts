import test from "node:test";
import assert from "node:assert/strict";
import { registerHooks } from "node:module";

// Node에는 WebGL이 없으므로 미리보기만 대체하고 실제 입력·제출·API 코드를 실행한다.
const hooks = registerHooks({
  resolve(specifier, context, nextResolve) {
    if (context.parentURL?.endsWith("/drawing/controller.js")) {
      if (specifier === "./cake3d.js") {
        return {
          url:
            "data:text/javascript," +
            encodeURIComponent(`
              export function cakeView(container, textures, options) {
                container.modelTextures = textures;
                container.modelOptions = options;
                container.modelEvents.push("create");
                return {
                  update(drawings) {
                    container.modelEvents.push("update");
                    container.modelDrawings = structuredClone(drawings);
                  },
                  view(kind) { container.modelEvents.push("view:" + kind); },
                  resize() { container.modelEvents.push("resize"); },
                  dispose() { container.modelEvents.push("dispose"); },
                };
              }
            `),
          shortCircuit: true,
        };
      }
      if (specifier === "../api/drawing" || specifier === "./cakeDimensions") {
        return nextResolve(`${specifier}.ts`, context);
      }
    }
    return nextResolve(specifier, context);
  },
});
const { mountDrawing } = await import("../src/drawing/controller.js");
hooks.deregister();

/** 캔버스 표시만 생략하고 컨트롤러가 사용하는 DOM 이벤트·상태를 보존한다. */
class Element extends EventTarget {
  onclick: (() => unknown) | null = null;
  dataset: Record<string, string> = {};
  open = false;
  disabled = false;
  hidden = false;
  textContent = "";
  // 생성뿐 아니라 원본 두 면의 선이 3D 텍스처에 실제 복원됐는지도 관찰한다.
  modelEvents: string[] = [];
  modelTextures: Record<string, Element> | null = null;
  modelOptions: { cream?: boolean } = {};
  modelDrawings: Record<string, { x: number; y: number }[][]> = {};
  drawnPoints: { operation: string; x: number; y: number }[] = [];
  style = { setProperty() {} };
  // 실제 주문 화면의 진행·실패 전환도 관찰할 수 있도록 classList 상태를 보관한다.
  classes = new Set<string>();
  classList = {
    add: (name: string) => this.classes.add(name),
    remove: (name: string) => this.classes.delete(name),
    contains: (name: string) => this.classes.has(name),
    toggle: (name: string, enabled: boolean) => {
      if (enabled) this.classes.add(name);
      else this.classes.delete(name);
    },
  };
  attributes = new Map<string, string>();
  getContext() {
    return new Proxy(
      {},
      {
        get:
          (_target, operation) =>
          (...args: number[]) => {
            if (operation === "moveTo" || operation === "lineTo") {
              this.drawnPoints.push({ operation, x: args[0], y: args[1] });
            }
          },
      },
    );
  }
  getBoundingClientRect() {
    return { left: 0, top: 0, width: 1024, height: 768 };
  }
  setPointerCapture() {}
  setAttribute(name: string, value: string) {
    this.attributes.set(name, value);
  }
  removeAttribute(name: string) {
    this.attributes.delete(name);
  }
  showModal() {
    this.open = true;
  }
  close() {
    this.open = false;
    this.dispatchEvent(new Event("close"));
  }
  focus() {}
  click() {
    const result = this.onclick?.();
    this.dispatchEvent(new Event("click"));
    return result;
  }
}

/** 화면 버튼과 포인터 입력을 통해 실제 확인 창까지 진행할 테스트 환경을 만든다. */
function setup(
  t: test.TestContext,
  statusResponses?: Record<string, unknown>[],
  existingOrderId?: string,
) {
  const elements = new Map<string, Element>();
  const top = new Element();
  top.dataset.surface = "top";
  const side = new Element();
  side.dataset.surface = "side";
  const document = Object.assign(new EventTarget(), {
    querySelector(selector: string) {
      if (!elements.has(selector)) elements.set(selector, new Element());
      return elements.get(selector)!;
    },
    querySelectorAll(selector: string) {
      return selector === "[data-surface]" ? [top, side] : [];
    },
    createElement() {
      return new Element();
    },
  });
  const original = Object.getOwnPropertyDescriptor(globalThis, "document");
  // 상태 화면 검사에만 HTMLElement 대역을 켜 기존 입력 검사와 타이머 범위를 분리한다.
  const originalElement = Object.getOwnPropertyDescriptor(
    globalThis,
    "HTMLElement",
  );
  if (statusResponses) {
    Object.defineProperty(globalThis, "HTMLElement", {
      value: Element,
      configurable: true,
    });
  }
  Object.defineProperty(globalThis, "document", {
    value: document,
    configurable: true,
  });
  let dispose = () => {};
  t.after(() => {
    dispose();
    if (original) Object.defineProperty(globalThis, "document", original);
    else Reflect.deleteProperty(globalThis, "document");
    if (originalElement)
      Object.defineProperty(globalThis, "HTMLElement", originalElement);
    else Reflect.deleteProperty(globalThis, "HTMLElement");
  });
  const requests: { url: string; init: RequestInit; body: any }[] = [];
  t.mock.method(globalThis, "fetch", async (url: string, init: RequestInit) => {
    if (init.method !== "POST") {
      requests.push({ url, init, body: null });
      const status =
        statusResponses?.length === 1
          ? statusResponses[0]
          : statusResponses?.shift();
      return Response.json({
        ...status,
        drawing: status?.drawing || {
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
    }
    requests.push({ url, init, body: JSON.parse(init.body as string) });
    return Response.json(
      { order_id: "ORD-BOTH-001", status: "received" },
      { status: 201 },
    );
  });
  dispose = mountDrawing({ existingOrderId });
  return {
    top,
    side,
    requests,
    element: document.querySelector,
    draw(x: number, y: number) {
      // 두 점으로 된 획을 실제 포인터 이벤트로 입력한다.
      for (const [type, offset] of [
        ["pointerdown", 0],
        ["pointermove", 10],
        ["pointerup", 10],
      ] as const) {
        const event = Object.assign(new Event(type), {
          pointerId: 1,
          clientX: x + offset,
          clientY: y,
        });
        document.querySelector("#draw").dispatchEvent(event);
      }
    },
    /** 임의의 점 목록을 down → move… → up 순서의 한 획으로 입력한다. */
    stroke(points: [number, number][]) {
      const types = [
        "pointerdown",
        ...points.slice(1).map(() => "pointermove"),
        "pointerup",
      ];
      types.forEach((type, index) => {
        const [x, y] = points[Math.min(index, points.length - 1)];
        const event = Object.assign(new Event(type), {
          pointerId: 1,
          clientX: x,
          clientY: y,
        });
        document.querySelector("#draw").dispatchEvent(event);
      });
    },
  };
}

test("두 면을 그리면 선택 면과 무관하게 TOP + SIDE를 한 번만 POST한다", async (t) => {
  const ui = setup(t);
  ui.draw(500, 350);
  ui.side.click();
  ui.draw(200, 380);
  ui.element("#review").click();
  await ui.element("#confirm").click();
  assert.equal(ui.requests.length, 1);
  assert.equal(ui.requests[0].url, "/api/drawing");
  assert.equal(ui.requests[0].init.method, "POST");
  assert.deepEqual(ui.requests[0].body, {
    canvas_width: 1024,
    canvas_height: 768,
    drawing_surface: "top",
    strokes: [
      {
        stroke_id: 0,
        points: [
          { x: 500, y: 350 },
          { x: 510, y: 350 },
        ],
      },
    ],
    additional_paths: [
      {
        canvas_width: 1024,
        canvas_height: 768,
        drawing_surface: "side",
        strokes: [
          {
            stroke_id: 0,
            points: [
              { x: 200, y: 380 },
              { x: 210, y: 380 },
            ],
          },
        ],
      },
    ],
  });
  assert.match(ui.element("#result").textContent, /ORD-BOTH-001.*접수 완료/);
});

for (const surface of ["top", "side"] as const) {
  test(`${surface} 한 면만 있으면 빈 반대 면을 선택해도 기존 본문으로 제출한다`, async (t) => {
    const ui = setup(t);
    ui[surface].click();
    ui.draw(500, 380);
    ui[surface === "top" ? "side" : "top"].click();
    ui.element("#review").click();
    await ui.element("#confirm").click();
    assert.equal(ui.requests.length, 1);
    assert.equal(ui.requests[0].body.drawing_surface, surface);
    assert.equal(ui.requests[0].body.strokes.length, 1);
    assert.equal("additional_paths" in ui.requests[0].body, false);
  });
}

test("확인한 양면 사본은 확인 창 이후 편집해도 바뀌지 않는다", async (t) => {
  const ui = setup(t);
  ui.draw(500, 350);
  ui.side.click();
  ui.draw(200, 380);
  ui.element("#review").click();
  ui.element("#clear").click();
  await ui.element("#confirm").click();
  assert.equal(ui.requests[0].body.additional_paths[0].strokes.length, 1);
});

test("옆면 하단 금지 영역에는 획을 시작하지 않는다", async (t) => {
  const ui = setup(t);
  ui.side.click();
  // 현재 케이크 편집 띠의 아래 경계 바로 위는 하단 10mm 금지 구역이다.
  ui.draw(300, 475);
  ui.element("#review").click();
  assert.equal(ui.element("#modal").open, false);
  assert.equal(ui.requests.length, 0);
});

/** 옆면만 그린 뒤 제출해 SIDE 획의 점 좌표를 돌려받는다. */
async function submitSide(ui: ReturnType<typeof setup>) {
  ui.element("#review").click();
  await ui.element("#confirm").click();
  assert.equal(ui.requests[0].body.drawing_surface, "side");
  return ui.requests[0].body.strokes;
}

// 옆면 띠는 x=64~960(896px)이다. 좌우 끝은 원통 이음매이므로 그 경계에 정확히 닿아야 한다.
test("옆면 좌우 경계 근처에서 시작·종료한 획은 이음매에 맞춘다", async (t) => {
  const ui = setup(t);
  ui.side.click();
  ui.stroke([
    [70, 380],
    [500, 381],
    [952, 382],
  ]);
  const [stroke] = await submitSide(ui);
  assert.deepEqual(stroke.points[0], { x: 64, y: 380 });
  assert.deepEqual(stroke.points.at(-1), { x: 960, y: 382 });
});

test("옆면 좌우로 빠르게 빠져나간 획은 경계 교점까지 남긴다", async (t) => {
  const ui = setup(t);
  ui.side.click();
  // 직전 점 900에서 바로 1000으로 나간다. 경계 x=960의 교점 높이는 보간한 386이다.
  ui.stroke([
    [500, 380],
    [900, 380],
    [1000, 390],
  ]);
  const [stroke] = await submitSide(ui);
  assert.deepEqual(stroke.points.at(-1), { x: 960, y: 386 });
});

test("옆면 경계에서 떨어진 끝점과 위아래 이탈은 이음매로 옮기지 않는다", async (t) => {
  const ui = setup(t);
  ui.side.click();
  ui.stroke([
    [200, 380],
    [300, 380],
  ]);
  // 위쪽 경계로 나가면 교점을 추가하지 않고 기존처럼 획만 끝낸다.
  ui.stroke([
    [400, 380],
    [400, 330],
    [400, 200],
  ]);
  const strokes = await submitSide(ui);
  assert.deepEqual(strokes[0].points, [
    { x: 200, y: 380 },
    { x: 300, y: 380 },
  ]);
  assert.deepEqual(strokes[1].points, [
    { x: 400, y: 380 },
    { x: 400, y: 330 },
  ]);
});

test("윗면 바깥 5mm 띠에는 획을 시작하지 않는다", (t) => {
  const ui = setup(t);
  ui.draw(815, 384);
  ui.element("#review").click();
  assert.equal(ui.element("#modal").open, false);
  assert.equal(ui.requests.length, 0);
});

test("최종 케이크 실패는 관리자 문의 화면을 유지하고 재주문·픽업을 하지 않는다", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval"] });
  const ui = setup(t, [
    {
      order_id: "ORD-BOTH-001",
      status: "failed",
      phase: "FAILED",
      progress_pct: 0,
      error_code: "CAKE_WEIGHT_FAILED",
      message: "입고함 집기 재시도 1/1 후에도 케이크 없음",
    },
  ]);
  ui.draw(500, 350);
  ui.element("#review").click();
  await ui.element("#confirm").click();
  await new Promise(setImmediate);
  assert.equal(
    ui.element("#order-title").textContent,
    "제작 과정에서 문제가 발생했습니다",
  );
  assert.equal(
    ui.element("#order-message").textContent,
    "관리자에게 문의해주세요.",
  );
  assert.ok(ui.element("#modal").classList.contains("is-order-failed"));
  assert.equal(ui.element("#order-status").attributes.get("role"), "alert");
  assert.equal(ui.element("#order-progress").hidden, true);
  assert.equal(ui.element("#order-percent").hidden, true);
  assert.equal(ui.element("#pickup").hidden, true);
  assert.match(ui.element("#order-number").textContent, /ORD-BOTH-001/);
  t.mock.timers.tick(65000);
  await new Promise(setImmediate);
  assert.equal(ui.element("#modal").open, true);
  assert.equal(ui.requests.length, 2); // 접수 POST 1회·최종 상태 GET 1회 뒤 중단한다.
});

test("재집기 중 running은 실패 화면 없이 진행하고 성공 결과에는 픽업을 표시한다", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval"] });
  const ui = setup(t, [
    {
      order_id: "ORD-BOTH-001",
      status: "running",
      phase: "PREPARING",
      progress_pct: 0,
    },
    {
      order_id: "ORD-BOTH-001",
      status: "completed",
      phase: "COMPLETED",
      progress_pct: 100,
    },
  ]);
  ui.draw(500, 350);
  ui.element("#review").click();
  await ui.element("#confirm").click();
  await new Promise(setImmediate);
  assert.equal(
    ui.element("#modal").classList.contains("is-order-failed"),
    false,
  );
  assert.equal(
    ui.element("#order-message").textContent,
    "케이크를 작업대에 준비하고 있어요",
  );
  assert.equal(ui.element("#order-progress").hidden, false);
  t.mock.timers.tick(3000);
  await new Promise(setImmediate);
  assert.equal(ui.element("#order-title").textContent, "케이크가 완성됐어요");
  assert.equal(ui.element("#pickup").hidden, false);
  assert.equal(
    ui.element("#modal").classList.contains("is-order-failed"),
    false,
  );
});

test("조회 장애와 다른 주문의 실패 응답은 현재 주문의 최종 실패로 오인하지 않는다", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval"] });
  const ui = setup(t, [
    {
      order_id: "OTHER-ORDER",
      status: "failed",
      progress_pct: 0,
    },
  ]);
  ui.draw(500, 350);
  ui.element("#review").click();
  await ui.element("#confirm").click();
  await new Promise(setImmediate);
  assert.equal(
    ui.element("#modal").classList.contains("is-order-failed"),
    false,
  );
  assert.match(ui.element("#order-message").textContent, /진행 상태를 확인/);
  t.mock.method(
    globalThis,
    "fetch",
    async () => new Response("", { status: 503 }),
  );
  t.mock.timers.tick(3000);
  await new Promise(setImmediate);
  assert.equal(
    ui.element("#modal").classList.contains("is-order-failed"),
    false,
  );
  assert.match(ui.element("#order-message").textContent, /진행 상태를 확인/);
});

test("재개한 기존 주문은 원본·상태 GET만 연결하고 새 주문을 POST하지 않는다", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval"] });
  const ui = setup(
    t,
    [
      {
        order_id: "ORD-BOTH-001",
        status: "running",
        phase: "FINISHING",
        progress_pct: 90,
      },
    ],
    "ORD-BOTH-001",
  );
  await new Promise(setImmediate);
  assert.equal(ui.element("#modal").open, true);
  assert.equal(
    ui.element("#order-message").textContent,
    "완성된 케이크를 옮기고 있어요",
  );
  assert.ok(ui.requests.length >= 2);
  assert.ok(ui.requests.every((request) => request.init.method !== "POST"));
  assert.ok(
    ui.requests.every((request) => request.url.includes("ORD-BOTH-001")),
  );
  assert.ok(ui.element("#review3d").modelEvents.includes("create"));
  assert.ok(ui.element("#review3d").modelEvents.includes("resize"));
});

test("경고 후 재개하면 3D를 생성하고 저장된 양면 그림을 표시 좌표로 복원한다", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval"] });
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
    additional_paths: [
      {
        canvas_width: 512,
        canvas_height: 384,
        drawing_surface: "side",
        strokes: [
          {
            stroke_id: 0,
            points: [
              { x: 100, y: 190 },
              { x: 110, y: 195 },
            ],
          },
        ],
      },
    ],
  };
  const before = JSON.stringify(drawing);
  const ui = setup(
    t,
    [
      {
        order_id: "ORD-BOTH-001",
        status: "running",
        phase: "DRAWING",
        progress_pct: 60,
        drawing,
      },
    ],
    "ORD-BOTH-001",
  );
  await new Promise(setImmediate);
  const preview = ui.element("#review3d");
  assert.equal(
    preview.modelEvents.filter((event) => event === "create").length,
    1,
  );
  assert.ok(preview.modelEvents.includes("view:angle"));
  // 경고에서 돌아온 확인 창과 실시간 뷰 모두 같은 양면 원본으로 크림을 복원해야 한다.
  assert.equal(preview.modelOptions.cream, true);
  assert.equal(ui.element("#live3d").modelOptions.cream, true);
  assert.deepEqual(preview.modelDrawings, {
    top: [
      [
        { x: 500, y: 350 },
        { x: 520, y: 360 },
      ],
    ],
    side: [
      [
        { x: 200, y: 380 },
        { x: 220, y: 390 },
      ],
    ],
  });
  assert.deepEqual(ui.element("#live3d").modelDrawings, preview.modelDrawings);
  assert.deepEqual(preview.modelTextures!.top.drawnPoints.slice(-2), [
    { operation: "moveTo", x: 500, y: 350 },
    { operation: "lineTo", x: 520, y: 360 },
  ]);
  assert.deepEqual(preview.modelTextures!.side.drawnPoints.slice(-2), [
    { operation: "moveTo", x: 200, y: 380 },
    { operation: "lineTo", x: 220, y: 390 },
  ]);
  assert.equal(JSON.stringify(drawing), before);
  assert.ok(ui.requests.every((request) => request.init.method !== "POST"));
});

test("기존 주문 조회가 잠시 실패해도 같은 주문 GET을 재시도해 픽업까지 연결한다", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval"] });
  const ui = setup(t, [{ order_id: "WRONG-ORDER" }], "ORD-BOTH-001");
  await new Promise(setImmediate);
  assert.match(ui.element("#order-message").textContent, /연결되면 자동/);
  t.mock.method(
    globalThis,
    "fetch",
    async (_url: string, init: RequestInit) => {
      assert.notEqual(init.method, "POST");
      return Response.json({
        order_id: "ORD-BOTH-001",
        status: "completed",
        progress_pct: 100,
        drawing: { drawing_surface: "top", strokes: [] },
      });
    },
  );
  t.mock.timers.tick(2000);
  await new Promise(setImmediate);
  assert.equal(ui.element("#pickup").hidden, false);
});
