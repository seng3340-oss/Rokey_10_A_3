import test from "node:test";
import assert from "node:assert/strict";
import { registerHooks } from "node:module";

// Node에는 WebGL이 없으므로 미리보기만 대체하고 실제 입력·제출·API 코드를 실행한다.
const hooks = registerHooks({
  resolve(specifier, context, nextResolve) {
    if (context.parentURL?.endsWith("/drawing/controller.js")) {
      if (specifier === "./cake3d.js") {
        return {
          url: "data:text/javascript,export function cakeView() { return { update() {}, view() {}, resize() {}, dispose() {} }; }",
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
  style = { setProperty() {} };
  classList = { toggle() {} };
  getContext() {
    return new Proxy({}, { get: () => () => {} });
  }
  getBoundingClientRect() {
    return { left: 0, top: 0, width: 1024, height: 768 };
  }
  setPointerCapture() {}
  setAttribute() {}
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
function setup(t: test.TestContext) {
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
  Object.defineProperty(globalThis, "document", { value: document, configurable: true });
  const dispose = mountDrawing();
  t.after(() => {
    dispose();
    if (original) Object.defineProperty(globalThis, "document", original);
    else Reflect.deleteProperty(globalThis, "document");
  });
  const requests: { url: string; init: RequestInit; body: any }[] = [];
  t.mock.method(globalThis, "fetch", async (url: string, init: RequestInit) => {
    requests.push({ url, init, body: JSON.parse(init.body as string) });
    return Response.json({ order_id: "ORD-BOTH-001", status: "received" }, { status: 201 });
  });
  return {
    top,
    side,
    requests,
    element: document.querySelector,
    draw(x: number, y: number) {
      // 두 점으로 된 획을 실제 포인터 이벤트로 입력한다.
      for (const [type, offset] of [["pointerdown", 0], ["pointermove", 10], ["pointerup", 10]] as const) {
        const event = Object.assign(new Event(type), {
          pointerId: 1,
          clientX: x + offset,
          clientY: y,
        });
        document.querySelector("#draw").dispatchEvent(event);
      }
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
    strokes: [{ stroke_id: 0, points: [{ x: 500, y: 350 }, { x: 510, y: 350 }] }],
    additional_paths: [{
      canvas_width: 1024,
      canvas_height: 768,
      drawing_surface: "side",
      strokes: [{ stroke_id: 0, points: [{ x: 200, y: 380 }, { x: 210, y: 380 }] }],
    }],
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

test("윗면 바깥 5mm 띠에는 획을 시작하지 않는다", (t) => {
  const ui = setup(t);
  ui.draw(815, 384);
  ui.element("#review").click();
  assert.equal(ui.element("#modal").open, false);
  assert.equal(ui.requests.length, 0);
});
