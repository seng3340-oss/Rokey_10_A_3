import test from "node:test";
import assert from "node:assert/strict";
import { submitSideMidlineTest } from "../src/api/sideMidlineTest.ts";

const storageKey = "cake.side-midline-request.v1";

/** 실제 저장소 대신 격리된 메모리를 사용해 재시작 시 남는 요청을 관찰한다. */
function mockStorage(t: test.TestContext) {
  const values = new Map<string, string>();
  const original = Object.getOwnPropertyDescriptor(globalThis, "localStorage");
  Object.defineProperty(globalThis, "localStorage", {
    configurable: true,
    value: {
      getItem: (key: string) => values.get(key) ?? null,
      setItem: (key: string, value: string) => values.set(key, value),
      removeItem: (key: string) => values.delete(key),
    },
  });
  t.after(() => {
    if (original) Object.defineProperty(globalThis, "localStorage", original);
    else Reflect.deleteProperty(globalThis, "localStorage");
  });
  return values;
}

// 일반 주문 경로·본문·저장 키와 섞이면 이 계약 검사가 실패한다.
test("독립 JSON 경로와 헤더로 전송하고 검증된 접수 뒤 저장 키를 지운다", async (t) => {
  const storage = mockStorage(t);
  storage.set("cake.pending-request.v1", "normal drawing untouched");
  t.mock.method(globalThis, "fetch", async (url, init) => {
    assert.equal(url, "/api/tests/side-midline");
    assert.equal(init.method, "POST");
    assert.equal(init.headers["Content-Type"], "application/json");
    assert.ok(init.headers["Idempotency-Key"]);
    assert.deepEqual(JSON.parse(init.body), { line_length_mm: 30 });
    return Response.json(
      { order_id: "SIDE-1", status: "received" },
      { status: 201 },
    );
  });
  assert.deepEqual(await submitSideMidlineTest(30), {
    order_id: "SIDE-1",
    status: "received",
  });
  assert.equal(storage.has(storageKey), false);
  assert.equal(
    storage.get("cake.pending-request.v1"),
    "normal drawing untouched",
  );
});

// 유효하지 않은 값은 키 생성과 네트워크보다 먼저 거절해야 한다.
test("유한한 5~50mm 이외 입력은 fetch 전에 거절한다", async (t) => {
  mockStorage(t);
  const fetch = t.mock.method(globalThis, "fetch", async () => {
    throw new Error("요청하면 안 됨");
  });
  for (const length of [NaN, Infinity, -Infinity, 0, 4.9, 50.1]) {
    await assert.rejects(submitSideMidlineTest(length), /5.*50/);
  }
  assert.equal(fetch.mock.callCount(), 0);
});

test("양 끝 길이 5mm와 50mm는 허용한다", async (t) => {
  mockStorage(t);
  t.mock.method(globalThis, "fetch", async () =>
    Response.json(
      { order_id: "BOUNDARY", status: "received" },
      { status: 201 },
    ),
  );
  await submitSideMidlineTest(5);
  await submitSideMidlineTest(50);
});

// 잘못된 접수는 미확정 요청을 보존하고 제작 완료로 처리하지 않는다.
test("잘못된 접수 본문과 201 이외 성공 상태를 거절한다", async (t) => {
  const storage = mockStorage(t);
  let response: Response;
  t.mock.method(globalThis, "fetch", async () => response);
  for (const body of [
    null,
    {},
    { order_id: " ", status: "received" },
    { order_id: 1, status: "received" },
    { order_id: "SIDE", status: "completed" },
  ]) {
    response = Response.json(body, { status: 201 });
    await assert.rejects(submitSideMidlineTest(30), /접수 응답/);
    assert.ok(storage.has(storageKey));
  }
  response = Response.json({ order_id: "SIDE", status: "received" });
  await assert.rejects(submitSideMidlineTest(30), /접수 응답/);
  response = new Response("not json", { status: 201 });
  await assert.rejects(submitSideMidlineTest(30), /접수 응답/);
});

test("409와 400·422의 서버 오류 설명을 전달한다", async (t) => {
  mockStorage(t);
  let response: Response;
  t.mock.method(globalThis, "fetch", async () => response);
  for (const status of [409, 400, 422]) {
    response = Response.json({ detail: `서버 안내 ${status}` }, { status });
    await assert.rejects(
      submitSideMidlineTest(30),
      new RegExp(`서버 안내 ${status}`),
    );
  }
  response = Response.json(
    { detail: [{ msg: "길이 범위 오류" }] },
    { status: 422 },
  );
  await assert.rejects(submitSideMidlineTest(30), /길이 범위 오류/);
});

test("통신 실패와 모듈 재시작 후 같은 본문은 같은 키, 접수 뒤에는 새 키를 쓴다", async (t) => {
  const storage = mockStorage(t);
  const keys: string[] = [];
  let fail = true;
  t.mock.method(globalThis, "fetch", async (_url, init) => {
    keys.push(init.headers["Idempotency-Key"]);
    assert.equal(init.body, '{"line_length_mm":30}');
    if (fail) throw new TypeError("연결 끊김");
    return Response.json(
      { order_id: "RETRY", status: "received" },
      { status: 201 },
    );
  });
  await assert.rejects(submitSideMidlineTest(30), /연결/);
  await assert.rejects(submitSideMidlineTest(30), /연결/);
  assert.ok(storage.has(storageKey));
  // 새 모듈의 메모리는 비어 있으므로 저장소에서 키를 복원해야 통과한다.
  const reloaded = await import(
    `../src/api/sideMidlineTest.ts?reload=${Date.now()}`
  );
  fail = false;
  await reloaded.submitSideMidlineTest(30);
  await reloaded.submitSideMidlineTest(30);
  assert.equal(keys[0], keys[1]);
  assert.equal(keys[0], keys[2]);
  assert.notEqual(keys[2], keys[3]);
});

test("길이를 바꾸면 별도 요청 키를 사용한다", async (t) => {
  mockStorage(t);
  const keys: string[] = [];
  t.mock.method(globalThis, "fetch", async (_url, init) => {
    keys.push(init.headers["Idempotency-Key"]);
    throw new TypeError("연결 끊김");
  });
  await assert.rejects(submitSideMidlineTest(25));
  await assert.rejects(submitSideMidlineTest(35));
  assert.notEqual(keys[0], keys[1]);
});

test("15초 타임아웃은 요청을 중단하고 재시도 키를 보존한다", async (t) => {
  const storage = mockStorage(t);
  t.mock.timers.enable({ apis: ["setTimeout"] });
  t.mock.method(
    globalThis,
    "fetch",
    async (_url, init) =>
      new Promise((_resolve, reject) => {
        init.signal.addEventListener("abort", () =>
          reject(new DOMException("중단", "AbortError")),
        );
      }),
  );
  const pending = assert.rejects(submitSideMidlineTest(30), /시간.*초과/);
  t.mock.timers.tick(15000);
  await pending;
  assert.ok(storage.has(storageKey));
});
