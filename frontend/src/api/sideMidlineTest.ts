import testCake from "../../../config/side_midline_test.json" with { type: "json" };

/** 독립 테스트 치수와 길이 제한이다. 일반 드로잉·3D의 원통 치수에는 적용하지 않는다. */
export const SIDE_MIDLINE_TEST = Object.freeze(testCake);

/** 제작 완료가 아닌 테스트 주문의 대기열 접수만 보장하는 서버 계약이다. */
export interface SideMidlineReceipt {
  order_id: string;
  status: "received";
}

/** 응답이 유실된 동일 길이 요청을 재시도하기 위한 본문과 멱등 키다. */
type PendingRequest = { body: string; key: string };
let pendingRequest: PendingRequest | null = null;
const storageKey = "cake.side-midline-request.v1";

/** 새로고침 뒤에도 미확정 본문이 같으면 기존 키를 복원한다. 일반 주문 키는 읽지 않는다. */
function requestKey(body: string): string {
  try {
    const stored = globalThis.localStorage?.getItem(storageKey);
    if (stored) {
      const parsed = JSON.parse(stored);
      if (
        parsed &&
        typeof parsed.body === "string" &&
        typeof parsed.key === "string" &&
        parsed.key.trim()
      ) {
        pendingRequest = parsed;
      }
    }
  } catch {
    // 저장 차단·손상 시에도 같은 페이지의 메모리 키로 재시도할 수 있다.
  }
  if (pendingRequest?.body !== body) {
    pendingRequest = {
      body,
      key:
        globalThis.crypto?.randomUUID?.() ||
        `side-midline-${Date.now()}-${Math.random().toString(36).slice(2)}`,
    };
  }
  try {
    globalThis.localStorage?.setItem(
      storageKey,
      JSON.stringify(pendingRequest),
    );
  } catch {
    // 저장소 사용 여부와 무관하게 메모리에는 미확정 키를 남긴다.
  }
  return pendingRequest!.key;
}

/** 검증된 접수의 키만 해제해 다음 테스트를 새 요청으로 구분한다. */
function clearRequest(key: string): void {
  if (pendingRequest?.key !== key) return;
  pendingRequest = null;
  try {
    const stored = globalThis.localStorage?.getItem(storageKey);
    if (stored && JSON.parse(stored)?.key === key) {
      globalThis.localStorage?.removeItem(storageKey);
    }
  } catch {
    // 저장소 오류가 이미 검증된 서버 접수 결과를 바꾸지는 않는다.
  }
}

/** 서버의 문자열 detail 또는 FastAPI 검증 오류 배열에서 사용자 안내만 추출한다. */
async function serverMessage(response: Response): Promise<string> {
  const fallback =
    response.status === 409
      ? "현재 주문이 처리 중입니다. 완료 후 다시 시도해주세요."
      : "옆면 테스트 요청을 확인하지 못했습니다. 서버 상태를 확인해주세요.";
  try {
    const problem = await response.json();
    if (typeof problem?.detail === "string" && problem.detail.trim()) {
      return problem.detail;
    }
    if (Array.isArray(problem?.detail)) {
      const messages = problem.detail
        .filter(
          (item: unknown): item is { msg: string } =>
            !!item &&
            typeof item === "object" &&
            "msg" in item &&
            typeof item.msg === "string",
        )
        .map((item: { msg: string }) => item.msg);
      if (messages.length) return messages.join(" / ");
    }
  } catch {
    // JSON이 아닌 오류 응답도 상태에 맞는 안내로 전달한다.
  }
  return fallback;
}

/**
 * mm 단위 길이만 독립 테스트 경로로 제출한다. 일반 그림·주문 상태는 변경하지 않는다.
 * 입력 오류는 fetch 전에 차단하고, 미확정 실패는 동일 본문 재시도용 키를 보존한다.
 */
export async function submitSideMidlineTest(
  lineLengthMm: number,
): Promise<SideMidlineReceipt> {
  const { min_line_length_mm: min, max_line_length_mm: max } =
    SIDE_MIDLINE_TEST;
  if (
    !Number.isFinite(lineLengthMm) ||
    lineLengthMm < min ||
    lineLengthMm > max
  ) {
    throw new Error(`선 길이는 ${min}~${max}mm 범위의 숫자로 입력해주세요.`);
  }
  const body = JSON.stringify({ line_length_mm: lineLengthMm });
  const key = requestKey(body);
  const controller = new AbortController();
  // 서버의 접수 여부가 불명확해도 15초 뒤 UI 대기는 끝내고 키는 유지한다.
  const timeout = setTimeout(() => controller.abort(), 15000);
  try {
    const response = await fetch("/api/tests/side-midline", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "Idempotency-Key": key,
      },
      body,
      signal: controller.signal,
    });
    if (!response.ok) throw new Error(await serverMessage(response));
    if (response.status !== 201) {
      throw new Error("서버의 테스트 접수 응답을 확인할 수 없습니다.");
    }
    const result: unknown = await response.json();
    if (
      !result ||
      typeof result !== "object" ||
      !("order_id" in result) ||
      typeof result.order_id !== "string" ||
      !result.order_id.trim() ||
      !("status" in result) ||
      result.status !== "received"
    ) {
      throw new Error("서버의 테스트 접수 응답을 확인할 수 없습니다.");
    }
    clearRequest(key);
    return { order_id: result.order_id, status: "received" };
  } catch (error) {
    if (controller.signal.aborted) {
      throw new Error(
        "응답 시간이 초과됐습니다. 같은 길이로 재시도하거나 운영자에게 접수 여부를 확인해주세요.",
      );
    }
    if (error instanceof SyntaxError) {
      throw new Error("서버의 테스트 접수 응답을 읽을 수 없습니다.");
    }
    if (error instanceof TypeError) {
      throw new Error(
        "서버에 연결할 수 없습니다. 같은 길이로 재시도할 수 있습니다.",
      );
    }
    throw error;
  } finally {
    // 성공·실패 모두 타이머를 해제해 종료된 요청에 abort가 뒤늦게 발생하지 않게 한다.
    clearTimeout(timeout);
  }
}
