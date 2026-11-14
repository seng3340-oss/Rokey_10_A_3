export type Surface = "top" | "side";
export interface Point2D {
  x: number;
  y: number;
}
export interface Stroke {
  stroke_id: number;
  points: Point2D[];
}
export interface DrawingPayload {
  canvas_width: number;
  canvas_height: number;
  drawing_surface: Surface;
  strokes: Stroke[];
}
// 응답 필드는 PM 원문에 없으므로 백엔드와 합의할 제안 계약이다.
export interface DrawingReceipt {
  order_id: string;
  status: "received";
}
export async function submitDrawing(
  payload: DrawingPayload,
): Promise<DrawingReceipt> {
  if (!payload.strokes.length)
    throw new Error(
      "선택한 면에 그림이 없습니다. 그림이 있는 면을 선택해주세요.",
    );
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 15000);
  try {
    const response = await fetch("/api/drawing", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
      signal: controller.signal,
    });
    if (!response.ok)
      throw new Error(
        "제작 요청을 확인하지 못했습니다. 서버 연결과 운영 상태를 확인해주세요.",
      );
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
      throw new Error(
        "서버의 주문 접수 응답을 확인할 수 없습니다. 운영자에게 확인해주세요.",
      );
    }
    return { order_id: result.order_id, status: "received" };
  } catch (error) {
    if (controller.signal.aborted)
      throw new Error(
        "응답 시간이 초과됐습니다. 중복 요청 전 운영자에게 접수 여부를 확인해주세요.",
      );
    if (error instanceof SyntaxError)
      throw new Error(
        "서버의 주문 접수 응답을 읽을 수 없습니다. 운영자에게 확인해주세요.",
      );
    if (error instanceof TypeError)
      throw new Error("서버에 연결할 수 없습니다. 그림은 유지됩니다.");
    throw error;
  } finally {
    clearTimeout(timeout);
  }
}
