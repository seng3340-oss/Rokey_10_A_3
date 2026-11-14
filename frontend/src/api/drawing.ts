/** 한 요청이 담을 수 있는 케이크의 면이다. 두 면을 합쳐 전송하지 않는다. */
export type Surface = "top" | "side";
/** 화면 표시 크기가 아닌 원본 캔버스 기준 좌표다. x는 오른쪽, y는 아래로 증가한다. */
export interface Point2D {
  x: number;
  y: number;
}
/** 손을 떼기 전까지 이어진 한 획이다. 유효 획을 전송 순서대로 번호 매긴다. */
export interface Stroke {
  stroke_id: number;
  points: Point2D[];
}
/** 선택 면의 원본 크기와 모든 유효 입력 점을 담는 기존 드로잉 요청 형식이다. */
export interface DrawingPayload {
  canvas_width: number;
  canvas_height: number;
  drawing_surface: Surface;
  strokes: Stroke[];
}
// 응답 필드는 PM 원문에 없으므로 백엔드와 합의할 제안 계약이다.
/** 제작 완료가 아닌 서버의 주문 접수 확인만 나타낸다. HMI 상태와는 별개다. */
export interface DrawingReceipt {
  order_id: string;
  status: "received";
}
/**
 * 선택한 면을 제출하고 주문 식별자가 검증된 접수 응답만 반환한다.
 * 통신·응답 오류는 사용자 안내용 예외로 전달하며 입력 그림은 수정하지 않는다.
 */
export async function submitDrawing(
  payload: DrawingPayload,
): Promise<DrawingReceipt> {
  // UI 외부에서 호출되어도 빈 그림으로 서버에 주문을 만들지 않도록 한 번 더 검사한다.
  if (!payload.strokes.length)
    throw new Error(
      "선택한 면에 그림이 없습니다. 그림이 있는 면을 선택해주세요.",
    );
  // 15초 동안 응답이 없으면 대기를 종료한다. 서버 접수 여부는 별도 확인이 필요하다.
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 15000);
  try {
    // 개발 중에는 Vite 프록시, 배포 시에는 동일 출처의 /api가 이 요청을 전달한다.
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
    // HTTP 성공만으로 접수를 확정하지 않는다. JSON 구조와 주문 번호를 함께 검증한다.
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
    // 타임아웃·잘못된 JSON·네트워크 단절을 구별해 재요청 판단에 필요한 안내를 준다.
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
    // 성공·실패와 관계없이 요청 타이머를 정리해 불필요한 후속 abort를 막는다.
  } finally {
    clearTimeout(timeout);
  }
}
