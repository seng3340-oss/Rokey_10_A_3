import type { DrawingPayload } from "./drawing";

/** 같은 브라우저의 작성 중 미리보기다. 주문·실행 요청으로 사용하지 않는다. */
export const DRAFT_CHANNEL = "cake.local-draft.v1";
export interface DraftPreview {
  kind: "draft-preview";
  surface: "top" | "side";
  drawings: DrawingPayload[];
}

/** 브라우저 채널도 외부 입력으로 취급해 크기·유한 좌표·면을 확인한다. */
export function parseDraft(value: unknown): DraftPreview | null {
  const data = value as DraftPreview;
  if (
    !data ||
    data.kind !== "draft-preview" ||
    !["top", "side"].includes(data.surface) ||
    !Array.isArray(data.drawings) ||
    data.drawings.length !== 2
  )
    return null;
  let count = 0;
  for (const [i, drawing] of data.drawings.entries()) {
    if (
      !drawing ||
      drawing.canvas_width !== 1024 ||
      drawing.canvas_height !== 768 ||
      drawing.drawing_surface !== ["top", "side"][i] ||
      !Array.isArray(drawing.strokes) ||
      drawing.strokes.length > 2048
    )
      return null;
    for (const stroke of drawing.strokes) {
      if (!stroke || !Array.isArray(stroke.points)) return null;
      count += stroke.points.length;
      if (count > 20000) return null;
      for (const p of stroke.points) {
        if (
          !p ||
          !Number.isFinite(p.x) ||
          !Number.isFinite(p.y) ||
          p.x < 0 ||
          p.x > 1024 ||
          p.y < 0 ||
          p.y > 768
        )
          return null;
      }
    }
  }
  return data;
}
