import { useEffect, useRef } from "react";
import type { DrawingPayload } from "../api/drawing";
import { surfaceDrawing } from "../api/hmi";
import { cakeView } from "../drawing/cake3d.js";
import { createCakeTextures, paintCakeTexture } from "../drawing/cakeTextures";
import { DRAWING_LAYOUT } from "../drawing/cakeDimensions";
import type { Point2D, Surface } from "../api/drawing";

/**
 * HMI 원본 카드는 위쪽 대각선 시점에서 천천히 회전한다. 한 번 만든 장면을 유지하고
 * 1초 조회의 그림 내용이 달라질 때만 두 면의 생크림 입체를 갱신한다. 주문 간 획 수가
 * 같아도 좌표가 다르면 갱신하며, 화면 해제 시 GPU·프레임·관찰자를 모두 정리한다.
 */
export function HmiCake3D({ drawing }: { drawing: DrawingPayload }) {
  const container = useRef<HTMLDivElement>(null);
  const resources = useRef<{
    textures: ReturnType<typeof createCakeTextures>;
    view: ReturnType<typeof cakeView>;
  } | null>(null);
  const signature = JSON.stringify(drawing);
  const latest = useRef(drawing);
  latest.current = drawing;

  useEffect(() => {
    const element = container.current;
    if (!element) return;
    const textures = createCakeTextures();
    const view = cakeView(element, textures, { autoRotate: true, cream: true });
    resources.current = { textures, view };
    view.view("full");
    return () => {
      resources.current = null;
      view.dispose();
    };
  }, []);

  useEffect(() => {
    const resource = resources.current;
    if (!resource) return;
    const drawings: Record<Surface, Point2D[][]> = { top: [], side: [] };
    for (const surface of ["top", "side"] as const) {
      const path = surfaceDrawing(latest.current, surface);
      // 저장된 캔버스 크기가 달라도 고객 화면과 동일한 표시 좌표계에 사본만 맞춘다.
      drawings[surface] = path
        ? path.strokes.map((stroke) =>
            stroke.points.map((point) => ({
              x: (point.x * DRAWING_LAYOUT.width) / path.canvas_width,
              y: (point.y * DRAWING_LAYOUT.height) / path.canvas_height,
            })),
          )
        : [];
      // 주문에 없는 면도 배경색으로 초기화해 이전 주문의 그림을 남기지 않는다.
      paintCakeTexture(resource.textures[surface], surface, drawings[surface]);
    }
    resource.view.update(drawings);
  }, [signature]);

  return (
    <div
      ref={container}
      className="hmi-cake3d cake3d"
      role="img"
      aria-label="천천히 회전하는 고객 원본 3D 케이크"
    />
  );
}
