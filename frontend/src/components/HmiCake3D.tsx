import { useEffect, useRef } from "react";
import type { DrawingPayload } from "../api/drawing";
import { surfaceDrawing } from "../api/hmi";
import { cakeView } from "../drawing/cake3d.js";
import { createCakeTextures, paintCakeTexture } from "../drawing/cakeTextures";

/**
 * 선택 주문의 원본 그림(윗면·옆면)을 한 번에 입힌 3D 케이크.
 *
 * - 고객 화면과 같은 cakeView(Three.js)를 재사용하며, 드래그로 돌려 볼 수 있다.
 * - 주문에 없는 면은 빈 면으로 둔다. 다른 주문의 그림을 섞지 않는다.
 * - 원본 그림 표시용이다. 로봇이 실제로 그린 결과(실측)를 뜻하지 않는다.
 *
 * 자원 관리: HMI는 3초마다 주문 상세를 다시 받아 drawing 객체가 매번 새로 만들어진다.
 * 그때마다 WebGL 장면을 다시 만들지 않도록, 그림 내용이 같으면(signature 동일) 유지하고
 * 내용이 바뀌거나 화면에서 빠질 때만 dispose()로 GPU 자원을 해제한다.
 */
export function HmiCake3D({ drawing }: { drawing: DrawingPayload }) {
  const container = useRef<HTMLDivElement>(null);
  // 최신 그림을 effect 밖에서 참조한다. effect는 signature가 바뀔 때만 다시 실행된다.
  const latest = useRef(drawing);
  latest.current = drawing;

  // 면별 획 수·점 수로 그림 내용이 바뀌었는지 판단한다(같은 주문의 재조회는 동일).
  const signature = (["top", "side"] as const)
    .map((surface) => {
      const path = surfaceDrawing(drawing, surface);
      if (!path) return `${surface}:-`;
      const points = path.strokes.reduce((sum, s) => sum + s.points.length, 0);
      return `${surface}:${path.strokes.length}/${points}`;
    })
    .join("|");

  useEffect(() => {
    const element = container.current;
    if (!element) return;
    // 두 면 텍스처를 그린 뒤 3D 장면을 만든다. 없는 면은 배경색만 칠한다.
    const textures = createCakeTextures();
    for (const surface of ["top", "side"] as const) {
      const path = surfaceDrawing(latest.current, surface);
      paintCakeTexture(
        textures[surface],
        surface,
        path ? path.strokes.map((stroke) => stroke.points) : [],
      );
    }
    const view = cakeView(element, textures);
    // 윗면과 옆면이 함께 보이는 기본 시점(전체)으로 시작한다.
    view.view("full");
    return () => view.dispose();
  }, [signature]);

  return (
    <div
      ref={container}
      className="hmi-cake3d"
      role="img"
      aria-label="고객 원본 그림을 입힌 3D 케이크. 드래그해서 돌려 볼 수 있습니다."
    />
  );
}
