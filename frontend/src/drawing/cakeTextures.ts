import { CAKE_DIMENSIONS, DRAWING_LAYOUT } from "./cakeDimensions";
import type { Point2D, Surface } from "../api/drawing";

/**
 * 저장된 주문 그림을 3D 케이크 텍스처로 옮기는 도우미.
 *
 * 고객 화면(controller.js의 updateTextures)과 같은 규칙을 따른다.
 *  - 입력 좌표: 전송 계약의 1024×768 캔버스 픽셀
 *  - 윗면은 512×512 정사각형, 옆면은 1024×256 띠 텍스처에 DRAWING_LAYOUT 영역만 잘라 담는다.
 *  - 선 굵기는 두 면이 같은 물리적 굵기(약 5px@옆면 기준)로 보이게 면별로 보정한다.
 * HMI는 그림을 읽기만 하며 좌표를 변경하지 않는다.
 */

/** cakeView()가 요구하는 두 면 캔버스. */
export interface CakeTextures {
  top: HTMLCanvasElement;
  side: HTMLCanvasElement;
}

/** 고객 화면과 같은 해상도의 빈 텍스처 캔버스 두 장을 만든다. */
export function createCakeTextures(): CakeTextures {
  const top = document.createElement("canvas");
  const side = document.createElement("canvas");
  top.width = top.height = 512;
  side.width = 1024;
  side.height = 256;
  return { top, side };
}

/**
 * 한 면의 텍스처를 배경색으로 지우고 획을 그린다. strokes가 비어 있으면 빈 면이 된다.
 * 호출 후 cakeView().update()를 불러야 GPU 텍스처가 갱신된다.
 */
export function paintCakeTexture(
  canvas: HTMLCanvasElement,
  surface: Surface,
  strokes: Point2D[][],
): void {
  const context = canvas.getContext("2d");
  if (!context) return;
  // 이전 변환이 누적되지 않도록 원래 좌표계에서 시작한다.
  context.setTransform(1, 0, 0, 1, 0, 0);
  context.fillStyle = "#fffaf1";
  context.fillRect(0, 0, canvas.width, canvas.height);

  // 캔버스 픽셀 좌표 → 텍스처 좌표: 면의 편집 영역만 잘라 텍스처 전체에 맞춘다.
  const region = DRAWING_LAYOUT[surface];
  context.scale(canvas.width / region.width, canvas.height / region.height);
  context.translate(-region.x, -region.y);

  // 옆면 1mm당 픽셀을 기준으로 윗면 선 굵기를 환산해 두 면의 펜 굵기를 맞춘다.
  const circumference =
    (Math.PI *
      (CAKE_DIMENSIONS.topDiameterMm + CAKE_DIMENSIONS.bottomDiameterMm)) /
    2;
  const sidePixelsPerMm = DRAWING_LAYOUT.side.width / circumference;
  context.lineWidth =
    surface === "top"
      ? (5 * (DRAWING_LAYOUT.top.width / CAKE_DIMENSIONS.topDiameterMm)) /
        sidePixelsPerMm
      : 5;
  context.strokeStyle = "#795041";
  context.lineCap = "round";
  context.lineJoin = "round";
  for (const stroke of strokes) {
    context.beginPath();
    stroke.forEach((point, index) => {
      if (index) context.lineTo(point.x, point.y);
      else context.moveTo(point.x, point.y);
    });
    context.stroke();
  }
  context.setTransform(1, 0, 0, 1, 0, 0);
}
