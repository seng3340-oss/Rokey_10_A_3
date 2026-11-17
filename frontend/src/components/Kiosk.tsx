import { useEffect } from "react";
import { KioskHeader } from "./KioskHeader";
import { DrawingWorkspace } from "./DrawingWorkspace";
import { ReviewDialog } from "./ReviewDialog";
import { mountDrawing } from "../drawing/controller.js";
/** 고객 화면의 세 영역을 조립하고 DOM 준비 후 필기 컨트롤러를 연결한다. */
export function Kiosk() {
  // 컨트롤러가 반환한 정리 함수를 React에 전달해 화면 해제 시 타이머와 3D를 해제한다.
  useEffect(() => mountDrawing(), []);
  return (
    <>
      <KioskHeader />
      <DrawingWorkspace />
      <ReviewDialog />
    </>
  );
}
