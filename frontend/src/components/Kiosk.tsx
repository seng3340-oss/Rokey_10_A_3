import { useEffect } from "react";
import { KioskHeader } from "./KioskHeader";
import { DrawingWorkspace } from "./DrawingWorkspace";
import { ReviewDialog } from "./ReviewDialog";
import { mountDrawing } from "../drawing/controller.js";
export function Kiosk() {
  useEffect(() => mountDrawing(), []);
  return (
    <>
      <KioskHeader />
      <DrawingWorkspace />
      <ReviewDialog />
    </>
  );
}
