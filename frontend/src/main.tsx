import { lazy, Suspense } from "react";
import { createRoot } from "react-dom/client";
import "./styles/kiosk.css";
const Kiosk = lazy(() =>
  import("./components/Kiosk").then((module) => ({ default: module.Kiosk })),
);
const Hmi = lazy(() =>
  import("./components/Hmi").then((module) => ({ default: module.Hmi })),
);
const element = document.getElementById("root");
if (!element) throw new Error("앱 루트 요소가 없습니다.");
createRoot(element).render(
  <Suspense fallback={<p role="status">화면을 준비하고 있습니다.</p>}>
    {location.pathname.startsWith("/hmi") ? <Hmi /> : <Kiosk />}
  </Suspense>,
);
