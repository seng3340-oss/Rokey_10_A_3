import { lazy, Suspense } from "react";
import { createRoot } from "react-dom/client";
import "./styles/kiosk.css";
// 고객 화면의 Three.js 코드를 분리해 선택한 화면에 필요한 코드만 불러온다.
const Kiosk = lazy(() =>
  import("./components/Kiosk").then((module) => ({ default: module.Kiosk })),
);
const Hmi = lazy(() =>
  import("./components/Hmi").then((module) => ({ default: module.Hmi })),
);
// HTML의 마운트 지점을 먼저 확인해 잘못된 페이지 구성에서는 명확하게 실패한다.
const element = document.getElementById("root");
if (!element) throw new Error("앱 루트 요소가 없습니다.");
// 기존 경로 분기를 유지한다. 비동기 화면 로딩 중에는 접근 가능한 상태 문구를 표시한다.
createRoot(element).render(
  <Suspense fallback={<p role="status">화면을 준비하고 있습니다.</p>}>
    {location.pathname.startsWith("/hmi") ? <Hmi /> : <Kiosk />}
  </Suspense>,
);
