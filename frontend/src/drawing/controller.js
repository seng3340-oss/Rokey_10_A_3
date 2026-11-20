import { DRAFT_CHANNEL } from "../api/draftPreview.ts";
import { cakeView } from "./cake3d.js";
import { submitDrawing } from "../api/drawing";
import { CAKE_DIMENSIONS, DRAWING_LAYOUT } from "./cakeDimensions";
/**
 * React가 화면을 붙인 뒤 캔버스·미리보기·확인 창의 동작을 연결한다.
 * 그림은 면별 브라우저 메모리에 보관하며, 반환한 정리 함수는 화면 해제 시 호출한다.
 */
/** @param {{existingOrderId?: string, onOrderReset?: () => void}} options 재개 주문은 조회만 연결한다. */
export function mountDrawing(options = {}) {
  // 접수 응답을 기다리는 동안 연속 클릭으로 같은 그림이 중복 제출되지 않게 한다.
  let submitting = false;
  // 데모는 명시적인 주소에서만 사용하고 실제 주문 요청을 보내지 않는다.
  const demoMode =
    new URLSearchParams(globalThis.location?.search || "").get("demo") === "1";
  let statusTimer = null;
  let feedbackTimer = null;
  let orderActive = false;
  let disposed = false;
  let resumeLoadController = null;
  let resumeLoadTimer = null;
  let lastActivity = performance.now();
  // 마지막 조작 후 3초를 기다린 다음 공통 60초 카운트를 시작한다.
  function remainingIdle() {
    return Math.max(
      0,
      Math.min(60000, 63000 - (performance.now() - lastActivity)),
    );
  }
  function recordActivity(event) {
    if (event.type === "pointermove" && !event.buttons) return;
    lastActivity = performance.now();
  }

  function showOrderStatus(orderId, demo) {
    const panel = document.querySelector("#order-status");
    if (typeof HTMLElement === "undefined" || !(panel instanceof HTMLElement))
      return;
    const dialog = modal;
    orderActive = true;
    panel.setAttribute("aria-hidden", "false");
    dialog.classList.add("is-ordering");
    // 이전 주문의 실패 강조·숨김 상태가 새 주문의 진행 화면으로 이어지지 않게 한다.
    dialog.classList.remove("is-order-failed");
    if (idleInterval !== null) clearInterval(idleInterval);
    idleInterval = null;
    const message = document.querySelector("#order-message");
    const title = document.querySelector("#order-title");
    const percent = document.querySelector("#order-percent");
    const progress = document.querySelector("#order-progress");
    progress.hidden = false;
    percent.hidden = false;
    panel.setAttribute("role", "status");
    const pickup = document.querySelector("#pickup");
    pickup.hidden = true;
    pickup.onclick = resetOrder;
    document.querySelector("#order-number").textContent = demo
      ? ""
      : `주문번호 ${orderId}`;
    // TM의 DrawingPath 순번을 접수한 원본의 면 순서와 대응시킨다.
    const orderedFaces = [
      reviewedPayload,
      ...(reviewedPayload.additional_paths || []),
    ]
      .filter((path) => path.strokes.length > 0)
      .map((path) => path.drawing_surface);

    /** 고객 문구는 서버의 공정 단계와 경로 순번을 기준으로 표시한다. */
    function stageMessage(order) {
      if (order.status !== "running") return "주문한 그림을 준비하고 있어요";
      if (order.phase === "PREPARING")
        return "케이크를 작업대에 준비하고 있어요";
      if (order.phase === "FINISHING") return "완성된 케이크를 옮기고 있어요";
      if (order.phase === "DRAWING") {
        const match = /DrawingPath\[(\d+)\/(\d+)\]/.exec(order.message || "");
        const index = match ? Number(match[1]) - 1 : 0;
        // 순번이 없는 복수 면 주문에서는 현재 면을 추정하지 않는다.
        const face =
          match || orderedFaces.length === 1 ? orderedFaces[index] : null;
        if (face === "top") return "윗면에 그림을 그리고 있어요";
        if (face === "side") return "옆면에 그림을 그리고 있어요";
        return "케이크에 그림을 그리고 있어요";
      }
      return "케이크를 준비하고 있어요";
    }
    // 화면 확인용 공정은 3초 안에 완료한다.
    const demoStartedAt = performance.now();

    // 실제 진행률은 주문 조회 API의 값만 사용한다. 예시는 별도로 표시한다.
    async function updateStatus() {
      try {
        let order;
        if (demo) {
          const exampleProgress = Math.min(
            100,
            Math.round(((performance.now() - demoStartedAt) / 2800) * 100),
          );
          order = {
            status: exampleProgress === 100 ? "completed" : "running",
            progress_pct: exampleProgress,
            phase:
              exampleProgress < 15
                ? "PREPARING"
                : exampleProgress < 90
                  ? "DRAWING"
                  : "FINISHING",
            message: `DrawingPath[${exampleProgress < 55 ? 1 : orderedFaces.length}/${orderedFaces.length}]`,
          };
        } else {
          const response = await fetch(
            `/api/orders/${encodeURIComponent(orderId)}`,
            { cache: "no-store" },
          );
          if (!response.ok) throw new Error("조회 실패");
          order = await response.json();
          // 접수한 주문의 결과만 고객 화면에 반영한다. 다른 주문 응답은 재조회한다.
          if (order.order_id !== orderId) throw new Error("주문 번호 불일치");
        }
        if (disposed || !dialog.open) return;
        if (
          !Number.isFinite(order.progress_pct) ||
          order.progress_pct < 0 ||
          order.progress_pct > 100
        )
          throw new Error("진행률 오류");
        const terminalError = [
          "paused",
          "failed",
          "rejected",
          "uncertain",
          "resolved",
        ].includes(order.status);
        title.textContent =
          order.status === "completed"
            ? "케이크가 완성됐어요"
            : terminalError
              ? "제작 과정에서 문제가 발생했습니다"
              : "케이크를 준비하고 있어요";
        message.textContent = terminalError
          ? "관리자에게 문의해주세요."
          : order.status === "completed"
            ? "완성된 케이크를 확인해주세요."
            : stageMessage(order);
        percent.textContent = `전체 공정 ${Math.round(order.progress_pct)}%`;
        progress.value = order.progress_pct;
        // 최종 실패는 진행률·완성 미리보기와 구분해 관리자 문의 화면으로 고정한다.
        // 서버가 재집기 중인 동안은 running이므로 이 화면으로 전환하지 않는다.
        dialog.classList.toggle("is-order-failed", terminalError);
        progress.hidden = terminalError;
        percent.hidden = terminalError;
        panel.setAttribute("role", terminalError ? "alert" : "status");
        if (order.status === "completed") {
          // 버튼 클릭 또는 실제 경과 60초 중 먼저 오는 시점에 초기화한다.
          pickup.hidden = false;
          lastActivity = performance.now();
          function updatePickup() {
            if (disposed) return;
            const remaining = remainingIdle();
            document.querySelector("#pickup-seconds").textContent = String(
              Math.ceil(remaining / 1000),
            );
            document
              .querySelector("#pickup-ring")
              .style.setProperty("--remaining", `${remaining / 600}%`);
            if (remaining === 0) resetOrder();
            else statusTimer = setTimeout(updatePickup, 100);
          }
          updatePickup();
          return;
        }
        // 실패에는 픽업 버튼·자동 초기화·자동 재주문을 제공하지 않는다.
        // 주문 번호와 안내를 남겨 관리자가 실제 장비 상태를 확인하도록 한다.
        if (terminalError) return;
      } catch {
        if (disposed || !dialog.open) return;
        message.textContent =
          "진행 상태를 확인하고 있어요. 잠시만 기다려주세요.";
        percent.textContent = "진행률 확인 중";
        progress.removeAttribute("value");
      }
      statusTimer = setTimeout(updateStatus, demo ? 100 : 3000);
    }
    function resetOrder() {
      if (disposed) return;
      if (statusTimer !== null) clearTimeout(statusTimer);
      pickup.hidden = true;
      dialog.close();
      dialog.classList.remove("is-ordering");
      panel.setAttribute("aria-hidden", "true");
      orderActive = false;
      drawings.top.length = 0;
      drawings.side.length = 0;
      current = null;
      reviewedPayload = null;
      surface = "top";
      document.querySelectorAll("[data-surface]").forEach((button) => {
        const selected = button.dataset.surface === surface;
        button.classList.toggle("selected", selected);
        button.setAttribute("aria-pressed", String(selected));
      });
      document.querySelector("#hint").textContent = "";
      render();
      updateTextures();
      publishDraft();
      options.onOrderReset?.();
    }
    updateStatus();
  }
  // JSX의 고정 ID를 연결점으로 사용한다. 화면 구조와 이 선택자는 함께 유지한다.
  const canvas = document.querySelector("#draw");
  const ctx = canvas.getContext("2d", { alpha: false });
  const modal = document.querySelector("#modal");
  // 각 면은 획 배열, 각 획은 점 배열이다. 면을 전환해도 반대 면의 그림은 보존한다.
  const drawings = { top: [], side: [] };
  // 현재 입력 면·진행 중인 획·포인터와 제출 전 확인한 데이터의 사본을 구분한다.
  let surface = "top";
  let current = null;
  let pointer = null;
  let reviewedPayload = null;
  // 같은 출처의 HMI에만 메모리 사본을 보낸다. 서버 접수나 ROS 호출은 하지 않는다.
  const draftChannel =
    typeof BroadcastChannel === "undefined"
      ? null
      : new BroadcastChannel(DRAFT_CHANNEL);
  function publishDraft() {
    draftChannel?.postMessage({
      kind: "draft-preview",
      surface,
      drawings: ["top", "side"].map((face) => ({
        canvas_width: 1024,
        canvas_height: 768,
        drawing_surface: face,
        strokes: drawings[face].map((points, stroke_id) => ({
          stroke_id,
          points,
        })),
      })),
    });
  }
  // 늦게 열린 HMI도 현재 그림을 받는다. 채널 외에는 초안을 저장하지 않는다.
  if (draftChannel)
    draftChannel.onmessage = (event) => {
      if (event.data?.kind === "request-draft") publishDraft();
    };
  // 입력 캔버스와 별개인 텍스처를 사용해 두 면을 동시에 3D에 표시한다.
  const textures = {
    top: document.createElement("canvas"),
    side: document.createElement("canvas"),
  };
  // 원형 윗면은 정사각형, 원통 옆면은 가로로 긴 이미지에 펼쳐 담는다.
  textures.top.width = textures.top.height = 512;
  textures.side.width = 1024;
  textures.side.height = 256;
  // 오른쪽 미리보기는 즉시 생성하고, 확인 창의 3D는 처음 열릴 때 생성한다.
  const live = cakeView(document.querySelector("#live3d"), textures, {
    cream: true,
  });
  let review = null;
  /** 확인창이 열린 뒤 3D를 준비한다. 최초 제출과 경고 후 재개가 같은 경로를 쓴다. */
  function prepareReview() {
    if (!review)
      review = cakeView(document.querySelector("#review3d"), textures, {
        cream: true,
      });
    review.update(drawings);
    review.view("angle");
    review.resize();
  }

  /** 저장된 두 면의 그림을 표시용 사본으로 복원하고 3D 텍스처를 다시 만든다.
   * 원본 주문 좌표는 유지한다. 표시 사본만 입력기의 1024×768 기준으로 환산한다.
   */
  function restoreOrderPreview(drawing) {
    drawings.top.length = 0;
    drawings.side.length = 0;
    for (const path of [drawing, ...(drawing.additional_paths || [])]) {
      const face = path.drawing_surface;
      if (face !== "top" && face !== "side")
        throw new Error("재개 주문의 표면 정보 오류");
      drawings[face] = path.strokes.map((stroke) =>
        stroke.points.map((point) => ({
          x: (point.x * 1024) / path.canvas_width,
          y: (point.y * 768) / path.canvas_height,
        })),
      );
    }
    // 진행률 카드 배치가 정해진 뒤 렌더러를 만들고 원본의 두 면을 함께 입힌다.
    // ResizeObserver가 태블릿의 줄어든 좌측 영역에 맞춰 버퍼 크기를 갱신한다.
    prepareReview();
    flushTextures();
  }

  let textureTimer = null;
  // 2D 필기는 즉시 표시하고 3D 텍스처는 최대 초당 10번만 갱신한다.
  function scheduleTextures() {
    if (textureTimer === null)
      textureTimer = setTimeout(() => {
        textureTimer = null;
        updateTextures();
      }, 100);
  }
  /** 입력 종료·화면 확인 시 예약 갱신을 비우고 마지막 선분까지 즉시 반영한다. */
  function flushTextures() {
    if (textureTimer !== null) {
      clearTimeout(textureTimer);
      textureTimer = null;
    }
    updateTextures();
  }
  /**
   * 옆면 띠의 좌우 끝을 이음매로 붙이는 거리(px).
   * 옆면 띠 896px이 평균 둘레 약 366.8mm에 대응하므로 12px은 약 5mm다.
   * 손으로 끝까지 그어도 경계 몇 px 앞에서 멈추는 경우를 흡수하되,
   * 이음매에서 일부러 떨어뜨린 획까지 붙이지 않도록 작은 값으로 둔다.
   */
  const SIDE_SEAM_SNAP_PX = 12;
  /** 고정 입력 좌표계에서 실제 필기 허용 영역을 판정한다. 점선보다 약간 안쪽이다. */
  function inside(p) {
    const top = DRAWING_LAYOUT.top;
    const side = DRAWING_LAYOUT.side;
    // 윗면과 옆면 위아래는 점선에서 5px 안쪽으로 제한하여 테두리에 필기가 걸치지 않게 한다.
    // 옆면 좌우는 원통을 펼친 이음매라 실제 테두리가 없다. 여백을 두면 끝까지 그은 선도
    // 로봇에서는 이음매에 약 4mm 빈틈이 생기므로 좌우는 띠 경계까지 허용한다.
    return surface === "top"
      ? Math.hypot(p.x - top.centerX, p.y - top.centerY) <=
          top.drawableRadius - 3
      : p.x >= side.x &&
          p.x <= side.x + side.width &&
          p.y >= side.y + 5 &&
          p.y <= side.drawableBottomY - 5;
  }
  /**
   * 옆면 획의 시작·끝점이 좌우 경계 근처이면 x를 경계값으로 맞춘 새 점을 반환한다.
   * 시작점(pointerdown)과 끝점(finish)에만 적용하며, 획 중간 점은 사용자가 그린 위치를 유지한다.
   * 띠 밖으로 조금 벗어난 점도 SIDE_SEAM_SNAP_PX 이내면 경계로 끌어와 입력으로 인정한다.
   */
  function snapToSeam(p) {
    if (surface !== "side") return p;
    const side = DRAWING_LAYOUT.side;
    const left = side.x;
    const right = side.x + side.width;
    if (Math.abs(p.x - left) <= SIDE_SEAM_SNAP_PX) return { x: left, y: p.y };
    if (Math.abs(p.x - right) <= SIDE_SEAM_SNAP_PX) return { x: right, y: p.y };
    return p;
  }
  /**
   * 옆면 획이 좌우 경계를 넘어 나갈 때, 직전 점과 바깥 점을 잇는 선분이 경계와 만나는 점을 구한다.
   * 이 점을 획 끝에 추가해야 빠르게 그은 선도 이음매까지 닿는다.
   * 위아래 경계로 나가거나 교점이 허용 높이 밖이면 null을 반환해 기존처럼 획만 끝낸다.
   */
  function seamExitPoint(previous, outside) {
    if (surface !== "side") return null;
    const side = DRAWING_LAYOUT.side;
    let edgeX = null;
    if (outside.x < side.x) edgeX = side.x;
    else if (outside.x > side.x + side.width) edgeX = side.x + side.width;
    if (edgeX === null || outside.x === previous.x) return null;
    // 직선 보간 비율. 직전 점은 영역 안이므로 0~1 사이 값이다.
    const ratio = (edgeX - previous.x) / (outside.x - previous.x);
    const exit = { x: edgeX, y: previous.y + (outside.y - previous.y) * ratio };
    return inside(exit) ? exit : null;
  }
  /** 화면 크기와 무관하게 포인터 위치를 원본 1024×768 캔버스 좌표로 환산한다. */
  function point(e) {
    const r = canvas.getBoundingClientRect();
    return {
      x: ((e.clientX - r.left) * 1024) / r.width,
      y: ((e.clientY - r.top) * 768) / r.height,
    };
  }
  /** 화면 확대율을 보정해 두 면의 선을 같은 표시 굵기로 그린다. */
  function paths(target, strokes, width = 5) {
    // 옆면은 세로 확대가 적용되어 있으므로 가로·세로 배율을 각각 보정한다.
    // 저장 좌표와 3D 텍스처는 기존 값을 사용한다.
    const bounds = canvas.getBoundingClientRect();
    const scaleX = target === ctx ? bounds.width / 1024 : 1;
    const scaleY = target === ctx ? bounds.height / 768 : 1;
    target.save();
    target.scale(1 / scaleX, 1 / scaleY);
    target.strokeStyle = "#795041";
    target.lineWidth = target === ctx ? 4 : width;
    target.lineCap = "round";
    target.lineJoin = "round";
    for (const stroke of strokes) {
      target.beginPath();
      stroke.forEach((point, index) => {
        const x = point.x * scaleX;
        const y = point.y * scaleY;
        if (index) target.lineTo(x, y);
        else target.moveTo(x, y);
      });
      target.stroke();
    }
    target.restore();
  }
  /** 면별 필기 영역을 잘라 텍스처 크기로 옮긴 뒤 두 3D 뷰에 변경을 알린다. */
  function updateTextures() {
    publishDraft();
    for (const name of ["top", "side"]) {
      const c = textures[name];
      const t = c.getContext("2d");
      // 직전 면·직전 갱신의 변환이 누적되지 않도록 매번 원래 좌표계에서 시작한다.
      t.setTransform(1, 0, 0, 1, 0, 0);
      t.fillStyle = "#fffaf1";
      t.fillRect(0, 0, c.width, c.height);
      // 물리 치수에서 계산한 동일 영역을 잘라 3D에 입힌다. 입력 여백은 제외한다.
      const region = DRAWING_LAYOUT[name];
      t.scale(c.width / region.width, c.height / region.height);
      t.translate(-region.x, -region.y);
      // 픽셀당 실제 길이가 다른 두 면의 선을 같은 물리적 굵기로 입힌다.
      const circumference =
        (Math.PI *
          (CAKE_DIMENSIONS.topDiameterMm + CAKE_DIMENSIONS.bottomDiameterMm)) /
        2;
      const sidePixelsPerMm = DRAWING_LAYOUT.side.width / circumference;
      const width =
        name === "top"
          ? (5 * (DRAWING_LAYOUT.top.width / CAKE_DIMENSIONS.topDiameterMm)) /
            sidePixelsPerMm
          : 5;
      paths(t, drawings[name], width);
      t.setTransform(1, 0, 0, 1, 0, 0);
    }
    // 표시 전용 입체 획에 사본을 만들게 하며 원본 좌표·주문 전송은 그대로 둔다.
    live.update(drawings);
    if (modal.open) review?.update(drawings);
  }
  /** 배경·점선·저장된 획과 편집 버튼을 현재 면에 맞춰 다시 동기화한다. */
  function render() {
    canvas.dataset.surface = surface;
    ctx.fillStyle = "#fdfcf9";
    ctx.fillRect(0, 0, 1024, 768);
    ctx.beginPath();
    const region = DRAWING_LAYOUT[surface];
    if (surface === "top")
      ctx.arc(region.centerX, region.centerY, region.radius, 0, 2 * Math.PI);
    else ctx.rect(region.x, region.y, region.width, region.height);
    ctx.fillStyle = surface === "top" ? "#e8e4df" : "#fff";
    ctx.fill();
    if (surface === "top") {
      // 외곽 5mm는 회색, 그릴 수 있는 안쪽은 흰색으로 표시한다. 그림을 축소하지 않는다.
      ctx.beginPath();
      ctx.arc(
        region.centerX,
        region.centerY,
        region.drawableRadius,
        0,
        2 * Math.PI,
      );
      ctx.fillStyle = "#fff";
      ctx.fill();
    }
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#d7c8b6";
    ctx.setLineDash([7, 8]);
    ctx.stroke();
    ctx.setLineDash([]);
    if (surface === "side") {
      // 케이크 전체 비율을 유지하면서 하단 금지 띠를 눈에 보이게 구분한다.
      const blockedHeight = region.y + region.height - region.drawableBottomY;
      ctx.fillStyle = "#e8e4df";
      ctx.fillRect(
        region.x,
        region.drawableBottomY,
        region.width,
        blockedHeight,
      );
      // fillRect는 경로를 바꾸지 않으므로 아래 clip은 원래 케이크 외곽을 쓴다.
    }
    // 점선 내부로 필기 표시를 제한하고, 이후 렌더링에는 클리핑을 남기지 않는다.
    ctx.save();
    ctx.clip();
    paths(ctx, drawings[surface]);
    ctx.restore();
    const n = drawings[surface].length;
    document.querySelector("#empty-hint").hidden = n > 0;
    document.querySelector("#stroke-count").textContent = n + "획";
    document.querySelector("#undo").disabled = !n;
    document.querySelector("#clear").disabled = !n;
    scheduleTextures();
  }
  /** 포인터 입력을 마무리한다. 선분을 만들지 못한 단일 점은 유효 획에서 제외한다. */
  function finish() {
    if (current && current.length < 2) {
      const index = drawings[surface].indexOf(current);
      if (index >= 0) drawings[surface].splice(index, 1);
    } else if (current) {
      // 끝점이 옆면 좌우 경계 근처에서 멈췄다면 이음매에 정확히 맞춘다.
      // 제출 데이터와 화면 표시가 같은 배열을 쓰므로 미리보기에도 그대로 반영된다.
      current[current.length - 1] = snapToSeam(current[current.length - 1]);
    }
    current = null;
    pointer = null;
    render();
    flushTextures();
  }
  // 첫 주 포인터만 받아 멀티터치로 하나의 획에 서로 다른 손가락이 섞이지 않게 한다.
  canvas.addEventListener("pointerdown", (e) => {
    if (pointer !== null || e.isPrimary === false) return;
    // 옆면 좌우 경계 근처에서 시작한 획은 이음매에서 시작하도록 먼저 맞춘다.
    const p = snapToSeam(point(e));
    if (!inside(p)) return;
    pointer = e.pointerId;
    current = [p];
    drawings[surface].push(current);
    // 캔버스 밖으로 이동해도 입력 종료 이벤트를 이어서 받을 수 있도록 캡처한다.
    canvas.setPointerCapture(pointer);
    render();
  });
  // 브라우저가 합쳐 전달한 입력도 모두 저장해 빠른 필기에서 점이 생략되지 않게 한다.
  canvas.addEventListener("pointermove", (e) => {
    if (e.pointerId !== pointer || !current) return;
    for (const event of e.getCoalescedEvents ? e.getCoalescedEvents() : [e]) {
      const p = point(event);
      const previous = current[current.length - 1];
      // 허용 영역 밖으로 나가면 획을 종료한다. 다시 들어오는 선을 임의로 잇지 않는다.
      if (!inside(p)) {
        // 옆면 좌우로 나간 경우에는 바깥 점을 버리기 전에 경계 교점을 마지막 점으로 남긴다.
        // 빠른 필기에서 직전 표본이 경계 몇 px 앞에 머물러 이음매가 비는 것을 막는다.
        const exit = seamExitPoint(previous, p);
        if (exit && (exit.x !== previous.x || exit.y !== previous.y)) {
          current.push(exit);
        }
        finish();
        return;
      }
      current.push(p);
      // 누적 경로 전체를 다시 그리지 않고 새 선분만 즉시 표시한다.
      paths(ctx, [[previous, p]]);
    }
    scheduleTextures();
  });
  // 정상 종료뿐 아니라 터치 취소·캡처 해제에서도 진행 중인 획을 정리한다.
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"])
    canvas.addEventListener(type, (e) => {
      if (e.pointerId === pointer) finish();
    });
  // 면 전환 전 획을 닫고, 선택 표시·접근성 상태·안내·3D 시점을 함께 바꾼다.
  for (const button of document.querySelectorAll("[data-surface]"))
    button.onclick = () => {
      finish();
      surface = button.dataset.surface;
      document.querySelectorAll("[data-surface]").forEach((b) => {
        b.classList.toggle("selected", b === button);
        b.setAttribute("aria-pressed", String(b === button));
      });
      canvas.setAttribute(
        "aria-label",
        surface === "top" ? "윗면에 그림 그리기" : "옆면에 그림 그리기",
      );
      document.querySelector("#hint").textContent = "";
      live.view(surface === "side" ? "side" : "angle");
      render();
    };
  // 일반 미리보기와 확인 창은 각자의 카메라를 조작하고 필기 데이터는 공유한다.
  for (const button of document.querySelectorAll("[data-view]"))
    button.onclick = () => live.view(button.dataset.view);
  for (const button of document.querySelectorAll("[data-review-view]"))
    button.onclick = () => review?.view(button.dataset.reviewView);
  // 취소·지우기는 현재 면에만 적용한다. 반대 면의 작업은 그대로 남긴다.
  document.querySelector("#undo").onclick = () => {
    finish();
    drawings[surface].pop();
    render();
  };
  document.querySelector("#clear").onclick = () => {
    finish();
    drawings[surface] = [];
    render();
  };
  /**
   * 각 면의 원본 획을 전송 형식으로 복사한다. 두 면은 아래에서 한 주문으로 묶는다.
   * 유효 획을 순서대로 재번호화하고, 좌표 정밀도만 소수 셋째 자리로 맞춘다.
   */
  function pathPayload(surface) {
    return {
      canvas_width: 1024,
      canvas_height: 768,
      drawing_surface: surface,
      strokes: drawings[surface]
        .filter((s) => s.length >= 2)
        .map((s, i) => ({
          stroke_id: i,
          points: s.map((p) => ({
            x: Number(p.x.toFixed(3)),
            y: Number(p.y.toFixed(3)),
          })),
        })),
    };
  }
  // 한 케이크의 두 면은 주문 하나로 제출한다. 면 안의 획 순서는 유지한다.
  function payload() {
    const paths = ["top", "side"]
      .map(pathPayload)
      .filter((path) => path.strokes.length > 0);
    if (!paths.length) return null;
    return paths.length === 2
      ? { ...paths[0], additional_paths: [paths[1]] }
      : paths[0];
  }
  // 확인 창을 열 때 요청 사본을 고정해 사용자가 확인한 면과 제출할 면을 일치시킨다.
  document.querySelector("#review").onclick = () => {
    finish();
    if (!drawings.top.length && !drawings.side.length) {
      document.querySelector("#hint").textContent = "그림을 먼저 그려주세요.";
      return;
    }
    // 두 면의 유효 획을 모아 확인 시점의 주문 사본을 만든다.
    reviewedPayload = payload();
    document.querySelector("#payload-panel").hidden = true;
    document.querySelector("#confirm").disabled = false;
    document.querySelector("#confirm").textContent = "제작 요청";
    modal.showModal();
    prepareReview();
  };
  document.querySelector("#back").onclick = () => modal.close();
  // 요청 중에는 버튼으로 중복 제출을 막고 실패한 경우에만 알림을 표시한다.
  document.querySelector("#confirm").onclick = async () => {
    if (submitting) return;
    if (feedbackTimer !== null) clearTimeout(feedbackTimer);
    document.querySelector("#payload-panel").hidden = true;
    const result = document.querySelector("#result");
    if (!reviewedPayload?.strokes.length) {
      document.querySelector("#payload-panel").hidden = false;
      result.textContent = "그림을 먼저 그려주세요.";
      return;
    }
    // 응답 대기 중에는 제출과 복귀를 잠시 막아 요청 대상이 바뀌지 않도록 한다.
    submitting = true;
    document.querySelector("#confirm").disabled = true;
    document.querySelector("#back").disabled = true;
    result.textContent = "";
    try {
      const receipt = demoMode
        ? { order_id: "화면 확인용" }
        : await submitDrawing(reviewedPayload);
      result.textContent = `주문 ${receipt.order_id} 접수 완료`;
      document.querySelector("#confirm").textContent = "접수 완료";
      showOrderStatus(receipt.order_id, demoMode);
      // 실패 시 그림과 확인한 요청을 유지하고 재시도할 수 있도록 버튼만 복구한다.
    } catch (error) {
      document.querySelector("#payload-panel").hidden = false;
      result.textContent =
        error instanceof Error ? error.message : "주문 접수에 실패했습니다.";
      document.querySelector("#confirm").disabled = false;
      // 오류는 배치 위에 6초 표시한다. 재시도하면 이전 숨김 예약을 취소한다.
      feedbackTimer = setTimeout(() => {
        document.querySelector("#payload-panel").hidden = true;
      }, 6000);
    } finally {
      submitting = false;
      document.querySelector("#back").disabled = false;
      resetIdle();
    }
  };
  render();

  // 전체 확인 화면의 무조작 시간만 계산한다. 복귀해도 입력 데이터는 지우지 않는다.
  const idleDuration = 60000;
  let idleInterval = null;
  /** 절대 마감 시각으로 남은 시간을 구해 탭 비활성화로 타이머가 늦어져도 보정한다. */
  function updateIdle() {
    if (!modal.open || orderActive) return;
    const remaining = remainingIdle();
    document.querySelector("#idle-seconds").textContent = String(
      Math.ceil(remaining / 1000),
    );
    document
      .querySelector("#idle-ring")
      .style.setProperty("--remaining", `${(remaining / idleDuration) * 100}%`);
    // 접수 중에는 창을 자동으로 닫지 않는다. 응답 처리 후 다시 60초를 부여한다.
    if (remaining === 0 && !submitting) {
      modal.close();
      document.querySelector("#hint").textContent =
        "60초 동안 조작이 없어 수정 화면으로 돌아왔어요. 그림은 그대로 남아 있어요.";
      document.querySelector("#review").focus();
    }
  }
  /** 열린 확인 창의 사용자 활동에만 무조작 시간을 새로 부여한다. */
  function resetIdle() {
    if (!modal.open || orderActive) return;
    lastActivity = performance.now();
    updateIdle();
  }
  // 사용자가 확인 화면을 연 시점부터 시작한다.
  document.querySelector("#review").addEventListener("click", () => {
    if (!modal.open || orderActive) return;
    if (idleInterval !== null) clearInterval(idleInterval);
    resetIdle();
    idleInterval = setInterval(updateIdle, 200);
  });
  // 실제 클릭·키 입력·스크롤은 활동으로 본다. 손을 대지 않은 포인터 이동은 제외한다.
  for (const type of ["pointerdown", "click", "keydown", "wheel", "scroll"])
    modal.addEventListener(type, resetIdle, { passive: true });
  modal.addEventListener(
    "pointermove",
    (event) => {
      if (event.buttons) resetIdle();
    },
    { passive: true },
  );
  // 공정 중에는 Esc로 새 주문 화면을 열지 않도록 현재 상태 화면을 유지한다.
  modal.addEventListener("cancel", (event) => {
    if (orderActive) event.preventDefault();
  });
  modal.addEventListener("close", () => {
    lastActivity = performance.now();
    if (idleInterval !== null) clearInterval(idleInterval);
    idleInterval = null;
  });
  // 다른 탭에서 돌아오면 누락된 타이머 틱을 기다리지 않고 남은 시간을 확인한다.
  const onVisibility = () => {
    if (modal.open) updateIdle();
  };
  document.addEventListener("visibilitychange", onVisibility);

  // 두 면 중 하나에 그림이 있으면 그리기 화면 오른쪽 위에 카운트를 표시한다.
  const drawingCounter = document.querySelector("#drawing-idle");
  function updateDrawingIdle() {
    const visible = drawings.top.length > 0 || drawings.side.length > 0;
    drawingCounter.classList.toggle("visible", visible);
    drawingCounter.setAttribute("aria-hidden", String(!visible));
    if (!visible || modal.open || submitting || orderActive || current) return;
    const remaining = remainingIdle();
    document.querySelector("#drawing-idle-seconds").textContent = String(
      Math.ceil(remaining / 1000),
    );
    drawingCounter.style.setProperty("--remaining", `${remaining / 600}%`);
    if (remaining === 0) {
      drawings.top.length = 0;
      drawings.side.length = 0;
      reviewedPayload = null;
      render();
      flushTextures();
      drawingCounter.classList.remove("visible");
      drawingCounter.setAttribute("aria-hidden", "true");
    }
  }
  const activityEvents = [
    "pointerdown",
    "pointermove",
    "pointerup",
    "click",
    "keydown",
    "wheel",
    "scroll",
  ];
  for (const type of activityEvents)
    document.addEventListener(type, recordActivity, {
      passive: true,
      capture: true,
    });
  // 브라우저의 실제 카운터가 있을 때만 화면 갱신 작업을 예약한다.
  const drawingInterval =
    typeof HTMLElement !== "undefined" && drawingCounter instanceof HTMLElement
      ? setInterval(updateDrawingIdle, 100)
      : null;

  // 재개 중인 기존 주문은 새 POST 없이 상태 화면만 다시 연결한다.
  // 원본을 읽기 전에도 모달로 입력을 차단하고, 조회 실패 시 자동 재주문하지 않는다.
  if (options.existingOrderId) {
    orderActive = true;
    modal.showModal();
    modal.classList.add("is-ordering");
    document
      .querySelector("#order-status")
      .setAttribute("aria-hidden", "false");
    /** 통신 복구 시 같은 주문 조회만 재시도한다. 주문을 새로 생성하지 않는다. */
    async function loadResumedOrder() {
      resumeLoadController = new AbortController();
      const timeout = setTimeout(() => resumeLoadController?.abort(), 5000);
      try {
        const response = await fetch(
          `/api/orders/${encodeURIComponent(options.existingOrderId)}`,
          {
            cache: "no-store",
            signal: resumeLoadController.signal,
          },
        );
        if (!response.ok) throw new Error("기존 주문 조회 실패");
        const order = await response.json();
        if (disposed) return;
        if (order.order_id !== options.existingOrderId || !order.drawing) {
          throw new Error("재개 주문 정보 불일치");
        }
        reviewedPayload = order.drawing;
        restoreOrderPreview(order.drawing);
        showOrderStatus(order.order_id, false);
      } catch {
        if (!disposed) {
          document.querySelector("#order-message").textContent =
            "재개 주문을 확인하고 있습니다. 연결되면 자동으로 이어집니다.";
          resumeLoadTimer = setTimeout(loadResumedOrder, 2000);
        }
      } finally {
        clearTimeout(timeout);
      }
    }
    void loadResumedOrder();
  }
  // React 화면 해제 시 예약 작업과 전역 리스너, 두 WebGL 뷰의 자원을 정리한다.
  return () => {
    disposed = true;
    resumeLoadController?.abort();
    if (resumeLoadTimer !== null) clearTimeout(resumeLoadTimer);
    if (drawingInterval !== null) clearInterval(drawingInterval);
    for (const type of activityEvents)
      document.removeEventListener(type, recordActivity, { capture: true });
    if (feedbackTimer !== null) clearTimeout(feedbackTimer);
    if (statusTimer !== null) clearTimeout(statusTimer);
    if (textureTimer !== null) clearTimeout(textureTimer);
    if (idleInterval !== null) clearInterval(idleInterval);
    document.removeEventListener("visibilitychange", onVisibility);
    draftChannel?.close();
    live.dispose();
    review?.dispose();
  };
}
