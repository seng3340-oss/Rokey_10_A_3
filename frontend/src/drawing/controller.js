import { cakeView } from "./cake3d.js";
import { submitDrawing } from "../api/drawing";
export function mountDrawing() {
  let submitting = false;
  const canvas = document.querySelector("#draw"),
    ctx = canvas.getContext("2d", { alpha: false }),
    modal = document.querySelector("#modal");
  const drawings = { top: [], side: [] };
  let surface = "top",
    current = null,
    pointer = null,
    reviewedPayload = null;
  const textures = {
    top: document.createElement("canvas"),
    side: document.createElement("canvas"),
  };
  textures.top.width = textures.top.height = 512;
  textures.side.width = 1024;
  textures.side.height = 256;
  const live = cakeView(document.querySelector("#live3d"), textures);
  let review = null,
    textureTimer = null;
  // 2D 필기는 즉시 표시하고 3D 텍스처는 최대 초당 10번만 갱신한다.
  function scheduleTextures() {
    if (textureTimer === null)
      textureTimer = setTimeout(() => {
        textureTimer = null;
        updateTextures();
      }, 100);
  }
  function flushTextures() {
    if (textureTimer !== null) {
      clearTimeout(textureTimer);
      textureTimer = null;
    }
    updateTextures();
  }
  function inside(p) {
    return surface === "top"
      ? Math.hypot(p.x - 512, p.y - 384) <= 315
      : p.x >= 69 && p.x <= 955 && p.y >= 215 && p.y <= 553;
  }
  function point(e) {
    const r = canvas.getBoundingClientRect();
    return {
      x: ((e.clientX - r.left) * 1024) / r.width,
      y: ((e.clientY - r.top) * 768) / r.height,
    };
  }
  function paths(target, strokes, width = 5) {
    target.strokeStyle = "#795041";
    target.lineWidth = width;
    target.lineCap = "round";
    target.lineJoin = "round";
    for (const s of strokes) {
      target.beginPath();
      s.forEach((p, i) =>
        i ? target.lineTo(p.x, p.y) : target.moveTo(p.x, p.y),
      );
      target.stroke();
    }
  }
  function updateTextures() {
    for (const name of ["top", "side"]) {
      const c = textures[name],
        t = c.getContext("2d");
      t.setTransform(1, 0, 0, 1, 0, 0);
      t.fillStyle = "#fffaf1";
      t.fillRect(0, 0, c.width, c.height);
      if (name === "top") {
        t.scale(c.width / 640, c.height / 640);
        t.translate(-192, -64);
      } else {
        t.scale(c.width / 896, c.height / 348);
        t.translate(-64, -210);
      }
      paths(t, drawings[name]);
      t.setTransform(1, 0, 0, 1, 0, 0);
    }
    live.update();
    if (modal.open) review?.update();
  }
  function render() {
    ctx.fillStyle = "#fdfcf9";
    ctx.fillRect(0, 0, 1024, 768);
    ctx.beginPath();
    if (surface === "top") ctx.arc(512, 384, 320, 0, 2 * Math.PI);
    else ctx.rect(64, 210, 896, 348);
    ctx.fillStyle = "#fff";
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#d7c8b6";
    ctx.setLineDash([7, 8]);
    ctx.stroke();
    ctx.setLineDash([]);
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
  function finish() {
    if (current && current.length < 2) {
      const index = drawings[surface].indexOf(current);
      if (index >= 0) drawings[surface].splice(index, 1);
    }
    current = null;
    pointer = null;
    render();
    flushTextures();
  }
  canvas.addEventListener("pointerdown", (e) => {
    if (pointer !== null || e.isPrimary === false) return;
    const p = point(e);
    if (!inside(p)) return;
    pointer = e.pointerId;
    current = [p];
    drawings[surface].push(current);
    canvas.setPointerCapture(pointer);
    render();
  });
  canvas.addEventListener("pointermove", (e) => {
    if (e.pointerId !== pointer || !current) return;
    for (const event of e.getCoalescedEvents ? e.getCoalescedEvents() : [e]) {
      const p = point(event);
      if (!inside(p)) {
        finish();
        return;
      }
      const previous = current[current.length - 1];
      current.push(p);
      // 누적 경로 전체를 다시 그리지 않고 새 선분만 즉시 표시한다.
      paths(ctx, [[previous, p]]);
    }
    scheduleTextures();
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"])
    canvas.addEventListener(type, (e) => {
      if (e.pointerId === pointer) finish();
    });
  for (const button of document.querySelectorAll("[data-surface]"))
    button.onclick = () => {
      finish();
      surface = button.dataset.surface;
      document.querySelectorAll("[data-surface]").forEach((b) => {
        b.classList.toggle("selected", b === button);
        b.setAttribute("aria-pressed", String(b === button));
      });
      document.querySelector("#surface-help").textContent =
        surface === "top"
          ? "점선 안쪽에 그려주세요. 케이크 윗면에 배치됩니다."
          : "띠 안에 그린 그림이 옆면을 둘러 배치돼요. 오른쪽 3D에서 돌려보세요.";
      canvas.setAttribute(
        "aria-label",
        surface === "top" ? "윗면에 그림 그리기" : "옆면에 그림 그리기",
      );
      document.querySelector("#hint").textContent = "";
      live.view(surface === "side" ? "side" : "angle");
      render();
    };
  for (const button of document.querySelectorAll("[data-view]"))
    button.onclick = () => live.view(button.dataset.view);
  for (const button of document.querySelectorAll("[data-review-view]"))
    button.onclick = () => review?.view(button.dataset.reviewView);
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
  function payload() {
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
  document.querySelector("#review").onclick = () => {
    finish();
    if (!drawings.top.length && !drawings.side.length) {
      document.querySelector("#hint").textContent = "그림을 먼저 그려주세요.";
      return;
    }
    reviewedPayload = payload();
    document.querySelector("#payload-panel").hidden = true;
    document.querySelector("#confirm").disabled = false;
    document.querySelector("#confirm").textContent = "선택한 면 제작 요청";
    modal.showModal();
    if (!review)
      review = cakeView(document.querySelector("#review3d"), textures);
    review.update();
    review.view("angle");
    review.resize();
  };
  document.querySelector("#back").onclick = () => modal.close();
  document.querySelector("#confirm").onclick = async () => {
    if (submitting) return;
    document.querySelector("#payload-panel").hidden = false;
    document.querySelector("#payload").textContent = JSON.stringify(
      reviewedPayload,
      null,
      2,
    );
    document.querySelector("#download").disabled =
      !reviewedPayload?.strokes.length;
    const result = document.querySelector("#result");
    if (!reviewedPayload?.strokes.length) {
      result.textContent =
        "선택한 면에 그림이 없습니다. 수정하기에서 그림이 있는 면을 선택해주세요.";
      return;
    }
    submitting = true;
    document.querySelector("#confirm").disabled = true;
    document.querySelector("#back").disabled = true;
    result.textContent = "주문 접수 응답을 기다리는 중입니다.";
    try {
      const receipt = await submitDrawing(reviewedPayload);
      result.textContent = `주문 ${receipt.order_id} 접수 완료 · 로봇 제작 완료를 뜻하지 않습니다.`;
      document.querySelector("#confirm").textContent = "접수 완료";
    } catch (error) {
      result.textContent =
        error instanceof Error ? error.message : "주문 접수에 실패했습니다.";
      document.querySelector("#confirm").disabled = false;
    } finally {
      submitting = false;
      document.querySelector("#back").disabled = false;
      resetIdle();
    }
  };
  document.querySelector("#download").onclick = () => {
    const url = URL.createObjectURL(
      new Blob([JSON.stringify(reviewedPayload, null, 2)], {
        type: "application/json",
      }),
    );
    const a = document.createElement("a");
    a.href = url;
    a.download = "drawing-" + reviewedPayload.drawing_surface + ".json";
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  render();

  // 전체 확인 화면의 무조작 시간만 계산한다. 복귀해도 입력 데이터는 지우지 않는다.
  const idleDuration = 60000;
  let idleDeadline = 0,
    idleInterval = null;
  function updateIdle() {
    if (!modal.open) return;
    const remaining = Math.max(0, idleDeadline - performance.now());
    document.querySelector("#idle-seconds").textContent = String(
      Math.ceil(remaining / 1000),
    );
    document
      .querySelector("#idle-ring")
      .style.setProperty("--remaining", `${(remaining / idleDuration) * 100}%`);
    if (remaining === 0 && !submitting) {
      modal.close();
      document.querySelector("#hint").textContent =
        "60초 동안 조작이 없어 수정 화면으로 돌아왔어요. 그림은 그대로 남아 있어요.";
      document.querySelector("#review").focus();
    }
  }
  function resetIdle() {
    if (!modal.open) return;
    idleDeadline = performance.now() + idleDuration;
    updateIdle();
  }
  // 사용자가 확인 화면을 연 시점부터 시작한다.
  document.querySelector("#review").addEventListener("click", () => {
    if (!modal.open) return;
    if (idleInterval !== null) clearInterval(idleInterval);
    resetIdle();
    idleInterval = setInterval(updateIdle, 200);
  });
  for (const type of ["pointerdown", "click", "keydown", "wheel", "scroll"])
    modal.addEventListener(type, resetIdle, { passive: true });
  modal.addEventListener(
    "pointermove",
    (event) => {
      if (event.buttons) resetIdle();
    },
    { passive: true },
  );
  modal.addEventListener("close", () => {
    if (idleInterval !== null) clearInterval(idleInterval);
    idleInterval = null;
  });
  const onVisibility = () => {
    if (modal.open) updateIdle();
  };
  document.addEventListener("visibilitychange", onVisibility);

  return () => {
    if (textureTimer !== null) clearTimeout(textureTimer);
    if (idleInterval !== null) clearInterval(idleInterval);
    document.removeEventListener("visibilitychange", onVisibility);
    live.dispose();
    review?.dispose();
  };
}
