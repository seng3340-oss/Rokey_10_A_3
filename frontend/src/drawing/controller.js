import { cakeView } from "./cake3d.js";
import { submitDrawing } from "../api/drawing";
/**
 * React가 화면을 붙인 뒤 캔버스·미리보기·확인 창의 동작을 연결한다.
 * 그림은 면별 브라우저 메모리에 보관하며, 반환한 정리 함수는 화면 해제 시 호출한다.
 */
export function mountDrawing() {
  // 접수 응답을 기다리는 동안 연속 클릭으로 같은 그림이 중복 제출되지 않게 한다.
  let submitting = false;
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
  const live = cakeView(document.querySelector("#live3d"), textures);
  let review = null;
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
  /** 고정 입력 좌표계에서 실제 필기 허용 영역을 판정한다. 점선보다 약간 안쪽이다. */
  function inside(p) {
    return surface === "top"
      ? Math.hypot(p.x - 512, p.y - 384) <= 315
      : p.x >= 69 && p.x <= 955 && p.y >= 215 && p.y <= 553;
  }
  /** 화면 크기와 무관하게 포인터 위치를 원본 1024×768 캔버스 좌표로 환산한다. */
  function point(e) {
    const r = canvas.getBoundingClientRect();
    return {
      x: ((e.clientX - r.left) * 1024) / r.width,
      y: ((e.clientY - r.top) * 768) / r.height,
    };
  }
  /** 저장된 점을 순서대로 연결한다. 입력 화면과 3D 텍스처에 같은 획 표현을 쓴다. */
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
  /** 면별 필기 영역을 잘라 텍스처 크기로 옮긴 뒤 두 3D 뷰에 변경을 알린다. */
  function updateTextures() {
    for (const name of ["top", "side"]) {
      const c = textures[name];
      const t = c.getContext("2d");
      // 직전 면·직전 갱신의 변환이 누적되지 않도록 매번 원래 좌표계에서 시작한다.
      t.setTransform(1, 0, 0, 1, 0, 0);
      t.fillStyle = "#fffaf1";
      t.fillRect(0, 0, c.width, c.height);
      // 윗면 원의 외접 정사각형(640), 옆면 펼침 영역(896×348)을 기준으로 맞춘다.
      // 음수 이동은 입력 캔버스에 있던 여백을 텍스처에서 제외하는 역할이다.
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
  /** 배경·점선·저장된 획과 편집 버튼을 현재 면에 맞춰 다시 동기화한다. */
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
    }
    current = null;
    pointer = null;
    render();
    flushTextures();
  }
  // 첫 주 포인터만 받아 멀티터치로 하나의 획에 서로 다른 손가락이 섞이지 않게 한다.
  canvas.addEventListener("pointerdown", (e) => {
    if (pointer !== null || e.isPrimary === false) return;
    const p = point(e);
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
      // 허용 영역 밖으로 나가면 획을 종료한다. 다시 들어오는 선을 임의로 잇지 않는다.
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
   * 선택한 한 면만 전송 형식으로 복사한다. 미리보기는 두 면을 보여도 요청은 한 면이다.
   * 유효 획을 순서대로 재번호화하고, 좌표 정밀도만 소수 셋째 자리로 맞춘다.
   */
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
  // 확인 창을 열 때 요청 사본을 고정해 사용자가 확인한 면과 제출할 면을 일치시킨다.
  document.querySelector("#review").onclick = () => {
    finish();
    if (!drawings.top.length && !drawings.side.length) {
      document.querySelector("#hint").textContent = "그림을 먼저 그려주세요.";
      return;
    }
    // 전체 그림이 있더라도 선택 면이 비어 있을 수 있어 제출 시에도 별도로 검사한다.
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
  // 요청 내용은 확인 창에 공개하고, 접수 검증을 통과한 응답에만 완료 문구를 표시한다.
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
    // 응답 대기 중에는 제출과 복귀를 잠시 막아 요청 대상이 바뀌지 않도록 한다.
    submitting = true;
    document.querySelector("#confirm").disabled = true;
    document.querySelector("#back").disabled = true;
    result.textContent = "주문 접수 응답을 기다리는 중입니다.";
    try {
      const receipt = await submitDrawing(reviewedPayload);
      result.textContent = `주문 ${receipt.order_id} 접수 완료 · 로봇 제작 완료를 뜻하지 않습니다.`;
      document.querySelector("#confirm").textContent = "접수 완료";
      // 실패 시 그림과 확인한 요청을 유지하고 재시도할 수 있도록 버튼만 복구한다.
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
  // 확인한 동일 데이터를 로컬 JSON으로 내보낸다. 다운로드용 임시 URL은 곧 해제한다.
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
  let idleDeadline = 0;
  let idleInterval = null;
  /** 절대 마감 시각으로 남은 시간을 구해 탭 비활성화로 타이머가 늦어져도 보정한다. */
  function updateIdle() {
    if (!modal.open) return;
    const remaining = Math.max(0, idleDeadline - performance.now());
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
  modal.addEventListener("close", () => {
    if (idleInterval !== null) clearInterval(idleInterval);
    idleInterval = null;
  });
  // 다른 탭에서 돌아오면 누락된 타이머 틱을 기다리지 않고 남은 시간을 확인한다.
  const onVisibility = () => {
    if (modal.open) updateIdle();
  };
  document.addEventListener("visibilitychange", onVisibility);

  // React 화면 해제 시 예약 작업과 전역 리스너, 두 WebGL 뷰의 자원을 정리한다.
  return () => {
    if (textureTimer !== null) clearTimeout(textureTimer);
    if (idleInterval !== null) clearInterval(idleInterval);
    document.removeEventListener("visibilitychange", onVisibility);
    live.dispose();
    review?.dispose();
  };
}
