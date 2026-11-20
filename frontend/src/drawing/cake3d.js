import * as THREE from "three";
import { MODEL_SIZE } from "./cakeDimensions";
import { createCreamLayer } from "./creamGeometry.js";
/**
 * 두 면의 원본 획을 생크림 입체로 표시하는 케이크와 조작·정리 함수를 반환한다.
 * 일반 미리보기와 확인 창이 각각 호출하며, 입력 그림 자체는 변경하지 않는다.
 * @param {HTMLElement} container
 * @param {{top: HTMLCanvasElement, side: HTMLCanvasElement}} textures
 * @param {{autoRotate?: boolean, cream?: boolean}} options HMI는 자동 회전도 선택한다.
 */
export function cakeView(container, textures, options = {}) {
  try {
    // 컨테이너마다 장면·카메라·렌더러를 분리해 두 미리보기의 시점을 독립시킨다.
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(34, 1, 0.1, 100);
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    // 고해상도 태블릿의 GPU 부담을 줄이되 승인 시안의 색 공간과 노출을 유지한다.
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.25));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 0.9;
    renderer.shadowMap.enabled = Boolean(options.cream);
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);
    // 본체·윗면·받침·테두리를 하나의 모형으로 구성하고 조명으로 입체감을 만든다.
    const group = new THREE.Group();
    scene.add(group);
    scene.add(new THREE.HemisphereLight(0xffffff, 0xc8b9a1, 1.8));
    const key = new THREE.DirectionalLight(0xfff7ec, 2);
    key.position.set(-3, 6, 4);
    // 크림의 얕은 그림자가 평평한 케이크 표면과 흰 입체 획을 구분한다.
    key.castShadow = Boolean(options.cream);
    key.shadow.mapSize.set(1024, 1024);
    key.shadow.camera.left = key.shadow.camera.bottom = -3;
    key.shadow.camera.right = key.shadow.camera.top = 3;
    key.shadow.normalBias = 0.003;
    key.shadow.bias = -0.0001;
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.7);
    fill.position.set(4, 2, -3);
    scene.add(fill);
    // 같은 원본 캔버스를 참조하되 각 뷰가 자신의 GPU 텍스처를 관리한다.
    const topTexture = new THREE.CanvasTexture(textures.top);
    const sideTexture = new THREE.CanvasTexture(textures.side);
    for (const t of [topTexture, sideTexture]) {
      t.colorSpace = THREE.SRGBColorSpace;
      t.anisotropy = Math.min(4, renderer.capabilities.getMaxAnisotropy());
    }
    // 실측 치수의 공통 축척을 사용하여 케이크 지름·높이와 판의 크기를 함께 맞춘다.
    // 윗면·아랫면 반지름을 따로 적용한 원뿔대에 옆면 그림을 입힌다.
    const side = new THREE.Mesh(
      new THREE.CylinderGeometry(
        MODEL_SIZE.cakeRadius,
        MODEL_SIZE.cakeBottomRadius,
        MODEL_SIZE.cakeHeight,
        96,
        1,
        true,
        Math.PI,
      ),
      new THREE.MeshStandardMaterial(
        options.cream
          ? { color: 0xf0e4cc, roughness: 0.93 }
          : { map: sideTexture, roughness: 0.85 },
      ),
    );
    side.position.y = MODEL_SIZE.cakeBottom + MODEL_SIZE.cakeHeight / 2;
    side.receiveShadow = true;
    group.add(side);
    // 원판을 수평으로 눕혀 원뿔대 위에 배치한다. 미세한 높이 차이로 겹침을 피한다.
    const top = new THREE.Mesh(
      new THREE.CircleGeometry(MODEL_SIZE.cakeRadius, 96),
      new THREE.MeshStandardMaterial(
        options.cream
          ? { color: 0xf5ead5, roughness: 0.92 }
          : { map: topTexture, roughness: 0.9 },
      ),
    );
    top.rotation.x = -Math.PI / 2;
    top.position.y = MODEL_SIZE.cakeTop + 0.001;
    top.receiveShadow = true;
    group.add(top);
    const cream = options.cream ? createCreamLayer() : null;
    if (cream) group.add(cream.group);
    // 받침과 가장자리 장식은 그림 텍스처와 분리해 케이크 윤곽을 유지한다.
    const board = new THREE.Mesh(
      new THREE.BoxGeometry(
        MODEL_SIZE.boardWidth,
        MODEL_SIZE.boardThickness,
        MODEL_SIZE.boardDepth,
      ),
      new THREE.MeshStandardMaterial({ color: 0xf7f5ed, roughness: 0.7 }),
    );
    board.position.y = MODEL_SIZE.boardThickness / 2;
    group.add(board);
    const rim = new THREE.Mesh(
      new THREE.TorusGeometry(0.991, 0.017, 8, 96),
      new THREE.MeshStandardMaterial({ color: 0xfffcf1, roughness: 1 }),
    );
    rim.rotation.x = Math.PI / 2;
    rim.position.y = MODEL_SIZE.cakeTop - 0.002;
    group.add(rim);
    // 밝은 받침과 그림자가 모형의 높이·바닥 위치를 구분한다.
    const shadowCanvas = document.createElement("canvas");
    shadowCanvas.width = shadowCanvas.height = 128;
    const sx = shadowCanvas.getContext("2d");
    const grad = sx.createRadialGradient(64, 64, 5, 64, 64, 64);
    grad.addColorStop(0, "rgba(70,45,20,0.20)");
    grad.addColorStop(1, "rgba(70,45,20,0)");
    sx.fillStyle = grad;
    sx.fillRect(0, 0, 128, 128);
    const shadow = new THREE.Mesh(
      new THREE.PlaneGeometry(
        MODEL_SIZE.boardWidth * 1.5,
        MODEL_SIZE.boardDepth * 1.5,
      ),
      new THREE.MeshBasicMaterial({
        map: new THREE.CanvasTexture(shadowCanvas),
        transparent: true,
        depthWrite: false,
      }),
    );
    shadow.rotation.x = -Math.PI / 2;
    shadow.position.y = -0.01;
    scene.add(shadow);
    // 수평 회전각과 수직 시야각을 구면 좌표로 관리하고, 카메라 거리는 고정한다.
    let theta = 0;
    let phi = 0.96;
    let drag = null;
    // 고객 미리보기는 카드 안에서 크게 표시한다. 확인 창은 기존 시야를 사용한다.
    const radius = container.id === "live3d" ? 6.0 : 5.8;
    let frame = null;
    let previousTime = 0;
    let lastRender = 0;
    let disposed = false;
    /** 예약된 한 프레임을 그린다. 보이지 않는 컨테이너는 크기를 얻을 때까지 건너뛴다. */
    function draw(timestamp) {
      frame = null;
      if (disposed) return;
      if (
        container.dataset.ready === "lost" ||
        (options.autoRotate && document.hidden)
      ) {
        previousTime = 0;
        return;
      }
      const w = container.clientWidth;
      const h = container.clientHeight;
      if (!w || !h) {
        previousTime = 0;
        return;
      }
      if (options.autoRotate && timestamp - lastRender < 32) {
        invalidate();
        return;
      }
      // 약 112초에 한 바퀴 돌아가는 각속도(rad/s)다. 숨긴 탭의 경과 시간은 버리고,
      // 드래그 중에는 자동 회전을 멈춰 직접 보고 있는 방향이 밀리지 않게 한다.
      if (options.autoRotate && previousTime && !drag) {
        theta += Math.min((timestamp - previousTime) / 1000, 0.1) * 0.056;
      }
      previousTime = timestamp;
      lastRender = timestamp;
      // 크기가 바뀐 경우에만 렌더러 버퍼를 다시 잡고 카메라 종횡비를 맞춘다.
      const size = renderer.getSize(new THREE.Vector2());
      if (size.x !== w || size.y !== h) renderer.setSize(w, h, false);
      camera.aspect = w / h;
      // HMI 세로 카드에서는 회전한 정사각 받침의 대각선까지 들어오도록 거리를 확보한다.
      // 고객 UI는 기존 거리를 유지하며 원래 화면 크기·시점은 바꾸지 않는다.
      const viewingRadius = options.autoRotate
        ? Math.max(radius, 7.9 / camera.aspect)
        : radius;
      // 공통 치수에서 계산한 중심 높이를 기준으로 카메라를 돌린다.
      camera.position.set(
        Math.sin(theta) * Math.sin(phi) * viewingRadius,
        MODEL_SIZE.focalHeight + Math.cos(phi) * viewingRadius,
        Math.cos(theta) * Math.sin(phi) * viewingRadius,
      );
      camera.lookAt(0, MODEL_SIZE.focalHeight, 0);
      camera.updateProjectionMatrix();
      renderer.render(scene, camera);
      if (options.autoRotate) invalidate();
    }
    /** 변화가 있을 때만 다음 프레임을 예약한다. 같은 프레임의 중복 요청은 합친다. */
    function invalidate() {
      if (!disposed && frame === null) frame = requestAnimationFrame(draw);
    }
    /** 전체·윗면·옆면 버튼이 지정한 시점으로 복귀한다. 극점은 조금 비켜 둔다. */
    function view(kind) {
      theta = 0;
      phi = kind === "top" ? 0.02 : kind === "side" ? 1.48 : 0.96;
      invalidate();
    }
    // 드래그를 시작한 포인터만 추적하고 컨테이너를 벗어나도 회전을 이어간다.
    container.addEventListener("pointerdown", (e) => {
      if (drag) return;
      drag = { id: e.pointerId, x: e.clientX, y: e.clientY };
      container.setPointerCapture(e.pointerId);
    });
    container.addEventListener("pointermove", (e) => {
      if (!drag || drag.id !== e.pointerId) return;
      // 가로 이동은 회전, 세로 이동은 기울기로 바꾸며 바닥 아래 시점은 제한한다.
      theta -= (e.clientX - drag.x) * 0.012;
      phi = Math.max(0.02, Math.min(1.53, phi - (e.clientY - drag.y) * 0.009));
      drag = { id: e.pointerId, x: e.clientX, y: e.clientY };
      invalidate();
    });
    for (const type of ["pointerup", "pointercancel", "lostpointercapture"])
      container.addEventListener(type, (e) => {
        if (drag?.id === e.pointerId) drag = null;
      });
    // 브라우저가 GPU 컨텍스트를 회수하면 상태를 기록하고 복원 시 텍스처를 다시 올린다.
    renderer.domElement.addEventListener("webglcontextlost", (e) => {
      e.preventDefault();
      container.dataset.ready = "lost";
    });
    renderer.domElement.addEventListener("webglcontextrestored", () => {
      container.dataset.ready = "true";
      update();
    });
    /** 원본 획의 표시용 크림만 갱신한다. 인자가 없으면 현재 획을 유지해 복원에 사용한다. */
    function update(drawings) {
      if (cream && drawings) {
        cream.update(drawings);
        container.dataset.creamMeshes = String(cream.group.children.length);
      }
      topTexture.needsUpdate = true;
      sideTexture.needsUpdate = true;
      invalidate();
    }
    // 반응형 배치나 확인 창 열기로 영역 크기가 바뀌면 새 크기로 렌더링한다.
    const observer = new ResizeObserver(invalidate);
    observer.observe(container);
    container.dataset.ready = "true";
    /** 숨긴 HMI는 프레임을 멈추고 다시 보일 때 현재 시점에서 회전을 재개한다. */
    function onVisibility() {
      previousTime = 0;
      invalidate();
    }
    if (options.autoRotate)
      document.addEventListener("visibilitychange", onVisibility);
    invalidate();
    return {
      update,
      view,
      resize: invalidate,
      /** 관찰·프레임 예약을 멈추고 장면의 기하·재질·텍스처와 WebGL 자원을 해제한다. */
      dispose() {
        disposed = true;
        if (options.autoRotate)
          document.removeEventListener("visibilitychange", onVisibility);
        observer.disconnect();
        if (frame !== null) cancelAnimationFrame(frame);
        cream?.dispose();
        // 크림 모드에서는 텍스처가 재질에 없으므로 별도로 해제한다.
        if (cream) {
          topTexture.dispose();
          sideTexture.dispose();
        }
        key.shadow.dispose();
        scene.traverse((object) => {
          object.geometry?.dispose();
          // 단일 재질과 재질 배열을 같은 순회로 처리해 장면 구성에 관계없이 정리한다.
          const materials = object.material
            ? Array.isArray(object.material)
              ? object.material
              : [object.material]
            : [];
          for (const material of materials) {
            material.map?.dispose();
            material.dispose();
          }
        });
        renderer.dispose();
        renderer.domElement.remove();
      },
    };
    // WebGL 초기화가 불가능해도 필기 화면은 유지한다. 호출부는 같은 인터페이스를 쓴다.
  } catch (error) {
    container.dataset.ready = "false";
    const note = document.createElement("p");
    note.className = "model-error";
    note.textContent =
      "이 브라우저에서는 3D를 표시할 수 없습니다. Chrome을 업데이트하거나 다른 기기에서 확인해주세요. 그림 입력은 계속 사용할 수 있습니다.";
    container.appendChild(note);
    return {
      // 대체 화면에는 GPU 자원이 없으므로 갱신·시점·크기 요청은 안전하게 무시한다.
      update() {},
      view() {},
      resize() {},
      dispose() {
        note.remove();
      },
    };
  }
}
