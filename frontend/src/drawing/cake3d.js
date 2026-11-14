import * as THREE from "three";
export function cakeView(container, textures) {
  try {
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(34, 1, 0.1, 100);
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.25));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 0.9;
    container.appendChild(renderer.domElement);
    const group = new THREE.Group();
    scene.add(group);
    scene.add(new THREE.HemisphereLight(0xffffff, 0xc8b9a1, 1.8));
    const key = new THREE.DirectionalLight(0xfff7ec, 2);
    key.position.set(-3, 6, 4);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.7);
    fill.position.set(4, 2, -3);
    scene.add(fill);
    const topTexture = new THREE.CanvasTexture(textures.top),
      sideTexture = new THREE.CanvasTexture(textures.side);
    for (const t of [topTexture, sideTexture]) {
      t.colorSpace = THREE.SRGBColorSpace;
      t.anisotropy = Math.min(4, renderer.capabilities.getMaxAnisotropy());
    }
    const side = new THREE.Mesh(
      new THREE.CylinderGeometry(1, 1, 1.0667, 96, 1, true, Math.PI),
      new THREE.MeshStandardMaterial({ map: sideTexture, roughness: 0.85 }),
    );
    side.position.y = 0.6;
    group.add(side);
    const top = new THREE.Mesh(
      new THREE.CircleGeometry(1, 96),
      new THREE.MeshStandardMaterial({ map: topTexture, roughness: 0.9 }),
    );
    top.rotation.x = -Math.PI / 2;
    top.position.y = 1.134;
    group.add(top);
    const board = new THREE.Mesh(
      new THREE.CylinderGeometry(1.16, 1.16, 0.0667, 96),
      new THREE.MeshStandardMaterial({ color: 0xe6d0a5, roughness: 0.7 }),
    );
    board.position.y = 0.033;
    group.add(board);
    const rim = new THREE.Mesh(
      new THREE.TorusGeometry(0.991, 0.017, 8, 96),
      new THREE.MeshStandardMaterial({ color: 0xfffcf1, roughness: 1 }),
    );
    rim.rotation.x = Math.PI / 2;
    rim.position.y = 1.131;
    group.add(rim);
    // 밝은 받침과 그림자가 모형의 높이·바닥 위치를 구분한다.
    const shadowCanvas = document.createElement("canvas");
    shadowCanvas.width = shadowCanvas.height = 128;
    const sx = shadowCanvas.getContext("2d"),
      grad = sx.createRadialGradient(64, 64, 5, 64, 64, 64);
    grad.addColorStop(0, "rgba(70,45,20,0.20)");
    grad.addColorStop(1, "rgba(70,45,20,0)");
    sx.fillStyle = grad;
    sx.fillRect(0, 0, 128, 128);
    const shadow = new THREE.Mesh(
      new THREE.PlaneGeometry(3.6, 3.6),
      new THREE.MeshBasicMaterial({
        map: new THREE.CanvasTexture(shadowCanvas),
        transparent: true,
        depthWrite: false,
      }),
    );
    shadow.rotation.x = -Math.PI / 2;
    shadow.position.y = -0.01;
    scene.add(shadow);
    let theta = 0,
      phi = 0.96,
      drag = null;
    const radius = 4.8;
    let frame = null;
    function draw() {
      frame = null;
      const w = container.clientWidth,
        h = container.clientHeight;
      if (!w || !h) return;
      const size = renderer.getSize(new THREE.Vector2());
      if (size.x !== w || size.y !== h) renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.position.set(
        Math.sin(theta) * Math.sin(phi) * radius,
        0.6 + Math.cos(phi) * radius,
        Math.cos(theta) * Math.sin(phi) * radius,
      );
      camera.lookAt(0, 0.6, 0);
      camera.updateProjectionMatrix();
      renderer.render(scene, camera);
    }
    function invalidate() {
      if (frame === null) frame = requestAnimationFrame(draw);
    }
    function view(kind) {
      theta = 0;
      phi = kind === "top" ? 0.02 : kind === "side" ? 1.48 : 0.96;
      invalidate();
    }
    container.addEventListener("pointerdown", (e) => {
      if (drag) return;
      drag = { id: e.pointerId, x: e.clientX, y: e.clientY };
      container.setPointerCapture(e.pointerId);
    });
    container.addEventListener("pointermove", (e) => {
      if (!drag || drag.id !== e.pointerId) return;
      theta -= (e.clientX - drag.x) * 0.012;
      phi = Math.max(0.02, Math.min(1.53, phi - (e.clientY - drag.y) * 0.009));
      drag = { id: e.pointerId, x: e.clientX, y: e.clientY };
      invalidate();
    });
    for (const type of ["pointerup", "pointercancel", "lostpointercapture"])
      container.addEventListener(type, (e) => {
        if (drag?.id === e.pointerId) drag = null;
      });
    renderer.domElement.addEventListener("webglcontextlost", (e) => {
      e.preventDefault();
      container.dataset.ready = "lost";
    });
    renderer.domElement.addEventListener("webglcontextrestored", () => {
      container.dataset.ready = "true";
      update();
    });
    function update() {
      topTexture.needsUpdate = true;
      sideTexture.needsUpdate = true;
      invalidate();
    }
    const observer = new ResizeObserver(invalidate);
    observer.observe(container);
    container.dataset.ready = "true";
    invalidate();
    return {
      update,
      view,
      resize: invalidate,
      dispose() {
        observer.disconnect();
        if (frame !== null) cancelAnimationFrame(frame);
        scene.traverse((object) => {
          object.geometry?.dispose();
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
  } catch (error) {
    container.dataset.ready = "false";
    const note = document.createElement("p");
    note.className = "model-error";
    note.textContent =
      "이 브라우저에서는 3D를 표시할 수 없습니다. Chrome을 업데이트하거나 다른 기기에서 확인해주세요. 그림 입력은 계속 사용할 수 있습니다.";
    container.appendChild(note);
    return {
      update() {},
      view() {},
      resize() {},
      dispose() {
        note.remove();
      },
    };
  }
}
