import * as THREE from "three";
/**
 * 두 면의 캔버스 텍스처로 회전 가능한 케이크를 만들고 조작·정리 함수를 반환한다.
 * 일반 미리보기와 확인 창이 각각 호출하며, 입력 그림 자체는 변경하지 않는다.
 */
export function cakeView(container, textures) {
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
    container.appendChild(renderer.domElement);
    // 본체·윗면·받침·테두리를 하나의 모형으로 구성하고 조명으로 입체감을 만든다.
    const group = new THREE.Group();
    scene.add(group);
    scene.add(new THREE.HemisphereLight(0xffffff, 0xc8b9a1, 1.8));
    const key = new THREE.DirectionalLight(0xfff7ec, 2);
    key.position.set(-3, 6, 4);
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
    // 반지름 1을 기준으로 지름 15cm·높이 8cm의 비율(높이 약 1.0667)을 표현한다.
    // 열린 원통은 옆면만 담당하고, 윗면은 별도 원판으로 올려 서로 다른 그림을 입힌다.
    const side = new THREE.Mesh(
      new THREE.CylinderGeometry(1, 1, 1.0667, 96, 1, true, Math.PI),
      new THREE.MeshStandardMaterial({ map: sideTexture, roughness: 0.85 }),
    );
    side.position.y = 0.6;
    group.add(side);
    // 원판을 수평으로 눕혀 원통 위에 배치한다. 미세한 높이 차이로 겹침을 피한다.
    const top = new THREE.Mesh(
      new THREE.CircleGeometry(1, 96),
      new THREE.MeshStandardMaterial({ map: topTexture, roughness: 0.9 }),
    );
    top.rotation.x = -Math.PI / 2;
    top.position.y = 1.134;
    group.add(top);
    // 받침과 가장자리 장식은 그림 텍스처와 분리해 케이크 윤곽을 유지한다.
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
    const sx = shadowCanvas.getContext("2d");
    const grad = sx.createRadialGradient(64, 64, 5, 64, 64, 64);
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
    // 수평 회전각과 수직 시야각을 구면 좌표로 관리하고, 카메라 거리는 고정한다.
    let theta = 0;
    let phi = 0.96;
    let drag = null;
    const radius = 4.8;
    let frame = null;
    /** 예약된 한 프레임을 그린다. 보이지 않는 컨테이너는 크기를 얻을 때까지 건너뛴다. */
    function draw() {
      frame = null;
      const w = container.clientWidth;
      const h = container.clientHeight;
      if (!w || !h) return;
      // 크기가 바뀐 경우에만 렌더러 버퍼를 다시 잡고 카메라 종횡비를 맞춘다.
      const size = renderer.getSize(new THREE.Vector2());
      if (size.x !== w || size.y !== h) renderer.setSize(w, h, false);
      camera.aspect = w / h;
      // 케이크 중심 높이 0.6을 기준으로 카메라를 돌려 모형이 화면 중앙에 남게 한다.
      camera.position.set(
        Math.sin(theta) * Math.sin(phi) * radius,
        0.6 + Math.cos(phi) * radius,
        Math.cos(theta) * Math.sin(phi) * radius,
      );
      camera.lookAt(0, 0.6, 0);
      camera.updateProjectionMatrix();
      renderer.render(scene, camera);
    }
    /** 변화가 있을 때만 다음 프레임을 예약한다. 같은 프레임의 중복 요청은 합친다. */
    function invalidate() {
      if (frame === null) frame = requestAnimationFrame(draw);
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
    /** 원본 캔버스의 변경을 GPU 업로드 대상으로 표시하고 한 번 다시 그린다. */
    function update() {
      topTexture.needsUpdate = true;
      sideTexture.needsUpdate = true;
      invalidate();
    }
    // 반응형 배치나 확인 창 열기로 영역 크기가 바뀌면 새 크기로 렌더링한다.
    const observer = new ResizeObserver(invalidate);
    observer.observe(container);
    container.dataset.ready = "true";
    invalidate();
    return {
      update,
      view,
      resize: invalidate,
      /** 관찰·프레임 예약을 멈추고 장면의 기하·재질·텍스처와 WebGL 자원을 해제한다. */
      dispose() {
        observer.disconnect();
        if (frame !== null) cancelAnimationFrame(frame);
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
