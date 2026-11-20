import * as THREE from "three";
import {
  CAKE_DIMENSIONS,
  DRAWING_LAYOUT,
  MODEL_SIZE,
} from "./cakeDimensions.ts";

// 표시용 크림의 지름은 시안의 3.5mm다. 로봇 경로·필기 원본의 굵기는 바꾸지 않는다.
const CREAM_RADIUS = 3.5 / CAKE_DIMENSIONS.topDiameterMm;

/**
 * 원본 캔버스 좌표를 크림 중심의 모형 좌표로 변환한다.
 * 옆면 x는 기존 원뿔대 텍스처와 같은 뒤쪽 이음매에서 시작하고 y는 아래로 증가한다.
 */
export function creamSurfacePoint(point, surface) {
  if (surface === "top") {
    return new THREE.Vector3(
      ((point.x - DRAWING_LAYOUT.top.centerX) / DRAWING_LAYOUT.top.radius) *
        MODEL_SIZE.cakeRadius,
      MODEL_SIZE.cakeTop + CREAM_RADIUS * 0.58,
      ((point.y - DRAWING_LAYOUT.top.centerY) / DRAWING_LAYOUT.top.radius) *
        MODEL_SIZE.cakeRadius,
    );
  }
  const region = DRAWING_LAYOUT.side;
  const height = THREE.MathUtils.clamp(
    1 - (point.y - region.y) / region.height,
    0,
    1,
  );
  const angle = ((point.x - region.x) / region.width) * Math.PI * 2 + Math.PI;
  const radius =
    THREE.MathUtils.lerp(
      MODEL_SIZE.cakeBottomRadius,
      MODEL_SIZE.cakeRadius,
      height,
    ) +
    CREAM_RADIUS * 0.58;
  return new THREE.Vector3(
    Math.sin(angle) * radius,
    MODEL_SIZE.cakeBottom + MODEL_SIZE.cakeHeight * height,
    Math.cos(angle) * radius,
  );
}

/**
 * 한 획을 부드러운 크림 튜브로 만든다. 반복 점을 줄이고 분할 수를 제한해 태블릿을 보호한다.
 * 입력은 읽기만 하며, 반환한 기하 자원은 획 교체·삭제 또는 화면 종료 때 해제해야 한다.
 */
function strokeGeometry(points, surface) {
  const centers = [];
  for (let index = 0; index < points.length; index += 1) {
    const point = points[index];
    if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) continue;
    const next = creamSurfacePoint(point, surface);
    if (
      !centers.length ||
      next.distanceTo(centers.at(-1)) > 0.012 ||
      index === points.length - 1
    ) {
      centers.push(next);
    }
  }
  if (centers.length < 2) return null;
  const curve = new THREE.CatmullRomCurve3(centers, false, "centripetal");
  const length = curve.getLength();
  if (length < 0.0001) return null;
  const segments = Math.min(1000, Math.max(12, Math.ceil(length / 0.014)));
  const sides = 16;
  const frames = curve.computeFrenetFrames(segments, false);
  const vertices = [];
  const indices = [];
  for (let step = 0; step <= segments; step += 1) {
    const t = step / segments;
    const center = curve.getPointAt(t);
    if (surface === "side") {
      // 3D 보간의 현이 케이크 내부를 가로지르지 않도록 중심을 원뿔대 표면에 되돌린다.
      const height = THREE.MathUtils.clamp(
        (center.y - MODEL_SIZE.cakeBottom) / MODEL_SIZE.cakeHeight,
        0,
        1,
      );
      const radius =
        THREE.MathUtils.lerp(
          MODEL_SIZE.cakeBottomRadius,
          MODEL_SIZE.cakeRadius,
          height,
        ) +
        CREAM_RADIUS * 0.58;
      const scale = radius / Math.max(Math.hypot(center.x, center.z), 0.0001);
      center.x *= scale;
      center.z *= scale;
      center.y = MODEL_SIZE.cakeBottom + height * MODEL_SIZE.cakeHeight;
    }
    // 끝은 둥글게 모으고, 길이 방향의 약한 부풀음과 여덟 갈래 주름으로 크림 질감을 낸다.
    const endDistance = Math.min(t, 1 - t) * length;
    const taper = Math.sqrt(Math.min(1, endDistance / CREAM_RADIUS + 0.02));
    const puff = 1 + 0.075 * Math.sin(((t * length) / 0.11) * Math.PI * 2);
    for (let side = 0; side <= sides; side += 1) {
      const angle = (side / sides) * Math.PI * 2;
      const ridge = 0.94 + 0.065 * Math.cos(angle * 8);
      const radius = CREAM_RADIUS * puff * ridge * taper;
      const offset = frames.normals[step]
        .clone()
        .multiplyScalar(Math.cos(angle) * radius)
        .addScaledVector(frames.binormals[step], Math.sin(angle) * radius);
      vertices.push(
        center.x + offset.x,
        center.y + offset.y,
        center.z + offset.z,
      );
      if (step < segments && side < sides) {
        const a = step * (sides + 1) + side;
        const b = a + sides + 1;
        indices.push(a, a + 1, b, b, a + 1, b + 1);
      }
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(vertices, 3),
  );
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

/**
 * 한 미리보기의 크림 획과 GPU 자원을 관리한다. update는 면별 원본 획 배열을 받는다.
 * 동일한 획은 재사용하고 현재 그리는 획만 교체한다. undo·초기화도 같은 경로로 반영한다.
 */
export function createCreamLayer() {
  const group = new THREE.Group();
  const material = new THREE.MeshPhysicalMaterial({
    color: 0xfffdf5,
    roughness: 0.58,
    clearcoat: 0.12,
    clearcoatRoughness: 0.9,
  });
  const entries = new Map();
  return {
    group,
    update(drawings) {
      const retained = new Set();
      for (const surface of ["top", "side"]) {
        (drawings[surface] ?? []).forEach((points, index) => {
          const key = `${surface}:${index}`;
          const signature = JSON.stringify(points);
          retained.add(key);
          const previous = entries.get(key);
          if (previous?.signature === signature) return;
          if (previous?.mesh) {
            group.remove(previous.mesh);
            previous.mesh.geometry.dispose();
          }
          const geometry = strokeGeometry(points, surface);
          const mesh = geometry ? new THREE.Mesh(geometry, material) : null;
          if (mesh) {
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            group.add(mesh);
          }
          entries.set(key, { signature, mesh });
        });
      }
      for (const [key, entry] of entries) {
        if (retained.has(key)) continue;
        if (entry.mesh) {
          group.remove(entry.mesh);
          entry.mesh.geometry.dispose();
        }
        entries.delete(key);
      }
    },
    /** 장면에서 분리한 뒤 모든 획과 공유 재질을 정확히 한 번 정리한다. */
    dispose() {
      group.removeFromParent();
      for (const entry of entries.values()) entry.mesh?.geometry.dispose();
      entries.clear();
      group.clear();
      material.dispose();
    },
  };
}
