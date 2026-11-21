// Visual models for everything solid in the world. Dimensions mirror the physics colliders.
import * as THREE from 'three';
import { ConvexGeometry } from 'three/examples/jsm/geometries/ConvexGeometry.js';
import { rockShapes, rockTransform } from '../sim/rockShapes.js';
import { WORLD, DOCK, CARGO_TYPES, DECK } from '../sim/config.js';
import { acmGeom } from '../sim/cargoContact.js';
import { heightAt, mulberry32 } from '../sim/terrain.js';
import { LANDER_SCALE } from '../sim/world.js';
import { cargoModuleCAD } from './cadModels.js';
import { mergeStatic } from './mergeStatic.js';

export const MAT = {
  white: new THREE.MeshStandardMaterial({ color: 0xd2d2cc, roughness: 0.6, metalness: 0.05 }),
  offwhite: new THREE.MeshStandardMaterial({ color: 0xbdbdb6, roughness: 0.7, metalness: 0.05 }),
  steel: new THREE.MeshStandardMaterial({ color: 0x8c9094, roughness: 0.45, metalness: 0.75 }),
  dark: new THREE.MeshStandardMaterial({ color: 0x24272b, roughness: 0.6, metalness: 0.4 }),
  black: new THREE.MeshStandardMaterial({ color: 0x0d0e10, roughness: 0.8, metalness: 0.1 }),
  gold: null, foil: null,
  rubber: new THREE.MeshStandardMaterial({ color: 0x1b1b1b, roughness: 0.95 }),
  glassBlue: new THREE.MeshStandardMaterial({ color: 0x0e1a38, roughness: 0.25, metalness: 0.6 }),
  orange: new THREE.MeshStandardMaterial({ color: 0xd9661f, roughness: 0.6 }),
  red: new THREE.MeshStandardMaterial({ color: 0xb3261e, roughness: 0.5 }),
  amberGlow: new THREE.MeshStandardMaterial({ color: 0x331a00, emissive: 0xffa31a, emissiveIntensity: 2.5 }),
  greenGlow: new THREE.MeshStandardMaterial({ color: 0x002a10, emissive: 0x33ff88, emissiveIntensity: 2.0 }),
  whiteGlow: new THREE.MeshStandardMaterial({ color: 0x222222, emissive: 0xffffff, emissiveIntensity: 3.0 }),
};

// Multi-layer insulation: crinkled aluminised Kapton. Albedo + a crinkle normal map.
function mliTextures(tint, seed) {
  const N = 256, rand = mulberry32(seed);
  const h = new Float32Array(N * N);
  for (let k = 0; k < 420; k++) { // facets from random folds
    const cx = rand() * N, cy = rand() * N, a = rand() * Math.PI, len = 10 + rand() * 70, w = 2 + rand() * 7, amp = (rand() - 0.5) * 2;
    const ca = Math.cos(a), sa = Math.sin(a);
    for (let y = Math.max(0, Math.floor(cy - len)); y < Math.min(N, cy + len); y++) for (let x = Math.max(0, Math.floor(cx - len)); x < Math.min(N, cx + len); x++) {
      const dx = x - cx, dy = y - cy, u = dx * ca + dy * sa, v = -dx * sa + dy * ca;
      if (Math.abs(u) < len && Math.abs(v) < w) h[y * N + x] += amp * (1 - Math.abs(v) / w) * (1 - Math.abs(u) / len);
    }
  }
  const mk = () => { const c = document.createElement('canvas'); c.width = c.height = N; return c; };
  const ca = mk(), cn = mk(), ga = ca.getContext('2d'), gn = cn.getContext('2d');
  const ia = ga.createImageData(N, N), inn = gn.createImageData(N, N);
  const base = new THREE.Color(tint);
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
    const i = y * N + x, hx = h[y * N + ((x + 1) % N)] - h[i], hy = h[((y + 1) % N) * N + x] - h[i];
    let nx = -hx * 2.2, ny = -hy * 2.2, nz = 1; const L = Math.hypot(nx, ny, nz); nx /= L; ny /= L; nz /= L;
    const k = i * 4; inn.data[k] = (nx * 0.5 + 0.5) * 255; inn.data[k + 1] = (ny * 0.5 + 0.5) * 255; inn.data[k + 2] = nz * 255; inn.data[k + 3] = 255;
    const b = 0.82 + 0.18 * Math.tanh(h[i] * 0.8) + (rand() < 0.003 ? 0.15 : 0);
    ia.data[k] = Math.min(255, base.r * 255 * b); ia.data[k + 1] = Math.min(255, base.g * 255 * b); ia.data[k + 2] = Math.min(255, base.b * 255 * b); ia.data[k + 3] = 255;
  }
  ga.putImageData(ia, 0, 0); gn.putImageData(inn, 0, 0);
  const map = new THREE.CanvasTexture(ca); map.colorSpace = THREE.SRGBColorSpace; map.wrapS = map.wrapT = THREE.RepeatWrapping;
  const nrm = new THREE.CanvasTexture(cn); nrm.wrapS = nrm.wrapT = THREE.RepeatWrapping;
  return { map, nrm };
}
{ const g = mliTextures(0xd9a63a, 3), f = mliTextures(0xc4c7cc, 5);
  MAT.gold = new THREE.MeshStandardMaterial({ color: 0xffffff, map: g.map, normalMap: g.nrm, normalScale: new THREE.Vector2(1.2, 1.2), roughness: 0.32, metalness: 0.95 });
  MAT.foil = new THREE.MeshStandardMaterial({ color: 0xffffff, map: f.map, normalMap: f.nrm, normalScale: new THREE.Vector2(1.2, 1.2), roughness: 0.28, metalness: 0.95 }); }

export function labelTexture(text, opts = {}) {
  const c = document.createElement('canvas'); c.width = opts.w || 512; c.height = opts.h || 128;
  const g = c.getContext('2d');
  g.fillStyle = opts.bg || 'rgba(0,0,0,0)'; g.fillRect(0, 0, c.width, c.height);
  g.fillStyle = opts.color || '#1a1a1a';
  let size = opts.size || 64;
  const font = () => `${opts.weight || 700} ${size}px "Helvetica Neue", Arial, sans-serif`;
  g.font = font();
  while (size > 10 && g.measureText(text).width > c.width * 0.94) { size -= 2; g.font = font(); } // shrink to fit
  g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(text, c.width / 2, c.height / 2);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; return t;
}

// AprilTag-like fiducial (deterministic 6x6 data bits inside a black border)
export function fiducialTexture(id) {
  const c = document.createElement('canvas'); c.width = c.height = 160; const g = c.getContext('2d');
  g.fillStyle = '#fff'; g.fillRect(0, 0, 160, 160); g.fillStyle = '#000'; g.fillRect(10, 10, 140, 140);
  const rand = mulberry32(id * 7919 + 13);
  for (let y = 0; y < 6; y++) for (let x = 0; x < 6; x++) if (rand() < 0.5) { g.fillStyle = '#fff'; g.fillRect(30 + x * 16.7, 30 + y * 16.7, 16.7, 16.7); }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.magFilter = THREE.NearestFilter; return t;
}
export function tagMesh(id, size) { return new THREE.Mesh(new THREE.PlaneGeometry(size, size), new THREE.MeshStandardMaterial({ map: fiducialTexture(id), roughness: 0.8 })); }

const box = (w, h, d, m) => { const me = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m); me.castShadow = me.receiveShadow = true; return me; };
const cyl = (r1, r2, h, m, seg = 32) => { const me = new THREE.Mesh(new THREE.CylinderGeometry(r1, r2, h, seg), m); me.castShadow = me.receiveShadow = true; return me; };

// ---------------- rocks ------------------------------------------------------
export class RockField {
  constructor(scene, terrain) {
    this.t = terrain; this.scene = scene;
    const shapes = rockShapes();
    this.geoms = shapes.map((pts) => {
      const v = []; for (let i = 0; i < pts.length; i += 3) v.push(new THREE.Vector3(pts[i], pts[i + 1], pts[i + 2]));
      const g = new ConvexGeometry(v); g.computeVertexNormals();
      // ConvexGeometry has no uv attribute, so the rock texture sampled a single texel (flat grey rocks).
      // Spherical mapping around the hull centre, two repeats around.
      const P = g.attributes.position, uv = new Float32Array(P.count * 2);
      for (let i = 0; i < P.count; i++) { const x = P.getX(i), y = P.getY(i), z = P.getZ(i), L = Math.hypot(x, y, z) || 1;
        uv[i * 2] = (Math.atan2(z, x) / (2 * Math.PI) + 0.5) * 2; uv[i * 2 + 1] = Math.acos(Math.max(-1, Math.min(1, y / L))) / Math.PI * 1.5; }
      g.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
      return g;
    });
    const tex = rockTexture();
    this.mat = new THREE.MeshStandardMaterial({ color: 0xffffff, map: tex, roughness: 0.92, metalness: 0, flatShading: false });
    this.meshes = [];
    this.rebuild();
  }
  rebuild() {
    for (const m of this.meshes) { this.scene.remove(m); m.dispose?.(); }
    this.meshes = [];
    const by = [[], [], [], [], [], []];
    for (const r of this.t.rocks) if (!r.skip) by[r.shape].push(r);
    const M = new THREE.Matrix4(), q = new THREE.Quaternion(), qt = new THREE.Quaternion(), s = new THREE.Vector3(), p = new THREE.Vector3(), c = new THREE.Color();
    const rand = mulberry32(5);
    by.forEach((list, k) => {
      const im = new THREE.InstancedMesh(this.geoms[k], this.mat, list.length + 64);
      im.castShadow = true; im.receiveShadow = true;
      list.forEach((r, i) => {
        const tr = rockTransform(r);
        q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), tr.yaw); qt.setFromAxisAngle(new THREE.Vector3(1, 0, 0), tr.tilt); q.multiply(qt);
        M.compose(p.set(...tr.pos), q, s.set(...tr.scale));
        im.setMatrixAt(i, M);
        const b = 0.55 + rand() * 0.25 + (r.injected ? 0.1 : 0);
        im.setColorAt(i, c.setRGB(b, b * 0.98, b * 0.95));
      });
      im.count = list.length;
      im.instanceMatrix.needsUpdate = true; if (im.instanceColor) im.instanceColor.needsUpdate = true;
      im.computeBoundingSphere();
      this.scene.add(im); this.meshes.push(im);
    });
  }
}
function rockTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 256; const g = c.getContext('2d'); const rand = mulberry32(21);
  g.fillStyle = '#7d7a75'; g.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 4000; i++) { const v = 90 + rand() * 90; g.fillStyle = `rgba(${v},${v * 0.98},${v * 0.95},${0.25 + rand() * 0.4})`; const s = 1 + rand() * 3; g.fillRect(rand() * 256, rand() * 256, s, s); }
  for (let i = 0; i < 25; i++) { g.strokeStyle = `rgba(40,38,35,${0.2 + rand() * 0.3})`; g.beginPath(); g.moveTo(rand() * 256, rand() * 256); g.lineTo(rand() * 256, rand() * 256); g.stroke(); }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.wrapS = t.wrapT = THREE.RepeatWrapping; t.anisotropy = 4; return t;
}

// ---------------- lander -----------------------------------------------------
export function buildLander(terrain) {
  // Built in model units and scaled by LANDER_SCALE, the same factor the physics colliders and the camera's
  // surveyed tags use (sim/world.js). Unscaled, the drawn lander was 2.6x smaller than its colliders.
  const outer = new THREE.Group();
  const g = new THREE.Group(); g.scale.setScalar(LANDER_SCALE); outer.add(g);
  const P = WORLD.pad, y0 = terrain.padH;
  outer.position.set(P.x, y0, P.z);
  const deck = cyl(4.6, 4.6, 2.6, MAT.gold, 40); deck.position.y = 2.6; g.add(deck);        // descent stage
  for (let k = 0; k < 4; k++) { // surveyed navigation fiducials (same placement as the sensor model)
    const a = k * Math.PI / 2, t = tagMesh(200 + k, 1.2);
    t.position.set(Math.cos(a) * 4.63, 3.2, Math.sin(a) * 4.63); t.rotation.y = Math.PI / 2 - a; g.add(t);
  }
  const body = cyl(3.4, 3.4, 9.2, MAT.white, 40); body.position.y = 2.6 + 1.3 + 4.6 - 1.3; g.add(body);
  const nose = new THREE.Mesh(new THREE.ConeGeometry(3.4, 3.2, 40), MAT.white); nose.position.y = 2.6 + 1.3 + 9.2 - 1.3 + 1.6; nose.castShadow = true; g.add(nose);
  for (let k = 0; k < 4; k++) {
    const a = Math.PI / 4 + k * Math.PI / 2;
    const foot = cyl(0.75, 0.9, 0.24, MAT.steel, 16); foot.position.set(Math.cos(a) * 6.6, 0.12, Math.sin(a) * 6.6); g.add(foot);
    const leg = cyl(0.17, 0.17, 4.4, MAT.steel, 10);
    leg.position.set(Math.cos(a) * 5.4, 1.75, Math.sin(a) * 5.4);
    leg.rotation.set(0, 0, 0); leg.lookAt(new THREE.Vector3(Math.cos(a) * 3.8, 3.6, Math.sin(a) * 3.8).add(new THREE.Vector3(0, 0, 0))); leg.rotateX(Math.PI / 2);
    g.add(leg);
    const tank = new THREE.Mesh(new THREE.SphereGeometry(1.1, 20, 14), MAT.foil); tank.position.set(Math.cos(a + Math.PI / 4) * 3.6, 2.9, Math.sin(a + Math.PI / 4) * 3.6); tank.castShadow = true; g.add(tank);
  }
  for (let k = 0; k < 3; k++) { const n = cyl(0.5, 1.0, 1.4, MAT.dark, 20); n.position.set((k - 1) * 1.6, 0.95, 0); g.add(n); }
  // offload crane boom
  const boom = box(0.4, 0.4, 9, MAT.offwhite); boom.position.set(0, 12.5, 4.2); boom.rotation.x = -0.35; g.add(boom);
  // landing-pad edge beacons (small, flush-mounted)
  for (let k = 0; k < 16; k++) { const a = k / 16 * Math.PI * 2; const bcn = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.15, 0.12, 10), new THREE.MeshStandardMaterial({ color: 0x302000, emissive: k % 2 ? 0xffb020 : 0xffffff, emissiveIntensity: 2.5 })); bcn.position.set(Math.cos(a) * WORLD.pad.radius, 0.06, Math.sin(a) * WORLD.pad.radius); outer.add(bcn); }
  const lbl = new THREE.Mesh(new THREE.PlaneGeometry(4, 1), new THREE.MeshBasicMaterial({ map: labelTexture('CARGO LANDER CL-1', { color: '#202020' }), transparent: true }));
  lbl.position.set(0, 9, 3.42); g.add(lbl);
  // Surface detail (visual only, at most 0.2 model units outside the body / deck colliders): ring frames, stringers,
  // an interstage band, the cargo-bay door the crane serves, RCS quads, a second strut pair on each leg, antennas.
  {
    const at = (m, x, y, z) => { m.position.set(x, y, z); g.add(m); return m; };
    const yB0 = 3.9, yB1 = 13.1;                                  // body cylinder: bottom / top
    for (const y of [yB0 + 0.15, 6.9, 10.0, yB1 - 0.15]) at(cyl(3.46, 3.46, 0.22, MAT.steel, 48), 0, y, 0);
    at(cyl(3.75, 4.3, 0.5, MAT.dark, 40), 0, yB0 - 0.05, 0);     // interstage adapter on the descent stage
    for (let k = 0; k < 12; k++) { const a = k / 12 * Math.PI * 2 + Math.PI / 12; const st = at(box(0.1, yB1 - yB0, 0.14, MAT.offwhite), Math.cos(a) * 3.43, (yB0 + yB1) / 2, Math.sin(a) * 3.43); st.rotation.y = -a; }
    // cargo-bay door on the crane (+z) side: frame, two leaves, hinges
    for (const [w, h, x, y] of [[3.3, 0.14, 0, 8.15], [3.3, 0.14, 0, 4.65], [0.14, 3.6, -1.62, 6.4], [0.14, 3.6, 1.62, 6.4], [0.08, 3.4, 0, 6.4]]) { const f = at(box(w, h, 0.12, MAT.dark), x, y, 3.3 + (x ? 0.02 : 0.12)); if (x) f.position.z = Math.sqrt(3.4 * 3.4 - x * x) + 0.03; }
    for (const y of [5.2, 6.4, 7.6]) for (const x of [-1.72, 1.72]) at(box(0.16, 0.3, 0.16, MAT.steel), x, y, Math.sqrt(3.4 * 3.4 - x * x) + 0.06);
    for (let k = 0; k < 4; k++) {                                 // RCS quads near the top of the body
      const a = k * Math.PI / 2 + Math.PI / 4, ca = Math.cos(a), sa = Math.sin(a);
      const pod = at(box(0.5, 0.6, 0.6, MAT.dark), ca * 3.6, 11.9, sa * 3.6); pod.rotation.y = -a;
      for (const [dy, tilt] of [[0.42, 0], [-0.42, Math.PI]]) { const n = at(cyl(0.1, 0.2, 0.3, MAT.steel, 12), ca * 3.72, 11.9 + dy, sa * 3.72); n.rotation.x = tilt; }
      const side = at(cyl(0.1, 0.2, 0.3, MAT.steel, 12), ca * 3.98, 11.9, sa * 3.98); side.rotation.z = Math.PI / 2; side.rotation.y = -a + Math.PI;
    }
    for (let k = 0; k < 4; k++) {                                 // side struts: footpad up to the deck, either side of the main leg
      const a = Math.PI / 4 + k * Math.PI / 2;
      for (const da of [-0.42, 0.42]) {
        const p0 = new THREE.Vector3(Math.cos(a) * 6.5, 0.3, Math.sin(a) * 6.5), p1 = new THREE.Vector3(Math.cos(a + da) * 4.55, 2.2, Math.sin(a + da) * 4.55);
        const s2 = cyl(0.09, 0.09, p0.distanceTo(p1), MAT.steel, 8); s2.position.copy(p0).add(p1).multiplyScalar(0.5);
        s2.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), p1.clone().sub(p0).normalize()); g.add(s2);
      }
    }
    at(cyl(0.05, 0.07, 2.4, MAT.steel, 8), 0.9, 17.2, -0.6);     // low-gain whip
    const hg = new THREE.Mesh(new THREE.SphereGeometry(0.9, 24, 8, 0, Math.PI * 2, 0, Math.PI / 3.2), new THREE.MeshStandardMaterial({ color: 0xe6e6e0, roughness: 0.5, metalness: 0.2, side: THREE.DoubleSide }));
    hg.rotation.set(Math.PI - 0.6, 0, 0.4); hg.castShadow = true; at(hg, -3.9, 13.6, -1.2); at(cyl(0.06, 0.06, 1.2, MAT.steel, 8), -3.65, 12.9, -1.1);
  }
  mergeStatic(g);
  return outer;
}

// ---------------- supply building -------------------------------------------
export function buildHabitat(terrain, ports) {
  const H = terrain.hab, y0 = terrain.habH;
  const root = new THREE.Group();
  const b = H.building;
  const bld = new THREE.Group(); bld.position.set(b.x, y0, b.z); bld.rotation.y = H.yaw; root.add(bld);
  const shell = box(b.w, b.h, b.d, MAT.offwhite); shell.position.y = b.h / 2; bld.add(shell);
  const roof = box(b.w + 0.4, 0.3, b.d + 0.4, MAT.foil); roof.position.y = b.h + 0.15; bld.add(roof);
  // regolith sandbag berm (radiation / micrometeoroid shielding), matches berm colliders
  const bagMat = new THREE.MeshStandardMaterial({ color: 0x7b766c, roughness: 1 });
  const berm = (w, h, d, x, z) => { const m = box(w, h, d, bagMat); m.position.set(x, h / 2, z); bld.add(m); };
  berm(2.4, 3.2, b.d + 3, -b.w / 2 - 1.2, 0);
  berm(b.w + 2, 3.2, 2.4, -1, b.d / 2 + 1.2); berm(b.w + 2, 3.2, 2.4, -1, -b.d / 2 - 1.2);
  const sign = new THREE.Mesh(new THREE.PlaneGeometry(6.4, 1.0), new THREE.MeshBasicMaterial({ map: labelTexture('SUPPLY BUILDING  SB-1', { color: '#151515' }), transparent: true }));
  sign.position.set(b.w / 2 + 0.14, b.h - 0.9, 5.0); sign.rotation.y = Math.PI / 2; bld.add(sign);   // between two port vestibules, above the seam
  // Shell detail (visual only, within 0.12 m of the wall collider): a rigid-panel pressure shell reads as frames,
  // panel seams and a plinth, not a blank box. Ribs are left off the docking face where a port vestibule stands.
  {
    const ribMat = new THREE.MeshStandardMaterial({ color: 0x9ea3a6, roughness: 0.5, metalness: 0.55 });
    const seamMat = new THREE.MeshStandardMaterial({ color: 0x74787b, roughness: 0.7, metalness: 0.3 });
    const put = (w, h, d, m, x, y, z) => { const me = box(w, h, d, m); me.position.set(x, y, z); bld.add(me); return me; };
    const cy = Math.cos(H.yaw), sy = Math.sin(H.yaw);
    const portZ = ports.map((p) => (p.x - b.x) * sy + (p.z - b.z) * cy);
    const clearOfPort = (z) => portZ.every((pz) => Math.abs(z - pz) > DOCK.vestibuleW / 2 + 0.6);
    const nz = 8, nx = 4;
    for (let i = 0; i <= nz; i++) { const z = -b.d / 2 + i * b.d / nz;
      if (clearOfPort(z)) put(0.2, b.h, 0.36, ribMat, b.w / 2 + 0.02, b.h / 2, z);
      put(0.2, b.h, 0.36, ribMat, -b.w / 2 - 0.02, b.h / 2, z); }
    for (let i = 0; i <= nx; i++) { const x = -b.w / 2 + i * b.w / nx;
      for (const sz of [-1, 1]) put(0.36, b.h, 0.2, ribMat, x, b.h / 2, sz * (b.d / 2 + 0.02)); }
    for (const y of [b.h / 3, 2 * b.h / 3]) { put(b.w + 0.1, 0.1, b.d + 0.1, seamMat, 0, y, 0); }
    put(b.w + 0.24, 0.5, b.d + 0.24, seamMat, 0, 0.25, 0);            // plinth
    put(b.w + 0.5, 0.35, b.d + 0.5, ribMat, 0, b.h + 0.42, 0).scale.set(1, 1, 1); // parapet cap over the foil roof edge
    const roofTop = box(b.w - 0.6, 0.3, b.d - 0.6, MAT.foil); roofTop.position.y = b.h + 0.5; bld.add(roofTop);
    // roof equipment: three body-mounted radiators, an equipment hatch and a high-gain dish (Earth link)
    for (const z of [-14, 0, 14]) { const r = put(7, 0.12, 9, MAT.white, -3, b.h + 1.25, z); r.rotation.z = 0.12;
      for (const sx of [-1, 1]) put(0.15, 0.9, 0.15, ribMat, -3 + sx * 3, b.h + 0.95, z); }
    put(2.6, 0.5, 2.6, seamMat, 6, b.h + 0.9, 18);
    const mast = cyl(0.12, 0.16, 3.2, MAT.steel, 12); mast.position.set(6.5, b.h + 2.2, -18); bld.add(mast);
    const dish = new THREE.Mesh(new THREE.SphereGeometry(1.6, 28, 10, 0, Math.PI * 2, 0, Math.PI / 3.2), new THREE.MeshStandardMaterial({ color: 0xe6e6e0, roughness: 0.5, metalness: 0.2, side: THREE.DoubleSide }));
    dish.position.set(6.5, b.h + 5.2, -18); dish.rotation.set(Math.PI - 0.5, 0, 0.3); dish.castShadow = true; bld.add(dish);
  }
  // floodlights over the docking face
  for (const s of [-12, 0, 12]) { const l = box(0.3, 0.25, 0.8, MAT.whiteGlow); l.position.set(b.w / 2 + 0.3, b.h - 0.3, s); bld.add(l); }

  const portVis = [];
  for (const p of ports) {
    const g = new THREE.Group();
    g.position.set((p.x + p.wallX) / 2, y0, (p.z + p.wallZ) / 2); g.rotation.y = H.yaw;
    const vest = box(DOCK.vestibuleLen, DOCK.vestibuleH, DOCK.vestibuleW, MAT.white); vest.position.y = DOCK.vestibuleH / 2; g.add(vest);
    // face plate + dust-exclusion coupling (bellows ring) around the 1.8 m opening
    const face = new THREE.Group(); face.position.set(DOCK.vestibuleLen / 2, DOCK.portCenterH, 0); face.rotation.y = Math.PI / 2; g.add(face);
    const ring = new THREE.Mesh(new THREE.TorusGeometry(DOCK.openingD / 2 + 0.12, 0.12, 14, 48), MAT.steel); ring.castShadow = true; face.add(ring);
    const bellows = new THREE.Group(); face.add(bellows);
    for (let k = 0; k < 5; k++) { const t = new THREE.Mesh(new THREE.TorusGeometry(DOCK.openingD / 2 + 0.05, 0.06, 10, 48), MAT.black); t.position.z = 0.04 + k * 0.035; bellows.add(t); }
    const door = new THREE.Mesh(new THREE.CircleGeometry(DOCK.openingD / 2, 40), MAT.dark); door.position.z = 0.01; face.add(door);
    const latchMat = MAT.orange;
    for (let k = 0; k < 4; k++) { const l = box(0.18, 0.18, 0.25, latchMat); const a = k * Math.PI / 2 + Math.PI / 4; l.position.set(Math.cos(a) * 1.08, Math.sin(a) * 1.08, 0.1); face.add(l); }
    const tag = tagMesh(100 + p.id, 0.55); tag.position.set(0, -1.5, 0.02); face.add(tag);
    const lbl = new THREE.Mesh(new THREE.PlaneGeometry(1.8, 0.45), new THREE.MeshBasicMaterial({ map: labelTexture(p.name, { color: '#111' }), transparent: true }));
    lbl.position.set(0, 1.15, 0.02); face.add(lbl);
    const status = new THREE.Mesh(new THREE.SphereGeometry(0.09, 12, 8), MAT.amberGlow.clone()); status.position.set(0.95, 1.15, 0.05); face.add(status);
    // hazard stripes on the apron in front of the port (guidance markings)
    const stripe = new THREE.Mesh(new THREE.PlaneGeometry(26, 0.25), new THREE.MeshStandardMaterial({ color: 0xd9a21f, roughness: 0.9 }));
    stripe.rotation.x = -Math.PI / 2; stripe.rotation.z = Math.PI / 2; stripe.position.set(DOCK.vestibuleLen / 2 + 13, 0.03, 0); g.add(stripe);
    root.add(g);
    portVis.push({ port: p, g, door, bellows, status, jacks: null });
  }
  // base structures (same definitions as the physics colliders)
  const domeMat = new THREE.MeshStandardMaterial({ color: 0xd9d6cc, roughness: 0.8, metalness: 0.0 });
  for (const it of terrain.base) {
    const y = heightAt(terrain, it.x, it.z);
    const g = new THREE.Group(); g.position.set(it.x, y, it.z); root.add(g);
    if (it.kind === 'dome') {
      const dome = new THREE.Mesh(new THREE.SphereGeometry(it.r, 56, 24, 0, Math.PI * 2, 0, Math.PI / 2), domeMat); dome.castShadow = dome.receiveShadow = true; g.add(dome);
      // regolith shielding bags piled around the base ring + restraint straps
      const ring = new THREE.Mesh(new THREE.TorusGeometry(it.r, 0.9, 10, 48), new THREE.MeshStandardMaterial({ color: 0x7b766c, roughness: 1 })); ring.rotation.x = Math.PI / 2; ring.scale.z = 1.4; g.add(ring);
      for (let k = 0; k < 8; k++) { const st = new THREE.Mesh(new THREE.TorusGeometry(it.r + 0.02, 0.05, 6, 64, Math.PI), MAT.offwhite); st.rotation.y = k * Math.PI / 8; g.add(st); }
      const hatch = cyl(0.9, 0.9, 0.3, MAT.steel, 24); hatch.rotation.z = Math.PI / 2; hatch.position.set(it.r * 0.7, it.r * 0.35, 0); g.add(hatch);
    } else if (it.kind === 'mast') {
      const m = cyl(0.25, it.r, it.h, MAT.steel, 10); m.position.y = it.h / 2; g.add(m);
      const dish = new THREE.Mesh(new THREE.SphereGeometry(1.6, 24, 12, 0, Math.PI * 2, 0, 1.0), MAT.white); dish.position.y = it.h - 0.5; dish.rotation.x = -1.2; dish.castShadow = true; g.add(dish);
      const beacon = new THREE.Mesh(new THREE.SphereGeometry(0.15, 10, 8), new THREE.MeshStandardMaterial({ color: 0x300000, emissive: 0xff2020, emissiveIntensity: 3 })); beacon.position.y = it.h + 0.2; g.add(beacon);
    } else if (it.kind === 'array') {
      const pole = cyl(0.2, 0.2, 14, MAT.steel, 8); pole.position.y = 7; g.add(pole);
      const panel = box(it.w, it.h, 0.12, MAT.glassBlue); panel.position.y = 9; g.add(panel);
      root.userData.arrays = (root.userData.arrays || []).concat(g); g.userData.panel = panel;
    } else if (it.kind === 'light') {
      const m = cyl(0.12, it.r, it.h, MAT.steel, 8); m.position.y = it.h / 2; g.add(m);
      const head = box(0.9, 0.3, 0.5, MAT.dark); head.position.y = it.h; g.add(head);
      const lens = box(0.8, 0.05, 0.4, MAT.whiteGlow); lens.position.y = it.h - 0.17; g.add(lens);
    } else {
      const body = cyl(it.r, it.r, it.len, it.kind === 'tunnel' ? MAT.offwhite : MAT.white, 40); body.rotation.z = Math.PI / 2;
      const holder = new THREE.Group(); holder.rotation.y = it.yaw; holder.position.y = it.r + (it.kind === 'tunnel' ? 0.2 : 0.4); g.add(holder); holder.add(body);
      for (const sx of [-1, 1]) { const end = new THREE.Mesh(new THREE.SphereGeometry(it.r, 32, 12, 0, Math.PI * 2, 0, Math.PI / 2), MAT.offwhite); end.scale.y = 0.35; end.rotation.z = -sx * Math.PI / 2; end.position.x = sx * it.len / 2; end.castShadow = true; holder.add(end); }
      if (it.kind === 'module') for (const x of [-2.5, 0, 2.5]) { const band = cyl(it.r + 0.03, it.r + 0.03, 0.35, MAT.gold, 40); band.rotation.z = Math.PI / 2; band.position.x = x; holder.add(band); }
      for (const x of [-it.len / 2 + 1, it.len / 2 - 1]) { const leg = box(0.3, it.r + 0.4, it.r * 1.4, MAT.steel); leg.position.set(x, -(it.r + 0.4) / 2 + 0.2 - (it.kind === 'tunnel' ? 0.2 : 0), 0); holder.add(leg); }
    }
  }
  // buried power cables from the arrays to the supply building, marked by flags-free cable covers
  { const a = terrain.base.find((i) => i.kind === 'array'); const pts = [];
    for (let k = 0; k <= 24; k++) { const u = k / 24; const x = a.x + (b.x - a.x) * u, z = a.z + (b.z - a.z) * u; pts.push(new THREE.Vector3(x, heightAt(terrain, x, z) + 0.06, z)); }
    const cable = new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 60, 0.12, 6, false), new THREE.MeshStandardMaterial({ color: 0x2a2a2a, roughness: 0.8 })); cable.receiveShadow = true; root.add(cable); }
  { const live = new Set(portVis.map((v) => v.g)); for (const c of [...root.children]) if (!live.has(c)) mergeStatic(c); }
  return { root, portVis };
}

// ---------------- cargo ------------------------------------------------------
export function buildCargo(c) {
  if (c.T.handling === 'arm') return buildACM(c);
  const T = c.T, g = new THREE.Group();
  const id = c.tag0 ?? c.id.charCodeAt(3);
  if (T.shape === 'cylinder') {
    const cad = cargoModuleCAD(T); g.add(cad); g.userData.cad = cad;
    // end bulkhead domes + collars out to +/-6.39 m (keep dome compact so ringG clears at > 6.395 m)
    const bx = T.length / 2;
    for (const sgn of [-1, 1]) {
      const dome = new THREE.Mesh(new THREE.SphereGeometry(T.radius, 48, 16, 0, Math.PI * 2, 0, Math.PI / 2), MAT.offwhite);
      dome.scale.set(1, 0.075, 1); dome.rotation.z = -sgn * Math.PI / 2; dome.position.x = sgn * bx; dome.castShadow = true; g.add(dome);
      dome.userData.xrayable = true;
      const collar = cyl(1.1, 1.18, 0.12, MAT.steel, 40); collar.rotation.z = Math.PI / 2; collar.position.x = sgn * 6.33; g.add(collar);
    }
    const innerZ = T.radius - 0.1, lugLen = (T.lugZ || T.radius + 0.35) + 0.25 - innerZ;
    for (const sx of [-1, 1]) for (const sz of [-1, 1]) { const lug = cyl(T.lugR || 0.14, T.lugR || 0.14, lugLen, MAT.steel, 12); lug.rotation.x = Math.PI / 2; lug.position.set(sx * T.lugX, 0, sz * (innerZ + lugLen / 2)); g.add(lug); }
    const lbl = new THREE.Mesh(new THREE.PlaneGeometry(5, 0.9), new THREE.MeshBasicMaterial({ map: labelTexture(`${c.id}  LIFE SUPPORT`, { color: '#1b1b1b' }), transparent: true, side: THREE.DoubleSide }));
    lbl.position.set(0, 0.6, T.radius + 0.03); g.add(lbl);
    const lbl2 = lbl.clone(); lbl2.position.z = -T.radius - 0.03; lbl2.rotation.y = Math.PI; g.add(lbl2);
  } else {
    if (T.visual === 'podrack') g.add(buildPodRack(T));
    else {
      const ctex = corrugatedTexture();
      const m = new THREE.MeshStandardMaterial({ color: T.color, map: ctex, roughness: 0.65, metalness: 0.3 });
      const b = box(T.length, T.height, T.width, m); g.add(b);
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) { const cc = box(0.18, 0.12, 0.16, MAT.dark); cc.position.set(sx * (T.length / 2 - 0.09), sy * (T.height / 2 - 0.06), sz * (T.width / 2 - 0.08)); g.add(cc); }
    }
    if (T.adapterR && T.adapterLen) g.add(buildRingAdapterSled(T));
    const lbl = new THREE.Mesh(new THREE.PlaneGeometry(3.6, 0.7), new THREE.MeshBasicMaterial({ map: labelTexture(`${c.id}  ${T.visual === 'podrack' ? 'CARGO PODS' : 'HAB SUPPLIES'}`, { color: '#111' }), transparent: true, side: THREE.DoubleSide }));
    lbl.position.set(0, 0.6, T.width / 2 + 0.02); g.add(lbl);
  }
  // forward (hatch) end: docking ring + hatch (outside cylinder bulkhead collar +/-6.39 m)
  const half = T.shape === 'cylinder' ? T.radius : T.height / 2;
  const fx = T.shape === 'cylinder' ? Math.max(T.length / 2 + 0.02, 6.41) : T.length / 2 + 0.01;
  const ringG = new THREE.Group(); ringG.position.x = fx; ringG.rotation.y = Math.PI / 2; g.add(ringG);
  const ring = new THREE.Mesh(new THREE.TorusGeometry(0.98, 0.1, 12, 40), MAT.steel); ringG.add(ring);
  const hatch = new THREE.Mesh(new THREE.CircleGeometry(0.88, 32), MAT.dark); hatch.position.z = 0.01; ringG.add(hatch);
  g.userData.hatch = hatch;
  // fiducials (same placement as sensors.js model)
  const aftX = T.shape === 'cylinder' ? -Math.max(T.length / 2 + 0.02, 6.41) : (T.adapterX || 0) - (T.adapterLen || T.length) / 2 - 0.02;
  const aftY = -(T.adapterR || half) * 0.62;
  const aft = tagMesh(id, 0.8); aft.position.set(aftX, aftY, 0); aft.rotation.y = -Math.PI / 2; g.add(aft);
  const fwd = tagMesh(id + 1, 0.6); fwd.position.set(fx + 0.01, half * 0.62, 0); fwd.rotation.y = Math.PI / 2; g.add(fwd);
  const sw = T.shape === 'cylinder' ? T.radius : T.width / 2;
  for (const sz of [-1, 1]) { const s = tagMesh(id + 2, 0.8); s.position.set(-1.2, -0.3, sz * (sw + 0.035)); s.rotation.y = sz > 0 ? 0 : Math.PI; g.add(s); }
  g.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
  return g;
}
// ---------------- ACM (arm-handled cargo module) -----------------------------------------------------------------
// Full-scale metres, origin at the CG (same frame as the Rapier body, see cargoContact.js acmGeom). Mirrors the CAD
// (plan s7: acm_L/W/H, split PETG shell, 2 x D12 trunnions, 4 foot pads, round + slot bushings, keeper bar, fiducials):
//   shell    lower half + upper half, mated at a tongue-and-groove seam with bolt heads along it
//   trunnion gold pins on the +-z faces 20 mm (x K) above the CG, with a retaining collar and end cap
//   feet     four square pads with a steel plate (group names 'foot0'..'foot3'; userData.feet holds the meshes)
//   deck     round bushing + slot bushing under the shell at DECK.pinSpan, keeper bar across the underside
export function buildACM(c) {
  const T = c.T, G = acmGeom(T), g = new THREE.Group(); g.name = 'acm';
  const id = c.tag0 ?? (c.id ? c.id.charCodeAt(c.id.length - 1) + 40 : 40);
  const { L, H, W, footH, foot } = G, yMid = G.boxY, yBot = yMid - H / 2, yTop = yMid + H / 2;
  const gold = new THREE.MeshStandardMaterial({ color: 0xd4a017, roughness: 0.3, metalness: 0.9 });
  const shellMat = new THREE.MeshStandardMaterial({ color: 0xc9ccd0, roughness: 0.55, metalness: 0.15 });
  const shellMat2 = new THREE.MeshStandardMaterial({ color: 0xb4b8be, roughness: 0.55, metalness: 0.15 });
  const mk = (m, w, h, d, x, y, z) => { const b = box(w, h, d, m); b.position.set(x, y, z); g.add(b); return b; };
  // shell halves split at the seam plane y = yMid + 0.12 H (open-side-up lower tray, lid on top)
  const ySeam = yBot + H * 0.58, hLo = ySeam - yBot, hHi = yTop - ySeam;
  mk(shellMat, L, hLo, W, 0, yBot + hLo / 2, 0).name = 'shellA';
  mk(shellMat2, L, hHi, W, 0, ySeam + hHi / 2, 0).name = 'shellB';
  // tongue-and-groove seam: a slightly proud dark flange band + bolt heads around the perimeter
  const bandH = H * 0.045;
  mk(MAT.dark, L + 0.06, bandH, W + 0.06, 0, ySeam, 0).name = 'seam';
  const bolt = (x, z) => { const b = cyl(0.07, 0.07, 0.05, MAT.steel, 10); b.position.set(x, ySeam + bandH / 2 + 0.02, z); g.add(b); };
  for (let i = -3; i <= 3; i++) { bolt(i * L / 7.5, W / 2 + 0.02); bolt(i * L / 7.5, -W / 2 - 0.02); }
  for (let i = -2; i <= 2; i++) { bolt(L / 2 + 0.02, i * W / 6); bolt(-L / 2 - 0.02, i * W / 6); }
  // edge ribs on the upper half (printed stiffeners show through the lid)
  for (const sx of [-1, 1]) mk(shellMat2, 0.12, 0.05, W * 0.92, sx * L * 0.3, yTop + 0.025, 0);
  for (const sz of [-1, 1]) mk(shellMat2, L * 0.92, 0.05, 0.12, 0, yTop + 0.025, sz * W * 0.3);
  // trunnions: gold rod through the shell, retaining collar and end cap, gusset plate on the wall
  const tr = T.trunnion;
  g.userData.trunnions = [];
  for (const sz of [-1, 1]) {
    const rod = cyl(tr.r, tr.r, tr.len, gold, 24); rod.rotation.x = Math.PI / 2;
    rod.position.set(0, G.trunY, sz * (W / 2 + tr.len / 2)); g.add(rod); g.userData.trunnions.push(rod);
    const collar = cyl(tr.r * 1.5, tr.r * 1.5, 0.05, MAT.steel, 24); collar.rotation.x = Math.PI / 2; collar.position.set(0, G.trunY, sz * (W / 2 + 0.03)); g.add(collar);
    const cap = cyl(tr.r * 1.2, tr.r * 1.2, 0.04, MAT.steel, 24); cap.rotation.x = Math.PI / 2; cap.position.set(0, G.trunY, sz * (W / 2 + tr.len + 0.01)); g.add(cap);
    mk(MAT.dark, 0.9, 0.9, 0.05, 0, G.trunY, sz * (W / 2 + 0.025));
  }
  // feet: pad (rubber) + steel plate bonded to the shell underside
  g.userData.feet = [];
  G.feet.forEach((f, i) => {
    const fg = new THREE.Group(); fg.name = 'foot' + i; fg.position.set(f.x, G.groundY + footH / 2, f.z);
    const pad = box(foot, footH * 0.7, foot, MAT.rubber); pad.position.y = -footH * 0.15; fg.add(pad);
    const plate = box(foot * 1.06, footH * 0.3, foot * 1.06, MAT.steel); plate.position.y = footH * 0.35; fg.add(plate);
    g.add(fg); g.userData.feet.push(fg);
  });
  // deck interface on the underside: round + slot bushing on the centreline (3-2-1), keeper bar across
  const bushY = yBot - 0.04, span = DECK.pinSpan;
  const round = cyl(0.3, 0.3, 0.08, MAT.steel, 24); round.position.set(-span / 2, bushY, 0); g.add(round);
  const hole = cyl(0.17, 0.17, 0.09, MAT.black, 24); hole.position.set(-span / 2, bushY - 0.005, 0); g.add(hole);
  const slot = box(0.6, 0.08, 1.1, MAT.steel); slot.position.set(span / 2, bushY, 0); g.add(slot);
  const slotHole = box(0.34, 0.09, 0.8, MAT.black); slotHole.position.set(span / 2, bushY - 0.005, 0); g.add(slotHole);
  const keeper = cyl(0.11, 0.11, W * 0.55, gold, 16); keeper.rotation.x = Math.PI / 2; keeper.position.set(0, yBot - 0.16, 0); keeper.name = 'keeper'; g.add(keeper);
  for (const sz of [-1, 1]) mk(MAT.steel, 0.22, 0.2, 0.16, 0, yBot - 0.08, sz * W * 0.26);
  // fiducials on the +-x faces, label on the +-z faces (below the trunnion gusset)
  for (const sx of [-1, 1]) { const t = tagMesh(id + (sx > 0 ? 0 : 1), Math.min(H, W) * 0.55); t.position.set(sx * (L / 2 + 0.012), yMid - 0.1, 0); t.rotation.y = sx > 0 ? Math.PI / 2 : -Math.PI / 2; g.add(t); }
  const mass = Math.round(T.mass);
  const labelMat = new THREE.MeshBasicMaterial({ map: labelTexture(`${c.id || 'ACM'}  ARM CARGO  ${mass} kg`, { color: '#1b1b1b', w: 1024, h: 160 }), transparent: true, side: THREE.DoubleSide });
  for (const sz of [-1, 1]) { const lbl = new THREE.Mesh(new THREE.PlaneGeometry(W * 0.62, W * 0.62 * 160 / 1024), labelMat); lbl.position.set(0, yBot + hLo * 0.3, sz * (W / 2 + 0.015)); if (sz < 0) lbl.rotation.y = Math.PI; g.add(lbl); }
  g.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
  g.userData.acm = true;
  return g;
}

// Ring-adapter sled for the 20-ft container (Mission 8): gives the box the same 7.96 m ring interface as the 40-ft
// module, so the same saddles, ring jaws and stands carry it. Every part mirrors a collider in world.js:
//   two clamp rings (0.82 m bands at T.adapterRings), the keel shroud under the box (the convex hull the under-module
//   rangefinders see: box bottom +-1.1 m down to a +-28 deg arc of the ring radius, aft sled end to just short of the
//   docking face) and the four trunnion pins at T.lugXs. The spokes, U-hoops and longerons are the load path that
//   joins them to the box; they have no collider of their own (they lie inside the ring envelope).
function buildRingAdapterSled(T) {
  const s = new THREE.Group(); s.name = 'ring-adapter-sled';
  const R = T.adapterR, H = T.height, W = T.width, tk = 0.14;
  const frame = new THREE.MeshStandardMaterial({ color: 0xa9b0b8, metalness: 0.85, roughness: 0.35, side: THREE.DoubleSide });
  const skin = new THREE.MeshStandardMaterial({ color: 0x6f757c, metalness: 0.6, roughness: 0.55, side: THREE.DoubleSide });
  const add = (m) => { m.castShadow = m.receiveShadow = true; s.add(m); return m; };
  // clamp rings: outer band, inner band and two flange faces
  for (const rx of T.adapterRings) {
    for (const r of [R, R - tk]) { const b = add(new THREE.Mesh(new THREE.CylinderGeometry(r, r, 0.82, 64, 1, true), frame)); b.rotation.z = Math.PI / 2; b.position.x = rx; }
    for (const sx of [-1, 1]) { const f = add(new THREE.Mesh(new THREE.RingGeometry(R - tk, R, 64), frame)); f.rotation.y = Math.PI / 2; f.position.x = rx + sx * 0.41; }
    // spokes from the box to the ring: four corner posts, one crown post and two side posts
    const dirs = [[H / 2, W / 2], [H / 2, -W / 2], [-H / 2, W / 2], [-H / 2, -W / 2], [H / 2, 0], [0, W / 2], [0, -W / 2]];
    for (const [dy, dz] of dirs) {
      const r0 = Math.hypot(dy, dz), len = R - tk - r0, mid = r0 + len / 2;
      const sp = add(new THREE.Mesh(new THREE.BoxGeometry(0.5, len, 0.16), frame));
      sp.position.set(rx, dy / r0 * mid, dz / r0 * mid); sp.rotation.x = Math.atan2(dz, dy);
    }
  }
  // keel shroud (same section and ends as the physics hull)
  const xAft = (T.adapterX || 0) - T.adapterLen / 2;
  const xFwd = Math.min(T.length / 2 - 0.9, (T.adapterRings?.[1] ?? 1.24) + 0.86);
  const a = 28 * Math.PI / 180, sh = new THREE.Shape();
  sh.moveTo(-1.1, -H / 2); sh.lineTo(1.1, -H / 2); sh.lineTo(R * Math.sin(a), -R * Math.cos(a));
  sh.absarc(0, 0, R, -Math.PI / 2 + a, -Math.PI / 2 - a, true); sh.closePath();
  const keel = add(new THREE.Mesh(new THREE.ExtrudeGeometry(sh, { depth: xFwd - xAft, bevelEnabled: false, curveSegments: 16 }), skin));
  keel.rotation.y = Math.PI / 2; keel.position.x = xAft;
  // trunnion U-hoops (lower half rings) with a tie beam, one at each pair of lug pins; pins as in world.js
  const lugs = T.lugXs || [-T.lugX, T.lugX], rz = R - 0.1;
  for (const lx of lugs) {
    const hoopG = new THREE.Group(); hoopG.position.x = lx; hoopG.rotation.y = Math.PI / 2; s.add(hoopG);
    const hoop = new THREE.Mesh(new THREE.TorusGeometry(rz, 0.1, 10, 48, Math.PI), frame); hoop.rotation.z = Math.PI; hoop.castShadow = true; hoopG.add(hoop);
    const tie = add(new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.2, 2 * rz), frame)); tie.position.set(lx, 0, 0);
    const lugLen = (T.lugZ || R + 0.35) + 0.25 - rz;
    for (const sz of [-1, 1]) { const lug = add(cyl(T.lugR || 0.14, T.lugR || 0.14, lugLen, frame, 12)); lug.rotation.x = Math.PI / 2; lug.position.set(lx, 0, sz * (rz + lugLen / 2)); }
  }
  // longerons joining the hoops and rings at axis height, and two braces from the aft hoop up to the box
  for (const sz of [-1, 1]) {
    const lon = add(new THREE.Mesh(new THREE.BoxGeometry(lugs[1] - lugs[0], 0.18, 0.18), frame)); lon.position.set((lugs[0] + lugs[1]) / 2, 0, sz * (rz - 0.02));
    const x1 = -T.length / 2, dx = x1 - lugs[0], dzb = rz - W / 2, len = Math.hypot(dx, dzb);
    const br = add(new THREE.Mesh(new THREE.BoxGeometry(len, 0.16, 0.16), frame));
    br.position.set((lugs[0] + x1) / 2, 0, sz * (rz + W / 2) / 2); br.rotation.y = sz * Math.atan2(dzb, dx);
  }
  mergeStatic(s);
  return s;
}
// Astrobotic-class pod rack (CARGO_TYPES.pod4): a 20-ft flat rack with the ISO footprint of the container, open sides,
// three shelves and 36 pods of 1.22 x 0.61 m (4 ft x 2 ft). The forward end is a bulkhead that carries the transfer ring,
// so the rack berths to a port like the other two cargo systems and the pods are passed through the 2 m opening.
// Collider: the same cuboid as the 20-ft container (world.js), so every part here stays inside that envelope.
function buildPodRack(T) {
  const r = new THREE.Group(); r.name = 'pod-rack';
  const L = T.length, H = T.height, W = T.width, P = T.pods;
  const frame = new THREE.MeshStandardMaterial({ color: 0x8f969d, metalness: 0.8, roughness: 0.4 });
  const plate = new THREE.MeshStandardMaterial({ color: 0xcfd2cc, metalness: 0.15, roughness: 0.6, side: THREE.DoubleSide });
  const at = (m, x, y, z) => { m.position.set(x, y, z); r.add(m); return m; };
  at(box(L, 0.18, W, frame), 0, -H / 2 + 0.09, 0);                                    // deck
  at(box(0.12, H, W, plate), L / 2 - 0.06, 0, 0);                                      // forward bulkhead (transfer ring side)
  for (const sz of [-1, 1]) {
    for (const x of [-L / 2 + 0.07, -L / 6, L / 6, L / 2 - 0.19]) at(box(0.14, H, 0.14, frame), x, 0, sz * (W / 2 - 0.07));   // posts
    at(box(L, 0.14, 0.14, frame), 0, H / 2 - 0.07, sz * (W / 2 - 0.07));               // top rails
    const d = Math.hypot(L / 3, H - 0.3);                                                // diagonal brace, aft bay
    const br = at(box(d, 0.08, 0.08, frame), -L / 3, 0, sz * (W / 2 - 0.07)); br.rotation.z = Math.atan2(H - 0.3, L / 3);
    const tagPlate = at(box(1.0, 1.0, 0.03, plate), -1.2, -0.3, sz * (W / 2 + 0.012)); tagPlate.castShadow = true;   // carries the side fiducial
    if (sz > 0) at(box(3.8, 0.9, 0.03, plate), 0, 0.6, W / 2 + 0.002);                 // placard behind the id label
  }
  for (const x of [-L / 2 + 0.07, L / 2 - 0.19]) at(box(0.14, 0.14, W, frame), x, H / 2 - 0.07, 0);
  const pitch = (H - 0.28) / P.tiers, y0 = -H / 2 + 0.18;
  for (let k = 1; k < P.tiers; k++) at(box(L - 0.3, 0.04, W - 0.2, plate), -0.06, y0 + k * pitch, 0);   // shelves
  // pods: one instanced capsule + one instanced colour band each
  const n = P.along * P.across * P.tiers, rad = P.dia / 2 - 0.015;
  const podM = new THREE.MeshStandardMaterial({ color: 0xf2f1ec, roughness: 0.45, metalness: 0.1 });
  const bandM = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.5, metalness: 0.3 });
  const pods = new THREE.InstancedMesh(new THREE.CapsuleGeometry(rad, P.length - 2 * rad - 0.03, 8, 20), podM, n);
  const bands = new THREE.InstancedMesh(new THREE.CylinderGeometry(rad + 0.008, rad + 0.008, 0.16, 20, 1, true), bandM, n);
  const M = new THREE.Matrix4(), q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2), one = new THREE.Vector3(1, 1, 1), col = new THREE.Color();
  const gapX = (L - 0.5 - P.along * P.length) / Math.max(1, P.along - 1), stepZ = (W - 0.3) / P.across;
  const tint = [0xd9661f, 0x2f7fd0, 0x3aa35a];                                           // one colour per tier: power, science, supplies
  let i = 0;
  for (let k = 0; k < P.tiers; k++) for (let a = 0; a < P.along; a++) for (let b = 0; b < P.across; b++) {
    const x = -L / 2 + 0.2 + P.length / 2 + a * (P.length + Math.min(gapX, 0.14)), y = y0 + k * pitch + 0.03 + P.dia / 2, z = (b - (P.across - 1) / 2) * stepZ;
    M.compose(new THREE.Vector3(x, y, z), q, one); pods.setMatrixAt(i, M); bands.setMatrixAt(i, M); bands.setColorAt(i, col.setHex(tint[k % tint.length])); i++;
  }
  pods.castShadow = pods.receiveShadow = bands.castShadow = true; r.add(pods, bands);
  mergeStatic(r);
  return r;
}
function corrugatedTexture() {
  const c = document.createElement('canvas'); c.width = 512; c.height = 64; const g = c.getContext('2d');
  for (let x = 0; x < 512; x++) { const v = 200 + 45 * Math.sin(x / 512 * Math.PI * 2 * 40); g.fillStyle = `rgb(${v},${v},${v})`; g.fillRect(x, 0, 1, 64); }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}

export function buildSaddles(terrain, cargo) {
  const g = new THREE.Group();
  for (const c of cargo) {
    const T = c.T;
    const ax = Math.cos(c.yaw), az = -Math.sin(c.yaw);
    for (const sx of [-1, 1]) {
      const lx = sx * T.lugX * 0.92;
      const s = box(0.7, T.saddleH + 0.1, 1.5, MAT.steel);
      s.position.set(c.x + ax * lx, c.groundY + (T.saddleH + 0.1) / 2 - 0.05, c.z + az * lx); s.rotation.y = c.yaw; g.add(s);
    }
  }
  return g;
}

export function buildGeofences(terrain) {
  const g = new THREE.Group();
  const mk = (x, z, r, color, label) => {
    const pts = [];
    for (let i = 0; i <= 128; i++) { const a = i / 128 * Math.PI * 2; const px = x + Math.cos(a) * r, pz = z + Math.sin(a) * r; pts.push(new THREE.Vector3(px, heightAt(terrain, px, pz) + 0.25, pz)); }
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineDashedMaterial({ color, dashSize: 4, gapSize: 3, transparent: true, opacity: 0.8 }));
    line.computeLineDistances(); g.add(line);
    // label on the ring edge facing the habitat side, so it never sits over the pad itself
    const a0 = Math.atan2(WORLD.habitat.z - z, WORLD.habitat.x - x) + 0.35;
    const lx = x + Math.cos(a0) * r, lz = z + Math.sin(a0) * r;
    const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: labelTexture(label, { color: '#' + color.toString(16).padStart(6, '0'), size: 44, w: 1024 }), transparent: true, depthWrite: false }));
    spr.scale.set(24, 3, 1); spr.position.set(lx, heightAt(terrain, lx, lz) + 4, lz); g.add(spr);
  };
  for (const z of terrain.restricted) mk(z.x, z.z, z.r, 0xff4d3a, `RESTRICTED: ${z.name}`);
  mk(WORLD.pad.x, WORLD.pad.z, WORLD.plumeHazardRadius, 0xffb020, 'LANDING PLUME HAZARD ZONE');
  return g;
}
