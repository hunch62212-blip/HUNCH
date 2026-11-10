// Chunked LOD terrain mesh built from the exact physics heightfield, plus a far horizon ring
// (procedural massifs, with lunar curvature drop) so the 2 km site sits in a real landscape.
import * as THREE from 'three';
import { heightAt, makeNoise, mulberry32 } from '../sim/terrain.js';
import { WORLD, MOON } from '../sim/config.js';

const CH = 128;                    // chunk size (m)
const LODS = [1, 2, 4, 8, 16];

export function regolithMaterial() {
  const normalMap = detailNormalTexture(1024, 7);
  normalMap.wrapS = normalMap.wrapT = THREE.RepeatWrapping;
  const rough = detailAlbedoTexture(512, 11);
  rough.wrapS = rough.wrapT = THREE.RepeatWrapping;
  const mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 1.0, metalness: 0.0, normalMap, normalScale: new THREE.Vector2(0.9, 0.9), map: rough });
  // Lunar regolith is strongly back-scattering (Lommel-Seeliger + opposition surge), so it
  // looks flatter-lit than Lambert. Blend the diffuse term toward cos(i)/(cos(i)+cos(e)).
  mat.userData.shaderKey = 'regolith';
  mat.onBeforeCompile = (sh) => {
    // two-scale detail (breaks texture tiling): fine 5.5 m tile + a rotated 32 m macro layer
    sh.fragmentShader = sh.fragmentShader
      .replace('vec4 sampledDiffuseColor = texture2D( map, vMapUv );', 'vec4 sampledDiffuseColor = texture2D( map, vMapUv ) * (0.78 + 0.44 * texture2D( map, mat2(0.82,0.57,-0.57,0.82) * vMapUv * 0.171 + vec2(0.31, 0.77) ).r);')
      .replace('vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;', 'vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0; vec3 mapN2 = texture2D( normalMap, mat2(0.82,0.57,-0.57,0.82) * vNormalMapUv * 0.171 + vec2(0.31, 0.77) ).xyz * 2.0 - 1.0; mapN = normalize( vec3( mapN.xy + mapN2.xy * 1.4, mapN.z ) );');
    sh.fragmentShader = sh.fragmentShader.replace('#include <lights_fragment_end>', `
      #include <lights_fragment_end>
      {
        vec3 Ld = normalize(directionalLights[0].direction);
        float mu0 = max(dot(normal, Ld), 0.0);
        float mu = max(dot(normal, normalize(vViewPosition)), 0.05);
        float ls = 2.0 * mu0 / (mu0 + mu + 1e-4);
        float k = (mu0 > 1e-3) ? clamp(ls / max(mu0, 1e-3), 0.6, 2.2) : 1.0;
        reflectedLight.directDiffuse *= mix(1.0, k, 0.55);
        // opposition surge (Hapke shadow-hiding): regolith brightens sharply toward zero phase angle,
        // the bright halo around the rover's own shadow seen in every Apollo surface photo
        float cosG = dot(normalize(vViewPosition), Ld);
        float g = acos(clamp(cosG, -1.0, 1.0));
        reflectedLight.directDiffuse *= 1.0 + 0.85 * exp(-g / 0.10) + 0.25 * exp(-g / 0.5);
      }`);
  };
  return mat;
}

function detailNormalTexture(N, seed) {
  const rand = mulberry32(seed), n1 = makeNoise(rand), n2 = makeNoise(rand);
  const h = new Float32Array(N * N);
  const craters = Array.from({ length: 90 }, () => ({ x: rand() * N, y: rand() * N, r: 2 + Math.pow(rand(), 3) * 26 }));
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
    const tile = (f) => { // tileable noise via 4-corner blend
      const u = x / N, v = y / N;
      const a = f(x, y), b = f(x - N, y), c = f(x, y - N), d = f(x - N, y - N);
      return (a * (1 - u) + b * u) * (1 - v) + (c * (1 - u) + d * u) * v;
    };
    let v = tile((px, py) => 0.6 * n1(px / 40, py / 40) + 0.3 * n2(px / 13, py / 13) + 0.15 * n1(px / 4, py / 4 + 9));
    for (const c of craters) {
      let dx = Math.abs(x - c.x), dy = Math.abs(y - c.y); dx = Math.min(dx, N - dx); dy = Math.min(dy, N - dy);
      const q = Math.hypot(dx, dy) / c.r; if (q < 1) v += (q * q - 0.7) * 0.6 * c.r / 10; else if (q < 1.6) v += 0.18 * (1.6 - q) * c.r / 10;
    }
    h[y * N + x] = v;
  }
  const c = document.createElement('canvas'); c.width = c.height = N;
  const g = c.getContext('2d'), img = g.createImageData(N, N);
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
    const hx = h[y * N + ((x + 1) % N)] - h[y * N + ((x - 1 + N) % N)];
    const hy = h[((y + 1) % N) * N + x] - h[((y - 1 + N) % N) * N + x];
    let nx = -hx * 1.2, ny = -hy * 1.2, nz = 1; const L = Math.hypot(nx, ny, nz); nx /= L; ny /= L; nz /= L;
    const i = (y * N + x) * 4; img.data[i] = (nx * 0.5 + 0.5) * 255; img.data[i + 1] = (ny * 0.5 + 0.5) * 255; img.data[i + 2] = (nz * 0.5 + 0.5) * 255; img.data[i + 3] = 255;
  }
  g.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.NoColorSpace; t.anisotropy = 8; return t;
}

function detailAlbedoTexture(N, seed) {
  const rand = mulberry32(seed), n1 = makeNoise(rand);
  const c = document.createElement('canvas'); c.width = c.height = N;
  const g = c.getContext('2d'), img = g.createImageData(N, N);
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
    const u = x / N, v = y / N;
    const f = (px, py) => n1(px / 7, py / 7) * 0.5 + n1(px / 2.2, py / 2.2) * 0.5;
    const val = (f(x, y) * (1 - u) + f(x - N, y) * u) * (1 - v) + (f(x, y - N) * (1 - u) + f(x - N, y - N) * u) * v;
    const speck = rand() < 0.02 ? 30 : 0;
    const b = Math.max(0, Math.min(255, 214 + val * 34 + speck));
    const i = (y * N + x) * 4; img.data[i] = b; img.data[i + 1] = b * 0.995; img.data[i + 2] = b * 0.985; img.data[i + 3] = 255;
  }
  g.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; return t;
}

export class TerrainMesh {
  constructor(scene, terrain) {
    this.t = terrain;
    this.material = regolithMaterial();
    this.group = new THREE.Group(); scene.add(this.group);
    this.n = Math.ceil(WORLD.size / CH); // the last row/column of chunks is clipped at the map edge (see build)
    this.chunks = [];
    for (let j = 0; j < this.n; j++) for (let i = 0; i < this.n; i++) {
      const mesh = new THREE.Mesh(undefined, this.material);
      mesh.receiveShadow = true; mesh.castShadow = false; mesh.matrixAutoUpdate = false;
      mesh.visible = false;
      this.group.add(mesh);
      this.chunks.push({ i, j, x0: i * CH - WORLD.size / 2, z0: j * CH - WORLD.size / 2, lod: -1, mesh, cache: new Map() });
    }
    this.horizon = this.buildHorizon();
    scene.add(this.horizon);
  }

  colorAt(x, z) {
    const t = this.t;
    const fx = Math.round((x + t.half) / t.res), fz = Math.round((z + t.half) / t.res);
    const a = t.albedo[Math.max(0, Math.min(t.S - 1, fz)) * t.S + Math.max(0, Math.min(t.S - 1, fx))] || 0;
    let c = 0.36 + 0.05 * a;
    const dp = Math.hypot(x - WORLD.pad.x, z - WORLD.pad.z);
    if (dp < 34) c *= 0.62 + 0.05 * Math.sin(dp * 0.9);            // sintered landing pad
    else if (dp < 60) c *= 1.0 + 0.18 * (1 - (dp - 34) / 26);       // plume-scoured bright halo
    const dh = Math.hypot(x - WORLD.habitat.x, z - WORLD.habitat.z);
    if (dh < 55) c *= 0.9;                                          // compacted apron
    return c;
  }

  build(ch, lod) {
    const key = lod;
    if (ch.cache.has(key)) return ch.cache.get(key);
    const t = this.t, n = CH / lod + 1;
    const vcount = n * n + 4 * n;
    const pos = new Float32Array(vcount * 3), nor = new Float32Array(vcount * 3), col = new Float32Array(vcount * 3), uv = new Float32Array(vcount * 2);
    const idx = [];
    let v = 0;
    const put = (x, z, y) => {
      pos[v * 3] = x; pos[v * 3 + 1] = y; pos[v * 3 + 2] = z;
      const e = Math.max(1, lod * 0.75);
      const dx = heightAt(t, x + e, z) - heightAt(t, x - e, z), dz = heightAt(t, x, z + e) - heightAt(t, x, z - e);
      let nx = -dx / (2 * e), nz = -dz / (2 * e), ny = 1; const L = Math.hypot(nx, ny, nz);
      nor[v * 3] = nx / L; nor[v * 3 + 1] = ny / L; nor[v * 3 + 2] = nz / L;
      const c = this.colorAt(x, z);
      col[v * 3] = c; col[v * 3 + 1] = c * 0.985; col[v * 3 + 2] = c * 0.965;
      uv[v * 2] = x / 5.5; uv[v * 2 + 1] = z / 5.5;
      return v++;
    };
    for (let b = 0; b < n; b++) for (let a = 0; a < n; a++) {
      // clamp to the heightfield edge: a chunk that overhangs it collapses onto the border (degenerate, invisible
      // triangles) instead of indexing past the end of a row
      const x = Math.min(ch.x0 + a * lod, t.half), z = Math.min(ch.z0 + b * lod, t.half);
      put(x, z, t.h[Math.round((z + t.half) / t.res) * t.S + Math.round((x + t.half) / t.res)]);
    }
    for (let b = 0; b < n - 1; b++) for (let a = 0; a < n - 1; a++) {
      const i0 = b * n + a, i1 = i0 + 1, i2 = i0 + n, i3 = i2 + 1;
      // match Rapier's heightfield triangulation (diagonal from (i,j) to (i+1,j+1))
      idx.push(i0, i2, i3, i0, i3, i1);
    }
    // skirts hide LOD cracks
    const skirt = (getIdx) => {
      const first = v;
      for (let k = 0; k < n; k++) { const s = getIdx(k); put(pos[s * 3], pos[s * 3 + 2], pos[s * 3 + 1] - 2.5 * lod); }
      for (let k = 0; k < n - 1; k++) { const a0 = getIdx(k), a1 = getIdx(k + 1), b0 = first + k, b1 = first + k + 1; idx.push(a0, b0, a1, a1, b0, b1, a0, a1, b0, a1, b1, b0); }
    };
    skirt((k) => k); skirt((k) => (n - 1) * n + k); skirt((k) => k * n); skirt((k) => k * n + n - 1);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    g.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    g.setIndex(idx);
    g.computeBoundingSphere(); g.computeBoundingBox();
    ch.cache.set(key, g);
    if (ch.cache.size > 3) { // keep memory bounded
      for (const k of ch.cache.keys()) if (k !== key && k !== ch.lod && ch.cache.size > 3) { ch.cache.get(k).dispose(); ch.cache.delete(k); }
    }
    return g;
  }

  update(camPos, budget = 10) {
    const want = [];
    for (const ch of this.chunks) {
      const cx = ch.x0 + CH / 2, cz = ch.z0 + CH / 2;
      const d = Math.max(0, Math.hypot(cx - camPos.x, cz - camPos.z) - CH * 0.7);
      const lod = d < 90 ? 1 : d < 260 ? 2 : d < 600 ? 4 : d < 1200 ? 8 : 16;
      if (lod !== ch.lod) want.push({ ch, lod, d });
    }
    want.sort((a, b) => a.d - b.d);
    let n = 0;
    for (const w of want) {
      if (n >= budget && w.ch.lod !== -1) break;
      w.ch.mesh.geometry = this.build(w.ch, w.lod);
      w.ch.lod = w.lod; w.ch.mesh.visible = true; w.ch.mesh.updateMatrix();
      if (!w.ch.cache.has(w.lod)) {} n++;
    }
  }

  buildHorizon() {
    const rand = mulberry32(77), n1 = makeNoise(rand), n2 = makeNoise(rand);
    const NR = 90, NA = 360, R0 = 900, R1 = 16000;
    const pos = [], col = [], idx = [];
    const half = WORLD.size / 2;
    for (let r = 0; r < NR; r++) {
      const rad = R0 * Math.pow(R1 / R0, r / (NR - 1));
      for (let a = 0; a < NA; a++) {
        const th = a / NA * Math.PI * 2;
        const x = Math.cos(th) * rad, z = Math.sin(th) * rad;
        const inside = Math.max(Math.abs(x), Math.abs(z)) < half - 2;
        const ex = Math.max(-half, Math.min(half, x)), ez = Math.max(-half, Math.min(half, z));
        const edgeH = heightAt(this.t, ex, ez);
        const dOut = Math.max(0, Math.max(Math.abs(x), Math.abs(z)) - half);
        const w = Math.min(1, dOut / 600);
        const ridge = 1 - Math.abs(n1(x / 2600, z / 2600));
        const mount = 260 * Math.pow(ridge, 3) * Math.min(1, dOut / 3000) + 50 * n2(x / 700, z / 700) * w + 14 * n1(x / 160, z / 160) * w;
        // a Malapert-like massif on the far horizon
        const mm = Math.max(0, 1 - Math.hypot(x - 9000, z + 4000) / 4200);
        let y = edgeH * (1 - w) + (edgeH + mount + 900 * mm * mm) * w - (rad * rad) / (2 * MOON.radius) * w;
        if (inside) y = heightAt(this.t, x, z) - 40;
        pos.push(x, y, z);
        const c = 0.33 + 0.04 * n2(x / 400, z / 400);
        col.push(c, c * 0.985, c * 0.965);
      }
    }
    for (let r = 0; r < NR - 1; r++) for (let a = 0; a < NA; a++) {
      const i0 = r * NA + a, i1 = r * NA + (a + 1) % NA, i2 = i0 + NA, i3 = i1 + NA;
      idx.push(i0, i2, i1, i1, i2, i3);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    g.setIndex(idx); g.computeVertexNormals();
    const m = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 1, metalness: 0 });
    const mesh = new THREE.Mesh(g, m); mesh.receiveShadow = false; mesh.frustumCulled = false;
    return mesh;
  }
}
