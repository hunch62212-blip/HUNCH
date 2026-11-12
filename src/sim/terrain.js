// Procedural lunar terrain: fBm highlands + power-law crater population + boulder field.
// Deterministic from a seed, so the map the rover learns stays valid between sessions.
import { WORLD, CARGO_TYPES, DOCK, REGOLITH } from './config.js';

export function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// 2D simplex noise (Gustavson), seeded permutation
export function makeNoise(rand) {
  const perm = new Uint8Array(512);
  const p = new Uint8Array(256);
  for (let i = 0; i < 256; i++) p[i] = i;
  for (let i = 255; i > 0; i--) { const j = Math.floor(rand() * (i + 1)); const t = p[i]; p[i] = p[j]; p[j] = t; }
  for (let i = 0; i < 512; i++) perm[i] = p[i & 255];
  const G = [[1, 1], [-1, 1], [1, -1], [-1, -1], [1, 0], [-1, 0], [0, 1], [0, -1]];
  const F2 = 0.5 * (Math.sqrt(3) - 1), G2 = (3 - Math.sqrt(3)) / 6;
  return function noise(xin, yin) {
    const s = (xin + yin) * F2;
    const i = Math.floor(xin + s), j = Math.floor(yin + s);
    const t = (i + j) * G2;
    const x0 = xin - (i - t), y0 = yin - (j - t);
    const i1 = x0 > y0 ? 1 : 0, j1 = x0 > y0 ? 0 : 1;
    const x1 = x0 - i1 + G2, y1 = y0 - j1 + G2;
    const x2 = x0 - 1 + 2 * G2, y2 = y0 - 1 + 2 * G2;
    const ii = i & 255, jj = j & 255;
    let n = 0;
    let t0 = 0.5 - x0 * x0 - y0 * y0;
    if (t0 > 0) { const g = G[perm[ii + perm[jj]] & 7]; t0 *= t0; n += t0 * t0 * (g[0] * x0 + g[1] * y0); }
    let t1 = 0.5 - x1 * x1 - y1 * y1;
    if (t1 > 0) { const g = G[perm[ii + i1 + perm[jj + j1]] & 7]; t1 *= t1; n += t1 * t1 * (g[0] * x1 + g[1] * y1); }
    let t2 = 0.5 - x2 * x2 - y2 * y2;
    if (t2 > 0) { const g = G[perm[ii + 1 + perm[jj + 1]] & 7]; t2 *= t2; n += t2 * t2 * (g[0] * x2 + g[1] * y2); }
    return 70 * n;
  };
}

// Local soil state at (x, z): { k: stiffness / cohesion multiplier, dphi: friction-angle offset (rad) }.
// Deterministic from the terrain seed (REGOLITH.vary). Cheap value noise: called per wheel per physics step.
export function soilAt(t, x, z, out = {}) {
  const V = t.soilVary;
  if (!V || !V.amp) { out.k = 1; out.dphi = 0; return out; }
  const u = x / V.scale, v = z / V.scale, s0 = t.soilSeed || 0;
  const n = 0.6 * Math.sin(u * 1.31 + s0) * Math.cos(v * 1.17 - s0 * 0.7) + 0.4 * Math.sin(u * 2.9 - v * 2.3 + s0 * 1.9);
  out.k = 1 + V.amp * n; out.dphi = V.phiDeg * Math.PI / 180 * n;
  return out;
}

function smoothstep(a, b, x) { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); }

// Fresh simple-crater profile (Pike 1977 / Melosh): depth/diameter ~0.2, rim ~0.04 D, ejecta ~ r^-3
function craterProfile(r, R, depth, rim) {
  const q = r / R;
  if (q < 1) return -depth + (depth + rim) * q * q;
  return rim * Math.pow(q, -3) * smoothstep(3.2, 1.0, q) ;
}

export function habitatLayout() {
  const { pad, habitat } = WORLD;
  let nx = pad.x - habitat.x, nz = pad.z - habitat.z;
  const L = Math.hypot(nx, nz); nx /= L; nz /= L;            // face normal, points toward the pad
  const tx = -nz, tz = nx;                                     // lateral (along the building face)
  const yaw = Math.atan2(-nz, nx);                             // rotation of building +X onto n
  const face = { x: habitat.x - nx * 4, z: habitat.z - nz * 4 }; // building wall plane
  // receiving building sized for the 7.96 m module: ports at the carried module-axis height (~10 m)
  const building = { x: face.x - nx * 10, z: face.z - nz * 10, w: 20, d: 46, h: 15.5 };
  // PORT-1 stays at u = -11.5 m (its approach is the validated one: tests dock_cyl40, cycle). PORT-3 (pod rack) is at
  // +1.5 m and PORT-2 at +14.5 m: 13 m between neighbours, because a docked sled's receiving-stand posts stand +-5.63 m
  // about its axis and must clear the next port's posts and the 9.9 m wide rover working there. (Two ports at +-11.5 m
  // with a third at 0 would have put two stands' posts 0.24 m apart.)
  const ports = [-11.5, 14.5, 1.5].map((off, i) => {
    const px = face.x + tx * off + nx * DOCK.vestibuleLen;
    const pz = face.z + tz * off + nz * DOCK.vestibuleLen;
    return { id: i + 1, name: `PORT-${i + 1}`, x: px, z: pz, nx, nz, tx, tz, wallX: face.x + tx * off, wallZ: face.z + tz * off };
  });
  return { nx, nz, tx, tz, yaw, face, building, ports, length: L };
}

// Every habitat-area structure, defined once and used for BOTH physics colliders and visuals.
export function baseStructures(hab) {
  const b = hab.building, n = { x: hab.nx, z: hab.nz }, t = { x: hab.tx, z: hab.tz };
  const back = { x: b.x - n.x * 38, z: b.z - n.z * 38 };
  const items = [];
  items.push({ kind: 'dome', x: back.x, z: back.z, r: 9 });
  items.push({ kind: 'mast', x: back.x + 14, z: back.z + 6, r: 0.35, h: 22 });
  for (let k = 0; k < 4; k++) items.push({ kind: 'array', x: back.x - 16 + k * 7, z: back.z - 18, w: 5, h: 10 });
  // pressurised tunnel: supply building back wall -> habitat dome
  const a0 = { x: b.x - n.x * 10, z: b.z - n.z * 10 }, a1 = { x: back.x + n.x * 8.6, z: back.z + n.z * 8.6 };
  items.push({ kind: 'tunnel', x: (a0.x + a1.x) / 2, z: (a0.z + a1.z) / 2, r: 1.25, len: Math.hypot(a1.x - a0.x, a1.z - a0.z), yaw: Math.atan2(-n.z, n.x) });
  // two horizontal habitat modules beside the dome, joined to it
  for (const off of [-1, 1]) {
    const c = { x: back.x + t.x * off * 15 - n.x * 2, z: back.z + t.z * off * 15 - n.z * 2 };
    items.push({ kind: 'module', x: c.x, z: c.z, r: 2.3, len: 9, yaw: Math.atan2(-t.z, t.x) });
  }
  // Apron light masts, 18.5 / 15.5 m outside the outer ports' approach axes (ports at u = -11.5 and +14.5). They stood at u = +-20, only
  // 8.5 m from the axis: 3.6 m from the side of a 9.9 m wide rover, inside the swing of a carried 40-ft module, and
  // inside the planner's soft keep-out, which bent the loaded approach path around each mast into an S-turn tighter
  // than the loaded turning circle (the rover orbited the PORT-1 lead-in point: test dock_cyl40).
  for (const [u, v] of [[-30, 22], [30, 22], [-30, 46], [30, 46]]) items.push({ kind: 'light', x: hab.face.x + t.x * u + n.x * v, z: hab.face.z + t.z * u + n.z * v, r: 0.2, h: 9 });
  return items;
}

export function padLayout() {
  const { pad, habitat } = WORLD;
  let dx = habitat.x - pad.x, dz = habitat.z - pad.z;
  const L = Math.hypot(dx, dz); dx /= L; dz /= L;
  const px = -dz, pz = dx; // perpendicular
  // Cargo offloaded by the lander crane onto ground saddles beside the lander
  // offloaded by the lander crane onto the team's skid stands (module_models.js buildStand) either side of the lander
  const cargo = [
    { id: 'CM-40A', type: 'cyl40', x: pad.x + px * 40 + dx * 6, z: pad.z + pz * 40 + dz * 6, yaw: Math.atan2(-dz, dx) },
    { id: 'CC-20B', type: 'box20', x: pad.x - px * 40 + dx * 6, z: pad.z - pz * 40 + dz * 6, yaw: Math.atan2(-dz, dx) },
    // pod rack: outboard of and behind the container (22 m to the side, 34 m back), inside the pad's 70 m level area;
    // 11 m clear of the rover's line into and out of CC-20B
    { id: 'CP-36C', type: 'pod4', x: pad.x - px * 62 - dx * 28, z: pad.z - pz * 62 - dz * 28, yaw: Math.atan2(-dz, dx), dest: 3, category: 'power', tag0: 60 },
  ];
  // parked clear of the lander and cargo, nose toward the habitat (Mission 1 starts from here)
  const roverStart = { x: pad.x - dx * 58, z: pad.z - dz * 58, yaw: Math.atan2(-dz, dx) };
  const staging = { x: pad.x + dx * 95, z: pad.z + dz * 95 };
  return { dir: { x: dx, z: dz }, perp: { x: px, z: pz }, cargo, roverStart, staging };
}

export function generateTerrain(seed = WORLD.seed) {
  const N = WORLD.size / WORLD.res;     // subdivisions
  const S = N + 1;                      // samples per side
  const half = WORLD.size / 2;
  const rand = mulberry32(seed);
  const n1 = makeNoise(rand), n2 = makeNoise(rand), n3 = makeNoise(rand);
  const h = new Float32Array(S * S);
  const albedo = new Float32Array(S * S); // brightness modulation (fresh ejecta is brighter)
  const hab = habitatLayout();
  const padL = padLayout();
  const { pad } = WORLD;

  // 1. Base topography: multi-octave fBm with a gentle regional tilt and a ridge line
  for (let iz = 0; iz < S; iz++) {
    const z = iz * WORLD.res - half;
    for (let ix = 0; ix < S; ix++) {
      const x = ix * WORLD.res - half;
      let v = 0;
      v += 9.0 * n1(x / 900, z / 900);
      v += 3.2 * n2(x / 260, z / 260);
      v += 1.1 * n3(x / 90, z / 90);
      v += 0.35 * n1(x / 31 + 17, z / 31 - 5);
      v += 0.10 * n2(x / 9.5, z / 9.5);
      const ridge = 1 - Math.abs(n3(x / 700 + 3, z / 700 - 9));
      v += 6 * ridge * ridge;
      v += 0.004 * x - 0.003 * z;
      h[iz * S + ix] = v;
      albedo[iz * S + ix] = 0.5 * n2(x / 140 - 4, z / 140 + 8);
    }
  }

  // 2. Craters: truncated power law, cumulative N(>D) ~ D^-1.8
  const craters = [];
  const crand = mulberry32(seed ^ 0x9e3779b9);
  const Dmin = 3, Dmax = 170, alpha = 1.8;
  const areaK = (WORLD.size / 2048) ** 2;            // populations were tuned on a 2048 m map: keep the same density
  const nCraters = Math.round(2300 * areaK);
  const a0 = Math.pow(Dmin, -alpha), a1 = Math.pow(Dmax, -alpha);
  for (let k = 0; k < nCraters; k++) {
    const D = Math.pow(a0 - crand() * (a0 - a1), -1 / alpha);
    craters.push({ x: (crand() * 2 - 1) * (half - 10), z: (crand() * 2 - 1) * (half - 10), D, fresh: Math.pow(crand(), 2.4) });   // most craters are old and degraded (micrometeorite gardening)
  }
  // Three large fresh craters near the straight-line route (random spots along it, from the seed), so the planner
  // always has to route around something
  const along = (f, lat) => ({ x: pad.x + padL.dir.x * f + padL.perp.x * lat, z: pad.z + padL.dir.z * f + padL.perp.z * lat });
  for (const [f0, f1, D, fresh] of [[300, 700, 95, 0.95], [800, 1200, 150, 0.85], [1300, 1700, 70, 0.7]]) {
    const p = along(f0 + crand() * (f1 - f0), (crand() * 2 - 1) * 40);
    craters.push({ ...p, D, fresh });
  }

  const keepClear = (x, z, r) => {
    const dp = Math.hypot(x - pad.x, z - pad.z), dh = Math.hypot(x - WORLD.habitat.x, z - WORLD.habitat.z);
    return dp < 120 + r || dh < 120 + r;
  };
  for (const c of craters) {
    if (keepClear(c.x, c.z, c.D * 0.6)) { c.skip = true; continue; }
    const R = c.D / 2;
    const freshness = 0.2 + 0.8 * c.fresh;
    const depth = c.D * (0.035 + 0.15 * freshness);
    const rim = c.D * 0.036 * freshness;
    const ext = R * 3.2;
    const ix0 = Math.max(0, Math.floor((c.x - ext + half) / WORLD.res)), ix1 = Math.min(N, Math.ceil((c.x + ext + half) / WORLD.res));
    const iz0 = Math.max(0, Math.floor((c.z - ext + half) / WORLD.res)), iz1 = Math.min(N, Math.ceil((c.z + ext + half) / WORLD.res));
    for (let iz = iz0; iz <= iz1; iz++) {
      const z = iz * WORLD.res - half;
      for (let ix = ix0; ix <= ix1; ix++) {
        const x = ix * WORLD.res - half;
        const r = Math.hypot(x - c.x, z - c.z);
        if (r > ext) continue;
        const wob = 1 + 0.06 * n1(x / (R * 0.5 + 1), z / (R * 0.5 + 1));
        h[iz * S + ix] += craterProfile(r * wob, R, depth, rim);
        if (r < R * 2.4) albedo[iz * S + ix] += 0.55 * c.fresh * c.fresh * smoothstep(R * 2.4, R * 0.9, r) * (c.D > 6 ? 1 : 0.4);
      }
    }
    c.depth = depth; c.rim = rim;
  }

  // 3. Flatten the landing pad (sintered regolith) and the habitat apron
  const sampleRaw = (x, z) => {
    const ix = Math.round((x + half) / WORLD.res), iz = Math.round((z + half) / WORLD.res);
    return h[iz * S + ix];
  };
  const padH = sampleRaw(pad.x, pad.z);
  const habH = sampleRaw(WORLD.habitat.x, WORLD.habitat.z);
  const flatten = (cx, cz, r0, r1, target) => {
    const ext = r1;
    const ix0 = Math.max(0, Math.floor((cx - ext + half))), ix1 = Math.min(N, Math.ceil((cx + ext + half)));
    const iz0 = Math.max(0, Math.floor((cz - ext + half))), iz1 = Math.min(N, Math.ceil((cz + ext + half)));
    for (let iz = iz0; iz <= iz1; iz++) for (let ix = ix0; ix <= ix1; ix++) {
      const x = ix - half, z = iz - half;
      const r = Math.hypot(x - cx, z - cz);
      const w = smoothstep(r1, r0, r);
      if (w > 0) h[iz * S + ix] = h[iz * S + ix] * (1 - w) + target * w;
    }
  };
  flatten(pad.x, pad.z, 70, 125, padH);
  flatten(WORLD.habitat.x - hab.nx * 6, WORLD.habitat.z - hab.nz * 6, 62, 110, habH);

  // 4. Boulders: cumulative N(>d) ~ d^-2.5, plus blocky ejecta around fresh craters
  const rocks = [];
  const rrand = mulberry32(seed ^ 0x51ed270b);
  const addRock = (x, z, d) => {
    if (Math.abs(x) > half - 4 || Math.abs(z) > half - 4) return;
    const dp = Math.hypot(x - pad.x, z - pad.z), dh = Math.hypot(x - WORLD.habitat.x, z - WORLD.habitat.z);
    if (dp < 80 || dh < 85) return;
    rocks.push({
      x, z, d,
      shape: Math.floor(rrand() * 6),
      sy: 0.45 + rrand() * 0.4,            // flattened (rocks are mostly embedded)
      sx: 0.8 + rrand() * 0.45, sz: 0.8 + rrand() * 0.45,
      yaw: rrand() * Math.PI * 2, tilt: (rrand() - 0.5) * 0.4,
      bury: 0.18 + rrand() * 0.25,
    });
  };
  const dmin = 0.25, dmax = 3.4, beta = 2.5;
  const b0 = Math.pow(dmin, -beta), b1 = Math.pow(dmax, -beta);
  for (let k = 0, nRocks = Math.round(5200 * areaK); k < nRocks; k++) {
    const d = Math.pow(b0 - rrand() * (b0 - b1), -1 / beta);
    addRock((rrand() * 2 - 1) * half, (rrand() * 2 - 1) * half, d);
  }
  for (const c of craters) {
    if (c.skip || c.D < 18 || c.fresh < 0.55) continue;
    const count = Math.floor(c.D * 0.9 * c.fresh);
    for (let k = 0; k < count; k++) {
      const ang = rrand() * Math.PI * 2;
      const r = c.D / 2 * (0.85 + Math.pow(rrand(), 1.6) * 1.6);
      const d = Math.min(3.0, Math.pow(b0 - rrand() * (b0 - Math.pow(2.2, -beta)), -1 / beta) * (0.8 + c.D / 160));
      addRock(c.x + Math.cos(ang) * r, c.z + Math.sin(ang) * r, d);
    }
  }
  // A boulder field across the route (random spot in the middle half, from the seed), for Mission 4/7
  const bf = along(500 + rrand() * 1000, (rrand() * 2 - 1) * 30);
  for (let k = 0; k < 220; k++) {
    const ang = rrand() * Math.PI * 2, r = Math.sqrt(rrand()) * 45;
    const d = 0.4 + Math.pow(rrand(), 2.2) * 2.4;
    addRock(bf.x + Math.cos(ang) * r * 1.6, bf.z + Math.sin(ang) * r, d);
  }

  const terrain = {
    N, S, half, res: WORLD.res, h, albedo, craters: craters.filter((c) => !c.skip), rocks,
    padH, habH, hab, pad: padL, seed, base: baseStructures(hab),
    restricted: [
      { name: 'Pad B - active landing site', x: pad.x + padL.dir.x * 1100 + padL.perp.x * 140, z: pad.z + padL.dir.z * 1100 + padL.perp.z * 140, r: 170 },
      { name: 'Science preserve (Site 7)', x: pad.x + padL.dir.x * 300 - padL.perp.x * 160, z: pad.z + padL.dir.z * 300 - padL.perp.z * 160, r: 120 },
    ],
  };
  // Rocks rest on the surface: compute base heights after flattening
  for (const r of rocks) r.y = heightAt(terrain, r.x, r.z);
  terrain.soilVary = REGOLITH.vary; terrain.soilSeed = (seed % 1000) * 0.0127;
  return terrain;
}

export function heightAt(t, x, z) {
  const fx = (x + t.half) / t.res, fz = (z + t.half) / t.res;
  let ix = Math.floor(fx), iz = Math.floor(fz);
  if (ix < 0) ix = 0; if (iz < 0) iz = 0; if (ix > t.N - 1) ix = t.N - 1; if (iz > t.N - 1) iz = t.N - 1;
  const ax = Math.min(1, Math.max(0, fx - ix)), az = Math.min(1, Math.max(0, fz - iz));
  const S = t.S, h = t.h;
  const h00 = h[iz * S + ix], h10 = h[iz * S + ix + 1], h01 = h[(iz + 1) * S + ix], h11 = h[(iz + 1) * S + ix + 1];
  // match Rapier's triangulation closely enough for placement purposes
  return (h00 * (1 - ax) + h10 * ax) * (1 - az) + (h01 * (1 - ax) + h11 * ax) * az;
}

export function normalAt(t, x, z) {
  const e = 0.75;
  const dx = heightAt(t, x + e, z) - heightAt(t, x - e, z);
  const dz = heightAt(t, x, z + e) - heightAt(t, x, z - e);
  const nx = -dx / (2 * e), nz = -dz / (2 * e);
  const L = Math.hypot(nx, 1, nz);
  return { x: nx / L, y: 1 / L, z: nz / L };
}

// Rapier wants heights[ix * (nz+1) + iz]
export function toRapierHeights(t) {
  const S = t.S, out = new Float32Array(S * S);
  for (let iz = 0; iz < S; iz++) for (let ix = 0; ix < S; ix++) out[ix * S + iz] = t.h[iz * S + ix];
  return out;
}

export { CARGO_TYPES };
