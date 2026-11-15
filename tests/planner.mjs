// Planner unit checks (CHANGES.md §17): unmapped ground next to a mapped slope / crater costs 12 /m, and the loaded
// smoothing keeps the planned centre line at R >= 18 m. Uses a real Sim for the costmap inputs; no mission is run.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { Planner } from '../src/sim/planner.js';
import { CLS, TILE, RES } from '../src/sim/slam.js';
import { check, atLeast, atMost } from './lib/check.mjs';
await RAPIER.init();
const sim = new Sim(RAPIER);
const P = sim.planner || sim.autonomy.planner, S = sim.slam;

// ---- 1. unmapped cost next to a mapped CRATER / SLOPE cell
S.clear();
const ox = 300, oz = 300;                           // empty corner of the map, away from every base structure
const t = S.getTile(Math.floor(ox / (TILE * RES)), Math.floor(oz / (TILE * RES)), true);
const mark = (wx, wz, cls) => { // a 2 x 2 m patch of mapped ground of the given class at world (wx, wz)
  for (let dx = -2; dx <= 2; dx++) for (let dz = -2; dz <= 2; dz++) {
    const ix = Math.floor(wx / RES) + dx, iz = Math.floor(wz / RES) + dz;
    const tt = S.getTile(Math.floor(ix / TILE), Math.floor(iz / TILE), true), i = (iz - tt.tz * TILE) * TILE + (ix - tt.tx * TILE);
    tt.n[i] = 5; tt.h[i] = 0; tt.haz[i] = 120; tt.cls[i] = cls; tt.dirty = false;
  }
};
mark(ox + 0.5, oz + 0.5, CLS.CRATER);
mark(ox + 100.5, oz + 0.5, CLS.SLOPE);
mark(ox + 200.5, oz + 0.5, CLS.FREE);
P.buildCostmap([]);
const cost = (x, z) => P.cost[P.idx(x, z)];
check('unmapped 10 m from a mapped crater cell costs 12 /m', cost(ox + 10, oz) === 12, `${cost(ox + 10, oz)}`);
check('unmapped 10 m from a mapped slope cell costs 12 /m', cost(ox + 110, oz) === 12, `${cost(ox + 110, oz)}`);
check('unmapped 30 m from a crater cell stays 1.5 /m', cost(ox + 30, oz) === 1.5, `${cost(ox + 30, oz)}`);
check('unmapped 10 m from a plain FREE cell stays 1.5 /m', cost(ox + 210, oz) === 1.5, `${cost(ox + 210, oz)}`);
check('mapped cells are not inflated', Math.abs(cost(ox + 0.5, oz + 0.5) - 6) < 0.01, `${cost(ox + 0.5, oz + 0.5)} (haz 120 -> 6)`);
// A* around the pocket: both the unknown strip in front of the crater and the open ground are unmapped; the path to a goal behind
// the crater must stay outside the 15 m ring if a detour of comparable length exists
const r = P.plan(ox - 60, oz - 40, ox + 60, oz + 40);
const inRing = r ? r.pts.filter((q) => Math.hypot(q.x - (ox + 0.5), q.z - (oz + 0.5)) < 15).length : -1;
check('A* routes around the 15 m unmapped ring of a crater', !!r && inRing === 0, r ? `${inRing} path points inside the ring` : 'no path');

// ---- 2. loaded smoothing keeps R >= 18 m
P.unknownNearHazardCost = 1.5;
S.clear();
// boulder fields (lethal disks) force bends in the plans
P.blocked = [{ x: 0, z: -150, r: 30, t: 0, fault: true }, { x: 120, z: -250, r: 25, t: 0, fault: true }, { x: -150, z: -200, r: 22, t: 0, fault: true }];
P.buildCostmap([]);
const rover = sim.rover;
const legs = [[-250, -150, 250, -160], [-100, -300, 220, -120], [-280, -280, 250, -230], [-60, -80, 200, -300]];
const planWith = (loaded) => {
  Object.defineProperty(rover, 'loaded', { get: () => loaded, configurable: true });
  return legs.map(([a, b, c, d]) => P.plan(a, b, c, d));
};
const empty = planWith(false), loadedPlans = planWith(true);
delete rover.loaded;
check('all test legs planned', empty.every(Boolean) && loadedPlans.every(Boolean), `${empty.filter(Boolean).length}/${legs.length} empty, ${loadedPlans.filter(Boolean).length}/${legs.length} loaded`);
if (!empty.every(Boolean) || !loadedPlans.every(Boolean)) process.exit(1);
let worstLoaded = Infinity, worstEmpty = Infinity;
for (const p of loadedPlans) worstLoaded = Math.min(worstLoaded, Planner.pathMinRadius(p.pts));
for (const p of empty) worstEmpty = Math.min(worstEmpty, Planner.pathMinRadius(p.pts));
console.log(`min radius: empty ${worstEmpty.toFixed(1)} m, loaded ${worstLoaded.toFixed(1)} m`);
// the arcs are exact R = 18 m; the estimate fits circles through points 3 m apart on a 1 m resampled path (~3 % low)
atLeast('loaded planned centre line: minimum turning radius (R_min 18 m, estimator tolerance 1 m)', worstLoaded, 17.0, ' m', 1);
check('empty plans are not filleted (point turns allowed)', worstEmpty < worstLoaded, `empty ${worstEmpty.toFixed(1)} m < loaded ${worstLoaded.toFixed(1)} m`);

// ---- 3. route straightening (v1.7.0): chords only over mapped, clear ground, never closer to an obstacle than the hard margin
{
  S.clear(); P.blocked = []; P.unknownNearHazardCost = 12;
  const x0 = 200, z0 = -400, LEN = 240, HW = 40;          // a flat mapped corridor 240 m x 80 m
  const cell = (wx, wz) => { const ix = Math.floor(wx / RES), iz = Math.floor(wz / RES), tt = S.getTile(Math.floor(ix / TILE), Math.floor(iz / TILE), true); return [tt, (iz - tt.tz * TILE) * TILE + (ix - tt.tx * TILE)]; };
  for (let x = x0; x < x0 + LEN; x += RES) for (let z = z0 - HW; z < z0 + HW; z += RES) {
    const [tt, i] = cell(x, z); tt.n[i] = 5; tt.h[i] = 0; tt.h0[i] = 0; tt.v[i] = 0.001; tt.haz[i] = 20; tt.cls[i] = CLS.FREE; tt.dirty = false;
  }
  const bx = x0 + 120, bz = z0; // a 3 m boulder in the middle of the corridor
  for (let x = bx - 1.5; x <= bx + 1.5; x += RES) for (let z = bz - 1.5; z <= bz + 1.5; z += RES) { const [tt, i] = cell(x, z); tt.h[i] = 2.0; tt.h0[i] = 2.0; tt.haz[i] = 255; tt.cls[i] = CLS.ROCK; }
  // the route as a first exploration drives it: a wide zigzag that swings round the boulder
  const zig = []; for (let s = 0; s <= LEN - 10; s += 1) zig.push({ x: x0 + 5 + s, z: z0 + 22 * Math.sin(s / 18), t: s });
  for (const loaded of [false, true]) {
    P.inflate = loaded ? 6.5 : 5.8;
    S.setPayload(loaded);
    P.buildCostmap([]);
    const zigLen = zig.reduce((a, q, k) => a + (k ? Math.hypot(q.x - zig[k - 1].x, q.z - zig[k - 1].z) : 0), 0);
    const st = P.straighten(zig, loaded);
    const tag = loaded ? 'loaded' : 'empty';
    check(`straighten (${tag}): shorter than the zigzag`, !!st && st.length < 0.8 * zigLen, st ? `${st.length.toFixed(0)} m vs ${zigLen.toFixed(0)} m` : 'none');
    if (!st) continue;
    const near = Math.min(...st.pts.map((q) => Math.hypot(q.x - bx, q.z - bz)));
    atLeast(`straighten (${tag}): clears the boulder by the hard margin`, near, P.inflate, ' m', 1);
    check(`straighten (${tag}): every point on mapped ground`, st.pts.every((q) => Math.abs(q.z - z0) < HW && q.x >= x0 && q.x < x0 + LEN));
    check(`straighten (${tag}): keeps both ends`, Math.hypot(st.pts[0].x - zig[0].x, st.pts[0].z - zig[0].z) < 0.01 && Math.hypot(st.pts.at(-1).x - zig.at(-1).x, st.pts.at(-1).z - zig.at(-1).z) < 0.01);
    if (loaded) atLeast('straighten (loaded): corners rounded at R >= 18 m', Planner.pathMinRadius(st.pts), 17.0, ' m', 1);
  }
  // a stretch over unmapped ground is left as driven
  const out = [{ x: x0 + 5, z: z0 }, { x: x0 + 5, z: z0 + 80 }, { x: x0 + 100, z: z0 + 80 }, { x: x0 + 100, z: z0 }];
  const raw = P.resample(out, 1.0), st2 = P.straighten(raw, false);
  check('straighten: no chord across unmapped ground', !st2 || st2.length >= raw.length - 5, st2 ? `${st2.length.toFixed(0)} m (driven ${raw.length - 1} m)` : 'none');
  S.setPayload(false);
}
