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
