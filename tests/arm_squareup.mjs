// Multi-seed square-up-only check for the arm pick-up (autonomyArm._acmSquareUp). Same scene as tests/arm_pickup.mjs
// (ACM 5-8 m past the nose, +-0.6 m lateral, +-4 deg yaw), seeds from SEEDS="2-14" (default 2-12), FIRST attempt only
// (no back-off retry). Passes when the rover stops inside ACM_TOL lateral/yaw at the grasp point.
import RAPIER from '@dimforge/rapier3d-compat';
import { check } from './lib/check.mjs';
import { Sim } from '../src/sim/sim.js';
import { ROVER } from '../src/sim/config.js';
import { ACM_TOL } from '../src/sim/autonomyArm.js';
await RAPIER.init();
const DEG = Math.PI / 180;
const [lo, hi] = (process.env.SEEDS || '2-12').split('-').map(Number);
for (let s0 = lo; s0 <= (hi || lo); s0++) {
  let seed = s0; const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
  const sim = new Sim(RAPIER), R = sim.rover, A = sim.autonomy;
  const P = sim.terrain.pad.roverStart, yaw = P.yaw;
  R.teleport(P.x, P.z, yaw); for (let i = 0; i < 480; i++) sim.stepOnce();
  const gap = 5 + 3 * rnd(), off = (rnd() - 0.5) * 1.2, dyaw = (rnd() - 0.5) * 8 * DEG;
  const fwd = { x: Math.cos(yaw), z: -Math.sin(yaw) }, rgt = { x: Math.sin(yaw), z: Math.cos(yaw) };
  const d = ROVER.frame.halfLen + gap;
  const c = sim.acm.spawn(P.x + fwd.x * d + rgt.x * off, P.z + fwd.z * d + rgt.z * off, yaw + dyaw, { mass: 2400 });
  for (let i = 0; i < 360; i++) sim.stepOnce();
  R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose());
  let ok = null;
  const gen = (function* () { yield* A._acmStandoff(c); R.brake = false; R.crawlZone = true; ok = yield* A._acmSquareUp(c); R.crawlZone = false; return { ok: true }; }).call(A);
  A.start('square-up', gen); const t0 = sim.time;
  while (A.gen && sim.time - t0 < 300) sim.stepOnce();
  const e = A.acmSense(c);
  const good = ok === true && Math.abs(e.r) <= ACM_TOL.lat && Math.abs(e.yawE) <= ACM_TOL.yaw;
  check(`seed ${s0}: squares up on the first attempt (lat ${(e.r * 100).toFixed(1)} cm, yaw ${(e.yawE / DEG).toFixed(2)} deg, ${(sim.time - t0).toFixed(0)} s)`, good);
}
