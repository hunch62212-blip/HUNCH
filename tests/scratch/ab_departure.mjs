// A/B: loaded departure from the CM-40A stand. Pick-up, then deliver(); stop when the rover is 200 m (truth) from the stand or after 25 sim-min.
// Reports the exit phase (time, straightness, stand contact force on the module) and how long the loaded rover needs to get clear of the pad.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover;
const c = sim.cargo[Number(process.argv[2] || 0)], T = c.T, ct = c.body.translation();
const warns = []; sim.on('log', (e) => { if (e.level === 'warn' || e.level === 'err') warns.push(`[${(e.t / 60).toFixed(1)} min] ${e.msg.slice(0, 110)}`); });
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
A.start('pickup', A.acquire(c)); let t0 = sim.time; while (A.gen && sim.time - t0 < 1500) sim.stepOnce();
if (!R.loaded) { console.log('PICKUP FAILED'); process.exit(1); }
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const p0 = R.pose(), ax = { x: Math.cos(p0.yaw), z: -Math.sin(p0.yaw) }, nW = warns.length;
A.start('deliver', A.deliver(c.dest)); t0 = sim.time;
let exitT = 0, exitF = 0, exitYaw = 0, exitLat = 0, exitProg = 0, tClear = { 30: null, 60: null, 100: null, 200: null }, wall0 = performance.now(), minMargin = 99, i = 0;
while (A.gen && sim.time - t0 < 25 * 60) {
  sim.stepOnce(); i++;
  const p = R.pose(), dx = p.x - p0.x, dz = p.z - p0.z, d = Math.hypot(dx, dz);
  if (/^Exit .* stand/.test(A.phase)) {
    exitT += 1 / 120; const f = sim.cargoForces.get(c); if (f) exitF = Math.max(exitF, f.total);
    exitYaw = Math.max(exitYaw, Math.abs(wrap(p.yaw - p0.yaw))); exitLat = Math.max(exitLat, Math.abs(dx * ax.z - dz * ax.x)); exitProg = dx * ax.x + dz * ax.z;
  }
  minMargin = Math.min(minMargin, R.tilt.margin);
  for (const k of Object.keys(tClear)) if (tClear[k] == null && d >= +k) tClear[k] = sim.time - t0;
  if (tClear[200] != null) break;
}
const f = (v) => (v == null ? 'not reached' : (v / 60).toFixed(1) + ' min');
console.log(`exit phase: ${exitT.toFixed(0)} s, progress along module axis ${exitProg.toFixed(1)} m (+ = toward the habitat), max heading change ${(exitYaw * 57.3).toFixed(2)} deg, max lateral drift ${(exitLat * 100).toFixed(0)} cm, peak stand contact force on the module ${exitF.toFixed(0)} N`);
console.log(`time from clamp to distance from the stand: 30 m ${f(tClear[30])} | 60 m ${f(tClear[60])} | 100 m ${f(tClear[100])} | 200 m ${f(tClear[200])}`);
console.log(`warnings after pickup: ${warns.length - nW}; min tip margin ${minMargin.toFixed(1)} deg; wall ${((performance.now() - wall0) / 1000).toFixed(0)} s`);
for (const w of warns.slice(nW)) console.log('  ' + w);
