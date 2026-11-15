// Pick-up lift and loading-bay stand exit (v1.7.0), CM-40A.
// Lift: the saddles must not slam the module (soft seat); the struts themselves must be at the carry stroke before anything moves.
// Exit: straight out along the module axis toward the habitat (every stand points its module there), heading held, no stand contact.
// Pass: module jolt during the rise <= 6 m/s^2 (one fast stroke gave 19; the pre-1.7.0 servo 7.2); the exit phase runs with the struts
// (not just the set point) at carry; >= 11.5 m forward; heading within 1 deg, lateral drift <= 0.15 m; <= 50 N of stand contact on
// the module; then the loaded rover is 100 m from the stand within 4 sim-min with no "boxed in" recovery.
import { check, atLeast, atMost } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER, MISSION } from '../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover;
const c = sim.cargo[0], T = c.T, ct = c.body.translation();
const warns = []; sim.on('log', (e) => { if (e.level === 'warn' || e.level === 'err') warns.push(e.msg); });
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false;
sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear();
sim.streamRocks([{ x: ct.x, z: ct.z }]);
A.start('pickup', A.acquire(c));
let t0 = sim.time, jolt = 0, pv = 0, riseT = 0;
while (A.gen && sim.time - t0 < 1500) { sim.stepOnce(); const v = c.body.linvel().y; if (/^M3 Rise/.test(A.phase)) { jolt = Math.max(jolt, Math.abs(v - pv) * 120); riseT += 1 / 120; } pv = v; }
check('pickup completed', R.loaded, `${(sim.time - t0).toFixed(0)} s sim, rise phase ${riseT.toFixed(1)} s`);
if (!R.loaded) process.exit(1);
atMost('module jolt while the saddles take it (soft seat)', jolt, 6, ' m/s2', 1);
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const p0 = R.pose(), ax = { x: Math.cos(p0.yaw), z: -Math.sin(p0.yaw) }, nW = warns.length;
A.start('deliver', A.deliver(c.dest));
let inExit = false, done = false, strutsAtStart = null, minProg = 0, maxProg = 0, maxYaw = 0, maxLat = 0, standF = 0, tExit = null, t100 = null;
t0 = sim.time;
while (A.gen && sim.time - t0 < 300) {
  sim.stepOnce();
  const p = R.pose(), dx = p.x - p0.x, dz = p.z - p0.z, prog = dx * ax.x + dz * ax.z;
  if (!done && /^Exit .* stand/.test(A.phase)) {
    if (!inExit) { inExit = true; tExit = sim.time; }
    if (strutsAtStart == null && prog > 0.05) strutsAtStart = R.rideActual;        // struts when the vehicle first moves
    minProg = Math.min(minProg, prog); maxProg = Math.max(maxProg, prog);
    maxYaw = Math.max(maxYaw, Math.abs(wrap(p.yaw - p0.yaw))); maxLat = Math.max(maxLat, Math.abs(dx * ax.z - dz * ax.x));
    const f = sim.cargoForces.get(c); if (f) standF = Math.max(standF, f.total);
  } else if (inExit) done = true;
  if (t100 == null && Math.hypot(dx, dz) >= 100) { t100 = sim.time - t0; break; }
}
check('stand-exit phase ran', inExit);
atLeast('struts (not the set point) at the carry stroke when the vehicle first moved', strutsAtStart ?? -9, ROVER.ride.carry - 0.05, ' m', 3);
atLeast('distance out along the module axis, toward the habitat', MISSION.standExitDir * maxProg, MISSION.standExitM - 0.5, ' m', 1);
atMost('distance the wrong way', -MISSION.standExitDir * minProg, 0.3, ' m', 2);
atMost('heading change during the exit', maxYaw * 180 / Math.PI, 1, ' deg', 2);
atMost('lateral drift during the exit', maxLat, 0.15, ' m', 2);
atMost('stand contact force on the module during the exit', standF, 50, ' N', 0);
check('loaded rover 100 m from the stand', t100 != null, t100 != null ? `${(t100 / 60).toFixed(1)} min after the clamp` : 'not within 5 min');
atMost('time to get 100 m clear', t100 ?? 999, 240, ' s', 0);
check('no boxed-in recovery on the way out', !warns.slice(nW).some((w) => /Boxed in/.test(w)), `${warns.length - nW} warning(s)`);
console.log(`exit: ${(sim.time - tExit).toFixed(0)} s to 100 m, ${maxProg.toFixed(1)} m straight, heading ${(maxYaw * 180 / Math.PI).toFixed(2)} deg, drift ${(maxLat * 100).toFixed(0)} cm, stand contact ${standF.toFixed(0)} N`);
