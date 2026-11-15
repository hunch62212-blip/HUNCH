// Landmark estimates and SLAM frame corrections (v1.7.0, Slam.shiftFrame warpAt / Autonomy.applyFrameShift).
// Pass rules: after a position fix, a cargo / port estimate seen before the previous fix does not move (its map tile does not
// move either); one seen since then moves with the map at its time stamp. The old rigid correction rotated every estimate about
// the current pose, so a 0.7 deg heading fix swung estimates 2 km away by ~25 m.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { check, atMost, within } from './lib/check.mjs';
await RAPIER.init();
const sim = new Sim(RAPIER), A = sim.autonomy, S = sim.slam;
const run = (sec) => { for (let i = 0; i < Math.round(sec * 120); i++) sim.stepOnce(); };
run(3);
// first fix: sets the anchor; everything seen before it is "old"
A.applyFrameShift(0.3, 0, 0);
const tOld = sim.time;
run(3);
const e = A.est();
const c0 = sim.cargo[0], c1 = sim.cargo[1];
const old = { x: e.x + 2000, z: e.z, yaw: 0.2, t: tOld - 1, src: 'test/AFT' };
const fresh = { x: e.x + 30, z: e.z, yaw: 0.2, t: sim.time, src: 'test/AFT' };
A.cargoEst.set(c0.id, { ...old }); A.cargoEst.set(c1.id, { ...fresh });
const dyaw = 0.72 * Math.PI / 180;
A.applyFrameShift(0, 0, dyaw);
const g0 = A.cargoEst.get(c0.id), g1 = A.cargoEst.get(c1.id);
atMost('estimate seen before the previous fix: not moved by a 0.72 deg heading fix 2 km away', Math.hypot(g0.x - old.x, g0.z - old.z), 0.01, ' m', 3);
check('estimate seen before the previous fix: heading unchanged', Math.abs(g0.yaw - old.yaw) < 1e-9);
const mv = Math.hypot(g1.x - fresh.x, g1.z - fresh.z);
within('estimate seen now (30 m ahead): moves with the current correction', mv, 0.33, 0.43, ' m', 3);
within('estimate seen now: heading turned by the fix', (g1.yaw - fresh.yaw) * 180 / Math.PI, 0.71, 0.73, ' deg', 3);
