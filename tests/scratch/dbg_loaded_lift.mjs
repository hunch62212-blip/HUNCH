// Loaded lift trajectory: autonomous approach, then rise to carry; strut extension (rideActual) and the module height over time.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
await RAPIER.init();
const ci = Number(process.argv[2] || 0);
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover, c = sim.cargo[ci], T = c.T, ct = c.body.translation();
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
A.start('approach', A.acquire(c, { approachOnly: true })); let t0 = sim.time; while (A.gen && sim.time - t0 < 600) sim.stepOnce();
const y0 = c.body.translation().y, C = ROVER.ride.carry; R.lift.target = C;
let over = 0, tIn = null, lastOut = 0, vMax = 0, yPrev = y0, mMax = 0, rows = [];
for (let i = 1; i <= 14 * 120; i++) { sim.stepOnce(); const t = i / 120, ra = R.rideActual, y = c.body.translation().y;
  over = Math.max(over, ra - C); if (Math.abs(ra - C) > 0.02) lastOut = t; vMax = Math.max(vMax, (y - yPrev) * 120); yPrev = y; mMax = Math.max(mMax, y - y0);
  if (i % 60 === 0) rows.push(`${t.toFixed(1)}s cmd ${R.lift.pos.toFixed(2)} struts ${ra.toFixed(2)} module +${(y - y0).toFixed(2)}`); }
console.log(rows.join(' | '));
console.log(`${c.id}: struts within 2 cm of carry for good after ${lastOut.toFixed(2)} s | strut overshoot ${(over * 100).toFixed(1)} cm | module rose ${(c.body.translation().y - y0).toFixed(2)} m (peak ${mMax.toFixed(2)}) at up to ${vMax.toFixed(2)} m/s`);
