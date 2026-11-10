// A/B: SLAM error on the pad->hab drive with LiDAR scan matching on vs off (MATCH=0).
// Prints total error and its along-track part (negative = estimate behind the true position).
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover, H = sim.terrain.hab;
if (process.env.MATCH === '0') sim.slam.matching = false;
A.start('goto', A.missionGoto(WORLD.habitat.x + H.nx * 45, WORLD.habitat.z + H.nz * 45, 'hab'));
const tmax = +(process.argv[2] || 300); let last = 0;
while (A.gen && sim.time < tmax) {
  sim.stepOnce();
  if (sim.time - last >= 30) { last = sim.time; const p = R.pose(), e = sim.slam.est;
    const fx = Math.cos(p.yaw), fz = -Math.sin(p.yaw), dx = e.x - p.x, dz = e.z - p.z;
    console.log(`t=${sim.time.toFixed(0)} dist ${R.odo.dist.toFixed(0)} err ${Math.hypot(dx, dz).toFixed(2)} along ${(dx * fx + dz * fz).toFixed(2)} yawErr ${((e.yaw - p.yaw) * 57.3).toFixed(2)}`); }
}
