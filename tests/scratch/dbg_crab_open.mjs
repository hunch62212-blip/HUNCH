// Open-loop crab test: commanded (vx, wz) for 8 s each; reports the angle between the direction of travel and the heading.
// argv[2]: 'pad' (flat landing pad apron) or 'stand' (the lane behind CP-36C);  argv[3]: ride height
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
await RAPIER.init();
const where = process.argv[2] || 'stand', ride = Number(process.argv[3] ?? ROVER.ride.kneel);
const sim = new Sim(RAPIER), R = sim.rover, c = sim.cargo[2], ct = c.body.translation(), cy = Math.cos(c.yaw), sy = Math.sin(c.yaw), D = 57.2958;
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
if (where === 'stand') R.teleport(ct.x - cy * 30, ct.z + sy * 30, c.yaw);
const run = (sec, fn) => { for (let i = 0; i < sec * 120; i++) { sim.stepOnce(); fn && fn(); } };
run(4); R.brake = false; R.lift.target = ride; run(8);
for (const [vx, wz] of [[0.12, 0], [0.12, 0.009], [0.12, -0.009], [0.4, 0], [0.4, 0.009], [0.06, 0.004], [0.12, 0]]) {
  R.cmd.vx = vx; R.cmd.wz = wz; run(2);
  const p0 = R.pose(); let yr = 0, n = 0, hd = 0; run(6, () => { yr += R.body.angvel().y; hd += R.pose().yaw; n++; });
  const p1 = R.pose(), travel = Math.atan2(-(p1.z - p0.z), p1.x - p0.x), headMean = hd / n;
  console.log(`${where} ride ${ride.toFixed(2)} cmd vx ${vx} wz ${(wz * 1000).toFixed(0)} mrad/s: moved ${(Math.hypot(p1.x - p0.x, p1.z - p0.z)).toFixed(2)} m, crab (travel - heading) ${(wrap(travel - headMean) * D).toFixed(2)} deg, yaw rate achieved ${(yr / n * 1000).toFixed(1)} mrad/s, roll ${(R.tilt.roll * D).toFixed(2)} pitch ${(R.tilt.pitch * D).toFixed(2)}`);
}
