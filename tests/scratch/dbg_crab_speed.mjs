// Crab angle vs commanded speed, straight (wz = 0), flat pad, each from a standing start and after 4 s of settling.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER), R = sim.rover, D = 57.2958, wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const run = (sec, fn) => { for (let i = 0; i < sec * 120; i++) { sim.stepOnce(); fn && fn(); } };
run(4); R.brake = false;
const out = [];
for (const vx of (process.argv[2] || '0.04,0.06,0.08,0.10,0.12,0.14,0.16,0.20,0.30,0.50').split(',').map(Number)) {
  R.cmd.vx = 0; R.cmd.wz = 0; run(3); R.cmd.vx = vx; run(4);
  const p0 = R.pose(); let hd = 0, n = 0, vlat = 0, fy = 0; run(6, () => { const p = R.pose(), lv = R.body.linvel(); hd += p.yaw; n++; vlat += lv.x * Math.sin(p.yaw) + lv.z * Math.cos(p.yaw); fy += R.wheels.reduce((a, w) => a + (w.Fy || 0), 0); });
  const p1 = R.pose(), travel = Math.atan2(-(p1.z - p0.z), p1.x - p0.x);
  out.push(`v ${vx.toFixed(2)}: crab ${(wrap(travel - hd / n) * D).toFixed(2)} deg (actual ${(Math.hypot(p1.x - p0.x, p1.z - p0.z) / 6).toFixed(3)} m/s, body lateral v ${(vlat / n * 1000).toFixed(1)} mm/s, sum Fy ${(fy / n).toFixed(0)} N, yaw drift ${(wrap(p1.yaw - p0.yaw) * D).toFixed(2)} deg)`);
}
console.log(out.join('\n'));
