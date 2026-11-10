// Slip-compensated wheel odometry vs ground truth at four speeds, on open ground.
// Pass: rover actually covers >= 70 % of v x 15 s (it was not blocked), odometry / truth within 5 %.
// Setup change vs the original: start 15 m off the pad edge (as headless.mjs does) and 15 s legs instead of
// 25 s; the original drove 280 m straight from the spawn point into an obstacle at 3.5 m/s.
import { atLeast, within } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const R = sim.rover, S = sim.sensors;
{ const sp = sim.terrain.pad, p = R.pose(); R.teleport(p.x - sp.perp.x * 15, p.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); }
const run = (sec) => { for (let i = 0; i < sec * 120; i++) sim.stepOnce(); };
const LEG = 15;
run(4); R.brake = false;
for (const v of [0.5, 1, 2, 2.9]) {
  const d0 = R.odo.dist, o0 = S.odom.dist; R.cmd.vx = v; run(LEG); // ramp included
  const truth = R.odo.dist - d0, odo = S.odom.dist - o0;
  console.log(`v ${v}: true ${truth.toFixed(1)} odo ${odo.toFixed(1)} ratio ${(odo / truth).toFixed(3)} slip ${R.wheels.map((w) => w.slip.toFixed(3)).join(' ')}`);
  atLeast(`rover moved at ${v} m/s (true dist / v x ${LEG} s)`, truth / (v * LEG), 0.7);
  within(`odometry / truth at ${v} m/s`, odo / truth, 0.95, 1.05);
}
