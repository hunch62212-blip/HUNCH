import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const R = sim.rover, S = sim.sensors;
const run = (sec) => { for (let i = 0; i < sec * 120; i++) sim.stepOnce(); };
run(4); R.brake = false;
for (const v of [1, 2, 3.5, 4.9]) { const d0 = R.odo.dist, o0 = S.odom.dist; R.cmd.vx = v; run(25); const d1 = R.odo.dist, o1 = S.odom.dist; // ramp included
  console.log(`v ${v}: true ${(d1 - d0).toFixed(1)} odo ${(o1 - o0).toFixed(1)} ratio ${((o1 - o0) / (d1 - d0)).toFixed(3)} slip ${R.wheels.map((w) => w.slip.toFixed(3)).join(' ')}`); }
