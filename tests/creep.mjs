// low-speed creep check on open ground
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const R = sim.rover;
const run = (sec, fn) => { const n = Math.round(sec * 120); for (let i = 0; i < n; i++) { fn && fn(); sim.stepOnce(); } };
run(4); R.brake = false;
for (const v of [0.3, 0.08, 0.04, 0.025, 0.01]) {
  R.cmd.vx = v; run(6); const p0 = R.pose(); run(4); const p1 = R.pose();
  const lv = R.body.linvel();
  console.log(`cmd ${v} actual ${(Math.hypot(p1.x - p0.x, p1.z - p0.z) / 4).toFixed(4)} m/s linvel ${Math.hypot(lv.x, lv.z).toFixed(4)} wheel r*om ${R.wheels.map((w) => (w.omega * 1.061).toFixed(3)).join(' ')} slip ${R.wheels.map((w) => w.slip.toFixed(2)).join(' ')} T ${R.wheels.map((w) => w.torque.toFixed(0)).join(' ')}`);
}
