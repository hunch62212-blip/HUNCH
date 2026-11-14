// Does the parked rover hold its heading? (spawn pose, brakes set, no command)
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER), R = sim.rover;
const deg = (r) => r * 180 / Math.PI;
const p0 = R.pose();
for (let s = 0; s <= 60; s += 5) {
  const p = R.pose(), av = R.body.angvel();
  console.log(`t=${String(s).padStart(2)} yaw ${deg(p.yaw).toFixed(2)} dpos ${Math.hypot(p.x - p0.x, p.z - p0.z).toFixed(3)} m  wy ${av.y.toFixed(5)} rad/s  loads ${R.wheels.map((w) => (w.load / 1000).toFixed(2)).join(' ')}  slip ${R.wheels.map((w) => (w.slip * 100).toFixed(0)).join(' ')}  level ${R.bogies.map((b) => b.level.toFixed(2)).join(' ')} roll ${deg(R.tilt.roll).toFixed(2)}`);
  for (let i = 0; i < 600; i++) sim.stepOnce();
}
