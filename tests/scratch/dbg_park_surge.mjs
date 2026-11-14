// Parked rover at rest: sample the fore-aft surge, pitch, wheel forces and strut state at 20 Hz
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER), R = sim.rover;
for (let i = 0; i < 120 * 20; i++) sim.stepOnce();
for (let k = 0; k < 50; k++) {
  for (let i = 0; i < 6; i++) sim.stepOnce();
  const p = R.pose(), lv = R.body.linvel(), av = R.body.angvel();
  const vf = lv.x * p.fwd.x + lv.z * p.fwd.z, vr = -lv.x * p.fwd.z + lv.z * p.fwd.x;
  console.log(`t=${sim.time.toFixed(2)} vf ${(vf * 1000).toFixed(1)} vr ${(vr * 1000).toFixed(1)} vy ${(lv.y * 1000).toFixed(1)} mm/s pitch ${(R.tilt.pitch * 57296).toFixed(0)} mdeg pitchRate ${(Math.hypot(av.x, av.z) * 1000).toFixed(2)} mrad/s Fx ${R.wheels.map((w) => w.Fx.toFixed(0)).join(' ')} | N ${R.wheels.map((w) => (w.load).toFixed(0)).join(' ')} | anch ${R.wheels.map((w) => (w.anchor ? 1 : 0)).join('')} Ldot ${R.bogies.map((b) => (b.Ldot * 1000).toFixed(1)).join(' ')}`);
}
