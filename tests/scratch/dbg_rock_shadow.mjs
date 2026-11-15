// Does extending the float64 position guard break chassis-vs-rock contact? rock_climb's 1.4 m boulder case at several guard limits and speeds.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
for (const lim of (process.argv[2] || '0.1,0.45,1,99').split(',').map(Number)) for (const v of [0.3, 0.6]) {
  globalThis.__SHADOW_MAX = lim;
  const sim = new Sim(RAPIER); sim.modules = sim.modules.filter((m) => m !== sim.autonomy);
  const R = sim.rover, sp = sim.terrain.pad, p0 = R.pose();
  R.teleport(p0.x - sp.perp.x * 15, p0.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); for (let i = 0; i < 240; i++) sim.stepOnce();
  const p = R.pose(), right = { x: -p.fwd.z, z: p.fwd.x };
  sim.addBoulder(p.x + p.fwd.x * 12 + right.x * 2.75, p.z + p.fwd.z * 12 + right.z * 2.75, 2.2);
  R.brake = false; R.cmd.vx = v;
  for (let k = 0; k < 120 * 24 / v * 0.9; k++) sim.stepOnce();
  const q = R.pose(), along = (q.x - p.x) * p.fwd.x + (q.z - p.z) * p.fwd.z;
  console.log(`guard limit ${lim} m/s, drive ${v} m/s into the 1.4 m boulder at 12 m: travelled ${along.toFixed(1)} m -> ${along < 12 ? 'BLOCKED (ok)' : 'PASSED THROUGH'}`);
}
