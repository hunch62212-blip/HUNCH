// Parked stillness numbers for the headless.mjs thresholds: same placement as headless.mjs (15 m off the pad edge)
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { MOON } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER), R = sim.rover;
{ const sp = sim.terrain.pad, p = R.pose(); R.teleport(p.x - sp.perp.x * 15, p.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); }
for (let i = 0; i < 480; i++) sim.stepOnce();
for (let rep = 0; rep < 3; rep++) {
  const p0 = R.pose(); let lo = Infinity, hi = -Infinity, maxW = 0, maxV = 0, maxSlip = 0;
  for (let i = 0; i < 20 * 120; i++) { sim.stepOnce(); for (const w of R.wheels) { lo = Math.min(lo, w.load); hi = Math.max(hi, w.load); maxSlip = Math.max(maxSlip, Math.abs(w.slip)); } maxW = Math.max(maxW, Math.abs(R.body.angvel().y)); const v = R.body.linvel(); maxV = Math.max(maxV, Math.hypot(v.x, v.z)); }
  const p1 = R.pose(), mean = R.mass * MOON.g / 6;
  console.log(`20 s parked: yaw drift ${(Math.abs(p1.yaw - p0.yaw) * 57.2958).toFixed(4)} deg, pos drift ${(Math.hypot(p1.x - p0.x, p1.z - p0.z) * 1000).toFixed(1)} mm, peak yaw rate ${(maxW * 1000).toFixed(2)} mrad/s, peak speed ${(maxV * 1000).toFixed(1)} mm/s, load spread ${((hi - lo) / mean).toFixed(3)} (${lo.toFixed(0)}..${hi.toFixed(0)} N), max |slip| ${maxSlip.toFixed(3)}`);
}
