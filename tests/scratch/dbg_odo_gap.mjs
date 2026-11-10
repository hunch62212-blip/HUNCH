// Per-second gap between true planar motion and SLAM dead-reckoning motion on the pad->hab drive.
// Prints seconds where the gap exceeds 0.3 m, with what the rover was doing.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover, H = sim.terrain.hab;
sim.slam.matching = false;
A.start('goto', A.missionGoto(WORLD.habitat.x + H.nx * 45, WORLD.habitat.z + H.nz * 45, 'hab'));
const tmax = +(process.argv[2] || 240);
let p0 = R.pose(), e0 = { ...sim.slam.est }, tot = 0, byKind = {};
while (A.gen && sim.time < tmax) {
  sim.stepOnce();
  if (sim.stepCount % 120 === 0) {
    const p = R.pose(), e = sim.slam.est;
    const tdx = p.x - p0.x, tdz = p.z - p0.z, edx = e.x - e0.x, edz = e.z - e0.z;
    const gap = Math.hypot(tdx - edx, tdz - edz); tot += gap;
    const tl = R.telemetry();
    const kind = `${A.phase}|${(A.detail || '').split(/[:(0-9]/)[0].trim()}`;
    byKind[kind] = (byKind[kind] || 0) + gap;
    if (gap > 0.3) console.log(`t=${sim.time.toFixed(0)} gap ${gap.toFixed(2)} true ${Math.hypot(tdx, tdz).toFixed(2)} est ${Math.hypot(edx, edz).toFixed(2)} v ${tl.fwdSpeed.toFixed(2)} brake ${R.brake} | ${A.detail}`);
    p0 = p; e0 = { ...e };
  }
}
console.log('total gap', tot.toFixed(1));
for (const [k, v] of Object.entries(byKind).sort((a, b) => b[1] - a[1]).slice(0, 8)) console.log(v.toFixed(1).padStart(7), k);
