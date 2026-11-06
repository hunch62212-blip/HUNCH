import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover;
sim.on('log', (e) => console.log(`[${(e.t/60).toFixed(1)}] ${e.level} ${e.msg}`));
A.start('cycle', A.missionCycle());
let last = 0;
while (A.gen && sim.time < 900) {
  sim.stepOnce();
  if (sim.time > 250 && sim.time - last > 15) {
    last = sim.time; const p = R.pose(), P = A.path;
    const a = P && P.length ? P[Math.min(A.pathIdx + 300, P.length - 1)] : null;
    const tilt = (R.tilt ?? 0);
    console.log(`t=${sim.time.toFixed(0)} pos(${p.x.toFixed(0)},${p.z.toFixed(0)}) yaw ${(p.yaw*57.3).toFixed(0)} v ${R.speed?.toFixed?.(2) ?? ''} path ${P?P.length:0} idx ${A.pathIdx} ahead ${a?`(${a.x.toFixed(0)},${a.z.toFixed(0)})`:'-'} end ${P&&P.length?`(${P[P.length-1].x.toFixed(0)},${P[P.length-1].z.toFixed(0)})`:'-'} det ${A.detail.slice(0,60)}`);
  }
}
