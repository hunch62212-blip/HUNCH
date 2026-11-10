import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { Planner } from '../../src/sim/planner.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const P = sim.autonomy.planner; sim.slam.clear();
P.blocked = [{ x: 0, z: -150, r: 30, t: 0, fault: true }, { x: 120, z: -250, r: 25, t: 0, fault: true }, { x: -150, z: -200, r: 22, t: 0, fault: true }];
P.buildCostmap([]);
Object.defineProperty(sim.rover, 'loaded', { get: () => true, configurable: true });
for (const [a,b,c,d] of [[-250, -150, 250, -160], [-100, -300, 220, -120], [-280, -280, 250, -230], [-60, -80, 200, -300]]) {
  const r = P.plan(a,b,c,d); const m = Planner.pathMinRadius(r.pts);
  let at = 0, mm = Infinity; for (let i=0;i+6<r.pts.length;i++){ const x=Planner.pathMinRadius(r.pts.slice(i,i+7)); if (x<mm){mm=x;at=i;} }
  console.log(m.toFixed(1), JSON.stringify(P.lastSmooth), 'worst at', at, r.pts[at+3].x.toFixed(0), r.pts[at+3].z.toFixed(0), 'n', r.pts.length);
}
