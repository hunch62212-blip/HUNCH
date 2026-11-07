import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover;
A.start('cycle', A.missionCycle());
let did = false;
const origRecover = A.recover.bind(A);
A.recover = function* (goal, why) {
  if (!did && sim.time > 400) { did = true; const e = A.est(); 
    const lp = A.local.evaluate(e, { x: e.x - Math.cos(e.yaw) * 10, z: e.z + Math.sin(e.yaw) * 10 }, 0.3, R.loaded, { dir: -1, horizon: 8, kmax: 0.12, relaxed: true });
    console.log('REV eval ok', lp.ok, 'hits', lp.hits.length, 'tilt', (R.tilt.pitch*57.3).toFixed(1), (R.tilt.roll*57.3).toFixed(1), 'pose', e.x.toFixed(0), e.z.toFixed(0));
    const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
    for (const h of lp.hits.slice(0, 12)) { const dx = h.x - e.x, dz = h.z - e.z; const f = dx * c - dz * s, r = dx * s + dz * c; const wh = A.slam.wheelHazard(h.x, h.z, true); console.log(`  hit fwd ${f.toFixed(1)} right ${r.toFixed(1)} haz ${JSON.stringify(wh)} h ${A.slam.heightAt(h.x,h.z)}`); }
    const hh = A.slam.heightAt(e.x, e.z); console.log('height here', hh, 'belly pts etc');
  }
  return yield* origRecover(goal, why);
};
while (A.gen && sim.time < 520) sim.stepOnce();
