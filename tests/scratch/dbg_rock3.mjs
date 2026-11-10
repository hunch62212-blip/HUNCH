import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); sim.modules = sim.modules.filter((m) => m !== sim.autonomy);
if (process.env.NOPREC) { sim._precisionPre = () => {}; sim._precisionPost = () => {}; }
const R = sim.rover, sp = sim.terrain.pad, p0 = R.pose();
R.teleport(p0.x - sp.perp.x * 15, p0.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); for (let i = 0; i < 240; i++) sim.stepOnce();
const p = R.pose(), right = { x: -p.fwd.z, z: p.fwd.x };
const i = sim.addBoulder(p.x + p.fwd.x * 12 + right.x * 2.75, p.z + p.fwd.z * 12 + right.z * 2.75, 2.2);
if (process.env.PRESTEP) sim.stepOnce();
if (process.env.TOUCH) { sim.stepOnce(); const c = sim.rockColliders.get(i); c.setTranslation(c.translation()); }
R.brake = false; R.cmd.vx = 0.6;
const W = R.wheels[1];
for (let k = 0; k < 120 * 16; k++) {
  sim.stepOnce();
  const q = R.pose(), al = (q.x - p.x) * p.fwd.x + (q.z - p.z) * p.fwd.z;
  if (al > 5.5 && k % 30 === 0) {
    let pairs = 0; sim.world.contactPairsWith(sim.rockColliders.get(i), () => pairs++);
    console.log(`along ${al.toFixed(2)} FR contact ${W.contact} cpY ${(W.cp.y - p.y).toFixed(2)} groundY ${W.groundY == null ? 'null' : (W.groundY - p.y).toFixed(2)} blocked ${W.blocked} airT ${(W.airT||0).toFixed(2)} rockPairs ${pairs}`);
    if (al > 8.5) break;
  }
}
