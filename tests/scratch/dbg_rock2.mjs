import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); sim.modules = sim.modules.filter((m) => m !== sim.autonomy);
const R = sim.rover, sp = sim.terrain.pad, p0 = R.pose();
R.teleport(p0.x - sp.perp.x * 15, p0.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); for (let i = 0; i < 240; i++) sim.stepOnce();
const p = R.pose(), right = { x: -p.fwd.z, z: p.fwd.x };
const bx = p.x + p.fwd.x * 12 + right.x * 2.75, bz = p.z + p.fwd.z * 12 + right.z * 2.75;
const i = sim.addBoulder(bx, bz, 2.2); const r = sim.terrain.rocks[i];
const col0 = sim.rockColliders.get(i);
console.log('after add: collider handle', col0?.handle, 'valid', col0 && sim.world.getCollider(col0.handle) != null);
R.brake = false; R.cmd.vx = 0.6;
for (let k = 0; k < 120 * 20; k++) {
  sim.stepOnce();
  if (k === 0 || k === 1 || k === 130 || k % 600 === 0) {
    const c = sim.rockColliders.get(i);
    const hit = sim.world.castRay(new RAPIER.Ray({ x: bx, y: r.y + 10, z: bz }, { x: 0, y: -1, z: 0 }), 20, true);
    console.log(`k ${k} t ${sim.time.toFixed(2)} collider ${c?.handle} same ${c === col0} ray-> ${hit ? sim.colliderRock.get(hit.collider.handle) ?? 'terrain/other' : 'none'} streamT ${sim._streamT.toFixed(2)}`);
  }
}
