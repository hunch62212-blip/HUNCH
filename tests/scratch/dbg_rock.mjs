import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { heightAt } from '../../src/sim/terrain.js';
await RAPIER.init();
const sim = new Sim(RAPIER); sim.modules = sim.modules.filter((m) => m !== sim.autonomy);
const R = sim.rover, sp = sim.terrain.pad, p0 = R.pose();
R.teleport(p0.x - sp.perp.x * 15, p0.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); for (let i = 0; i < 240; i++) sim.stepOnce();
const p = R.pose(), right = { x: -p.fwd.z, z: p.fwd.x };
const bx = p.x + p.fwd.x * 12 + right.x * 2.75, bz = p.z + p.fwd.z * 12 + right.z * 2.75;
const i = sim.addBoulder(bx, bz, 2.2); const r = sim.terrain.rocks[i];
sim.stepOnce();
const ray = new RAPIER.Ray({ x: bx, y: r.y + 10, z: bz }, { x: 0, y: -1, z: 0 });
const hit = sim.world.castRay(ray, 20, true);
console.log('rover', p.x.toFixed(1), p.z.toFixed(1), 'ground', heightAt(sim.terrain, p.x, p.z).toFixed(2), 'body y', p.y.toFixed(2), '| rock idx', i, 'skip', r.skip, 'collider', sim.rockColliders.has(i), 'rock y', r.y.toFixed(2), 'top', r.top?.toFixed(2), '| ray hit at y', hit ? (r.y + 10 - hit.timeOfImpact).toFixed(2) : 'none', hit && sim.colliderRock.get(hit.collider.handle));
const pd = sim.terrain.pad; console.log('pad', JSON.stringify(sim.terrain.pad.roverStart), 'dist rover->pad centre', Math.hypot(p.x - (-841.2), p.z - (-540.8)).toFixed(1));
R.brake = false; R.cmd.vx = 0.6;
for (let k = 0; k < 120 * 40; k++) {
  sim.stepOnce();
  if (k % 240 === 0) { const q = R.pose(); const al = (q.x - p.x) * p.fwd.x + (q.z - p.z) * p.fwd.z, lat = (q.x - p.x) * right.x + (q.z - p.z) * right.z;
    console.log(`t ${(k / 120).toFixed(0)} along ${al.toFixed(1)} lat ${lat.toFixed(2)} yaw ${((q.yaw - p.yaw) * 57.3).toFixed(1)} y ${(q.y - p.y).toFixed(2)} wheels ${R.wheels.map((w) => (w.cp.y - p.y).toFixed(2)).join(' ')} blocked ${R.wheels.map(w=>w.blocked?1:0).join('')}`); }
}
