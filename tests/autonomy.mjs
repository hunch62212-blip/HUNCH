// Headless autonomy run: full logistics cycle (Mission 6) at max speed, no rendering.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover;
sim.on('log', (e) => console.log(`[${(e.t / 60).toFixed(1).padStart(6)} min] ${e.level.padEnd(4)} ${e.msg}`));
const which = process.argv[2] || 'cycle';
const gens = { m1: () => A.missionLeaveLanding(), m2: () => A.missionApproach(sim.cargo[0]), m3: () => A.missionPickup(sim.cargo[0]), cycle: () => A.missionCycle() };
A.start(which, gens[which]());
const t0 = Date.now();
let lastPrint = 0;
const maxSim = Number(process.argv[3] || 7200);
while (A.gen && sim.time < maxSim) {
  sim.stepOnce();
  if (sim.time - lastPrint > 60) {
    lastPrint = sim.time;
    const p = R.pose(), e = sim.slam.est;
    console.log(`   t=${(sim.time / 60).toFixed(1)}min ${A.phase} | ${A.detail} | pos (${p.x.toFixed(0)},${p.z.toFixed(0)}) slamErr ${Math.hypot(e.x - p.x, e.z - p.z).toFixed(2)}m yawErr ${((e.yaw - p.yaw) * 57.3).toFixed(2)}deg | soc ${(R.power.soc * 100).toFixed(1)}% | tiles ${sim.slam.tiles.size} | plan ${A.lastPlan ? A.lastPlan.ms.toFixed(0) + '+' + A.lastPlan.buildMs.toFixed(0) + 'ms' : '-'} | wall ${((Date.now() - t0) / 1000).toFixed(0)}s`);
  }
}
console.log('SUMMARY', JSON.stringify(A.summary()), 'history', JSON.stringify(A.history));
console.log('routes', sim.slam.routes.map((r) => `${r.id} ${r.from}->${r.to} ${r.length.toFixed(0)}m`).join(', '));
console.log('wall s', (Date.now() - t0) / 1000, 'sim s', sim.time.toFixed(0));
