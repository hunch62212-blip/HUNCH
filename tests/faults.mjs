// Mission 7 + 10: autonomous 2 km traverse to the habitat with a blocked road and a failed wheel.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { runCommand } from '../src/sim/commands.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover;
sim.on('log', (e) => { if (!/Local planner blocked/.test(e.msg)) console.log(`[${(e.t / 60).toFixed(1).padStart(5)} min] ${e.level.padEnd(4)} ${e.msg}`); });
runCommand(sim, {}, 'goto hab');
let last = 0, injected = 0;
while (A.gen && sim.time < 3600) {
  sim.stepOnce();
  if (!injected && sim.time > 300) { injected = 1; runCommand(sim, {}, 'fault block'); runCommand(sim, {}, 'fault wheel 3'); }
  if (sim.time - last > 120) { last = sim.time; const p = R.pose(), e = sim.slam.est; console.log(`   t=${(sim.time / 60).toFixed(0)}min ${A.detail.slice(0, 90)} | slamErr ${Math.hypot(e.x - p.x, e.z - p.z).toFixed(2)} m | match ${sim.slam.lastMatch.ok}`); }
}
const p = R.pose();
console.log('final pos', p.x.toFixed(0), p.z.toFixed(0), 'summary', JSON.stringify(A.summary()), 'routes', sim.slam.routes.map((r) => `${r.from}->${r.to} ${r.length.toFixed(0)}m`).join(','));
