// Mission 7 + 10: autonomous 2 km traverse to the habitat with a blocked road and a failed wheel.
// Pass: both faults injected, wheel 3 stays failed, the rover still reaches the habitat within 3600 s sim,
// and the successful leg is stored as a route.
import { check } from './lib/check.mjs';
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
  if (!injected && sim.time > 300) { injected = 1; runCommand(sim, {}, 'fault block'); runCommand(sim, {}, 'fault wheel 3'); injected = sim.terrain.rocks.length; }
  if (sim.time - last > 120) { last = sim.time; const p = R.pose(), e = sim.slam.est; console.log(`   t=${(sim.time / 60).toFixed(0)}min ${A.detail.slice(0, 90)} | slamErr ${Math.hypot(e.x - p.x, e.z - p.z).toFixed(2)} m | match ${sim.slam.lastMatch.ok}`); }
}
const p = R.pose();
console.log('final pos', p.x.toFixed(0), p.z.toFixed(0), 'summary', JSON.stringify(A.summary()), 'routes', sim.slam.routes.map((r) => `${r.from}->${r.to} ${r.length.toFixed(0)}m`).join(','));
const rec = A.history[A.history.length - 1];
check('faults injected (boulder field + wheel 3)', injected > 1 && R.wheels[2].fault === 'motor', `wheel 3 fault ${R.wheels[2].fault}`);
check('reached habitat despite faults', !A.gen && !!(rec && rec.ok), `${A.gen ? 'still running' : 'finished'} at ${(sim.time / 60).toFixed(1)} min`);
check('route stored for the leg', sim.slam.routes.length > 0, `${sim.slam.routes.length} route(s)`);
