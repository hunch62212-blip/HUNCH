// Mission 9: SCOUT-1 leads SPAN-40 along the route, maps ahead and keeps separation.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { runCommand } from '../src/sim/commands.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const S = sim.scout, R = sim.rover;
runCommand(sim, {}, 'scout on'); runCommand(sim, {}, 'goto hab');
let minSep = 1e9, last = 0;
while (sim.time < 300) {
  sim.stepOnce();
  const p = R.pose(); const sep = Math.hypot(S.x - p.x, S.z - p.z); if (sim.time > 20) minSep = Math.min(minSep, sep);
  if (sim.time - last > 30) { last = sim.time; console.log(`t=${sim.time.toFixed(0)} scout ${S.mode} v=${S.v.toFixed(2)} sep=${sep.toFixed(1)} m  mapped=${S.cellsMapped} cells  soc=${(S.soc * 100).toFixed(1)}%  pitch=${(S.pitch * 57.3).toFixed(1)}  | ${S.msg.slice(0, 70)}`); }
  if (!Number.isFinite(S.x)) { console.log('NaN!'); break; }
}
console.log('min separation after start', minSep.toFixed(1), 'm; scout odometer', S.odo.toFixed(0), 'm');
