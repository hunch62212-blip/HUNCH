// Mission 9: SCOUT-1 leads AEGIS-V1 along the route, maps ahead and keeps separation.
// Pass: state stays finite; never closer than 7 m (AEGIS-V1 half-diagonal ~6.9 m); maps cells; gets ahead
// of AEGIS-V1 along the route at some point in 300 s.
import { check, atLeast } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { runCommand } from '../src/sim/commands.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const S = sim.scout, R = sim.rover;
runCommand(sim, {}, 'scout on'); runCommand(sim, {}, 'goto hab');
let minSep = 1e9, last = 0, nan = false, maxAhead = -1e9;
while (sim.time < 300) {
  sim.stepOnce();
  const p = R.pose(); const sep = Math.hypot(S.x - p.x, S.z - p.z); if (sim.time > 20) minSep = Math.min(minSep, sep); maxAhead = Math.max(maxAhead, (S.x - p.x) * p.fwd.x + (S.z - p.z) * p.fwd.z);
  if (sim.time - last > 30) { last = sim.time; console.log(`t=${sim.time.toFixed(0)} scout ${S.mode} v=${S.v.toFixed(2)} sep=${sep.toFixed(1)} m  mapped=${S.cellsMapped} cells  soc=${(S.soc * 100).toFixed(1)}%  pitch=${(S.pitch * 57.3).toFixed(1)}  | ${S.msg.slice(0, 70)}`); }
  if (!Number.isFinite(S.x)) { nan = true; break; }
}
console.log('min separation after start', minSep.toFixed(1), 'm; scout odometer', S.odo.toFixed(0), 'm');
check('scout state finite', !nan, nan ? 'NaN position' : 'ok');
atLeast('min separation after start', minSep, 7, ' m', 1);
atLeast('cells mapped by scout', S.cellsMapped, 1, '', 0);
atLeast('scout gets ahead of AEGIS-V1 (max lead along heading)', maxAhead, 10, ' m', 1);
