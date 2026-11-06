// Pickup timing: rover placed 30 m behind the module on its axis, then the autonomous acquire sequence.
// Prints the sim time spent in each phase so pick-up speed changes can be measured.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER } from '../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover;
const c = sim.cargo[Number(process.argv[2] || 0)], T = c.T;
sim.on('log', (e) => { if (!/Local planner blocked/.test(e.msg)) console.log(`[${e.t.toFixed(0).padStart(5)} s] ${e.level.padEnd(4)} ${e.msg}`); });
const ct = c.body.translation();
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false;
sim.slam.clear(); sim.slam.relocalize(R.pose());
sim.streamRocks([{ x: ct.x, z: ct.z }]);
const t0 = sim.time;
A.start('pickup timing', A.acquire(c));
const phases = new Map(); let ph = null, phT = sim.time;
while (A.gen && sim.time - t0 < 1500) {
  sim.stepOnce();
  if (A.phase !== ph) { if (ph) phases.set(ph, (phases.get(ph) || 0) + sim.time - phT); ph = A.phase; phT = sim.time; }
}
if (ph) phases.set(ph, (phases.get(ph) || 0) + sim.time - phT);
for (const [k, v] of phases) console.log(`  ${k.padEnd(40)} ${v.toFixed(1)} s`);
console.log(`PICKUP ${R.loaded ? 'OK' : 'FAILED'} in ${(sim.time - t0).toFixed(1)} s sim, cargo state ${c.state}`);
process.exit(R.loaded ? 0 : 1);
