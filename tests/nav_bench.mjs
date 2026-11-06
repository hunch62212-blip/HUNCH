// Navigation benchmark: pad -> habitat (exploring), then habitat -> pad (should reuse the route).
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { runCommand } from '../src/sim/commands.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover;
let warns = 0;
sim.on('log', (e) => { if (e.level !== 'info') { if (e.level === 'warn') warns++; console.log(`[${(e.t / 60).toFixed(1).padStart(5)} min] ${e.level.padEnd(4)} ${e.msg}`); } });
const legs = (process.argv[2] || 'hab,pad').split(',');
const t0 = Date.now();
for (const leg of legs) {
  const s0 = sim.time, d0 = R.odo.dist; warns = 0;
  runCommand(sim, {}, `goto ${leg}`);
  let last = sim.time;
  while (A.gen && sim.time - s0 < 3600) {
    sim.stepOnce();
    const dense = process.env.DENSE && leg === 'pad' && sim.time - s0 > Number(process.env.DENSE);
    if (sim.time - last > (dense ? 6 : 180)) { last = sim.time; const p = R.pose(), e = sim.slam.est; console.log(`   ${(sim.time / 60).toFixed(1)} min | ${A.phase.slice(0, 40)} | ${A.detail.slice(0, 110)} | true (${p.x.toFixed(0)},${p.z.toFixed(0)}) est (${e.x.toFixed(0)},${e.z.toFixed(0)}) yaw ${(e.yaw * 57.3).toFixed(0)} | slamErr ${Math.hypot(e.x - p.x, e.z - p.z).toFixed(1)} m`); }
  }
  const ok = A.history.length && A.history[A.history.length - 1].ok;
  const dt = sim.time - s0, dd = R.odo.dist - d0;
  console.log(`LEG ${leg}: ${ok ? 'OK' : 'FAIL'} time ${(dt / 60).toFixed(1)} min, dist ${dd.toFixed(0)} m, avg ${(dd / dt).toFixed(2)} m/s, warnings ${warns}, wall ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  if (!ok) break;
}
