// Headless autonomy run at max speed, no rendering. Modes: m1 (leave landing zone), m2 (approach),
// m3 (approach + pickup), cycle (Mission 6 full logistics cycle).
// Pass: mission finished (not timed out) and reported OK; m2/m3 end with the rover under / holding the module;
// cycle delivers every cargo item that starts at the pad.
import { check, equal } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER, process.env.SEED ? { seed: Number(process.env.SEED) } : {});
const A = sim.autonomy, R = sim.rover;
sim.on('log', (e) => console.log(`[${(e.t / 60).toFixed(1).padStart(6)} min] ${e.level.padEnd(4)} ${e.msg}`));
const which = process.argv[2] || 'cycle';
const gens = { m1: () => A.missionLeaveLanding(), m2: () => A.missionApproach(sim.cargo[0]), m3: () => A.missionPickup(sim.cargo[0]), cycle: () => A.missionCycle() };
A.start(which, gens[which]());
const t0 = Date.now();
let lastPrint = 0;
const maxSim = Number(process.argv[3] || 7200);
// WARP=1: run as the browser does at 50x (transit warp on long cruise legs, full physics elsewhere)
const warp = !!process.env.WARP;
if (warp) sim.timeScale = 50;
while (A.gen && sim.time < maxSim) {
  if (warp) sim.stepAuto(); else sim.stepOnce();
  if (sim.time - lastPrint > Number(process.env.PRINT || 60)) {
    lastPrint = sim.time;
    const p = R.pose(), e = sim.slam.est;
    console.log(`   t=${(sim.time / 60).toFixed(1)}min ${A.phase} | ${A.detail} | pos (${p.x.toFixed(0)},${p.z.toFixed(0)}) slamErr ${Math.hypot(e.x - p.x, e.z - p.z).toFixed(2)}m yawErr ${(Math.atan2(Math.sin(e.yaw - p.yaw), Math.cos(e.yaw - p.yaw)) * 57.3).toFixed(2)}deg | soc ${(R.power.soc * 100).toFixed(1)}% | tiles ${sim.slam.tiles.size} | plan ${A.lastPlan ? A.lastPlan.ms.toFixed(0) + '+' + A.lastPlan.buildMs.toFixed(0) + 'ms' : '-'} | wall ${((Date.now() - t0) / 1000).toFixed(0)}s`);
  }
}
console.log('SUMMARY', JSON.stringify(A.summary()), 'history', JSON.stringify(A.history));
console.log('routes', sim.slam.routes.map((r) => `${r.id} ${r.from}->${r.to} ${r.length.toFixed(0)}m`).join(', '));
console.log('wall s', (Date.now() - t0) / 1000, 'sim s', sim.time.toFixed(0), warp ? `warp steps ${sim.warp.steps} (${(sim.warp.simS / 60).toFixed(1)} sim min on rails)` : '');
const last = A.history[A.history.length - 1];
check(`${which}: mission finished before ${maxSim} s sim cap`, !A.gen, `sim ${sim.time.toFixed(0)} s`);
check(`${which}: mission reported OK`, !!(last && last.ok), last ? `ok=${last.ok}` : 'no mission record');
if (which === 'm3') check('m3: cargo held', R.loaded, `cargo state ${sim.cargo[0].state}`);
if (which === 'cycle') equal('cycle: deliveries', A.summary().deliveries ?? last?.deliveries, sim.cargo.length);
