// Power dock: rover teleported near the habitat at 30 % SoC, autonomous "charge" mission, then a night soak.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover, D = sim.powerDock, L = D.layout;
sim.on('log', (e) => { if (!/Local planner blocked/.test(e.msg)) console.log(`[${e.t.toFixed(0).padStart(5)} s] ${e.level.padEnd(4)} ${e.msg}`); });
R.teleport(L.entry.x + 8, L.entry.z + 8, L.yaw); for (let i = 0; i < 480; i++) sim.stepOnce(); R.brake = false;
sim.slam.clear(); sim.slam.relocalize(R.pose()); sim.streamRocks([R.pose()]);
R.power.wh = 0.3 * 420000; R.power.soc = 0.3;
console.log('dock solar kW', (D.solarW / 1000).toFixed(1), 'bank', (D.soc * 100).toFixed(1));
A.start('charge', A.missionCharge(0.9));
const t0 = sim.time; let last = 0;
while (A.gen && sim.time - t0 < 2400) { sim.stepOnce(); if (sim.time - last > 60) { last = sim.time; console.log(`  ${A.phase} | ${A.detail.slice(0, 120)}`); } }
console.log(`CHARGE ${R.power.soc >= 0.89 ? 'OK' : 'FAILED'} soc ${(R.power.soc * 100).toFixed(1)}% in ${((sim.time - t0) / 60).toFixed(1)} min, bank ${(D.soc * 100).toFixed(1)}%`);
// night endurance check (analytic from the model)
sim.setSun(sim.sunAz, -6); for (let i = 0; i < 240; i++) sim.stepOnce();
console.log('night: solar', D.solarW.toFixed(0), 'W, dock load', D.loadW, 'W, bank carries survival load for', D.nightHours().toFixed(0), 'h (lunar night 354 h)');
