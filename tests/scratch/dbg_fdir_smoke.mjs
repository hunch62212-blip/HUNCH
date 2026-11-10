// Ad-hoc FDIR smoke run: a few injected failures, health report, one field repair.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { runCommand } from '../../src/sim/commands.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
sim.on('log', (e) => { if (/FDIR|SIM|REPAIR|SAFE|SURV|array|refused/i.test(e.msg)) console.log(e.t.toFixed(1), e.level, e.msg.slice(0, 220)); });
const t0 = Date.now();
const run = (s) => { const end = sim.time + s; while (sim.time < end) sim.stepOnce(); };
run(20); console.log('20 s sim took', Date.now() - t0, 'ms; heater', sim.health.heaterW.toFixed(0), 'Tv', sim.health.th.Tv.toFixed(2), 'Tb', sim.health.th.Tb.toFixed(2), 'louver', sim.health.th.louver.L.toFixed(2));
runCommand(sim, {}, 'fault mc 3'); run(2);
runCommand(sim, {}, 'fault imu A'); run(4);
runCommand(sim, {}, 'fault bat 2'); run(4);
runCommand(sim, {}, 'fault fc A'); run(10);
console.log('redund', JSON.stringify(sim.health.redund), 'capK', sim.rover.power.capK, 'soc', sim.rover.power.soc.toFixed(3));
for (const l of runCommand(sim, {}, 'health')) console.log(l.msg);
runCommand(sim, {}, 'repair mc 3'); run(16 * 60);
for (const l of runCommand(sim, {}, 'health')) console.log(l.msg);
console.log('Tv', sim.health.th.Tv.toFixed(2), 'heater', sim.health.heaterW.toFixed(0), 'wall', Date.now() - t0, 'ms');
