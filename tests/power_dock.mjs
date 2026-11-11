// Power dock: rover teleported near the habitat at 30 % SoC, autonomous "charge" mission, then a night soak.
// Pass: the charge mission finishes OK and reaches 90 %; it takes 3.5-5.5 h (252 kWh at the 60 kW heat-limited charger
// with the CC-CV taper above 80 %); the pack stays under its 50 C limit; the coupler releases and the rover backs off;
// the bank then carries the dock's own night load for a full 354 h lunar night.
import { check, within, atLeast, atMost } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER, POWER_DOCK } from '../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover, D = sim.powerDock, L = D.layout;
sim.on('log', (e) => { if (!/Local planner blocked/.test(e.msg)) console.log(`[${e.t.toFixed(0).padStart(5)} s] ${e.level.padEnd(4)} ${e.msg}`); });
R.teleport(L.entry.x + 8, L.entry.z + 8, L.yaw); for (let i = 0; i < 480; i++) sim.stepOnce(); R.brake = false;
sim.slam.clear(); sim.slam.relocalize(R.pose()); sim.streamRocks([R.pose()]);
R.power.wh = 0.3 * ROVER.battery.capacityWh; R.power.soc = 0.3;
console.log('dock solar kW', (D.solarW / 1000).toFixed(1), 'bank', (D.soc * 100).toFixed(1));
A.start('charge', A.missionCharge(0.9));
const t0 = sim.time; let last = 0, maxTb = -1e9, maxChargeW = 0;
// 30 -> 90 % at the 60 kW thermal charge limit takes ~4.5 h
while (A.gen && sim.time - t0 < 20000) { sim.stepOnce(); maxTb = Math.max(maxTb, sim.health.th.Tb); maxChargeW = Math.max(maxChargeW, D.chargeW); if (sim.time - last > 60) { last = sim.time; console.log(`  ${A.phase} | ${A.detail.slice(0, 120)}`); } }
const rec = A.history[A.history.length - 1];
check('charge mission finished OK', !A.gen && rec && rec.ok, rec ? `ok=${rec.ok}` : 'no mission record (timed out)');
atLeast('state of charge after the charge', R.power.soc * 100, 89, ' %', 1);
within('time to charge 30 -> 90 % at the heat-limited charger', (sim.time - t0) / 3600, 3.5, 5.5, ' h', 2);
atMost('peak charge power (charger limit)', maxChargeW / 1000, POWER_DOCK.chargeMaxW / 1000 + 0.01, ' kW', 1);
atMost('peak pack temperature while charging', maxTb, 50, ' C', 1);
check('coupler released and rover clear of the pad', !D.connected && !D.roverOnPad(), `connected ${D.connected}`);
console.log(`CHARGE ${R.power.soc >= 0.89 ? 'OK' : 'FAILED'} soc ${(R.power.soc * 100).toFixed(1)}% in ${((sim.time - t0) / 60).toFixed(1)} min, bank ${(D.soc * 100).toFixed(1)}%`);
// night endurance check (analytic from the model)
sim.setSun(sim.sunAz, -6); for (let i = 0; i < 240; i++) sim.stepOnce();
console.log('night: solar', D.solarW.toFixed(0), 'W, dock load', D.loadW, 'W, bank carries survival load for', D.nightHours().toFixed(0), 'h (lunar night 354 h)');
atMost('dock solar input at night', D.solarW, 500, ' W', 0);
atLeast('bank endurance on the dock night load (lunar night 354 h)', D.nightHours(), 354, ' h', 0);
