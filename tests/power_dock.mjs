// Power dock (prototype-scale electrics, CHANGES.md / docs/LUNAR_CHARGING.md): rover teleported near the habitat at 30 % SoC,
// autonomous "charge" mission over the inductive pad, then bank day-refill, cold-hold and a full-night soak.
// Pass: the charge mission finishes OK and reaches 90 %; it takes 3.5-5.5 h (the same 0.14 C CC-CV as the old 60 kW
// heat-limited charger: 1.4 A into the 10 Ah prototype pack, tapering above 80 %); peak current <= 1.4 A (<= 70 kW
// full-scale); energy delivered to the 128 Wh pack matches the SoC gain; the pack stays under 50 C; the link releases and
// the rover backs off; the 2 x 126 W arrays refill the 2.4 kWh bank in under a day; a cold bank is not charged; the bank
// carries the dock's own night load for a full 354 h lunar night and the vault stays above its heater set point.
import { check, within, atLeast, atMost } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER, POWER_DOCK, PROTO_PACK } from '../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover, D = sim.powerDock, L = D.layout;
sim.on('log', (e) => { if (!/Local planner blocked/.test(e.msg)) console.log(`[${e.t.toFixed(0).padStart(5)} s] ${e.level.padEnd(4)} ${e.msg}`); });
R.teleport(L.entry.x + 8, L.entry.z + 8, L.yaw); for (let i = 0; i < 480; i++) sim.stepOnce(); R.brake = false;
sim.slam.clear(); sim.slam.relocalize(R.pose()); sim.streamRocks([R.pose()]);
R.power.wh = 0.3 * ROVER.battery.capacityWh; R.power.soc = 0.3;
console.log('dock solar W', D.solarW.toFixed(0), 'bank', (D.soc * 100).toFixed(1));
const bank0 = D.soc, del0 = D.deliveredWh;
A.start('charge', A.missionCharge(0.9));
const t0 = sim.time; let last = 0, maxTb = -1e9, maxChargeA = 0, maxDockW = 0;
// 30 -> 90 % at 0.14 C takes ~4.5 h
while (A.gen && sim.time - t0 < 20000) { sim.stepOnce(); maxTb = Math.max(maxTb, sim.health.th.Tb); maxChargeA = Math.max(maxChargeA, D.chargeA); maxDockW = Math.max(maxDockW, R.power.dockW); if (sim.time - last > 60) { last = sim.time; console.log(`  ${A.phase} | ${A.detail.slice(0, 120)}`); } }
const rec = A.history[A.history.length - 1];
check('charge mission finished OK', !A.gen && rec && rec.ok, rec ? `ok=${rec.ok}` : 'no mission record (timed out)');
atLeast('state of charge after the charge', R.power.soc * 100, 89, ' %', 1);
within('time to charge 30 -> 90 % at 0.14 C', (sim.time - t0) / 3600, 3.5, 5.5, ' h', 2);
atMost('peak charge current into the prototype pack (CC limit)', maxChargeA, POWER_DOCK.chargeCC_A + 0.01, ' A', 2);
atMost('peak full-scale equivalent charge power', maxDockW / 1000, 70, ' kW', 1);
{ const dSoc = R.power.soc - 0.3; within('prototype Wh delivered / 128 Wh vs state-of-charge gain', (D.deliveredWh - del0) / PROTO_PACK.Wh, dSoc, dSoc * 1.15, '', 3); }
atLeast('dock bank state of charge after the day charge (solar covers it)', D.soc, bank0 - 0.01, '', 3);
atMost('peak pack temperature while charging', maxTb, 50, ' C', 1);
check('coupler released and rover clear of the pad', !D.connected && !D.roverOnPad(), `connected ${D.connected}`);
console.log(`CHARGE ${R.power.soc >= 0.89 ? 'OK' : 'FAILED'} soc ${(R.power.soc * 100).toFixed(1)}% in ${((sim.time - t0) / 60).toFixed(1)} min, bank ${(D.soc * 100).toFixed(1)}%`);
// day refill: rover clear of the pad, bank at its floor, a full day of sun
sim.setSun(sim.sunAz, 9);
D.bankWh = POWER_DOCK.bankFloor * POWER_DOCK.bankWh; D.Tbank = POWER_DOCK.vault.initC;
for (let i = 0; i < 1440; i++) D.update(60);
console.log('day refill: solar', D.solarW.toFixed(0), 'W, bank', (D.soc * 100).toFixed(1), '%, refill time', D.refillHours().toFixed(1), 'h');
atLeast('bank state of charge after 24 h of sun from the floor', D.soc, 0.99, '', 3);
atMost('bank refill time from the 10 % floor', D.refillHours(), 24, ' h', 1);
// cold bank: LiFePO4 takes no charge below 0 C; the MPPT backs off while the vault heater warms it
{ D.Tbank = -10; const w0 = D.bankWh; for (let i = 0; i < 10; i++) D.update(1);
  check('cold bank is not charged (below 0 C)', D.bankWh <= w0 + 1e-9 && D.bankCold && D.curtailed, `bank ${D.bankWh.toFixed(2)} Wh, Tbank ${D.Tbank.toFixed(1)} C, status ${D.status}`);
  check('vault heater runs when the bank is cold', D.heaterW > 0, `heater ${D.heaterW.toFixed(1)} W`); }
D.Tbank = POWER_DOCK.vault.initC;
// night endurance
sim.setSun(sim.sunAz, -6); for (let i = 0; i < 240; i++) sim.stepOnce();
console.log('night: solar', D.solarW.toFixed(1), 'W, dock load', D.loadW.toFixed(2), 'W, bank carries survival load for', D.nightHours().toFixed(0), 'h (lunar night 354 h)');
atMost('dock solar input at night', D.solarW, POWER_DOCK.nightSolarW, ' W', 1);
atLeast('bank endurance on the dock night load (lunar night 354 h)', D.nightHours(), 354, ' h', 0);
// full 354 h night from the night reserve, bank drawn down by rover charging beforehand
D.bankWh = POWER_DOCK.nightReserveSoc * POWER_DOCK.bankWh;
for (let i = 0; i < 21240; i++) D.update(60);
console.log('after a 354 h night from the reserve: bank', (D.soc * 100).toFixed(1), '%, vault', D.Tbank.toFixed(1), 'C');
atLeast('bank state of charge after a 354 h night from the 50 % reserve', D.soc, POWER_DOCK.bankFloor, '', 3);
atLeast('vault temperature after the night', D.Tbank, POWER_DOCK.vault.setC - 2, ' C', 1);
