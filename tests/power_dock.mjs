// Power dock (prototype-scale electrics, CHANGES.md / docs/LUNAR_CHARGING.md): rover teleported near the habitat at 30 % SoC,
// autonomous "charge" mission over the inductive pad, then bank day-refill, cold-hold and a full-night soak.
// Pass: the charge mission finishes OK and reaches 90 %; it takes 3.5-5.5 h (the same 0.14 C CC-CV as the old 60 kW
// heat-limited charger: 1.4 A into the 10 Ah prototype pack, tapering above 80 %); peak current <= 1.4 A (~21 kW into the 150 kWh
// pack, capped at the 20 kW charger); energy delivered to the 128 Wh pack matches the SoC gain; the pack stays under 50 C; the link releases and
// the rover backs off; the 2 x 126 W arrays refill the 2.4 kWh bank in under a day; a cold bank is not charged; the bank
// carries the dock's own night load for a full 354 h lunar night and the vault stays above its heater set point.
// Segmented inductive pad (docs/CHARGING_DESIGN.md, src/sim/chargingParams.js): the autonomous park lands the receiver inside the
// full-power box with the kneeled air gap in range and the link at >= 0.88 efficiency; the link carries the rover bus load as
// well as the charge (pass-through); receiver and pad coils stay under their derate temperature; at ride height (gap too large)
// or off the pad the link does not engage; parked and kneeled on the pad at night with the bank under the 50 % charge reserve,
// the dock carries the rover's load (night park, capped at 600 W full-scale) and the bank carries a 154 W survival load
// through a full 354 h night without reaching its floor.
import { check, within, atLeast, atMost } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER, POWER_DOCK, PROTO_PACK } from '../src/sim/config.js';
import { CHARGING } from '../src/sim/chargingParams.js';
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
const t0 = sim.time; let last = 0, maxTb = -1e9, maxChargeA = 0, maxDockW = 0, maxTrx = -1e9, maxTga = -1e9, eng = null; const bus0 = D.busWh;
// 30 -> 90 % at 0.14 C takes ~4.5 h
while (A.gen && sim.time - t0 < 20000) {
  sim.stepOnce(); maxTb = Math.max(maxTb, sim.health.th.Tb); maxChargeA = Math.max(maxChargeA, D.chargeA); maxDockW = Math.max(maxDockW, R.power.dockW);
  maxTrx = Math.max(maxTrx, D.Trx); maxTga = Math.max(maxTga, D.Tga);
  if (!eng && D.connected && D.chargeW > 0) eng = { ...D.align, eff: D.linkEff, frac: D.linkFrac };
  if (sim.time - last > 60) { last = sim.time; console.log(`  ${A.phase} | ${A.detail.slice(0, 120)}`); }
}
const rec = A.history[A.history.length - 1];
check('charge mission finished OK', !A.gen && rec && rec.ok, rec ? `ok=${rec.ok}` : 'no mission record (timed out)');
atLeast('state of charge after the charge', R.power.soc * 100, 89, ' %', 1);
within('time to charge 30 -> 90 % at 0.14 C', (sim.time - t0) / 3600, 3.5, 5.5, ' h', 2);
atMost('peak charge current into the prototype pack (CC limit)', maxChargeA, POWER_DOCK.chargeCC_A + 0.01, ' A', 2);
atMost('peak full-scale equivalent charge power (capped at the 20 kW charger, ROVER.battery.dockMaxW)', maxDockW / 1000, ROVER.battery.dockMaxW / 1000 + 0.01, ' kW', 1);
{ const dSoc = R.power.soc - 0.3; within('prototype Wh delivered / 128 Wh vs state-of-charge gain', (D.deliveredWh - del0) / PROTO_PACK.Wh, dSoc, dSoc * 1.15, '', 3); }
atLeast('dock bank state of charge after the day charge (solar covers it)', D.soc, bank0 - 0.01, '', 3);
atMost('peak pack temperature while charging', maxTb, 50, ' C', 1);
check('coupler released and rover clear of the pad', !D.connected && !D.roverOnPad(), `connected ${D.connected}`);
// inductive link at engagement (autonomous park): receiver inside the full-power box, kneeled gap in range, efficiency
check('link engaged during the mission', !!eng, eng ? 'engaged' : 'never coupled');
if (eng) {
  console.log(`engaged: receiver along ${eng.along.toFixed(3)} m, across ${eng.across.toFixed(3)} m, yaw ${(eng.dyaw * 180 / Math.PI).toFixed(2)} deg, gap ${(eng.gap * 1000).toFixed(0)} mm, eff ${eng.eff.toFixed(3)}, coupling ${eng.frac.toFixed(2)}`);
  atMost('receiver along-track offset at engagement (full-power box)', Math.abs(eng.along), CHARGING.tol.along, ' m', 3);
  atMost('receiver cross-track offset at engagement (full-power box)', Math.abs(eng.across), CHARGING.tol.across, ' m', 3);
  within('kneeled air gap at engagement', eng.gap * 1000, CHARGING.gap.min * 1000, CHARGING.gap.max * 1000, ' mm', 0);
  atLeast('link efficiency at engagement (DC dock bus to DC rover bus)', eng.eff, 0.88, '', 3);
}
// pass-through: the link also carried the rover bus load, so the pack got the whole charge current
atLeast('rover bus energy carried by the link during the charge (full-scale)', (D.busWh - bus0) * D.scale / 1000, 0.1, ' kWh', 2);
atMost('peak receiver coil temperature (derate starts at 90 C)', maxTrx, CHARGING.thermal.rx.derateC, ' C', 1);
atMost('peak pad coil temperature (derate starts at 90 C)', maxTga, CHARGING.thermal.ga.derateC, ' C', 1);
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
// ---- interface geometry: no link at ride height (gap too large) or off the pad; link when kneeled on the centre
const settle = (sec) => { for (let i = 0; i < sec * 120; i++) sim.stepOnce(); };
const onPad = (across) => {
  const c = Math.cos(L.yaw), sn = Math.sin(L.yaw); // pad right unit (sin yaw, cos yaw)
  R.lift.target = 0; R.teleport(L.pad.x + sn * across, L.pad.z + c * across, L.yaw); sim.slam.relocalize(R.pose()); R.brake = true; settle(4);
};
onPad(0);
{ const al = D.padAlignment(); console.log(`on the pad centre at ride 0: gap ${(al.gap * 1000).toFixed(0)} mm, status ${D.status}`);
  check('no link at ride height (air gap too large)', !D.connected && al.gap > CHARGING.gap.max, `gap ${(al.gap * 1000).toFixed(0)} mm, connected ${D.connected}`); }
R.lift.target = ROVER.ride.kneel; settle(12);
{ const al = D.padAlignment(); console.log(`kneeled on the pad centre: gap ${(al.gap * 1000).toFixed(0)} mm, eff ${D.linkEff.toFixed(3)}, status ${D.status}`);
  check('link engages kneeled on the pad centre', D.connected, `gap ${(al.gap * 1000).toFixed(0)} mm, status ${D.status}`);
  within('kneeled air gap on the pad centre (design 75 mm)', al.gap * 1000, 40, 110, ' mm', 0); }
// ---- night park: no sun, bank below the 50 % charge reserve: no charging, but the dock carries the rover's bus load
D.bankWh = 0.45 * POWER_DOCK.bankWh; settle(10);
{ const want = Math.max(0, R.power.loadW - R.power.solarW - (R.power.auxSolarW || 0));
  console.log(`night park: rover load ${want.toFixed(0)} W, dock carries ${(D.busW * D.scale).toFixed(0)} W, charge ${D.chargeW.toFixed(2)} W proto, status ${D.status}`);
  check('night park: no charging below the night reserve', D.night && D.chargeW === 0, `night ${D.night}, chargeW ${D.chargeW}`);
  check('night park: dock carries the rover bus load', D.keepAlive && D.connected, D.status);
  within('night park: link power vs rover load (capped at 600 W)', D.busW * D.scale, Math.min(want, CHARGING.night.keepAliveMaxW) * 0.9, Math.min(want, CHARGING.night.keepAliveMaxW) * 1.1 + 1, ' W', 0); }
// a full 354 h night parked with the 154 W survival load (docs/POWER_BUDGET.md) from the 50 % reserve (bank-only, D.update(60))
D.bankWh = POWER_DOCK.nightReserveSoc * POWER_DOCK.bankWh; D.Tbank = POWER_DOCK.vault.initC;
console.log('night park endurance estimate', D.nightParkHours(154).toFixed(0), 'h');
atLeast('night park endurance from the 50 % reserve with a 154 W rover (lunar night 354 h)', D.nightParkHours(154), 354, ' h', 0);
{ const b0 = D.busWh;
  for (let i = 0; i < 21240; i++) { R.power.loadW = 154; R.power.solarW = 0; R.power.auxSolarW = 0; D.update(60); }
  const kWh = (D.busWh - b0) * D.scale / 1000;
  console.log(`after a 354 h night park: bank ${(D.soc * 100).toFixed(1)} %, rover load carried ${kWh.toFixed(1)} kWh, vault ${D.Tbank.toFixed(1)} C, status ${D.status}`);
  within('rover survival energy carried by the dock over the night (154 W x 354 h = 54.5 kWh)', kWh, 54.5 * 0.97, 54.5 * 1.001, ' kWh', 1);
  atLeast('bank state of charge after a 354 h night park', D.soc, POWER_DOCK.bankFloor, '', 3);
  check('still carrying the rover at the end of the night', D.keepAlive, D.status); }
// off the pad across the track (receiver 0.8 m off the centreline, capture limit 0.6 m): no link even kneeled
onPad(0.8); R.lift.target = ROVER.ride.kneel; settle(12);
check('no link with the receiver outside the capture envelope (0.8 m across)', !D.connected && !D.roverOnPad(), `across ${D.align.across.toFixed(2)} m, status ${D.status}`);
