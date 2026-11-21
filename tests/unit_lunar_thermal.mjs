// Lunar thermal / mechanism unit checks (docs/changes/B.md). No Sim (pure models).
import { MechNode } from '../src/sim/actuators.js';
import { ArmSystem } from '../src/sim/arm.js';
import { MECH, SINK, SOLAR, RAD_ALPHA } from '../src/sim/lunarParams.js';
let fail = 0; const ok = (c, m) => { console.log(`${c ? 'PASS' : 'FAIL'} ${m}`); if (!c) fail++; };
const night = { sinkK: (SINK.viewSky * SINK.regNightK ** 4) ** 0.25, solarWm2: 0 };
const soak = (n, h, env) => { for (let t = 0; t < h * 3600; t += 10) n.step(10, env); return n.T; };
for (const k of ['joint', 'latch', 'strut']) {
  const cold = soak(new MechNode(MECH[k], MECH.lockC), 14 * 24, { ...night, heaterOK: false });
  const warm = new MechNode(MECH[k], MECH.lockC); soak(warm, 14 * 24, { ...night, heaterOK: true });
  ok(cold < MECH.lockC, `${k}: 14 d night, no heater -> ${cold.toFixed(1)} C < ${MECH.lockC} C (locks)`);
  ok(warm.T > MECH.lockC && !warm.locked, `${k}: ${MECH[k].heaterW} W heater holds ${warm.T.toFixed(1)} C`);
  const dutyW = (() => { let e = 0; for (let t = 0; t < 86400; t += 10) { warm.step(10, { ...night, heaterOK: true }); e += warm.heatW * 10; } return e / 86400; })();
  ok(dutyW <= MECH[k].heaterW && dutyW > 0, `${k}: ${MECH[k].heaterW} W heater, night mean ${dutyW.toFixed(1)} W`);
}
const day = new MechNode(MECH.joint, MECH.lockC); soak(day, 24, { sinkK: night.sinkK, solarWm2: SOLAR.S0 * 0.15, heaterOK: false });
ok(day.T > MECH.lockC, `joint in low sun, no heater: ${day.T.toFixed(1)} C stays above lockout`);
// arm lockout
const arm = new ArmSystem();
arm.thermalEnv = { ...night, heaterOK: false };
for (let t = 0; t < 14 * 86400; t += 10) arm._mechThermal(10, {});
ok(arm.coldLock, `arm cold-soaked 14 d without heaters -> coldLock (coldest ${arm.coldestMech().name} ${arm.coldestMech().T.toFixed(0)} C)`);
const r = arm.moveTo('stow'); ok(r && r.ok === false && /cold lockout/.test(r.reason), `moveTo refused: ${r && r.reason}`);
ok(arm.latch(true)?.ok === false, 'latch refused under cold lockout');
arm.thermalEnv = { ...night, heaterOK: true };
let eJ = 0, tClear = null;
for (let t = 0; t < 72 * 3600; t += 10) { arm._mechThermal(10, {}); eJ += arm.mechHeaterW * 10; if (tClear == null && !arm.coldLock) tClear = t; }
ok(!arm.coldLock && tClear != null, `heaters on: lockout clears after ${(tClear / 3600).toFixed(1)} h (arm heaters mean ${(eJ / 72 / 3600).toFixed(1)} W over 72 h)`);
// radiator solar load (health.js formula): one 12 m^2 vault side face-on to a 9 deg sun
const q = (a, cos) => a * SOLAR.S0 * 12 / 2 * cos;
ok(q(RAD_ALPHA.clean, 1) > 700 && Math.abs(q(RAD_ALPHA.fins, 1) / q(RAD_ALPHA.clean, 1) - 5) < 1e-9, `vault side face-on: ${q(RAD_ALPHA.clean, 1).toFixed(0)} W clean, ${q(RAD_ALPHA.fins, 1).toFixed(0)} W dusty-fins`);
console.log(fail ? `${fail} FAIL` : 'all PASS'); process.exit(fail ? 1 : 0);
