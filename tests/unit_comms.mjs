// Operator link: telemetry delay line, queued E-STOP, LOS monitor (docs/changes/B.md). Mock sim, no Rapier.
import { DelayLine, LosMonitor, earthDir } from '../src/sim/commsLink.js';
import { requestEstop, runCommand } from '../src/sim/commands.js';
import { COMMS } from '../src/sim/lunarParams.js';
let fail = 0; const ok = (c, m) => { console.log(`${c ? 'PASS' : 'FAIL'} ${m}`); if (!c) fail++; };
const d = new DelayLine(8); ok(d.at(1) === undefined, 'empty line -> undefined');
for (let t = 0; t <= 20; t++) d.push(t * 0.1, t);
ok(d.at(2.0) === 20 && d.at(1.75) === 17 && d.at(1.3 + 1e-9) === 13, 'reads the newest sample at or before t');
ok(d.at(0) === 13, 'cap 8: older than the buffer returns the oldest kept');
const logs = [];
const mk = (mode, delayS) => ({ time: 0, comms: { mode, delayS, queue: [], earthDelayS: 1.3, earthDriveCap: 0.3 }, estops: 0, autonomy: { faults: {} }, rover: {},
  emergencyStop() { this.estops++; }, log: (m, l) => logs.push([m, l]) });
let s = mk('local', 0); requestEstop(s, 'test'); ok(s.estops === 1 && s.comms.queue.length === 0, 'local link: E-STOP immediate');
s = mk('earth', 2.6); const r1 = requestEstop(s, 'HUD'), r2 = requestEstop(s, 'gamepad');
ok(s.estops === 0 && s.comms.queue.length === 1 && r1.queued && r2.dup, 'Earth link: E-STOP queued once, repeats deduped');
ok(Math.abs(s.comms.queue[0].t - 2.6) < 1e-9, 'E-STOP due at t + 2.6 s');
s.comms.queue[0].fn(); ok(s.estops === 1, 'queued E-STOP latches on arrival');
s = mk('local', 0); runCommand(s, null, 'comms earth'); ok(s.comms.delayS === COMMS.earthDelayS && s.comms.mode === 'earth', `"comms earth" -> ${s.comms.delayS} s`);
ok(Math.abs(Math.hypot(...Object.values(earthDir())) - 1) < 1e-9 && earthDir().y > 0.17, 'Earth direction unit, ~10 deg up');
// LOS monitor: mock world blocks everything when `wall`
let wall = false;
const sim = { time: 0, comms: { mode: 'earth', delayS: 2.6, queue: [] }, autonomy: { faults: { comms: false } }, terrain: null, log: () => {},
  RAPIER: { Ray: class { constructor(o, d) { this.origin = { ...o }; this.dir = { ...d }; } } },
  world: { castRay: () => (wall ? { toi: 1 } : null) },
  rover: { body: {}, pose: () => ({ x: 400, y: 0, z: 400, q: { x: 0, y: 0, z: 0, w: 1 } }) } };
let restored = 0; const L = new LosMonitor(sim, { onRestore: () => restored++ });
L.check = function () { const ok2 = !wall; return (this.last = { hab: ok2, earth: ok2, ok: ok2, t: sim.time }); };   // heightAt needs a terrain: skip geometry
const tick = (n) => { for (let i = 0; i < n; i++) { L.update(); sim.time += 0.5; } };
tick(4); ok(!sim.autonomy.faults.comms, 'clear sky: no LOS');
wall = true; tick(2); ok(!sim.autonomy.faults.comms, 'one blocked check: not yet (persist 2)');
tick(2); ok(sim.autonomy.faults.comms, 'two blocked checks: COMMS LOS raised');
wall = false; tick(2); ok(!sim.autonomy.faults.comms && restored === 1, 'link back: LOS cleared, queued commands sent once');
console.log(fail ? `${fail} FAIL` : 'all PASS'); process.exit(fail ? 1 : 0);
