// A/B: Mission 6 cycle with active roll levelling on, off (LEVEL=0), frozen while driving loaded (LEVEL=freeze),
// or with roll-rate damping added (LEVEL=pd TD=<s>: seat rate = 0.6 (err + TD d(err)/dt)). Reports peak |roll| per minute,
// time spent with fewer than 4 wheels in contact while loaded, and how the mission ends.
import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { Rover } from '../../src/sim/rover.js';
import { ROVER } from '../../src/sim/config.js';
await RAPIER.init();
if (process.env.LEVEL === 'pd') {
  const TD = +(process.env.TD || 2);
  Rover.prototype._leveling = function (dt, Q) {
    const B = ROVER.bogie, S = ROVER.susp;
    const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(Q), right = new THREE.Vector3(0, 0, 1).applyQuaternion(Q);
    const pitch = Math.asin(Math.max(-1, Math.min(1, fwd.y))), roll = Math.asin(Math.max(-1, Math.min(1, right.y)));
    this.tilt.pitch = pitch; this.tilt.roll = roll;
    const rollRate = this._rollPrev == null ? 0 : (roll - this._rollPrev) / dt; this._rollPrev = roll;
    if (!this.leveling || this.estop) return;
    const Lavg = this.bogies.reduce((a, b) => a + b.L, 0) / this.bogies.length, Ltarget = B.Lnom + this.rideOffset;
    for (const b of this.bogies) {
      const h = b.z * Math.sin(roll), hdot = b.z * Math.cos(roll) * rollRate;
      const Ldes = b.L - h * 0.85 + (Ltarget - Lavg);
      const err = Math.max(B.Lmin + 0.06, Math.min(B.Lmax - 0.06, Ldes)) - b.L;
      const rate = Math.max(-S.levelRate, Math.min(S.levelRate, 0.6 * (err - TD * 0.85 * hdot)));
      b.level = Math.max(-S.levelRange, Math.min(S.levelRange, b.level + rate * dt));
    }
  };
}
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover;
const mode = process.env.LEVEL || '1'; if (mode === '0') R.leveling = false;
sim.on('log', (e) => { if (/boxed|FAILED|complete|Tilt reflex|delivered|SOFT CAPTURE/.test(e.msg)) console.log(`[${e.t.toFixed(0)}] ${e.msg}`); });
A.start('cycle', A.missionCycle());
const tmax = +(process.argv[2] || 900);
let peak = 0, last = 0, lowContact = 0, loadedT = 0;
while (A.gen && sim.time < tmax) {
  sim.stepOnce();
  if (mode === 'freeze') R.leveling = !(R.loaded && /^Navigate/.test(A.phase));
  peak = Math.max(peak, Math.abs(R.tilt.roll * 57.3));
  if (R.loaded) { loadedT += 1 / 120; let n = 0; for (const b of R.bogies) for (const w of b.wheels) if (w.contact) n++; if (n < 4) lowContact += 1 / 120; }
  if (sim.time - last >= 60) { last = sim.time; console.log(`t=${sim.time.toFixed(0)} peak roll ${peak.toFixed(1)} deg | loaded ${loadedT.toFixed(0)} s, <4 wheels down ${(100 * lowContact / Math.max(1, loadedT)).toFixed(0)} % | ${A.phase}`); peak = 0; }
}
console.log('end', sim.time.toFixed(0), A.gen ? 'running' : 'finished', JSON.stringify(A.summary?.() ?? {}));
