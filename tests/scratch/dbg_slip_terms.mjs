// Per-wheel check of the torque-based slip estimate against the physics' own slip, on the pad->hab drive.
// Ground speed per wheel from: raw r*omega, the current estimate, the estimate after removing wheel
// spin-up torque (I*domega/dt), and the true slip. Also counts "saturated" (estimate = 100 % slip) samples.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD, ROVER } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover, S = sim.sensors;
const H = sim.terrain.hab; sim.slam.matching = false;
A.start('goto', A.missionGoto(WORLD.habitat.x + H.nx * 45, WORLD.habitat.z + H.nz * 45, 'hab'));
const tmax = +(process.argv[2] || 180), dt = 1 / 120, r = ROVER.wheel.radius, I = ROVER.wheel.inertia;
const prev = new Map(); const sum = { raw: 0, est: 0, estI: 0, tru: 0, n: 0, sat: 0, satI: 0, inertPct: 0 };
const spd = (rw, s) => (s >= 0 ? rw * (1 - s) : rw / (1 - s));
while (A.gen && sim.time < tmax) {
  sim.stepOnce();
  for (const b of R.bogies) for (const w of b.wheels) {
    const om0 = prev.get(w); prev.set(w, w.omega);
    if (om0 == null || !w.contact || w.load < 50) continue;
    const rw = w.omega * r, Ti = I * (w.omega - om0) / dt;
    const sE = S.slipFromTorque(w.torque, w.load), sI = S.slipFromTorque(w.torque - Ti, w.load);
    sum.raw += Math.abs(rw); sum.est += Math.abs(spd(rw, sE)); sum.estI += Math.abs(spd(rw, sI)); sum.tru += Math.abs(spd(rw, w.slip));
    sum.n++; if (Math.abs(sE) > 0.99) sum.sat++; if (Math.abs(sI) > 0.99) sum.satI++;
    sum.inertPct += Math.abs(Ti) / Math.max(1, Math.abs(w.torque));
  }
}
const f = (k) => (sum[k] / sum.tru).toFixed(3);
console.log(`samples ${sum.n}  ground speed / true-slip speed: raw ${f('raw')}  current est ${f('est')}  est minus spin-up torque ${f('estI')}`);
console.log(`saturated (100 % slip) samples: current ${(100 * sum.sat / sum.n).toFixed(2)} %  minus spin-up ${(100 * sum.satI / sum.n).toFixed(2)} %  | mean spin-up share of torque ${(100 * sum.inertPct / sum.n).toFixed(1)} %`);
