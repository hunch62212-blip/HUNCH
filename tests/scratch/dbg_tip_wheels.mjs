// cycle: during the loaded roll-over, per-wheel contact height (relative to the lowest wheel), contact flag and
// load, plus which rock (if any) each wheel is touching. Shows whether a wheel climbs an obstacle or the vehicle
// rolls on even ground.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover;
A.start('cycle', A.missionCycle());
const t0 = +(process.argv[2] || 300), t1 = +(process.argv[3] || 322);
let last = 0;
while (A.gen && sim.time < t1) {
  sim.stepOnce();
  if (sim.time >= t0 && sim.time - last >= 0.5) { last = sim.time;
    const ws = R.bogies.flatMap((b) => b.wheels.map((w) => ({ n: b.name || '', y: w.cp?.y, c: w.contact, N: w.load, L: b.L })));
    const ymin = Math.min(...ws.filter((w) => w.c).map((w) => w.y));
    console.log(`t=${sim.time.toFixed(1)} roll ${(R.tilt.roll * 57.3).toFixed(1)} pitch ${(R.tilt.pitch * 57.3).toFixed(1)} v ${R.telemetry().fwdSpeed.toFixed(2)} | ` +
      ws.map((w, i) => `${i}:${w.c ? (w.y - ymin).toFixed(2) : '--'}m ${(w.N / 1000).toFixed(0)}kN L${w.L.toFixed(2)}`).join('  '));
  }
}
