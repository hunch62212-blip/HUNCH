// Timestep single source and fixed warp steps (docs/changes/C.md):
//   - the Rapier world steps at PHYS.dt (world.js sets it from config, and asserts it)
//   - Sim.stepOnce returns the sim seconds it advanced (PHYS.dt on physics)
//   - the transit-warp rails step is fixed per time scale (Sim.railsStep), whatever the wall-clock backlog
//   - the quiescent hold (berthing leak check) steps without physics, keeps idle power drain, and expires on its own
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { PHYS } from '../src/sim/config.js';
import { check, within, equal } from './lib/check.mjs';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover;

within('world.timestep = PHYS.dt', sim.world.timestep, PHYS.dt * (1 - 1e-6), PHYS.dt * (1 + 1e-6), ' s', 8);
for (let i = 0; i < 240; i++) sim.stepOnce();   // settle
const dtRet = sim.stepOnce();
within('stepOnce returns PHYS.dt on a physics step', dtRet, PHYS.dt - 1e-12, PHYS.dt + 1e-12, ' s', 8);

// rails step: fixed per scale, a whole number of physics steps, independent of the frame's backlog
{
  const hs = [], realWarp = sim.warpStep.bind(sim), realNow = performance.now.bind(performance);
  let fake = 0; performance.now = () => fake;
  sim.warpStep = (h) => { hs.push(h); fake += 1; return { dt: h, autoRan: true }; };
  const rows = [];
  for (const scale of [5, 10, 50]) {
    sim.timeScale = scale;
    const want = sim.railsStep(); const before = hs.length;
    for (const realDt of [0.004, 0.016, 0.05, 0.1]) { sim._acc = 0; Object.defineProperty(A, 'railsOk', { value: true, writable: true, configurable: true }); sim.advance(realDt, 20); }
    const got = hs.slice(before);
    rows.push({ scale, want, n: got.length, distinct: [...new Set(got.map((h) => h.toFixed(9)))] });
    check(`${scale}x: every rails step is railsStep() = ${want.toFixed(4)} s whatever the frame time`, got.length > 0 && got.every((h) => Math.abs(h - want) < 1e-12), `${got.length} steps, sizes ${rows.at(-1).distinct.join(', ')}`);
    const n = want / PHYS.dt;
    check(`${scale}x: rails step is a whole number of physics steps`, Math.abs(n - Math.round(n)) < 1e-9, `${n.toFixed(3)} x PHYS.dt`);
  }
  within('5x rails step (~1 frame of sim time)', rows[0].want, 0.08, 0.09, ' s', 4);
  equal('>= 30x rails step capped at warp.H', rows[2].want, sim.warp.H);
  sim.warpStep = realWarp; performance.now = realNow; sim.timeScale = 1; A.railsOk = false; sim.warp.active = false; sim._acc = 0;
}

// quiescent hold
{
  R.brake = true; R.cmd.vx = R.cmd.vz = R.cmd.wz = 0;
  for (let i = 0; i < 600; i++) sim.stepOnce();
  const t0 = sim.time, wh0 = R.power.wh, p0 = R.body.translation(), steps0 = sim.stepCount;
  sim.quiescentHold = { until: t0 + 60, gen: A.gen };
  const h = sim.stepOnce();
  within('hold step advances 0.5 s', h, 0.5 - 1e-9, 0.5 + 1e-9, ' s', 4);
  let n = 1; while (sim.quiescentHold && n < 1000) { sim.stepOnce(); n++; }
  within('60 s hold runs on ~120 hold steps (not 7,200 physics steps)', sim.stepCount - steps0, 115, 125, ' steps', 0);
  within('hold ends exactly at its end time', sim.time - t0, 60 - 1e-6, 60 + PHYS.dt + 1e-6, ' s', 4);
  const p1 = R.body.translation();
  // (the call that finds the hold expired runs one physics step: allow a couple of float32 ulps at ~800 m, 61 um each)
  within('rover does not move during the hold', Math.hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z), 0, 2e-4, ' m', 6);
  check('idle power keeps flowing through the hold (battery energy changed)', R.power.wh !== wh0, `${wh0.toFixed(1)} -> ${R.power.wh.toFixed(1)} Wh`);
  within('after the hold, stepOnce is a physics step again', sim.stepOnce(), PHYS.dt - 1e-12, PHYS.dt + 1e-12, ' s', 8);
  // a hold that belongs to another mission generator never applies
  sim.quiescentHold = { until: sim.time + 60, gen: { stale: true } };
  within('hold of a stale mission is ignored', sim.stepOnce(), PHYS.dt - 1e-12, PHYS.dt + 1e-12, ' s', 8);
  check('stale hold is cleared', sim.quiescentHold === null, String(sim.quiescentHold));
}
