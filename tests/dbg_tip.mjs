import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { heightAt } from '../src/sim/terrain.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover;
A.start('cycle', A.missionCycle());
const t0 = +process.argv[2] || 440, t1 = +process.argv[3] || 505; let last = 0;
while (A.gen && sim.time < t1) { sim.stepOnce(); if (sim.time > t0 && sim.time - last >= 1) { last = sim.time; const p = R.pose(), tl = R.telemetry(), v = R.body.linvel(), av = R.body.angvel();
  // ground slope under the rover along heading and sideways (true terrain)
  const g = (dx, dz) => heightAt(sim.terrain, p.x + dx, p.z + dz);
  const fx = p.fwd.x, fz = p.fwd.z; const sl = Math.atan2(g(fx*4, fz*4) - g(-fx*4, -fz*4), 8) * 57.3; const sr = Math.atan2(g(-fz*4, fx*4) - g(fz*4, -fx*4), 8) * 57.3;
  console.log(`t=${sim.time.toFixed(0)} ${A.detail.slice(0,60)} | v ${tl.fwdSpeed.toFixed(2)} cmd ${R.cmd.vx.toFixed(2)}/${R.cmd.wz.toFixed(3)} pitch ${(tl.pitch*57.3).toFixed(1)} roll ${(tl.roll*57.3).toFixed(1)} margin ${R.tilt.margin.toFixed(1)} groundSlope f ${sl.toFixed(1)} s ${sr.toFixed(1)} loads ${R.wheels.map(w=>(w.load/1000).toFixed(0)).join(',')}`); } }
