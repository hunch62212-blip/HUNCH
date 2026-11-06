import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover;
sim.on('log', (e) => console.log(`[${(e.t / 60).toFixed(2)}] ${e.level} ${e.msg}`));
A.start('cycle', A.missionCycle());
let last = 0;
while (A.gen && sim.time < 720) { sim.stepOnce(); if (sim.time - last > 4 && sim.time > 600) { last = sim.time; const p = R.pose(), e = sim.slam.est, tl = R.telemetry();
  console.log(`t=${sim.time.toFixed(0)} ${A.phase} | ${A.detail} | pos ${p.x.toFixed(1)},${p.z.toFixed(1)} yaw ${(p.yaw*57.3).toFixed(1)} v ${tl.fwdSpeed.toFixed(2)} err ${Math.hypot(e.x-p.x,e.z-p.z).toFixed(2)} yawErr ${((e.yaw-p.yaw)*57.3).toFixed(1)} pitch ${(tl.pitch*57.3).toFixed(1)} roll ${(tl.roll*57.3).toFixed(1)} scanN ${sim.sensors.scan?.n}`); } }
