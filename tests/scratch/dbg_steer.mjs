import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { runCommand } from '../../src/sim/commands.js';
await RAPIER.init();
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover;
sim.on('log', (e) => { if (/steering|retry|FAULT/.test(e.msg)) console.log(`[${(e.t / 60).toFixed(2)}] ${e.msg.slice(0, 100)}`); });
runCommand(sim, {}, 'goto hab');
let inj = 0, last = 0, sc = 0, sa = 0, n = 0, cntOld = 0;
while (A.gen && sim.time < 480) {
  sim.stepOnce();
  if (!inj && sim.time > 300) { inj = 1; runCommand(sim, {}, 'fault block'); runCommand(sim, {}, 'fault wheel 3'); }
  if (sim.time > 290) {
    const f = R._cmdF, yr = sim.sensors.imu.yawRate;
    if (f && Math.abs(f.vx) > 0.5 && Math.abs(f.wz) > 0.02) { sc += Math.abs(f.wz); sa += yr * Math.sign(f.wz); n++; }
    if (sim.time - last > 5) { last = sim.time; console.log(`t=${sim.time.toFixed(0)} v ${f ? f.vx.toFixed(2) : '-'} cmdWz ${f ? f.wz.toFixed(3) : '-'} yr ${yr.toFixed(3)} | avgC ${(A._yawCmdAvg||0).toFixed(3)} avgA ${(A._yawActAvg||0).toFixed(3)} noSteer ${(A._noSteerT||0).toFixed(1)} | 5s ratio ${n ? (sa/sc).toFixed(2) : '-'} | ${A.phase.slice(0,40)}`); sc = sa = n = 0; }
  }
}
