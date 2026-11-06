import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { WORLD } from '../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover;
const H = sim.terrain.hab;
sim.on('log', (e) => { if (e.level !== 'info') console.log(`[${(e.t / 60).toFixed(2)}] ${e.level} ${e.msg}`); });
A.speedSetting = +(process.env.VMAX || 99);
A.start('goto', A.missionGoto(WORLD.habitat.x + H.nx * 45, WORLD.habitat.z + H.nz * 45, 'hab'));
let last = 0; const tmax = +(process.argv[2] || 900);
while (A.gen && sim.time < tmax) { sim.stepOnce(); if (sim.time - last >= 30) { last = sim.time; const p = R.pose(), e = sim.slam.est, tl = R.telemetry();
  console.log(`t=${sim.time.toFixed(0)} ${R.rpm.proto.toFixed(0)}rpm ${R.rpm.tier} v ${tl.fwdSpeed.toFixed(2)} dist ${R.odo.dist.toFixed(0)} err ${Math.hypot(e.x-p.x,e.z-p.z).toFixed(2)} yawErr ${((e.yaw-p.yaw)*57.3).toFixed(2)} odoScale? lc ${sim.slam.lcCount ?? ''}`); } }
console.log('done', sim.time.toFixed(0), A.gen ? 'running' : 'finished', 'dist', R.odo.dist.toFixed(0));
