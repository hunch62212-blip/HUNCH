// SLAM drift on an autonomous pad -> habitat drive (default 900 s sim cap; VMAX env caps cruise speed).
// Pass: arrives; SLAM position error never exceeds 5 % of distance travelled (assumed threshold) after
// the first 100 m, and is <= 5 m at the end (habitat fiducials in view).
import { check, atMost } from '../lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
import { Slam } from '../../src/sim/slam.js';
const orig = Slam.relaxChain;
Slam.relaxChain = (nodes, goal) => { const r = orig(nodes, goal); const n = nodes.length;
  console.log('RELAX n', n, 'goal', goal.dx.toFixed(2), goal.dz.toFixed(2), (goal.dyaw*57.3).toFixed(2), 'iters', r.iters);
  for (const i of [1, 50, 100, 150, 200, n-8, n-6, n-4, n-3, n-2, n-1]) if (i>=0&&i<n) console.log(' node', i, 'd', nodes[i].d.toFixed(0), 't', nodes[i].t.toFixed(0), 'dx', (r.P[i].x-nodes[i].x).toFixed(2), 'dz', (r.P[i].z-nodes[i].z).toFixed(2), 'dyaw', ((r.P[i].yaw-nodes[i].yaw)*57.3).toFixed(2));
  return r; };
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover;
const H = sim.terrain.hab;
sim.on('log', (e) => { if (e.level !== 'info') console.log(`[${(e.t / 60).toFixed(2)}] ${e.level} ${e.msg}`); });
if (process.env.NEAR) sim.autonomy.planner.unknownNearHazardCost = +process.env.NEAR;
A.speedSetting = +(process.env.VMAX || 99);
A.start('goto', A.missionGoto(WORLD.habitat.x + H.nx * 45, WORLD.habitat.z + H.nz * 45, 'hab'));
let last = 0, worstRatio = 0, worstAt = 0; const tmax = +(process.argv[2] || 1500); // the ~2 km pad -> habitat leg needs ~1,100 s at ~1.85 m/s (900 s ended at ~1,680 m)
while (A.gen && sim.time < tmax) { sim.stepOnce(); if (R.odo.dist > 100 && sim.stepCount % 120 === 0) { const p = R.pose(), e = sim.slam.est, r = Math.hypot(e.x - p.x, e.z - p.z) / R.odo.dist; if (r > worstRatio) { worstRatio = r; worstAt = R.odo.dist; } } if (sim.time - last >= 30) { last = sim.time; const p = R.pose(), e = sim.slam.est, tl = R.telemetry();
  console.log(`t=${sim.time.toFixed(0)} ${R.rpm.proto.toFixed(0)}rpm ${R.rpm.tier} v ${tl.fwdSpeed.toFixed(2)} dist ${R.odo.dist.toFixed(0)} err ${Math.hypot(e.x-p.x,e.z-p.z).toFixed(2)} yawErr ${((e.yaw-p.yaw)*57.3).toFixed(2)} odoScale? lc ${sim.slam.lcCount ?? ''}`); } }
console.log('done', sim.time.toFixed(0), A.gen ? 'running' : 'finished', 'dist', R.odo.dist.toFixed(0));
const rec = A.history[A.history.length - 1], p = R.pose(), e = sim.slam.est;
check('reached habitat apron', !A.gen && !!(rec && rec.ok), `${A.gen ? 'still running' : 'finished'} at ${sim.time.toFixed(0)} s`);
atMost('worst SLAM error / distance travelled (after 100 m)', worstRatio * 100, 5, ' %', 2);
console.log('worst ratio at odometer', worstAt.toFixed(0), 'm');
atMost('final SLAM position error', Math.hypot(e.x - p.x, e.z - p.z), 5, ' m', 2);
