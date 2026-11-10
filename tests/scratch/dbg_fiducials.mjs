// Pad->hab drive: does the camera see the habitat port / lander fiducials, at what range, and do fixes fire?
// Every 30 s: true distance to the goal, estimated distance, SLAM error, nearest fiducial seen in that window.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover, S = sim.sensors;
const H = sim.terrain.hab, goal = { x: WORLD.habitat.x + H.nx * 45, z: WORLD.habitat.z + H.nz * 45 };
sim.on('log', (e) => { if (/fiducial|Fiducial|loop closure|FAILED|complete/.test(e.msg)) console.log(`[${e.t.toFixed(0)}] ${e.msg}`); });
A.start('goto', A.missionGoto(goal.x, goal.z, 'hab'));
const tmax = +(process.argv[2] || 900); let last = 0, near = null, n = 0;
while (A.gen && sim.time < tmax) {
  sim.stepOnce();
  for (const d of S.detections || []) if (d.known) { n++; if (!near || d.range < near.range) near = { id: d.id, range: d.range }; }
  if (sim.time - last >= 30) { last = sim.time; const p = R.pose(), e = sim.slam.est;
    console.log(`t=${sim.time.toFixed(0)} true->goal ${Math.hypot(goal.x - p.x, goal.z - p.z).toFixed(0)} est->goal ${Math.hypot(goal.x - e.x, goal.z - e.z).toFixed(0)} err ${Math.hypot(e.x - p.x, e.z - p.z).toFixed(1)} | fiducial samples ${n} nearest ${near ? near.id + ' ' + near.range.toFixed(0) + ' m' : '-'} | ${A.detail}`);
    near = null; n = 0; }
}
