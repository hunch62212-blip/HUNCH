import { Slam } from '../../src/sim/slam.js';
// 260-node winding 2.2 km route, estimate drifting +0.02 deg/m heading wander; end fix of ~35 m
let x = 0, z = 0, yaw = 0, tx = 0, tz = 0, tyaw = 0; const nodes = [], truth = [];
for (let i = 0; i < 260; i++) {
  nodes.push({ x, z, yaw, d: i * 8.5 }); truth.push({ x: tx, z: tz, yaw: tyaw });
  const turn = 0.25 * Math.sin(i / 17) + (i % 41 === 0 ? 0.6 : 0);
  tyaw += turn * 0.2; tx += 8.5 * Math.cos(tyaw); tz += -8.5 * Math.sin(tyaw);
  yaw += turn * 0.2 + 0.00004 * 8.5 * (1 + 0.5 * Math.sin(i / 9)); x += 8.5 * Math.cos(yaw); z += -8.5 * Math.sin(yaw);
}
const last = nodes[259], tl = truth[259];
const goal = { dx: tl.x - last.x, dz: tl.z - last.z, dyaw: tl.yaw - last.yaw };
console.log('end error', Math.hypot(goal.dx, goal.dz).toFixed(2), 'm', (goal.dyaw * 57.3).toFixed(2), 'deg');
const t0 = performance.now(), r = Slam.relaxChain(nodes, goal);
console.log('iters', r.iters, 'ms', (performance.now() - t0).toFixed(0));
let maxE = 0, maxFrac = 0, maxY = 0;
r.P.forEach((p, i) => { const e = Math.hypot(p.x - truth[i].x, p.z - truth[i].z); maxE = Math.max(maxE, e);
  const f = nodes[i].d / nodes[259].d, fx = nodes[i].x + f * goal.dx, fz = nodes[i].z + f * goal.dz;
  maxFrac = Math.max(maxFrac, Math.hypot(p.x - fx, p.z - fz)); maxY = Math.max(maxY, Math.abs(p.yaw - nodes[i].yaw) * 57.3); });
console.log('max err to truth after', maxE.toFixed(2), 'm; before', Math.max(...nodes.map((q, i) => Math.hypot(q.x - truth[i].x, q.z - truth[i].z))).toFixed(2), '; max dev from odometer-fraction', maxFrac.toFixed(2), 'm; max yaw change', maxY.toFixed(2), 'deg');
for (const i of [50, 130, 200, 250, 257, 258, 259]) console.log(i, 'dx', (r.P[i].x - nodes[i].x).toFixed(2), 'dz', (r.P[i].z - nodes[i].z).toFixed(2), 'dyaw', ((r.P[i].yaw - nodes[i].yaw) * 57.3).toFixed(2));
