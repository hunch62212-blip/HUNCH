import { Slam } from '../../src/sim/slam.js';
// straight 100 m drive east (yaw 0) in the truth; the estimate has a 0.15 deg/node heading drift
const nodes = []; let x = 0, z = 0, yaw = 0;
for (let i = 0; i < 11; i++) { nodes.push({ x, z, yaw, d: i * 10 }); x += 10 * Math.cos(yaw); z += -10 * Math.sin(yaw); yaw += -0.15 * Math.PI / 180; }
const last = nodes[10]; // truth end = (100, 0, 0): error to remove
const goal = { dx: 100 - last.x, dz: 0 - last.z, dyaw: 0 - last.yaw };
console.log('end error', goal.dx.toFixed(3), goal.dz.toFixed(3), (goal.dyaw * 57.3).toFixed(2), 'deg');
const r = Slam.relaxChain(nodes, goal);
console.log('iters', r.iters);
r.P.forEach((p, i) => console.log(i, p.x.toFixed(3), p.z.toFixed(3), (p.yaw * 57.3).toFixed(3)));
