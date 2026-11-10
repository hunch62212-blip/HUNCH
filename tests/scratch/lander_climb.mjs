// Repro: rover driven into the lander (screenshot bug, seed 866201175)
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
const seed = +(process.argv[2] || 866201175);
const sim = new Sim(RAPIER, { seed });
const R = sim.rover;
const deg = (r) => r * 180 / Math.PI;
const st = (l) => { const p = R.pose(), t = R.telemetry(); console.log(l.padEnd(10), 't', sim.time.toFixed(1), 'pos', p.x.toFixed(1), p.y.toFixed(2), p.z.toFixed(1), 'yaw', deg(p.yaw).toFixed(0), 'v', t.fwdSpeed.toFixed(2), 'roll', deg(t.roll).toFixed(1), 'pitch', deg(t.pitch).toFixed(1), 'margin', t.margin.toFixed(1)); };
const run = (s) => { for (let i = 0; i < s * 120; i++) sim.stepOnce(); };
run(3); st('settle');
const P = WORLD.pad, p0 = R.pose();
console.log('pad', P.x, P.z, 'dist to lander', Math.hypot(p0.x - P.x, p0.z - P.z).toFixed(1));
// aim at the lander and drive into it, forward then reverse
const yawTo = Math.atan2(-(P.z - p0.z), P.x - p0.x);
R.teleport(p0.x, p0.z, yawTo); run(2); st('aimed');
R.brake = false;
const mode = process.argv[3] || 'leg';
// aim at a leg footpad (45 deg) or the engine centre
const LS = 2.6, a = Math.PI / 4;
const tx = mode === 'leg' ? P.x + Math.cos(a) * 5.4 * LS : P.x, tz = mode === 'leg' ? P.z + Math.sin(a) * 5.4 * LS : P.z;
const p1 = R.pose(); R.teleport(p1.x, p1.z, Math.atan2(-(tz - p1.z), tx - p1.x)); run(2);
for (const vx of [3, -2, 3]) { R.cmd.vx = vx; for (let k = 0; k < 12; k++) { run(3); st('vx ' + vx); } }
// worst case over the run
