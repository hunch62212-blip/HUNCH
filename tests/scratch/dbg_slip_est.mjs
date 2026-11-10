// Is the torque-based slip estimate the cause of SLAM under-reading distance?
// Sums true planar distance, raw encoder distance (r*omega, no slip correction) and the
// slip-corrected odometry distance, split by whether the rover is accelerating / steady / turning hard.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD, ROVER } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover, S = sim.sensors;
const H = sim.terrain.hab; sim.slam.matching = false;
A.start('goto', A.missionGoto(WORLD.habitat.x + H.nx * 45, WORLD.habitat.z + H.nz * 45, 'hab'));
const tmax = +(process.argv[2] || 240), dt = 1 / 120;
const acc = {}; const add = (k, t, raw, odo) => { const a = acc[k] ||= { t: 0, raw: 0, odo: 0 }; a.t += t; a.raw += raw; a.odo += odo; };
let p0 = R.pose(), o0 = S.odom.dist, raw = 0, vPrev = 0;
while (A.gen && sim.time < tmax) {
  sim.stepOnce();
  let w = 0, n = 0; for (const b of R.bogies) for (const wh of b.wheels) { w += wh.omega * ROVER.wheel.radius; n++; }
  raw += Math.abs(w / n) * dt;
  if (sim.stepCount % 12 === 0) { // 0.1 s buckets
    const p = R.pose(), t = Math.hypot(p.x - p0.x, p.z - p0.z), o = S.odom.dist - o0;
    const v = R.telemetry().fwdSpeed, a = (v - vPrev) / 0.1, yr = Math.abs(S.imu.yawRate);
    const k = Math.abs(a) > 0.15 ? 'accelerating' : yr > 0.05 ? 'turning hard' : t < 0.05 ? 'stopped' : 'steady';
    add(k, t, raw, o); add('ALL', t, raw, o);
    p0 = p; o0 = S.odom.dist; raw = 0; vPrev = v;
  }
}
console.log('bucket        true m   raw enc/true   slip-corrected/true');
for (const [k, a] of Object.entries(acc)) console.log(k.padEnd(13), a.t.toFixed(0).padStart(7), (a.raw / a.t).toFixed(3).padStart(12), (a.odo / a.t).toFixed(3).padStart(14));
