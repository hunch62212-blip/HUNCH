// Loaded vs empty constant-curvature turn: does the IMU yaw integral match the true heading change? (CHANGES.md §17)
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover, c = sim.cargo[0];
function turn(label, v, wz, secs) {
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  R.brake = false; A.stop?.();
  let y0 = R.pose().yaw, yu = 0, prev = y0, gyro = 0, vsum = 0, n = 0, wsum = 0;
  for (let i = 0; i < secs * 120; i++) {
    R.cmd.vx = v; R.cmd.vz = 0; R.cmd.wz = wz; sim.stepOnce();
    const y = R.pose().yaw; yu += wrap(y - prev); prev = y;
    gyro += sim.sensors.imu.yawRate / 120; if (i > 600) { const lv = R.body.linvel(); vsum += Math.hypot(lv.x, lv.z); wsum += Math.abs(R.body.angvel().y); n++; }
  }
  console.log(`${label}: true dyaw ${(yu * 57.2958).toFixed(2)} deg, gyro ${(gyro * 57.2958).toFixed(2)} deg, ratio ${(gyro / yu).toFixed(4)}, mean v ${(vsum / n).toFixed(3)} m/s, mean |w| ${(wsum / n).toFixed(4)} rad/s`);
}
{ const sp = sim.terrain.pad, p = R.pose(); R.teleport(p.x - sp.perp.x * 15, p.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); }
for (let i = 0; i < 600; i++) sim.stepOnce();
turn('empty v0.8 wz0.04', 0.8, 0.04, 60);
// pick up the module, then the same turn loaded
const ct = c.body.translation(), back = c.T.length / 2 + 4.9 + 22 - (c.T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw); for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
A.start('pick', A.acquire(c)); const tp = sim.time; while (A.gen && sim.time - tp < 1500) sim.stepOnce();
console.log('loaded', R.loaded);
turn('loaded v0.8 wz0.04', 0.8, 0.04, 60);
turn('loaded v0.8 wz-0.04', 0.8, -0.04, 60);
turn('loaded v1.5 wz0.07', 1.5, 0.07, 40);
turn('loaded v0.5 wz0.025', 0.5, 0.025, 80);
