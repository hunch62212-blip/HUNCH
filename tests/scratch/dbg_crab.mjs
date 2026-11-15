// Why does the rover slide sideways in the last metres under CP-36C? Body-frame lateral velocity, roll, per-side wheel sinkage / slip / torque.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
import { heightAt } from '../../src/sim/terrain.js';
await RAPIER.init();
const ci = Number(process.argv[2] || 2);
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover, c = sim.cargo[ci], T = c.T, ct = c.body.translation();
const cy = Math.cos(c.yaw), sy = Math.sin(c.yaw);
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - cy * back, ct.z + sy * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
// ground profile across the lane at several stations along the axis (height right track minus left track -> roll the ground imposes)
const prof = []; for (const a of [14, 10, 6, 3, 1, 0, -2]) { const x = ct.x - cy * a, z = ct.z + sy * a, half = Math.abs(ROVER.corners[0][1]);
  prof.push(`${a} m: ${(Math.atan2(heightAt(sim.terrain, x + sy * half, z + cy * half) - heightAt(sim.terrain, x - sy * half, z - cy * half), 2 * half) * 57.3).toFixed(2)}`); }
console.log('ground cross slope under the wheel tracks (deg, + = right track higher) at distance to go:', prof.join('  '));
const t0 = sim.time; A.start('pickup', A.acquire(c)); let i = 0, acc = { vl: 0, n: 0 };
while (A.gen && sim.time - t0 < 75) { sim.stepOnce(); if (!/Drive under/.test(A.phase)) continue;
  const p = R.pose(), dx = p.x - ct.x, dz = p.z - ct.z, along = -(dx * cy - dz * sy) - (T.carryX || 0), lat = dx * sy + dz * cy;
  if (along > 3.2) continue;
  const lv = R.body.linvel(), right = { x: Math.sin(p.yaw), z: Math.cos(p.yaw) }, vLat = lv.x * right.x + lv.z * right.z, vF = lv.x * Math.cos(p.yaw) - lv.z * Math.sin(p.yaw);
  acc.vl += vLat; acc.n++;
  if (++i % 240) continue;
  const side = (s) => R.wheels.filter((w, k) => (ROVER.corners[k][1] < 0) === (s < 0));
  const st = (ws, f) => (ws.reduce((a, w) => a + f(w), 0) / ws.length);
  const L = side(-1), Rr = side(1);
  console.log(`along ${along.toFixed(2)} lat ${(lat * 100).toFixed(1)} cm | v fwd ${(vF * 1000).toFixed(0)} mm/s lateral(mean) ${(acc.vl / acc.n * 1000).toFixed(1)} mm/s (+ = right) | roll ${(R.tilt.roll * 57.3).toFixed(2)} pitch ${(R.tilt.pitch * 57.3).toFixed(2)} | cmd vx ${R.cmd.vx.toFixed(3)} wz ${(R.cmd.wz * 1000).toFixed(1)} mrad/s yawRate ${(R.body.angvel().y * 1000).toFixed(1)} | sink L ${(st(L, (w) => w.sink) * 1000).toFixed(0)} R ${(st(Rr, (w) => w.sink) * 1000).toFixed(0)} mm | slip L ${(st(L, (w) => w.slip) * 100).toFixed(0)} R ${(st(Rr, (w) => w.slip) * 100).toFixed(0)} % | torque L ${st(L, (w) => w.torque).toFixed(0)} R ${st(Rr, (w) => w.torque).toFixed(0)} | load L ${(st(L, (w) => w.load) / 1000).toFixed(1)} R ${(st(Rr, (w) => w.load) / 1000).toFixed(1)} kN`);
  acc = { vl: 0, n: 0 }; }
