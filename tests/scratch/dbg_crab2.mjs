// Raw check: direction the rover's origin actually travels vs the direction it points, last 3 m under CP-36C.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
await RAPIER.init();
const ci = Number(process.argv[2] || 2);
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover, c = sim.cargo[ci], T = c.T, ct = c.body.translation();
const cy = Math.cos(c.yaw), sy = Math.sin(c.yaw), D = 57.2958, wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - cy * back, ct.z + sy * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
const t0 = sim.time; A.start('pickup', A.acquire(c)); let prev = null, i = 0;
while (A.gen && sim.time - t0 < 75) { sim.stepOnce(); if (!/Drive under/.test(A.phase)) continue;
  const p = R.pose(), dx = p.x - ct.x, dz = p.z - ct.z, along = -(dx * cy - dz * sy) - (T.carryX || 0), lat = dx * sy + dz * cy;
  if (along > 3.2 || ++i % 240) continue;
  const com = R.body.worldCom(), cass = sim.sensors.cass;
  if (prev) { const mx = p.x - prev.x, mz = p.z - prev.z, travel = Math.atan2(-mz, mx), cmx = com.x - prev.cx, cmz = com.z - prev.cz;
    console.log(`along ${along.toFixed(2)} | origin moved ${(Math.hypot(mx, mz) * 100).toFixed(1)} cm, travel dir vs module axis ${(wrap(travel - c.yaw) * D).toFixed(2)} deg, vs own heading ${(wrap(travel - p.yaw) * D).toFixed(2)} deg | heading vs axis ${(wrap(p.yaw - c.yaw) * D).toFixed(2)} | COM travel vs heading ${(wrap(Math.atan2(-cmz, cmx) - p.yaw) * D).toFixed(2)} | lat origin ${(lat * 100).toFixed(1)} cm | CASS lateral ${cass?.valid ? (cass.lateral * 100).toFixed(1) : '-'} cm yaw ${cass?.valid ? (cass.yaw * D).toFixed(2) : '-'} | roll ${(R.tilt.roll * D).toFixed(2)} | origin-COM offset fwd ${(((com.x - p.x) * Math.cos(p.yaw) - (com.z - p.z) * Math.sin(p.yaw))).toFixed(2)} right ${(((com.x - p.x) * Math.sin(p.yaw) + (com.z - p.z) * Math.cos(p.yaw))).toFixed(2)} m`); }
  prev = { x: p.x, z: p.z, cx: com.x, cz: com.z }; }
