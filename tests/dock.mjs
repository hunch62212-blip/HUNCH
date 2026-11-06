// Mission 5 in isolation: rover + held module placed 60 m out from PORT-1, then autonomous delivery.
import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover, c = sim.cargo[Number(process.argv[2] || 0)];
sim.on('log', (e) => { if (!/Local planner blocked/.test(e.msg)) console.log(`[${e.t.toFixed(0).padStart(5)} s] ${e.level.padEnd(4)} ${e.msg}`); });
const ct = c.body.translation();
const cx = c.T.carryX || 0; R.teleport(ct.x - Math.cos(c.yaw) * cx, ct.z + Math.sin(c.yaw) * cx, c.yaw); for (let i = 0; i < 720; i++) sim.stepOnce();
R.clampClose(); for (let i = 0; i < 900; i++) sim.stepOnce(); R.lift.target = 0.6; for (let i = 0; i < 1800; i++) sim.stepOnce();
console.log('loaded', R.loaded);
// rigidly move rover + cargo to 60 m in front of the port, facing it
const port = sim.ports[c.dest - 1];
const yaw = Math.atan2(port.nz, -port.nx);
const tx = port.x + port.nx * 60, tz = port.z + port.nz * 60;
const p = R.pose(), q0 = p.q.clone(), qT = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
const dq = qT.clone().multiply(q0.clone().invert());
const gy = sim.terrain.habH;
for (const b of [R.body, c.body]) {
  const t = b.translation(), r = b.rotation();
  const rel = new THREE.Vector3(t.x - p.x, t.y - p.y, t.z - p.z).applyQuaternion(dq);
  b.setTranslation({ x: tx + rel.x, y: gy + 3.05 + rel.y, z: tz + rel.z }, true);
  const nq = dq.clone().multiply(new THREE.Quaternion(r.x, r.y, r.z, r.w)); b.setRotation(nq, true);
  b.setLinvel({ x: 0, y: 0, z: 0 }, true); b.setAngvel({ x: 0, y: 0, z: 0 }, true);
}
for (const bg of R.bogies) { bg.hpVelPrev = null; for (const w of bg.wheels) { w.groundY = null; w.anchor = null; } }
sim.streamRocks([{ x: tx, z: tz }]);
for (let i = 0; i < 240; i++) sim.stepOnce();
sim.slam.clear(); sim.slam.relocalize(R.pose());
A.start('M5 test', A.missionDeliver(c.dest));
let last = 0;
while (A.gen && sim.time < 2400) { sim.stepOnce(); if (sim.time - last > 15) { last = sim.time; console.log(`   ${A.phase} | ${A.detail.slice(0, 110)}`); } }
console.log('accuracy', JSON.stringify(A.metrics.accuracy), 'cargo state', c.state, 'port occupied', !!port.occupied);
