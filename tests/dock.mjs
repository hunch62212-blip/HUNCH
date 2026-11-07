// Mission 5 in isolation: rover + held module placed 60 m out from its port, then autonomous delivery.
// Pass: mission OK, cargo docked/delivered, port occupied, soft-capture error within 10 mm lateral / vertical
// (README: docking-ring camera gives mm-level guidance) and 1 deg yaw (assumed).
import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { Sim } from '../src/sim/sim.js';
import { ROVER } from '../src/sim/config.js';
import { check, equal, atMost } from './lib/check.mjs';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover, c = sim.cargo[Number(process.argv[2] || 0)];
sim.on('log', (e) => { if (!/Local planner blocked/.test(e.msg)) console.log(`[${e.t.toFixed(0).padStart(5)} s] ${e.level.padEnd(4)} ${e.msg}`); });
// Setup: autonomous pickup from 22 m behind the module (as pickup_time.mjs). The original setup used a
// 0.6 m hoist (SPAN-40); AEGIS-V1 has no hoist travel (ROVER.lift.travel = 0) and picks up by kneel/drive-under/rise.
const ct = c.body.translation(), back = c.T.length / 2 + ROVER.frame.halfLen + 22 - (c.T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw); for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); sim.streamRocks([{ x: ct.x, z: ct.z }]);
A.start('M5 setup pickup', A.acquire(c)); const tp = sim.time;
while (A.gen && sim.time - tp < 1500) sim.stepOnce();
console.log('loaded', R.loaded);
if (!check(`setup: ${c.id} picked up`, R.loaded, `cargo state ${c.state}`)) process.exit(1);
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
const rec = A.history[A.history.length - 1];
check('M5 delivery mission OK', rec && rec.name === 'M5 test' && rec.ok, rec ? `${rec.name} ok=${rec.ok}` : 'no mission record');
check('cargo delivered', ['delivered', 'docked'].includes(c.state), `state ${c.state}`);
check('port occupied', !!port.occupied, String(!!port.occupied));
const acc = A.metrics.accuracy[A.metrics.accuracy.length - 1];
if (acc) {
  atMost('soft capture lateral error', Math.abs(acc.lateralMm), 10, ' mm', 1);
  atMost('soft capture vertical error', Math.abs(acc.verticalMm), 10, ' mm', 1);
  atMost('soft capture yaw error', Math.abs(acc.yawDeg), 1, ' deg', 2);
} else check('soft capture accuracy recorded', false, 'no accuracy record');
