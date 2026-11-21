// Mission 5 in isolation: rover + held module placed 60 m out from its port, then autonomous delivery.
// Pass: mission OK, cargo docked/delivered, port occupied, soft-capture error within 10 mm lateral / vertical
// (README: docking-ring camera gives mm-level guidance) and 1 deg yaw (assumed). CHANGES.md §17: 5 mm, peak force < 3000 N, attempt 1.
import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { Sim } from '../src/sim/sim.js';
import { ROVER } from '../src/sim/config.js';
import { check, equal, atMost, within } from './lib/check.mjs';
await RAPIER.init();
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover, c = sim.cargo[Number(process.argv[2] || 0)];
sim.on('log', (e) => { if (!/Local planner blocked/.test(e.msg)) console.log(`[${e.t.toFixed(0).padStart(5)} s] ${e.level.padEnd(4)} ${e.msg}`); });
// Setup: autonomous pickup from 22 m behind the module (as pickup_time.mjs). The original setup used a
// 0.6 m hoist (SPAN-40); AEGIS-V1 has no hoist travel (ROVER.lift.travel = 0) and picks up by kneel/drive-under/rise.
const ct = c.body.translation(), back = c.T.length / 2 + ROVER.frame.halfLen + 22 - (c.T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw); for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]); // estimates cleared: sightings during the settle used the pre-teleport pose
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
// settle until the suspension has stopped bouncing (>= 2 s, <= 15 s): relocalising mid-bounce gives SLAM a
// height reference off by the bounce (~0.35 m), and every later scan maps the ground that far off
for (let i = 0; i < 1800; i++) { sim.stepOnce(); const v = R.body.linvel(), w = R.body.angvel(); if (i >= 240 && Math.abs(v.y) < 0.01 && Math.hypot(v.x, v.z) < 0.02 && Math.hypot(w.x, w.z) < 0.003) break; }
sim.slam.clear(); sim.slam.relocalize(R.pose());
let hatchT = null; sim.on('hatch', () => { hatchT = sim.time; });
A.start('M5 test', A.missionDeliver(c.dest));
let last = 0;
while (A.gen && sim.time < 4800) { sim.stepOnce(); if (sim.time - last > 15) { last = sim.time; console.log(`   ${A.phase} | ${A.detail.slice(0, 110)}`); } }
console.log('accuracy', JSON.stringify(A.metrics.accuracy), 'cargo state', c.state, 'port occupied', !!port.occupied);
const rec = A.history[A.history.length - 1];
check('M5 delivery mission OK', rec && rec.name === 'M5 test' && rec.ok, rec ? `${rec.name} ok=${rec.ok}` : 'no mission record');
check('cargo delivered', ['delivered', 'docked'].includes(c.state), `state ${c.state}`);
check('port occupied', !!port.occupied, String(!!port.occupied));
const acc = A.metrics.accuracy[A.metrics.accuracy.length - 1];
if (acc) {
  atMost('soft capture lateral error', Math.abs(acc.lateralMm), 5, ' mm', 1);
  atMost('soft capture vertical error', Math.abs(acc.verticalMm), 5, ' mm', 1);
  // < 3 kN (CHANGES.md §17) for every cargo type: the module pushes on the port's bellows coupling (DOCK.bellows), not a rigid wall
  atMost('peak port contact force (CHANGES.md §17)', acc.peakN, 3000, ' N', 0);
  check('soft capture on attempt 1', acc.attempts === 1, `attempt ${acc.attempts}`);
  atMost('soft capture yaw error', Math.abs(acc.yawDeg), 1, ' deg', 2);
} else check('soft capture accuracy recorded', false, 'no accuracy record');
// pressure boundary before hatch (LLASO-P2 constraint): the hatch opens only after the leak check has passed and the vestibule is at cabin pressure
check('hatch opened only after the leak check passed', hatchT != null && A.lastBerth && A.lastBerth.leakPassT != null && A.lastBerth.leakPassT < hatchT, `leak PASS at ${A.lastBerth?.leakPassT?.toFixed(0)} s, hatch at ${hatchT?.toFixed(0)} s`);
within('vestibule pressure when the hatch opened', A.lastBerth ? A.lastBerth.pressure : NaN, 69, 72, ' kPa', 1);
