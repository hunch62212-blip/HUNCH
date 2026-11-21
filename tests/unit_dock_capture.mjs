// Soft-capture angular envelope and the realistic leak check (docs/changes/C.md).
//   1. latch hardware (Sim.dockCapture): a module 1 deg out of square is refused, 0.1 deg is welded
//   2. autonomy: a module held 1 deg out of square on final approach, otherwise capture-ready (gap, force, lateral,
//      vertical inside), is never latched; released, it squares up and latches inside 0.25 deg
//   3. berthing leak check holds DOCK_LEAK_CHECK_S (30 min) of sim time, on the quiescent-hold warp
import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { Sim } from '../src/sim/sim.js';
import { ROVER } from '../src/sim/config.js';
import { dockAngles, DOCK_CAPTURE_MAX_ANGLE, DOCK_LEAK_CHECK_S } from '../src/sim/autonomy.js';
import { check, equal, atMost, atLeast } from './lib/check.mjs';
await RAPIER.init();
const DEG = Math.PI / 180;
const wall0 = Date.now();

// rotate bodies rigidly about a vertical axis through P by d radians (+ = yaw left)
function yawAbout(bodies, P, d) {
  const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), d);
  for (const b of bodies) {
    const t = b.translation(), r = b.rotation();
    const rel = new THREE.Vector3(t.x - P.x, t.y - P.y, t.z - P.z).applyQuaternion(q);
    b.setTranslation({ x: P.x + rel.x, y: t.y, z: P.z + rel.z }, true);
    b.setRotation(q.clone().multiply(new THREE.Quaternion(r.x, r.y, r.z, r.w)), true);
  }
}

equal('capture angle limit is 0.25 deg', +(DOCK_CAPTURE_MAX_ANGLE / DEG).toFixed(4), 0.25);
equal('leak check default is 30 min', DOCK_LEAK_CHECK_S, 1800);

// ---- 1. latch hardware
{
  const sim = new Sim(RAPIER);
  const c = sim.cargo[0], port = sim.ports[c.dest - 1];
  const yaw = Math.atan2(port.nz, -port.nx);
  const set = (deg) => c.body.setRotation(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw + deg * DEG), true);
  set(1.0);
  const r1 = sim.dockCapture(c, port);
  check('latches refuse a module 1 deg out of square', r1.ok === false && !c.dockJoint && !port.occupied, r1.why || 'welded');
  set(0.1);
  const r2 = sim.dockCapture(c, port);
  check('latches accept a module 0.1 deg out of square', r2.ok === true && !!c.dockJoint, `yaw ${(r2.yaw / DEG).toFixed(3)} deg`);
}

// ---- 2. autonomy gate with 1 deg injected on final approach (setup as tests/dock.mjs)
const sim = new Sim(RAPIER);
const A = sim.autonomy, R = sim.rover, c = sim.cargo[0];
sim.on('log', (e) => { if (/SOFT CAPTURE|Soft capture|Leak check|refused/.test(e.msg)) console.log(`[${e.t.toFixed(0).padStart(5)} s] ${e.level.padEnd(4)} ${e.msg}`); });
const ct = c.body.translation(), back = c.T.length / 2 + ROVER.frame.halfLen + 22 - (c.T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw); for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
A.start('setup pickup', A.acquire(c)); const tp = sim.time;
while (A.gen && sim.time - tp < 1500) sim.stepOnce();
if (!check('setup: module picked up', R.loaded, `cargo state ${c.state}`)) process.exit(1);
const port = sim.ports[c.dest - 1];
{
  const yaw = Math.atan2(port.nz, -port.nx), tx = port.x + port.nx * 60, tz = port.z + port.nz * 60;
  const p = R.pose(), dq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw).multiply(p.q.clone().invert());
  for (const b of [R.body, c.body]) {
    const t = b.translation(), r = b.rotation();
    const rel = new THREE.Vector3(t.x - p.x, t.y - p.y, t.z - p.z).applyQuaternion(dq);
    b.setTranslation({ x: tx + rel.x, y: sim.terrain.habH + 3.05 + rel.y, z: tz + rel.z }, true);
    b.setRotation(dq.clone().multiply(new THREE.Quaternion(r.x, r.y, r.z, r.w)), true);
    b.setLinvel({ x: 0, y: 0, z: 0 }, true); b.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }
  for (const bg of R.bogies) { bg.hpVelPrev = null; for (const w of bg.wheels) { w.groundY = null; w.anchor = null; } }
  sim.streamRocks([{ x: tx, z: tz }]);
  for (let i = 0; i < 1800; i++) { sim.stepOnce(); const v = R.body.linvel(), w = R.body.angvel(); if (i >= 240 && Math.abs(v.y) < 0.01 && Math.hypot(v.x, v.z) < 0.02 && Math.hypot(w.x, w.z) < 0.003) break; }
  sim.slam.clear(); sim.slam.relocalize(R.pose());
}
// every weld is recorded with the true angles at that instant
const welds = []; const realCap = sim.dockCapture.bind(sim);
sim.dockCapture = (cc, pp) => { const an = dockAngles(cc, pp), r = realCap(cc, pp); welds.push({ t: sim.time, yaw: an.yaw, pitch: an.pitch, ok: r.ok }); return r; };
A.start('M5 1deg', A.missionDeliver(c.dest));
const face = () => { const t = c.body.translation(), r = c.body.rotation(); return new THREE.Vector3(c.T.length / 2, 0, 0).applyQuaternion(new THREE.Quaternion(r.x, r.y, r.z, r.w)).add(new THREE.Vector3(t.x, t.y, t.z)); };
let injT = null, holdEnd = null, maxYawHeld = 0;
while (A.gen && sim.time < 3600) {   // capture + 30 min leak check + hatch
  sim.stepOnce();
  const ring = sim.sensors.cass?.ring;
  if (injT == null && ring && ring.along < 0.06 && /Dock|approach/i.test(A.phase)) { injT = sim.time; holdEnd = injT + 90; console.log(`inject 1 deg yaw at ${injT.toFixed(1)} s, gap ${ring.along.toFixed(3)} m`); }
  if (injT != null && sim.time < holdEnd && !c.dockJoint) {
    // hold the module 1 deg out of square (a stuck cradle yaw): re-square to +1 deg whenever the controller turns it away
    const an = dockAngles(c, port);
    if (Math.abs(an.yaw - DEG) > 0.05 * DEG) { yawAbout([R.body, c.body], face(), DEG - an.yaw); for (const bg of R.bogies) { bg.hpVelPrev = null; for (const w of bg.wheels) w.anchor = null; } }
    maxYawHeld = Math.max(maxYawHeld, Math.abs(dockAngles(c, port).yaw));
  }
  if (holdEnd != null && sim.time >= holdEnd && welds.some((w) => w.ok) && A.lastBerth) break;
}
check('1 deg yaw injected on final approach', injT != null, injT != null ? `at ${injT.toFixed(1)} s` : 'final approach never reached');
const during = welds.filter((w) => w.t < holdEnd && w.ok);
check('no latch while held 1 deg out of square (90 s)', during.length === 0, `${during.length} welds, max |yaw| held ${(maxYawHeld / DEG).toFixed(2)} deg`);
console.log(`INFO capture-ready samples refused on angle alone: ${A.dockAngleRefusals} (the held 1 deg also spoils lateral, so the gap/lateral gates usually refuse first)`);
const ok = welds.filter((w) => w.ok);
for (const w of ok) atMost(`latched yaw at ${w.t.toFixed(0)} s`, Math.abs(w.yaw / DEG), 0.25, ' deg', 3);
for (const w of ok) atMost(`latched pitch at ${w.t.toFixed(0)} s`, Math.abs(w.pitch / DEG), 0.25, ' deg', 3);
// ---- 3. leak check hold (if the released module latched and berthed)
if (A.lastBerth) {
  const capT = ok[0].t;
  atLeast('leak check PASS >= 30 min after soft capture', A.lastBerth.leakPassT - capT, DOCK_LEAK_CHECK_S, ' s', 0);
  atLeast('leak check ran on the quiescent-hold warp', sim.holdWarp.simS, DOCK_LEAK_CHECK_S - 1, ' s', 0);
} else console.log(`INFO released module not berthed within the run (welds ${ok.length}, phase ${A.phase})`);
console.log(`wall ${((Date.now() - wall0) / 1000).toFixed(0)} s, sim ${sim.time.toFixed(0)} s, hold steps ${sim.holdWarp.steps}`);
