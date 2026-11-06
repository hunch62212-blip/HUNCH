// Ground-truth pickup geometry check: rover aligned behind module, kneel, drive straight under, rise, clamp.
import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { Sim } from '../src/sim/sim.js';
import { ROVER } from '../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const R = sim.rover;
const ci = Number(process.argv[2] || 0);
const c = sim.cargo[ci], T = c.T;
const run = (sec, fn) => { const n = Math.round(sec * 120); for (let i = 0; i < n; i++) { if (fn && fn(i / 120)) return; sim.stepOnce(); } };
const ct = c.body.translation();
const yaw = c.yaw;
const back = T.length / 2 + ROVER.frame.halfLen + 3 - (T.carryX || 0);
R.teleport(ct.x - Math.cos(yaw) * back, ct.z + Math.sin(yaw) * back, yaw);
run(4); R.brake = false;
if (process.argv[3]?.startsWith('off')) { const k = +process.argv[3].slice(3); c.colliders[k].setCollisionGroups(0); }
if (process.argv[3] === 'nocargo') for (const cc of c.colliders) cc.setCollisionGroups(0);
if (process.argv[3] === 'nopost') { sim.world.forEachCollider((cc) => { if (!cc.parent() || cc.parent().handle === sim.fixed.handle) { const t = cc.translation(); for (const st of c.stand) if (Math.hypot(t.x - st.x, t.z - st.z) < 0.3) cc.setCollisionGroups(0); } }); }
console.log('cargo y', ct.y.toFixed(2), 'stand', c.stand ? 'yes' : 'no', 'module bottom', (ct.y - (T.adapterR || T.radius)).toFixed(2));
R.lift.target = ROVER.ride.kneel; run(7);
const rel = () => { const p = R.pose(); const t = c.body.translation(); return new THREE.Vector3(t.x - p.x, t.y - p.y, t.z - p.z).applyQuaternion(p.q.clone().invert()); };
console.log('kneeled ride', R.lift.pos.toFixed(2), 'rel', rel().toArray().map((v) => v.toFixed(2)));
let t = 0;
run(80, () => { const r = rel(); R.cmd.vx = r.x - (T.carryX || 0) > 1.5 ? 0.3 : (process.env.CREEP ? +process.env.CREEP : 0.05); R.cmd.wz = process.env.NOWZ ? 0 : Math.max(-0.03, Math.min(0.03, -0.2 * r.z)); t += 1 / 120; if (Math.round(t * 120) % 240 === 0) { const ss = sim.sensors.cass; console.log(`W ${R.wheels.map((w) => `${w.blocked ? 'B' : ''}${(w.load / 1000).toFixed(1)}/${w.slip.toFixed(2)}/${w.torque.toFixed(0)}`).join(' ')} body y ${R.pose().y.toFixed(2)} pitch ${(R.telemetry().pitch * 57.3).toFixed(1)}`); { const ctt = c.body.translation(); console.log(`  cargo pos ${(ctt.x - ct.x).toFixed(3)} ${(ctt.y - ct.y).toFixed(3)} ${(ctt.z - ct.z).toFixed(3)} forces ${JSON.stringify(sim.cargoForces.get(c) || null)}`); } console.log(`t ${t.toFixed(0)} rel x ${r.x.toFixed(2)} z ${r.z.toFixed(2)} v ${R.telemetry().fwdSpeed.toFixed(2)} cass ${ss.valid ? `lat ${ss.lateral.toFixed(3)} yaw ${ss.yaw.toFixed(3)} cx ${ss.cargoX}` : '-'}`); } if (Math.round(t * 120) % 240 === 0 && r.x < 1) { for (let i = 0; i < R.body.numColliders(); i++) { const col = R.body.collider(i); for (const cc of c.colliders) sim.world.contactPair(col, cc, (man) => { if (man.numContacts()) { const lt = col.translation(); console.log('  touch rover col', i, col.shapeType(), 'cargo col', c.colliders.indexOf(cc), 'depth', man.contactDist(0).toFixed(3), 'n', JSON.stringify(man.normal())); } }); } }
  return r.x - (T.carryX || 0) < 0.03 || t > 120; });
R.cmd.vx = 0; run(3);
{ const w = sim.world; const names = new Map(); R.body.numColliders && 0;
  for (let i = 0; i < R.body.numColliders(); i++) { const col = R.body.collider(i);
    w.contactPairsWith(col, (o) => { w.contactPair(col, o, (man) => { if (man.numContacts() > 0) { const own = sim.colliderOwner.get(o.handle); const t = col.translation(); console.log('CONTACT rover collider', i, 'shape', col.shapeType(), 'at', [t.x - R.pose().x, t.y - R.pose().y, t.z - R.pose().z].map((v) => v.toFixed(2)).join(','), 'with', own ? own.kind + ':' + (own.ref?.id || '') : (o.handle === sim.terrainCollider.handle ? 'terrain' : 'other ' + o.shapeType())); } }); }); } }
console.log('under: rel', rel().toArray().map((v) => v.toFixed(3)), 'check', JSON.stringify(R.grappleCheck(), (k, v) => (k === 'c' ? undefined : v)));
const y0 = c.body.translation().y;
R.lift.target = ROVER.ride.carry; run(8);
console.log('lifted by', (c.body.translation().y - y0).toFixed(3), JSON.stringify(R.grappleCheck(), (k, v) => (k === 'c' ? undefined : v)));
console.log(R.clampClose()); run(7); console.log('clamp', R.lift.clamp, R.lift.msg);
R.cmd.vx = 0; run(1); console.log('carry v', R.telemetry().fwdSpeed.toFixed(2), 'margin', R.telemetry().margin.toFixed(1), 'cargo state', c.state);
if (!process.env.TRACE) { let mn = 99, worst = null; R.cmd.vx = 0; run(4);
  for (const [vx, wz, t] of [[0.5, 0, 6], [0, 0, 3], [0.5, 0.04, 6], [0, 0, 3], [-0.4, 0, 5], [0, 0, 3]]) { R.cmd.vx = vx; R.cmd.wz = wz; run(t, () => { if (R.tilt.margin < mn) { mn = R.tilt.margin; worst = { vx, wz, contacts: R.wheels.map((w) => w.contact ? 1 : 0).join(''), loads: R.wheels.map((w) => (w.load / 1000).toFixed(1)).join(' '), com: R.comWorld.clone().sub(new THREE.Vector3(R.pose().x, R.pose().y, R.pose().z)).toArray().map((v) => v.toFixed(2)) }; } }); }
  console.log('stability min margin', mn.toFixed(1), JSON.stringify(worst)); }
if (process.env.TRACE) { let k = 0; R.cmd.vx = 0; R.cmd.wz = 0; console.log('--- trace after stability');
  for (const [vx, t] of [[0, 4], [0.5, 6], [0, 6], [-0.4, 5], [0, 5]]) { R.cmd.vx = vx; run(t, () => { if (k++ % 60 === 0) { const tl = R.telemetry(); console.log(`vx ${vx} v ${tl.fwdSpeed.toFixed(2)} pitch ${(tl.pitch * 57.3).toFixed(1)} roll ${(tl.roll * 57.3).toFixed(1)} margin ${R.tilt.margin.toFixed(1)} L ${R.bogies.map((b) => b.L.toFixed(2)).join(' ')} load ${R.wheels.map((w) => (w.load / 1000).toFixed(1)).join(' ')} ride ${R.lift.pos.toFixed(2)}`); } }); } }

if (process.env.PIVOT) { const w = +process.env.PIVOT; R.cmd.vx = 0; R.cmd.wz = 0; run(3);
  let mp = 0, mr = 0; R.cmd.wz = w; run(20, () => { const tl = R.telemetry(); mp = Math.max(mp, Math.abs(tl.pitch)); mr = Math.max(mr, Math.abs(tl.roll)); });
  console.log('pivot wz', w, 'max pitch', (mp * 57.3).toFixed(1), 'max roll', (mr * 57.3).toFixed(1), 'yaw rate', R.body.angvel().y.toFixed(3)); }
