import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
for (let i = 0; i < 240; i++) sim.stepOnce();
for (const c of sim.cargo) { const r = c.body.rotation(), q = new THREE.Quaternion(r.x, r.y, r.z, r.w), f = new THREE.Vector3(1, 0, 0).applyQuaternion(q), t = c.body.translation();
  const yawBody = Math.atan2(-f.z, f.x), d = Math.atan2(Math.sin(yawBody - c.yaw), Math.cos(yawBody - c.yaw));
  console.log(c.id, 'c.yaw', (c.yaw * 57.3).toFixed(2), 'body yaw', (yawBody * 57.3).toFixed(2), 'diff deg', (d * 57.3).toFixed(3), '| initial', c.initial && [c.initial.x, c.initial.y, c.initial.z].map((v) => v.toFixed(2)).join(','), 'now', [t.x, t.y, t.z].map((v) => v.toFixed(2)).join(','), '| carryX', c.T.carryX, 'adapterX', c.T.adapterX); }
