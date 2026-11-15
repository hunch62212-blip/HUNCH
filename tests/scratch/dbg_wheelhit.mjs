// What the wheel shape-casts are landing on during the last 1.5 m under a module (anything that is not the terrain heightfield).
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER, GROUPS } from '../../src/sim/config.js';
await RAPIER.init();
const ci = Number(process.argv[2] || 2);
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover, c = sim.cargo[ci], T = c.T, ct = c.body.translation();
const cy = Math.cos(c.yaw), sy = Math.sin(c.yaw);
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - cy * back, ct.z + sy * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
const real = sim.world.castShape.bind(sim.world); let rec = null;
sim.world.castShape = (...a) => { const h = real(...a); if (rec && h && h.collider.handle !== sim.terrainCollider.handle) { const g = h.collider.collisionGroups() >>> 16, own = sim.colliderOwner.get(h.collider.handle);
    const kind = own ? own.kind + (own.ref?.id ? ':' + own.ref.id : '') : sim.colliderRock.has(h.collider.handle) ? 'rock' : (g & GROUPS.STRUCT ? 'struct' : 'group ' + g);
    const p = R.pose(), o = a[0], bx = (o.x - p.x) * Math.cos(p.yaw) - (o.z - p.z) * Math.sin(p.yaw), bz = (o.x - p.x) * Math.sin(p.yaw) + (o.z - p.z) * Math.cos(p.yaw);
    const key = `${kind} under wheel at body (${bx.toFixed(1)}, ${bz.toFixed(1)})`; const r = rec.get(key) || { n: 0, ny: 0, toi: 0, nlat: 0 }; r.n++; r.ny += h.normal1.y; r.toi += h.time_of_impact;
    r.nlat += h.normal1.x * Math.sin(p.yaw) + h.normal1.z * Math.cos(p.yaw); rec.set(key, r); } return h; };
const t0 = sim.time; A.start('pickup', A.acquire(c)); let i = 0;
while (A.gen && sim.time - t0 < 75) { rec = /Drive under/.test(A.phase) ? (rec || new Map()) : null; sim.stepOnce(); if (!rec) continue;
  const p = R.pose(), dx = p.x - ct.x, dz = p.z - ct.z, along = -(dx * cy - dz * sy) - (T.carryX || 0), lat = dx * sy + dz * cy;
  if (++i % 240) continue;
  if (rec.size) console.log(`along ${along.toFixed(2)} lat ${(lat * 100).toFixed(0)} cm: ` + [...rec].map(([k, r]) => `${k}: ${r.n} casts, normal up ${(r.ny / r.n).toFixed(2)} lateral ${(r.nlat / r.n).toFixed(2)}, cast length ${(r.toi / r.n).toFixed(2)} m`).join(' ; '));
  rec = new Map(); }
console.log('done');
