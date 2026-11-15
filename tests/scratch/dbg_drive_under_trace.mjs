// Trace of the pod-rack approach: phase, detail, true lateral / yaw error to the module axis, every 2 s.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
import { gauss } from '../../src/sim/sensors.js';
await RAPIER.init();
const ci = Number(process.argv[2] || 2), skip = Number(process.argv[3] || 0);
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover, c = sim.cargo[ci], T = c.T, ct = c.body.translation();
for (let i = 0; i < skip; i++) gauss();
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const t0 = sim.time; A.start('pickup', A.acquire(c)); let i = 0;
const touching = () => { const w = sim.world, out = new Map();
  for (let k = 0; k < R.body.numColliders(); k++) { const col = R.body.collider(k);
    w.contactPairsWith(col, (o) => { w.contactPair(col, o, (man) => { if (man.numContacts() > 0) { const own = sim.colliderOwner.get(o.handle); const key = own ? `${own.kind}${own.ref?.id ? ':' + own.ref.id : ''}` : (o.handle === sim.terrainCollider.handle ? null : sim.colliderRock.has(o.handle) ? 'rock' : 'static#' + o.handle); if (key) out.set(key + ' vs rover collider ' + k, 1); } }); }); }
  return [...out.keys()]; };
const START = Number(process.argv[4] || 44);
while (A.gen && sim.time - t0 < 90) { sim.stepOnce(); if (sim.time - t0 > START && /Drive under/.test(A.phase) && i % 60 === 0) { const tc = touching(); const cn = c.body.translation(); const f = sim.cargoForces.get(c);
    if (tc.length || Math.hypot(cn.x - ct.x, cn.z - ct.z) > 0.005) console.log(`   ${(sim.time - t0).toFixed(1)}s CONTACT ${tc.join(' ; ') || '-'} | module moved ${(Math.hypot(cn.x - ct.x, cn.z - ct.z) * 100).toFixed(1)} cm | module contact force ${(f ? f.total / 1000 : 0).toFixed(1)} kN | roll ${(R.tilt.roll * 57.3).toFixed(2)} pitch ${(R.tilt.pitch * 57.3).toFixed(2)} | body y ${R.pose().y.toFixed(2)} rideActual ${(R.bogies.reduce((a, b) => a + b.L, 0) / 6 - ROVER.bogie.Lnom).toFixed(2)}`); }
  if (++i % 240) continue;
  const p = R.pose(), dx = p.x - ct.x, dz = p.z - ct.z, cy = Math.cos(c.yaw), sy = Math.sin(c.yaw);
  const along = -(dx * cy - dz * sy), lat = dx * sy + dz * cy, g = A.cargoGuess(c), e = sim.slam.est;
  console.log(`${(sim.time - t0).toFixed(0).padStart(3)}s ${A.phase.slice(0, 22).padEnd(22)} TRUE along ${along.toFixed(2)} lat ${(lat * 100).toFixed(0).padStart(4)} cm yaw ${(wrap(p.yaw - c.yaw) * 57.3).toFixed(2).padStart(6)} | guess yaw err ${(wrap(g.yaw - c.yaw) * 57.3).toFixed(2)} pos err ${Math.hypot(g.x - ct.x - (e.x - p.x), g.z - ct.z - (e.z - p.z)).toFixed(2)} m src ${g.src} | est yaw err ${(wrap(e.yaw - p.yaw) * 57.3).toFixed(2)} | v ${R.telemetry().fwdSpeed.toFixed(2)} ride ${R.lift.pos.toFixed(2)} | ${(A.detail || '').slice(0, 70)}`); }
console.log(R.loaded ? 'HELD' : 'not held at 90 s');
