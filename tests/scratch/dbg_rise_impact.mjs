// What the module feels when the saddles meet it during the rise. Autonomous approach (approachOnly), then the rise is commanded here:
//   single  one stroke kneel -> carry          two  kneel -> just under contact, pause, -> carry
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
await RAPIER.init();
const mode = process.argv[2] || 'single', ci = Number(process.argv[3] || 0);
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover, c = sim.cargo[ci], T = c.T, ct = c.body.translation();
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
A.start('approach', A.acquire(c, { approachOnly: true })); let t0 = sim.time; while (A.gen && sim.time - t0 < 600) sim.stepOnce();
const c0 = c.body.translation(), y0 = c0.y; let vPk = 0, aPk = 0, shift = 0, fPk = 0, pv = 0, pitchPk = 0, tDone = null, tStart = sim.time;
const step = () => { sim.stepOnce(); const v = c.body.linvel(), t = c.body.translation(); vPk = Math.max(vPk, Math.abs(v.y)); aPk = Math.max(aPk, Math.abs(v.y - pv) * 120); pv = v.y;
  shift = Math.max(shift, Math.hypot(t.x - c0.x, t.z - c0.z)); const f = sim.cargoForces.get(c); if (f) fPk = Math.max(fPk, f.total); pitchPk = Math.max(pitchPk, Math.abs(R.tilt.pitch)); };
const go = (target, sec) => { R.lift.target = target; for (let i = 0; i < sec * 120; i++) { step(); if (tDone == null && target === ROVER.ride.carry && Math.abs(R.rideActual - target) < 0.02) tDone = sim.time - tStart; } };
if (mode === 'two') { go(ROVER.ride.contact - 0.04, 2.2); go(ROVER.ride.carry, 8); } else go(ROVER.ride.carry, 10);
const g = R.grappleCheck();
console.log(`${mode} ${c.id}: module lifted ${(c.body.translation().y - y0).toFixed(2)} m | struts at carry after ${tDone?.toFixed(1)} s | module peak v ${vPk.toFixed(2)} m/s, peak accel ${aPk.toFixed(1)} m/s2 (${(aPk / 1.62).toFixed(1)} lunar g) | horizontal shift ${(shift * 100).toFixed(1)} cm | peak contact ${(fPk / 1000).toFixed(0)} kN (weight ${(T.mass * 1.62 / 1000).toFixed(0)} kN) | rover pitch ${(pitchPk * 57.3).toFixed(2)} deg | grapple ${g.ok ? 'OK' : g.why}`);
