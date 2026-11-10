import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER, { seed: 866201175 });
const R = sim.rover, deg = (r) => r * 180 / Math.PI;
const run = (s, cb) => { for (let i = 0; i < s * 120; i++) { sim.stepOnce(); cb && i % 30 === 0 && cb(); } };
run(3);
const P = WORLD.pad, LS = 2.6, a = Math.PI / 4;
const tx = P.x + Math.cos(a) * 5.4 * LS, tz = P.z + Math.sin(a) * 5.4 * LS;
const p1 = R.pose(); R.teleport(p1.x, p1.z, Math.atan2(-(tz - p1.z), tx - p1.x)); run(2);
R.brake = false; R.cmd.vx = 3; run(36);
R.cmd.vx = -2;
const W = sim.world;
run(8, () => {
  const p = R.pose(), t = R.telemetry();
  const touch = [];
  const n = R.body.numColliders();
  for (let i = 0; i < n; i++) { const c = R.body.collider(i); W.contactPairsWith(c, (o) => { let has = false; W.contactPair(c, o, (m) => { if (m.numContacts() > 0) has = true; }); if (has) { const g = (o.collisionGroups() >>> 16); const sh = o.shape.type; touch.push(`c${i}->g${g}/s${sh}@${o.translation().y.toFixed(1)}`); } }); }
  console.log(sim.time.toFixed(2), 'y', p.y.toFixed(2), 'roll', deg(t.roll).toFixed(1), 'pitch', deg(t.pitch).toFixed(1), 'wheels', R.wheels.map((w) => (w.blocked ? 'B' : w.contact ? 'c' : '-') + (w.cp ? w.cp.y.toFixed(1) : '')).join(' '), touch.join(' '));
});
