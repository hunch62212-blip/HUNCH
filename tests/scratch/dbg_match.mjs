// What does LiDAR scan-to-map matching do on the pad->hab drive? For every match call: was it accepted,
// which axes the degeneracy check zeroed, and whether the applied correction moved the estimate toward
// the true pose ("helped") or away from it. Sums along-track (heading) and cross-track parts separately.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover, S = sim.slam;
const H = sim.terrain.hab;
A.start('goto', A.missionGoto(WORLD.habitat.x + H.nx * 45, WORLD.habitat.z + H.nz * 45, 'hab'));
const tmax = +(process.argv[2] || 240);
const st = { calls: 0, skipped: 0, ok: 0, rejected: 0, degX: 0, degZ: 0, alongApplied: 0, alongNeeded: 0, crossApplied: 0, crossNeeded: 0, rawAlong: 0 };
const orig = S._match.bind(S);
S._match = (lev, m) => {
  st.calls++;
  const p = R.pose(), e0 = { x: S.est.x, z: S.est.z };
  const fx = Math.cos(p.yaw), fz = -Math.sin(p.yaw); // heading in world x,z
  const errAlong = (p.x - e0.x) * fx + (p.z - e0.z) * fz, errCross = -(p.x - e0.x) * fz + (p.z - e0.z) * fx;
  const t0 = S.lastMatch?.t;
  orig(lev, m);
  if (S.lastMatch?.t !== sim.time) { st.skipped++; return; }
  if (!S.lastMatch.ok) { st.rejected++; return; }
  st.ok++; if (S.degenerate?.x) st.degX++; if (S.degenerate?.z) st.degZ++;
  const dx = S.est.x - e0.x, dz = S.est.z - e0.z;
  st.alongApplied += dx * fx + dz * fz; st.crossApplied += -dx * fz + dz * fx;
  st.rawAlong += S.lastMatch.dx * fx + S.lastMatch.dz * fz;
  st.alongNeeded += errAlong; st.crossNeeded += errCross;
};
let last = 0; let odo0 = 0, true0 = 0;
while (A.gen && sim.time < tmax) {
  sim.stepOnce();
  if (sim.time - last >= 60) { last = sim.time; const p = R.pose(), e = S.est; console.log(`t=${sim.time.toFixed(0)} err ${Math.hypot(e.x - p.x, e.z - p.z).toFixed(1)} m | calls ${st.calls} accepted ${st.ok} rejected ${st.rejected} no-run ${st.skipped} | degenerate x ${st.degX} z ${st.degZ}`); }
}
console.log(`sum of applied along-track corrections ${st.alongApplied.toFixed(2)} m (before degeneracy/gain ${st.rawAlong.toFixed(2)} m); cross-track ${st.crossApplied.toFixed(2)} m`);
console.log(`mean along-track error at accepted matches ${(st.alongNeeded / Math.max(1, st.ok)).toFixed(2)} m, cross ${(st.crossNeeded / Math.max(1, st.ok)).toFixed(2)} m`);
console.log(`cellsKnown ${S.cellsKnown}`);
