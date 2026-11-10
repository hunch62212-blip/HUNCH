// When the local planner finds no safe arc, are the hazards it hit real? For each hit point: the map's wheel
// hazard (step, slope) vs the same test on the true world geometry (ray casts down through terrain + rocks +
// structures) at the matching true position. Usage: node dbg_boxed.mjs [cycle|hab|faults] [tmax]
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
import { Q_TERRAIN } from '../../src/sim/world.js';
import { runCommand } from '../../src/sim/commands.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover, S = sim.slam;
const which = process.argv[2] || 'cycle', tmax = +(process.argv[3] || 700);
if (which === 'cycle') A.start('cycle', A.missionCycle());
else { if (which === 'faults') sim._faultsAt = 300; const H = sim.terrain.hab; A.start('goto', A.missionGoto(WORLD.habitat.x + H.nx * 45, WORLD.habitat.z + H.nz * 45, 'hab')); }
const ray = new RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 0, y: -1, z: 0 });
const trueH = (x, z) => { ray.origin.x = x; ray.origin.y = 400; ray.origin.z = z; const h = sim.world.castRay(ray, 800, true, undefined, Q_TERRAIN, undefined, R.body); return h ? 400 - h.timeOfImpact : NaN; };
// same stencil and limits as Slam.wheelHazard
const hazard = (hf, x, z) => { const h0 = hf(x, z); if (h0 !== h0) return null; const v = [hf(x + .5, z), hf(x - .5, z), hf(x, z + .5), hf(x, z - .5)];
  let mx = h0, mn = h0; for (const q of v) if (q === q) { mx = Math.max(mx, q); mn = Math.min(mn, q); }
  const gx = v[0] - v[1], gz = v[2] - v[3], slope = Math.hypot(gx === gx ? gx : 0, gz === gz ? gz : 0), step = mx - mn - slope * 0.5;
  const L = S.limits; return { step, slopeDeg: Math.atan(slope) * 57.3, lethal: step > L.stepLethal || slope > Math.tan(L.slopeLethal / 57.3) * 1.05 }; };
let dumps = 0;
const orig = A.local.evaluate.bind(A.local);
A.local.evaluate = (pose, tgt, v, loaded, opts) => {
  const r = orig(pose, tgt, v, loaded, opts);
  if (!r.ok && dumps < 6 && sim.time - (A._dbgT || -99) > 15) {
    A._dbgT = sim.time; dumps++;
    const p = R.pose(), e = S.est, ox = p.x - e.x, oz = p.z - e.z;
    let real = 0, phantom = 0, unk = 0; const ex = [];
    for (const h of r.hits) {
      const m = hazard((x, z) => S.heightAt(x, z), h.x, h.z), t = hazard(trueH, h.x + ox, h.z + oz);
      if (!t) unk++; else if (t.lethal) real++; else phantom++;
      if (ex.length < 4) ex.push(`map step ${m?.step.toFixed(2)} slope ${m?.slopeDeg.toFixed(0)} | true step ${t?.step.toFixed(2)} slope ${t?.slopeDeg.toFixed(0)}`);
    }
    console.log(`t=${sim.time.toFixed(0)} ${opts?.dir === -1 ? 'REVERSE' : 'forward'} loaded=${loaded} slamErr ${Math.hypot(ox, oz).toFixed(1)} m | hits ${r.hits.length}: real ${real}, phantom ${phantom}, no-truth ${unk} | limits step ${S.limits.stepLethal} slope ${S.limits.slopeLethal}`);
    for (const s of ex) console.log('   ', s);
  }
  return r;
};
sim.on('log', (e) => { if (/boxed|FAILED|Tilt reflex/.test(e.msg)) console.log(`[${e.t.toFixed(0)}] ${e.msg}`); });
while (A.gen && sim.time < tmax) { sim.stepOnce(); if (sim._faultsAt && sim.time > sim._faultsAt) { sim._faultsAt = 0; runCommand(sim, {}, 'fault block'); runCommand(sim, {}, 'fault wheel 3'); } }
