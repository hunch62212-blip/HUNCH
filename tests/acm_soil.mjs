// ACM-on-regolith contact (plan s5.5, P1c): Bekker sinkage, creep settling, breakout adhesion on lift, Mohr-Coulomb sliding.
import { check, within, atLeast, atMost } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { buildWorld } from '../src/sim/world.js';
import { acmSoilStep, acmFootSummary } from '../src/sim/cargoContact.js';
import { plateSinkage, plateLoad } from '../src/sim/terramechanics.js';
import { heightAt } from '../src/sim/terrain.js';
import { MOON, REGOLITH, ACM_SOIL as S, CARGO_TYPES } from '../src/sim/config.js';
await RAPIER.init();
const W = buildWorld(RAPIER), DT = 1 / 120, mm = 1000;
const T = CARGO_TYPES.acm, Wt = T.mass * MOON.g, b = S.foot, A = b * b;

// Nominal-soil expectations are derived from the foot geometry, not hard-coded: at K = 8.16 the foot is k(0.03) = 0.245 m
// (A = 0.060 m^2) under 2,400 kg / 4 feet = 972 N, i.e. 16.2 kPa and 19.6 mm of Bekker sinkage (the 1:16.3 numbers were
// 4.05 kPa, 4.9 mm, 5.7 mm at 600 s). The ACM mass is not rescaled with K (owner decision pending, see CHANGES.md 35).
const pFoot = Wt / 4 / A, zNom = pFoot / (REGOLITH.kc / b + REGOLITH.kphi) * mm; // closed-form Bekker, n = 1
within('plateSinkage(foot pressure) = p / (kc/b + kphi) nominal soil', plateSinkage(pFoot, b) * mm, zNom * 0.99, zNom * 1.01, ' mm');
within('plateLoad inverts plateSinkage', plateLoad(plateSinkage(4050, b), b, A), 4050 * A * 0.999, 4050 * A * 1.001, ' N', 1);

// Quantitative cases run on a flat, nominal-soil test plane (the contact never touches the Rapier heightfield; it reads
// terrain.h and soilVary). The last case repeats a short sit on the real, rough terrain with its soil variation.
const realH = W.terrain.h.slice(), realVary = W.terrain.soilVary;
const flatten = (on) => { if (on) { W.terrain.h.fill(5); W.terrain.soilVary = null; } else { W.terrain.h.set(realH); W.terrain.soilVary = realVary; } };
flatten(true);
const site = { x: 20, z: 20, k: 1 };
const k = site.k;
const soilK = { ...REGOLITH, kc: REGOLITH.kc * k, kphi: REGOLITH.kphi * k, c: REGOLITH.c * k };
const zStatic = plateSinkage(Wt / 4 / A, b, soilK);

const step = (c, n, f) => { for (let i = 0; i < n; i++) { f && f(i * DT); acmSoilStep(c, DT, W); W.world.step(W.eventQueue); } };
const meanD = (c) => c.feet.reduce((a, f) => a + f.d, 0) / c.feet.length;

// 1. drop onto the surface from rest, settle, then creep for 600 s
{
  const c = W.spawnACM(site.x, site.z, 0.3, { settled: false, drop: 0 });
  check('ACM registered as cargo type acm', W.cargo.includes(c) && c.type === 'acm' && c.T.handling === 'arm');
  step(c, 10 * 120);
  const z10 = meanD(c);
  within('sinkage after 10 s vs Bekker (local soil)', z10 / zStatic, 0.9, 1.1, ' x');
  within('sinkage after 10 s, nominal Bekker +-10 %', z10 * mm, zNom * 0.9, zNom * 1.1, ' mm');
  within('foot loads sum to the lunar weight', acmFootSummary(c).N / Wt, 0.98, 1.02, ' x');
  const spread = Math.max(...c.feet.map((f) => f.N)) - Math.min(...c.feet.map((f) => f.N));
  atMost('foot load spread on flat ground', spread / (Wt / 4), 0.05, '', 3);
  step(c, 590 * 120);
  const z600 = meanD(c);
  within('sinkage after 600 s vs 1.15 x Bekker', z600 / (1.15 * zStatic), 0.9, 1.1, ' x');
  within('sinkage after 600 s, nominal 1.15 x Bekker +-10 %', z600 * mm, zNom * 1.15 * 0.9, zNom * 1.15 * 1.1, ' mm');
  atLeast('settling grows the sinkage 10 s -> 600 s', z600 / z10, 1.08, ' x');
  const pa = c.body.translation(); step(c, 5 * 120); const pb = c.body.translation();
  atMost('at rest after settling: drift over 5 s', Math.hypot(pb.x - pa.x, pb.y - pa.y, pb.z - pa.z) * mm, 0.5, ' mm', 4);
  console.log(`  sinkage: ${(z10 * mm).toFixed(2)} mm @10 s, ${(z600 * mm).toFixed(2)} mm @600 s (Bekker local ${(zStatic * mm).toFixed(2)} mm, soil k ${k.toFixed(3)})`);

  // 2. breakout: the hook is the plan's spring-damper (k_h, c_h of s5.4) whose anchor rises at seatV
  const vT = 0.02, y0 = c.body.translation().y, kh = T.mass * (2 * Math.PI * 3) ** 2, ch = 2 * 0.5 * Math.sqrt(kh * T.mass);
  let peak = 0, peakSoil = 0, minN = 0;
  step(c, 12 * 120, (tt) => {
    const y = c.body.translation().y - y0, v = c.body.linvel();
    const F = Math.max(0, kh * (vT * tt - y) + ch * (vT - v.y));
    c.body.resetForces(true); c.body.addForce({ x: 0, y: F, z: 0 }, true);
    const Nsum = acmFootSummary(c).N;
    peak = Math.max(peak, F); minN = Math.min(minN, Nsum); peakSoil = Math.max(peakSoil, Wt - Nsum);
  });
  c.body.resetForces(true);
  const rise = c.body.translation().y - y0;
  atLeast('ACM lifted clear of the regolith', rise, 0.1, ' m');
  atMost('peak breakout load W - sum(N) (soil only) / W', peakSoil / Wt, 1.05, '', 4);
  atLeast('adhesion adds some breakout load (> W)', peakSoil / Wt, 1.001, '', 4);
  atMost('peak spring-hook force at seatV (soil + hook dynamics) / W', peak / Wt, 1.25, '', 3);
  atMost('foot suction limited to -cAdh*A per foot', -minN, 4 * S.cAdhK * soilK.c * A * 1.001 + 1, ' N', 1);
  check('feet released after lift (N = 0)', Math.abs(acmFootSummary(c).N) < 1e-6, `${acmFootSummary(c).contacts} feet in contact`);
  console.log(`  breakout: soil load ${(peakSoil / Wt).toFixed(4)} W, hook peak ${(peak / Wt).toFixed(3)} W, max suction ${(-minN).toFixed(1)} N`);
}

// 3. sliding friction: constant horizontal force above / below the Mohr-Coulomb limit
let slideRuns = 0;
function slideTest(frac) {
  // each run gets its own spot: a second ACM spawned on top of the first one's (still standing) collider is pushed apart
  // by Rapier, which at the K = 8.16 sinkage over-loads the feet (sliding resistance read 0.91 x the limit)
  const c = W.spawnACM(site.x + 12 + 40 * slideRuns++, site.z, 0, { settled: true });
  step(c, 4 * 120);
  const Fmax = c.feet.reduce((a, f) => a + soilK.c * A + Math.max(f.N, 0) * Math.tan(soilK.phi), 0);
  const p0 = c.body.translation();
  step(c, 8 * 120, () => c.body.applyImpulse({ x: frac * Fmax * DT, y: 0, z: 0 }, true));
  const p1 = c.body.translation();
  const Ft = c.feet.reduce((a, f) => a + f.Ft, 0), slipping = c.feet.every((f) => f.slipping);
  const v = c.body.linvel().x;
  return { Fmax, dx: p1.x - p0.x, Ft, slipping, v };
}
{
  const lo = slideTest(0.8);
  // holding = shear creep inside the footprint (rear feet are unloaded by the pitch load transfer and shift a little), not sliding (metres): 10 % of the foot width
  atMost('below the limit the ACM holds (displacement)', lo.dx * mm, 0.1 * b * mm, ' mm', 2);
  check('below the limit no foot slides', !lo.slipping);
  const hi = slideTest(1.3);
  const aExp = (1.3 - 1) * hi.Fmax / T.mass, aMeas = hi.dx * 2 / 8 ** 2;
  atLeast('above the limit the ACM slides', hi.dx, 1, ' m');
  check('all feet slip above the limit', hi.slipping);
  within('sliding resistance = c*A + N*tan(phi) (sum of foot shear)', hi.Ft / hi.Fmax, 0.97, 1.03, ' x');
  within('acceleration under (1.3 - 1) x limit', aMeas / aExp, 0.8, 1.2, ' x');
  console.log(`  slide limit ${hi.Fmax.toFixed(0)} N (mu_eff ${(hi.Fmax / Wt).toFixed(3)}), a=${aMeas.toFixed(3)} (exp ${aExp.toFixed(3)}) m/s^2`);
}

// 4. real terrain with soil variation: sits stably, carries the weight, no flip
{
  flatten(false);
  let best = null;
  for (let x = -300; x <= 300; x += 5) for (let z = -300; z <= 300; z += 5) {
    const hs = [[-3, -3], [3, -3], [-3, 3], [3, 3]].map(([dx, dz]) => heightAt(W.terrain, x + dx, z + dz));
    const f = Math.max(...hs) - Math.min(...hs);
    if (!best || f < best.f) best = { x, z, f };
  }
  const c = W.spawnACM(best.x, best.z, 1.1, { settled: false, drop: 0.01 });
  step(c, 30 * 120);
  const q = c.body.rotation(), tilt = 2 * Math.asin(Math.min(1, Math.hypot(q.x, q.z)));
  within('rough terrain: foot loads carry the weight', acmFootSummary(c).N / Wt, 0.97, 1.03, ' x');
  atMost('rough terrain: body tilt (terrain spread ' + (best.f * 100).toFixed(1) + ' cm)', tilt * 180 / Math.PI, 3, ' deg', 2);
  atMost('rough terrain: sinkage bounded', Math.max(...c.feet.map((f) => f.d)) * mm, 40, ' mm', 1);
  console.log('  feet N/W4:', c.feet.map((f) => (f.N / (Wt / 4)).toFixed(2)).join(' '), 'zp mm:', c.feet.map((f) => (f.zp * mm).toFixed(1)).join(' '));
}
