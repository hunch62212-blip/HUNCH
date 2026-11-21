// ACM-on-regolith contact (plan s5.5). The ACM box collider does not collide with TERRAIN (world.js G_ACM): four
// feet carry it as custom force elements, like the wheels. Per foot:
//   - Bekker pressure-sinkage loading, plastic sinkage memory z_p, stiff unloading (K_u x secant stiffness)
//   - normal damping (zeta) while the foot is pressed, creep of z_p toward 1.15 z_eq over tauSettle
//   - breakout adhesion: N >= -cAdhK * c * A while the foot is still in its imprint
//   - bristle shear anchor (k_b), Mohr-Coulomb capped: |F_t| <= c A + N tan(phi), else the anchor slides
import { REGOLITH as R, ACM_SOIL as S, CARGO_TYPES } from './config.js';
import { heightAt, soilAt } from './terrain.js';
import { plateSinkage, plateLoad } from './terramechanics.js';

// Shared ACM geometry (body frame: origin at the CG, +x forward, +y up, +z right; metres). Used by world.js
// (colliders) and props.js (visual) so physics == visuals. dims = [x, y, z]; trunnion.y and cgY are measured up
// from the foot soles (the ground plane).
export function acmGeom(T = CARGO_TYPES.acm, soleInset = 0.16) {
  const [L, H, W] = T.dims, foot = S.foot, footH = foot * 0.17;           // pad height ~ 5 mm / 30 mm of the CAD
  const cg = T.cgY;
  const fx = L / 2 - foot / 2 - soleInset, fz = W / 2 - foot / 2 - soleInset;
  return {
    L, H, W, foot, footH, cg,
    groundY: -cg,                                  // local y of the foot soles
    boxY: footH + H / 2 - cg,                      // local y of the shell centre
    trunY: T.trunnion.y - cg,                      // local y of the trunnion axis (20 mm above CG at 1:16)
    feet: [[fx, fz], [fx, -fz], [-fx, fz], [-fx, -fz]].map(([x, z]) => ({ x, z })),
  };
}

export class FootSoil {
  constructor(x, z) {
    this.x = x; this.z = z;                        // body-frame position
    this.N = 0; this.zp = 0; this.d = 0; this.Ft = 0; this.Fx = 0; this.Fz = 0;
    this.contact = false; this.slipping = false; this.zEq = 0;
    this.anchored = false; this.ax = 0; this.az = 0; this.hasPrev = false; this.px0 = 0; this.py0 = 0; this.pz0 = 0;
    this.soil = { ...R }; this.k = 1;
  }
}

export function makeFeet(T = CARGO_TYPES.acm) { return acmGeom(T).feet.map((f) => new FootSoil(f.x, f.z)); }

const _s = {};
function localSoil(f, terrain, x, z) {
  soilAt(terrain, x, z, _s);
  const o = f.soil; f.k = _s.k;
  o.n = R.n; o.kc = R.kc * _s.k; o.kphi = R.kphi * _s.k; o.c = R.c * _s.k; o.phi = R.phi + _s.dphi; o.K = R.K;
  return o;
}

// Advance one foot given its vertical clearance and point velocity. Returns { N, Fx, Fz } (world, N up).
// Pure on the FootSoil state, so tests can drive it without Rapier.
export function footStep(f, dt, ctx) {
  const { px, pz, pyBottom, hTerr, vx, vy, vz, mShare } = ctx;
  const soil = ctx.soil, b = S.foot, A = b * b;
  const d = hTerr - pyBottom;                      // penetration
  f.d = d;
  if (d <= 0) {                                    // foot is clear of the surface: forget the imprint
    f.N = 0; f.zp = 0; f.contact = false; f.anchored = false; f.slipping = false; f.Ft = f.Fx = f.Fz = 0; f.zEq = 0;
    return { N: 0, Fx: 0, Fz: 0 };
  }
  f.contact = true;
  const kSec0 = A * (soil.kc / b + soil.kphi);
  let Nel;
  if (d >= f.zp) { f.zp = d; Nel = plateLoad(d, b, A, soil); }              // loading
  else {                                                                    // unloading along a stiff line
    const Np = plateLoad(f.zp, b, A, soil), Ku = S.Ku * Np / f.zp;
    Nel = Np - Ku * (f.zp - d);
  }
  // normal damping, c = 2 zeta sqrt(k m), only while the elastic force presses
  const cs = 2 * S.zeta * Math.sqrt(kSec0 * mShare);
  let N = Nel;
  if (Nel > 0) N = Math.max(0, Nel + cs * (-vy));
  const nAdh = -S.cAdhK * soil.c * A;
  if (N < nAdh) N = nAdh;
  f.N = N;
  // creep: plastic sinkage drifts to 1.15 z_eq(current load), never backwards
  if (Nel > 0) {
    f.zEq = plateSinkage(Nel / A, b, soil);
    const dz = (1 + S.creep) * f.zEq - f.zp;
    if (dz > 0) f.zp += dz / S.tauSettle * dt;
  }
  // bristle shear
  if (!f.anchored) { f.anchored = true; f.ax = px; f.az = pz; }
  const kb = S.bristleK, cb = 2 * Math.sqrt(kb * mShare);
  let Fx = -kb * (px - f.ax) - cb * vx, Fz = -kb * (pz - f.az) - cb * vz;
  const Fmax = soil.c * A + Math.max(N, 0) * Math.tan(soil.phi);
  const mag = Math.hypot(Fx, Fz);
  f.slipping = false;
  if (mag > Fmax) {
    const s = Fmax / mag; Fx *= s; Fz *= s;
    f.ax = px + Fx / kb; f.az = pz + Fz / kb;     // anchor slides, dragged at the Coulomb limit
    f.slipping = true;
  }
  f.Fx = Fx; f.Fz = Fz; f.Ft = Math.hypot(Fx, Fz);
  return { N, Fx, Fz };
}

// Per-physics-step: apply the four feet's forces to the ACM body as impulses. `sim` needs `.terrain`.
// Call for every ACM in GROUND mode (before world.step). Stores c.feet[i].{N, zp, d, Ft, slipping}.
export function acmSoilStep(c, dt, sim) {
  const body = c.body;
  if (!body || !body.isDynamic()) return;
  const g = c.geom || (c.geom = acmGeom(c.T));
  const feet = c.feet || (c.feet = makeFeet(c.T));
  const t = body.translation(), q = body.rotation(), v = body.linvel(), w = body.angvel();
  const mShare = c.T.mass / feet.length;
  const qx = q.x, qy = q.y, qz = q.z, qw = q.w;
  c.soilN = 0;
  for (let i = 0; i < feet.length; i++) {
    const f = feet[i];
    // r = q * (fx, groundY, fz)
    const lx = f.x, ly = g.groundY, lz = f.z;
    const tx = 2 * (qy * lz - qz * ly), ty = 2 * (qz * lx - qx * lz), tz = 2 * (qx * ly - qy * lx);
    const rx = lx + qw * tx + (qy * tz - qz * ty), ry = ly + qw * ty + (qz * tx - qx * tz), rz = lz + qw * tz + (qx * ty - qy * tx);
    const px = t.x + rx, py = t.y + ry, pz = t.z + rz;
    let vx = v.x + w.y * rz - w.z * ry, vy = v.y + w.z * rx - w.x * rz, vz = v.z + w.x * ry - w.y * rx;
    // Rapier's reported velocity has the step's gravity kick folded in (about -g dt at rest, which the damping term would
    // read as a sink rate and under-load the foot by ~9 %): use the foot point's finite-difference velocity instead.
    if (f.hasPrev) { vx = (px - f.px0) / dt; vy = (py - f.py0) / dt; vz = (pz - f.pz0) / dt; }
    f.px0 = px; f.py0 = py; f.pz0 = pz; f.hasPrev = true;
    const soil = localSoil(f, sim.terrain, px, pz);
    const o = footStep(f, dt, { px, pz, pyBottom: py, hTerr: heightAt(sim.terrain, px, pz), vx, vy, vz, mShare, soil });
    if (o.N !== 0 || o.Fx !== 0 || o.Fz !== 0) body.applyImpulseAtPoint({ x: o.Fx * dt, y: o.N * dt, z: o.Fz * dt }, { x: px, y: py, z: pz }, true);
    c.soilN += o.N;
  }
}

// Total vertical foot force (N) and sinkage summary, for telemetry / the pick state machine.
export function acmFootSummary(c) {
  const f = c.feet || [];
  let N = 0, d = 0, n = 0;
  for (const x of f) { N += x.N; if (x.contact) { d += x.d; n++; } }
  return { N, sinkage: n ? d / n : 0, contacts: n };
}
