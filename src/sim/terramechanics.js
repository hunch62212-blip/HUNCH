// Wheel-soil interaction for lunar regolith.
//  - Sinkage: Bekker pressure-sinkage for an (equivalent) rigid wheel, with slip-sinkage (Lyasko)
//  - Compaction (rolling) resistance: Bekker
//  - Thrust: Janosi-Hanamoto shear stress-displacement integrated over the contact patch
//  - Lateral: same shear law driven by lateral slip (tan alpha)
import { REGOLITH as R } from './config.js';

export function wheelSoil(W, b, D, slip, tanAlpha, soil = R) {
  if (W <= 1) return { z: 0, l: 0, Hmax: 0, H: 0, Rc: 0, Fy: 0, A: 0 };
  const n = soil.n;
  const kEq = soil.kc + b * soil.kphi;
  // static sinkage, rigid equivalent wheel of diameter D
  let z = Math.pow((3 * W) / ((3 - n) * kEq * Math.sqrt(D)), 2 / (2 * n + 1));
  // slip-sinkage: mild below ~40 % slip, then the spinning wheel digs itself in (the excavation that embedded
  // Spirit and bogged the Apollo 15 LRV). Compaction resistance grows with z^(n+1), so a wheel left spinning at
  // high slip on a slope or in loose soil loses its drawbar pull: back off instead of powering through.
  const s = Math.min(1, Math.abs(slip)), dig = Math.max(0, s - 0.4);
  z *= 1 + 0.55 * s + 1.5 * dig * dig;
  z = Math.min(z, soil.zMax ?? 0.5);
  const l = Math.sqrt(Math.max(1e-6, D * z));             // contact patch length
  const A = b * l;
  const Hmax = A * soil.c + W * Math.tan(soil.phi);
  const Rc = b * (soil.kc / b + soil.kphi) * Math.pow(z, n + 1) / (n + 1);
  const H = Math.sign(slip) * Hmax * shearFactor(Math.abs(slip) * l / soil.K);
  const Fy = -Math.sign(tanAlpha) * 0.92 * Hmax * (1 - Math.exp(-Math.abs(tanAlpha) * l / (soil.K * 1.6)));
  return { z, l, Hmax, H, Rc, Fy, A };
}

// 1 - (K/(s l))(1 - e^{-s l / K}) with a safe small-argument limit
export function shearFactor(x) {
  if (x < 1e-4) return x * 0.5;
  return 1 - (1 - Math.exp(-x)) / x;
}

// ---- Plate (foot) helpers for the ACM-on-regolith contact (cargoContact.js; plan s5.5) ---------------------------
// Bekker pressure-sinkage for a square plate of side b: p = (kc/b + kphi) z^n.
// Sinkage under a uniform pressure p (Pa).
export function plateSinkage(p, b, soil = R) {
  if (!(p > 0)) return 0;
  return Math.pow(p / (soil.kc / b + soil.kphi), 1 / soil.n);
}
// Normal force (N) of a plate of area A and side b at sinkage z (m).
export function plateLoad(z, b, A, soil = R) {
  if (!(z > 0)) return 0;
  return A * (soil.kc / b + soil.kphi) * Math.pow(z, soil.n);
}
