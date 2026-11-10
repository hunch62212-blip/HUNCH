// Engineering quantities derived from the live vehicle state, for the HUD instruments and the 3D engineering
// overlay (docs/GRAPHICS_PHYSICS_RESEARCH.md lists the sources). Pure: reads the rover, changes nothing.
//
//   SSF  static stability factor      T_eff / (2 h)          (NHTSA rollover metric; h = CoM height over the
//                                                             support plane, T_eff = 2 x nearest side edge)
//   roll / pitch threshold            atan(SSF), atan(L_eff / 2h)   tilt at which the CoM passes the edge
//   lateral acceleration              a_y = v * r            (r = yaw rate)
//   rollover utilisation              (tan|roll| + |a_y| / g) / SSF   (crane-LMI style: 100 % = wheels lift)
//   force-angle margin                Papadopoulos & Rey (computed in Rover._stability)
//   stopping distance                 v t_r + v^2 / (2 a)    (a = autonomy braking limit, t_r = reaction time)
//   drawbar pull                      DP = sum(H) - sum(R_c) (Bekker-Wong thrust minus compaction resistance)
//   traction / resistance coefficients sum(H) / W_n, sum(R_c) / W_n   (W_n = sum of wheel normal loads)
//   slip (load weighted)              sum(|s| N) / sum(N)
//   mechanical drive power            sum(T * omega)
//   specific energy                   P_elec / v             (Wh/km)
//   cost of transport                 P_elec / (m g v)       (dimensionless; lunar g)
//   Froude number                     v^2 / (g L)            (dynamic similarity with the 1:16.3 prototype)
//   kinetic energy                    m v^2 / 2
//   grade force                       m g sin(pitch)
import * as THREE from 'three';
import { MOON, ROVER } from './config.js';

export const REACTION_S = 0.5;           // operator / autonomy reaction time for the stopping distance
export const brakeDecel = (loaded) => (loaded ? 0.2 : 0.6); // same limits as the autonomy speed governor

const _q = new THREE.Quaternion(), _f = new THREE.Vector3(), _r = new THREE.Vector3();

export function vehicleDynamics(R) {
  const g = MOON.g;
  const loaded = !!R.loaded;
  const m = R.mass + (loaded ? R.lift.cargo.T.mass : 0);
  const lv = R.body.linvel(), av = R.body.angvel(), rq = R.body.rotation();
  _q.set(rq.x, rq.y, rq.z, rq.w);
  _f.set(1, 0, 0).applyQuaternion(_q); _f.y = 0; _f.normalize();       // body forward, horizontal
  _r.set(-_f.z, 0, _f.x);                                               // body right (THREE: z right of x)
  const v = lv.x * _f.x + lv.z * _f.z;                                  // signed forward speed
  const yawRate = av.y;
  const aLat = v * yawRate;
  const roll = R.tilt?.roll || 0, pitch = R.tilt?.pitch || 0;

  // support polygon geometry about the CoM (world-horizontal projection, body axes)
  const c = R.comWorld, poly = R.tilt?.poly;
  let h = NaN, halfT = NaN, halfL = NaN;
  if (c && poly && poly.length >= 3) {
    let ySum = 0, latL = Infinity, latR = Infinity, lonF = Infinity, lonB = Infinity;
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      ySum += a.y;
      // distance from the CoM projection to this edge, classified by the edge's outward direction in body axes
      let ex = b.x - a.x, ez = b.z - a.z; const el = Math.hypot(ex, ez) || 1; ex /= el; ez /= el;
      let nx = ez, nz = -ex;                                            // one of the edge normals
      const mx = (a.x + b.x) / 2 - c.x, mz = (a.z + b.z) / 2 - c.z;
      if (nx * mx + nz * mz < 0) { nx = -nx; nz = -nz; }                // make it point outward
      const d = Math.abs((c.x - a.x) * nx + (c.z - a.z) * nz);
      const side = nx * _r.x + nz * _r.z, fwd = nx * _f.x + nz * _f.z;
      if (Math.abs(side) >= Math.abs(fwd)) { if (side > 0) latR = Math.min(latR, d); else latL = Math.min(latL, d); }
      else if (fwd > 0) lonF = Math.min(lonF, d); else lonB = Math.min(lonB, d);
    }
    h = Math.max(0.1, c.y - ySum / poly.length);
    halfT = Math.min(latL, latR); halfL = Math.min(lonF, lonB);
    if (!Number.isFinite(halfT)) halfT = NaN; if (!Number.isFinite(halfL)) halfL = NaN;
  }
  const SSF = halfT / h;
  const rollLimitDeg = Math.atan(SSF) * 180 / Math.PI;
  const pitchLimitDeg = Math.atan(halfL / h) * 180 / Math.PI;
  const aRoll = g * SSF;
  const rolloverUtil = Number.isFinite(SSF) ? (Math.tan(Math.abs(roll)) + Math.abs(aLat) / g) / SSF : NaN;
  const pitchUtil = Number.isFinite(halfL) ? Math.tan(Math.abs(pitch)) / (halfL / h) : NaN;

  const dec = brakeDecel(loaded);
  const stopDist = Math.abs(v) * REACTION_S + v * v / (2 * dec);

  let N = 0, H = 0, Rc = 0, slipN = 0, Pmech = 0, contacts = 0;
  for (const w of R.wheels) {
    if (!w.contact) continue;
    contacts++;
    N += w.load || 0; H += w.H || 0; Rc += w.Rc || 0; slipN += Math.abs(w.slip || 0) * (w.load || 0);
    Pmech += (w.torque || 0) * (w.omega || 0);
  }
  const Pelec = R.power?.driveW || 0;
  const moving = Math.abs(v) > 0.1;
  const L = 2 * Math.abs(ROVER.corners[0][0]);

  return {
    m, g, v, yawRate, aLat, roll, pitch, loaded,
    h, halfT, halfL, SSF, rollLimitDeg, pitchLimitDeg, aRoll, rolloverUtil, pitchUtil,
    marginDeg: R.tilt?.margin ?? NaN,
    decel: dec, stopDist,
    contacts, Nsum: N, drawbar: H - Rc, tractionCoef: N > 0 ? H / N : 0, resistCoef: N > 0 ? Rc / N : 0,
    slip: N > 0 ? slipN / N : 0,
    Pmech, Pelec, eta: Pelec > 1 && Pmech > 0 ? Pmech / Pelec : NaN,
    whPerKm: moving ? Pelec / Math.abs(v) / 3.6 : NaN,
    cot: moving ? Pelec / (m * g * Math.abs(v)) : NaN,
    froude: v * v / (g * L),
    ke: 0.5 * m * v * v,
    gradeF: m * g * Math.sin(pitch),
  };
}

// Constant-curvature motion prediction (what a racing game draws as the "line", and what an AR braking-distance
// HUD anchors to the road): integrate the current speed and yaw rate for `horizon` seconds.
export function predictPath(pose, v, yawRate, horizon = 8, n = 40) {
  const pts = [];
  let x = pose.x, z = pose.z, yaw = pose.yaw;
  const dt = horizon / n;
  for (let i = 0; i <= n; i++) {
    pts.push({ x, z, yaw, t: i * dt, s: Math.abs(v) * i * dt });
    x += Math.cos(yaw) * v * dt; z += -Math.sin(yaw) * v * dt; yaw += yawRate * dt;
  }
  return pts;
}
