// Two mirrored 3-DOF arms (shoulder / elbow / wrist pitch) that handle the ACM (docs/ARM_CARGO_BUILD_PLAN.md s4, s5.3,
// s5.4, s5.6). Plain JS: no Three, no Rapier, deterministic, unit-testable headless (tests/arm.mjs).
//
// Frames: body frame, +X forward, +Y up, y = 0 is the ground at nominal ride. Joint angles are planar (x-y plane):
//   p_elbow = S + L1*[cos q1, sin q1];   p_tip = p_elbow + L2*[cos(q1+q2), sin(q1+q2)];   hook angle psi = q1+q2+q3
// psi = 0 means the V-seat gate faces up (the carry attitude); the stow cradle uses psi = sum(ARM.stow.q).
//
// API
//   fk(q) / ik(p, psi, {branch})                       static planar kinematics ('up' = elbow with the larger y)
//   gravityTorques(q, mTip, g | gBody)                 static: holding torque of joints 1, 2 for a tip mass
//   computeDeckPlaceX()                                balance rule: loaded CG -> x = 0 (about -0.734 m)
//   new ArmSystem(opts)
//     .poses                                           presets: stow, preGrasp, grasp, apex, prePlace, place
//     .moveTo(nameOrQ, {offsets, sync})  .stow() .deploy()
//     .engage(massKg, {x, y}) .latch(on) .deckLatch(on) .release()
//     .canLift(massKg, {marginF}) .liftMargin(massKg, q) .tipRatio({mass}) .cg() .massProps() .axleShare()
//     .step(dt, ctx)       ctx = { g, gBody:{x,y}, railK, marginF, groundN }
//     .telemetry()   .powerW  .stowed  .payload  .faults  .mode
// Servo coupling: every joint is driven through Servo.step(thetaRef, q, qd, V, h, ffTorque) at h = dt / n with n = ARM.substeps
// per 1/120 s. Not modelled (P3a): the ACM pendulum swing while carried (the payload is a rigid point mass at the trunnion
// here), the ZV input shaper, the reaction wrench on the rover body.
import { ARM, ACT, CARGO_TYPES, ROVER, DECK, MOON, K } from './config.js';
import { Servo, MechNode } from './actuators.js';
import { MECH, SINK } from './lunarParams.js';

const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
const wrap = (a) => { while (a > Math.PI) a -= 2 * Math.PI; while (a <= -Math.PI) a += 2 * Math.PI; return a; };

// ---- [ASSUMED] constants local to arm.js -------------------------------------------------------------------------
export const ARM_ASSUMED = {
  deckGuideMass: 150,    // [ASSUMED] plan s3.3 deck guides + latch, kg, sits at DECK.placeX
  wristInertia: 20,      // [ASSUMED] kg*m^2 about the wrist axis (wrist servo + hook + latch)
  latchJ: 2,             // [ASSUMED] kg*m^2 load inertia on a latch servo
  latchClosed: 0.6,      // [ASSUMED] rad of latch-gate travel from open (0) to closed
  latchRate: 1.8, latchAcc: 30,        // [ASSUMED] rad/s, rad/s^2: 0.6 rad in about 0.39 s (plan s6.4 step 7: gates close in 0.4 s)
  deckRate: 1.1, deckAcc: 40,          // [ASSUMED] deck latch: 0.6 rad in about 0.57 s (DECK.latchTime 0.6 s)
  latchTol: 0.03,        // [ASSUMED] rad, switch closed/open window
  etaPlan: 0.6,          // [ASSUMED] drivetrain efficiency used for the peakW planner (plan s4.3)
  asymFrac: 0.1,         // [ASSUMED] |F_L - F_R| / W above this flags an asymmetric load
  stallTime: 1.0,        // [ASSUMED] s at the torque limit, not moving, before a servo stall fault
  stallTorqueFrac: 0.9,  // [ASSUMED] of the available (current-limited) torque
  liftoffFrac: 0.98,     // [ASSUMED] hook share of the weight that counts as lifted off the ground
  poseAllow: { engaged: ['grasp', 'apex'], breakout: ['grasp', 'apex'], carried: ['apex', 'prePlace', 'place'] }, // [ASSUMED] named poses a manual command may start per payload state: no 'stow' through the deck/ground while holding the ACM
  standbyW: 3,           // [ASSUMED] W controller/sensor standby while the arms sit unpowered in their cradles
  cradleTime: 1.5,       // [ASSUMED] s stowed and settled before the servos are powered down
  atTol: 0.01,           // [ASSUMED] rad, "at target" window
};
const A = ARM_ASSUMED;
const LIM = ARM.limits;
const LIMV = [LIM.q1, LIM.q2, LIM.q3];
const S = ARM.shoulder, L1 = ARM.L1, L2 = ARM.L2, LK = ARM.links;
const ACM = CARGO_TYPES.acm;
const GUIDE_M = A.deckGuideMass;

// ---- kinematics ---------------------------------------------------------------------------------------------------
export function fk(q) {
  const c1 = Math.cos(q[0]), s1 = Math.sin(q[0]), c12 = Math.cos(q[0] + q[1]), s12 = Math.sin(q[0] + q[1]);
  const ex = S.x + L1 * c1, ey = S.y + L1 * s1;
  return { elbow: [ex, ey], tip: [ex + L2 * c12, ey + L2 * s12], psi: q[0] + q[1] + q[2] };
}

export function inLimits(q) { return q.every((v, i) => v >= LIMV[i][0] - 1e-9 && v <= LIMV[i][1] + 1e-9); }

// Two-link IK. branch 'up' picks the solution whose elbow is higher (plan s4.1), 'down' the other. ok = reachable and within limits.
export function ik(p, psi = 0, { branch = 'up', fitPsi = false } = {}) {
  const dx = p[0] - S.x, dy = p[1] - S.y, d2 = dx * dx + dy * dy, d = Math.sqrt(d2);
  const reasons = [];
  if (d > L1 + L2 + 1e-9) reasons.push('reach');
  const D = clamp((d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1, 1);
  const gam = Math.acos(D), th = Math.atan2(dy, dx), beta = Math.atan2(L2 * Math.sin(gam), L1 + L2 * D);
  const cands = [[th + beta, -gam], [th - beta, gam]].map(([a, b]) => [wrap(a), b]);
  const yEl = (c) => S.y + L1 * Math.sin(c[0]);
  let pick = cands[0], other = cands[1];
  if ((yEl(cands[1]) > yEl(cands[0])) === (branch === 'up')) { pick = cands[1]; other = cands[0]; }
  const q = [pick[0], pick[1], wrap(psi - pick[0] - pick[1])];
  // fitPsi: if the wrist limit blocks the wanted hook angle, take the nearest reachable angle (still inside the seat cone)
  if (fitPsi) q[2] = clamp(q[2], LIM.q3[0], LIM.q3[1]);
  if (!inLimits(q)) reasons.push('limits');
  return { q, ok: reasons.length === 0, reasons, alt: [other[0], other[1], wrap(psi - other[0] - other[1])] };
}

// ---- dynamics -----------------------------------------------------------------------------------------------------
// Joint 1, 2 equation of motion terms for tip mass mt (wrist + hook + payload share). Plan s5.3.
export function dynamics(q, qd, mt, gb) {
  const c2 = Math.cos(q[1]), s2 = Math.sin(q[1]);
  const { m1, lc1, I1, mElbow, m2, lc2, I2 } = LK;
  const M11 = I1 + I2 + m1 * lc1 * lc1 + mElbow * L1 * L1 + m2 * (L1 * L1 + lc2 * lc2 + 2 * L1 * lc2 * c2) + mt * (L1 * L1 + L2 * L2 + 2 * L1 * L2 * c2);
  const M12 = I2 + m2 * (lc2 * lc2 + L1 * lc2 * c2) + mt * (L2 * L2 + L1 * L2 * c2);
  const M22 = I2 + m2 * lc2 * lc2 + mt * L2 * L2;
  const h = -(m2 * L1 * lc2 + mt * L1 * L2) * s2;
  const C1 = h * (2 * qd[0] * qd[1] + qd[1] * qd[1]), C2 = -h * qd[0] * qd[0];
  const t1x = -Math.sin(q[0]), t1y = Math.cos(q[0]), t12x = -Math.sin(q[0] + q[1]), t12y = Math.cos(q[0] + q[1]);
  const g1 = gb.x * t1x + gb.y * t1y, g12 = gb.x * t12x + gb.y * t12y;
  const G1 = -((m1 * lc1 + mElbow * L1 + m2 * L1 + mt * L1) * g1 + (m2 * lc2 + mt * L2) * g12);
  const G2 = -((m2 * lc2 + mt * L2) * g12);
  return { M11, M12, M22, C1, C2, G1, G2 };
}

const gVec = (g) => (typeof g === 'object' ? g : { x: 0, y: -g });
// Holding torque [joint 1, joint 2] (N*m) at pose q for tip mass mTip (kg) under gravity g (m/s^2) or body-frame vector.
export function gravityTorques(q, mTip, g = MOON.g) { const r = dynamics(q, [0, 0], mTip, gVec(g)); return [r.G1, r.G2]; }

// Combined mass points of one arm's moving links at pose q (x, y), plus the tip mass.
function linkPoints(q) {
  const c1 = Math.cos(q[0]), s1 = Math.sin(q[0]), c12 = Math.cos(q[0] + q[1]), s12 = Math.sin(q[0] + q[1]);
  const ex = S.x + L1 * c1, ey = S.y + L1 * s1;
  return [
    { m: LK.m1, x: S.x + LK.lc1 * c1, y: S.y + LK.lc1 * s1 },
    { m: LK.mElbow, x: ex, y: ey },
    { m: LK.m2, x: ex + LK.lc2 * c12, y: ey + LK.lc2 * s12 },
    { m: LK.mWrist + LK.mHook, x: ex + L2 * c12, y: ey + L2 * s12 },
  ];
}
const ARM_MOVING_MASS = LK.m1 + LK.mElbow + LK.m2 + LK.mWrist + LK.mHook;

// ROVER mass without the arms (kg) and the arm-system rover-frame masses that never move.
const ROVER_MASS = ROVER.sprungMass + ROVER.corners.length * ROVER.unsprungMassPerBogie;
const FRONT_X = Math.max(...ROVER.corners.map((c) => c[0]));   // FL-FR contact line (plan s5.6)

// Plan s3.3 balance rule: loaded CG -> x = 0 gives DECK.placeX. Sum of moments of everything but the deck load.
export function computeDeckPlaceX(acmMass = ACM.mass, qStow = ARM.stow.q) {
  let mom = ROVER_MASS * ROVER.comLocal.x + ARM.arms * ARM.fixedMassPerArm * S.x;
  for (const p of linkPoints(qStow)) mom += ARM.arms * p.m * p.x;     // moving links at their FK positions in the stow pose
  return -mom / (GUIDE_M + acmMass);
}

// Rover-frame points the arm geometry cares about for the ACM on the ground / on the deck.
const TRUN_Y = ACM.trunnion.y;                 // trunnion height above the ACM bottom
const CG_DROP = ACM.trunnion.y - ACM.cgY;      // CG hangs this far below the trunnion

function buildPoses(placeX) {
  const deckY = ROVER.frame.deckY, P = ARM.pick;
  const defs = {
    stow:     { p: null, psi: ARM.stow.q[0] + ARM.stow.q[1] + ARM.stow.q[2] },
    preGrasp: { p: [P.xBody - P.preFwd, TRUN_Y - P.preBelow], psi: 0 },
    grasp:    { p: [P.xBody, TRUN_Y], psi: 0 },
    apex:     { p: [S.x, S.y + ARM.apex.dy], psi: 0 },
    prePlace: { p: [placeX, deckY + TRUN_Y + ARM.prePlaceH], psi: 0 },
    place:    { p: [placeX, deckY + TRUN_Y], psi: 0 },
  };
  const out = {};
  for (const [name, d] of Object.entries(defs)) {
    if (name === 'stow') { const q = ARM.stow.q.slice(); out[name] = { q, p: fk(q).tip, psi: d.psi, ok: inLimits(q), reasons: [] }; continue; }
    const r = ik(d.p, d.psi, { branch: 'up', fitPsi: name === 'prePlace' || name === 'place' });
    out[name] = { q: r.q, p: d.p, psi: r.q[0] + r.q[1] + r.q[2], ok: r.ok, reasons: r.reasons };
  }
  return out;
}

// ---- the system ----------------------------------------------------------------------------------------------------
const SUBDT = 1 / 960;
const JOINT_SPECS = [ACT.shoulder, ACT.elbow, ACT.wrist];

class ArmSide {
  constructor(side, q0) {
    this.side = side;                                  // +1 starboard (+z), -1 port
    this.q = q0.slice(); this.qd = [0, 0, 0]; this.ref = q0.slice();
    const co = ARM.ctrl;
    // each joint / latch housing is a radiating thermal node (lunarParams MECH): it cold-soaks toward the lunar sink
    // instead of sitting in 20 C "air"; the winding sheds into it (actuators.js MechNode)
    this.mech = JOINT_SPECS.map(() => new MechNode(MECH.joint, MECH.lockC));
    this.latchMech = new MechNode(MECH.latch, MECH.lockC);
    this.servos = JOINT_SPECS.map((spec, i) => new Servo(spec, { q0: q0[i], kp: co.kp, ki: co.ki, kd: co.kd, deadband: co.deadband, mech: this.mech[i] }));
    this.latchServo = new Servo(ACT.latch, { q0: 0, rateMax: A.latchRate, accMax: A.latchAcc, brake: false, mech: this.latchMech });
    this.latchQ = 0; this.latchQd = 0; this.latchCmd = 0;
    this.tau = [0, 0, 0]; this.I = [0, 0, 0]; this.P = [0, 0, 0]; this.u = [0, 0, 0];
    this.stallT = [0, 0, 0]; this.stalled = [false, false, false]; this.atLimit = [false, false, false];
    this.latchI = 0; this.latchP = 0;
    this.hookF = 0; this.seated = false;
  }
  get latchClosed() { return Math.abs(this.latchQ - A.latchClosed) < A.latchTol; }
  get latchOpen() { return Math.abs(this.latchQ) < A.latchTol; }
}

export class ArmSystem {
  constructor(opts = {}) {
    this.placeX = opts.placeX ?? computeDeckPlaceX();
    this.poses = buildPoses(this.placeX);
    this.arms = [new ArmSide(-1, ARM.stow.q), new ArmSide(1, ARM.stow.q)];
    this.deckMech = new MechNode(MECH.latch, MECH.lockC);
    this.deck = { servo: new Servo(ACT.latch, { q0: 0, rateMax: A.deckRate, accMax: A.deckAcc, brake: false, mech: this.deckMech }), q: 0, qd: 0, cmd: 0, I: 0, P: 0 };
    // lunar thermal environment of the mechanisms, written each tick by health.js _thermal: { sinkK, solarWm2, heaterOK }.
    // null (bench tests): a night-time sky / regolith sink, no sun.
    this.thermalEnv = null;
    this.coldLock = false; this.mechHeaterW = 0;
    this.g = opts.g ?? MOON.g;
    this.plan = null;
    this.t = 0; this.timeScale = 1; this.hold = false;
    this.asleep = false;
    this.cradled = true;                               // stowed in the cradles, servos unpowered (only ARM_ASSUMED.standbyW)
    this._stillT = 0;
    this.payload = null;                               // { mass, restY, state, lf, ... }
    this.faults = new Set(); this.events = [];
    this.hookFh = ARM.hook.fh;
    this.powerW = ARM.hotelW; this.armW = 0; this.energyWh = 0;
    this.tip = { ratio: 0, ok: true, marginF: null };
    this.asym = 0; this.twist = 0; this.asymFlag = false;
    this.status = 'STOWED'; this.mode = 'STOWED';
    this.keepOut = null; this.strikes = 0; this._koIn = false;
    this.railK = 1;
    this._cgCache = null;
  }

  // wrappers matching the plan's signatures (arms are planar and mirrored, side only changes z)
  fk(side, q) { const r = fk(q); return { ...r, z: (side >= 0 ? 1 : -1) * S.z }; }
  ik(side, p, psi, o) { return ik(p, psi, o); }

  get stowed() {
    const qs = ARM.stow.q;
    return !this.plan && this.arms.every((a) => a.q.every((v, i) => Math.abs(v - qs[i]) < 0.02) && a.qd.every((v) => Math.abs(v) < 1e-3));
  }
  get done() { return !this.plan; }
  get latched() { return this.arms.every((a) => a.latchClosed); }
  get gatesOpen() { return this.arms.every((a) => a.latchOpen); }
  get deckLatched() { return Math.abs(this.deck.q - A.latchClosed) < A.latchTol; }
  get deckOpen() { return Math.abs(this.deck.q) < A.latchTol; }
  get lifted() { return !!this.payload && this.payload.state === 'carried'; }
  get carrying() { return !!this.payload; }

  // ---- commands -------------------------------------------------------------------------------------------------
  // target: preset name, [q1, q2, q3] for both arms, or [[...],[...]] per arm. offsets: optional per-arm q offset (fault injection).
  // Power the servos up from the cradle (no transient: servos start at the current pose).
  wake() {
    if (!this.cradled) return;
    this.cradled = false; this._stillT = 0;
    for (const a of this.arms) { a.servos.forEach((sv, j) => sv.reset(a.q[j])); a.ref = a.q.slice(); a.latchServo.reset(a.latchQ); }
    this.deck.servo.reset(this.deck.q);
  }
  moveTo(target, { offsets = null, sync = true } = {}) {
    this.wake();
    if (this.faults.has('stall') || this.faults.has('overtemp')) return { ok: false, reason: 'fault' };
    if (this.coldLock) { this.status = 'REFUSED cold lockout'; return { ok: false, reason: `cold lockout: ${this.coldestMech().name} at ${this.coldestMech().T.toFixed(0)} C (grease limit ${MECH.lockC} C)` }; }
    let qt;
    if (typeof target === 'string') {
      const allow = this.payload && A.poseAllow[this.payload.state];
      if (allow && !allow.includes(target)) { this.status = `REFUSED pose ${target} while ${this.payload.state}`; return { ok: false, reason: `pose '${target}' not allowed while the payload is ${this.payload.state}` }; }
      const ps = this.poses[target];
      if (!ps) return { ok: false, reason: 'unknown pose' };
      if (!ps.ok) return { ok: false, reason: ps.reasons.join(',') };
      qt = [ps.q.slice(), ps.q.slice()];
    } else qt = Array.isArray(target[0]) ? target.map((r) => r.slice()) : [target.slice(), target.slice()];
    if (offsets) qt = qt.map((r, i) => r.map((v, j) => v + (offsets[i]?.[j] ?? 0)));
    if (!qt.every(inLimits)) return { ok: false, reason: 'limits' };
    // pre-lift governor: do not start moving a grounded payload that would tip the rover (plan s5.6)
    if (this.payload && this.payload.state !== 'carried' && this.payload.latched !== false) {
      const c = this.canLift(this.payload.mass, { marginF: this._ctxMargin });
      if (!c.ok) { this.status = 'REFUSED ' + c.reasons.join(','); return { ok: false, reason: c.reasons.join(','), check: c }; }
    }
    // synchronised quintic: duration set by the slowest joint (peak rate 1.875 D/T, peak acc 5.77 D/T^2)
    let T = 0.5;
    for (let i = 0; i < 2; i++) for (let j = 0; j < 3; j++) {
      const D = Math.abs(qt[i][j] - this.arms[i].ref[j]);
      T = Math.max(T, 1.875 * D / ARM.rateMax, Math.sqrt(5.7735 * D / ARM.accMax));
    }
    this.plan = { q0: this.arms.map((a) => a.ref.slice()), q1: qt, T, t: 0, name: typeof target === 'string' ? target : 'q' };
    this.hold = false; this.status = 'MOVING ' + this.plan.name;
    return { ok: true, T };
  }
  stow() { return this.moveTo('stow'); }
  // Hook-trunnion interference (plan s6.4 / s5.4). The V-seat opens upward: a hook that rises past the centre of an un-seated
  // trunnion (strictly above it inside the hook mouth) would pass through it, and lifts it if it is under it. The keep-out is
  // the half-disc of radius mouth/2 (the V-seat half-width: the hook capture) above the trunnion centre, y > y_trun + ARM.keepOutMargin; the seat itself
  // (tip at y_trun, the hook cradling it from below) is allowed. A tip passing high over the trunnion (stow swing) is not touching it.
  static keepOutHit(tip, trun) { return Math.hypot(tip[0] - trun.x, tip[1] - trun.y) < ARM.hook.mouth / 2 && tip[1] > trun.y + ARM.keepOutMargin; }
  // Sample the planned quintic (both arms are mirrored, one is enough) from the current reference to `target` against `trun`.
  pathHitsKeepOut(target, trun, n = 80) {
    let q1;
    if (typeof target === 'string') { const ps = this.poses[target]; if (!ps) return { hit: false }; q1 = ps.q; } else q1 = Array.isArray(target[0]) ? target[0] : target;
    const q0 = this.arms[0].ref;
    for (let i = 0; i <= n; i++) {
      const t = i / n, s = t * t * t * (10 - 15 * t + 6 * t * t), q = q0.map((v, j) => v + (q1[j] - v) * s);
      const tip = fk(q).tip;
      if (ArmSystem.keepOutHit(tip, trun)) return { hit: true, at: t, tip };
    }
    return { hit: false };
  }
  deploy() { return this.moveTo('preGrasp'); }

  // Hooks are at the ACM trunnion {x, y} (rover frame): seat each arm whose hook is within the hook mouth. Gates stay open.
  engage(massKg = ACM.mass, trunnion = null) {
    this.wake();
    const tr = trunnion ?? { x: this.poses.grasp.p[0], y: this.poses.grasp.p[1] };
    const seated = this.arms.map((a) => { const t = fk(a.q).tip; return Math.hypot(t[0] - tr.x, t[1] - tr.y) <= ARM.hook.mouth; });
    seated.forEach((s, i) => { this.arms[i].seated = s; });
    if (!seated.some(Boolean)) return { ok: false, reason: 'no seat' };
    this.payload = { mass: massKg, restY: tr.y, state: 'engaged', lf: 0, W: massKg * this.g };
    this.status = 'ENGAGED';
    return { ok: true, seated };
  }
  latch(on = true) { if (this.coldLock) return { ok: false, reason: 'cold lockout' }; this.wake(); for (const a of this.arms) a.latchCmd = on ? A.latchClosed : 0; }
  deckLatch(on = true) { if (this.coldLock) return { ok: false, reason: 'cold lockout' }; this.wake(); this.deck.cmd = on ? A.latchClosed : 0; }
  release() {
    this.wake();
    for (const a of this.arms) { a.latchCmd = 0; a.seated = false; a.hookF = 0; }
    this.payload = null; this.status = 'RELEASED';
  }

  // ---- lift checks ----------------------------------------------------------------------------------------------
  // Required joint torque and available (current-limited) torque for a payload of massKg shared by both arms at pose q.
  liftMargin(massKg = ACM.mass, q = this.poses.grasp.q, g = this.g) {
    const mt = LK.mWrist + LK.mHook + massKg / ARM.arms / 1;
    const gt = gravityTorques(q, mt, g);
    const V = [ACT.shoulder.V, ACT.elbow.V, ACT.wrist.V];
    const out = [];
    for (let j = 0; j < 2; j++) {
      const avail = this.arms[0].servos[j].eval(1, 0, V[j] * this.railK).tau;
      const req = Math.abs(gt[j]);
      out.push({ req, avail, stall: JOINT_SPECS[j].Ts, margin: avail / req, ratioStall: req / JOINT_SPECS[j].Ts });
    }
    return { joints: out, margin: Math.min(out[0].margin, out[1].margin), rated: massKg <= ARM.ratedKg };
  }

  // ASME B30.5-style tip ratio about the FL-FR contact line (plan s5.6): overturning / restoring moment of everything.
  // Uses the measured hook load once lifted, the manifest x 1.25 before. opts: mass override, q (both arms), tip {x, y}.
  tipRatio(opts = {}) {
    let over = 0, rest = 0;
    for (const p of this._massPoints(opts)) { const l = p.x - FRONT_X; if (l > 0) over += p.m * l; else rest += p.m * -l; }
    return rest > 0 ? over / rest : Infinity;
  }
  // mass list: rover, fixed arm parts, moving links, deck guides, payload at its CG
  _massPoints(opts = {}) {
    const pts = [{ m: ROVER_MASS, x: ROVER.comLocal.x, y: ROVER.comLocal.y }];
    for (const a of this.arms) {
      pts.push({ m: ARM.fixedMassPerArm, x: S.x, y: S.y });
      const q = opts.q ? (Array.isArray(opts.q[0]) ? opts.q[a.side > 0 ? 1 : 0] : opts.q) : a.q;
      for (const p of linkPoints(q)) pts.push(p);
    }
    const pl = this.payload;
    const mass = opts.mass ?? (pl ? (pl.state === 'carried' ? this._hookLoad() / this.g : pl.mass * 1.25) : 0);
    if (mass > 0) {
      const tip = opts.tip ?? this._trunnion();
      pts.push({ m: mass, x: tip.x, y: tip.y - CG_DROP });
    }
    const gm = opts.guide ?? GUIDE_M;
    pts.push({ m: gm, x: this.placeX, y: ROVER.frame.deckY });
    return pts;
  }
  _trunnion() {
    const t0 = fk(this.arms[0].q).tip, t1 = fk(this.arms[1].q).tip;
    return { x: (t0[0] + t1[0]) / 2, y: (t0[1] + t1[1]) / 2 };
  }
  _hookLoad() { return this.arms[0].hookF + this.arms[1].hookF; }

  // Pre-lift governor: tip ratio (manifest x 1.25 at the current trunnion), marginF, rated capacity, torque margin.
  canLift(massKg = ACM.mass, { marginF = null, tip = null, q = null } = {}) {
    const reasons = [];
    const ratio = this.tipRatio({ mass: massKg * 1.25, tip: tip ?? this._trunnion(), q: q ?? undefined });
    if (ratio > ARM.tipRatioMax) reasons.push('tip');
    if (marginF != null && marginF < ARM.minMarginDeg) reasons.push('margin');
    if (massKg > ARM.ratedKg) reasons.push('rated');
    const lm = this.liftMargin(massKg, this.arms[0].q);
    if (lm.margin < 1) reasons.push('torque');
    return { ok: reasons.length === 0, reasons, tipRatio: ratio, marginF, torqueMargin: lm.margin };
  }

  // Combined centre of mass of rover + arm parts + payload + deck guides, rover frame.
  // opts: { mass, tip: {x, y} } puts a payload of that mass with its trunnion at tip (e.g. the ACM seated on the deck)
  cg(opts = {}) {
    const pts = this._massPoints({ mass: opts.mass ?? (this.payload ? this.payload.mass : 0), tip: opts.tip });
    let m = 0, x = 0, y = 0;
    for (const p of pts) { m += p.m; x += p.m * p.x; y += p.m * p.y; }
    return { m, x: x / m, y: y / m };
  }
  // Moving links + the suspended payload at their CoM (rover frame), for rover._stability
  massProps() {
    let m = 0, x = 0, y = 0;
    for (const a of this.arms) for (const p of linkPoints(a.q)) { m += p.m; x += p.m * p.x; y += p.m * p.y; }
    if (this.payload) { const t = this._trunnion(), pm = this.payload.mass; m += pm; x += pm * t.x; y += pm * (t.y - CG_DROP); }
    return { m, x: x / m, y: y / m, cgBelow: CG_DROP };
  }
  // Front-axle share of the weight from the combined CG: share = (x_cg - x_rear)/(x_front - x_rear) about the bogie pivot rocker (plan s3.1)
  axleShare() {
    const c = this.cg(), xB = -ROVER.corners[0][0] * 0.5, xF = FRONT_X;
    return clamp((c.x - xB) / (xF - xB), 0, 1);
  }

  // ---- simulation ------------------------------------------------------------------------------------------------
  // every mechanism thermal node with a label, for telemetry and the lockout message
  mechNodes() {
    const out = [];
    for (const a of this.arms) { const s = a.side > 0 ? 'R' : 'L'; ['shoulder', 'elbow', 'wrist'].forEach((n, j) => out.push({ name: `${s} ${n}`, node: a.mech[j] })); out.push({ name: `${s} hook latch`, node: a.latchMech }); }
    out.push({ name: 'deck latch', node: this.deckMech });
    return out;
  }
  coldestMech() { let c = null; for (const m of this.mechNodes()) if (!c || m.node.T < c.T) c = { name: m.name, T: m.node.T }; return c; }
  // radiate / heat every housing (also while cradled: a stowed arm cold-soaks too); motion lockout below MECH.lockC
  _mechThermal(dt, ctx) {
    const env = this.thermalEnv || { sinkK: (SINK.viewSky * SINK.regNightK ** 4) ** 0.25, solarWm2: 0 };
    const heaterOK = env.heaterOK !== false; // survival heaters stay on the battery even when the drive is unpowered
    let w = 0, lock = false;
    for (const m of this.mechNodes()) { m.node.step(dt, { sinkK: env.sinkK, solarWm2: env.solarWm2, heaterOK }); w += m.node.heatW; lock = lock || m.node.locked; }
    this.mechHeaterW = w;
    if (lock && !this.coldLock) this.events.push('cold_lockout');
    this.coldLock = lock;
  }

  step(dt, ctx = {}) {
    this._mechThermal(dt, ctx);
    const g = ctx.g ?? this.g, gb = ctx.gBody ?? { x: 0, y: -g };
    const gm = Math.hypot(gb.x, gb.y) || g;
    this.g = g; this.railK = ctx.railK ?? 1; this._ctxMargin = ctx.marginF ?? null;
    const n = Math.max(1, Math.round(dt / SUBDT)), h = dt / n;
    const pl = this.payload;
    if (pl) pl.W = pl.mass * gm;

    // unpowered states: rover in survival (powered === false), or stowed in the cradles (they carry the weight, brakes hold)
    if (ctx.powered === false || this.cradled) {
      if (this.cradled && !this.arms.every((a) => a.q.every((v, i) => Math.abs(v - ARM.stow.q[i]) < 0.02))) this.wake();
      else { this.armW = 0; this.powerW = (ctx.powered === false ? 0 : A.standbyW) + this.mechHeaterW; this.energyWh += this.powerW * dt / 3600; this.t += dt; return this.telemetry(); }
    }

    // reference generation (quintic through the plan, time scaled by the power and governor limits)
    if (this.plan) {
      const P = this.plan;
      if (!this.hold && !this.coldLock) P.t += dt * this.timeScale; // grease too stiff: the plan freezes where it is
      const tau = clamp(P.t / P.T, 0, 1), s = tau * tau * tau * (10 - 15 * tau + 6 * tau * tau);
      for (let i = 0; i < 2; i++) for (let j = 0; j < 3; j++) this.arms[i].ref[j] = P.q0[i][j] + (P.q1[i][j] - P.q0[i][j]) * s;
      if (tau >= 1) { this.plan = null; this.status = 'AT ' + P.name; }
    }

    let Pel = 0, Pmech = 0;
    for (const a of this.arms) { a.P.fill(0); }
    const Wpl = pl ? pl.W : 0;
    for (let sub = 0; sub < n; sub++) {
      this._hookLoads(gm, ctx);
      for (let ai = 0; ai < 2; ai++) {
        const a = this.arms[ai];
        const share = pl ? a.hookF / gm : 0;                       // payload mass this arm carries (kg)
        const mt = LK.mWrist + LK.mHook + share;
        const d = dynamics(a.q, a.qd, mt, gb);
        const tau = [0, 0, 0];
        for (let j = 0; j < 3; j++) {
          const spec = JOINT_SPECS[j], V = spec.V * this.railK;
          const ff = ARM.ctrl.gravityFF ? (j === 0 ? d.G1 : j === 1 ? d.G2 : 0) : 0;
          const r = a.servos[j].step(a.ref[j], a.q[j], a.qd[j], V, h, ff);
          tau[j] = r.tau; a.I[j] = r.I; a.u[j] = r.u; a.P[j] += r.P_el / n;
          Pmech += Math.abs(r.P_mech);
          a.tau[j] = r.tau;
          // stall = saturated at the available torque, not moving
          const avail = a.servos[j].eval(1, 0, V).tau;
          const stalledNow = r.sat && Math.abs(a.qd[j]) < 0.01 && Math.abs(r.tau) >= A.stallTorqueFrac * avail && !r.braked;
          a.stallT[j] = stalledNow ? a.stallT[j] + h : Math.max(0, a.stallT[j] - 2 * h);
          a.stalled[j] = a.stallT[j] >= A.stallTime;
        }
        // trunnion friction on the wrist: mu*|F|*r_t opposes wrist rotation (damps the pendulum, plan s5.4)
        const fric = ARM.hook.mu * a.hookF * ACM.trunnion.r * Math.tanh(a.qd[2] / 0.01);
        // 2-link equation of motion, semi-implicit Euler
        const r1 = tau[0] - d.C1 - d.G1, r2 = tau[1] - d.C2 - d.G2;
        const det = d.M11 * d.M22 - d.M12 * d.M12;
        const a1 = (d.M22 * r1 - d.M12 * r2) / det, a2 = (d.M11 * r2 - d.M12 * r1) / det;
        const a3 = (tau[2] - fric) / A.wristInertia;
        const acc = [a1, a2, a3];
        for (let j = 0; j < 3; j++) {
          a.qd[j] += acc[j] * h; a.q[j] += a.qd[j] * h;
          const [lo, hi] = LIMV[j];
          if (a.q[j] < lo) { a.q[j] = lo; if (a.qd[j] < 0) a.qd[j] = 0; a.atLimit[j] = true; }
          else if (a.q[j] > hi) { a.q[j] = hi; if (a.qd[j] > 0) a.qd[j] = 0; a.atLimit[j] = true; }
          else a.atLimit[j] = false;
        }
        // latch gate
        const lr = a.latchServo.step(a.latchCmd, a.latchQ, a.latchQd, ACT.latch.V * this.railK, h, 0);
        const lacc = (lr.tau - 0) / A.latchJ;
        a.latchQd += lacc * h; a.latchQ += a.latchQd * h;
        if (a.latchQ < 0) { a.latchQ = 0; a.latchQd = Math.max(0, a.latchQd); }
        if (a.latchQ > A.latchClosed + 0.2) { a.latchQ = A.latchClosed + 0.2; a.latchQd = Math.min(0, a.latchQd); }
        a.latchI = lr.I; a.latchP = lr.P_el;
      }
      const dr = this.deck.servo.step(this.deck.cmd, this.deck.q, this.deck.qd, ACT.latch.V * this.railK, h, 0);
      this.deck.qd += dr.tau / A.latchJ * h; this.deck.q += this.deck.qd * h;
      if (this.deck.q < 0) { this.deck.q = 0; this.deck.qd = Math.max(0, this.deck.qd); }
      this.deck.I = dr.I; this.deck.P = dr.P_el;
    }
    // post-substep bookkeeping
    this._hookLoads(gm, ctx);
    for (const a of this.arms) { for (let j = 0; j < 3; j++) Pel += a.P[j]; Pel += a.latchP; }
    Pel += this.deck.P;
    this.armW = Pel;
    this.powerW = Pel + (this.asleep ? 0 : ARM.hotelW) + this.mechHeaterW;
    this.energyWh += this.powerW * dt / 3600;

    // power-limited planner: slow the profile when the mechanical estimate exceeds peakW (plan s4.3)
    const pMechEst = Pmech / n / A.etaPlan;
    const want = pMechEst > ARM.peakW ? clamp(ARM.peakW / pMechEst, 0.1, 1) : 1;
    this.timeScale += (want - this.timeScale) * Math.min(1, dt / 0.5);

    this._checks(dt, ctx, gm);
    // hook-trunnion interference watch (autonomy sets keepOut = the un-seated trunnion {x, y} while the hooks are not engaged)
    if (this.keepOut && !this.arms.some((a) => a.seated)) {
      const bad = this.arms.some((a) => ArmSystem.keepOutHit(fk(a.q).tip, this.keepOut));
      if (bad && !this._koIn) { this.strikes++; this.events.push('hook_trunnion'); }
      this._koIn = bad;
    } else this._koIn = false;
    // power down into the cradles once stowed, settled, gates and deck latch at rest, and any payload is held by the deck latch
    const latchesRest = this.arms.every((a) => Math.abs(a.latchQ - a.latchCmd) < A.latchTol && Math.abs(a.latchQd) < 1e-3) && Math.abs(this.deck.q - this.deck.cmd) < A.latchTol;
    if (this.stowed && latchesRest && (!this.payload || this.deckLatched)) this._stillT += dt; else this._stillT = 0;
    if (this._stillT >= A.cradleTime) {
      this.cradled = true;
      for (const a of this.arms) { a.q = ARM.stow.q.slice(); a.qd = [0, 0, 0]; a.ref = ARM.stow.q.slice(); a.hookF = 0; a.tau.fill(0); a.I.fill(0); a.P.fill(0); a.latchI = 0; a.latchP = 0; }
      this.armW = 0; this.powerW = A.standbyW + this.mechHeaterW;
    }
    this.t += dt;
    return this.telemetry();
  }

  // load share between the two hooks: spring-damper ends of the trunnion (plan s5.4), lift-off detection
  _hookLoads(gm, ctx) {
    const pl = this.payload;
    if (!pl) { for (const a of this.arms) a.hookF = 0; return; }
    const W = pl.W, tr = this._trunnion(), y = this.arms.map((a) => fk(a.q).tip[1]);
    // fraction of the weight the hooks carry: measured from the foot soil load when given, else from the lift height
    // groundN < 0 is soil suction (breakout adhesion) holding the feet down: the hooks then carry MORE than the weight (lf up to 2)
    let lf = ctx.groundN != null ? 1 - clamp(ctx.groundN / W, -1, 1) : clamp((tr.y - pl.restY) / ARM.pick.clearance, 0, 1);
    if (pl.state === 'carried' && ctx.groundN == null) lf = 1;
    const wasGround = pl.state !== 'carried';
    pl.lf = lf;
    const carry = this.arms.map((a) => a.seated && (a.latchClosed || this._seatOk(a)));
    const k_h = pl.mass / 2 * (2 * Math.PI * this.hookFh) ** 2;
    const F = [0, 0], Fl = W * lf;
    const nC = carry.filter(Boolean).length;
    if (nC === 2) {
      const ym = (y[0] + y[1]) / 2;
      F[0] = clamp(Fl / 2 + k_h * (y[0] - ym), 0, Fl); F[1] = Fl - F[0];
    } else if (nC === 1) { F[carry[0] ? 0 : 1] = Fl; }
    this.arms[0].hookF = F[0]; this.arms[1].hookF = F[1];
    const dF = F[1] - F[0];
    this.asym = Fl > 0 ? Math.abs(dF) / Fl : 0;
    this.asymFlag = Fl > 0 && (this.asym > A.asymFrac || nC === 1);
    this.twist = Math.abs(dF) * ACM.dims[2] / 2;     // N*m about the ACM x axis, hook lever = half the ACM span in z
    // lift-off from the ground
    if (wasGround && lf >= A.liftoffFrac && (tr.y - pl.restY) >= ARM.pick.clearance - 1e-6) { pl.state = 'carried'; this.events.push('liftoff'); }
    else if (pl.state === 'carried' && ctx.groundN != null && lf < 0.5) pl.state = 'engaged';
    else if (!wasGround && lf >= A.liftoffFrac) pl.state = 'carried';
    else if (pl.state !== 'carried') pl.state = lf > 0.02 ? 'breakout' : 'engaged';
  }
  _seatOk(a) { return Math.abs(wrap(a.q[0] + a.q[1] + a.q[2])) <= ARM.hook.seatHalf + Math.atan(ARM.hook.mu); }

  // faults and governors: drop, stall, over-temperature, tip, twist
  _checks(dt, ctx, gm) {
    const pl = this.payload;
    if (pl && pl.state === 'carried') {
      for (const a of this.arms) {
        if (!a.seated || a.hookF <= 0) continue;
        const ang = Math.abs(wrap(a.q[0] + a.q[1] + a.q[2]));
        const cone = ARM.hook.seatHalf + Math.atan(ARM.hook.mu);
        const outward = a.hookF * Math.sin(Math.min(ang, Math.PI / 2));
        if ((!a.latchClosed && ang > cone) || (a.latchClosed && outward > ARM.hook.latchMaxN)) {
          this.faults.add('drop'); this.events.push('drop'); this.payload = null; for (const b of this.arms) { b.hookF = 0; b.seated = false; }
          break;
        }
      }
    }
    if (this.arms.some((a) => a.stalled.some(Boolean))) this.faults.add('stall');
    if (this.arms.some((a) => a.servos.some((s, j) => s.thermal && s.T_w > JOINT_SPECS[j].Tmax))) this.faults.add('overtemp');
    if (this.twist > ACM.twistMax) this.faults.add('twist');
    // tip governor while moving with the payload suspended: stop and flag (set-down is the autonomy's job)
    const ratio = this.tipRatio();
    this.tip = { ratio, ok: ratio <= ARM.tipRatioMax && (ctx.marginF == null || ctx.marginF >= ARM.minMarginDeg), marginF: ctx.marginF ?? null };
    if (this.payload && this.payload.state === 'carried' && !this.tip.ok) { this.faults.add('tip'); this.hold = true; }
    const motors = this.arms.flatMap((a) => a.qd.map(Math.abs));
    this.mode = this.faults.size ? 'FAULT' : this.plan ? (this.hold ? 'HOLD' : 'MOVING') : this.stowed ? 'STOWED' : (this.payload ? 'CARRY' : 'IDLE');
    this._moving = Math.max(...motors) > 1e-3;
  }

  clearFaults() { this.faults.clear(); this.hold = false; for (const a of this.arms) { a.stallT.fill(0); a.stalled.fill(false); } }

  telemetry() {
    const deg = (v) => v * 180 / Math.PI;
    const pct = (a) => a.tau.map((t, j) => Math.abs(t) / JOINT_SPECS[j].Ts);
    const pl = this.payload;
    return {
      mode: this.cradled ? (this.faults.size ? 'FAULT' : 'STOWED') : this.mode, cradled: this.cradled, status: this.status, stowed: this.stowed, plan: this.plan ? this.plan.name : null, timeScale: this.timeScale,
      q: this.arms.map((a) => a.q.slice()), qDeg: this.arms.map((a) => a.q.map(deg)), qd: this.arms.map((a) => a.qd.slice()),
      tau: this.arms.map((a) => a.tau.slice()), pctStall: this.arms.map(pct),
      currentA: this.arms.map((a) => a.I.slice()), tempC: this.arms.map((a) => a.servos.map((s) => s.T_w)),
      mechC: this.arms.map((a) => a.mech.map((m) => m.T)), latchMechC: this.arms.map((a) => a.latchMech.T), deckMechC: this.deckMech.T, coldLock: this.coldLock, mechHeaterW: this.mechHeaterW,
      braked: this.arms.map((a) => a.servos.map((s) => s.braked)), stalled: this.arms.map((a) => a.stalled.slice()),
      latch: this.arms.map((a) => (a.latchClosed ? 'closed' : a.latchOpen ? 'open' : 'moving')), deckLatch: this.deckLatched ? 'closed' : this.deckOpen ? 'open' : 'moving',
      payload: pl ? { mass: pl.mass, state: pl.state, loadFrac: pl.lf } : null,
      hookLoad: this.arms.map((a) => a.hookF), asym: this.asym, asymFlag: this.asymFlag, twistNm: this.twist,
      tipRatio: this.tip.ratio, tipOk: this.tip.ok, axleShare: this.axleShare(),
      armW: this.armW, powerW: this.powerW, energyWh: this.energyWh, faults: [...this.faults],
    };
  }
}
