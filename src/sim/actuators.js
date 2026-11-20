// Actuator models: DC gearmotor and hobby/industrial servo (docs/ARM_CARGO_BUILD_PLAN.md s5.1, s5.2, s6.2).
// Pure and deterministic: no Three / Rapier, no randomness, no globals. Used by arm.js (joints) and the drive loop.
//
// Units are SI, joint/output referred: torque N*m, speed rad/s, angle rad, voltage V, current A, power W, T in deg C.
// Sign convention: tau > 0 pushes the output in the +q direction; omega > 0 is +q motion.
//
// API summary
//   dz(x, h)                         dead zone: x-h (x>h) | x+h (x<-h) | 0
//   new Rail({Vsrc, Rsrc})           .bus(sumI) -> V_bus = Vsrc - Rsrc*sumI; .last
//   new DCMotor(spec, opts)          .step(u, omega, V_bus, dt) -> {tau, I, P_el, T_w, ...}
//                                    .eval(u, omega, V_bus) -> {tau, I} (no state change)
//   new Servo(spec, opts)            .step(thetaRef, q, qd, V_bus, dt, ffTorque=0) -> {tau, I, P_el, T_w, ...}
//   fromSpec(spec, opts)             Servo for specs with a gear stage (fs/backlash), DCMotor if gearLimitNm; opts.kind overrides
// spec fields (ACT in config.js): V, Ts, w0, Is, I0, Ilim, TsV, w0V, backlash, fs, Rth, Cth, Tmax, brake, gearLimitNm.
// Anything not listed in the spec is [ASSUMED] below and can be overridden through opts.

const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
const sign = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);

export function dz(x, h) { return x > h ? x - h : x < -h ? x + h : 0; }

// piecewise-linear table lookup, clamped at the ends. tab = [[x, y], ...] sorted by x.
function interp(tab, x) {
  if (x <= tab[0][0]) return tab[0][1];
  const n = tab.length;
  if (x >= tab[n - 1][0]) return tab[n - 1][1];
  for (let i = 1; i < n; i++) {
    if (x <= tab[i][0]) { const [x0, y0] = tab[i - 1], [x1, y1] = tab[i]; return y0 + (y1 - y0) * (x - x0) / (x1 - x0); }
  }
  return tab[n - 1][1];
}

// Shared supply rail: every actuator on the rail sees V_bus = V_src - R_src * (sum of |I|).
export class Rail {
  constructor({ Vsrc, Rsrc = 0 } = {}) { this.Vsrc = Vsrc; this.Rsrc = Rsrc; this.last = Vsrc; this.sumI = 0; }
  bus(sumI) { this.sumI = sumI; this.last = this.Vsrc - this.Rsrc * sumI; return this.last; }
}

const T_DERATE_SPAN = 30;   // [ASSUMED] K above Tmax over which current limit ramps linearly to zero

export class DCMotor {
  // opts: etaBack (back-driven efficiency, 1), regen (fraction of braking power returned, 0 = clipped), Tamb (20 C)
  constructor(spec, opts = {}) {
    this.spec = spec;
    const { V, Ts, w0, Is, I0 = 0 } = spec;
    this.Vnom = V; this.Ts = Ts; this.w0 = w0; this.Is = Is; this.I0 = I0;
    this.Ilim = spec.Ilim ?? Is;
    this.R = V / Is;                                   // winding + driver resistance
    this.k = (V - I0 * this.R) / w0;                   // output-referred back-EMF constant, V*s/rad
    this.eta = Ts / (this.k * (Is - I0));              // calibrated so stall matches the datasheet
    this.etaBack = opts.etaBack ?? spec.etaBack ?? 1;
    this.regen = opts.regen ?? 0;
    this.holdFactor = opts.holdFactor ?? 1.3;          // [ASSUMED] back-driven holding limit, x Ts
    this.gearLimitNm = spec.gearLimitNm ?? Infinity;
    this.thermal = spec.Rth != null && spec.Cth != null;
    this.Rth = spec.Rth; this.Cth = spec.Cth; this.Tmax = spec.Tmax ?? Infinity;
    this.Tamb = opts.Tamb ?? 20;
    this.T_w = this.Tamb;
    this.wEps = 0.005 * w0;                             // below this |omega| the Coulomb loss becomes stiction
    this.E_el = 0; this.E_mech = 0;                    // integrated energies (J), for sanity checks
    this.last = { tau: 0, I: 0, P_el: 0, T_w: this.T_w };
  }

  // Voltage-table scale factors relative to the linear V-proportional model (torque, speed).
  _vscale(Vbus) {
    const s = this.spec;
    let mT = 1, mW = 1;
    if (s.TsV) mT = interp(s.TsV, Vbus) / (this.Ts * Vbus / this.Vnom);
    if (s.w0V) mW = interp(s.w0V, Vbus) / (this.w0 * Vbus / this.Vnom);
    return [mT, mW];
  }
  derate() { return this.thermal ? clamp(1 - (this.T_w - this.Tmax) / T_DERATE_SPAN, 0, 1) : 1; }

  // Quasi-static torque/current at command u in [-1, 1], output speed omega, bus voltage Vbus.
  eval(u, omega, Vbus) {
    const [mT, mW] = this._vscale(Vbus);
    const ke = this.k / mW, kt = this.eta * this.k * mT;
    const Ilim = this.Ilim * this.derate();
    const I = clamp((u * Vbus - ke * omega) / this.R, -Ilim, Ilim);
    // Coulomb no-load loss, regularised: pure stiction at rest (friction balances I up to I0), full Coulomb above wEps
    const w = Math.min(Math.abs(omega) / this.wEps, 1);
    const Ifric = (1 - w) * clamp(I, -this.I0, this.I0) + w * this.I0 * sign(omega);
    let tau = kt * (I - Ifric);
    if (tau * omega < 0) {                              // back-driven
      tau /= this.etaBack;
      const cap = this.holdFactor * this.Ts * mT;
      if (Math.abs(tau) > cap) tau = sign(tau) * cap;
    }
    return { tau, I };
  }

  step(u, omega, Vbus, dt) {
    u = clamp(u, -1, 1);
    const { tau, I } = this.eval(u, omega, Vbus);
    return this._finish(tau, I, u * Vbus * I, omega, dt);
  }

  _finish(tau, I, Praw, omega, dt) {
    const P_el = Praw < 0 ? Praw * this.regen : Praw;    // regen clipped (prototype) or partially recovered
    if (this.thermal) {
      const dT = (I * I * this.R - (this.T_w - this.Tamb) / this.Rth) / this.Cth;
      this.T_w += dT * dt;
    }
    const P_mech = tau * omega;
    this.E_el += P_el * dt; this.E_mech += P_mech * dt;
    const gearOver = Math.abs(tau) > this.gearLimitNm;
    return (this.last = { tau, I, P_el, P_mech, T_w: this.T_w, gearOver, derate: this.derate() });
  }
}

// Servo = motor + gear stage (inertia Jg, backlash, compliant contact) + position loop (P/PID with deadband).
// Output pot measures q (output side). Control law (plan s5.2):
//   e = thetaRef - q (0 inside deadband); u = clamp(Kp*e + Ki*int(e) - Kd*omega_g + ffTorque/Ts, -1, 1)
//   Jg*domega_g/dt = tau_motor(u, omega_g) - tau_t;   tau_t = ks*dz(theta_g - q, b/2) + cs*(omega_g - qd)*[engaged]
export class Servo extends DCMotor {
  // opts: kp, ki, kd, deadband, thetaSat (rad, default 5 deg), tauM (motor mech. time constant, 0.03 s),
  //       Jg, zeta (contact damping, 0.7), rateMax/accMax (optional internal reference slew), brake, q0
  constructor(spec, opts = {}) {
    super(spec, opts);
    const thSat = opts.thetaSat ?? 5 * Math.PI / 180;
    this.kp = opts.kp ?? 1 / thSat;
    this.ki = opts.ki ?? 0.2; this.kd = opts.kd ?? 0.05;
    this.deadband = opts.deadband ?? 0.1 * Math.PI / 180;
    this.b = spec.backlash ?? 0;
    this.fs = spec.fs ?? 12;
    this.Jg = opts.Jg ?? spec.Jg ?? (opts.tauM ?? 0.03) * this.eta * this.k * this.k / this.R;   // [ASSUMED] tau_m
    this.ks = this.Jg * (2 * Math.PI * this.fs) ** 2;
    this.cs = 2 * (opts.zeta ?? 0.7) * Math.sqrt(this.ks * this.Jg);
    this.rateMax = opts.rateMax ?? Infinity; this.accMax = opts.accMax ?? Infinity;
    this.hasBrake = (opts.brake ?? spec.brake) === true;
    this.subDt = 1 / 960;
    this.reset(opts.q0 ?? 0);
  }

  reset(q = 0) {
    this.thG = q; this.wG = 0; this.integ = 0; this.ref = q; this.refV = 0;
    this.braked = false; this.still = 0; this.brakeSlip = false; this.T_w = this.Tamb;
    this.tau = 0;
  }

  // slew-limited reference (rate then acceleration), optional
  _slew(target, dt) {
    if (!Number.isFinite(this.rateMax) && !Number.isFinite(this.accMax)) { this.ref = target; return this.ref; }
    const err = target - this.ref;
    // velocity that can still be stopped on the target with accMax, capped by rateMax
    const vWant = clamp(Math.sqrt(2 * this.accMax * Math.abs(err)) * sign(err), -this.rateMax, this.rateMax);
    this.refV += clamp(vWant - this.refV, -this.accMax * dt, this.accMax * dt);
    if (Math.abs(err) <= Math.abs(this.refV) * dt) { this.ref = target; this.refV = 0; } else this.ref += this.refV * dt;
    return this.ref;
  }

  step(thetaRef, q, qd, Vbus, dt, ffTorque = 0) {
    const n = Math.max(1, Math.ceil(dt / this.subDt - 1e-9)), h = dt / n;
    let tauA = 0, IA = 0, PA = 0, Pm = 0, sat = false, u = 0;
    for (let i = 0; i < n; i++) {
      const r = this._sub(thetaRef, q + qd * h * i, qd, Vbus, h, ffTorque);
      tauA += r.tau; IA += r.I; PA += r.P_el; Pm += r.P_mech; sat = sat || r.sat; u = r.u;
    }
    this.tau = tauA / n;
    return (this.last = { tau: this.tau, I: IA / n, P_el: PA / n, P_mech: Pm / n, T_w: this.T_w, u, sat,
      braked: this.braked, thetaG: this.thG, err: this.ref - q, gearOver: Math.abs(this.tau) > this.gearLimitNm,
      derate: this.derate(), brakeSlip: this.brakeSlip });
  }

  _sub(thetaRef, q, qd, Vbus, h, ffTorque) {
    const ref = this._slew(thetaRef, h);
    let e = ref - q;
    const inDead = Math.abs(e) < this.deadband;
    if (inDead) e = 0;
    const [mT] = this._vscale(Vbus);
    const TsEff = this.Ts * mT;

    // power-off brake: engage after 0.5 s of standstill with no error, release on error or moving reference
    if (this.hasBrake) {
      const still = inDead && Math.abs(qd) < 1e-3 && Math.abs(this.refV) < 1e-9;
      this.still = still ? this.still + h : 0;
      if (!this.braked && this.still >= 0.5) { this.braked = true; this.wG = 0; this.integ = 0; }
      if (this.braked && !still) this.braked = false;
    }

    const x = this.thG - q, inGap = Math.abs(x) <= this.b / 2;
    let tauT = this.ks * dz(x, this.b / 2) + (inGap ? 0 : this.cs * (this.wG - qd));
    if (this.braked) {
      // gear stage locked by the brake; it carries the load up to holdFactor*Ts, then slips
      const cap = this.holdFactor * TsEff;
      this.brakeSlip = false;
      if (Math.abs(tauT) > cap) { this.brakeSlip = true; this.braked = false; this.still = 0; }
      else return { tau: tauT, I: 0, P_el: 0, P_mech: 0, sat: false, u: 0 };
    }

    // PID. Integrate only while unsaturated and outside the deadband (anti-windup)
    let uRaw = this.kp * e + this.ki * this.integ - this.kd * this.wG + ffTorque / TsEff;
    const u = clamp(uRaw, -1, 1), sat = u !== uRaw;
    if (!sat && !inDead) this.integ = clamp(this.integ + e * h, -1 / Math.max(this.ki, 1e-9), 1 / Math.max(this.ki, 1e-9));
    else if (inDead) this.integ *= 1 - Math.min(1, 2 * h);          // bleed off inside the deadband

    const m = this.eval(u, this.wG, Vbus);
    const I = m.I;
    // semi-implicit Euler on the gear stage
    this.wG += (m.tau - tauT) / this.Jg * h;
    this.thG += this.wG * h;
    if (this.thermal) this.T_w += ((I * I * this.R - (this.T_w - this.Tamb) / this.Rth) / this.Cth) * h;
    const Praw = u * Vbus * I, P_el = Praw < 0 ? Praw * this.regen : Praw;
    this.E_el += P_el * h; this.E_mech += m.tau * this.wG * h;
    return { tau: tauT, I, P_el, P_mech: m.tau * this.wG, sat, u };
  }
}

// Build the right actuator from a datasheet-form spec (e.g. ACT.elbow, ACT.proto.ds3218, ACT.proto.gm37d30).
export function fromSpec(spec, opts = {}) {
  const kind = opts.kind ?? (spec.gearLimitNm != null && spec.backlash == null ? 'motor' : 'servo');
  return kind === 'motor' ? new DCMotor(spec, opts) : new Servo(spec, opts);
}
