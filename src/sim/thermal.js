// AEGIS-V1 thermal environment, instrument / motor-controller thermal nodes, random thermal scenarios and the
// thermal "brain" (a utility-AI controller that decides how to keep every part of the rover in its temperature range).
//
// Physics (lumped-capacitance nodes, explicit Euler at 10 Hz):
//  * Regolith surface temperature follows radiative equilibrium with the sun:
//      T_reg = ((1 - A) S sin(i) / (eps sigma))^(1/4),  floor T_night;  first-order lag tau_reg
//    A 0.12, eps 0.95, S 1361 W/m^2, i = sun elevation (+ terrain slope during a "sun-facing slope" scenario).
//    South-pole site at a 9 deg sun: ~242 K. Sun-facing 30 deg slope: ~344 K (71 C). Night: ~95 K. PSR: ~60 K.
//  * A surface that sees half sky (3 K) and half regolith radiates to an effective sink T_sink^4 = F T_reg^4,
//    F = 0.5. health.js uses it for the vault, battery and wheel-hub radiators; the nodes below use it directly.
//  * Node: C dT/dt = P_internal + alpha A_p S (sunlit) + eps A F sigma T_reg^4 - eps A sigma T^4 + G (T_mount - T)
//          + heater - TEC heat pumped out
//    TEC (thermo-electric cooler) pumps Q_c <= Q_max for electrical input Q_c / COP; the mount takes Q_c + input.
//
// Brain (src/sim/thermal.js ThermalBrain): every second it predicts each temperature 2 min ahead from its filtered
// rate of change (spike detection), turns those into hot / cold urgencies (0..1) and scores the options of each
// actuator channel with response curves (utility AI, the technique game AIs use to pick the best action). The
// winning option is applied with a short commitment time so it does not chatter:
//   drive:       nominal | REV DOWN 1 (150 rpm, 85 % torque) | REV DOWN 2 (120 rpm, 70 %) | COOL-DOWN STOP |
//                REV UP (stator heating 120 W per motor, rpm cap released) | REV UP MAX (350 W per motor)
//   radiators:   thermostat | pre-cool (louvers forced open ahead of a heat spike) | conserve (closed; heater set points +3 K while driving) |
//                shed load (avionics low-power, -25 % vault heat)
//   instruments: per LiDAR / camera: nominal | heat | TEC cool | TEC cool + sunshade | protect (power off)
// All [ASSUMED] numbers are sized so the brain can hold every instrument inside its operating range through the
// hottest and coldest scenarios; with the brain off, the LiDAR overheats on a sun-facing slope (tests/thermal.mjs).
import { THERMAL } from './config.js';
import { mulberry32 } from './terrain.js';

const SIGMA = 5.670e-8, S0 = 1361, K0 = 273.15, DEG = Math.PI / 180;
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const smooth = (x, a, b) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
const bell = (x, c, w) => Math.exp(-(((x - c) / w) ** 2));

export const ENV = {
  albedo: 0.12, emisReg: 0.95, nightK: 95, psrK: 60, eclipseK: 120, tauRegS: 240, viewF: 0.5,
};

// [ASSUMED] instrument and controller nodes.
//   C J/K of the part in the loop (LiDAR: 0.9 kg optics / electronics block; camera 0.35 kg; controllers: MOSFET
//   cold plates ~13 kg Al), so they respond in ~10-20 min, like the real parts;  Pint W electronics; alphaA m^2 solar absorptance x projected area; epsA m^2 IR emittance x area;
//   G W/K conductance to the mount (deck ~ vault temperature); op / surv C operating / survival range;
//   band C the brain's target band; tecMaxW W heat a TEC can pump; heaterW W film heater.
export const NODES = {
  lidar: { name: 'LiDAR head', C: 800, Pint: 22, alphaA: 0.010, epsA: 0.1275, G: 0.5, op: [-20, 50], surv: [-40, 70], band: [0, 35], tecMaxW: 28, cop: 0.6, heaterW: 15 },
  camera: { name: 'Nav / hazard camera', C: 300, Pint: 6, alphaA: 0.004, epsA: 0.0425, G: 0.25, op: [-30, 60], surv: [-50, 80], band: [-5, 40], tecMaxW: 10, cop: 0.6, heaterW: 6 },
  // motor controllers (bays MC-L / MC-R): MOSFET + gate-drive losses, own tub-wall radiator, strap to the vault
  mc: { name: 'Motor controllers', C: 6000, Pint: 40, lossFrac: 0.03, alphaA: 0, epsA: 0.51, G: 2.0, op: [-40, 70], surv: [-55, 85], band: [-20, 55] },
};
// motor winding / wheel hub limits used by the brain (winding: rover.js w.temp, hub: health.js th.hub)
export const DRIVE_LIMITS = { windWarnC: 90, windMaxC: 125, hubColdWarnC: -30, hubColdMaxC: -60, windColdWarnC: -30, windColdMaxC: -60 };

// Preplanned scenarios. dur [min, max] s of sim time; need: 'sun' = only while the sun is up.
export const SCENARIOS = {
  'slope-heat': { name: 'Sun-facing slope heat soak', need: 'sun', dur: [360, 720], text: 'the route crosses a 30 deg sun-facing slope: regolith heads for ~70 C' },
  'psr-cold': { name: 'Crater shadow cold snap', dur: [240, 480], text: 'into a permanently shadowed crater floor: no sun, regolith ~60 K' },
  'eclipse': { name: 'Earth eclipses the Sun', need: 'sun', dur: [300, 600], text: 'the Earth covers the Sun: regolith cools toward ~120 K (Apollo-measured)' },
  'dust-fins': { name: 'Regolith dust on the hub radiators', dur: [300, 600], text: 'wheel-end radiator emissivity down 60 % until the EDS clears it' },
  'climb': { name: 'Long heavy climb', dur: [240, 420], text: 'motors near stall torque: +600 W copper loss per motor, controllers +3 % losses' },
  'glint': { name: 'Sun glint into the LiDAR / camera', need: 'sun', dur: [180, 300], text: 'sunlight focused through the sensor windows: LiDAR +35 W, camera +8 W' },
  'heater-stuck': { name: 'Vault heater relay stuck on', dur: [180, 360], text: '+600 W into the avionics vault until the relay is power-cycled' },
  'mc-overload': { name: 'Motor-controller switching losses up', dur: [240, 480], text: 'gate-drive fault: +400 W in the controller bays (scales with the current allowed)' },
  'cold-start': { name: 'Cold-soaked wheel ends', dur: [1, 1], text: 'after a long stop in shadow (freeze heaters at their limit) the hubs are at -68 C, the windings -72 C' },
};

export class Thermal {
  constructor(sim, opts = {}) {
    this.sim = sim;
    this.rate = 10;
    this.env = { sunEl: 0, sunlit: true, regK: 240, sinkK: 200, incBoost: 0, shadow: false, blocked: false, floorK: ENV.nightK };
    this.n = {};
    for (const k of Object.keys(NODES)) this.n[k] = { T: 15, rate: 0, heaterW: 0, tecW: 0, tecQ: 0, shade: false, extraW: 0, inW: 0 };
    this.ok = { lidar: true, camera: true };
    this.powerW = 0;           // electrical power the thermal actions draw (heaters, TECs, stator heating)
    this.fx = { hubEmisK: 1, vaultExtraW: 0, mcExtraW: 0, motorExtraW: 0, glint: 0 };
    this.cmd = { rpm: Infinity, torqueK: 1, statorW: 0, louver: null, setBias: 0, vaultLoadK: 1, hold: false };
    this.brain = new ThermalBrain(this);
    const rnd = mulberry32(((sim.terrain?.seed ?? 1) ^ 0x7e3a11) >>> 0);
    this.scen = { on: opts.scenarios ?? false, meanGapS: opts.meanGapS ?? 200, minGapS: 60, rnd, next: 0, active: null, history: [] };
    this.scen.next = (opts.firstS ?? 75);
    this.stats = { outOfRangeS: { lidar: 0, camera: 0, mc: 0, wind: 0 }, max: {}, min: {}, scenarios: 0 };
  }

  log(msg, lvl = 'info') { this.sim.log(msg, lvl); }

  // ---------------- environment ----------------
  _environment(dt) {
    const sim = this.sim, R = sim.rover, sun = sim.sunDir, E = this.env;
    E.sunEl = Math.asin(clamp(sun.y, -1, 1)) / DEG;
    E.sunlit = sun.y > 0 && (R.power?.sunlit ?? 1) > 0.5 && !E.shadow && !E.blocked;
    let target = E.floorK;
    if (E.sunlit) {
      const inc = clamp((E.sunEl + E.incBoost) * DEG, 0, Math.PI / 2);
      target = Math.max(E.floorK, ((1 - ENV.albedo) * S0 * Math.sin(inc) / (ENV.emisReg * SIGMA)) ** 0.25);
    }
    E.regTargetK = target;
    E.regK += (target - E.regK) * Math.min(1, dt / ENV.tauRegS);
    E.sinkK = (ENV.viewF * E.regK ** 4) ** 0.25;
  }

  // ---------------- instrument + controller nodes ----------------
  _nodes(dt) {
    const sim = this.sim, R = sim.rover, E = this.env, H = sim.health;
    const mount = H?.th?.Tv ?? 20;
    const powered = !!R.powered;
    const regIR = ENV.viewF * SIGMA * E.regK ** 4;
    let elec = 0;
    for (const [k, N] of Object.entries(NODES)) {
      const s = this.n[k];
      let P = 0;
      // controller losses scale with the current the brain allows (REV DOWN cuts torque; a cool-down stop idles them)
      if (k === 'mc') P = (powered ? N.Pint : 5) + N.lossFrac * (1 + (this.fx.mcLossK || 0)) * Math.max(0, R.power?.driveW || 0) + this.fx.mcExtraW * (this.cmd.hold ? 0.3 : this.cmd.torqueK);
      else {
        const on = powered && this.ok[k] && (k === 'lidar' ? sim.sensors?.lidarOn : sim.sensors?.cameraOn) !== false;
        P = (on ? N.Pint : 0) + (k === 'lidar' ? 35 : 8) * this.fx.glint;
      }
      const solar = E.sunlit ? N.alphaA * S0 * (s.shade ? 0.2 : 1) : 0;
      const emis = k === 'mc' ? (this.fx.mcEmisK ?? 1) : 1;
      const rad = N.epsA * emis * SIGMA * (s.T + K0) ** 4;
      const absIR = N.epsA * emis * regIR;
      const cond = N.G * (mount - s.T);
      const q = P + solar + absIR - rad + cond + s.heaterW - s.tecQ;
      s.inW = P + solar + absIR;
      const T0 = s.T;
      s.T += dt * q / N.C;
      const r = (s.T - T0) / dt * 60;                 // K/min
      s.rate += (r - s.rate) * Math.min(1, dt / 20);  // filtered rate (spike detector)
      elec += s.heaterW + (s.tecQ > 0 ? s.tecQ / (N.cop || 0.6) : 0);
    }
    // stator heating ("rev up") and scenario motor heat are booked on the windings (rover.js adds w.heatW to the loss)
    let stator = 0;
    for (const w of R.wheels) {
      const statorW = (powered && !w.fault ? this.cmd.statorW : 0);
      w.heatW = statorW + (powered ? this.fx.motorExtraW * (this.cmd.hold ? 0.1 : this.cmd.torqueK ** 2) : 0); // copper loss ~ I^2
      stator += statorW;
      w._prevT = w._prevT ?? w.temp;
      const r = (w.temp - w._prevT) / dt * 60; w._prevT = w.temp;
      w.tRate = (w.tRate ?? 0) + (r - (w.tRate ?? 0)) * Math.min(1, dt / 20);
    }
    this.powerW = elec + stator;
    // transit warp moves the rover on rails (rover.js does not integrate the windings): integrate them here
    if ((sim.warp?.active || this._fast) && H?.th?.hub) {
      const M = { R: 0.05, C: 30000 };
      for (const w of R.wheels) {
        const loss = Math.max(0, w.power || 0) * 0.12 + (w.heatW || 0);
        const Th = H.th.hub[w.id] ?? 0;
        w.hubQ = (w.temp - Th) / M.R;
        w.temp += dt * (loss - w.hubQ) / M.C;
      }
    }
    // instrument health: off outside the survival-side of the operating range, back on 3 K inside it
    for (const k of ['lidar', 'camera']) {
      const N = NODES[k], T = this.n[k].T;
      if (this.ok[k] && (T > N.op[1] + 8 || T < N.op[0] - 8)) { this.ok[k] = false; this.log(`THERMAL: ${N.name} at ${T.toFixed(0)} C is outside its range (${N.op[0]}..${N.op[1]} C): instrument protected OFF`, 'err'); }
      else if (!this.ok[k] && T < N.op[1] - 3 && T > N.op[0] + 3) { this.ok[k] = true; this.log(`THERMAL: ${N.name} back in range (${T.toFixed(0)} C), on again`, 'ok'); }
    }
  }

  // ---------------- scenarios ----------------
  setScenarios(on, meanGapS) {
    const S = this.scen; S.on = on; if (meanGapS) S.meanGapS = meanGapS;
    if (on) S.next = Math.min(S.next, this.sim.time + 45);
    this.log(`Thermal scenarios ${on ? `ON (one every ~${Math.round(S.meanGapS / 60)} min of sim time on average)` : 'OFF'}`, 'ok');
  }
  trigger(id) {
    const S = this.scen, sunUp = this.sim.sunDir.y > 0.01;
    if (!id || id === 'random') {
      const ids = Object.keys(SCENARIOS).filter((k) => SCENARIOS[k].need !== 'sun' || sunUp);
      id = ids[Math.floor(S.rnd() * ids.length)];
    }
    const D = SCENARIOS[id]; if (!D) return false;
    if (D.need === 'sun' && !sunUp) { this.log(`Scenario "${D.name}" needs the sun up`, 'warn'); return false; }
    if (S.active) this._end();
    const dur = D.dur[0] + S.rnd() * (D.dur[1] - D.dur[0]);
    S.active = { id, t0: this.sim.time, t1: this.sim.time + dur, name: D.name };
    this.stats.scenarios++;
    S.history.unshift({ id, t: this.sim.time, name: D.name }); S.history.length = Math.min(S.history.length, 12);
    this._apply(id, true);
    this.log(`THERMAL SCENARIO: ${D.name} (${Math.round(dur / 60) || '<1'} min) - ${D.text}`, 'warn');
    return true;
  }
  _apply(id, on) {
    const E = this.env, f = this.fx, H = this.sim.health, R = this.sim.rover;
    switch (id) {
      case 'slope-heat': E.incBoost = on ? 30 : 0; break;
      case 'psr-cold': E.shadow = on; E.floorK = on ? ENV.psrK : ENV.nightK; break;
      case 'eclipse': E.blocked = on; E.floorK = on ? ENV.eclipseK : ENV.nightK; break;
      case 'dust-fins': f.hubEmisK = on ? 0.4 : 1; f.mcEmisK = on ? 0.6 : 1; break;
      case 'climb': f.motorExtraW = on ? 600 : 0; f.mcLossK = on ? 1 : 0; break;
      case 'glint': f.glint = on ? 1 : 0; break;
      case 'heater-stuck': f.vaultExtraW = on ? 600 : 0; break;
      case 'mc-overload': f.mcExtraW = on ? 400 : 0; break;
      case 'cold-start':
        if (on) {
          if (H?.th?.hub) for (let i = 0; i < 6; i++) H.th.hub[i] = -68;
          for (const w of R.wheels) { w.temp = -72; w._prevT = -72; }
          this.n.lidar.T = Math.min(this.n.lidar.T, -24); this.n.camera.T = Math.min(this.n.camera.T, -34); // just inside protection
        }
        break;
    }
  }
  _end() {
    const S = this.scen, a = S.active; if (!a) return;
    this._apply(a.id, false);
    S.active = null;
    this.log(`Thermal scenario ended: ${a.name}`, 'info');
  }
  _scenarios() {
    const S = this.scen, t = this.sim.time;
    if (S.active && t >= S.active.t1) { this._end(); S.next = t + S.minGapS + (-Math.log(1 - S.rnd()) * Math.max(0, S.meanGapS - S.minGapS)); }
    if (S.on && !S.active && t >= S.next) {
      if (!this.trigger('random')) S.next = t + 30;
    }
  }

  // ---------------- main loop ----------------
  update(dt) {
    this._scenarios();
    this._environment(dt);
    this.brain.tick(dt);
    this._nodes(dt);
    // statistics (for tests and the HUD)
    const st = this.stats, R = this.sim.rover;
    for (const k of ['lidar', 'camera', 'mc']) {
      const T = this.n[k].T, N = NODES[k];
      st.max[k] = Math.max(st.max[k] ?? -1e9, T); st.min[k] = Math.min(st.min[k] ?? 1e9, T);
      const inUse = k === 'mc' || R.powered;
      if (inUse && (T > N.op[1] || T < N.op[0])) st.outOfRangeS[k] += dt;
    }
    const wmax = Math.max(...R.wheels.map((w) => w.temp));
    st.max.wind = Math.max(st.max.wind ?? -1e9, wmax);
    st.min.wind = Math.min(st.min.wind ?? 1e9, Math.min(...R.wheels.map((w) => w.temp)));
    if (wmax > DRIVE_LIMITS.windMaxC) st.outOfRangeS.wind += dt;
  }

  // Fast thermal-only stepping (rover parked, no physics): health's vault / battery / hub model and these nodes. Used by
  // tests/thermal.mjs to run long scenarios in seconds, and by the "thermal soak" command to look ahead.
  soak(seconds, dt = 0.1) {
    const sim = this.sim; this._fast = true;
    try { for (let t = 0; t < seconds; t += dt) { sim.health?._thermal(dt); this.update(dt); sim.time += dt; } }
    finally { this._fast = false; }
  }

  // one-line state for the console / AI copilot
  summary() {
    const E = this.env, n = this.n, b = this.brain, R = this.sim.rover, H = this.sim.health;
    const wmax = Math.max(...R.wheels.map((w) => w.temp)), hub = H?.th?.hub ? Math.min(...H.th.hub) : NaN;
    return `regolith ${(E.regK - K0).toFixed(0)} C (sink ${E.sinkK.toFixed(0)} K, ${E.sunlit ? 'sunlit' : 'shadow'}) | LiDAR ${n.lidar.T.toFixed(1)} C cam ${n.camera.T.toFixed(1)} C MC ${n.mc.T.toFixed(1)} C windings max ${wmax.toFixed(0)} C hubs min ${hub.toFixed(0)} C vault ${H?.th?.Tv?.toFixed(1)} C battery ${H?.th?.Tb?.toFixed(1)} C | brain ${b.on ? 'ON' : 'OFF'}: drive ${b.choice.drive.label}, radiators ${b.choice.rad.label}, LiDAR ${b.choice.lidar.label}, camera ${b.choice.camera.label} | thermal power ${this.powerW.toFixed(0)} W${this.scen.active ? ` | scenario: ${this.scen.active.name}` : ''}`;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Utility-AI thermal brain
// how strong an intervention each option is (for escalate-fast / relax-slow)
const SEVERITY = { nominal: 0, auto: 0, down1: 1, up1: 1, heat: 1, cool: 1, precool: 1, conserve: 1, down2: 2, up2: 2, coolshade: 2, shed: 2, stop: 3 };
export class ThermalBrain {
  constructor(th) {
    this.th = th; this.on = true;
    this.horizonS = 120;           // look-ahead for spike detection
    this.choice = {}; this.since = {}; this.reason = {}; this.scores = {};
    for (const ch of ['drive', 'rad', 'lidar', 'camera']) { this.choice[ch] = { id: 'nominal', label: 'nominal' }; this.since[ch] = -1e9; this.reason[ch] = ''; }
    this._t = 0;
    this.decisions = 0;
    this.urg = { hot: 0, cold: 0, why: '' };
  }

  // predicted temperature: now + filtered rate x horizon (rate capped so one noisy step cannot fake a spike)
  pred(T, ratePerMin) { return T + clamp(ratePerMin, -20, 20) * this.horizonS / 60; }

  _ctx() {
    const th = this.th, sim = th.sim, R = sim.rover, H = sim.health, A = sim.autonomy, L = DRIVE_LIMITS;
    const ws = R.wheels;
    const wHot = ws.reduce((a, w) => (this.pred(w.temp, w.tRate || 0) > this.pred(a.temp, a.tRate || 0) ? w : a), ws[0]);
    const wCold = ws.reduce((a, w) => (w.temp < a.temp ? w : a), ws[0]);
    const windHotP = this.pred(wHot.temp, wHot.tRate || 0);
    const mc = th.n.mc, mcP = this.pred(mc.T, mc.rate);
    const hubMin = H?.th?.hub ? Math.min(...H.th.hub) : 0;
    const Tv = H?.th?.Tv ?? 20, Tb = H?.th?.Tb ?? 20;
    const vRate = this._rate('Tv', Tv), bRate = this._rate('Tb', Tb);
    const hotParts = [
      ['motor winding', windHotP, L.windWarnC, L.windMaxC, wHot.temp],
      ['motor controllers', mcP, NODES.mc.band[1], NODES.mc.op[1] + 5, mc.T],
      ['battery', this.pred(Tb, bRate), 38, THERMAL.battery.hotC + 3, Tb],
    ];
    let hot = 0, hotWhy = '';
    for (const [name, P, warn, max, now] of hotParts) { const u = smooth(P, warn, max); if (u > hot) { hot = u; hotWhy = `${name} ${now.toFixed(0)} C -> ${P.toFixed(0)} C in 2 min (limit ${max} C)`; } }
    const coldParts = [
      ['wheel hubs (gear grease)', hubMin, L.hubColdWarnC, L.hubColdMaxC],
      ['motor windings', wCold.temp, L.windColdWarnC, L.windColdMaxC],
    ];
    let cold = 0, coldWhy = '';
    for (const [name, T, warn, max] of coldParts) { const u = smooth(-T, -warn, -max); if (u > cold) { cold = u; coldWhy = `${name} ${T.toFixed(0)} C (stiff below ${warn} C)`; } }
    const v = R.body.linvel(), moving = Math.hypot(v.x, v.z) > 0.05;
    const wantsDrive = moving || (A && A.mode === 'AUTO') || Math.abs(R.cmd.vx) + Math.abs(R.cmd.wz) > 0.01;
    const slope = Math.max(Math.abs(R.tilt?.pitch || 0), Math.abs(R.tilt?.roll || 0)) / DEG;
    const vaultP = this.pred(Tv, vRate);
    return { R, H, A, hot, hotWhy, cold, coldWhy, moving, wantsDrive, slope, loaded: !!R.loaded, soc: R.power?.soc ?? 1, Tv, Tb, vaultP, vRate, powered: !!R.powered, survival: !!H?.survival?.on };
  }
  _rate(key, T) {
    this._r = this._r || {}; const r = this._r[key] || (this._r[key] = { T, rate: 0 });
    const d = (T - r.T) / Math.max(1e-3, this._dt) * 60; r.T = T; r.rate += (d - r.rate) * Math.min(1, this._dt / 20);
    return r.rate;
  }

  // option tables: [id, label, utility(ctx) -> [score, reason]]
  _driveOptions(c) {
    const loadedSlope = c.loaded ? smooth(c.slope, 5, 12) : smooth(c.slope, 10, 18);
    const lowSoc = smooth(-c.soc, -0.35, -0.15);
    return [
      ['nominal', 'nominal', () => [0.35 * (1 - c.hot) * (1 - c.cold), 'all drive temperatures in band']],
      ['down1', 'REV DOWN 1 (150 rpm, 85 % torque)', () => [bell(c.hot, 0.45, 0.22) - 0.15 * loadedSlope, `heat rising: ${c.hotWhy}`]],
      ['down2', 'REV DOWN 2 (120 rpm, 70 % torque)', () => [bell(c.hot, 0.78, 0.2) - 0.2 * loadedSlope, `hot: ${c.hotWhy}`]],
      ['stop', 'COOL-DOWN STOP (park, radiate)', () => [1.15 * smooth(c.hot, 0.9, 1.0), `critical: ${c.hotWhy}`]],
      ['up1', 'REV UP (stator heating 120 W per motor)', () => [smooth(c.cold, 0.15, 0.55) * (c.wantsDrive ? 1 : 0.8) - 0.5 * lowSoc, `cold: ${c.coldWhy}`]],
      ['up2', 'REV UP MAX (stator heating 350 W per motor)', () => [smooth(c.cold, 0.55, 0.95) * 1.1 - 0.6 * lowSoc, `very cold: ${c.coldWhy}`]],
    ];
  }
  _radOptions(c, th) {
    const heatEvent = th.scen.active && ['slope-heat', 'heater-stuck', 'climb', 'mc-overload'].includes(th.scen.active.id) ? 1 : 0;
    const vaultHot = smooth(c.vaultP, 30, THERMAL.vault.hotC);
    const vaultCold = smooth(-c.vaultP, -2, 8);
    return [
      ['auto', 'thermostat louvers', () => [0.3, 'vault and battery in band']],
      ['precool', 'PRE-COOL (louvers forced open)', () => [Math.max(vaultHot * 0.9, 0.45 * heatEvent * (c.Tv > 18 ? 1 : 0)), `vault ${c.Tv.toFixed(1)} C -> ${c.vaultP.toFixed(1)} C${heatEvent ? ', heat event under way' : ''}`]],
      ['shed', 'SHED LOAD (avionics low-power, louvers open)', () => [1.1 * smooth(c.vaultP, THERMAL.vault.hotC - 6, THERMAL.vault.hotC + 2), `vault ${c.Tv.toFixed(1)} C heading for ${THERMAL.vault.hotC} C`]],
      ['conserve', 'CONSERVE (louvers closed, heaters +3 K when driving)', () => [0.9 * vaultCold, `vault ${c.Tv.toFixed(1)} C -> ${c.vaultP.toFixed(1)} C`]],
    ];
  }
  _instrOptions(k, c, th) {
    const N = NODES[k], s = th.n[k], P = this.pred(s.T, s.rate);
    const hot = smooth(P, N.band[1] - 4, N.op[1]), cold = smooth(-P, -(N.band[0] + 4), -N.op[0]);
    const sun = th.env.sunlit ? 1 : 0;
    return [
      ['nominal', 'nominal', () => [0.3, `${s.T.toFixed(1)} C in band ${N.band[0]}..${N.band[1]} C`]],
      ['heat', 'HEAT (film heater)', () => [cold, `${s.T.toFixed(1)} C -> ${P.toFixed(1)} C, cold`]],
      ['cool', 'TEC COOL', () => [hot * (sun ? 0.85 : 1), `${s.T.toFixed(1)} C -> ${P.toFixed(1)} C, hot`]],
      ['coolshade', 'TEC COOL + SUNSHADE', () => [hot * sun * (0.6 + 0.6 * smooth(P, N.band[1], N.op[1])), `${s.T.toFixed(1)} C -> ${P.toFixed(1)} C in the sun`]],
    ];
  }

  _decide(ch, options, now) {
    let best = null, bestS = -1e9, bestR = '';
    const cur = this.choice[ch].id;
    const scores = [];
    for (const [id, label, fn] of options) {
      let [s, why] = fn();
      if (id === cur) s += 0.08;                         // inertia: keep the current action unless clearly beaten
      scores.push([id, s]);
      if (s > bestS) { bestS = s; best = { id, label }; bestR = why; }
    }
    this.scores[ch] = scores;
    // Escalate fast, relax slowly: a stronger action may replace a weaker one after 8 s, but stepping back down
    // (toward nominal) waits 60 s and needs a clear margin. Without this, REV DOWN 2 cooled the controllers, the
    // predicted temperature dropped, REV DOWN 1 won, they heated again... a 10 s limit cycle.
    const dwell = now - this.since[ch];
    const sev = (id) => SEVERITY[id] ?? 0;
    const relaxing = sev(best.id) < sev(cur);
    const curScore = (scores.find(([id]) => id === cur) || [cur, -1e9])[1];
    const okToSwitch = relaxing ? dwell >= 60 && bestS > curScore + 0.12 : dwell >= 8;
    if (best.id !== cur && okToSwitch) {
      const prev = this.choice[ch];
      this.choice[ch] = best; this.since[ch] = now; this.reason[ch] = bestR; this.decisions++;
      const names = { drive: 'drive', rad: 'radiators', lidar: 'LiDAR', camera: 'camera' };
      const lvl = /STOP|MAX|SHED/.test(best.label) ? 'err' : best.id === 'nominal' || best.id === 'auto' ? 'ok' : 'warn';
      this.th.log(`THERMAL BRAIN: ${names[ch]} ${prev.label} -> ${best.label}${best.id === 'nominal' || best.id === 'auto' ? '' : ` (${bestR})`}`, lvl);
    } else if (best.id === cur) this.reason[ch] = bestR;
  }

  tick(dt) {
    const th = this.th, sim = th.sim, R = sim.rover, H = sim.health, now = sim.time;
    this._t -= dt; this._dt = (this._dtAcc || 0) + dt; this._dtAcc = this._dt;
    if (this._t > 0) { this._applyContinuous(dt); return; }
    this._t = 1; this._dtAcc = 0;
    const c = this._ctx(); this.ctx = c;
    if (!this.on) {
      for (const ch of Object.keys(this.choice)) this.choice[ch] = { id: 'off', label: 'brain OFF' };
      Object.assign(th.cmd, { rpm: Infinity, torqueK: 1, statorW: 0, louver: null, setBias: 0, vaultLoadK: 1, hold: false });
      for (const k of ['lidar', 'camera']) { th.n[k].heaterW = 0; th.n[k].tecQ = 0; th.n[k].shade = false; }
      this._push(); return;
    }
    if (c.powered) this._decide('drive', this._driveOptions(c), now);
    else if (this.choice.drive.id !== 'nominal') { this.choice.drive = { id: 'nominal', label: 'nominal (drive off)' }; }
    this._decide('rad', this._radOptions(c, th), now);
    for (const k of ['lidar', 'camera']) this._decide(k, this._instrOptions(k, c, th), now);
    // drive channel -> commands
    const d = this.choice.drive.id, cmd = th.cmd;
    cmd.rpm = d === 'down1' ? 150 : d === 'down2' ? 120 : d === 'stop' ? 120 : Infinity;
    cmd.torqueK = d === 'down1' ? 0.85 : d === 'down2' ? 0.7 : 1;
    cmd.statorW = d === 'up1' ? 120 : d === 'up2' ? 350 : 0;
    cmd.hold = d === 'stop';
    // radiators -> commands
    const r = this.choice.rad.id;
    cmd.louver = r === 'precool' || r === 'shed' ? 1 : r === 'conserve' ? 0 : null;
    // set-point bias only while the drive is powered (pre-warming for a drive): in survival every heater watt comes out
    // of a 354 h night budget (tests/fdir.mjs: +5 K cost ~1.4 kW, 148 % of the pack); survival keeps the louvers shut only
    cmd.setBias = r === 'conserve' && c.powered && !c.survival ? 3 : 0;
    cmd.vaultLoadK = r === 'shed' ? 0.75 : 1;
    if (cmd.hold && H) H.hold(2, 'thermal brain: cool-down stop');
    this._applyContinuous(dt);
    this._push();
  }

  // instrument heaters / TECs: proportional control toward the band, every tick (10 Hz)
  _applyContinuous() {
    const th = this.th;
    if (!this.on) return;
    for (const k of ['lidar', 'camera']) {
      const N = NODES[k], s = th.n[k], mode = this.choice[k].id;
      const powered = th.sim.rover.powered;
      // survival (drive and sensors off): heaters only, to the survival floor + 5 K
      const lo = powered ? N.band[0] + 3 : N.surv[0] + 5, hi = N.band[1] - 3;
      s.heaterW = mode === 'heat' || s.T < lo ? clamp((lo + 2 - s.T) * 3, 0, N.heaterW) : 0;
      s.tecQ = powered && (mode === 'cool' || mode === 'coolshade' || s.T > N.band[1]) ? clamp((s.T - hi) * 4, 0, N.tecMaxW) : 0;
      s.shade = mode === 'coolshade';
    }
  }
  // publish commands for the rover (rpm cap, torque, stator heating) and health (louvers, set points, load)
  _push() {
    const th = this.th, R = th.sim.rover, cmd = th.cmd;
    R.thermalRpm = cmd.rpm; R.thermalTorqueK = cmd.torqueK;
  }
}
