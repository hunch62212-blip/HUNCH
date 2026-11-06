/* Deployable LiDAR mast (rover v12): three nested aluminium tubes on a lead screw, over a two-pin base gimbal.
 *
 * Why this replaces the old mast. The v10/v11 mast was a fixed tube plus an inner tube that was only STRETCHED (its foot stayed put, its top
 * rose) and a head that was moved by a smoothstep at 0.18 m/s. Nothing could slide, nothing was driven, and a bad frame order left a visible
 * gap between the tube and the head. A real telescope cannot do that: each stage is a rigid tube that translates inside the one below it,
 * so a stage can only leave its parent by the distance the actuator has moved it, and must keep an overlap to stay stiff.
 *
 * Mechanism (every joint is a pin, a slide or a screw; nothing is lerped):
 *   - Stage 1: fixed tube clamped to a base clevis. The clevis is a two-axis pin gimbal (roll about x, pitch about z, +-0.20 rad, 0.5 rad/s)
 *     that the IMU stabiliser leans against the chassis tilt (js/rover_sensors.js). Three triangular gusset plates tie the tube to the clevis.
 *   - Stage 2 slides inside stage 1, driven by a Tr8x2 lead screw (lead 2 mm, self-locking: tan(lead angle) 0.09 < friction 0.15, 0 W to hold).
 *   - Stage 3 (carries the LiDAR) slides inside stage 2, driven by a closed cord loop over a pulley at the top of stage 1, so it always moves the
 *     same distance relative to stage 2 as stage 2 does to stage 1: the tip travels twice the nut.
 *   - Slew: tip speed <= 0.025 m/s (nut 0.0125 m/s, 375 rpm of the screw), trapezoid profile at 0.020 m/s^2; hard stops at 0 and 0.45 m.
 *
 * Frame: mast foot (deck level at the mast), y up. Pure logic, no THREE. Used by js/fusion_rover.js (the model), js/module_sortie.js and
 * rover_model.html (the actuator), tools/mast_test.js (checks). Material and damping numbers are ASSUMED where marked.
 */
(function (root) {
  'use strict';
  var G0 = 9.80665;
  var MAT = { E: 69e9, rho: 2700, yieldPa: 240e6, name: '6061-T6 tube' };

  var GEO = {
    strokeM: 0.45,                       // tip travel
    flangeY: 0.012,                      // top of the base flange: stage 1 stands on it
    // tubes bottom to top: outer diameter, wall, length, retracted bottom y
    tubes: [
      { id: 'S1', od: 0.026, wall: 0.0015, L: 0.270, y0: 0.012 },
      { id: 'S2', od: 0.021, wall: 0.0015, L: 0.268, y0: 0.016 },
      { id: 'S3', od: 0.016, wall: 0.0012, L: 0.262, y0: 0.022 }
    ],
    collar: { od: 0.032, h: 0.014 },     // lock collar and PTFE bushing on the top of stage 1
    bushingK: 2.0e6,                     // N/m, contact stiffness of one PTFE bushing (ASSUMED)
    minOverlapPerOD: 1.5,                // sliding-fit rule: engagement >= 1.5 x the inner tube's OD
    lidarBase: { h: 0.028, r: 0.032 },   // YDLIDAR X2L base sits on the top cap of stage 3
    lidarHeadY: 0.032,                   // head centre above the top of stage 3
    topCap: 0.004                        // stage-3 end cap under the lidar base
  };
  GEO.tubes.forEach(function (t) { t.id_ = t.od - 2 * t.wall; });
  var STAGES = GEO.tubes.length;
  var NODES = STAGES - 1;                // each moving stage travels stroke / NODES relative to its parent

  var ACT = {
    vTip: 0.025,                         // m/s, tip slew limit (client spec)
    aTip: 0.020,                         // m/s^2, trapezoid acceleration
    lead: 0.002, screwEff: 0.30,         // m per rev, Tr8x2 efficiency (self-locking)
    muThread: 0.15,                      // thread friction (ASSUMED)
    pitchD: 0.007,                       // m, pitch diameter of the thread
    cordFriction: 0.35,                  // N, cord and pulley drag (ASSUMED)
    motorV: 12.0, idleW: 0.0, elecW: 0.6 // driver and logic while the motor runs
  };
  ACT.leadAngle = Math.atan(ACT.lead / (Math.PI * ACT.pitchD));
  ACT.selfLocking = Math.tan(ACT.leadAngle) < ACT.muThread;
  ACT.screwRpmAtSlew = (ACT.vTip / NODES) / ACT.lead * 60;

  var GIMBAL = { rangeRad: 0.20, rateRadS: 0.5, accelRadS2: 2.0, pinD: 0.004 };

  var TIP = { lidarKg: 0.19, mountKg: 0.04, antennaKg: 0.006, zeta: 0.04 };   // BOM lidar 0.19 kg; zeta of a greased PTFE-bushed tube: ASSUMED
  TIP.kg = TIP.lidarKg + TIP.mountKg + TIP.antennaKg;

  function clamp(x, a, b) { return x < a ? a : (x > b ? b : x); }
  function tubeI(t) { var ri = t.od / 2 - t.wall, ro = t.od / 2; return Math.PI / 4 * (Math.pow(ro, 4) - Math.pow(ri, 4)); }
  function tubeA(t) { var ri = t.od / 2 - t.wall, ro = t.od / 2; return Math.PI * (ro * ro - ri * ri); }
  function tubeKg(t) { return MAT.rho * tubeA(t) * t.L; }

  /** Stage bottoms and tops for a tip extension e (m, 0..stroke). Stage k (k>=1) is carried k * (e / NODES): the screw moves S2, the cord S3. */
  function stages(e) {
    e = clamp(e, 0, GEO.strokeM);
    var s = e / NODES, out = [];
    for (var k = 0; k < STAGES; k++) {
      var t = GEO.tubes[k], bottom = t.y0 + k * s;
      out.push({ id: t.id, bottom: bottom, top: bottom + t.L, od: t.od, id_: t.id_, L: t.L, centre: bottom + t.L / 2 });
    }
    return out;
  }
  /** Engagement (m) of each moving stage inside its parent: parent top minus child bottom. */
  function overlaps(e) { var st = stages(e), o = []; for (var k = 1; k < STAGES; k++) o.push(st[k - 1].top - st[k].bottom); return o; }
  function minOverlapM(k) { return GEO.minOverlapPerOD * GEO.tubes[k].od; }                 // k = index of the inner tube
  /** y of the top of stage 3 (where the LiDAR base clamps) and of the head centre */
  function tipTopY(e) { var st = stages(e); return st[STAGES - 1].top + GEO.topCap; }
  function lidarHeadY(e) { return tipTopY(e) + GEO.lidarBase.h + GEO.lidarHeadY - GEO.lidarBase.h * 0.5; }

  /* ---------- structure: cantilever with tip mass, tube sections that overlap act together (bushing engagement factor) ---------- */
  var ETA = 0.70;                                  // share of the inner tube's I that works in an overlap (clearance in the bushings; ASSUMED)
  /** bending stiffness EI(y) at height y for extension e */
  function EIat(e, y) {
    var st = stages(e), ei = 0, k;
    for (k = 0; k < STAGES; k++) {
      if (y < st[k].bottom || y > st[k].top) continue;
      var inner = k > 0 && y <= st[k - 1].top;     // this tube is inside its parent here
      ei += MAT.E * tubeI(GEO.tubes[k]) * (inner ? ETA : 1);
    }
    return ei;
  }
  function modal(e) {
    var y0 = GEO.flangeY, H = tipTopY(e) + 0.020, n = 400, dy = (H - y0) / n, del = 0, th = 0, mT = 0, k;
    for (var i = 0; i < n; i++) {
      var y = y0 + (i + 0.5) * dy, ei = EIat(e, y);
      if (ei <= 0) continue;
      var arm = H - y; del += arm * arm / ei * dy; th += arm / ei * dy;
    }
    var st = stages(e), ov = overlaps(e);
    for (k = 0; k < NODES; k++) {                  // rotational spring of the bushing pair at each joint: two bushings span the overlap
      var span = Math.max(0.01, ov[k]), kr = 2 * GEO.bushingK * Math.pow(span / 2, 2), armJ = H - (st[k].top - span / 2);
      del += armJ * armJ / kr; th += armJ / kr;
    }
    for (k = 0; k < STAGES; k++) mT += tubeKg(GEO.tubes[k]);
    var kTip = 1 / del;                            // N/m at the lidar
    var mEff = TIP.kg + 0.236 * mT;                // Rayleigh effective mass of a uniform cantilever: 33/140 of the beam
    var w = Math.sqrt(kTip / mEff);
    return { H: H, kTip: kTip, tipRotPerN: th, mTubes: mT, mEff: mEff, wn: w, fn: w / (2 * Math.PI), zeta: TIP.zeta };
  }
  /** Worst bending stress (Pa) for a tip force F (N). Conservative: at each tube's critical section (the foot for stage 1, the exit from its parent's
   *  collar for the stages above) that tube alone carries the whole moment F x (H - y); the parent's help inside the overlap is ignored. */
  function stressAtJoint(e, F) {
    var st = stages(e), H = modal(e).H, worst = 0;
    for (var k = 0; k < STAGES; k++) {
      var yCrit = k === 0 ? GEO.flangeY : st[k - 1].top;
      var sigma = F * (H - yCrit) * (GEO.tubes[k].od / 2) / tubeI(GEO.tubes[k]);
      worst = Math.max(worst, sigma);
    }
    return worst;
  }

  /** Base-shake response. acc(t): horizontal acceleration of the foot (m/s^2). Integrates x'' + 2 zeta w x' + w^2 x = -a(t) (x = tip relative to the foot).
   *  Returns peak tip deflection, peak tilt of the scan plane, peak stress, safety factor, settling time (to 5 % of the peak) and the trace. */
  function shock(e, acc, T, dt) {
    var m = modal(e), w = m.wn, z = m.zeta, x = 0, v = 0, n = Math.ceil(T / dt), peak = 0, peakF = 0, trace = [], tPeak = 0, i;
    var sub = Math.max(1, Math.ceil(dt * w * 20 / 1)), h = dt / sub;
    for (i = 0; i < n; i++) {
      var t = i * dt;
      for (var s = 0; s < sub; s++) { var a = acc(t + s * h); v += (-a - 2 * z * w * v - w * w * x) * h; x += v * h; }
      var ax = Math.abs(x); if (ax > peak) { peak = ax; tPeak = t; }
      peakF = Math.max(peakF, m.kTip * ax);
      trace.push(x);
    }
    var thresh = 0.05 * peak, tSettle = 0;
    for (i = trace.length - 1; i >= 0; i--) if (Math.abs(trace[i]) > thresh) { tSettle = i * dt; break; }
    var sig = stressAtJoint(e, peakF);                                     // the static tip weight is axial, so only the dynamic force bends the tubes
    return { peakDeflM: peak, tPeak: tPeak, peakTiltRad: m.tipRotPerN * peakF, peakF: peakF, peakStressPa: sig, safety: sig > 0 ? MAT.yieldPa / sig : 99,
      settleS: Math.max(0, tSettle - tPeak), fn: m.fn, zeta: z, trace: trace };
  }

  /* ---------- actuator: trapezoid slew on the tip, hard stops, lead-screw load and power ---------- */
  function createActuator(opts) {
    opts = opts || {};
    var g = opts.g != null ? opts.g : 1.622;
    var S = {
      e: opts.e0 || 0, v: 0, target: opts.e0 || 0, u: 0, moving: false, atStop: 0, stalled: false,
      screwRpm: 0, forceN: 0, motorW: 0, wh: 0, events: []
    };
    var weightN = function () {                                    // load on the nut: S2 + S3 + head, lifted by the screw (cord halves the force on the nut? it doubles it)
      var m = TIP.kg, k; for (k = 1; k < STAGES; k++) m += tubeKg(GEO.tubes[k]);
      return m * g;
    };
    function command(target) { S.target = clamp(target, 0, GEO.strokeM); }
    function commandU(u) { command(clamp(u, 0, 1) * GEO.strokeM); }
    function step(dt) {
      var err = S.target - S.e, dir = err > 0 ? 1 : (err < 0 ? -1 : 0);
      var vStop = Math.sqrt(2 * 0.8 * ACT.aTip * Math.abs(err));   // fastest speed from which it can still stop on the target; planned at 80 % of the allowed decel so the rate-limited speed can track the steepening curve
      var vWant = dir * Math.min(ACT.vTip, vStop);
      S.v += clamp(vWant - S.v, -ACT.aTip * dt, ACT.aTip * dt);
      var de = S.v * dt;
      if (dir !== 0 && dir * de >= Math.abs(err)) { S.e = S.target; S.v = 0; }   // the next step would reach the target: seat on it (speed here is already <= 2 a dt)
      else if (dir === 0) S.v = 0;
      else S.e += de;
      S.atStop = 0;
      if (S.e <= 0) { S.e = 0; if (S.v < 0) { S.v = 0; S.atStop = -1; } }
      if (S.e >= GEO.strokeM) { S.e = GEO.strokeM; if (S.v > 0) { S.v = 0; S.atStop = 1; } }
      S.moving = Math.abs(S.v) > 1e-6;
      S.u = S.e / GEO.strokeM;
      // lead screw: nut speed is a half of the tip's; the nut carries the weight of the moving stages plus the cord drag
      var nutV = Math.abs(S.v) / NODES;
      S.screwRpm = nutV / ACT.lead * 60;
      var up = S.v >= 0;
      S.forceN = weightN() * (up ? 1 : -1) + ACT.cordFriction * (S.moving ? (up ? 1 : -1) : 0);
      var P = S.moving ? Math.max(0, S.forceN * (up ? 1 : -1)) * nutV / ACT.screwEff : 0;    // lowering back-drives through friction: drawn power is the friction share only
      S.motorW = S.moving ? P + ACT.elecW : ACT.idleW;             // holding: the screw self-locks, the driver is off
      S.wh += S.motorW * dt / 3600;
      return S;
    }
    return { state: S, command: command, commandU: commandU, step: step,
      atTarget: function (tolM) { return Math.abs(S.e - S.target) <= (tolM != null ? tolM : 0.001) && !S.moving; },
      /** snap to a position with no motion (a CAD pose or a test setup); not a motion command */
      setPose: function (u) { S.e = S.target = clamp(u, 0, 1) * GEO.strokeM; S.v = 0; S.u = clamp(u, 0, 1); S.moving = false; } };
  }

  /* ---------- gimbal: two pins, slew and accel limited, hard range stops ---------- */
  function createGimbal() {
    var S = { x: 0, z: 0, vx: 0, vz: 0, satX: false, satZ: false };
    function axis(p, v, tgt, dt) {
      var asked = tgt; tgt = clamp(tgt, -GIMBAL.rangeRad, GIMBAL.rangeRad);
      var want = clamp((tgt - p) / 0.15, -GIMBAL.rateRadS, GIMBAL.rateRadS);          // 150 ms position loop, then the rate limit
      v += clamp(want - v, -GIMBAL.accelRadS2 * dt, GIMBAL.accelRadS2 * dt);
      p += v * dt;
      var sat = Math.abs(asked) > GIMBAL.rangeRad;                                      // the request is beyond the pins' travel
      if (p > GIMBAL.rangeRad) { p = GIMBAL.rangeRad; v = Math.min(0, v); sat = true; }
      if (p < -GIMBAL.rangeRad) { p = -GIMBAL.rangeRad; v = Math.max(0, v); sat = true; }
      return [p, v, sat];
    }
    function step(dt, tx, tz) {
      var a = axis(S.x, S.vx, tx, dt), b = axis(S.z, S.vz, tz, dt);
      S.x = a[0]; S.vx = a[1]; S.satX = a[2]; S.z = b[0]; S.vz = b[1]; S.satZ = b[2];
      return S;
    }
    return { state: S, step: step };
  }

  /** Structural checks the model must satisfy at every extension: nested radially, engaged axially, no tube leaves its parent. */
  function check(e) {
    var st = stages(e), ov = overlaps(e), issues = [], k;
    for (k = 1; k < STAGES; k++) {
      if (st[k].od >= st[k - 1].id_) issues.push('S' + (k + 1) + ' does not fit inside S' + k);
      if (ov[k - 1] < minOverlapM(k) - 1e-9) issues.push('S' + (k + 1) + ' engagement ' + (ov[k - 1] * 1000).toFixed(1) + ' mm < ' + (minOverlapM(k) * 1000).toFixed(1) + ' mm');
      if (st[k].bottom < st[k - 1].bottom + 0.002) issues.push('S' + (k + 1) + ' foot below its parent foot');
    }
    return issues;
  }

  var api = {
    MAT: MAT, GEO: GEO, ACT: ACT, GIMBAL: GIMBAL, TIP: TIP, STAGES: STAGES, NODES: NODES,
    stages: stages, overlaps: overlaps, minOverlapM: minOverlapM, tipTopY: tipTopY, lidarHeadY: lidarHeadY,
    tubeI: tubeI, tubeKg: tubeKg, modal: modal, stressAtJoint: stressAtJoint, shock: shock, check: check,
    createActuator: createActuator, createGimbal: createGimbal, EIat: EIat
  };
  root.NASA_SensorMast = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
