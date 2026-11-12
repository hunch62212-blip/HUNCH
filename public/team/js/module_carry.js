/* Cargo module and the rover's kneel-and-rise ring clamp (rover v10).
 *
 * Module (modelled on the "Lunar Cargo Module" reference image): a long silver MLI-wrapped cylinder with gold end rings,
 * a docking face with a hatch on the front (+x) end, a probe on the rear (-x) end, two side cargo doors, handrails, a roof
 * solar panel and two small dishes. Sized from the rover (24 in cube, 0.6096 m): length 1.226 x the rover's (0.748 m, v11: 15 % longer),
 * diameter 80 % of its width (0.488 m). It lies ALONG the rover (axis on the rover's x).
 * Mass 3.45 kg: same hollow pressurised-module density as v7 (about 25 kg per m3 of envelope). ASSUMED, not from CAD.
 *
 * Stand (the reference image's skid frame): two side frames OUTSIDE the tyre track (posts at z = +-0.33 m) hold the module
 * by four side lift lugs (x = +-0.18 m) in V notches. Nothing crosses under the module, so the rover drives straight in along it.
 *
 * Pickup (no lead screws: the air suspension does the lifting):
 *   1. the suspension KNEELS (ride -40 mm): the saddles pass 32 mm under the module;
 *   2. the rover reverses, tail first, along the module axis until the clamp bands are over the rings (axial error +-12 mm);
 *   3. the suspension RISES: at ride -8 mm the saddles touch, their 120 deg V centres the module sideways, then the
 *      weight moves onto the rover over 4 mm of strut travel and the lugs lift 58 mm (46 mm over the notch horns, so the
 *      module can slide out over the posts);
 *   4. each ring's two articulated jaws rotate up round the module on a circular track (60 deg arcs; saddle + jaws wrap 240 deg, so the
 *      module cannot come out of the top) and textured Vespel SP-21 / sintered-bronze flexure pads squeeze to 90 N; worm drives self-lock (0 W holding).
 *      The forward clamp ring floats on +-3.4 mm (+-55 mm full-scale) linear bearings for lunar day/night delta-CTE thermal compliance.
 *   5. drive. Unload is the reverse: over the stand, jaws open and retract, the rover lowers until the lugs seat, kneels clear, drives out.
 *
 * Frame: rover root, x forward, y up, z port. State h = ride extension - kneel extension (0 = fully knelt), so the lift
 * numbers read like a lift travel. Pure logic, no THREE; fixed dt and the caller's gravity.
 */
(function (root) {
  'use strict';
  var ROVER_L = 0.6096, ROVER_W = 0.6096;
  var L = +(1.226 * ROVER_L).toFixed(3), D = +(0.8 * ROVER_W).toFixed(3);   // v11: 15 % longer than v10 (1.066 x the rover)
  var MODULE = {
    id: 'module40', label: '40 ft cylindrical cargo module (1:16.3)', scale: +(12.192 / L).toFixed(2),
    fullM: [12.192, +(D * 12.192 / L).toFixed(2), +(D * 12.192 / L).toFixed(2)],
    L: L, D: D, R: D / 2, W: D, H: D,
    kg: 3.45,                                          // 25 kg/m3 of envelope x pi R^2 L (ASSUMED, hollow pressurised module)
    axis: 'x',                                         // lies along the rover
    collarL: 0.040, probeL: 0.030,                     // docking face (+x) and probe (-x), inside L
    bandX: 0.085, bandW: 0.050,                        // clamp bands (wear sleeves) over the rings
    lugX: 0.21, lugZ: 0.345, lugR: 0.010                // side lift lugs (trunnion pins) and where the stand notches hold them
  };
  var CRADLE = {
    seatX: 0.0, stopX: 0.0,                            // module centre over the six-wheel base centre: even weight on the wheels
    ringX: 0.085,                                      // two rings, just aft of the avionics pod (pod front edge x 0.1125)
    seatY: 0.318,                                      // module bottom when seated, rover root frame (belly charge coupler; clear of top avionics lid)
    standY: 0.310,                                     // module bottom on the stand, above ground + tyre sinkage, at ride 0
    kneelExt: -0.040, carryExt: 0.050,                 // suspension ride: knelt for the approach, raised to carry (= the auto loaded ride, v11: 50 mm)
    hornH: 0.012,                                      // stand V-notch horns rise 12 mm above the notch bottom
    cteComplianceM: 0.0034,                            // +-3.4 mm CAD (+-55 mm full scale) slotted axial compliance at fwd ring/trunnions across 40..400 K
    saddleRim: D / 2 + 0.012, saddleArcDeg: 120, jawArcDeg: 60, jawStowDeg: 60, ringT: 0.012,
    shareTravel: 0.004,                                // strut travel over which the weight moves from the stand to the rover
    liftV: 0.050, liftXferV: 0.015,                    // m/s: suspension high-flow speed in a pickup, and the slow speed through the 4 mm weight transfer (ASSUMED)
    liftEta: 0.25,                                     // pump efficiency (matches js/suspension.js pumpEta)
    captureZ: 0.025, captureYaw: 0.06, captureX: 0.030, // V saddle catches +-25 mm sideways, +-0.06 rad yaw; band must be within 30 mm
    centerTol: 0.003, axialTol: 0.012,                 // sideways centring left after the V; axial error the 50 mm band accepts
    lidCloseS: 2.0,                                    // jaw rotation time
    padGap: 0.012, padKpad: 20000, padVsq: 0.006,      // radial clearance, pad stiffness N/m, squeeze speed m/s
    clampMaxN: 90, clampLockN: 45, clampKF: 35,        // per-ring force limit, "locked" threshold, worm-drive N per amp (ASSUMED)
    motorI0: 0.15, motorV: 12.0,
    muPad: 0.55,                                       // textured Vespel SP-21 / sintered-bronze cryogenic pad on steel wear sleeve
    notchDetentN: 6                                    // V notch holds the lugs against rolling drag
  };
  CRADLE.axisY = CRADLE.seatY + MODULE.R;              // module axis when seated (rover root frame)
  CRADLE.contactExt = CRADLE.standY - CRADLE.seatY;    // ride at which the saddles touch the module
  CRADLE.contactH = +(CRADLE.contactExt - CRADLE.kneelExt).toFixed(4);
  CRADLE.liftMaxH = +(CRADLE.carryExt - CRADLE.kneelExt).toFixed(4);
  CRADLE.liftTravelS = CRADLE.liftMaxH / CRADLE.liftV;
  CRADLE.topTol = 0.0005;                              // the air struts stop within their 0.2 mm dead-band of the target: 'at the top' means within 0.5 mm
  CRADLE.railY = CRADLE.standY;                        // old name (module bottom plane on the stand)

  /* v12: the jaw drive is a worm and wheel, so it LOCKS. A single-start worm (module 1.0 mm, 10 mm pitch diameter, 30-tooth wheel) has lead angle
   * lambda = atan(lead / (pi d)); with bronze on steel at mu = 0.12 the friction angle phi = atan(mu) is larger, so the wheel cannot turn the worm:
   *   forward efficiency  eta_f = tan(lambda) / tan(lambda + phi)          (motor turns the jaw: about 45 %)
   *   back-drive          eta_b = tan(lambda - phi) / tan(lambda) < 0      (no torque at the motor can be produced by a load on the jaw: it holds with the motor OFF)
   * The pad force per motor amp follows from the same numbers: F = eta_f * Kt * ratio / r_eff, which replaces the old fixed 35 N/A. */
  var WORM = { moduleMm: 1.0, pitchDmm: 10.0, starts: 1, wheelTeeth: 30, mu: 0.12, motorKt: 0.0206, rEffM: 0.0079, backlashDeg: 0.25 };
  WORM.leadMm = Math.PI * WORM.moduleMm * WORM.starts;
  WORM.lambda = Math.atan(WORM.leadMm / (Math.PI * WORM.pitchDmm));
  WORM.phi = Math.atan(WORM.mu);
  WORM.selfLocking = WORM.lambda < WORM.phi;
  WORM.etaForward = Math.tan(WORM.lambda) / Math.tan(WORM.lambda + WORM.phi);
  WORM.etaBack = Math.tan(WORM.lambda - WORM.phi) / Math.tan(WORM.lambda);
  WORM.ratio = WORM.wheelTeeth / WORM.starts;
  WORM.clampNPerA = WORM.etaForward * WORM.motorKt * WORM.ratio / WORM.rEffM;
  CRADLE.clampKF = +WORM.clampNPerA.toFixed(1);
  CRADLE.aBrakeMax = 1.2;                              // m/s2: hardest braking the rover is allowed (the clamp must hold the module through it on a slope)
  /** motor torque (N m) needed to keep the jaw from opening under an outward load on the pad (N): 0 while the worm self-locks */
  WORM.holdTorque = function (loadN) { return WORM.selfLocking ? 0 : loadN * WORM.rEffM / WORM.ratio * Math.abs(Math.tan(WORM.lambda - WORM.phi)); };

  function clamp(x, a, b) { return x < a ? a : (x > b ? b : x); }

  /** opts: { g, x0 (module centre x in the rover frame at the start, m), z0 (sideways offset, m), yaw0 (rad) } */
  function create(opts) {
    opts = opts || {};
    var g = opts.g != null ? opts.g : 1.622;
    var C = CRADLE;
    var S = {
      g: g, phase: 'STAGED', t: 0,
      xm: opts.x0 != null ? opts.x0 : -0.8, zm: opts.z0 || 0, psi: opts.yaw0 || 0, vz: 0,
      ext: C.kneelExt, extWant: C.kneelExt, extTarget: C.kneelExt, extCmd: null, h: 0, share: 0, liftClearM: 0,
      lid: 0, squeeze: 0, FA: 0, FB: 0, locked: false, lockedAt: null,
      jam: false, blocked: false, events: [],
      liftN: 0, liftMps: 0, lidMoving: 0, clampA: 0, peakLiftN: 0, onCups: true, onStand: true,
      wh: { lift: 0, clamp: 0 }, lockMoving: 0, lockTarget: 0
    };
    function ev(msg) { S.events.push({ t: S.t, msg: msg }); }
    function centred() { return Math.abs(S.zm) <= C.centerTol && Math.abs(S.xm - C.seatX) <= C.axialTol; }

    /** One fixed step. cmd: { dx: rover move along its x this step (m), lift: 0|1 (kneel | carry ride), clamp: 0|1,
     *  ext: measured ride extension (m) if a real suspension runs it; sync: re-seed the ramp from it; without ext the ride follows the ramp itself } */
    function step(dt, cmd) {
      cmd = cmd || {};
      S.t += dt;
      var dx = cmd.dx || 0, wantLift = cmd.lift != null ? cmd.lift : 0, wantClamp = cmd.clamp != null ? cmd.clamp : 0;
      S.liftN = 0; S.liftMps = 0; S.lidMoving = 0;

      // ---- module position: it stays on the stand while the rover moves under it ----
      var onRover = S.h > C.contactH + 1e-6;
      if (!onRover && !S.locked) {
        S.xm -= dx;
        if (Math.abs(dx) > 0 && S.phase === 'STAGED' && Math.abs(S.xm - C.seatX) < 0.35) { S.phase = 'ENTERING'; ev('rover tail is under the module'); }
      } else if (Math.abs(dx) > 0.0015 && !S.locked) {
        S.blocked = true; ev('DRAG: the rover moved ' + (dx * 1000).toFixed(1) + ' mm while the saddles were taking the module');
      }

      // ---- ride height wanted: kneel or carry, with the interlocks ----
      var hWant = wantLift ? C.liftMaxH : 0;
      if (wantLift && S.lid > 0 && S.lid < 1) hWant = S.h;                       // never move while a jaw is mid-swing
      if (!wantLift && (S.lid > 0 || S.squeeze > 0)) hWant = Math.max(S.h, C.liftMaxH);   // never lower with the jaws shut: open first
      if (wantLift && S.h < C.contactH && S.h + C.liftV * dt >= C.contactH - 1e-9) {
        if (Math.abs(S.zm) > C.captureZ || Math.abs(S.psi) > C.captureYaw || Math.abs(S.xm - C.seatX) > C.captureX) {
          if (!S.jam) { S.jam = true; ev('JAM: saddle met the module off its band (axial ' + ((S.xm - C.seatX) * 1000).toFixed(0) + ' mm, side ' + (S.zm * 1000).toFixed(0) + ' mm, yaw ' + (S.psi * 57.3).toFixed(1) + ' deg)'); }
        }
      }
      if (S.jam) hWant = Math.min(S.h, C.contactH - 0.002);
      S.extTarget = C.kneelExt + hWant; S.extWant = S.extTarget;

      // ---- ride: a commanded ramp (fast, but only 15 mm/s through the 4 mm where the weight changes hands); the real suspension follows it ----
      var h0 = S.h;
      if (S.extCmd == null || cmd.sync) S.extCmd = cmd.ext != null ? cmd.ext : S.ext;
      var inXfer = S.extCmd > C.contactExt - 0.003 && S.extCmd < C.contactExt + C.shareTravel + 0.002;
      var spd = inXfer ? C.liftXferV : C.liftV;
      S.extCmd += clamp(S.extTarget - S.extCmd, -spd * dt, spd * dt);
      S.extWant = S.extCmd;                                                     // what the caller should ask the struts for (a ramp, not a jump)
      S.ext = cmd.ext != null ? cmd.ext : S.extCmd;
      S.h = S.ext - C.kneelExt;
      var dh = S.h - h0;
      S.share = clamp((S.h - C.contactH) / C.shareTravel, 0, 1);                 // weight moves to the rover over 4 mm of strut travel
      S.liftClearM = Math.max(0, S.h - C.contactH - C.shareTravel);
      if (Math.abs(dh) > 1e-9) {
        S.liftMps = Math.abs(dh) / dt;
        S.liftN = MODULE.kg * g * S.share * (dh > 0 ? 1 : 0.2) + 1.0;            // module share on the struts (valve bleed when lowering)
        S.peakLiftN = Math.max(S.peakLiftN, S.liftN);
        S.wh.lift += (S.liftN * S.liftMps / C.liftEta) * dt / 3600;
      }
      // ---- V saddle centring once it carries the module (sideways and yaw; the rover is braked, so x holds) ----
      if (S.h > C.contactH + 1e-6 && !S.locked) {
        var k = 900 * S.share + 60, c2 = 2 * Math.sqrt(k), NS = 8, hs = dt / NS;
        for (var q = 0; q < NS; q++) { S.vz += (-k * S.zm - c2 * S.vz) * hs; S.zm += S.vz * hs; S.psi += (-k * 2 * S.psi) * hs * 0.05; }
        if (S.phase !== 'LOCKED' && S.h >= C.liftMaxH - C.topTol) S.phase = 'SEATED';
      }
      S.onCups = S.onStand = S.share < 1;

      // ---- jaws: rotate up once seated, then the pads squeeze to the force limit ----
      var seated = S.h >= C.liftMaxH - C.topTol;
      var wantClose = wantClamp && seated && centred() && !S.jam;
      if (wantClamp && seated && Math.abs(S.xm - C.seatX) > C.axialTol && !S.blocked) { S.blocked = true; ev('CLAMP BLOCKED: clamp band is ' + ((S.xm - C.seatX) * 1000).toFixed(1) + ' mm off the rings'); }
      var motors = 0;
      if (wantClose) {
        S.lockTarget = 1;
        if (S.lid < 1) { S.lid = Math.min(1, S.lid + dt / C.lidCloseS); motors = 2; }
        else if (S.squeeze < 1) { S.squeeze = Math.min(1, S.squeeze + C.padVsq * dt / (C.clampMaxN / C.padKpad)); motors = 2; }
        S.FA = S.FB = C.clampMaxN * S.squeeze;
        S.clampA = motors ? C.motorI0 + S.FA / C.clampKF : 0;                   // motors off (locked, or fully open): no current at all, the worm holds
        if (!S.locked && S.squeeze >= 1) {
          S.locked = true; S.lockedAt = S.t; S.phase = 'LOCKED'; S.zm = 0; S.vz = 0; S.psi = 0;
          ev('both rings clamped at ' + S.FA.toFixed(0) + ' N each; the worm drives hold it with the motors off');
        }
      } else if (!wantClamp && (S.lid > 0 || S.squeeze > 0)) {
        S.lockTarget = 0;
        if (S.squeeze > 0) { S.squeeze = Math.max(0, S.squeeze - C.padVsq * dt / (C.clampMaxN / C.padKpad)); motors = 2; }
        else if (S.lid > 0) { S.lid = Math.max(0, S.lid - dt / C.lidCloseS); motors = 2; }
        S.FA = S.FB = C.clampMaxN * S.squeeze;
        S.clampA = motors ? C.motorI0 + S.FA / C.clampKF : 0;                   // motors off (locked, or fully open): no current at all, the worm holds
        if (S.locked && S.squeeze <= 0) { S.locked = false; S.phase = 'SEATED'; ev('pads released'); }
      } else { S.clampA = 0; }
      if (!S.locked && S.lid <= 0 && S.h <= C.contactH && S.phase !== 'STAGED' && S.phase !== 'ENTERING') S.phase = 'RELEASED';
      S.lidMoving = motors; S.lockMoving = motors;
      S.holdW = motors ? S.clampA * C.motorV * 2 : (S.locked && !WORM.selfLocking ? C.motorI0 * C.motorV * 2 : 0);   // locked with the motors off: the worm holds, 0 W (a drive that could back-drive would need its holding current)
      if (motors) S.wh.clamp += (S.clampA * C.motorV * 2) * dt / 3600;
      S.winchN = S.liftN; S.winchMps = S.liftMps;                                 // power-model names for the lift
      return S;
    }

    var api = {
      state: S, step: step, MODULE: MODULE, CRADLE: CRADLE,
      /** axial (along the module axis) friction capacity of both rings, N */
      axialCapacityN: function () { return C.muPad * 2 * (S.FA + S.FB); },
      /** how many times the clamp covers an axial load (braking / slope along the rover), dimensionless */
      holdMargin: function (aAx, slopeRad) { var need = MODULE.kg * (Math.abs(aAx) + g * Math.sin(Math.abs(slopeRad || 0))); return need > 1e-6 ? api.axialCapacityN() / need : 99; },
      /** hold margin under the hardest permitted braking (CRADLE.aBrakeMax) on a slope: friction capacity / (m (a + g sin slope)) */
      holdMarginBraking: function (slopeRad) { return api.holdMargin(C.aBrakeMax, slopeRad); },
      /** the system CoM (rover + the module share it carries) as the weight moves over the strut travel; rover: { kg, x, y, z } in the rover frame */
      systemCom: function (rover) {
        var m = rover.kg, mm = MODULE.kg * S.share, e = S.ext;
        var ym = C.axisY + e, yr = rover.y + e;
        return { kg: m + mm, x: (rover.x * m + S.xm * mm) / (m + mm), y: (yr * m + ym * mm) / (m + mm), z: (rover.z * m + S.zm * mm) / (m + mm), share: S.share };
      },
      /** share of the module weight on the rover (the stand carries the rest) */
      shareOnRover: function () { return S.share; },
      /** point masses for the dynamics model (rover frame: x fwd, y up, z port) */
      cargoPoints: function () {
        if (S.share <= 0) return [];
        return [{ x: S.xm, y: C.axisY, z: S.zm, kg: MODULE.kg * S.share }];
      },
      /** the load path while carried: weight on the two saddles, into the deck and the air struts; pad force is internal */
      loadPath: function () { var w = MODULE.kg * g * S.share; return { saddleEachN: w / 2, padEachN: S.FA, chassisN: w }; }
    };
    return api;
  }

  /** Static CoM / tip numbers for the loaded rover (rover masses and CoM from NASA_RoverBOM or given). */
  function loadedBalance(rover, halfTrack, halfBase, ext) {
    var m = rover.kg, mm = MODULE.kg, e = ext != null ? ext : CRADLE.carryExt;
    var ym = CRADLE.axisY + e, yr = rover.y + e;
    var x = (rover.x * m + CRADLE.seatX * mm) / (m + mm);
    var y = (yr * m + ym * mm) / (m + mm);
    var z = (rover.z * m) / (m + mm);
    return { massKg: m + mm, x: x, y: y, z: z,
      rollTip: function (g) { return (halfTrack - Math.abs(z)) / y * g; },
      pitchTip: function (g) { return (halfBase - Math.abs(x)) / y * g; } };
  }

  var api = { MODULE: MODULE, CRADLE: CRADLE, WORM: WORM, create: create, loadedBalance: loadedBalance };
  root.NASA_ModuleCarry = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
