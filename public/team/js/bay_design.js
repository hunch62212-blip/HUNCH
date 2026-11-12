/* Regolith-proof loading and unloading bays (rover v13).
 *
 * Lunar regolith is sharp, electrostatically clinging and everywhere the rover has been. The v12 bays ignored that: the depot stand stood on bare
 * soil and the storage hangar opened its whole front (all four bays at once) by rolling up a slat door in the open, so every stored module was
 * exposed to whatever the transfer kicked up. v13 treats the dust as the design driver.
 *
 *  PICKUP (landing-zone stand)
 *    sintered-regolith apron under and around the stand (the soil cannot be kicked up from a fused plate), a lead-in funnel that squares the rover to the
 *    stand axis, skirts on the stand posts, and boots over the saddle rollers until the module is actually seated. The clamp's friction pads are the thing
 *    that must stay clean, so the model follows the dust from the approach to the pads and on to the holding margin of the ring clamps.
 *  RECEIVING (storage hangar)
 *    a dust lock in front of the bays. The module goes in through the OUTER door, the door closes behind it, brush rollers and an electrodynamic pulse clean
 *    it until the residue is under the budget, and only then does the INNER door of the one target bay open. The two doors are interlocked: they are never
 *    open together, and a lock that cannot clean the module refuses to open inward and says so. Each door is a pair of rigid leaves on a covered track with
 *    brush seals, driven by jerk-limited moves (js/motion.js SCurve).
 *
 * Dust masses use ASSUMED coefficients (marked) because no regolith-simulant test has been run: the structure, the interlock and the sequencing are
 * checked in tools/bay_design_test.js, the coefficients need a simulant bench test (JSC-1A or similar) before they are numbers to design to.
 * Pure logic. Used by js/module_sortie.js (sequence) and js/module_models.js (geometry numbers), tools/bay_design_test.js.
 */
(function (root) {
  'use strict';
  var MO = root.NASA_Motion || (typeof require !== 'undefined' ? require('./motion.js') : null);
  var MC = root.NASA_ModuleCarry || (typeof require !== 'undefined' ? require('./module_carry.js') : null);
  var MOD = MC.MODULE, SC = MO.SCurve;

  // ------------------------------------------------------------------------------------------------ geometry and limits (model scale, 1:12)
  var G = {
    bays: 4, pitch: 0.86, depth: 0.98, H: 0.82, header: 0.12,           // pitch 0.86: the lifting yoke is 0.78 m across the hooks, so a bay needs 0.80 m clear
    lockDepth: 0.86,                     // the dust lock: long enough to hold the 0.748 m module with 55 mm either end
    outerGap: 0.06,                      // module end to the outer door while it still stands on the depot stand
    leafW: 0.40, leafTravel: 0.40,       // each door is two leaves meeting on the centreline; they slide into side pockets in the posts
    openW: 0.80, openH: 0.70,            // clear opening of the outer door and of every inner door (pins 0.72 m, yoke 0.78 m, module plus yoke 0.625 m tall at the lift)
    brushRoll: { r: 0.022, len: 0.60, n: 3 },     // two side rollers and one top roller inside the lock
    brushSweepM: 0.62, brushRpm: 120,
    sealBrushMm: 12                      // brush seal bristle length on every leaf edge
  };
  /* v14 lug-pin interface: the stand and the yoke share each pin WITHOUT sharing any of it.
       pin (trunnion) runs from the hull (z = R) out to z = lugZ + 14 mm. The stand post (30 mm square, its V notch on top) holds the OUTBOARD span.
       The yoke holds the INBOARD span with a same-handed C-hook: a vertical arm 34 mm off the pin in x (so it never meets the pin), a foot that
       reaches under the pin in -x. The yoke lowers with the feet 45 mm to the side of the pins, slides -x to put them under, then lifts.
       Releasing is the reverse (lower, slide +x, rise until the feet clear the pin tops, only then travel). */
  var PIN = { r: MOD.lugR, z0: MOD.R, z1: MOD.lugZ + 0.014 };
  var YK = {
    armZ: 0.290, armT: 0.024, armDX: 0.034, armW: 0.016,                 // arm centre z, its z thickness, x offset from the pin axis, x thickness
    hookZ: [0.268, 0.318], footT: 0.018, footReach: 0.020,               // foot spans z 0.268..0.318 under the pin, 18 mm thick, reaches 20 mm past the pin axis
    engage: 0.045,                                                       // x shift between "beside the pins" and "under the pins"
    hookClear: 0.006
  };
  YK.riseClear = YK.hookClear + YK.footT + 2 * PIN.r + 0.010;             // rise from hook height before any travel: foot top 10 mm over the pin top
  var POST = { z: [MOD.lugZ - 0.015, MOD.lugZ + 0.015] };                  // the stand post (and its notch) under the pin
  function pinInterface() {
    var gapHookPost = POST.z[0] - YK.hookZ[1];
    var armHull = (YK.armZ - YK.armT / 2) - MOD.R;                         // worst case at the equator: the hull is widest there
    var armPinX = YK.armDX - YK.armW / 2 - PIN.r;                          // arm face to pin surface in x
    var footEnd = YK.armDX - YK.engage - YK.footReach;                     // foot tip x (from the pin axis) while lowering beside the pins
    return {
      bearingM: YK.hookZ[1] - YK.hookZ[0], notchM: POST.z[1] - POST.z[0], gapHookPostM: gapHookPost, hookOnPin: YK.hookZ[0] >= PIN.z0 && YK.hookZ[1] <= PIN.z1 && MOD.lugZ + 0.006 <= PIN.z1,   // hook and notch horns (12 mm) both land on the pin
      armHullM: armHull, armPinXM: armPinX, lowerClearM: (YK.armDX + YK.engage - YK.footReach - YK.armDX) - PIN.r, yokeHalfW: YK.armZ + YK.armT / 2,
      disjoint: YK.hookZ[1] < POST.z[0]
    };
  }
  var LIM = {
    door: { vmax: 0.06, amax: 0.04, jmax: 0.15 },          // leaf drive: a worm-geared motor, slow and gentle (m/s, m/s2, m/s3)
    brush: { vmax: 0.05, amax: 0.04, jmax: 0.20 },         // brush carriage sweep along the module
    yokeLift: { vmax: 0.02, amax: 0.02, jmax: 0.08 },
    yokeLateral: { vmax: 0.10, amax: 0.08, jmax: 0.40 },
    yokeSlide: { vmax: 0.09, amax: 0.07, jmax: 0.35 },
    yokeSlideIn: { vmax: 0.05, amax: 0.04, jmax: 0.20 },   // through the door frame the module moves slower
    yokeLower: { vmax: 0.02, amax: 0.02, jmax: 0.08 },
    yokeRetract: { vmax: 0.22, amax: 0.12, jmax: 0.6 },
    kneel: { vmax: 0.03, amax: 0.03, jmax: 0.10 }
  };

  // ------------------------------------------------------------------------------------------------ dust budget (ASSUMED coefficients, flagged)
  var DUST = {
    moduleAreaM2: Math.PI * MOD.D * MOD.L + 2 * Math.PI * MOD.R * MOD.R,
    exposedFrac: 0.15, exposedLoadGm2: 8.0, restLoadGm2: 0.3,     // ASSUMED: the underside and lug end meet the wheel plume; the rest only settles
    brushPassEff: 0.88,                                            // ASSUMED: one pass of a conductive PEEK/phosphor-bronze bristle roller on a cylinder
    edsEff: 0.75,                                                  // ASSUMED: electrodynamic pulse train on a dielectric skin
    sealLeakMg: 0.5,                                               // ASSUMED: brush-seal leakage per door opening
    targetMg: 5.0,                                                 // requirement: residue carried into a bay per transfer
    maxPasses: 4,
    // v14: the four lug pins were left out of the budget, and the side rollers cannot reach them. Each pin now has a split cup brush on the
    // lock cradle that runs out along the pin and back during every roller pass (one pass = one stroke out and back).
    pinAreaM2: 4 * (2 * Math.PI * MOD.lugR * (MOD.lugZ + 0.014 - MOD.R) + Math.PI * MOD.lugR * MOD.lugR),
    pinLoadGm2: 8.0,                                               // ASSUMED: the pins sit in the exposed band (same as exposedLoadGm2)
    pinBrushEff: 0.85                                              // ASSUMED: a cup brush on a 20 mm pin, one stroke
  };
  function hullMassMg(o) { o = o || {}; var k = o.loadScale != null ? o.loadScale : 1; return k * (DUST.moduleAreaM2 * (DUST.exposedFrac * DUST.exposedLoadGm2 + (1 - DUST.exposedFrac) * DUST.restLoadGm2)) * 1000; }
  function pinMassMg(o) { o = o || {}; var k = o.loadScale != null ? o.loadScale : 1; return k * DUST.pinAreaM2 * DUST.pinLoadGm2 * 1000; }
  function arrivalMassMg(o) { return hullMassMg(o) + pinMassMg(o); }
  /** residue (mg) after a number of brush passes and an optional electrodynamic pulse; hull and pins are cleaned by different brushes */
  function residueMg(passes, eds, o) {
    o = o || {}; var eff = o.brushEff != null ? o.brushEff : DUST.brushPassEff, e = o.edsEff != null ? o.edsEff : DUST.edsEff;
    var pe = o.pinBrush === false ? 0 : (o.pinBrushEff != null ? o.pinBrushEff : DUST.pinBrushEff);
    return (hullMassMg(o) * Math.pow(1 - eff, passes) + pinMassMg(o) * Math.pow(1 - pe, passes)) * (eds ? 1 - e : 1);
  }
  /* Uncertainty (v14). Every coefficient above is ASSUMED, so the design is checked over a band around each one, not at the point value:
     log-uniform x0.5..x2 on the arriving dust, uniform brush 0.70..0.95, pin brush 0.60..0.95, pulse 0.50..0.90 (seeded, reproducible).
     Reports how often the lock cleans within its pass limit, the worst residue it ever lets in (must be <= budget, every time: a lock that
     cannot clean faults instead), and the break-even brush efficiency a simulant bench test has to show. */
  function dustMonteCarlo(n, seed) {
    var sd = seed || 12345; function rnd() { sd = (sd * 16807) % 2147483647; return sd / 2147483647; }
    var ok = 0, worstAdmitted = 0, worstNeed = 0, faults = 0;
    for (var i = 0; i < n; i++) {
      var o = { loadScale: Math.pow(2, 2 * rnd() - 1), brushEff: 0.70 + 0.25 * rnd(), pinBrushEff: 0.60 + 0.35 * rnd(), edsEff: 0.50 + 0.40 * rnd() };
      var need = passesNeeded(true, o);
      if (isFinite(need) && need <= DUST.maxPasses) { ok++; worstAdmitted = Math.max(worstAdmitted, residueMg(need, true, o)); worstNeed = Math.max(worstNeed, need); }
      else faults++;
    }
    return { n: n, cleanFrac: ok / n, faultFrac: faults / n, worstAdmittedMg: worstAdmitted, worstPasses: worstNeed };
  }
  /** lowest single-pass brush efficiency (hull rollers and pin brushes alike) at which the nominal module still cleans within maxPasses */
  function breakEvenBrushEff(o) {
    o = o || {}; var lo = 0, hi = 1;
    for (var k = 0; k < 40; k++) { var m = (lo + hi) / 2, q = { brushEff: m, pinBrushEff: m, edsEff: o.edsEff, loadScale: o.loadScale }; var nd = passesNeeded(true, q); if (isFinite(nd) && nd <= DUST.maxPasses) hi = m; else lo = m; }
    return hi;
  }
  /** passes needed to get under the target; Infinity if the brushes are dead */
  function passesNeeded(eds, o) {
    var t = (o && o.targetMg) || DUST.targetMg, eff = o && o.brushEff != null ? o.brushEff : DUST.brushPassEff;
    if (eff <= 0) return residueMg(0, eds, o) <= t ? 0 : Infinity;
    if (o && o.pinBrushEff != null && o.pinBrushEff <= 0 && residueMg(12, eds, o) > t) return Infinity;
    for (var p = 0; p <= 12; p++) if (residueMg(p, eds, o) <= t) return p; return Infinity;
  }

  // Pickup: dust that reaches the clamp's friction pads on the approach, and what it does to the holding margin
  var PICKUP = {
    kickGm2: 20.0,                       // ASSUMED: loading on the interface after a 0.25 m/s approach over loose, un-fused regolith
    apron: 0.10, funnel: 0.30, boots: 0.20,      // ASSUMED fractions that get through each barrier
    frictionL0Gm2: 5.0                   // ASSUMED: pad friction halves at this loading (mu_eff = mu / (1 + load / L0))
  };
  function pickupLoading(o) {
    o = o || {}; var l = PICKUP.kickGm2;
    if (o.apron !== false) l *= PICKUP.apron; if (o.funnel !== false) l *= PICKUP.funnel; if (o.boots !== false) l *= PICKUP.boots;
    return l;
  }
  function frictionDerate(loadGm2) { return 1 / (1 + loadGm2 / PICKUP.frictionL0Gm2); }

  // ------------------------------------------------------------------------------------------------ receiving bay: state machine with interlock
  /** doors report an opening fraction 0..1; the machine never lets the outer and inner doors be open together */
  function createReceivingBay(o) {
    o = o || {};
    var st = {
      phase: 'IDLE', t: 0, bay: -1, fault: null, cycles: 0, log: [],
      outer: { x: 0 }, inner: [], brush: { x: 0, rpm: 0, passes: 0 }, eds: { on: false, pulses: 0 },
      residueMg: 0, ingressMg: 0, moduleIn: false, transferred: false, sealed: true, maxBoth: 0
    };
    var i; for (i = 0; i < G.bays; i++) st.inner.push({ x: 0 });
    var mvO = new MO.Move(LIM.door), mvI = new MO.Move(LIM.door), mvB = new MO.Move(LIM.brush), rpmMv = new MO.Move({ vmax: 400, amax: 200, jmax: 800 });
    var brushEff = o.brushEff != null ? o.brushEff : DUST.brushPassEff, edsOn = o.eds !== false, edsEff = o.edsEff != null ? o.edsEff : DUST.edsEff;
    var need = 0, cleaned = 0, spun = false, sweepDir = 1, seal = 0;
    function open(x) { return Math.abs(x) / G.leafTravel; }                 // 0 closed .. 1 open
    function say(m) { st.log.push({ t: st.t, phase: st.phase, msg: m }); }
    function go(p, m) { st.phase = p; st.t = 0; if (m) say(m); }
    function anyInnerOpen() { return st.inner.some(function (d) { return d.x > 0.002; }); }

    var api = {
      state: st, G: G,
      /** ask for a transfer into bay k; refused while busy, faulted, or when the bay is out of range */
      request: function (k) {
        if (st.phase !== 'IDLE' || st.fault || k < 0 || k >= G.bays) return false;
        st.bay = k; st.moduleIn = false; st.transferred = false; st.ingressMg = 0; need = 0; cleaned = 0; spun = false; seal = 0; st.brush.passes = 0;
        if (anyInnerOpen()) { st.fault = 'INTERLOCK: an inner door is not closed'; return false; }
        mvO.start(st.outer.x, G.leafTravel); go('OUTER_OPEN', 'outer door opening (inner doors confirmed shut)');
        return true;
      },
      /** the yoke reports the module is fully inside the lock */
      moduleAdmitted: function () { st.moduleIn = true; },
      /** the yoke reports it has put the module on the bay cradle and backed out into the lock */
      moduleTransferred: function () { st.transferred = true; },
      /** the outer leaves are clear of the module's path */
      outerClear: function () { return st.phase === 'ADMIT' || (st.phase === 'OUTER_OPEN' && open(st.outer.x) >= 0.999); },
      innerClear: function () { return st.phase === 'TRANSFER' && open(st.inner[st.bay].x) >= 0.999; },
      readyToAdmit: function () { return st.phase === 'ADMIT'; },
      readyToCross: function () { return st.phase === 'CROSS'; },
      readyToTransfer: function () { return st.phase === 'TRANSFER'; },
      busy: function () { return st.phase !== 'IDLE'; },
      step: function (dt) {
        st.t += dt;
        mvO.step(dt); mvI.step(dt); mvB.step(dt); rpmMv.step(dt);
        st.outer.x = mvO.pos; if (st.bay >= 0) st.inner[st.bay].x = mvI.pos;
        st.brush.rpm = rpmMv.pos; st.brush.x = mvB.pos;
        var both = Math.min(open(st.outer.x), st.bay >= 0 ? open(st.inner[st.bay].x) : 0);
        st.maxBoth = Math.max(st.maxBoth, both);
        switch (st.phase) {
          case 'OUTER_OPEN':
            if (mvO.done) go('ADMIT', 'outer door fully open: the yoke may carry the module in');
            break;
          case 'ADMIT':
            if (st.moduleIn) { mvO.start(st.outer.x, 0); go('OUTER_CLOSE', 'module inside: outer door closing behind it'); }
            break;
          case 'OUTER_CLOSE':
            if (mvO.done) { go('SEAL_CHECK', 'outer door closed'); seal = 0; }
            break;
          case 'SEAL_CHECK':
            seal += dt; st.sealed = true;
            if (seal >= 1.0) {
              need = passesNeeded(edsOn, { brushEff: brushEff, edsEff: edsEff });
              st.residueMg = residueMg(0, false, { brushEff: brushEff });
              if (!isFinite(need) || need > DUST.maxPasses) { st.fault = 'LOCK CANNOT CLEAN: ' + (need === Infinity ? 'brushes not removing dust' : need + ' passes needed, ' + DUST.maxPasses + ' allowed') + '; the module stays in the lock and the inner door stays shut'; go('FAULT', st.fault); break; }
              rpmMv.start(0, G.brushRpm); go('SPIN_UP', 'lock closed and seal confirmed; dust ' + st.residueMg.toFixed(0) + ' mg on the module, ' + need + ' brush pass(es) needed' + (edsOn ? ' plus the electrodynamic pulse' : ''));
            }
            break;
          case 'SPIN_UP':
            if (rpmMv.done) { cleaned = 0; if (need === 0) go('PULSE', 'no brush pass needed'); else { sweepDir = 1; mvB.start(0, G.brushSweepM); go('BRUSH', 'brush pass 1 of ' + need); } }
            break;
          case 'BRUSH':
            if (mvB.done) {
              cleaned++; st.brush.passes = cleaned; st.residueMg = residueMg(cleaned, false, { brushEff: brushEff });
              if (cleaned >= need) { rpmMv.start(st.brush.rpm, 0); go('SPIN_DOWN', 'brush passes done, residue ' + st.residueMg.toFixed(1) + ' mg'); }
              else { sweepDir *= -1; mvB.start(st.brush.x, sweepDir > 0 ? G.brushSweepM : 0); go('BRUSH', 'brush pass ' + (cleaned + 1) + ' of ' + need); }
            }
            break;
          case 'SPIN_DOWN':
            if (rpmMv.done) { if (edsOn) { st.eds.on = true; st.eds.pulses = 0; go('PULSE', 'electrodynamic pulse train'); } else go('CLEAN_OK'); }
            break;
          case 'PULSE':
            if (!st.eds.on) { st.eds.on = true; st.eds.pulses = 0; }
            if (st.t >= 3.0) { st.eds.on = false; st.eds.pulses = 3; go('CLEAN_OK'); }
            break;
          case 'CLEAN_OK':
            st.residueMg = residueMg(cleaned, edsOn, { brushEff: brushEff, edsEff: edsEff }) + DUST.sealLeakMg;
            st.ingressMg = st.residueMg;
            if (st.residueMg > DUST.targetMg + DUST.sealLeakMg + 1e-9) { st.fault = 'residue ' + st.residueMg.toFixed(1) + ' mg is over the ' + DUST.targetMg + ' mg budget'; go('FAULT', st.fault); break; }
            go('CROSS', 'module clean: ' + st.residueMg.toFixed(1) + ' mg expected to enter the bay (budget ' + DUST.targetMg + ' mg + seal leakage)');
            break;
          case 'CROSS':                                                  // the yoke moves the module sideways to the target bay; nothing to open yet
            if (st.crossed) { st.crossed = false; mvI.start(st.inner[st.bay].x, G.leafTravel); go('INNER_OPEN', 'inner door of bay ' + (st.bay + 1) + ' opening (outer door confirmed shut)'); }
            break;
          case 'INNER_OPEN':
            if (open(st.outer.x) > 0.002) { st.fault = 'INTERLOCK: outer door not shut while the inner door opens'; go('FAULT', st.fault); break; }
            if (mvI.done) go('TRANSFER', 'inner door open: the yoke may carry the module into the bay');
            break;
          case 'TRANSFER':
            if (st.transferred) { mvI.start(st.inner[st.bay].x, 0); go('INNER_CLOSE', 'module on its cradle and the yoke clear: inner door closing'); }
            break;
          case 'INNER_CLOSE':
            if (mvI.done) { st.cycles++; go('IDLE', 'bay ' + (st.bay + 1) + ' sealed again; cycle ' + st.cycles + ' complete'); }
            break;
          case 'FAULT': break;
        }
        return st.phase;
      },
      crossed: function () { st.crossed = true; },
      /** operator clears a fault: the lock opens its outer door so the module can be taken back out */
      clearFault: function () { if (st.phase === 'FAULT') { st.fault = null; st.moduleIn = false; mvO.start(st.outer.x, G.leafTravel); go('OUTER_OPEN', 'fault cleared: outer door opening to return the module'); } }
    };
    return api;
  }

  var api = { G: G, LIM: LIM, DUST: DUST, PICKUP: PICKUP, PIN: PIN, YOKE: YK, POST: POST, pinInterface: pinInterface, dustMonteCarlo: dustMonteCarlo, breakEvenBrushEff: breakEvenBrushEff, hullMassMg: hullMassMg, pinMassMg: pinMassMg, arrivalMassMg: arrivalMassMg, residueMg: residueMg, passesNeeded: passesNeeded, pickupLoading: pickupLoading, frictionDerate: frictionDerate, createReceivingBay: createReceivingBay };
  root.NASA_BayDesign = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
