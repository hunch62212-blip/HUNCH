/* LiDAR mounting (rover v13): where the scanners sit, what they can see, and whether anything has to move.
 *
 * v12 carried one scanner on a telescoping mast at the front starboard corner. With a module aboard the stowed head sat inside the module's
 * outline (axis 0.562 m, radius 0.244 m, mast 0.275 m off the centreline), so the mast had to be deployed before every load and unload. That
 * is a 19 s motion that exists only because of where the sensor was put.
 *
 * v13 moves the sensing below the cargo: two fixed scanners on the deck corners, on a diagonal, head centre 0.262 m above the ground. The module's
 * lowest surface is 0.318 m above the ground in the rover frame (seated), 0.35 m while the rover kneels under it, so a head whose top is 0.287 m
 * passes under it with 31 mm to spare in every phase of loading and unloading. Nothing moves, so nothing can foul the module, the stand posts or
 * the hangar door, and a single failed scanner leaves the other one working.
 *
 * Everything here is geometry in the rover root frame (+x forward, +y up, +z port, ground y = 0) and ray casting against that geometry, so the
 * choice is checked, not asserted:
 *   solids(state)            the rover's own hard parts (pod, saddle posts, jaw drives, wheels, deck) and, when aboard, the module
 *   selfHit(unit, d, t, st)  first own-structure hit along a ray, used by the simulator to throw away returns that are the rover itself
 *   coverage(unit, st, opt)  which azimuths a layer can use, as arcs, and the largest blind arc
 *   candidates()/study()     the mounting options that were compared, with the numbers behind the choice
 * Pure logic. Used by js/fusion_rover.js (model), viewer.html (scan origin and self filter), tools/lidar_mount_test.js.
 */
(function (root) {
  'use strict';
  var MC = root.NASA_ModuleCarry || (typeof require !== 'undefined' ? require('./module_carry.js') : null);
  var RB = root.NASA_RockerBogie || (typeof require !== 'undefined' ? require('./rocker_bogie.js') : null);
  var MOD = MC.MODULE, CR = MC.CRADLE, G = RB.GEO;

  var DECK_Y = 0.225;
  var HEAD = { r: 0.0303, h: 0.050, massKg: 0.19, w: 2.0, rangeM: 8.0 };         // YDLIDAR X2L (js/rover_bom.js): 60.5 mm across, 50 mm tall
  var PLINTH_H = 0.012;                                                         // bolted spigot between the deck and the head
  var HEAD_Y = DECK_Y + PLINTH_H + HEAD.h / 2;                                  // optical centre, 0.262 m
  // the two scanners (rover frame): both at the front, port and starboard (the rear scanner could not track anything useful). Same plinth, mirrored.
  var UNITS = [
    { id: 'FP', x: 0.250, y: HEAD_Y, z: 0.150, label: 'front port' },
    { id: 'FS', x: 0.250, y: HEAD_Y, z: -0.150, label: 'front starboard' }
  ];
  // layer tilt: ground rings at 2, 4 and 8 m whatever the head height is (the old fixed pitches assumed a 0.55 m head)
  var RING_M = [8.0, 4.0, 2.0];
  function pitches(h) { return RING_M.map(function (d) { return -Math.atan(h / d); }); }

  // ------------------------------------------------------------------------------------------------ own structure
  function box(x0, x1, y0, y1, z0, z1, tag) { return { k: 'box', x0: x0, x1: x1, y0: y0, y1: y1, z0: z0, z1: z1, tag: tag }; }
  var STATIC = (function () {
    var s = [];
    s.push(box(0.1125, 0.2675, DECK_Y, DECK_Y + 0.055, -0.100, 0.100, 'avionics pod'));
    s.push(box(-0.29, 0.29, DECK_Y - 0.014, DECK_Y, -0.19, 0.19, 'deck'));
    s.push(box(-0.27, -0.23, DECK_Y, DECK_Y + 0.030, 0.13, 0.17, 'E-stop'));
    [1, -1].forEach(function (sg) {
      var rx = sg * CR.ringX;
      [0, 0.15, -0.15].forEach(function (zp) { s.push(box(rx - 0.010, rx + 0.010, DECK_Y, 0.34, zp - 0.010, zp + 0.010, 'saddle post')); });
      s.push(box(rx - 0.013, rx + 0.013, DECK_Y, DECK_Y + 0.006, -0.165, 0.165, 'foot rail'));
      [0.07, -0.07].forEach(function (zj) { s.push(box(rx - sg * 0.012 - 0.015, rx - sg * 0.012 + 0.015, 0.271, 0.297, zj - 0.015, zj + 0.015, 'jaw drive')); });
    });
    // wheels as horizontal cylinders along z
    [G.xF, G.xM, G.xR].forEach(function (wx) { [1, -1].forEach(function (sg) { s.push({ k: 'cylz', cx: wx, cy: G.axleY, cz: sg * G.zW, r: 0.065, hw: 0.0375, tag: 'wheel' }); }); });
    return s;
  })();

  /** the rover's hard parts, plus the module (cylinder along x) when state.module = { x, y, z, psi } is given in the rover frame */
  function solids(st) {
    st = st || {};
    var out = STATIC.slice();
    if (st.module) out.push({ k: 'cylx', cx: st.module.x, cy: st.module.y, cz: st.module.z, r: MOD.R, hl: MOD.L / 2, tag: 'module' });
    return out;
  }

  // ray = origin o, unit direction d (rover frame). Each returns the entry distance t >= 0 or Infinity.
  function rayBox(o, d, b) {
    var t0 = 0, t1 = Infinity, ax = ['x', 'y', 'z'], lo = [b.x0, b.y0, b.z0], hi = [b.x1, b.y1, b.z1], oo = [o.x, o.y, o.z], dd = [d.x, d.y, d.z];
    for (var i = 0; i < 3; i++) {
      if (Math.abs(dd[i]) < 1e-12) { if (oo[i] < lo[i] || oo[i] > hi[i]) return Infinity; continue; }
      var a = (lo[i] - oo[i]) / dd[i], c = (hi[i] - oo[i]) / dd[i]; if (a > c) { var q = a; a = c; c = q; }
      if (a > t0) t0 = a; if (c < t1) t1 = c; if (t0 > t1) return Infinity;
    }
    return t0;
  }
  function rayCyl(o, d, c, axis) {                       // infinite round cylinder about an axis ('x' or 'z') clipped to its length
    var u = axis === 'x' ? ['y', 'z'] : ['x', 'y'], ao = axis === 'x' ? 'x' : 'z';
    var ox = o[u[0]] - c['c' + u[0]], oy = o[u[1]] - c['c' + u[1]], dx = d[u[0]], dy = d[u[1]];
    var A = dx * dx + dy * dy, B = 2 * (ox * dx + oy * dy), C = ox * ox + oy * oy - c.r * c.r, half = c.hl != null ? c.hl : c.hw, ctr = c['c' + ao];
    if (A < 1e-12) return Infinity;
    var disc = B * B - 4 * A * C; if (disc < 0) return Infinity;
    var sq = Math.sqrt(disc), t0 = (-B - sq) / (2 * A), t1 = (-B + sq) / (2 * A);
    for (var k = 0; k < 2; k++) {
      var t = k === 0 ? t0 : t1; if (t < 0) continue;
      if (Math.abs(o[ao] + d[ao] * t - ctr) <= half) return t;
    }
    // ray that starts inside the radius and leaves through an end cap, or enters through one
    if (C < 0 && Math.abs(d[ao]) > 1e-12) { var te = ((ctr + (d[ao] > 0 ? half : -half)) - o[ao]) / d[ao]; return te >= 0 && te < Infinity ? 0 : Infinity; }
    return Infinity;
  }
  function hitSolid(o, d, s) { return s.k === 'box' ? rayBox(o, d, s) : (s.k === 'cylx' ? rayCyl(o, d, s, 'x') : rayCyl(o, d, s, 'z')); }

  /** first own-structure hit along a ray from a unit's head; Infinity if the ray leaves the rover clear. maxT clips. */
  function selfHit(unit, d, maxT, st, list) {
    var o = { x: unit.x, y: unit.y, z: unit.z }, best = maxT == null ? Infinity : maxT, L = list || solids(st), who = null;
    for (var i = 0; i < L.length; i++) {
      var s = L[i];
      // the unit's own head and plinth are not obstacles to itself
      var t = hitSolid(o, d, s); if (t < best && t > 0.004) { best = t; who = s.tag; }
    }
    return best === (maxT == null ? Infinity : maxT) && who == null ? Infinity : best;
  }
  function selfHitWho(unit, d, st, list) {
    var o = { x: unit.x, y: unit.y, z: unit.z }, best = Infinity, who = null, L = list || solids(st);
    for (var i = 0; i < L.length; i++) { var t = hitSolid(o, d, L[i]); if (t < best && t > 0.004) { best = t; who = L[i].tag; } }
    return { t: best, who: who };
  }

  /** usable azimuths of one layer: a ray is usable if it reaches the ground (or the 8 m range) without touching the rover first.
   *  Returns { usable: Uint8Array(360), arcs: [[a0,a1]...] degrees from +x towards +z, usableDeg, blindMaxDeg } */
  function coverage(unit, st, opt) {
    opt = opt || {}; var pitch = opt.pitch != null ? opt.pitch : pitches(unit.y)[2], list = solids(st), use = new Uint8Array(360), cp = Math.cos(pitch), sp = Math.sin(pitch);
    var tGround = unit.y / Math.max(1e-6, -sp);
    for (var a = 0; a < 360; a++) {
      var th = a * Math.PI / 180, d = { x: Math.cos(th) * cp, y: sp, z: Math.sin(th) * cp };
      var h = selfHit(unit, d, tGround, st, list);
      use[a] = h === Infinity || h >= tGround ? 1 : 0;
    }
    return summarise(use);
  }
  function summarise(use) {
    var n = use.length, usable = 0, arcs = [], i, start = -1;
    for (i = 0; i < n; i++) usable += use[i];
    // blind arcs (circular)
    var blind = [], run = 0, bestRun = 0, first = -1;
    var s0 = 0; while (s0 < n && !use[s0]) s0++;
    if (s0 === n) return { usable: use, usableDeg: 0, blindMaxDeg: 360, arcs: [], blind: [[0, 360]] };
    for (i = 0; i < n; i++) {
      var k = (s0 + i) % n;
      if (!use[k]) { if (run === 0) first = k; run++; } else { if (run) { blind.push([first, run]); if (run > bestRun) bestRun = run; run = 0; } }
    }
    if (run) { blind.push([first, run]); if (run > bestRun) bestRun = run; }
    return { usable: use, usableDeg: usable, blindMaxDeg: bestRun, blind: blind };
  }
  /** union of several units' usable sets (each in its own azimuth frame = the rover's; the units are only translated, so azimuth is comparable at range) */
  function unionCoverage(covs) {
    var use = new Uint8Array(360); covs.forEach(function (c) { for (var i = 0; i < 360; i++) if (c.usable[i]) use[i] = 1; });
    return summarise(use);
  }

  // ------------------------------------------------------------------------------------------------ the options that were compared
  var MODULE_ABOARD = { module: { x: CR.seatX, y: CR.axisY, z: 0 } };
  var MAST_V12 = { x: 0.255, z: -0.275, stowedY: 0.540, strokeM: 0.45, deploySeconds: 19.3 };
  /** Smallest gap (m) between a head and the module's outline in the cross-section, with the module allowed to arrive CR.captureZ sideways (the
   *  V saddle catches +-25 mm). Negative = the head is inside the module's swept volume. The head is a short vertical cylinder. */
  function moduleClearance(u) {
    var best = Infinity;
    for (var k = 0; k <= 8; k++) {
      var y = u.y - HEAD.h / 2 + HEAD.h * k / 8, dz = Math.max(0, Math.abs(u.z) - HEAD.r - CR.captureZ);
      best = Math.min(best, Math.hypot(y - CR.axisY, dz) - MOD.R);
    }
    return best;
  }
  function candidates() {
    var topSeated = CR.seatY, topKnelt = CR.seatY - CR.kneelExt - 0.0;          // module underside in the rover frame (seated, and while the rover kneels under a module on its stand)
    function clearUnder(headTop) { return +(topSeated - headTop).toFixed(4); }
    var C = [];
    C.push({ id: 'A', name: 'telescoping mast, front starboard (v12)', units: [{ id: 'M', x: MAST_V12.x, y: MAST_V12.stowedY, z: MAST_V12.z }], moving: 1, deployS: MAST_V12.deploySeconds, massKg: 0.19 + 0.110, powerW: 2.0 + 0.64,
      stowedHeadTop: MAST_V12.stowedY + HEAD.h / 2, note: 'stowed, the head is 1 mm off the module and inside its +-25 mm capture swing, and the module hides 150 deg of the view: it must be raised before every load and unload' });
    C.push({ id: 'B', name: 'fixed pole clear of the module top', units: [{ id: 'P', x: 0.255, y: MOD.R * 2 + CR.seatY + 0.06, z: -0.275 }], moving: 0, deployS: 0, massKg: 0.19 + 0.090, powerW: 2.0,
      stowedHeadTop: MOD.R * 2 + CR.seatY + 0.06 + HEAD.h / 2, note: 'needs a pole above the module crown: taller than the 24 in cube' });
    C.push({ id: 'C', name: 'single nose pod under the module overhang', units: [{ id: 'N', x: 0.300, y: HEAD_Y, z: 0 }], moving: 0, deployS: 0, massKg: 0.19, powerW: 2.0, stowedHeadTop: HEAD_Y + HEAD.h / 2, note: 'one sensor: blind to the rear and to the sides behind the pod' });
    C.push({ id: 'D', name: 'two fixed deck-corner scanners on a diagonal (chosen)', units: UNITS, moving: 0, deployS: 0, massKg: 0.38, powerW: 4.0, stowedHeadTop: HEAD_Y + HEAD.h / 2, note: 'no motion; either one still scans if the other fails' });
    return C.map(function (c) {
      var cov = c.units.map(function (u) { return coverage(u, MODULE_ABOARD, { pitch: pitches(Math.max(0.05, u.y))[2] }); });
      var un = unionCoverage(cov);
      var singleWorst = Math.min.apply(null, cov.map(function (v) { return v.usableDeg; }));
      var cubeOk = c.stowedHeadTop <= 0.6096 + 1e-9;
      var clear = clearUnder(c.stowedHeadTop);
      var mc = Math.min.apply(null, c.units.map(function (u) { return moduleClearance(u); }));
      return { id: c.id, name: c.name, note: c.note, units: c.units.length, movingParts: c.moving, deploySeconds: c.deployS, massKg: c.massKg, powerW: c.powerW,
        headTopM: +c.stowedHeadTop.toFixed(3), fitsCube: cubeOk, moduleUndersideClearanceM: clear, moduleClearanceM: +mc.toFixed(4), hitsModule: mc < 0,
        usableDegAboard: un.usableDeg, blindMaxDegAboard: un.blindMaxDeg, usableDegPerUnitMin: singleWorst };
    });
  }
  function study() {
    var c = candidates();
    c.forEach(function (r) {
      // hard constraints first: nothing may touch the module or leave the cube, nothing may need to move to make room for cargo
      r.feasible = r.fitsCube && !r.hitsModule && r.movingParts === 0;
      r.score = r.feasible ? r.usableDegAboard / 360 * 70 + (r.units >= 2 ? 20 : 0) + (r.massKg < 0.45 ? 10 : 0) : 0;
    });
    c.sort(function (a, b) { return b.score - a.score; });
    return c;
  }

  /* v14 FDIR: what the rover can still see with a scanner lost, and how fast it may then drive.
     In a sector no scanner covers, hazards are found only by the stereo cameras and the bumper ToF (ASSUMED 1.2 m detection range for a 0.10 m
     rock). The speed cap is the speed whose reaction distance plus braking distance fits inside that range less a 0.3 m margin:
     v * latency + v^2 / (2 * decel) <= range - margin. Both scanners lost: SAFE HOLD, nothing moves until an operator approves. */
  var CAM = { rangeM: 1.2, marginM: 0.3, latencyS: 0.3, decel: 0.25 };
  function capFor(rangeM) { var R = Math.max(0, rangeM - CAM.marginM), a = 1 / (2 * CAM.decel), b = CAM.latencyS; return (-b + Math.sqrt(b * b + 4 * a * R)) / (2 * a); }
  function lossPolicy(lost, st) {
    lost = lost || []; st = st || MODULE_ABOARD;
    var alive = UNITS.filter(function (u) { return lost.indexOf(u.id) < 0; });
    if (!alive.length) return { mode: 'SAFE_HOLD', alive: [], usableDeg: 0, fwdDeg: 0, vCap: 0, reason: 'no scanner left: stop, hold, operator to approve any motion' };
    var un = unionCoverage(alive.map(function (u) { return coverage(u, st); })), fwd = 0, i;
    for (i = 270; i < 450; i++) fwd += un.usable[i % 360];
    var blindSide = 0; for (i = 0; i < 360; i++) if (!un.usable[i]) blindSide += Math.sin(i * Math.PI / 180) > 0 ? 1 : -1;      // which side (+z / -z) the new blind arc is on
    var full = alive.length === UNITS.length;
    return { mode: full ? 'NOMINAL' : 'DEGRADED', alive: alive.map(function (u) { return u.id; }), usableDeg: un.usableDeg, fwdDeg: fwd, blindMaxDeg: un.blindMaxDeg,
      vCap: full ? Infinity : capFor(CAM.rangeM), turnAway: full ? 0 : (blindSide > 0 ? -1 : 1),
      reason: full ? 'both scanners' : 'one scanner: ' + un.usableDeg + ' deg seen; the blind flank relies on cameras and ToF, so speed is capped and turns go toward the seen side' };
  }
  /* v14 range on the Moon, not on the datasheet's white card. A time-of-flight / triangulation scanner's maximum range on a diffuse target goes
     as sqrt(reflectance x window transmission) (received power ~ rho T / r^2 at a fixed detection threshold). The X2L's 8 m is quoted on a bright
     target (ASSUMED 80 %); mare regolith reflects about 12 % in the near infrared (ASSUMED, lunar normal albedo at 0.8-0.9 um is 0.1-0.15) and the
     cover window loses transmission as dust settles on it. A hazard (a 0.20 m rock, 0.10 m tall) also needs two returns across it, which the
     angular step limits. The autonomy may only drive as fast as it can stop inside what it actually sees. */
  var OPT = { datasheetRangeM: HEAD.rangeM, datasheetRho: 0.80, regolithRho: 0.12, windowT: 0.85, stepDeg: 0.84, hazardD: 0.20, returns: 2 };
  function rangeOnMoon(o) {
    o = o || {}; var rho = o.rho != null ? o.rho : OPT.regolithRho, T = o.windowT != null ? o.windowT : OPT.windowT;
    var rRefl = OPT.datasheetRangeM * Math.sqrt(rho * T / OPT.datasheetRho);
    var rRes = OPT.hazardD / (OPT.returns * OPT.stepDeg * Math.PI / 180);
    return { reflectanceM: rRefl, resolutionM: rRes, detectM: Math.min(rRefl, rRes) };
  }
  function perceptionCap(o) { var d = rangeOnMoon(o).detectM; return capFor(d + CAM.marginM - 0.5) ; }   // stop with 0.5 m to spare inside the seen range
  var api = { OPT: OPT, rangeOnMoon: rangeOnMoon, perceptionCap: perceptionCap, CAM: CAM, capFor: capFor, lossPolicy: lossPolicy, GEO: { units: UNITS, head: HEAD, plinthH: PLINTH_H, deckY: DECK_Y, headY: HEAD_Y, ringM: RING_M, moduleUndersideM: CR.seatY }, pitches: pitches, solids: solids, selfHit: selfHit, selfHitWho: selfHitWho,
    coverage: coverage, unionCoverage: unionCoverage, candidates: candidates, study: study, moduleClearance: moduleClearance, MODULE_ABOARD: MODULE_ABOARD, rayBox: rayBox, rayCyl: rayCyl };
  root.NASA_LidarMount = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
