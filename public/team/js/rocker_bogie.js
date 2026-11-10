/*
 * Rocker-bogie suspension for rover v6: geometry, contact solve, exact link kinematics, wheel loads.
 * One file so the CAD model (fusion_rover.js), the simulator (viewer.html), the load model (rover_dynamics.js)
 * and the checks (tools/) all use the same numbers.
 *
 * Layout (rover frame: +x forward, +y up from the ground, +z port), per side s = +1 (port, L) / -1 (starboard, R):
 *   rocker: one bent link pivoting on the chassis at P = (0, 0.194). Its front arm carries the front wheel;
 *           its rear arm carries the bogie pivot B = (-0.12, 0.125).
 *   bogie:  a short link on B carrying the middle wheel (x 0) and the rear wheel (x -0.24).
 *   wheels: x = +0.24 / 0 / -0.24, z = +-0.235, axle y 0.065 (130 mm wheels), one 300 RPM gearmotor each.
 *   differential: a bevel-gear differential on the transverse pivot shaft (in the tub, above the battery) makes
 *           the two rockers turn equal and opposite in the body frame, so the body pitch is the mean of the sides.
 * With the rocker pivot directly above the middle wheel and the bogie pivot midway between the middle and rear
 * wheels, the static split on level ground is equal (1/3 of the side load per wheel), and it stays equal when one
 * wheel climbs: the linkage is statically determinate. Link geometry is a modelling choice (photo not read):
 * PLACEHOLDER until the printed part is measured.
 */
(function (global) {
  'use strict';
  var GEO = {
    wheelR: 0.065, wheelW: 0.075, axleY: 0.065,
    xF: 0.24, xM: 0, xR: -0.24, zW: 0.235,          // v11: track 0.47 m (was 0.45) for the roll margin; overall width 0.60 m, inside the 24 in cube.
                                            // Wheelbase 0.48 m (was 0.43): the loaded fore-aft tip margin needs >= 25 deg (sim decision, 2026-11-07)
    P: { x: 0, y: 0.194 },                  // rocker pivot (chassis), on the differential shaft
    B: { x: -0.12, y: 0.125 },               // bogie pivot on the rocker's rear arm, midway between the middle and rear wheels
    zRocker: 0.175, zBogie: 0.157,          // link mid-planes (outboard of the tub wall z 0.146, inboard of the motors z 0.188)
    linkT: 0.012, linkH: 0.018,             // printed link section (PETG), m
    rhoMax: 0.21, betaMax: 0.29             // travel stops (v11, was 0.18 / 0.26): rocker +-0.21 rad from the body, bogie +-0.29 rad from the rocker.
                                            // The deck now has open wheel arches (the deck is 0.38 m wide, the tyres run outside it), so the limit is the
                                            // tyre top reaching the deck top level (0.215 m with the cleats at the worst combination; tools/cad_model_check.js)
  };
  // Sand tyre (v7): 75 mm wide woven-steel hoop with chevron cleats and flotation flares. The numbers below are ASSUMED
  // scalings, not measurements: sinkage goes as (b_ref / b)^1.11 (Bekker-Wong, n ~ 0.9), grip gains with cleat height,
  // and the wide flexible hoop lowers compaction resistance. Confirm on a bevameter / drawbar test.
  var TIRE = { widthM: 0.075, refWidthM: 0.050, cleatM: 0.008, cleats: 20 };
  TIRE.sinkScale = Math.pow(TIRE.refWidthM / TIRE.widthM, 1.11);     // 0.64
  TIRE.muGain = 1.12;                                              // cleats bite the regolith (ASSUMED)
  TIRE.crr = 0.12;                                                 // was 0.15 for the 50 mm rigid tyre (ASSUMED)
  var IDS = ['FL', 'ML', 'BL', 'FR', 'MR', 'BR'];   // same order as fusion_rover wheelGroups

  function rot(dx, dy, a) { var c = Math.cos(a), s = Math.sin(a); return { x: dx * c - dy * s, y: dx * s + dy * c }; }

  /**
   * Exact wheel-centre positions (body frame x, y) for one side. rho = rocker angle, beta = bogie angle, both in the
   * body frame (rotation about +z, nose-up positive). Returns { F, M, R, B } with B the moved bogie pivot.
   */
  function sidePose(rho, beta) {
    var P = GEO.P, B = GEO.B, AY = GEO.axleY;
    var f = rot(GEO.xF - P.x, AY - P.y, rho), b = rot(B.x - P.x, B.y - P.y, rho);
    var Bw = { x: P.x + b.x, y: P.y + b.y };
    var m = rot(GEO.xM - B.x, AY - B.y, beta), r = rot(GEO.xR - B.x, AY - B.y, beta);
    return { F: { x: P.x + f.x, y: P.y + f.y }, M: { x: Bw.x + m.x, y: Bw.y + m.y }, R: { x: Bw.x + r.x, y: Bw.y + r.y }, B: Bw };
  }

  /** Small Gaussian elimination with partial pivoting (A is n x n, b length n). */
  function gauss(A, b) {
    var n = b.length, M = A.map(function (r, i) { return r.concat([b[i]]); });
    for (var c = 0; c < n; c++) {
      var piv = c;
      for (var r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
      var t = M[c]; M[c] = M[piv]; M[piv] = t;
      if (Math.abs(M[c][c]) < 1e-12) return null;
      for (var r2 = c + 1; r2 < n; r2++) { var f = M[r2][c] / M[c][c]; for (var k = c; k <= n; k++) M[r2][k] -= f * M[c][k]; }
    }
    var x = new Array(n);
    for (var i = n - 1; i >= 0; i--) { var s = M[i][n]; for (var j = i + 1; j < n; j++) s -= M[i][j] * x[j]; x[i] = s / M[i][i]; }
    return x;
  }

  /**
   * Contact solve. d[i] = height the wheel centre must sit at (ground + radius - sinkage), in a frame that turns
   * with the rover's heading but stays level, minus the nominal axle height, for wheels in IDS order.
   * Unknowns: body heave h, pitch slope th (nose-up), roll slope ph (+z side up), rocker angle rho (port side;
   * starboard is -rho through the differential), bogie angles bL, bR. Six wheels, six unknowns: determinate.
   * Travel limits clamp the links; then the body is raised until no wheel is below the ground, and a wheel more
   * than 2 mm above its ground is reported lifted. Linear in the angles (the mesh uses the exact pose).
   */
  function solve(d) {
    var xF = GEO.xF, xM = GEO.xM, xR = GEO.xR, xB = GEO.B.x, z = GEO.zW;
    var A = [], b = [];
    [1, -1].forEach(function (s, si) {
      var zs = s * z, bL = si === 0 ? 1 : 0, bR = si === 1 ? 1 : 0;
      A.push([1, xF, zs, s * (xF - GEO.P.x), 0, 0]);
      A.push([1, xM, zs, s * (xB - GEO.P.x), bL * (xM - xB), bR * (xM - xB)]);
      A.push([1, xR, zs, s * (xB - GEO.P.x), bL * (xR - xB), bR * (xR - xB)]);
      b.push(d[si * 3], d[si * 3 + 1], d[si * 3 + 2]);
    });
    var u = gauss(A, b) || [ (d[0] + d[1] + d[2] + d[3] + d[4] + d[5]) / 6, 0, 0, 0, 0, 0 ];
    var clamped = false, fixed = [false, false, false, false, false, false];
    // Clamp a link at its stop, then least-squares the body (and any free link) onto the wheels; twice, because
    // re-settling the body can push the other link to its stop.
    for (var pass = 0; pass < 2; pass++) {
      var any = false;
      var lim = function (v, m) { return v > m ? m : (v < -m ? -m : v); };
      var rc = lim(u[3], GEO.rhoMax); if (rc !== u[3]) { u[3] = rc; fixed[3] = true; any = true; }
      var bl = u[3] + lim(u[4] - u[3], GEO.betaMax); if (bl !== u[4]) { u[4] = bl; fixed[4] = true; any = true; }   // bogie travel is relative to its rocker
      var br = -u[3] + lim(u[5] + u[3], GEO.betaMax); if (br !== u[5]) { u[5] = br; fixed[5] = true; any = true; }
      if (!any) break;
      clamped = true;
      var free = [0, 1, 2, 3, 4, 5].filter(function (j) { return !fixed[j]; });
      var NtN = free.map(function () { return free.map(function () { return 0; }); }), Ntb = free.map(function () { return 0; });
      for (var e = 0; e < 6; e++) {
        var rhs = b[e]; for (var q = 0; q < 6; q++) if (fixed[q]) rhs -= A[e][q] * u[q];
        for (var a1 = 0; a1 < free.length; a1++) { Ntb[a1] += A[e][free[a1]] * rhs; for (var a2 = 0; a2 < free.length; a2++) NtN[a1][a2] += A[e][free[a1]] * A[e][free[a2]]; }
      }
      var sol = gauss(NtN, Ntb); if (sol) free.forEach(function (j, k2) { u[j] = sol[k2]; });
    }
    var cl = function (v, m) { return v > m ? m : (v < -m ? -m : v); };      // final hard stops
    u[3] = cl(u[3], GEO.rhoMax); u[4] = u[3] + cl(u[4] - u[3], GEO.betaMax); u[5] = -u[3] + cl(u[5] + u[3], GEO.betaMax);
    var y = new Array(6), pen = -1e9;
    for (var i = 0; i < 6; i++) { y[i] = A[i][0] * u[0] + A[i][1] * u[1] + A[i][2] * u[2] + A[i][3] * u[3] + A[i][4] * u[4] + A[i][5] * u[5]; pen = Math.max(pen, d[i] - y[i]); }
    if (pen > 0) { u[0] += pen; for (var k = 0; k < 6; k++) y[k] += pen; }
    var lifted = y.map(function (yi, j) { return yi - d[j] > 0.002; });
    return { h: u[0], th: u[1], ph: u[2], rho: u[3], bL: u[4], bR: u[5], wheelY: y, lifted: lifted, clamped: clamped };
  }

  /**
   * Wheel normal loads (N, IDS order). W = weight normal to the contact plane, momX / momZ = moments about the
   * contact-plane origin (W x_com + F_x h, W z_com + F_z h). Roll: the two sides share W by the moment about the
   * wheel lines (z +-0.235). Pitch: the differential hands each rocker half the pitch moment as a torque; a rocker
   * splits its side load between the front wheel and the bogie by its lever arms; the bogie splits equally.
   * A lifted wheel carries nothing: its share goes to the wheels still down on that side.
   */
  function loads(W, momX, momZ, lifted) {
    var z = GEO.zW, aF = GEO.xF - GEO.P.x, aB = GEO.P.x - GEO.B.x;
    var NLtot = W / 2 + momZ / (2 * z), NRtot = W - NLtot;
    var out = [];
    [NLtot, NRtot].forEach(function (V, si) {
      var T = (momX - W * GEO.P.x) / 2;                     // pitch torque each rocker carries
      var Nf = (T + aB * V) / (aF + aB), Nb = V - Nf;
      var side = [Nf, Nb / 2, Nb / 2];
      if (lifted) {
        var down = 0, sum = 0;
        for (var j = 0; j < 3; j++) { if (!lifted[si * 3 + j]) { down++; sum += Math.max(0, side[j]); } }
        var Vs = Math.max(0, V);
        for (var k = 0; k < 3; k++) side[k] = lifted[si * 3 + k] ? 0 : (sum > 1e-9 ? Math.max(0, side[k]) * Vs / sum : (down ? Vs / down : 0));
      }
      out.push(Math.max(0, side[0]), Math.max(0, side[1]), Math.max(0, side[2]));
    });
    return out;
  }

  var api = { GEO: GEO, TIRE: TIRE, IDS: IDS, sidePose: sidePose, solve: solve, loads: loads, gauss: gauss };
  global.NASA_RockerBogie = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
