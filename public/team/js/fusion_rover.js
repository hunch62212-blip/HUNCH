/**
 * NASA HUNCH LLASO — AEGIS-V1 Autonomous Lunar Cargo Rover, 3D assembly  v4.0
 * ============================================================================
 * Fits a 24 in (0.6096 m) cube: 0.584 x 0.604 m swept footprint, 0.556 m tall (top of lidar).
 * Six 360-degree swerve modules (6 drive gearmotors + 6 continuous-rotation steering servos), belly tub with
 * the 12.8 V 5 Ah LiFePO4 pack centred and low, 2-link grabber arm that stows over the deck, two cargo
 * canisters carried crosswise in V-saddle cradles with trunnion latches, dock charge pad on the avionics pod.
 * Part masses and sizes: see NASA_RoverBOM (js/rover_bom.js) and review/docs/rover-v4-design.md.
 *
 * Root frame: +X forward, +Y up, ±Z sides, ground at y = 0, wheel axles at y = 0.065.
 *
 * Simulator contract (do not rename): wheels[], wheelGroups[{group,steer,tire,lowerEye,id,localPos,baseZ}],
 * window.shockLinks, lidarHead, fanRotors, headlights, avLidGroup, cargoLidGroup, upperShellGroup,
 * roverArm{base,shoulder,elbow,wrist,gripper,fingerL,fingerR,tip,grabberCam,L1,L2,stow}, grabberCamera,
 * cargoBayGroup, cargoBaySlots, comMarkerGroup, batteryBay, roverRig, ROVER_GEOM.
 */
(function (global) {
  'use strict';

  // ---------------------------------------------------------------------------
  // helpers
  // ---------------------------------------------------------------------------
  function mk(geo, mat, x, y, z, parent, shadow) {
    var m = new THREE.Mesh(geo, mat);
    m.position.set(x || 0, y || 0, z || 0);
    if (shadow) m.castShadow = true;
    if (parent) parent.add(m);
    return m;
  }
  function box(w, h, d, mat, x, y, z, parent, shadow) { return mk(new THREE.BoxGeometry(w, h, d), mat, x, y, z, parent, shadow); }
  function cyl(rt, rb, h, mat, x, y, z, parent, axis, seg, shadow) {
    var m = mk(new THREE.CylinderGeometry(rt, rb, h, seg || 20), mat, x, y, z, parent, shadow);
    if (axis === 'x') m.rotation.z = Math.PI / 2;      // cylinder axis along X
    else if (axis === 'z') m.rotation.x = Math.PI / 2; // cylinder axis along Z
    return m;
  }
  function canvasTex(w, h, draw, rep) {
    var cv = document.createElement('canvas'); cv.width = w; cv.height = h;
    var c = cv.getContext('2d'); draw(c, w, h);
    var t = new THREE.CanvasTexture(cv);
    if (rep) { t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(rep[0], rep[1]); }
    return t;
  }

  // Painted skin: very fine orange-peel + panel-line texture so the white coat reads as paint, not plastic.
  function paintTexture() {
    return canvasTex(512, 512, function (c, w, h) {
      c.fillStyle = '#f4f6f8'; c.fillRect(0, 0, w, h);
      for (var i = 0; i < 2600; i++) {
        var s = 236 + Math.random() * 18;
        c.fillStyle = 'rgba(' + (s | 0) + ',' + (s | 0) + ',' + ((s + 4) | 0) + ',0.5)';
        c.fillRect(Math.random() * w, Math.random() * h, 2, 2);
      }
    }, [2, 2]);
  }
  function diamondTexture() {
    return canvasTex(256, 256, function (c, w, h) {
      c.fillStyle = '#20252d'; c.fillRect(0, 0, w, h);
      c.strokeStyle = '#3a424e'; c.lineWidth = 3;
      for (var y = 0; y < h; y += 32) for (var x = 0; x < w; x += 32) {
        c.beginPath(); c.moveTo(x + 4, y + 28); c.lineTo(x + 28, y + 4); c.stroke();
        c.beginPath(); c.moveTo(x + 4, y + 4); c.lineTo(x + 28, y + 28); c.stroke();
      }
    }, [4, 4]);
  }
  function treadTexture() {
    return canvasTex(256, 128, function (c, w, h) {
      c.fillStyle = '#17191e'; c.fillRect(0, 0, w, h);
      c.fillStyle = '#262a31';
      for (var x = 0; x < w; x += 16) c.fillRect(x, 0, 8, h);
    }, [6, 1]);
  }
  function foilTexture() {
    return canvasTex(256, 256, function (c, w, h) {
      c.fillStyle = '#d9a21b'; c.fillRect(0, 0, w, h);
      for (var i = 0; i < 90; i++) {
        c.fillStyle = 'rgba(' + (255 - Math.random() * 60 | 0) + ',' + (200 - Math.random() * 60 | 0) + ',40,0.45)';
        var x = Math.random() * w, y = Math.random() * h;
        c.beginPath(); c.moveTo(x, y); c.lineTo(x + 30 + Math.random() * 40, y + 10 - Math.random() * 20); c.lineTo(x + 10, y + 30); c.fill();
      }
    }, [2, 2]);
  }

  // ---------------------------------------------------------------------------
  // Rounded, bevelled box (extruded rounded rectangle) so panels catch light on their edges like machined /
  // moulded parts instead of reading as raw boxes. r = corner radius in the XY plane, edges bevelled along Z.
  function rbox(w, h, d, r, mat, x, y, z, parent, shadow) {
    var bev = Math.max(0.0008, Math.min(r * 0.5, d * 0.25, 0.006));
    var hw = w / 2 - bev, hh = h / 2 - bev, rr = Math.max(0.0006, Math.min(r, hw - 1e-4, hh - 1e-4));
    var s = new THREE.Shape();
    s.moveTo(-hw + rr, -hh); s.lineTo(hw - rr, -hh); s.quadraticCurveTo(hw, -hh, hw, -hh + rr);
    s.lineTo(hw, hh - rr); s.quadraticCurveTo(hw, hh, hw - rr, hh); s.lineTo(-hw + rr, hh);
    s.quadraticCurveTo(-hw, hh, -hw, hh - rr); s.lineTo(-hw, -hh + rr); s.quadraticCurveTo(-hw, -hh, -hw + rr, -hh);
    var depth = Math.max(0.0005, d - 2 * bev);
    var geo = new THREE.ExtrudeGeometry(s, { depth: depth, bevelEnabled: true, bevelThickness: bev, bevelSize: bev, bevelSegments: 3, curveSegments: 6 });
    geo.translate(0, 0, -depth / 2);
    geo.computeVertexNormals();
    return mk(geo, mat, x, y, z, parent, shadow);
  }
  // spur gear outline (extruded), axis along Y after the rotation below
  function gear(rad, teeth, thick, mat, x, y, z, parent) {
    var s = new THREE.Shape(), n = teeth * 4, td = rad * 0.10;
    for (var i = 0; i <= n; i++) {
      var a = i / n * Math.PI * 2, q = i % 4, rr = (q === 1 || q === 2) ? rad : rad - td;
      if (i === 0) s.moveTo(Math.cos(a) * rr, Math.sin(a) * rr); else s.lineTo(Math.cos(a) * rr, Math.sin(a) * rr);
    }
    var hole = new THREE.Path(); hole.absarc(0, 0, rad * 0.35, 0, Math.PI * 2, true); s.holes.push(hole);
    var g = new THREE.ExtrudeGeometry(s, { depth: thick, bevelEnabled: false, curveSegments: 4 });
    g.translate(0, 0, -thick / 2); g.rotateX(Math.PI / 2);
    return mk(g, mat, x, y, z, parent, true);
  }

  /* =============================================================================================
   * AEGIS-V1 rover v6 — rocker-bogie, skid-steer cargo rover inside a 24 in (0.6096 m) cube (cradle rails extend
   *   the length). v6 replaces the v4 swerve modules below with js/rocker_bogie.js; the rest of this note is v4/v5.
   * Root frame: +X forward, +Y up, ±Z sides, ground at y = 0 (wheel radius 0.065 = axle height).
   *   6 swerve modules at x = +0.24 / 0 / -0.24, z = ±0.235. Each: 130 mm wheel with an in-hub 37 mm
   *   gearmotor (drive), a continuous-rotation steering servo + AS5600 absolute encoder + 6-wire slip ring
   *   (unlimited 360 deg swivel), and a coil-over strut. 12 motors in all. A module sweeps r = 0.077 m when it
   *   turns, so modules never touch and the swept outline stays inside 0.292 x 0.302 m half-size.
   *   Belly tub (z ±0.14, y 0.065-0.185) carries the 5 Ah pack and electronics low and central.
   *   Deck top y = 0.205: avionics pod + charge pad (front), arm turret (front port), mast + lidar
   *   (front starboard), two cargo canisters crosswise in V-saddle cradles (aft).
   * ============================================================================================= */
  var GEOM = {
    wheelR: 0.065, wheelW: 0.075, axleY: 0.065, deckY: 0.225, modX: 0.24, modZ: 0.235,   // modX / modZ = wheel x / z (rocker_bogie GEO)
    halfLen: 0.30, halfWid: 0.302, height: 0.58, collR: 0.33,
    pad: { x: 0.19, y: 0.286, z: 0 },          // dock coupler reference: 11 mm under the copper face (y 0.297) so the pogos compress 4 mm
    battery: { L: 0.155, W: 0.100, H: 0.100 }      // 12.8 V 10 Ah envelope (Green Cell CUBE listing; geometry TBC)
  };

  function initFusionRover(opts) {
    opts = opts || {};
    var roverGroup = opts.roverGroup;
    var wheelGroups = opts.wheelGroups || [];
    var wheels = opts.wheels || [];
    var root = new THREE.Group();
    root.name = 'aegisRoverAssembly';
    roverGroup.add(root);
    (opts.hide || []).forEach(function (o) { if (o) o.visible = false; });

    // ── materials (white thermal paint, graphite structure, anodised aluminium, one blue accent) ─────────
    var paintMap = paintTexture();
    var mPaint = new THREE.MeshPhysicalMaterial({ color: 0xf4f6f8, map: paintMap, roughness: 0.32, metalness: 0.05, clearcoat: 1.0, clearcoatRoughness: 0.06 });
    var mPaintShade = new THREE.MeshPhysicalMaterial({ color: 0xdfe4ea, roughness: 0.36, metalness: 0.05, clearcoat: 0.8, clearcoatRoughness: 0.1 });
    var mBlue = new THREE.MeshPhysicalMaterial({ color: 0x0b4fb3, roughness: 0.28, metalness: 0.3, clearcoat: 0.6, clearcoatRoughness: 0.1 });
    var mGraphite = new THREE.MeshStandardMaterial({ color: 0x23272e, roughness: 0.5, metalness: 0.5 });
    var mGraphiteMatte = new THREE.MeshStandardMaterial({ color: 0x2b3038, roughness: 0.78, metalness: 0.25 });
    var mAlu = new THREE.MeshStandardMaterial({ color: 0xc9d1db, roughness: 0.28, metalness: 0.85 });
    var mAluDark = new THREE.MeshStandardMaterial({ color: 0x6f7a87, roughness: 0.34, metalness: 0.8 });
    var mSteel = new THREE.MeshStandardMaterial({ color: 0xaab2bc, roughness: 0.18, metalness: 0.95 });
    var mFoil = new THREE.MeshStandardMaterial({ color: 0xffffff, map: foilTexture(), roughness: 0.4, metalness: 0.6 });
    var mRubber = new THREE.MeshStandardMaterial({ color: 0x15171b, roughness: 0.93, metalness: 0.02 });
    var mTire = new THREE.MeshStandardMaterial({ color: 0x1b1d22, map: treadTexture(), roughness: 0.88, metalness: 0.04 });
    var mBell = new THREE.MeshStandardMaterial({ color: 0x1b1d22, roughness: 0.8, metalness: 0.05 });   // air-spring bellows (one shared material)
    var mCopper = new THREE.MeshStandardMaterial({ color: 0xc27a2c, roughness: 0.3, metalness: 0.85 });
    var mLens = new THREE.MeshPhysicalMaterial({ color: 0x050b14, emissive: 0x0b66b8, emissiveIntensity: 0.45, roughness: 0.04, metalness: 0.4, clearcoat: 1 });
    var mGlass = new THREE.MeshPhysicalMaterial({ color: 0x0c1a2b, roughness: 0.05, metalness: 0.2, clearcoat: 1, transparent: true, opacity: 0.85 });
    var mPcb = new THREE.MeshStandardMaterial({ color: 0x0f5a2e, roughness: 0.6, metalness: 0.2 });
    var mLedWhite = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xfff3dd, emissiveIntensity: 2.2 });
    var mLedGreen = new THREE.MeshStandardMaterial({ color: 0x22c55e, emissive: 0x16a34a, emissiveIntensity: 2.0 });
    var mLedRed = new THREE.MeshStandardMaterial({ color: 0xef4444, emissive: 0xb91c1c, emissiveIntensity: 2.2 });
    var mAmber = new THREE.MeshStandardMaterial({ color: 0xf59e0b, emissive: 0xd97706, emissiveIntensity: 0.9 });
    var mDeck = new THREE.MeshStandardMaterial({ color: 0xffffff, map: diamondTexture(), roughness: 0.55, metalness: 0.4 });

    var fanRotors = [], headlights = [];
    var G = GEOM, DY = G.deckY, R = G.wheelR, AY = G.axleY;

    // =========================================================================
    // 1. STRUCTURE — belly tub, skid plate, deck, frame rails
    // =========================================================================
    var TUB_W = 0.276, TUB_L = 0.50, TUB_Y0 = 0.066, TUB_Y1 = 0.205;
    // The tub is a shell, not a solid block: the battery bay is a real cavity that opens on the aft face (door below),
    // so with the door down you look into a lined bay with guide rails and the blind-mate connector.
    var BAYX1 = 0.108, BAYY0 = 0.074, BAYY1 = 0.184;                                                         // bay cavity: aft face to x 0.108, y 0.074-0.184
    var bayZc = (global.NASA_RoverBOM && NASA_RoverBOM.batteryZ != null) ? NASA_RoverBOM.batteryZ : 0, bayHW = (G.battery.W + 0.018) / 2;
    var zP = bayZc + bayHW, zS = bayZc - bayHW;                                                              // bay side walls (port +z / starboard -z)
    var mLiner = new THREE.MeshStandardMaterial({ color: 0x9aa3af, roughness: 0.75, metalness: 0.2 });
    rbox(TUB_L / 2 - BAYX1, TUB_Y1 - TUB_Y0, TUB_W, 0.01, mGraphite, (BAYX1 + TUB_L / 2) / 2, (TUB_Y0 + TUB_Y1) / 2, 0, root, true);   // forward block
    rbox(BAYX1 + TUB_L / 2, TUB_Y1 - BAYY1, TUB_W, 0.006, mGraphite, (BAYX1 - TUB_L / 2) / 2, (BAYY1 + TUB_Y1) / 2, 0, root, true);   // roof slab
    rbox(BAYX1 + TUB_L / 2, BAYY0 - TUB_Y0, TUB_W, 0.006, mGraphite, (BAYX1 - TUB_L / 2) / 2, (TUB_Y0 + BAYY0) / 2, 0, root, true);   // floor slab
    rbox(BAYX1 + TUB_L / 2, BAYY1 - BAYY0, TUB_W / 2 - zP, 0.004, mGraphite, (BAYX1 - TUB_L / 2) / 2, (BAYY0 + BAYY1) / 2, (zP + TUB_W / 2) / 2, root, true);    // port wall
    rbox(BAYX1 + TUB_L / 2, BAYY1 - BAYY0, zS + TUB_W / 2, 0.004, mGraphite, (BAYX1 - TUB_L / 2) / 2, (BAYY0 + BAYY1) / 2, (zS - TUB_W / 2) / 2, root, true);  // starboard wall
    // bay lining: pale panels so the cavity reads when the door is down
    var lx0 = -TUB_L / 2 + 0.004, llen = BAYX1 - lx0, lcx = (BAYX1 + lx0) / 2;
    box(llen, 0.002, 2 * bayHW, mLiner, lcx, BAYY0 + 0.001, bayZc, root);
    box(llen, 0.002, 2 * bayHW, mLiner, lcx, BAYY1 - 0.001, bayZc, root);
    box(llen, BAYY1 - BAYY0, 0.002, mLiner, lcx, (BAYY0 + BAYY1) / 2, zP - 0.001, root);
    box(llen, BAYY1 - BAYY0, 0.002, mLiner, lcx, (BAYY0 + BAYY1) / 2, zS + 0.001, root);
    box(0.002, BAYY1 - BAYY0, 2 * bayHW, mLiner, BAYX1 - 0.001, (BAYY0 + BAYY1) / 2, bayZc, root);          // back wall
    [-1, 1].forEach(function (sz) { box(llen - 0.01, 0.006, 0.006, mAlu, lcx, BAYY0 + 0.005, bayZc + sz * (bayHW - 0.008), root); });   // sled guide rails
    box(0.014, 0.050, 0.060, mGraphite, BAYX1 - 0.009, 0.130, bayZc, root);                                  // blind-mate connector housing
    [-0.016, 0, 0.016].forEach(function (dz) { box(0.006, 0.010, 0.008, mCopper, BAYX1 - 0.018, 0.140, bayZc + dz, root); box(0.006, 0.010, 0.008, mAlu, BAYX1 - 0.018, 0.118, bayZc + dz, root); });
    box(llen - 0.02, 0.003, 0.012, mLedWhite, lcx, BAYY1 - 0.004, bayZc, root);                              // service light strip under the roof
    var bayLight = new THREE.PointLight(0xffffff, 0, 0.6, 1); bayLight.position.set(-0.12, BAYY1 - 0.02, bayZc); root.add(bayLight);   // service light: on while the door is down
    rbox(TUB_L - 0.06, 0.008, TUB_W - 0.04, 0.02, mAlu, 0, TUB_Y0 - 0.002, 0, root);                      // skid plate
    [1, -1].forEach(function (s) {                                                                           // sloped entry/exit skids
      var sk = rbox(0.06, 0.008, TUB_W - 0.05, 0.01, mAluDark, s * (TUB_L / 2 - 0.012), TUB_Y0 + 0.012, 0, root); sk.rotation.z = s * 0.45;
    });
    // tub side panels: white skin with louvred cooling grilles and the wordmark plate
    [1, -1].forEach(function (s) {
      rbox(TUB_L - 0.07, 0.090, 0.006, 0.012, mPaint, 0, 0.127, s * (TUB_W / 2 + 0.002), root, true);
      for (var lv = 0; lv < 7; lv++) rbox(0.006, 0.040, 0.004, 0.002, mGraphiteMatte, 0.10 + lv * 0.014, 0.128, s * (TUB_W / 2 + 0.006), root);
      rbox(0.14, 0.022, 0.003, 0.004, mBlue, -0.11, 0.150, s * (TUB_W / 2 + 0.006), root);
    });
    // deck: rounded white plate with a diamond-tread walkway between the cargo cradles
    // v11: the deck is 0.38 m wide and the tyres run outside it (open wheel arches), so a wheel can rise to deck level without touching anything
    rbox(0.58, 0.014, 0.38, 0.030, mPaint, 0, DY - 0.007, 0, root, true);
    rbox(0.10, 0.002, 0.30, 0.02, mDeck, -0.21, DY + 0.001, 0, root);                                     // diamond-tread step pad, aft of the rear ring
    [1, -1].forEach(function (s) {                                                                           // blue edge piping
      rbox(0.54, 0.006, 0.006, 0.003, mBlue, 0, DY - 0.010, s * 0.188, root);
    });
    // frame rails under the deck (aluminium extrusion) on the tub top, inboard of the rocker-bogie links and wheels
    [1, -1].forEach(function (s) {
      rbox(0.56, 0.018, 0.020, 0.003, mAluDark, 0, DY - 0.023, s * 0.125, root, true);
    });

    // =========================================================================
    // 2. ROCKER-BOGIE — per side: a bent rocker on the chassis pivot (front wheel + bogie pivot) and a bogie
    //    (middle + rear wheel); a bevel differential on the transverse shaft ties the two rockers together.
    //    Geometry and travel come from js/rocker_bogie.js (one source for the mesh, the sim and the checks).
    // =========================================================================
    var RB = global.NASA_RockerBogie, RG_ = RB.GEO, P = RG_.P, BP0 = RG_.B, LT = RG_.linkT, LH = RG_.linkH;
    var mPetg = new THREE.MeshStandardMaterial({ color: 0x15171b, roughness: 0.62, metalness: 0.08 });   // black printed PETG, as in the reference photo
    function arm(x0, y0, x1, y1, z, parent) {                                // straight link segment between two points
      var L = Math.hypot(x1 - x0, y1 - y0), m = rbox(L + LH, LH, LT, 0.006, mPetg, (x0 + x1) / 2, (y0 + y1) / 2, z, parent, true);
      m.rotation.z = Math.atan2(y1 - y0, x1 - x0); return m;
    }
    // differential: housing on the centreline above the battery sled, shaft across the tub to both rockers
    cyl(0.014, 0.014, 0.05, mAluDark, P.x, P.y, 0, root, 'z', 24, true);
    gear(0.012, 16, 0.006, mSteel, P.x, P.y + 0.016, 0, root);
    cyl(0.005, 0.005, 2 * RG_.zRocker, mSteel, P.x, P.y, 0, root, 'z', 14);
    var rockers = [], bogies = [];
    [1, -1].forEach(function (s) {
      var zr = s * RG_.zRocker, zb = s * RG_.zBogie;
      cyl(0.011, 0.011, 0.010, mAlu, P.x, P.y, s * (TUB_W / 2 + 0.004), root, 'z', 20);              // flanged bearing in the tub wall
      var rk = new THREE.Group(); rk.name = 'rocker-' + (s > 0 ? 'L' : 'R'); rk.position.set(P.x, P.y, zr); root.add(rk);
      arm(0, 0, RG_.xF - P.x, RG_.axleY - P.y, 0, rk);                                                  // front arm (to the front wheel)
      arm(0, 0, BP0.x - P.x, BP0.y - P.y, 0, rk);                                                       // rear arm (to the bogie pivot)
      cyl(0.013, 0.013, LT + 0.004, mPetg, 0, 0, 0, rk, 'z', 24, true);                                // pivot boss
      cyl(0.006, 0.006, LT + 0.008, mSteel, 0, 0, 0, rk, 'z', 12);                                      // pivot pin
      cyl(0.011, 0.011, LT + 0.002, mPetg, RG_.xF - P.x, RG_.axleY - P.y, 0, rk, 'z', 20);              // front axle boss
      cyl(0.010, 0.010, LT + 0.002, mPetg, BP0.x - P.x, BP0.y - P.y, 0, rk, 'z', 20);                    // bogie-pivot boss
      var bg = new THREE.Group(); bg.name = 'bogie-' + (s > 0 ? 'L' : 'R'); bg.position.set(BP0.x, BP0.y, zb); root.add(bg);
      arm(0, 0, RG_.xM - BP0.x, RG_.axleY - BP0.y, 0, bg);
      arm(0, 0, RG_.xR - BP0.x, RG_.axleY - BP0.y, 0, bg);
      cyl(0.010, 0.010, LT + 0.004, mPetg, 0, 0, 0, bg, 'z', 20, true);
      cyl(0.005, 0.005, RG_.zRocker - RG_.zBogie + LT, mSteel, 0, 0, s * (RG_.zRocker - RG_.zBogie) / 2, bg, 'z', 12);   // pin into the rocker
      [RG_.xM, RG_.xR].forEach(function (x) { cyl(0.010, 0.010, LT + 0.002, mPetg, x - BP0.x, RG_.axleY - BP0.y, 0, bg, 'z', 20); });
      rockers.push(rk); bogies.push(bg);
    });

    // wheels: 130 mm, one 300 RPM gearmotor in each hub, an axle stub from its link
    var wheelDefs = [
      { id: 'FL', x: RG_.xF, s: 1, link: 'r' }, { id: 'ML', x: RG_.xM, s: 1, link: 'b' }, { id: 'BL', x: RG_.xR, s: 1, link: 'b' },
      { id: 'FR', x: RG_.xF, s: -1, link: 'r' }, { id: 'MR', x: RG_.xM, s: -1, link: 'b' }, { id: 'BR', x: RG_.xR, s: -1, link: 'b' }
    ];
    wheelGroups.forEach(function (w) { if (w && w.group) w.group.visible = false; });
    wheels.forEach(function (t) { if (t) t.visible = false; });
    wheels.length = 0; wheelGroups.length = 0;
    var tireGeo = new THREE.CylinderGeometry(R - 0.012, R - 0.012, G.wheelW, 48, 1, true);        // hoop; the 8 mm cleats stand proud of it
    var mMesh = new THREE.MeshStandardMaterial({ color: 0x59606b, roughness: 0.55, metalness: 0.7, side: THREE.DoubleSide, map: treadTexture() });   // woven steel hoop
    wheelDefs.forEach(function (wd) {
      var wz = wd.s * RG_.zW, linkZ = wd.s * (wd.link === 'r' ? RG_.zRocker : RG_.zBogie);
      var wg = new THREE.Group(); wg.position.set(wd.x, AY, wz);
      var stub = (Math.abs(wz) - G.wheelW / 2) - (Math.abs(linkZ) + LT / 2);
      cyl(0.006, 0.006, stub + 0.004, mSteel, 0, 0, -wd.s * (G.wheelW / 2 + stub / 2), wg, 'z', 12);    // axle stub, link to hub
      // in-hub drive: 37 mm gearmotor along the axle, encoder cap on the outboard face
      cyl(0.0185, 0.0185, 0.074, mGraphite, 0, 0, 0, wg, 'z', 28, true);
      cyl(0.0125, 0.0125, 0.010, mAluDark, 0, 0, wd.s * 0.042, wg, 'z', 20);
      cyl(0.008, 0.008, 0.004, mPcb, 0, 0, wd.s * 0.047, wg, 'z', 14);
      // SAND TYRE (v7): wide 75 mm flexible woven-steel hoop (spring-tyre idea: the hoop flattens a contact patch, so
      // ground pressure is low), 20 chevron cleats 8 mm tall that bite sideways as well as forward, conical flotation
      // flares on both edges, eight tension spokes to a white hub with a blue cap (reference photo).
      var mLugW = new THREE.MeshStandardMaterial({ color: 0x1b1d22, roughness: 0.88, metalness: 0.04, transparent: true });   // per-wheel, faded at speed
      var mSpokeW = new THREE.MeshStandardMaterial({ color: 0xaab2bc, roughness: 0.18, metalness: 0.95, transparent: true });
      var mBlurW = new THREE.MeshBasicMaterial({ color: 0x3a3f48, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide });
      var spin = new THREE.Group(); spin.rotation.x = Math.PI / 2; wg.add(spin);
      var tire = new THREE.Mesh(tireGeo, mMesh); tire.castShadow = true; tire.receiveShadow = true; spin.add(tire);
      for (var gi = 0; gi < 20; gi++) {
        var a = gi / 20 * Math.PI * 2;
        [1, -1].forEach(function (half) {                                                                     // chevron = two angled blades
          var lug = new THREE.Mesh(new THREE.BoxGeometry(0.008, G.wheelW * 0.50, 0.010), mLugW);
          lug.position.set(Math.cos(a) * (R - 0.0085), half * G.wheelW * 0.24, Math.sin(a) * (R - 0.0085));
          lug.rotation.y = -a; lug.rotation.x = half * 0.50; lug.castShadow = true; tire.add(lug);
        });
      }
      [1, -1].forEach(function (s) {
        var flare = new THREE.Mesh(new THREE.CylinderGeometry(R - 0.012, R - 0.001, 0.012, 40, 1, true), mTire);   // flotation flare
        flare.position.y = s * (G.wheelW / 2 - 0.002); if (s < 0) flare.rotation.x = Math.PI; tire.add(flare);
        var bead = new THREE.Mesh(new THREE.TorusGeometry(R - 0.014, 0.0035, 8, 48), mRubber);
        bead.rotation.x = Math.PI / 2; bead.position.y = s * (G.wheelW / 2 - 0.002); tire.add(bead);
      });
      cyl(R - 0.020, R - 0.020, 0.004, mAluDark, 0, G.wheelW / 2 - 0.006, 0, tire, null, 36);                // thin inner flange rings
      cyl(R - 0.020, R - 0.020, 0.004, mAluDark, 0, -(G.wheelW / 2 - 0.006), 0, tire, null, 36);
      for (var sp = 0; sp < 8; sp++) {
        var sa = sp / 8 * Math.PI * 2;
        [1, -1].forEach(function (s) {
          var spoke = new THREE.Mesh(new THREE.BoxGeometry(0.036, 0.004, 0.005), mSpokeW);
          spoke.position.set(Math.cos(sa) * 0.036, s * (G.wheelW / 2 - 0.006), Math.sin(sa) * 0.036); spoke.rotation.y = -sa; tire.add(spoke);
        });
      }
      var blurDisc = new THREE.Mesh(new THREE.CylinderGeometry(R - 0.002, R - 0.002, G.wheelW - 0.004, 40, 1, true), mBlurW); blurDisc.renderOrder = 2; tire.add(blurDisc);
      tire.userData.blur = { lug: mLugW, spoke: mSpokeW, disc: mBlurW };
      cyl(0.021, 0.021, G.wheelW + 0.004, mPaint, 0, 0, 0, tire, null, 28);                                 // hub
      cyl(0.012, 0.012, G.wheelW + 0.007, mBlue, 0, 0, 0, tire, null, 20);                                  // blue centre cap
      root.add(wg);
      wheels.push(tire);
      // air-spring strut (self-adjusting suspension, js/suspension.js): a bellows and a damper rod over the axle, inboard of the tyre,
      // from the carrier up to the link bracket. setLength(x): extra stroke in m (ride-height raise plus body sag); the bellows stretch with it.
      var strutGrp = new THREE.Group(); strutGrp.position.set(0, 0.012, -wd.s * (Math.abs(wz) - Math.abs(linkZ)) * 0.92); wg.add(strutGrp);
      var bellows = new THREE.Mesh(new THREE.CylinderGeometry(0.0085, 0.0085, 1, 14), mBell); strutGrp.add(bellows);
      var rings = []; for (var rk = 0; rk < 5; rk++) { var rg2 = new THREE.Mesh(new THREE.TorusGeometry(0.0095, 0.0022, 6, 14), mBlue); rg2.rotation.x = Math.PI / 2; strutGrp.add(rg2); rings.push(rg2); }
      var rod = new THREE.Mesh(new THREE.CylinderGeometry(0.0035, 0.0035, 1, 10), mSteel); strutGrp.add(rod);
      cyl(0.011, 0.011, 0.008, mAluDark, 0, 0, 0, strutGrp, null, 16);                                           // lower cap on the carrier
      var topCap = cyl(0.011, 0.011, 0.008, mAluDark, 0, 0.07, 0, strutGrp, null, 16);                           // upper cap on the link bracket
      var strutApi = { setLength: function (x) {
        var top = 0.07 + Math.max(-0.060, Math.min(0.12, x));                                      // v11: kneel -40 mm, raise +60 mm, strut stroke +-50 mm
        bellows.scale.y = top; bellows.position.y = top / 2;
        rod.scale.y = top * 0.9; rod.position.y = top * 0.55; topCap.position.y = top;
        for (var q = 0; q < rings.length; q++) rings[q].position.y = top * (q + 0.5) / rings.length;
      } };
      strutApi.setLength(0);
      wheelGroups.push({ group: wg, tire: tire, id: wd.id, side: wd.s, strut: strutApi, localPos: new THREE.Vector3(wd.x, AY, wz), baseZ: wz });
    });
    // pose(rho, bL, bR): rocker angle (port; starboard is -rho through the differential) and bogie angles, body frame.
    // Moves the links and the wheel groups to the exact linkage pose; returns the six wheel (x, y) in IDS order.
    function poseBogie(rho, bL, bR) {
      var out = [];
      [[rho, bL], [-rho, bR]].forEach(function (ab, si) {
        var sp = RB.sidePose(ab[0], ab[1]);
        rockers[si].rotation.z = ab[0];
        bogies[si].position.x = sp.B.x; bogies[si].position.y = sp.B.y; bogies[si].rotation.z = ab[1];
        [sp.F, sp.M, sp.R].forEach(function (p, k) { var wg = wheelGroups[si * 3 + k]; wg.group.position.x = p.x; wg.group.position.y = p.y; out.push(p); });
      });
      return out;
    }
    global.roverBogie = { rockers: rockers, bogies: bogies, pose: poseBogie };
    global.shockLinks = [];          // v6: no coil-overs

    // =========================================================================
    // 3. BELLY TUB INTERNALS — 5 Ah pack (hot-swap, centred, low), compute and drivers forward
    // =========================================================================
    var internals = new THREE.Group(); internals.name = 'internals'; root.add(internals);
    [-0.015, 0.025, 0.065].forEach(function (z, k) { rbox(0.065, 0.022, 0.095 * 0.6, 0.004, mGraphite, 0.205, 0.095, z - 0.025, internals); });   // 3 x MDD10A

    // ── BATTERY BAY: hot-swap pack in the centre of the tub; drop-down door on the aft face, pack on a sled ──
    var BP = G.battery, BAY_W = BP.W + 0.018, BAY_H = BP.H + 0.010, BAY_Y0 = 0.074, HOME_X = (global.NASA_RoverBOM && NASA_RoverBOM.batteryX != null) ? NASA_RoverBOM.batteryX : 0.02;
    var BZ = (global.NASA_RoverBOM && NASA_RoverBOM.batteryZ != null) ? NASA_RoverBOM.batteryZ : 0;     // sled offset from the centreline (value from js/rover_bom.js; 0 since v13)
    [-1, 1].forEach(function (sg) {                                                                          // bezel ring around the opening (open in the middle)
      rbox(0.006, 0.008, BAY_W + 0.016, 0.003, mGraphiteMatte, -TUB_L / 2 - 0.001, sg > 0 ? BAY_Y0 + BAY_H + 0.004 : BAY_Y0 - 0.004, BZ, root);
      rbox(0.006, BAY_H, 0.008, 0.003, mGraphiteMatte, -TUB_L / 2 - 0.001, BAY_Y0 + BAY_H / 2, BZ + sg * (BAY_W / 2 + 0.004), root);
    });
    var batDoor = new THREE.Group(); batDoor.name = 'batteryDoor'; batDoor.position.set(-TUB_L / 2 - 0.006, BAY_Y0, BZ); root.add(batDoor);
    rbox(0.006, BAY_H, BAY_W, 0.008, mPaint, 0, BAY_H / 2, 0, batDoor, true);
    rbox(0.007, 0.010, BAY_W - 0.02, 0.003, mBlue, -0.001, BAY_H - 0.012, 0, batDoor);
    cyl(0.004, 0.004, BAY_W + 0.010, mAluDark, 0, 0, 0, batDoor, 'z', 10);
    var batLatches = [];
    [-1, 1].forEach(function (sz) {
      var lt = new THREE.Group(); lt.position.set(-0.004, BAY_H - 0.024, sz * (BAY_W / 2 - 0.016)); batDoor.add(lt);
      cyl(0.008, 0.008, 0.005, mAluDark, 0, 0, 0, lt, 'x', 14);
      box(0.006, 0.005, 0.022, mAmber, -0.003, 0, 0, lt);
      batLatches.push(lt);
    });
    var batPack = new THREE.Group(); batPack.name = 'batteryPack'; batPack.position.set(HOME_X, BAY_Y0 + 0.056, BZ); root.add(batPack);
    rbox(BP.L, BP.H, BP.W, 0.008, mGraphite, 0, 0.003, 0, batPack, true);                                // 12.8 V 10 Ah LiFePO4 (155 x 100 x 100)
    rbox(0.100, 0.002, 0.070, 0.004, mBlue, 0, 0.054, 0, batPack);
    rbox(BP.L + 0.010, 0.006, BP.W + 0.010, 0.004, mAluDark, 0, -0.050, 0, batPack);                     // sled
    rbox(0.012, 0.030, 0.056, 0.004, mAlu, -BP.L / 2 - 0.006, 0.004, 0, batPack);                        // pull handle
    var packLedMat = new THREE.MeshStandardMaterial({ color: 0x22c55e, emissive: 0x16a34a, emissiveIntensity: 2.0 });
    box(0.003, 0.008, 0.060, packLedMat, -BP.L / 2 - 0.001, 0.030, 0, batPack);
    global.batteryBay = {
      door: batDoor, pack: batPack, latches: batLatches, led: packLedMat,
      set: function (open, ext, soc, latch) {
        batDoor.rotation.z = open * 1.52;
        bayLight.intensity = Math.min(1, open) * 0.5;
        batPack.position.x = HOME_X + (global.batteryBay.ballast || 0) - ext * (0.36 + 0.02 - HOME_X);   // ballast shift on the lead screw; the swap slide still ends where the dock robot takes the pack
        var lq = latch == null ? 1 - open : latch;
        batLatches.forEach(function (lt) { lt.rotation.x = (1 - lq) * 1.5708; });
        if (soc != null) {
          var col = soc > 0.5 ? 0x22c55e : soc > 0.25 ? 0xf59e0b : 0xef4444;
          packLedMat.color.setHex(col); packLedMat.emissive.setHex(soc > 0.5 ? 0x16a34a : soc > 0.25 ? 0xd97706 : 0xb91c1c);
        }
      }
    };
    global.batteryBay.set(0, 0, 0.8, 1);

    // tub nose: headlights + ToF + bumper, tub tail: tail lights + charge port
    var NOSE = TUB_L / 2;
    rbox(0.012, 0.030, TUB_W + 0.02, 0.008, mAluDark, NOSE + 0.006, 0.085, 0, root, true);              // bumper bar
    [-0.10, 0.10].forEach(function (z) {
      rbox(0.008, 0.020, 0.050, 0.006, mGlass, NOSE + 0.003, 0.150, z, root);
      var hl = rbox(0.004, 0.012, 0.040, 0.004, mLedWhite, NOSE + 0.007, 0.150, z, root);
      headlights.push(hl);
    });
    cyl(0.008, 0.009, 0.010, mLens, NOSE + 0.005, 0.120, 0, root, 'x', 16);                               // front ToF
    [-0.10, 0.10].forEach(function (z) { rbox(0.006, 0.012, 0.050, 0.004, mLedRed, -NOSE - 0.004, 0.172, z, root); });
    cyl(0.009, 0.009, 0.010, mGraphite, -NOSE - 0.004, 0.172, 0, root, 'x', 14);

    // =========================================================================
    // 4. AVIONICS POD (front deck) with the dock charge pad on its roof
    // =========================================================================
    var upperShell = new THREE.Group(); upperShell.name = 'upperShell'; root.add(upperShell);
    var avLid = new THREE.Group(); avLid.name = 'avLidGroup'; avLid.position.set(0.115, DY + 0.060, 0); upperShell.add(avLid);   // hinged at its aft edge
    // Pod is an open tray (floor + four walls) with the compute boards inside; the lid is the full cover with the charge pad on top.
    var PX0 = 0.1125, PX1 = 0.2675, PW = 0.200, PH = 0.055, PT = 0.006, PXC = (PX0 + PX1) / 2;
    rbox(PX1 - PX0, PT, PW, 0.004, mPaint, PXC, DY + PT / 2, 0, upperShell, true);                            // floor
    [-1, 1].forEach(function (sz) { rbox(PX1 - PX0, PH, PT, 0.003, mPaint, PXC, DY + PH / 2, sz * (PW / 2 - PT / 2), upperShell, true); });   // side walls
    [-1, 1].forEach(function (sx) { rbox(PT, PH, PW - 2 * PT, 0.003, mPaint, sx > 0 ? PX1 - PT / 2 : PX0 + PT / 2, DY + PH / 2, 0, upperShell, true); });   // front + aft walls
    var pcb = DY + PT + 0.006;                                                                                // boards stand on 6 mm standoffs
    [[0.2, 0.05], [0.2, -0.05], [0.14, -0.055], [0.14, 0.05]].forEach(function (c) {
      [-1, 1].forEach(function (a) { [-1, 1].forEach(function (b) { cyl(0.0025, 0.0025, 0.006, mAlu, c[0] + a * 0.034, DY + PT + 0.003, c[1] + b * 0.02, upperShell, null, 8); }); });
    });
    rbox(0.085, 0.003, 0.056, 0.002, mPcb, 0.2, pcb, 0.05, upperShell, true);                                 // Raspberry Pi 5
    cyl(0.017, 0.017, 0.007, mGraphite, 0.2, pcb + 0.005, 0.05, upperShell, null, 20);                         // active cooler
    var piFan = new THREE.Group(); piFan.position.set(0.2, pcb + 0.0095, 0.05); upperShell.add(piFan);
    for (var fb = 0; fb < 7; fb++) { var bl = box(0.013, 0.001, 0.004, mAluDark, 0.006, 0, 0, piFan); bl.rotation.y = fb / 7 * Math.PI * 2; }
    piFan.userData.spin = 'y'; fanRotors.push(piFan);
    rbox(0.101, 0.003, 0.053, 0.002, mPcb, 0.2, pcb, -0.05, upperShell, true);                                // Arduino Mega 2560
    box(0.012, 0.010, 0.030, mGraphite, 0.2 + 0.044, pcb + 0.006, -0.05, upperShell);                         // USB-B jack
    rbox(0.025, 0.003, 0.062, 0.002, mPcb, 0.14, pcb, -0.055, upperShell, true);                              // PCA9685 servo driver
    for (var hp = 0; hp < 6; hp++) box(0.004, 0.006, 0.006, mAmber, 0.14, pcb + 0.004, -0.055 + (hp - 2.5) * 0.009, upperShell);
    rbox(0.036, 0.016, 0.066, 0.003, mGraphite, 0.14, pcb + 0.006, 0.05, upperShell, true);                   // 12 V to 5 V buck
    box(0.030, 0.003, 0.052, mAlu, 0.14, pcb + 0.0155, 0.05, upperShell);                                      // buck heatsink
    rbox(0.004, 0.030, 0.150, 0.004, mGraphiteMatte, 0.268, DY + 0.030, 0, upperShell);                    // front vent
    for (var vk = 0; vk < 5; vk++) rbox(0.003, 0.003, 0.140, 0.001, mAluDark, 0.271, DY + 0.018 + vk * 0.006, 0, upperShell);
    var P = G.pad;
    rbox(0.155, 0.006, 0.200, 0.004, mPaint, 0.0775, -0.002, 0, avLid, true);                              // lid plate: covers the whole pod
    rbox(0.140, 0.008, 0.170, 0.014, mGraphite, P.x - 0.115, 0.004, 0, avLid, true);                       // pad base (lid)
    [-0.045, 0.045].forEach(function (z) { rbox(0.110, 0.004, 0.030, 0.003, mCopper, P.x - 0.115, 0.010, z, avLid); });
    rbox(0.016, 0.003, 0.070, 0.002, mLedGreen, P.x - 0.115 + 0.060, 0.009, 0, avLid);
    [1, -1].forEach(function (s) {                                                                          // coupler guide lips
      var lip = rbox(0.140, 0.014, 0.006, 0.003, mBlue, P.x - 0.115, 0.012, s * 0.086, avLid); lip.rotation.x = -s * 0.5;
    });
    // stereo camera pair on the pod nose (v10: moved off the mast, which now stands beside the module)
    rbox(0.008, 0.012, 0.096, 0.003, mGraphite, 0.268, DY + 0.050, 0, upperShell, true);
    [-0.034, 0.034].forEach(function (z) { cyl(0.0055, 0.0065, 0.006, mLens, 0.270, DY + 0.050, z, upperShell, 'x', 16); });

    // =========================================================================
    // 5. LIDAR (v13). Two fixed YDLIDAR X2L scanners on opposite deck corners (front port, rear starboard), each on a 12 mm plinth. js/lidar_mount.js
    //    holds the study: the head tops out 31 mm under the module underside and 47 mm clear of its outline, inside the tyre track and the stand posts,
    //    so nothing needs to move before a load and nothing pops out. Looking under the module they keep a full 360 deg ring with the module aboard.
    //    The v12 telescoping mast (js/sensor_mast.js, tools/mast_test.js) stays in the repo as the rejected variant: its stowed head sat 24 mm inside the
    //    module swing and it needed 19 s of motion before each load.
    // =========================================================================
    var LMt = global.NASA_LidarMount, LG = LMt.GEO, lidarHeads = [];
    LG.units.forEach(function (u) {
      var pod = new THREE.Group(); pod.name = 'lidar_' + u.id; pod.position.set(u.x, DY, u.z); root.add(pod);
      cyl(0.036, 0.040, LG.plinthH, mAluDark, 0, LG.plinthH / 2, 0, pod, null, 28, true);                    // plinth bolted through the deck
      for (var bk = 0; bk < 4; bk++) { var ba = bk * Math.PI / 2 + Math.PI / 4; cyl(0.0022, 0.0022, 0.003, mSteel, Math.cos(ba) * 0.030, LG.plinthH + 0.0015, Math.sin(ba) * 0.030, pod, null, 8); }
      cyl(LG.head.r + 0.001, LG.head.r + 0.001, 0.008, mGraphite, 0, LG.plinthH + 0.004, 0, pod, null, 32, true);   // X2L base ring
      var head = new THREE.Group(); head.position.set(0, LG.plinthH + 0.008 + (LG.head.h - 0.008) / 2, 0); pod.add(head);   // spins about the vertical
      cyl(LG.head.r, LG.head.r, LG.head.h - 0.008, mGraphiteMatte, 0, 0, 0, head, null, 32, true);
      rbox(0.012, 0.010, 0.016, 0.003, mLens, LG.head.r - 0.002, 0.004, 0, head);                             // emitter / receiver window
      cyl(0.022, 0.022, 0.003, mAlu, 0, (LG.head.h - 0.008) / 2 - 0.0015, 0, head, null, 28);                 // top plate
      lidarHeads.push(head);
    });
    var lidarHead = lidarHeads[0];
    var mastPivot = new THREE.Group(); mastPivot.name = 'mastPivot'; root.add(mastPivot);                    // kept only so older callers that lean the mast have a node
    // The mast API is kept as a fixed stub for older callers (sortie, CAD page, tests): the scanners do not move, so every command is already satisfied.
    global.sensorMast = {
      group: mastPivot, pivot: mastPivot, upper: mastPivot, ext: 0, maxExt: 0, actuator: null, stageMesh: [], fixed: true,
      set: function () {}, command: function () {}, step: function () { return { u: 0, e: 0 }; }, atTarget: function () { return true; }, get u() { return 0; }
    };

    // =========================================================================
    // 6. RING CLAMPS (v10) — two rings at x = +-0.085 round the module's axis (the module lies along the rover).
    //    Each ring: a fixed 120 deg SADDLE (V seat, three rollers) on three deck posts, and two 60 deg JAWS on a circular track
    //    in a parallel plane 24 mm inboard. Retracted, a jaw lies under the saddle; it rotates up round the module axis until
    //    saddle + jaws wrap 240 deg (the opening at the top is narrower than the module) and two rubber pads squeeze it.
    //    No lead screws: the air suspension kneels the rover under the module and rises to lift it (js/module_carry.js).
    // =========================================================================
    var cargoBay = new THREE.Group(); root.add(cargoBay);
    var MCr = global.NASA_ModuleCarry, CRd = MCr.CRADLE, MDd = MCr.MODULE;
    var MDr = MDd.R, AXY = CRd.axisY;
    var RIN = CRd.saddleRim, RT = CRd.ringT, RMID = RIN + RT;                 // ring centre-line radius
    var rad = Math.PI / 180, SAD = CRd.saddleArcDeg, JAW = CRd.jawArcDeg, STOW = CRd.jawStowDeg;
    var ringDefs = [];
    [1, -1].forEach(function (sg) {
      var RX = sg * CRd.ringX, JX = -sg * 0.024;                              // jaws run in a plane 24 mm inboard of the saddle
      // ring frame: origin on the module axis, local x = rover -z, local y = up, local z = rover x (the ring's axis)
      var ring = new THREE.Group(); ring.position.set(RX, AXY, 0); ring.rotation.y = Math.PI / 2; cargoBay.add(ring);
      var sad = new THREE.Group(); ring.add(sad);
      var arcS = new THREE.Mesh(new THREE.TorusGeometry(RMID, RT, 12, 40, SAD * rad), mAlu);
      arcS.rotation.z = (-90 - SAD / 2) * rad; arcS.castShadow = true; sad.add(arcS);
      [-40, 0, 40].forEach(function (dg) {                                    // three rollers in the V
        var an = (-90 + dg) * rad; cyl(0.0065, 0.0065, 0.030, mSteel, Math.cos(an) * (RIN + 0.0065), Math.sin(an) * (RIN + 0.0065), 0, sad, 'z', 14, true);
      });
      // three posts from the deck to the saddle: centre and +-0.15 m (all below the module surface)
      [[0, -RMID - RT], [0.15, -Math.sqrt(RMID * RMID - 0.0225) - RT], [-0.15, -Math.sqrt(RMID * RMID - 0.0225) - RT]].forEach(function (pp) {
        var top = AXY + pp[1] + 0.004, hgt = top - DY;
        rbox(0.020, hgt, 0.020, 0.004, mAluDark, RX, DY + hgt / 2, -pp[0], cargoBay, true);
      });
      rbox(0.026, 0.006, 0.330, 0.003, mAluDark, RX, DY + 0.003, 0, cargoBay, true);                      // foot rail bolted to the deck
      // jaws: each a group rotating about the ring axis; closed it covers -30..+30 deg (port) or 150..210 (starboard)
      var jaws = [], padDefs = [];
      [1, -1].forEach(function (side) {
        var jg = new THREE.Group(); jg.position.set(0, 0, JX); ring.add(jg);                              // ring local z = rover x
        var base = side > 0 ? -JAW / 2 : 180 - JAW / 2;
        var arcJ = new THREE.Mesh(new THREE.TorusGeometry(RMID, RT * 0.9, 12, 24, JAW * rad), mPaint);
        arcJ.rotation.z = base * rad; arcJ.castShadow = true; jg.add(arcJ);
        [base + 14, base + JAW - 14].forEach(function (an) {                 // two pads per jaw, each closes along its normal
          var a2 = an * rad, nx = Math.cos(a2), ny = Math.sin(a2), r0 = RIN - 0.004;
          var pd = rbox(0.008, 0.032, 0.024, 0.002, mRubber, nx * r0, ny * r0, 0, jg, true); pd.rotation.z = a2;
          pd.userData = { x0: nx * r0, y0: ny * r0, nx: nx, ny: ny, travel: Math.max(0, r0 - 0.004 - MDr - 0.0005) };
          padDefs.push(pd);
        });
        var tip = cyl(0.007, 0.007, 0.020, mBlue, Math.cos((base + (side > 0 ? JAW : 0)) * rad) * RMID, Math.sin((base + (side > 0 ? JAW : 0)) * rad) * RMID, 0, jg, 'z', 12);
        jaws.push({ jaw: jg, side: side });
      });
      // jaw drive: worm gearmotor + pinion on the track, under the saddle (one per jaw)
      [1, -1].forEach(function (k) { rbox(0.030, 0.026, 0.030, 0.005, mGraphite, RX + JX * 0.5, AXY - RMID - 0.010, k * 0.07, cargoBay, true); });
      var led = cyl(0.004, 0.004, 0.003, mLedRed, 0, -RMID - RT - 0.004, 0.012 * sg, sad, 'z', 10);
      ringDefs.push({ ring: ring, saddle: sad, lid: jaws[0].jaw, jaws: jaws, led: led, pads: padDefs, x: RX });
    });
    // module seat (parent for the module mesh while carried): axis at CRd.axisY, centre over the base centre
    var moduleSeat = new THREE.Object3D(); moduleSeat.name = 'moduleSeat'; moduleSeat.position.set(CRd.seatX, AXY, 0); cargoBay.add(moduleSeat);
    global.cargoBaySlots = [moduleSeat];
    var ledOn = new THREE.MeshStandardMaterial({ color: 0x22c55e, emissive: 0x16a34a, emissiveIntensity: 2.0 });
    global.moduleCradle = {
      seat: moduleSeat, rings: ringDefs, railY: CRd.standY, seatX: CRd.seatX,
      /** apply a NASA_ModuleCarry state: jaw closure lid 0..1, pad squeeze 0..1 (the saddles are fixed; the suspension lifts) */
      apply: function (S) {
        var lid = Math.max(0, Math.min(1, S.lid || 0)), sq = Math.max(0, Math.min(1, S.squeeze || 0));
        var ease = global.NASA_Motion ? global.NASA_Motion.minJerk(lid) : lid * lid * (3 - 2 * lid);   // minimum-jerk start and stop of the jaw rotation (zero speed and acceleration at both ends)
        for (var i = 0; i < ringDefs.length; i++) {
          var r = ringDefs[i];
          for (var ji = 0; ji < r.jaws.length; ji++) r.jaws[ji].jaw.rotation.z = -r.jaws[ji].side * STOW * rad * (1 - ease);   // retracted under the saddle
          for (var pi = 0; pi < r.pads.length; pi++) { var pq = r.pads[pi].userData; r.pads[pi].position.set(pq.x0 - pq.nx * pq.travel * sq, pq.y0 - pq.ny * pq.travel * sq, 0); }
          r.led.material = S.locked ? ledOn : mLedRed;
        }
      }
    };
    global.moduleCradle.apply({ lid: 0, squeeze: 0, locked: false });
    global.cargoBayGroup = cargoBay; window.cargoBayGroup = cargoBay;
    var cargoLid = new THREE.Group(); cargoLid.name = 'cargoLidGroup'; root.add(cargoLid);                  // (kept for the viewer API)

    // =========================================================================
    // 8. DETAILS — guarded E-stop (aft port deck corner), status light bar, CoM marker
    // =========================================================================
    var EX = -0.250, EZ = 0.150;
    rbox(0.040, 0.012, 0.040, 0.006, mGraphite, EX, DY + 0.006, EZ, root);
    cyl(0.014, 0.012, 0.012, mLedRed, EX, DY + 0.018, EZ, root, null, 24);
    rbox(0.006, 0.006, 0.36, 0.003, mLedWhite, 0.268, DY - 0.006, 0, root);                               // forward light strip
    var com = (global.NASA_RoverBOM && NASA_RoverBOM.com) ? NASA_RoverBOM.com() : { x: 0, y: 0.15 };
    var comMarker = new THREE.Group(); comMarker.position.set(com.x || 0, (com.y || 0.15) + 0.20, 0); root.add(comMarker);
    cyl(0.026, 0.026, 0.003, mAmber, 0, 0, 0, comMarker, null, 24);
    box(0.060, 0.004, 0.004, mBlue, 0, 0, 0, comMarker); box(0.004, 0.004, 0.060, mBlue, 0, 0, 0, comMarker);
    comMarker.visible = false;
    global.comMarkerGroup = comMarker;

    // ── expose handles to the simulator runtime ──────────────────────────────
    global.lidarHead = lidarHead; global.lidarHeads = lidarHeads;                                     // both scanners; the simulator spins every one
    global.fanRotors = (Array.isArray(global.fanRotors) ? global.fanRotors : []).concat(fanRotors);      // keep the viewer's own rotors
    global.headlights = (Array.isArray(global.headlights) ? global.headlights : []).concat(headlights);  // spotlights + LED lenses toggle together
    global.headlightLeds = headlights;
    global.avLidGroup = avLid;
    global.cargoLidGroup = cargoLid;
    global.upperShellGroup = upperShell;
    global.sensorMastGroup = mastPivot;                                                              // inert node: the scanners are fixed
    global.ROVER_GEOM = GEOM;
    window.roverArm = null;          // v7: the grabber arm is removed (modules are carried by the ring clamps)
    root.userData.internals = internals;

    /* motion rig for the CAD pages (the simulator runs its own physics and drives the same groups)
       roll(d)         spin all tyres for d metres of travel
       setTravel(a)    per-wheel suspension travel (m, + = wheel up), clamped ±30 mm
       setSteer(a)     no-op (skid steer since v6)
       updateShocks()  no-op unless shockLinks is filled (no coil-overs since v6) */
    var _up = new THREE.Vector3(0, 1, 0), _a = new THREE.Vector3(), _b = new THREE.Vector3(), _d = new THREE.Vector3();
    var finishes = {
      flight: { paint: 0xf4f6f8, shade: 0xdfe4ea, map: paintMap, rough: 0.32, clear: 1.0 },
      petg: { paint: 0x1c1f24, shade: 0x2a2e35, map: null, rough: 0.46, clear: 0.35 }
    };
    var rig = {
      root: root, wheelGroups: wheelGroups, tires: wheels, R: R, finish: 'flight', geom: GEOM,
      roll: function (d) { for (var i = 0; i < wheels.length; i++) wheels[i].rotation.y -= d / R; },
      setTravel: function (arr) {
        for (var i = 0; i < wheelGroups.length; i++) {
          var tr = arr ? (arr[i] || 0) : 0;
          wheelGroups[i].group.position.y = AY + Math.max(-0.03, Math.min(0.03, tr));
        }
        rig.updateShocks();
      },
      setSteer: function () { /* v6+ skid steer: the wheels do not steer */ },
      updateShocks: function () {
        for (var i = 0; i < shockLinks.length; i++) {
          var L = shockLinks[i], par = L.shock.parent; if (!par) continue;
          L.upper.getWorldPosition(_a); L.lower.getWorldPosition(_b); par.worldToLocal(_a); par.worldToLocal(_b);
          var dx = _a.x - _b.x, dy = _a.y - _b.y, dz = _a.z - _b.z, len = Math.max(0.01, Math.sqrt(dx * dx + dy * dy + dz * dz));
          _d.x = dx / len; _d.y = dy / len; _d.z = dz / len;
          L.shock.position.set((_a.x + _b.x) / 2, (_a.y + _b.y) / 2, (_a.z + _b.z) / 2);
          L.shock.quaternion.setFromUnitVectors(_up, _d);
          L.shock.scale.set(1, len / (L.rest || 0.03), 1);
        }
      },
      setFinish: function (k) {
        var f = finishes[k] || finishes.flight; rig.finish = finishes[k] ? k : 'flight';
        mPaint.color.setHex(f.paint); mPaint.map = f.map; mPaint.roughness = f.rough; mPaint.clearcoat = f.clear; mPaint.needsUpdate = true;
        mPaintShade.color.setHex(f.shade); mPaintShade.roughness = f.rough + 0.04; mPaintShade.needsUpdate = true;
      },
      materials: { paint: mPaint, paintShade: mPaintShade, foil: mFoil, alu: mAlu }
    };
    root.userData.rig = rig; global.roverRig = rig;
    root.updateMatrixWorld(true); rig.updateShocks();
    return Promise.resolve(root);
  }

  function chassisPlanformM() {
    return { chL: 2 * GEOM.halfLen, chW: 2 * GEOM.halfWid };
  }

  global.NASA_FusionRover = {
    init: initFusionRover, GEOM: GEOM,
    loadManifest: function () {
      return Promise.resolve({ roverRadius: GEOM.collR, modelCenter: { x: 0, y: 0, z: 0 } });
    }
  };
})(typeof window !== 'undefined' ? window : globalThis);
