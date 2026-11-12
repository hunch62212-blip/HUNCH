/* Shared 3D models of the cargo module and its skid stand (v11.1): one builder for the simulator (js/module_sortie.js) and the rover CAD
 * page (rover_model.html), so both always show the same hardware. Needs THREE. MOD / CR come from js/module_carry.js.
 *   NASA_ModuleModels.buildModuleMesh()            -> THREE.Group, axis along local x, centre at the origin
 *   NASA_ModuleModels.buildStand(label)            -> THREE.Group, local y = 0 is the module bottom plane (not added to a scene)
 *   NASA_ModuleModels.placeStand(g, x, z, yB, getH) -> puts the stand at (x, yB, z) with its legs reaching the ground getH(x, z)
 */
(function (root) {
  'use strict';
  var MC = root.NASA_ModuleCarry, MOD = MC.MODULE, CR = MC.CRADLE;
  // ------------------------------------------------------------------------------------------------- meshes
  function mat(c, r, m, e) { return new THREE.MeshStandardMaterial({ color: c, roughness: r == null ? 0.5 : r, metalness: m == null ? 0.2 : m, emissive: e || 0x000000 }); }
  var _tex = {};
  /* Crinkled MLI foil: soft random creases drawn on a canvas, used as a bump map (and a faint roughness map) so the silver
     blanket breaks up the reflections the way the reference image does. Procedural, no image files. */
  function foilTex() {
    if (_tex.foil) return _tex.foil;
    var c = document.createElement('canvas'); c.width = c.height = 256; var g = c.getContext('2d');
    g.fillStyle = '#808080'; g.fillRect(0, 0, 256, 256);
    var a = 7; function rnd() { a = (a * 1664525 + 1013904223) >>> 0; return a / 4294967296; }
    // v11.1: soft, long creases like a stretched blanket (the reference image), not crumpled foil: fewer, wider, blurred strokes
    if ('filter' in g) g.filter = 'blur(2.5px)';
    for (var i = 0; i < 70; i++) {
      var x = rnd() * 256, y = rnd() * 256, len = 40 + rnd() * 110, ang = (rnd() - 0.5) * 0.9;     // mostly along the hull (u), gently curved
      g.strokeStyle = rnd() > 0.5 ? 'rgba(255,255,255,' + (0.08 + rnd() * 0.14) + ')' : 'rgba(0,0,0,' + (0.08 + rnd() * 0.14) + ')';
      g.lineWidth = 3 + rnd() * 7; g.beginPath(); g.moveTo(x, y);
      g.quadraticCurveTo(x + Math.cos(ang + 0.3) * len * 0.5, y + Math.sin(ang + 0.3) * len * 0.5, x + Math.cos(ang) * len, y + Math.sin(ang) * len); g.stroke();
    }
    if ('filter' in g) g.filter = 'none';
    var t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(2, 1);
    if (typeof renderer !== 'undefined') t.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
    return (_tex.foil = t);
  }
  function textTex(lines, w, h, bg, fg) {
    var c = document.createElement('canvas'); c.width = w; c.height = h; var g = c.getContext('2d');
    if (bg) { g.fillStyle = bg; g.fillRect(0, 0, w, h); } else g.clearRect(0, 0, w, h);
    g.fillStyle = fg || '#1b1f24'; g.textBaseline = 'middle';
    lines.forEach(function (l) { g.font = l.font; g.fillText(l.text, l.x, l.y); });
    var t = new THREE.CanvasTexture(c); if (typeof renderer !== 'undefined') t.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy()); return t;
  }
  /* A curved patch on the hull: a cylinder segment along local x at radius r, centred at angle a (rad, 0 = top, + = toward +z),
     spanning da (rad) and length len along x. Curved, so panels and decals sit on the hull instead of sinking into it. */
  function hullPatch(r, a, da, x, len, m, segs) {
    var geo = new THREE.CylinderGeometry(r, r, len, segs || 16, 1, true, a - Math.PI / 2 - da / 2, da);   // three.js theta 0 is +z; after the roll below, theta = a - 90 deg is angle a
    var me = new THREE.Mesh(geo, m); me.rotation.z = -Math.PI / 2; me.position.x = x; return me;   // cylinder y axis -> x
  }
  function onHull(r, a, x) { return new THREE.Vector3(x, Math.cos(a) * r, Math.sin(a) * r); }

  /* Cargo module after the reference image: silver MLI blanket, gold end rings, seam bands, a berthing hatch on the +x face,
     a probe on the -x face, a white cargo door on each side, gold handrails, a roof solar panel and two dishes. It lies along
     the rover (axis = local x). Clamp bands (wear sleeves) at x = +-bandX, four lift lugs at x = +-lugX for the skid stand. */
  function buildModuleMesh() {
    var g = new THREE.Group(); g.name = 'module40';
    var L = MOD.L, R = MOD.R, X0 = -L / 2 + MOD.probeL, X1 = L / 2 - MOD.collarL;
    var foil = new THREE.MeshPhysicalMaterial({ color: 0xd8dde3, metalness: 1.0, roughness: 0.22, bumpMap: foilTex(), bumpScale: 0.9, clearcoat: 0.35, clearcoatRoughness: 0.25, envMapIntensity: 1.35 });
    var faceM = new THREE.MeshPhysicalMaterial({ color: 0xb9c0c8, metalness: 1.0, roughness: 0.3, clearcoat: 0.3, envMapIntensity: 1.3 });   // smooth end faces (the bump map needs cylinder UVs)
    var foilB = new THREE.MeshPhysicalMaterial({ color: 0xc4cad1, metalness: 1.0, roughness: 0.30, bumpMap: foilTex(), bumpScale: 0.7, clearcoat: 0.2, envMapIntensity: 1.2 });
    var gold = new THREE.MeshPhysicalMaterial({ color: 0xc9a23a, metalness: 1.0, roughness: 0.28, clearcoat: 0.5, clearcoatRoughness: 0.2, envMapIntensity: 1.4 });
    var white = new THREE.MeshPhysicalMaterial({ color: 0xeef1f4, metalness: 0.05, roughness: 0.42, clearcoat: 0.6, clearcoatRoughness: 0.2 });
    var steel = new THREE.MeshPhysicalMaterial({ color: 0xb7bec7, metalness: 1.0, roughness: 0.2, envMapIntensity: 1.3 });
    var dark = mat(0x20262e, 0.55, 0.5), sealPtfe = mat(0xd0d4dc, 0.45, 0.15); // cryo spring-energized PTFE / metallic wiper seal (replaces rubber)
    var cell = new THREE.MeshPhysicalMaterial({ color: 0x0d1b3a, metalness: 0.4, roughness: 0.18, clearcoat: 1.0, clearcoatRoughness: 0.04, envMapIntensity: 1.2 });
    // hull: three blanket bays between seams, slightly different sheen like the reference
    var bays = [[X0, -0.13, foil], [-0.13, 0.13, foilB], [0.13, X1, foil]];
    bays.forEach(function (b) {
      var len = b[1] - b[0], cy = new THREE.Mesh(new THREE.CylinderGeometry(R, R, len, 64, 1, true), b[2]);
      cy.rotation.z = Math.PI / 2; cy.position.x = (b[0] + b[1]) / 2; cy.castShadow = cy.receiveShadow = true; g.add(cy);
    });
    [-0.13, 0.13].forEach(function (x) { var sm = new THREE.Mesh(new THREE.TorusGeometry(R + 0.0008, 0.0016, 6, 64), dark); sm.rotation.y = Math.PI / 2; sm.position.x = x; g.add(sm); });
    for (var k = 0; k < 6; k++) {                                                                   // longitudinal blanket seams
      var sa = k / 6 * Math.PI * 2 + 0.26, sp = onHull(R + 0.0006, sa, 0);
      var ls = new THREE.Mesh(new THREE.BoxGeometry(X1 - X0, 0.0012, 0.0016), dark); ls.position.set((X0 + X1) / 2, sp.y, sp.z); ls.rotation.x = -sa; g.add(ls);
    }
    // gold end rings and the two end faces
    [[X1, 1], [X0, -1]].forEach(function (e) {
      var rg = new THREE.Mesh(new THREE.TorusGeometry(R + 0.002, 0.007, 10, 64), gold); rg.rotation.y = Math.PI / 2; rg.position.x = e[0]; rg.castShadow = true; g.add(rg);
      var cap = new THREE.Mesh(new THREE.CylinderGeometry(R * 0.995, R, e[1] > 0 ? MOD.collarL : MOD.probeL, 64), faceM);
      cap.rotation.z = Math.PI / 2; cap.position.x = e[0] + e[1] * (e[1] > 0 ? MOD.collarL : MOD.probeL) / 2; cap.castShadow = true; g.add(cap);
    });
    var FX = L / 2 + 0.0005;                                                                          // +x face: berthing ring, hatch, cargo plate, thrusters
    var face = new THREE.Mesh(new THREE.CircleGeometry(R * 0.985, 64), faceM); face.rotation.y = Math.PI / 2; face.position.x = FX; g.add(face);
    var berth = new THREE.Mesh(new THREE.TorusGeometry(R * 0.62, 0.010, 10, 56), steel); berth.rotation.y = Math.PI / 2; berth.position.x = FX + 0.004; berth.name = 'berthingRing'; g.add(berth);
    var hatch = new THREE.Mesh(new THREE.CylinderGeometry(R * 0.50, R * 0.50, 0.010, 48), white); hatch.rotation.z = Math.PI / 2; hatch.position.x = FX + 0.004; g.add(hatch);
    [0.30, 0.42].forEach(function (rr) { var hr = new THREE.Mesh(new THREE.TorusGeometry(R * rr, 0.0018, 6, 40), dark); hr.rotation.y = Math.PI / 2; hr.position.x = FX + 0.0095; g.add(hr); });
    var wheelH = new THREE.Mesh(new THREE.TorusGeometry(R * 0.10, 0.0028, 6, 20), steel); wheelH.rotation.y = Math.PI / 2; wheelH.position.x = FX + 0.011; g.add(wheelH);
    var dustRing = new THREE.Mesh(new THREE.TorusGeometry(R * 0.62, 0.0045, 6, 56), sealPtfe); dustRing.rotation.y = Math.PI / 2; dustRing.position.x = FX + 0.013; dustRing.name = 'dustCapCollar'; g.add(dustRing);   // cryo PTFE/Inconel seal on the berthing ring rim
    var plateT = textTex([{ text: 'CARGO', font: '700 46px Helvetica, Arial', x: 26, y: 40 }, { text: '⚠ HANDLE BY LUGS', font: '600 22px Helvetica, Arial', x: 26, y: 92 }], 256, 128, '#eef1f4', '#22272e');
    var plate = new THREE.Mesh(new THREE.PlaneGeometry(0.075, 0.038), new THREE.MeshStandardMaterial({ map: plateT, roughness: 0.5 })); plate.rotation.y = Math.PI / 2; plate.position.set(FX + 0.001, -R * 0.74, R * 0.32); g.add(plate);
    for (var q = 0; q < 4; q++) {                                                                     // four reaction-control nozzles on the rim
      var qa = q / 4 * Math.PI * 2 + Math.PI / 4, nz = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.011, 0.022, 14, 1, true), steel);
      nz.rotation.z = -Math.PI / 2; nz.position.set(FX + 0.008, Math.cos(qa) * R * 0.86, Math.sin(qa) * R * 0.86); g.add(nz);
    }
    var BX = -L / 2 - 0.0005;                                                                         // -x face: probe and its dust cap
    var faceB = new THREE.Mesh(new THREE.CircleGeometry(R * 0.985, 64), faceM); faceB.rotation.y = -Math.PI / 2; faceB.position.x = BX; g.add(faceB);
    var probe = new THREE.Mesh(new THREE.CylinderGeometry(R * 0.10, R * 0.34, 0.026, 32), steel); probe.rotation.z = Math.PI / 2; probe.position.x = BX - 0.013; probe.castShadow = true; g.add(probe);
    var capB = new THREE.Mesh(new THREE.CylinderGeometry(R * 0.11, R * 0.11, 0.008, 24), mat(0x0b4fb3, 0.4, 0.3)); capB.rotation.z = Math.PI / 2; capB.position.x = BX - 0.028; capB.name = 'dustCapProbe'; g.add(capB);
    // side cargo doors (white, curved) with a window plate and hinge strip, one each side
    [1, -1].forEach(function (s) {
      var a = s * Math.PI / 2, door = hullPatch(R + 0.0015, a, 0.62, 0.185, 0.135, white, 20); g.add(door);
      var frame = hullPatch(R + 0.0010, a, 0.70, 0.185, 0.150, dark, 20); g.add(frame);
      var dT = textTex([{ text: 'CARGO', font: '700 40px Helvetica, Arial', x: 70, y: 34 }, { text: 'CAUTION', font: '700 22px Helvetica, Arial', x: 86, y: 98 }], 256, 128, null, '#2a2f36');
      dT.center.set(0.5, 0.5); dT.rotation = s * Math.PI / 2;                                         // text runs along the module
      var lab = hullPatch(R + 0.0021, a, 0.30, 0.185, 0.070, new THREE.MeshStandardMaterial({ map: dT, transparent: true, roughness: 0.5 }), 12); g.add(lab);
      var win = new THREE.Mesh(new THREE.CylinderGeometry(0.011, 0.011, 0.004, 20), mat(0x0c1a2b, 0.1, 0.6)); var wp = onHull(R + 0.003, a + s * 0.0, 0.155); win.position.copy(wp); win.position.y -= 0.012; win.rotation.x = s * Math.PI / 2; g.add(win);
      for (var h = 0; h < 3; h++) { var hg = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.004, 0.004), steel); hg.position.copy(onHull(R + 0.002, a - s * 0.31, 0.135 + h * 0.05)); hg.rotation.x = a - s * 0.31; g.add(hg); }   // door hinges on its upper edge
      // decal strip (module name) aft of the door
      var nT = textTex([{ text: 'LUNAR CARGO MODULE', font: '700 44px Helvetica, Arial', x: 14, y: 40 }, { text: '40FT-LOGISTICS-01  ·  1:' + MOD.scale + '  ·  ' + MOD.kg.toFixed(1) + ' kg', font: '500 26px Helvetica, Arial', x: 16, y: 92 }], 512, 128, null, '#15191e');
      nT.center.set(0.5, 0.5); nT.rotation = s * Math.PI / 2;
      var nm = hullPatch(R + 0.0019, a, 0.30, -0.15, 0.200, new THREE.MeshStandardMaterial({ map: nT, transparent: true, roughness: 0.5 }), 12); g.add(nm);
    });
    // gold handrails along both upper shoulders, on standoffs
    [0.95, -0.95].forEach(function (a) {
      var rp = onHull(R + 0.016, a, 0), rl = new THREE.Mesh(new THREE.CylinderGeometry(0.0030, 0.0030, X1 - X0 - 0.12, 10), gold);
      rl.rotation.z = Math.PI / 2; rl.position.set(0, rp.y, rp.z); g.add(rl);
      for (var so = 0; so < 5; so++) { var sx = -0.27 + so * 0.135, sp2 = onHull(R + 0.008, a, sx); var st = new THREE.Mesh(new THREE.CylinderGeometry(0.0022, 0.0022, 0.016, 6), gold); st.position.copy(sp2); st.rotation.x = a; g.add(st); }
    });
    // roof: solar panel (two strings of cells) and two small dishes, as in the reference
    var sol = new THREE.Group(); sol.position.set(0.12, R + 0.010, 0); g.add(sol);
    var sb = new THREE.Mesh(new THREE.BoxGeometry(0.17, 0.006, 0.11), steel); sol.add(sb);
    for (var ci = 0; ci < 8; ci++) for (var cj = 0; cj < 4; cj++) { var cc = new THREE.Mesh(new THREE.BoxGeometry(0.019, 0.002, 0.024), cell); cc.position.set(-0.075 + ci * 0.0214, 0.004, -0.039 + cj * 0.026); sol.add(cc); }
    [[-0.04, 0.035], [0.04, -0.035]].forEach(function (pp) { var lg = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.012, 6), dark); lg.position.set(pp[0], -0.006, pp[1]); sol.add(lg); });
    [[-0.17, 0.05, 0.034], [-0.24, -0.04, 0.026]].forEach(function (d) {
      var dg = new THREE.Group(); dg.position.set(d[0], R + 0.002, d[1]); g.add(dg);
      var post = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.004, 0.035, 8), steel); post.position.y = 0.017; dg.add(post);
      var dish = new THREE.Mesh(new THREE.SphereGeometry(d[2], 20, 8, 0, Math.PI * 2, 0, 0.9), white); dish.material = white; dish.rotation.x = Math.PI; dish.position.y = 0.035 + d[2] * 0.95; dish.rotation.z = 0.5; dg.add(dish);
      var feed = new THREE.Mesh(new THREE.CylinderGeometry(0.0015, 0.0015, d[2] * 0.9, 6), dark); feed.position.y = 0.035 + d[2] * 0.55; dg.add(feed);
    });
    // clamp bands (wear sleeves) and their wiper lips; the ring pads grip here (v9 seal scheme)
    [1, -1].forEach(function (sg) {
      var bd = new THREE.Mesh(new THREE.CylinderGeometry(R + 0.0015, R + 0.0015, MOD.bandW, 64, 1, true), steel); bd.rotation.z = Math.PI / 2; bd.position.x = sg * MOD.bandX; g.add(bd);
      [-1, 1].forEach(function (e) { var wl = new THREE.Mesh(new THREE.TorusGeometry(R + 0.0022, 0.0020, 6, 64), sealPtfe); wl.rotation.y = Math.PI / 2; wl.position.x = sg * MOD.bandX + e * (MOD.bandW / 2 + 0.002); g.add(wl); });
    });
    // four gasketed access panels (captive screws), curved on the hull, low on the sides clear of the doors and bands
    [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(function (pp) {
      var a = pp[0] * 2.25, x = pp[1] * 0.275;
      g.add(hullPatch(R + 0.0012, a, 0.30, x, 0.075, dark, 10));
      g.add(hullPatch(R + 0.0018, a, 0.27, x, 0.068, white, 10));
      for (var b = 0; b < 8; b++) { var ba = a + (b % 2 ? 1 : -1) * 0.115, bx = x + (Math.floor(b / 2) - 1.5) * 0.019; var sp3 = onHull(R + 0.0026, ba, bx); var hx = new THREE.Mesh(new THREE.CylinderGeometry(0.0026, 0.0026, 0.002, 6), dark); hx.position.copy(sp3); hx.rotation.x = ba; g.add(hx); }
    });
    // four side lift lugs (slotted +/-3.4 mm CAD / +/-55 mm full-scale delta-CTE axial compliance on forward trunnions)
    [1, -1].forEach(function (sx) { [1, -1].forEach(function (sz) {
      var base = new THREE.Mesh(new THREE.BoxGeometry(sx > 0 ? 0.056 : 0.045, 0.045, 0.010), steel); base.position.set(sx * MOD.lugX, 0, sz * (R - 0.002)); g.add(base);
      var pin = new THREE.Mesh(new THREE.CylinderGeometry(MOD.lugR, MOD.lugR, MOD.lugZ + 0.014 - R, 16), steel); pin.rotation.x = Math.PI / 2; pin.position.set(sx * MOD.lugX, 0, sz * (R + MOD.lugZ + 0.014) / 2); pin.castShadow = true; g.add(pin);
      var collar = new THREE.Mesh(new THREE.CylinderGeometry(MOD.lugR * 1.6, MOD.lugR * 1.6, 0.005, 16), gold); collar.rotation.x = Math.PI / 2; collar.position.set(sx * MOD.lugX, 0, sz * (MOD.lugZ + 0.017)); g.add(collar);
    }); });
    g.traverse(function (o) { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    return g;
  }

  /* Skid stand after the reference image's frame: two side frames OUTSIDE the tyre track (posts at z = +-lugZ), each with two
     posts carrying a V notch for a lug, a top rail between them, splayed legs on dish foot pads and a low ground rail.
     Nothing crosses under the module, so the rover drives straight in along it. Local y = 0 is the module bottom plane. */
  function buildPallet(label) {
    var g = new THREE.Group(); g.userData.posts = [];
    var frame = new THREE.MeshPhysicalMaterial({ color: 0x8d949c, metalness: 0.9, roughness: 0.38, envMapIntensity: 1.2 });
    var frameD = new THREE.MeshPhysicalMaterial({ color: 0x5c636c, metalness: 0.85, roughness: 0.45 });
    var pad = new THREE.MeshPhysicalMaterial({ color: 0xb8bfc7, metalness: 1.0, roughness: 0.3 });
    var notchY = MOD.R - MOD.lugR;                                                                     // lug pin bottom
    [1, -1].forEach(function (s) {
      var Z = s * MOD.lugZ;
      [1, -1].forEach(function (sx) {
        var X = sx * MOD.lugX;
        var post = new THREE.Mesh(new THREE.BoxGeometry(0.030, 1, 0.030), frame); post.castShadow = true; g.add(post);
        g.userData.posts.push({ m: post, kind: 'post', x: X, z: Z, top: notchY });
        [1, -1].forEach(function (k) {                                                                 // the V notch: two horns
          var hn = new THREE.Mesh(new THREE.BoxGeometry(0.030, 0.020, 0.012), frameD); hn.position.set(X + k * 0.013, notchY + CR.hornH - 0.006, Z); hn.rotation.z = -k * 0.6; g.add(hn);
        });
        var leg = new THREE.Mesh(new THREE.BoxGeometry(0.020, 1, 0.020), frameD); leg.castShadow = true; g.add(leg);   // splayed leg to a foot pad
        g.userData.posts.push({ m: leg, kind: 'leg', x: X + sx * 0.05, z: Z + s * 0.10, fromX: X, fromZ: Z, fromY: notchY * 0.45 });
        var fp = new THREE.Mesh(new THREE.CylinderGeometry(0.030, 0.036, 0.010, 20), pad); g.add(fp);
        g.userData.posts.push({ m: fp, kind: 'pad', x: X + sx * 0.05, z: Z + s * 0.10 });
        var fp2 = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.028, 0.008, 20), pad); g.add(fp2);
        g.userData.posts.push({ m: fp2, kind: 'pad', x: X, z: Z });
      });
      var top = new THREE.Mesh(new THREE.BoxGeometry(2 * MOD.lugX + 0.03, 0.022, 0.024), frame); top.position.set(0, notchY - 0.030, Z); top.castShadow = true; g.add(top);
      var mid = new THREE.Mesh(new THREE.BoxGeometry(2 * MOD.lugX, 0.016, 0.018), frameD); g.add(mid);
      g.userData.posts.push({ m: mid, kind: 'rail', x: 0, z: Z, frac: 0.40 });
      for (var d = 0; d < 2; d++) {                                                                    // X-brace between the posts
        var br = new THREE.Mesh(new THREE.BoxGeometry(0.010, 1, 0.010), frameD); g.add(br);
        g.userData.posts.push({ m: br, kind: 'brace', z: Z, d: d ? 1 : -1 });
      }
    });
    // Regolith-proof pickup (js/bay_design.js PICKUP): compact sintered apron under the stand (< 16 m full-scale width) so approach stays dust-free,
    // flared lead-in guides, 321-SS welded bellows boots on every post foot, and Vespel/Al hinged dust shields over each V notch.
    var tile = new THREE.MeshStandardMaterial({ color: 0x4d5258, roughness: 0.92, metalness: 0.05 });
    var tileEdge = new THREE.MeshStandardMaterial({ color: 0x2f3338, roughness: 0.8, metalness: 0.2 });
    var bellowsMat = new THREE.MeshStandardMaterial({ color: 0x9ca3af, roughness: 0.42, metalness: 0.85 });
    var vespelMat = new THREE.MeshStandardMaterial({ color: 0x3b2f2f, roughness: 0.72, metalness: 0.1 });
    var AX = 2 * MOD.lugX + 0.56, AZ = 2 * MOD.lugZ + 0.24;                                                // compact apron (< 16 m full-scale outrigger footprint)
    var apron = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), tile); apron.receiveShadow = true; g.add(apron);
    g.userData.posts.push({ m: apron, kind: 'apron', sx: AX, sz: AZ });
    for (var jx = -4; jx <= 4; jx++) {                                                                  // tile joints
      var jt = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.0015, AZ), tileEdge); g.add(jt);
      g.userData.posts.push({ m: jt, kind: 'apronRel', x: jx * AX / 8.5, z: 0, dy: 0.0016 });
    }
    [1, -1].forEach(function (sx) { [1, -1].forEach(function (sz) {                                      // compact funnel wings within stand apron
      var w = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.045, 0.012), tileEdge); w.castShadow = true; g.add(w);
      w.rotation.y = -sx * sz * 0.28;
      g.userData.posts.push({ m: w, kind: 'apronRel', x: sx * (AX / 2 - 0.14), z: sz * (MOD.lugZ + 0.06), dy: 0.024 });
    }); });
    [1, -1].forEach(function (sz) { [1, -1].forEach(function (sx) {
      var bx0 = sx * MOD.lugX, bz0 = sz * MOD.lugZ;
      var boot = new THREE.Mesh(new THREE.CylinderGeometry(0.030, 0.042, 0.055, 18, 3), bellowsMat); boot.castShadow = true; g.add(boot);
      g.userData.posts.push({ m: boot, kind: 'apronRel', x: bx0, z: bz0, dy: 0.030 });
      var cap = new THREE.Mesh(new THREE.BoxGeometry(0.036, 0.004, 0.022), vespelMat); g.add(cap);       // Vespel cap hinged on the outboard edge of the notch
      cap.position.set(bx0, notchY + CR.hornH + 0.012, bz0 + sz * 0.020); cap.rotation.x = sz * 1.15;
    }); });
    var sc = document.createElement('canvas'); sc.width = 512; sc.height = 96; var x2 = sc.getContext('2d');
    x2.fillStyle = '#2f353d'; x2.fillRect(0, 0, 512, 96); x2.fillStyle = '#e6e9ec'; x2.font = '600 40px -apple-system, Helvetica, Arial'; x2.textAlign = 'center'; x2.textBaseline = 'middle'; x2.fillText(label, 256, 50);
    var sign = new THREE.Mesh(new THREE.PlaneGeometry(0.30, 0.056), new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(sc), side: THREE.DoubleSide })); sign.position.set(0, notchY - 0.07, MOD.lugZ + 0.014); g.add(sign);
    return g;
  }
  var _up = null;
  /* Place a stand with its module bottom plane at world height yB; posts, legs and pads reach the terrain. */
  function placePallet(g, x, z, yB, getH) {
    if (!_up) _up = new THREE.Vector3(0, 1, 0);
    g.position.set(x, yB, z); g.rotation.set(0, 0, 0);
    function gy(lx, lz) { return getH(x + lx, z + lz) - yB; }                                         // ground in stand-local y
    var aTop = gy(0, 0) + 0.003;                                                                       // apron surface 3 mm over the ground at the stand centre
    g.userData.posts.forEach(function (p) {
      var m = p.m;
      if (p.kind === 'apron') {                                                                        // a slab whose sides reach the lowest ground under it
        var lo = aTop; [-0.5, 0.5].forEach(function (a) { [-0.5, 0.5].forEach(function (b) { lo = Math.min(lo, gy(a * p.sx, b * p.sz)); }); });
        var th = aTop - lo + 0.02; m.scale.set(p.sx, th, p.sz); m.position.set(0, aTop - th / 2, 0);
      } else if (p.kind === 'apronRel') m.position.set(p.x, aTop + p.dy, p.z);
      else if (p.kind === 'post') { var b = gy(p.x, p.z), h = Math.max(0.02, p.top - b); m.scale.y = h; m.position.set(p.x, b + h / 2, p.z); }
      else if (p.kind === 'pad') m.position.set(p.x, gy(p.x, p.z) + 0.005, p.z);
      else if (p.kind === 'leg') {
        var a = new THREE.Vector3(p.fromX, p.fromY, p.fromZ), c = new THREE.Vector3(p.x, gy(p.x, p.z) + 0.010, p.z), dv = a.clone().sub(c), len = dv.length();
        m.scale.y = len; m.position.copy(a.clone().add(c).multiplyScalar(0.5)); m.quaternion.setFromUnitVectors(_up, dv.normalize());
      } else if (p.kind === 'rail') { var b2 = Math.max(gy(-MOD.lugX, p.z), gy(MOD.lugX, p.z)); m.position.set(0, b2 + (MOD.R - MOD.lugR - b2) * p.frac, p.z); }
      else if (p.kind === 'brace') {
        var y0 = Math.max(gy(-MOD.lugX, p.z), gy(MOD.lugX, p.z)) + 0.02, y1 = MOD.R - MOD.lugR - 0.045;
        var A = new THREE.Vector3(-MOD.lugX + 0.02, p.d > 0 ? y0 : y1, p.z), B = new THREE.Vector3(MOD.lugX - 0.02, p.d > 0 ? y1 : y0, p.z), dv2 = B.clone().sub(A);
        m.scale.y = dv2.length(); m.position.copy(A.clone().add(B).multiplyScalar(0.5)); m.quaternion.setFromUnitVectors(_up, dv2.normalize());
      }
    });
  }
  root.NASA_ModuleModels = { mat: mat, foilTex: foilTex, textTex: textTex, hullPatch: hullPatch, onHull: onHull,
    buildModuleMesh: buildModuleMesh, buildStand: buildPallet, placeStand: placePallet };
})(typeof window !== 'undefined' ? window : globalThis);
