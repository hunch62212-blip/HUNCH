// SPAN-40 visual model. Geometry follows src/sim/config.js so visuals match the physics.
import * as THREE from 'three';
import { ROVER } from '../sim/config.js';
import { MAT, labelTexture } from './props.js';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

// Merge all non-animated meshes under `group` into one mesh per material (draw-call reduction).
export function mergeStatic(group) {
  group.updateMatrixWorld(true);
  const inv = new THREE.Matrix4().copy(group.matrixWorld).invert();
  const buckets = new Map(), victims = [];
  group.traverse((o) => {
    if (!o.isMesh || o === group) return;
    for (let a = o; a && a !== group; a = a.parent) if (a.userData.dynamic) return;
    if (o.material.transparent || o.material.map && o.material.map.isCanvasTexture && o.geometry.type === 'PlaneGeometry') return; // keep labels
    const g = o.geometry.index ? o.geometry.clone() : o.geometry.clone();
    g.applyMatrix4(new THREE.Matrix4().multiplyMatrices(inv, o.matrixWorld));
    for (const k of Object.keys(g.attributes)) if (!['position', 'normal', 'uv'].includes(k)) g.deleteAttribute(k);
    if (!g.attributes.uv) g.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(g.attributes.position.count * 2), 2));
    const key = o.material.uuid;
    if (!buckets.has(key)) buckets.set(key, { mat: o.material, geos: [] });
    buckets.get(key).geos.push(g.index ? g.toNonIndexed() : g);
    victims.push(o);
  });
  for (const v of victims) v.parent.remove(v);
  for (const { mat, geos } of buckets.values()) {
    const m = new THREE.Mesh(mergeGeometries(geos, false), mat); m.castShadow = true; m.receiveShadow = true; group.add(m);
  }
}

const F = ROVER.frame, B = ROVER.bogie, W = ROVER.wheel;
const paint = new THREE.MeshStandardMaterial({ color: 0xcdcdc6, roughness: 0.55, metalness: 0.08 });
const paintDark = new THREE.MeshStandardMaterial({ color: 0x3a3f45, roughness: 0.5, metalness: 0.5 });
const accent = new THREE.MeshStandardMaterial({ color: 0xd65a1a, roughness: 0.5, metalness: 0.2 });
const titanium = new THREE.MeshStandardMaterial({ color: 0xa9adb3, roughness: 0.32, metalness: 0.9 });
const solar = new THREE.MeshStandardMaterial({ color: 0xffffff, map: solarTexture(), roughness: 0.22, metalness: 0.55 });
const radiator = new THREE.MeshStandardMaterial({ color: 0xf6f6f6, roughness: 0.2, metalness: 0.2 });

function solarTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 512; const g = c.getContext('2d');
  g.fillStyle = '#0a1634'; g.fillRect(0, 0, 512, 512);
  for (let y = 0; y < 12; y++) for (let x = 0; x < 10; x++) {
    g.fillStyle = `rgb(${18 + (x * 7 + y * 3) % 9},${34 + (x + y) % 7},${78 + (x * 5) % 13})`;
    g.fillRect(4 + x * 50.8, 4 + y * 42.3, 47, 39);
    g.strokeStyle = 'rgba(200,210,230,0.35)'; g.lineWidth = 1;
    for (let k = 1; k < 4; k++) { g.beginPath(); g.moveTo(4 + x * 50.8 + k * 11.7, 4 + y * 42.3); g.lineTo(4 + x * 50.8 + k * 11.7, 43 + y * 42.3); g.stroke(); }
  }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; return t;
}

function hazardTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 128; const g = c.getContext('2d');
  g.fillStyle = '#e8b31e'; g.fillRect(0, 0, 128, 128); g.fillStyle = '#151515';
  for (let i = -128; i < 256; i += 32) { g.beginPath(); g.moveTo(i, 0); g.lineTo(i + 16, 0); g.lineTo(i + 144, 128); g.lineTo(i + 128, 128); g.closePath(); g.fill(); }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}

function meshTireTexture() {
  // woven spring-steel mesh with titanium chevron treads (Apollo LRV / NASA spring-tyre heritage)
  const c = document.createElement('canvas'); c.width = 1024; c.height = 256; const g = c.getContext('2d');
  g.clearRect(0, 0, 1024, 256);
  g.strokeStyle = 'rgba(205,208,212,1)'; g.lineWidth = 2.2;
  for (let i = -256; i < 1024 + 256; i += 14) {
    g.beginPath(); g.moveTo(i, 0); g.lineTo(i + 256, 256); g.stroke();
    g.beginPath(); g.moveTo(i + 256, 0); g.lineTo(i, 256); g.stroke();
  }
  g.fillStyle = 'rgba(150,154,160,1)';
  for (let k = 0; k < 24; k++) {
    const x = k * (1024 / 24);
    g.save(); g.translate(x + 20, 128);
    g.beginPath(); g.moveTo(-14, -110); g.lineTo(6, 0); g.lineTo(-14, 110); g.lineTo(-4, 110); g.lineTo(16, 0); g.lineTo(-4, -110); g.closePath(); g.fill();
    g.restore();
  }
  g.fillStyle = 'rgba(120,124,130,1)'; g.fillRect(0, 0, 1024, 10); g.fillRect(0, 246, 1024, 10);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; t.wrapS = THREE.RepeatWrapping; return t;
}
const tireMat = new THREE.MeshStandardMaterial({ map: meshTireTexture(), transparent: false, alphaTest: 0.35, side: THREE.DoubleSide, roughness: 0.35, metalness: 0.85 });

const mk = (geo, mat, cast = true) => { const m = new THREE.Mesh(geo, mat); m.castShadow = cast; m.receiveShadow = true; return m; };
const boxM = (w, h, d, mat) => mk(new THREE.BoxGeometry(w, h, d), mat);
const cylM = (r1, r2, h, mat, seg = 24) => mk(new THREE.CylinderGeometry(r1, r2, h, seg), mat);
function strut(a, b, r, mat) { // cylinder between two points
  const d = new THREE.Vector3().subVectors(b, a), L = d.length();
  const m = cylM(r, r, L, mat, 8);
  m.position.copy(a).addScaledVector(d, 0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  return m;
}

function trussGirder(len, h, w) {
  const g = new THREE.Group();
  const chord = 0.16;
  for (const y of [h / 2 - chord / 2, -h / 2 + chord / 2]) for (const z of [w / 2 - 0.06, -w / 2 + 0.06]) { const c = boxM(len, chord, 0.12, paint); c.position.set(0, y, z); g.add(c); }
  const bays = Math.round(len / 1.2);
  for (let i = 0; i <= bays; i++) {
    const x = -len / 2 + i * len / bays;
    for (const z of [w / 2 - 0.06, -w / 2 + 0.06]) {
      const p = boxM(0.1, h - chord, 0.1, paint); p.position.set(x, 0, z); g.add(p);
      if (i < bays) g.add(strut(new THREE.Vector3(x, -h / 2 + chord, z), new THREE.Vector3(x + len / bays, h / 2 - chord, z), 0.035, paint));
    }
    const tie = boxM(0.08, 0.08, w - 0.1, paintDark); tie.position.set(x, h / 2 - chord / 2, 0); g.add(tie);
  }
  return g;
}

export class RoverModel {
  constructor(scene) {
    this.root = new THREE.Group();
    scene.add(this.root);
    const R = this.root;
    // ---- frame
    for (const sz of [-1, 1]) {
      const gdr = trussGirder(F.length, F.girderH, F.girderW); gdr.position.set(0, F.girderY, sz * F.girderZ); R.add(gdr);
      const deck = boxM(F.length - 1.2, 0.04, F.girderW - 0.06, MAT.gold); deck.position.set(0, F.girderY + F.girderH / 2 + 0.02, sz * F.girderZ); R.add(deck);
      const lbl = new THREE.Mesh(new THREE.PlaneGeometry(4.4, 0.55), new THREE.MeshBasicMaterial({ map: labelTexture('SPAN-40  ·  LLASO-P2', { color: '#202428', size: 58, w: 1024 }), transparent: true }));
      lbl.position.set(0.4, F.girderY, sz * (F.girderZ + F.girderW / 2 + 0.01)); if (sz < 0) lbl.rotation.y = Math.PI; R.add(lbl);
    }
    for (const sx of [-1, 1]) {
      const cb = boxM(0.84, 0.66, F.girderZ * 2 + 0.3, paint); cb.position.set(sx * F.crossX, F.crossBottomY + 0.33, 0); R.add(cb);
      const stripe = boxM(0.86, 0.12, F.girderZ * 2 + 0.32, accent); stripe.position.set(sx * F.crossX, F.crossBottomY + 0.08, 0); R.add(stripe);
    }
    // avionics + battery thermal enclosures (gold MLI), radiators
    const av = boxM(1.6, 0.55, 2.4, MAT.gold); av.position.set(F.crossX, F.crossBottomY + 0.95, 0); R.add(av);
    const rad = boxM(0.04, 1.1, 2.2, radiator); rad.position.set(F.crossX - 0.85, F.crossBottomY + 1.2, 0); R.add(rad);
    const av2 = boxM(1.4, 0.5, 2.0, MAT.gold); av2.position.set(-F.crossX, F.crossBottomY + 0.92, 0); R.add(av2);
    // ---- legs, battery/motor pods
    this.bogieVis = [];
    for (const sx of [1, -1]) for (const sz of [-1, 1]) {
      const leg = boxM(0.64, 2.2, 0.52, paint); leg.position.set(sx * B.x, 0.5, sz * B.z); R.add(leg);
      const legStripe = boxM(0.66, 0.1, 0.54, accent); legStripe.position.set(sx * B.x, -0.1, sz * B.z); R.add(legStripe);
      const pod = boxM(1.5, 0.9, 0.64, MAT.gold); pod.position.set(sx * (B.x - 0.1), -0.25, sz * (B.z + 0.25)); R.add(pod);
      const podRad = boxM(1.3, 0.7, 0.03, radiator); podRad.position.set(sx * (B.x - 0.1), -0.25, sz * (B.z + 0.58)); R.add(podRad);
      const hp = new THREE.Vector3(sx * B.x, B.hardpointY, sz * B.z);
      // strut assembly (animated): damper body fixed, piston + spring follow the bogie
      const damper = cylM(0.11, 0.11, 0.75, paintDark, 12); damper.position.set(hp.x, hp.y - 0.38, hp.z); R.add(damper);
      const piston = cylM(0.055, 0.055, 1, titanium, 10); piston.userData.dynamic = true; R.add(piston);
      const springGeo = new THREE.TubeGeometry(new HelixCurve(0.17, 7), 160, 0.022, 6, false);
      const spring = mk(springGeo, accent); spring.userData.dynamic = true; R.add(spring);
      // bogie: steering turret + walking beam + 2 wheels
      const bogie = new THREE.Group(); bogie.userData.dynamic = true; R.add(bogie);
      const turret = cylM(0.34, 0.34, 0.3, paintDark, 20); bogie.add(turret);
      const steerMotor = boxM(0.3, 0.3, 0.3, MAT.gold); steerMotor.position.set(0.0, 0.25, 0); bogie.add(steerMotor);
      const beam = new THREE.Group(); beam.userData.dynamic = true; bogie.add(beam);
      const beamBar = boxM(2.05, 0.2, 0.22, paint); beamBar.position.y = -0.05; beam.add(beamBar);
      const wheels = [];
      for (const wx of [1, -1]) {
        const arm = boxM(0.2, 0.4, 0.2, paint); arm.position.set(wx * B.halfSpan, -0.22, 0); beam.add(arm);
        const wg = new THREE.Group(); wg.position.set(wx * B.halfSpan, -B.wheelBelowPivot, 0); beam.add(wg);
        const spin = new THREE.Group(); spin.userData.dynamic = true; wg.add(spin);
        const tire = mk(new THREE.CylinderGeometry(W.radius, W.radius, W.width, 64, 1, true), tireMat); tire.rotation.x = Math.PI / 2; spin.add(tire);
        for (const side of [-1, 1]) {
          const rim = mk(new THREE.TorusGeometry(W.radius - 0.03, 0.03, 8, 48), titanium); rim.position.z = side * W.width / 2; spin.add(rim);
          for (let k = 0; k < 8; k++) { const a = k / 8 * Math.PI * 2; const sp = strut(new THREE.Vector3(0, 0, side * 0.12), new THREE.Vector3(Math.cos(a) * (W.radius - 0.04), Math.sin(a) * (W.radius - 0.04), side * W.width / 2), 0.022, titanium); spin.add(sp); }
        }
        const hub = cylM(0.24, 0.24, 0.36, MAT.gold, 20); hub.rotation.x = Math.PI / 2; wg.add(hub);
        for (let k = 0; k < 10; k++) { const fin = boxM(0.04, 0.12, 0.3, titanium); const a = k / 10 * Math.PI * 2; fin.position.set(Math.cos(a) * 0.28, Math.sin(a) * 0.28, 0); fin.rotation.z = a; wg.add(fin); }
        const hubCap = cylM(0.12, 0.12, 0.42, paintDark, 16); hubCap.rotation.x = Math.PI / 2; wg.add(hubCap);
        mergeStatic(spin);
        wheels.push({ wg, spin });
      }
      this.bogieVis.push({ hp, piston, spring, bogie, beam, wheels });
    }
    // ---- cradle: trolleys on the girder undersides, hoist blocks, swinging cradle arms
    this.cradle = new THREE.Group(); this.cradle.userData.dynamic = true; R.add(this.cradle);
    this.arms = [];
    for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
      const trolley = new THREE.Group(); this.cradle.add(trolley);
      const t = boxM(0.9, 0.3, 0.5, paintDark); t.position.set(0, F.girderY - F.girderH / 2 - 0.15, sz * (F.girderZ - 0.3)); trolley.add(t);
      const hoist = new THREE.Group(); trolley.add(hoist);
      const block = boxM(0.5, 0.5, 0.4, MAT.gold); block.position.set(0, F.girderY - F.girderH / 2 - 0.55, sz * (F.girderZ - 0.45)); hoist.add(block);
      const hinge = new THREE.Group(); hinge.position.set(0, F.girderY - F.girderH / 2 - 0.8, sz * 2.25); hoist.add(hinge);
      const armShape = new ArmCurve(sz);
      const arm = mk(new THREE.TubeGeometry(armShape, 48, 0.11, 8, false), paint); hinge.add(arm);
      const pad = boxM(0.5, 0.1, 0.7, MAT.black); pad.position.copy(armShape.getPoint(1)); pad.position.y += 0.05; hinge.add(pad);
      this.arms.push({ trolley, hoist, hinge, sx, sz, arm, armShape });
    }
    // ---- solar mast (sun-tracking, vertical panels for the polar low sun)
    this.mast = new THREE.Group(); this.mast.userData.dynamic = true; this.mast.position.set(-1.2, F.girderY + 0.4, -F.girderZ); R.add(this.mast);
    const mastPole = cylM(0.16, 0.2, 3.4, paint, 12); mastPole.position.y = 1.7; this.mast.add(mastPole);
    this.mastHead = new THREE.Group(); this.mastHead.position.y = 3.3; this.mast.add(this.mastHead);
    const boom = boxM(0.12, 0.12, 6.9, paintDark); this.mastHead.add(boom);
    for (const s of [-1, 1]) { const p = boxM(0.06, 3.6, 3.2, solar); p.position.set(0.08, 0, s * 1.7); this.mastHead.add(p); const fr = boxM(0.04, 3.66, 3.26, titanium); fr.position.set(0.02, 0, s * 1.7); this.mastHead.add(fr); }
    // ---- sensors
    const L = ROVER.lidar.mountLocal, C = ROVER.camera.mountLocal;
    const lidarMast = cylM(0.05, 0.05, 0.5, paintDark, 8); lidarMast.position.set(L.x, L.y - 0.3, L.z); R.add(lidarMast);
    const lidarBase = cylM(0.1, 0.1, 0.08, MAT.dark, 20); lidarBase.position.set(L.x, L.y - 0.07, L.z); R.add(lidarBase);
    this.lidarTilt = new THREE.Group(); this.lidarTilt.userData.dynamic = true; this.lidarTilt.position.set(L.x, L.y + 0.02, L.z); this.lidarTilt.rotation.z = (ROVER.lidar.tiltDeg || 0) * Math.PI / 180; R.add(this.lidarTilt);
    this.lidarHead = new THREE.Group(); this.lidarTilt.add(this.lidarHead);
    const lh = cylM(0.09, 0.09, 0.12, MAT.black, 24); this.lidarHead.add(lh);
    const win = boxM(0.04, 0.06, 0.08, MAT.greenGlow); win.position.set(0.09, 0, 0); this.lidarHead.add(win);
    const camBox = new THREE.Group(); camBox.position.set(C.x, C.y, C.z); camBox.rotation.z = ROVER.camera.pitchDeg * Math.PI / 180; R.add(camBox);
    camBox.add(boxM(0.22, 0.16, 0.2, paintDark)); const lens = cylM(0.055, 0.06, 0.1, MAT.glassBlue, 16); lens.rotation.z = Math.PI / 2; lens.position.x = 0.15; camBox.add(lens);
    const camMount = boxM(0.5, 0.28, 0.1, paintDark); camMount.position.set(C.x - 0.4, C.y - 0.14, 0); R.add(camMount);
    // high-gain antenna
    const hga = new THREE.Group(); hga.position.set(-F.crossX, F.crossBottomY + 1.6, 1.4); R.add(hga);
    hga.add(cylM(0.05, 0.05, 0.6, paintDark, 8));
    const dish = mk(new THREE.SphereGeometry(0.45, 24, 10, 0, Math.PI * 2, 0, 0.9), paint); dish.position.y = 0.4; dish.rotation.x = -0.9; hga.add(dish);
    this.hga = hga;
    // ---- lights + beacon
    this.lights = [];
    for (const sz of [-1, 1]) {
      const fixture = boxM(0.25, 0.18, 0.3, MAT.whiteGlow); fixture.position.set(F.crossX + 0.45, F.crossBottomY + 0.3, sz * 2.2); R.add(fixture);
      const sl = new THREE.SpotLight(0xf2f6ff, 60, 70, 0.62, 0.55, 1.6);
      sl.position.copy(fixture.position); sl.target.position.set(F.crossX + 14, -3, sz * 2.0); R.add(sl, sl.target); this.lights.push(sl);
      const rl = boxM(0.2, 0.14, 0.24, MAT.whiteGlow); rl.position.set(-F.crossX - 0.45, F.crossBottomY + 0.3, sz * 2.2); R.add(rl);
    }
    this.beaconMat = new THREE.MeshStandardMaterial({ color: 0x331800, emissive: 0xffa31a, emissiveIntensity: 0 });
    this.beacon = mk(new THREE.SphereGeometry(0.12, 16, 10), this.beaconMat, false); this.beacon.userData.dynamic = true; this.beacon.position.set(0, F.girderY + F.girderH / 2 + 0.15, -F.girderZ); R.add(this.beacon);
    this.beacon2 = this.beacon.clone(); this.beacon2.userData.dynamic = true; this.beacon2.position.z = F.girderZ; R.add(this.beacon2);
    // e-stop mushroom (physical, on the leg) - for realism
    const es = cylM(0.09, 0.09, 0.08, MAT.red, 16); es.rotation.x = Math.PI / 2; es.position.set(-B.x, 0.6, -B.z - 0.3); R.add(es);
    this.addDetails(R);
    // the spotlights live in the static part of the tree; keep them out of the merge
    this.root.traverse((o) => { if (o.isMesh && o.material !== MAT.whiteGlow) { o.castShadow = true; o.receiveShadow = true; } });
    for (const b of this.bogieVis) { mergeStatic(b.bogie); mergeStatic(b.beam); }
    mergeStatic(this.root);
  }

  addDetails(R) {
    const harness = new THREE.MeshStandardMaterial({ color: 0x1b1d20, roughness: 0.7 });
    const harnessO = new THREE.MeshStandardMaterial({ color: 0xd65a1a, roughness: 0.6 });
    const hazard = new THREE.MeshStandardMaterial({ map: hazardTexture(), roughness: 0.6 });
    const tube = (pts, r, mat) => { const c = new THREE.CatmullRomCurve3(pts.map((p) => new THREE.Vector3(...p))); return mk(new THREE.TubeGeometry(c, Math.max(8, pts.length * 10), r, 6, false), mat); };
    for (const sz of [-1, 1]) {
      const zi = sz * (F.girderZ - F.girderW / 2 - 0.05), top = F.girderY + F.girderH / 2;
      // power + data harness along the inner girder face, dropping down each leg to the motor pods
      R.add(tube([[-6.9, top - 0.12, zi], [0, top - 0.1, zi], [6.9, top - 0.12, zi]], 0.045, harness));
      R.add(tube([[-6.9, top - 0.22, zi], [6.9, top - 0.22, zi]], 0.03, harnessO));
      for (const sx of [-1, 1]) R.add(tube([[sx * (B.x - 0.4), top - 0.15, zi], [sx * (B.x - 0.36), 0.8, sz * (B.z - 0.3)], [sx * (B.x - 0.3), -0.2, sz * (B.z - 0.28)], [sx * B.x, B.hardpointY - 0.2, sz * (B.z - 0.25)]], 0.035, harness));
      // handrail on top of each girder (EVA maintenance access)
      for (let x = -6.6; x <= 6.6 + 1e-6; x += 1.65) { const post = cylM(0.02, 0.02, 0.5, titanium, 6); post.position.set(x, top + 0.29, sz * (F.girderZ + 0.18)); R.add(post); }
      const rail = cylM(0.022, 0.022, 13.2, titanium, 6); rail.rotation.z = Math.PI / 2; rail.position.set(0, top + 0.54, sz * (F.girderZ + 0.18)); R.add(rail);
      // linear rails the cradle trolleys run on
      const lr = boxM(13.6, 0.06, 0.12, titanium); lr.position.set(0, F.girderY - F.girderH / 2 - 0.03, sz * (F.girderZ - 0.3)); R.add(lr);
      // nav lights: red port (left), green starboard (right)
      for (const sx of [-1, 1]) { const nl = mk(new THREE.SphereGeometry(0.06, 10, 8), sz < 0 ? new THREE.MeshStandardMaterial({ color: 0x300000, emissive: 0xff2020, emissiveIntensity: 2.2 }) : MAT.greenGlow, false); nl.position.set(sx * 7.15, top + 0.05, sz * (F.girderZ + 0.3)); R.add(nl); }
    }
    // maintenance ladder on the rear-left leg
    for (let y = -0.4; y <= 1.6; y += 0.3) { const rung = cylM(0.018, 0.018, 0.42, titanium, 6); rung.rotation.x = Math.PI / 2; rung.position.set(-B.x - 0.36, y, -B.z + 0.0); R.add(rung); }
    for (const dz of [-0.21, 0.21]) { const side = cylM(0.02, 0.02, 2.2, titanium, 6); side.position.set(-B.x - 0.36, 0.6, -B.z + dz); R.add(side); }
    // hazard striping on lower legs
    for (const sx of [1, -1]) for (const sz of [-1, 1]) { const hz = boxM(0.66, 0.3, 0.54, hazard); hz.position.set(sx * B.x, -0.42, sz * B.z); R.add(hz); }
    // sun sensor, star tracker, UHF whips, IMU housing
    const ss = boxM(0.18, 0.08, 0.18, MAT.black); ss.position.set(-F.crossX, F.crossBottomY + 1.25, -1.0); R.add(ss);
    const st = cylM(0.09, 0.11, 0.26, MAT.dark, 16); st.position.set(-F.crossX + 0.3, F.crossBottomY + 1.3, -1.5); st.rotation.z = 0.5; R.add(st);
    for (const z of [-2.4, 2.4]) { const whip = cylM(0.008, 0.012, 1.2, MAT.black, 6); whip.position.set(-F.crossX - 0.3, F.crossBottomY + 1.3, z); R.add(whip); }
    const imu = boxM(0.3, 0.2, 0.3, MAT.gold); imu.position.set(0, F.girderY + F.girderH / 2 + 0.1, 0.9 * F.girderZ); R.add(imu);
    // vehicle number on cross beams
    for (const sx of [-1, 1]) {
      const num = new THREE.Mesh(new THREE.PlaneGeometry(1.4, 0.5), new THREE.MeshBasicMaterial({ map: labelTexture('SPAN-40 · 01', { color: '#1d2126', size: 60 }), transparent: true }));
      num.position.set(sx * (F.crossX + 0.43), F.crossBottomY + 0.36, 0); num.rotation.y = sx * Math.PI / 2; R.add(num);
    }
  }

  update(rover, time) {
    const p = rover.body.translation(), q = rover.body.rotation();
    this.root.position.set(p.x, p.y, p.z);
    this.root.quaternion.set(q.x, q.y, q.z, q.w);
    rover.bogies.forEach((b, i) => {
      const v = this.bogieVis[i];
      const pivotY = B.hardpointY - b.L;
      v.bogie.position.set(b.x, pivotY, b.z);
      v.bogie.rotation.set(0, b.steer, 0);
      v.beam.rotation.z = Math.asin(Math.max(-1, Math.min(1, b.beamS)));
      // piston from damper bottom to pivot, spring between hardpoint and pivot
      const top = v.hp.y - 0.7, bot = pivotY + 0.15;
      v.piston.position.set(b.x, (top + bot) / 2, b.z); v.piston.scale.y = Math.max(0.05, top - bot);
      v.spring.position.set(b.x, bot, b.z); v.spring.scale.y = Math.max(0.2, (v.hp.y - 0.05) - bot);
      b.wheels.forEach((w, j) => { v.wheels[j].spin.rotation.z = -w.angle; });
    });
    // cradle
    const Lf = rover.lift;
    const closed = Lf.clampT;
    const CT = (Lf.cargo || Lf.pending)?.T;
    const trolleyX = (CT ? CT.lugX : Lf.trolleyX) + 0.45, carryX = CT ? CT.carryX || 0 : 0;
    const isBox = CT?.shape === 'box';
    for (const a of this.arms) {
      a.trolley.position.set(carryX + a.sx * trolleyX + Lf.stageX, 0, 0);
      a.hoist.position.set(0, Lf.pos, Lf.stageZ);
      a.hinge.rotation.x = -a.sz * (1 - closed) * 0.62;
      a.hinge.scale.y = isBox ? 0.9 : 1;
    }
    this.mastHead.rotation.y = rover.power.mastYaw;
    this.mast.rotation.y = 0;
    this.lidarHead.rotation.y = time * 2 * Math.PI * 10;
    for (const l of this.lights) l.intensity = rover.lights && rover.powered ? 60 : 0;
    const moving = Math.hypot(rover.cmd.vx, rover.cmd.vz, rover.cmd.wz) > 0.01;
    let e = 0;
    if (rover.estop) { this.beaconMat.emissive.setHex(0xff2010); e = 3; }
    else if (!rover.powered) { this.beaconMat.emissive.setHex(0xffa31a); e = 0.6; }
    else { this.beaconMat.emissive.setHex(rover.sim.autonomy.mode === 'AUTO' ? 0x3d9bff : 0xffa31a); e = moving ? ((time * 2) % 1 < 0.5 ? 3.5 : 0.2) : 1.0; }
    this.beaconMat.emissiveIntensity = e;
  }
}

class HelixCurve extends THREE.Curve {
  constructor(r, turns) { super(); this.r = r; this.turns = turns; }
  getPoint(t, out = new THREE.Vector3()) { const a = t * this.turns * Math.PI * 2; return out.set(Math.cos(a) * this.r, t, Math.sin(a) * this.r); }
}
// cradle arm: drops from the hinge then curls under the cargo toward the centre line
class ArmCurve extends THREE.Curve {
  constructor(sz) { super(); this.sz = sz; }
  getPoint(t, out = new THREE.Vector3()) {
    const drop = 3.15, inward = 1.55;
    if (t < 0.7) { const k = t / 0.7; return out.set(0, -drop * k, this.sz * 0.05 * k); }
    const k = (t - 0.7) / 0.3, a = k * Math.PI / 2;
    return out.set(0, -drop - Math.sin(a) * 0.25, this.sz * (0.05 - (1 - Math.cos(a)) * inward * 1.0) - this.sz * Math.sin(a) * 0.1);
  }
}
