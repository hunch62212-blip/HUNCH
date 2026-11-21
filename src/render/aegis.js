// AEGIS-V1 renderer: the team's own procedural model (public/team/js, loaded unchanged by teamModels.js)
// scaled 1:K to full size and driven by the physics. The six wheels follow the rocker-bogie linkage
// (rocker angle + two bogie angles solved from the wheel heights), the tyres spin with the motors,
// the roller-screw struts show the ride height (kneel / drive / carry) and the ring jaws close on the module.
import * as THREE from 'three';
import { ROVER, K } from '../sim/config.js';
import { buildAegis, loadTeamModels } from './teamModels.js';
import { MODE, bayOf, partPos } from '../sim/health.js';
import { mergeStatic } from './mergeStatic.js';
import { heightAt } from '../sim/terrain.js';
import { KEEPALIVE, FENDER, RADIATOR } from '../sim/config.js';

// sim bogie index (FL=0, FR=1, ML=2, MR=3, RL=4, RR=5 where L=-z, R=+z) for fusion_rover wheelGroups order ([0..2] at +z, [3..5] at -z)
const MODEL_TO_SIM = [1, 3, 5, 0, 2, 4];

// Ride height the wheels are drawn at, model metres: the commanded ride minus the mean of d (d[i] = (commanded strut length - actual) / K),
// i.e. the mean strut extension the physics has, limited to the model's wheel-arch range. Exported for tests/v17_suite.mjs.
export const DRAWN_RIDE_RANGE = [-0.045, 0.065];
export function drawnRide(rideOffset, d) {
  return Math.max(DRAWN_RIDE_RANGE[0], Math.min(DRAWN_RIDE_RANGE[1], rideOffset / K - (d[0] + d[1] + d[2] + d[3] + d[4] + d[5]) / 6));
}

export class AegisModel {
  constructor(scene) {
    this.scene = scene;
    this.root = new THREE.Group(); this.root.name = 'AEGIS-V1'; this.root.scale.setScalar(K);
    scene.add(this.root);
    this.ready = false; this.lights = [];
  }

  async init() {
    const { group, root, rig } = await buildAegis();
    this.rig = rig; this.model = root;
    this.root.add(group);
    // the team model's lamps are authored for a 0.6 m rover; swap them for lights sized for the real one
    root.traverse((o) => {
      if (o.isLight) o.visible = false;
      if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; }
    });
    for (const sz of [-1, 1]) {
      const s = new THREE.SpotLight(0xfff3dc, 0, 140, 0.55, 0.5, 1.2);
      s.position.set(0.27, 0.23, sz * 0.12); s.target.position.set(3, 0.05, sz * 0.12);
      this.root.add(s); this.root.add(s.target); this.lights.push(s);
    }
    this.wheelGroups = rig.wheelGroups; this.tires = rig.tires;
    // each tyre is ~60 rigid parts (40 chevron cleats, spokes, flanges, hub) that only ever spin together
    // (the cleat and spoke materials are per-wheel and fade with speed as a whole, so they may merge although transparent)
    for (const t of this.tires) mergeStatic(t, undefined, { transparent: true });
    this.bogie = window.roverBogie; this.cradle = window.moduleCradle; this.heads = window.lidarHeads || [window.lidarHead].filter(Boolean);
    this.lastRide = null;
    this.buildSurvivalHardware();
    this.ready = true;
    this._bogieD = new Array(6).fill(0);
    return this;
  }

  // Survival / serviceability hardware added to the team model (full-scale metres, body frame): louvered side
  // radiators, service-bay doors with BIT status lamps, omni antennas, beacon, hazard cameras, the keep-alive
  // solar blanket and a locator that marks the failed part. Positions follow src/sim/health.js BAYS.
  // Everything is mounted on the team body as measured (tub walls z +-2.23..2.33, deck top y 3.67 to z +-2.94 and
  // x +-4.73, front face x 4.08..4.25, rear face x -4.26..-4.34): deck items are seated on the surface by a ray cast
  // so nothing floats (CHANGES.md §20).
  buildSurvivalHardware() {
    const X = this.extras = new THREE.Group(); X.scale.setScalar(1 / K); this.root.add(X);
    const mat = (c, o = {}) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.55, metalness: 0.3, ...o });
    const box = (w, h, d, m, x, y, z) => { const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m); b.position.set(x, y, z); b.castShadow = true; X.add(b); return b; };
    const cyl = (r, h, m, x, y0, z) => { const c = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 10), m); c.position.set(x, y0 + h / 2, z); c.castShadow = true; X.add(c); return c; };
    // deck surface height (body m) under (x, z), from a ray cast onto the team model
    const ray = new THREE.Raycaster();
    this.root.updateMatrixWorld(true);
    const deckY = (x, z, fallback = ROVER.frame.deckY) => {
      const o = this.root.localToWorld(new THREE.Vector3(x / K, 15 / K, z / K));
      const d = this.root.localToWorld(new THREE.Vector3(x / K, 0, z / K)).sub(o).normalize();
      ray.set(o, d);
      const h = ray.intersectObject(this.model, true)[0];
      return h ? this.root.worldToLocal(h.point.clone()).y * K : fallback;
    };
    const steelM = mat(0x7d8792, { metalness: 0.7, roughness: 0.4 });
    // radiators on the tub walls under the deck overhang: back panel on two mounting rails, 12 louver slats that turn
    // with the thermostat
    const R = RADIATOR, radM = mat(0xe8edf2, { roughness: 0.25, metalness: 0.6 }), slatM = mat(0xb9c4cf, { metalness: 0.8, roughness: 0.2 });
    this.slats = { L: [], R: [] };
    for (const [s, sz] of [['L', -1], ['R', 1]]) {
      box(R.len, R.h, 0.05, radM, 0, R.y, sz * R.z);
      for (const dy of [-0.3, 0.3]) box(R.len - 0.2, 0.08, 0.16, steelM, 0, R.y + dy, sz * (R.z - 0.09)); // rails back to the wall
      for (let i = 0; i < 12; i++) this.slats[s].push(box(R.len - 0.1, 0.07, 0.012, slatM, 0, R.y - R.h / 2 + 0.07 + i * 0.065, sz * (R.z + 0.07)));
    }
    // zenith Z-93 OSR secondary thermal strips on the unshaded fore/aft deck margins (clear of ring saddles)
    const osrM = mat(0xf4f6f8, { roughness: 0.22, metalness: 0.15 });
    for (const sx of [-3.6, 3.6]) box(1.1, 0.04, 3.6, osrM, sx, deckY(sx, 0) + 0.03, 0);
    this.buildFenders(mat);
    // service-bay doors with a status lamp (green = BIT pass, amber / red flashing = alarm in that bay)
    const doorM = mat(0x3b4754), lampG = new THREE.SphereGeometry(0.08, 10, 8);
    this.lamps = {};
    const lamp = (bay, x, y, z) => { const l = new THREE.Mesh(lampG, new THREE.MeshBasicMaterial({ color: 0x3ddc84 })); l.position.set(x, y, z); X.add(l); this.lamps[bay] = l; };
    box(0.05, 1.0, 2.2, doorM, 4.28, 2.1, 0); lamp('AV', 4.33, 2.45, 0.95);
    for (const [bay, x, y, sz] of [['MCL', 1.95, 1.9, -1], ['MCR', 1.95, 1.9, 1], ['BTL', -1.95, 1.7, -1], ['BTR', -1.95, 1.7, 1]]) { box(1.6, 1.0, 0.05, doorM, x, y, sz * 2.4); lamp(bay, x + 0.65, y + 0.38, sz * 2.45); }
    // omni whips on the deck edge. They sit under the carried module's overhang (its underside is ~6.5 m up at
    // |z| = 2.95), so they are 2.0 m tall; at the south pole Earth is a few degrees above the horizon, so an omni
    // needs a horizontal view, not a zenith one. The front pair stays aft of the front-corner LiDAR heads.
    const whipM = mat(0x222831), tipM = mat(0xdde3ea), baseM = mat(0x4a5560, { metalness: 0.6 });
    const whip = (x, z) => {
      const y = deckY(x, z);
      cyl(0.11, 0.1, baseM, x, y, z);
      const c = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.04, 2.0, 8), whipM); c.position.set(x, y + 0.1 + 1.0, z); c.castShadow = true; X.add(c);
      const t = new THREE.Mesh(new THREE.SphereGeometry(0.07, 8, 6), tipM); t.position.set(x, y + 2.15, z); X.add(t);
    };
    for (const [x, z] of [[3.0, 2.95], [3.0, -2.95], [-4.5, 2.95], [-3.7, -2.95]]) whip(x, z);
    // beacon on a short post at the starboard-aft deck corner
    { const x = -4.5, z = 2.5, y = deckY(x, z); cyl(0.06, 0.45, steelM, x, y, z);
      this.beacon = new THREE.Mesh(new THREE.SphereGeometry(0.16, 12, 8), new THREE.MeshBasicMaterial({ color: 0x553300 }));
      this.beacon.position.set(x, y + 0.6, z); X.add(this.beacon); }
    // hazard cameras flush on the front and rear faces of the tub
    const camM = mat(0x111418, { metalness: 0.5 });
    for (const x of [4.17, -4.43]) for (const z of [-0.6, 0.6]) box(0.18, 0.14, 0.22, camM, x, 2.65, z);
    // keep-alive array: outrigger (post on the port-aft deck corner, arm out past the module footprint), canister,
    // telescoping mast, 2 x 6 m blanket
    const C = KEEPALIVE.corner, aY = KEEPALIVE.armY, postZ = -2.9;
    { const y0 = deckY(C.x, postZ), top = aY + 0.7; box(0.22, top - y0, 0.22, steelM, C.x, (y0 + top) / 2, postZ); }  // post
    box(0.2, 0.2, Math.abs(C.z - postZ) + 0.35, steelM, C.x, aY, (C.z + postZ) / 2 - 0.05);           // arm
    // tie rod from the post top down to the arm, above the arm so it never meets the rear fender
    { const dy = 0.6, dz = 1.4, b = box(0.06, Math.hypot(dy, dz), 0.06, steelM, C.x, aY + 0.1 + dy / 2, postZ - dz / 2); b.rotation.x = Math.atan2(dz, dy); }
    box(0.5, 1.0, 0.5, mat(0xd0d6dc), C.x, aY + 0.1 + 0.5, C.z);                                       // canister on the arm end
    this.mastBase = aY + 0.1 + 1.0;
    this.mast = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.08, 1, 10), mat(0xaab4bf, { metalness: 0.8 }));
    this.mast.position.set(C.x, this.mastBase, C.z); X.add(this.mast);
    this.blanketPivot = new THREE.Group(); this.blanketPivot.position.set(C.x, this.mastBase, C.z); X.add(this.blanketPivot);
    const cells = new THREE.MeshStandardMaterial({ color: 0x1b2a55, metalness: 0.4, roughness: 0.35, side: THREE.DoubleSide, emissive: 0x050a1a });
    this.blanket = new THREE.Mesh(new THREE.PlaneGeometry(2.0, 6.0), cells); this.blanket.position.set(1.1, 3.0, 0); this.blanketPivot.add(this.blanket);
    // failed-part locator (drawn through the body)
    this.locator = new THREE.Group();
    const ball = new THREE.Mesh(new THREE.SphereGeometry(0.55, 16, 10), new THREE.MeshBasicMaterial({ color: 0xff4d3a, wireframe: true, transparent: true, opacity: 0.9, depthTest: false }));
    ball.renderOrder = 10; this.locator.add(ball);
    const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 4, 6), new THREE.MeshBasicMaterial({ color: 0xff4d3a, depthTest: false }));
    stem.position.y = 2.5; stem.renderOrder = 10; this.locator.add(stem);
    this.locator.visible = false; X.add(this.locator);
  }

  // Wheel fenders and radiator ejecta deflectors (CHANGES.md §17, FENDER in config.js). The tyre axis is along the body z
  // axis. Each fender is an open arched shell (a partial cylinder, radius = tyre radius + clearance) centred on the wheel
  // hub, tilted aft, with a trailing flap. It is parented to the wheel assembly, so it rides with the wheel on the
  // rocker-bogie and never touches the tyre, and it is built in model metres (the wheel group is inside the K-scaled root).
  // The deflector is a plate on the deck edge, outboard of the radiator, the other half of the dust exclusion.
  buildFenders(mat) {
    const F = FENDER, Rw = ROVER.wheel.radius, rIn = (Rw + F.clearance) / K, w = F.width / K, t = F.thickness / K;
    const fenderM = new THREE.MeshStandardMaterial({ color: 0x8b95a1, roughness: 0.5, metalness: 0.55, side: THREE.DoubleSide });
    const arc = F.arcDeg * Math.PI / 180, bias = F.aftBiasDeg * Math.PI / 180;
    // CylinderGeometry axis is +y; rotate it onto z. thetaStart is measured from +z toward +x in the cylinder frame
    const shell = new THREE.CylinderGeometry(rIn + t, rIn + t, w, 28, 1, true, 0, arc);
    shell.rotateX(Math.PI / 2);                       // axis -> z
    // after rotateX the arc starts at the bottom (-y) and sweeps through +x (forward) to the top; its centre sits at
    // angle arc/2 - 90 deg. Turn that centre onto the top (+y, 90 deg) plus the aft bias (towards -x)
    shell.rotateZ(Math.PI + bias - arc / 2);
    const flapG = new THREE.BoxGeometry(t, F.flapH / K, w);
    this.fenders = [];
    for (let i = 0; i < this.wheelGroups.length; i++) {
      const g = new THREE.Group(); g.name = 'fender';
      const m = new THREE.Mesh(shell, fenderM); m.castShadow = true; g.add(m);
      // trailing flap at the aft end of the arc
      const endA = Math.PI / 2 + bias + arc / 2, fl = new THREE.Mesh(flapG, fenderM);
      fl.position.set(Math.cos(endA) * (rIn + t), Math.sin(endA) * (rIn + t) - F.flapH / K / 2, 0); fl.castShadow = true; g.add(fl);
      this.wheelGroups[i].group.add(g); this.fenders.push(g);
    }
    // ejecta skirts: a plate hinged to the tub wall along each radiator's lower edge, sloping outboard and down. It
    // keeps wheel and rocker-beam ejecta off the louvers without covering them (the radiators still see sideways).
    // Outer edge at |z| ~2.72: inboard of the rocker beams (2.95) and fenders (3.09).
    const D = F.deflector, X = this.extras, plateM = new THREE.MeshStandardMaterial({ color: 0x9aa4af, roughness: 0.45, metalness: 0.6 });
    const a = D.angleDeg * Math.PI / 180, hingeZ = RADIATOR.z - 0.02;
    for (const sz of [-1, 1]) {
      const p = new THREE.Mesh(new THREE.BoxGeometry(D.len, 0.03, D.depth), plateM);
      p.rotation.x = sz * a;
      p.position.set(0, D.y - (D.depth / 2) * Math.sin(a), sz * (hingeZ + (D.depth / 2) * Math.cos(a)));
      p.castShadow = true; X.add(p);
    }
  }

  updateSurvivalHardware(rover, time) {
    const H = rover.sim.health; if (!H || !this.extras) return;
    for (const s of ['L', 'R']) { const open = H.th.louver[s]; for (const sl of this.slats[s]) sl.rotation.x = (1 - open) * 1.2 * (s === 'L' ? 1 : -1); }
    const sevN = { caution: 1, warning: 2, critical: 3 }, worst = {};
    let crit = false;
    for (const a of H.alarms.values()) { const m = MODE[a.id], b = bayOf(m, a.n); worst[b] = Math.max(worst[b] || 0, sevN[m.sev]); if (m.sev === 'critical') crit = true; }
    for (const [bay, l] of Object.entries(this.lamps)) { const w = worst[bay] || 0; l.material.color.setHex(w ? (w >= 3 ? 0xff2a1a : 0xffa020) : 0x3ddc84); l.visible = !w || Math.sin(time * 8) > -0.3; }
    this.beacon.material.color.setHex((H.survival.on || crit) && Math.sin(time * 6) > 0.6 ? 0xffb020 : 0x553300);
    const A = H.array, k = A.k, h = KEEPALIVE.mastH;
    this.mast.scale.y = Math.max(0.01, k * h); this.mast.position.y = this.mastBase + k * h / 2;
    this.blanketPivot.visible = k > 0.02;
    this.blanketPivot.position.y = this.mastBase + k * (h - 6.0);
    this.blanket.scale.y = Math.max(0.01, k); this.blanket.position.y = 3.0 * k;
    { const r = rover.body.rotation(); const fx = 1 - 2 * (r.y * r.y + r.z * r.z), fz = 2 * (r.x * r.z - r.w * r.y); this.blanketPivot.rotation.y = A.yaw - Math.atan2(-fz, fx) + Math.PI / 2; }
    const sel = H.selected && H.alarms.get(H.selected);
    this.locator.visible = !!sel;
    if (sel) { const p = partPos(MODE[sel.id], sel.n); this.locator.position.set(p[0], p[1], p[2]); this.locator.children[0].scale.setScalar(1 + 0.25 * Math.sin(time * 5)); }
  }

  update(rover, time) {
    const p = rover.body.translation(), q = rover.body.rotation();
    this.root.position.set(p.x, p.y, p.z);
    this.root.quaternion.set(q.x, q.y, q.z, q.w);
    if (!this.ready) return;
    const RB = window.NASA_RockerBogie.GEO;
    // wheel height change in model metres, relative to the nominal stance (ride height removed)
    const bogies = rover.bogies;
    for (let _i = 0; _i < bogies.length; _i++) this._bogieD[_i] = (ROVER.bogie.Lnom + rover.rideOffset - bogies[_i].L) / K;
    const d = this._bogieD;
    const side = (i0) => { // sim indices of a side in F, M, R order
      const [f, m, r] = i0; const dyF = d[f], dyM = d[m], dyR = d[r];
      return { rho: (dyF - (dyM + dyR) / 2) / (RB.xF - RB.B.x), beta: (dyM - dyR) / (RB.xM - RB.xR) };
    };
    const posZ = side([1, 3, 5]), negZ = side([0, 2, 4]);
    const cl = (v, m) => Math.max(-m, Math.min(m, v));
    const rho = cl((posZ.rho - negZ.rho) / 2, RB.rhoMax);
    this.bogie.pose(rho, cl(posZ.beta, RB.betaMax), cl(negZ.beta, RB.betaMax));
    // Ride height as the struts have it, in model metres, limited to the wheel-arch range -0.045 .. +0.065. Two faults fixed here (v1.7.0):
    //  1. this line was cl(rover.rideOffset / K, -0.045, 0.065), but cl(v, m) clamps to +-m and ignores a third argument, so it
    //     returned max(0.045, min(-0.045, v)) = +0.045 whatever the ride height was. The wheels were always drawn 0.045 x K = 0.73 m
    //     below their nominal position: 0.73 m into the ground at ride 0, 1.39 m at full kneel, and the strut graphics never moved.
    //  2. it used the COMMANDED ride height; the body follows the command with a lag and sags under a new load. The mean of d is
    //     (commanded - actual) / K, so subtracting it gives the actual mean strut extension, which is what the wheels must show.
    const ride = drawnRide(rover.rideOffset, d);
    for (let i = 0; i < 6; i++) {
      const wg = this.wheelGroups[i], w = rover.wheels[MODEL_TO_SIM[i]];
      wg.group.position.y -= ride;           // body rises on the struts, wheels stay down (kneel+bump wheel-arch limit preserved)
      wg.strut.setLength(ride);
      this.tires[i].rotation.y = -w.angle;
    }
    // Terrain clamp, the second line after the strut-true ride height above: whatever is left (rocker-bogie pose limits, the pose's
    // small-angle kinematics, tyre sinkage) must not draw a tread below the heightfield. The hub's world position is compared with
    // the ground under it and lifted along the body's up axis if needed. The fenders are children of the wheel group
    // (buildSurvivalHardware), so they ride with the wheel. this.terrainClamp = false switches it off for measurements.
    const T = this.terrainClamp === false ? null : rover.sim?.terrain;
    if (T) {
      const q = this.root.quaternion, upY = Math.max(0.5, 1 - 2 * (q.x * q.x + q.z * q.z)), v = this._hubV || (this._hubV = new THREE.Vector3());
      for (let i = 0; i < 6; i++) {
        const g = this.wheelGroups[i].group;
        v.set(g.position.x * K, g.position.y * K, g.position.z * K).applyQuaternion(q).add(this.root.position);
        const minHubY = heightAt(T, v.x, v.z) + ROVER.wheel.radius;   // = (groundY + wheel radius) in world metres
        if (v.y < minHubY) g.position.y += (minHubY - v.y) / (K * upY);
      }
    }
    // ring jaws
    const Lf = rover.lift;
    this.cradle.apply({ lid: Math.min(1, Lf.clampT * 1.25), squeeze: Math.max(0, (Lf.clampT - 0.8) * 5), locked: Lf.clamp === 'closed' });
    // The straddle-cargo cradle (saddle tubes, posts, ring jaws) occupies the same deck volume as the ACM arms, hooks and the
    // ACM at the place pose, so it is hidden (render only) while an ACM is carried / on deck or the arms are not stowed.
    const A = rover.arm;
    const acmAboard = !!(A && A.payload) || !!rover.sim?.cargo?.some((c) => c.T?.handling === 'arm' && (c.mode === 'CARRIED' || c.mode === 'DECK' || c.onDeck));
    const hideCradle = acmAboard || !!(A && !A.stowed);
    const cb = window.cargoBayGroup; if (cb) cb.visible = !hideCradle;
    for (const h of this.heads) h.rotation.y = time * 2 * Math.PI * 10;
    // Headlights sized against the sun (DirectionalLight 5.2 in scene.js): illuminance E = I / d^2. At 900 cd,
    // always on, the ground 5 m ahead got 36 = 7x full sun and the main view and the camera feed burned out white.
    // Now ~70 cd in shadow or at night (E ~0.7 at 10 m, a usable work light) and dimmed to 10 % in sunlight,
    // where real vehicle lamps add nothing next to ~1300 W/m^2 of unfiltered sun. Eased so a shadow edge is not a flash.
    const sunlit = Number(rover.power?.sunlit ?? 1) > 0.5 && (rover.sim?.sunDir?.y ?? 1) > 0;
    const want = rover.lights && rover.powered ? (sunlit ? 15 : 70) : 0; // 70 cd at night: 150 still burned the near ground white
    this._lampI = this._lampI == null ? want : this._lampI + (want - this._lampI) * 0.08;
    for (const s of this.lights) s.intensity = this._lampI;
    this.updateSurvivalHardware(rover, time);
  }
}

// ---- cargo module + stands (team module_models.js at full scale) ----
export async function buildTeamCargo(c) {
  const T = await loadTeamModels();
  const g = new THREE.Group();
  const m = T.ModuleModels.buildModuleMesh(); m.scale.setScalar(K); g.add(m);
  g.userData.cad = null;
  mergeStatic(g);                 // the module is one rigid body: 161 parts -> one mesh per material
  return g;
}
export async function buildTeamStands(cargo, heightAt, terrain) {
  const T = await loadTeamModels();
  const out = new THREE.Group();
  for (const c of cargo) {
    const wrap = new THREE.Group(); wrap.scale.setScalar(K); out.add(wrap);
    const st = T.ModuleModels.buildStand(`${c.id}`);
    const lugs = c.T.lugXs || [-c.T.lugX, c.T.lugX];
    const midX = (lugs[0] + lugs[1]) / 2;
    const spanX = lugs[1] - lugs[0], nomSpanX = 2 * (c.T.shape === 'cylinder' ? c.T.lugX : 0.21 * K);
    const stX = c.initial.x + Math.cos(c.yaw) * midX, stZ = c.initial.z - Math.sin(c.yaw) * midX;
    const yB = (c.initial.y - (c.T.adapterR || c.T.radius)) / K;
    T.ModuleModels.placeStand(st, stX / K, stZ / K, yB, (mx, mz) => heightAt(terrain, mx * K, mz * K) / K);
    st.scale.x = spanX / nomSpanX;
    // yaw about the stand centre (offset to box20 asymmetric lug midpoint when applicable)
    const pivot = new THREE.Group(); pivot.position.copy(st.position); st.position.set(0, 0, 0); pivot.rotation.y = c.yaw; pivot.add(st);
    wrap.add(pivot);
    mergeStatic(pivot);           // ~55 small parts per stand, none of them move
    c.standVis = pivot;
  }
  return out;
}
