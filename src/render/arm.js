// Render model of the two ACM arms and the deck interface (docs/ARM_CARGO_BUILD_PLAN.md s4, s6.3). Full scale, metres, rover body
// frame (+X forward, +Y up, +Z right), so it is parented under the AegisModel root with a 1/K scale (same as `extras`).
// There is no baked animation: every frame the joint angles, the hook latch gates and the deck latch are copied from rover.arm
// (src/sim/arm.js), and all geometry positions come from ARM / ROVER / DECK in config.js, so a changed stow pose or link length
// moves the parts with it (the stow cradles are positioned from fk(ARM.stow.q) when the model is built / rebuilt).
//
//   per arm (side s = -1 port / +1 starboard, plane z = s * ARM.shoulder.z)
//     shoulder tower (deck plate, two trapezoid plates, servo housing on the inboard side) -> L1 (box-section beam, lightening holes,
//     flanges, cable run, clips) -> elbow (housing outboard, horn inboard) -> L2 -> wrist (housing outboard, horn inboard) ->
//     hook block with a 45 deg V-seat (ARM.hook.seatHalf) and a latch gate that swings shut over the mouth.
//   deck: round tapered pin + diamond tapered pin at DECK.pinSpan, rotary deck latch (two claws on one shaft), two stow cradles.
import * as THREE from 'three';
import { ARM, ROVER, DECK, CARGO_TYPES, K } from '../sim/config.js';
import { fk, ARM_ASSUMED } from '../sim/arm.js';
import { acmGeom } from '../sim/cargoContact.js';

const S = ARM.shoulder;
const GATE_OPEN = 1.2;          // rad drawn for a fully open gate (the servo travel is only ARM_ASSUMED.latchClosed = 0.6 rad: exaggerated so it reads)
const CLAW_OPEN = 1.3;          // rad drawn for a fully open deck-latch claw
const HOOK_Z0 = 0.20, HOOK_Z1 = 0.80;   // hook block extent inboard of the arm plane (global z 2.66 .. 2.06: outboard of the ACM wall 1.96, around the trunnion 1.96 .. 2.28)
const HOOK_PTS = [[-0.8, -0.6], [0.8, -0.6], [0.8, 0.45], [0.55, 0.45], [0, -0.1], [-0.55, 0.45], [-0.8, 0.45]];   // V notch apex 0.1 m under the trunnion axis

const mat = (c, o = {}) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.5, metalness: 0.4, ...o });
const M = {
  alu: mat(0xc9d0d8, { metalness: 0.7, roughness: 0.35 }),
  alu2: mat(0xaab3bd, { metalness: 0.7, roughness: 0.4 }),
  servo: mat(0x242b33, { metalness: 0.5, roughness: 0.45 }),
  steel: mat(0x7d8792, { metalness: 0.7, roughness: 0.4 }),
  horn: mat(0xd9dde2, { metalness: 0.8, roughness: 0.3 }),
  hook: mat(0x4b5662, { metalness: 0.6, roughness: 0.4 }),
  gate: mat(0xe08a1e, { metalness: 0.4, roughness: 0.45 }),
  cable: mat(0x14171b, { metalness: 0.1, roughness: 0.7 }),
  cableR: mat(0xb02a22, { metalness: 0.1, roughness: 0.7 }),
  pad: mat(0x59636e, { metalness: 0.6, roughness: 0.5 }),
  pin: mat(0xd4a017, { metalness: 0.9, roughness: 0.3 }),
  deck: mat(0x39424c, { metalness: 0.5, roughness: 0.55 }),
};

function add(parent, geo, m, x = 0, y = 0, z = 0) { const o = new THREE.Mesh(geo, m); o.position.set(x, y, z); o.castShadow = true; o.receiveShadow = true; parent.add(o); return o; }
const box = (p, w, h, d, m, x, y, z) => add(p, new THREE.BoxGeometry(w, h, d), m, x, y, z);
// cylinder whose axis is z
function cylZ(p, r, len, m, x, y, z, seg = 24) { const o = add(p, new THREE.CylinderGeometry(r, r, len, seg), m, x, y, z); o.rotation.x = Math.PI / 2; return o; }
// flat plate: extruded polygon in the x-y plane, spanning z0 .. z1 (local group frame)
function plate(p, pts, z0, z1, m, holes = []) {
  const sh = new THREE.Shape(pts.map(([x, y]) => new THREE.Vector2(x, y)));
  for (const [cx, cy, r] of holes) { const h = new THREE.Path(); h.absarc(cx, cy, r, 0, Math.PI * 2, true); sh.holes.push(h); }
  const g = new THREE.ExtrudeGeometry(sh, { depth: Math.abs(z1 - z0), bevelEnabled: false, curveSegments: 16 });
  return add(p, g, m, 0, 0, Math.min(z0, z1));
}

// box-section beam along +x from 0 to len, centred on y = 0: two web plates with lightening holes, top and bottom flanges,
// end bosses, cable run on the top flange with clips
function beam(p, len, h, w, s) {
  const t = 0.05, n = Math.max(1, Math.floor((len - 1.0) / 0.7)), r = h * 0.27;
  const holes = []; for (let i = 0; i < n; i++) holes.push([len / 2 + (i - (n - 1) / 2) * 0.7, 0, r]);
  const web = [[0, -h / 2], [len, -h / 2], [len, h / 2], [0, h / 2]];
  plate(p, web, -w / 2, -w / 2 + t, M.alu, holes);
  plate(p, web, w / 2 - t, w / 2, M.alu, holes);
  box(p, len, t, w, M.alu2, len / 2, h / 2 - t / 2, 0);
  box(p, len, t, w, M.alu2, len / 2, -h / 2 + t / 2, 0);
  for (const x of [0.5, len - 0.5]) box(p, 0.1, h - 2 * t, w - 2 * t, M.alu2, x, 0, 0);     // cross ribs
  // cable run: power (black) + signal (red) along the top flange, offset to the outboard side, held by clips
  const zc = s * (w / 2 - 0.07);
  for (const [dz, cm] of [[0, M.cable], [-s * 0.06, M.cableR]]) { const c = add(p, new THREE.CylinderGeometry(0.022, 0.022, len - 0.5, 8), cm, len / 2, h / 2 + 0.05, zc + dz); c.rotation.z = Math.PI / 2; }
  for (let i = 0; i <= 4; i++) box(p, 0.07, 0.09, 0.2, M.steel, 0.25 + i * (len - 0.5) / 4, h / 2 + 0.045, zc - s * 0.03);
}

// servo housing + gear-stage ring on the side `sd` (+1 outboard, -1 inboard) of the arm plane, horn disc on the other side
function joint(housingParent, hornParent, s, sd, x = 0, y = 0) {
  const zs = s * sd;                                                                  // global side sign of the housing
  box(housingParent, 0.62, 0.55, 0.30, M.servo, x, y, zs * 0.32);
  cylZ(housingParent, 0.30, 0.06, M.steel, x, y, zs * 0.50);                          // cover cap
  box(housingParent, 0.24, 0.12, 0.34, M.steel, x + 0.0, y + 0.33, zs * 0.32);        // connector block
  const horn = new THREE.Group(); horn.position.set(x, y, 0); hornParent.add(horn);
  cylZ(horn, 0.30, 0.05, M.horn, 0, 0, -zs * 0.18);
  for (let i = 0; i < 6; i++) { const a = i * Math.PI / 3; cylZ(horn, 0.03, 0.03, M.steel, Math.cos(a) * 0.22, Math.sin(a) * 0.22, -zs * 0.21, 8); }
  cylZ(housingParent, 0.07, 0.34, M.steel, x, y, -zs * 0.02, 12);                      // output shaft through the beam plane
  return horn;
}

function buildArm(s, placeFn) {
  const g = new THREE.Group(); g.name = s < 0 ? 'arm-port' : 'arm-starboard';
  g.position.set(0, 0, s * S.z);
  const deckY = ROVER.frame.deckY, towerTop = S.y - 0.28;
  // ---- shoulder tower on the deck: base plate, two trapezoid plates, inboard servo housing (shoulder servo)
  const tower = new THREE.Group(); g.add(tower);
  box(tower, 1.4, 0.07, 0.9, M.deck, S.x, deckY + 0.035, -s * 0.36);
  for (const zz of [-0.20, -0.44]) {
    const z0 = s * zz;
    plate(tower, [[S.x - 0.65, deckY + 0.07], [S.x + 0.65, deckY + 0.07], [S.x + 0.3, towerTop], [S.x - 0.3, towerTop]], z0 - 0.03, z0 + 0.03, M.alu);
  }
  for (const xx of [-0.5, 0.5]) for (const zz of [-0.1, -0.6]) add(tower, new THREE.CylinderGeometry(0.05, 0.05, 0.05, 8), M.steel, S.x + xx, deckY + 0.095, s * zz);   // base bolts
  const shoulderGroup = new THREE.Group(); shoulderGroup.position.set(S.x, S.y, 0); g.add(shoulderGroup);
  joint(tower, shoulderGroup, s, -1, S.x, S.y);
  beam(shoulderGroup, ARM.L1, 0.46, 0.26, s);
  // ---- elbow
  const elbow = new THREE.Group(); elbow.position.set(ARM.L1, 0, 0); shoulderGroup.add(elbow);
  joint(shoulderGroup, elbow, s, +1, ARM.L1, 0);
  beam(elbow, ARM.L2, 0.40, 0.24, s);
  // ---- wrist + hook
  const wrist = new THREE.Group(); wrist.position.set(ARM.L2, 0, 0); elbow.add(wrist);
  joint(elbow, wrist, s, +1, ARM.L2, 0);
  const z0 = -s * HOOK_Z0, z1 = -s * HOOK_Z1;
  plate(wrist, HOOK_PTS, z0, z1, M.hook);
  for (const [px, py] of [[-0.65, -0.45], [0.65, -0.45], [0, -0.45]]) cylZ(wrist, 0.04, 0.03, M.steel, px, py, z0 + s * 0.015, 8);
  // latch gate: pivot on the aft side of the V mouth, plate lying across the mouth when closed; servo on the outboard side
  const gate = new THREE.Group(); gate.position.set(-0.62, 0.52, 0); wrist.add(gate);
  box(gate, 1.25, 0.09, 0.14, M.gate, 0.625, 0, -s * 0.45);
  cylZ(gate, 0.07, 0.2, M.steel, 0, 0, -s * 0.45, 12);
  cylZ(wrist, 0.06, 0.5, M.steel, -0.62, 0.52, -s * 0.45, 12);
  box(wrist, 0.24, 0.2, 0.2, M.servo, -0.62, 0.35, -s * 0.2 - s * 0.12);              // latch servo (MG90S class) behind the block
  return { group: g, shoulder: shoulderGroup, elbow, wrist, gate };
}

export class ArmModel {
  // parent: the AegisModel (its root is K-scaled model space); the arms live in a 1/K group, like AegisModel.extras
  constructor(aegis) {
    this.group = new THREE.Group(); this.group.name = 'ACM-arms'; this.group.scale.setScalar(1 / K);
    aegis.root.add(this.group);
    this.sides = []; this.cradles = new THREE.Group(); this.deck = new THREE.Group();
    this.group.add(this.cradles, this.deck);
    for (const s of [-1, 1]) { const a = buildArm(s); this.group.add(a.group); this.sides.push(a); }
    this._built = false;
  }

  // deck pins, deck latch and stow cradles: positions read from rover.arm (placeX) and fk(ARM.stow.q)
  buildDeck(arm) {
    for (const g of [this.cradles, this.deck]) { while (g.children.length) g.remove(g.children[0]); }
    const deckY = ROVER.frame.deckY, px = arm.placeX, span = DECK.pinSpan;
    // round tapered locating pin and diamond tapered pin (3-2-1: round fixes x,z; diamond fixes yaw)
    for (const [dx, round] of [[-span / 2, true], [span / 2, false]]) {
      add(this.deck, new THREE.CylinderGeometry(0.3, 0.3, 0.03, 24), M.pad, px + dx, deckY + 0.015, 0);
      const pin = add(this.deck, new THREE.CylinderGeometry(round ? 0.09 : 0.075, round ? 0.2 : 0.17, 0.16, round ? 28 : 4, 1), M.pin, px + dx, deckY + 0.11, 0);
      if (!round) { pin.rotation.y = 0; pin.scale.set(0.5, 1, 1.15); }
    }
    // rotary deck latch: two claws on one shaft aft of the keeper, claw swings over the keeper bar (jaw = tooth at the claw tip)
    const latchX = px + 0.5, shaftY = deckY + 0.2;
    this.claws = [];
    for (const sz of [-1, 1]) {
      const z = sz * 0.75;
      box(this.deck, 0.5, 0.2, 0.34, M.servo, latchX + 0.1, deckY + 0.1, z);               // housing
      const claw = new THREE.Group(); claw.position.set(latchX, shaftY, z); this.deck.add(claw);
      box(claw, 0.56, 0.08, 0.1, M.gate, -0.28, 0, 0); box(claw, 0.08, 0.28, 0.1, M.gate, -0.56, -0.12, 0);
      cylZ(claw, 0.07, 0.2, M.steel, 0, 0, 0, 12);
      this.claws.push(claw);
    }
    this.deckLatchBox = box(this.deck, 0.3, 0.14, 1.6, M.servo, latchX + 0.4, deckY + 0.07, 0);
    // stow cradles under the hooks at the stow pose (psi = sum of ARM.stow.q), a pedestal with a V-saddle to the lowest hook corner
    const q = ARM.stow.q, f = fk(q), psi = q[0] + q[1] + q[2], c = Math.cos(psi), sn = Math.sin(psi);
    let low = null;
    for (const [hx, hy] of HOOK_PTS) { const wy = f.tip[1] + hx * sn + hy * c, wx = f.tip[0] + hx * c - hy * sn; if (!low || wy < low.y) low = { x: wx, y: wy }; }
    for (const s of [-1, 1]) {
      const zc = s * S.z - s * 0.45, top = Math.max(deckY + 0.5, low.y - 0.06), h = top - deckY;
      box(this.cradles, 1.0, 0.06, 0.6, M.deck, low.x, deckY + 0.03, zc);
      box(this.cradles, 0.22, h, 0.3, M.steel, low.x, deckY + h / 2, zc);
      for (const sx of [-1, 1]) { const w = box(this.cradles, 0.7, 0.07, 0.5, M.pad, low.x + sx * 0.2, top - 0.02, zc); w.rotation.z = sx * 0.5; }   // V saddle
      box(this.cradles, 0.12, 0.35, 0.5, M.pad, low.x - 0.5, top + 0.1, zc);               // stop
    }
    this._built = true; this._stowKey = q.join(',');
  }

  update(rover) {
    const A = rover.arm; if (!A) return;
    if (!this._built || this._stowKey !== ARM.stow.q.join(',')) this.buildDeck(A);
    const CL = ARM_ASSUMED.latchClosed;
    for (let i = 0; i < 2; i++) {
      const a = A.arms[i], v = this.sides[i];
      v.shoulder.rotation.z = a.q[0]; v.elbow.rotation.z = a.q[1]; v.wrist.rotation.z = a.q[2];
      v.gate.rotation.z = (1 - Math.min(1, Math.max(0, a.latchQ / CL))) * GATE_OPEN;
    }
    const d = 1 - Math.min(1, Math.max(0, A.deck.q / CL));
    for (const c of this.claws) c.rotation.z = -d * CLAW_OPEN;
  }
}

// ---- ACM pose adapter --------------------------------------------------------------------------------------------------
// Where to draw an ACM cargo entry c this frame. Returns 'ground' | 'carried' | 'deck' and fills pos (THREE.Vector3) and quat.
//   deck    : c.mode === 'DECK' or c.onDeck or arm.payload.onDeck -> seated on the deck pins at arm.placeX
//   carried : arm.payload set and (state 'carried' or the sim has no body for it) -> hanging from the mean hook trunnion
//   ground  : the sim rigid body (feet sinkage is already in the body height)
const _q = new THREE.Quaternion(), _v = new THREE.Vector3();
export function acmPose(rover, c, pos, quat) {
  const A = rover.arm, pl = A.payload, T = c.T || CARGO_TYPES.acm, G = acmGeom(T);
  const bp = rover.body.translation(), bq = rover.body.rotation();
  const onDeck = c.mode === 'DECK' || c.onDeck || (pl && pl.onDeck);
  const hook = pl && (pl.state === 'carried' || !c.body);
  const body = (lx, ly) => { quat.set(bq.x, bq.y, bq.z, bq.w); pos.set(lx, ly, 0).applyQuaternion(quat).add(_v.set(bp.x, bp.y, bp.z)); };
  if (onDeck) { body(A.placeX, ROVER.frame.deckY + T.cgY); return 'deck'; }
  if (hook) { const t0 = fk(A.arms[0].q).tip, t1 = fk(A.arms[1].q).tip; body((t0[0] + t1[0]) / 2, (t0[1] + t1[1]) / 2 - G.trunY); return 'carried'; }
  if (c.body) { const t = c.body.translation(), r = c.body.rotation(); pos.set(t.x, t.y, t.z); quat.set(r.x, r.y, r.z, r.w); return 'ground'; }
  return null;
}
