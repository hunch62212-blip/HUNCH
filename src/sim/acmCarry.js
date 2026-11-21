// ACM body states and the rover <-> ACM coupling (docs/ARM_CARGO_BUILD_PLAN.md s5.4, s6.3). Owned by Sim as `sim.acm`.
//
//   GROUND   dynamic Rapier body held up by the four soil feet (cargoContact.js acmSoilStep). While the arm has it engaged
//            (c.held) the two hooks are the plan's spring-damper (k_h, c_h of s5.4) between the hook tip and the trunnion end,
//            one-sided vertically (a V-seat only lifts), with the reaction applied to the rover at the hook.
//   CARRIED  kinematic, driven from the mean hook trunnion pose (rigid: the pendulum swing is not modelled, arm.js header).
//            Entered when arm.lifted (hook load = weight, clearance reached); left, back to GROUND, when the soles come
//            within 5 cm of the regolith again (set-down).
//   DECK     kinematic, attached to the rover deck at arm.placeX on the pins (3-2-1 locate). deckSeat() checks the pin
//            capture and blends the body to the exact seated pose; c.onDeck = true.
// CARRIED and DECK bodies use collision group G_ACM_ARM (everything but the rover); GROUND uses G_ACM (everything but terrain).
//
// While the ACM is CARRIED or on the DECK its weight (and the moving arm links) are fed to the rover as point loads
// (Rover.extraLoads) so the suspension, ride, tip margin and CG see them. rover.arm.step gets ctx.groundN here (a wrapper set
// in the constructor), the measured foot load that arm.js uses for the hook load split during breakout.
import * as THREE from 'three';
import { ARM, ACM_PICK, ROVER, MOON, WORLD } from './config.js';
import { G_ACM, G_ACM_ARM } from './world.js';
import { acmSoilStep } from './cargoContact.js';
import { fk } from './arm.js';
import { heightAt } from './terrain.js';

const wrapPi = (a) => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; };
const nearPi = (a) => Math.round(a / Math.PI) * Math.PI;
const smooth = (cur, tgt, dt, tau) => cur + (tgt - cur) * Math.min(1, dt / tau);
export const ACM_ASSUMED = {
  blendTau: 0.35,        // [ASSUMED] s, relaxation of the kinematic pose onto the hook / pin pose (cheek lead-in, pin taper)
  setDownClear: 0.05,    // [ASSUMED] m, sole height above the regolith at which a lowered ACM becomes a soil body again
  holdT: 1.5,            // [ASSUMED] s, minimum time in a state before the opposite hand-over is allowed (hysteresis)
  liftClear: 0.15,       // [ASSUMED] m, sole clearance required for GROUND -> CARRIED
  hookC: 2 * ARM.hook.zeta,
};
const A = ACM_ASSUMED;

export class AcmHandler {
  constructor(sim) {
    this.sim = sim; this.RAPIER = sim.RAPIER;
    this.log = [];
    this._v = new THREE.Vector3(); this._qR = new THREE.Quaternion(); this._qA = new THREE.Quaternion(); this._qy = new THREE.Quaternion();
    // arm.step gets the measured foot load of the held ACM (arm.js _hookLoads ctx.groundN)
    const arm = sim.rover.arm, step0 = arm.step.bind(arm);
    arm.step = (dt, ctx = {}) => step0(dt, { ...ctx, groundN: this.groundN() });
  }

  // two demo ACMs (2400 kg, 1200 kg) on the pad apron, either side of the approach axis
  spawnDemo() {
    const sim = this.sim, P = sim.terrain.pad, y = Math.atan2(-P.dir.z, P.dir.x);
    const at = (u, v) => ({ x: WORLD.pad.x + P.dir.x * u + P.perp.x * v, z: WORLD.pad.z + P.dir.z * u + P.perp.z * v });
    const a = at(-36, 14), b = at(-36, -14);
    return [this.spawn(a.x, a.z, y, { mass: 2400 }), this.spawn(b.x, b.z, y, { mass: 1200 })];
  }
  list() { return this.sim.cargo.filter((c) => c.T.handling === 'arm'); }
  get onDeck() { return this.list().find((c) => c.mode === 'DECK') || null; }
  get carried() { return this.list().find((c) => c.mode === 'CARRIED') || null; }
  held() { return this.list().find((c) => c.held) || null; }

  spawn(x, z, yaw = 0, opts = {}) {
    const c = this.sim.spawnACM(x, z, yaw, opts);
    for (const cc of c.colliders) this.sim.colliderOwner.set(cc.handle, { kind: 'cargo', ref: c });
    c.held = false; c.onDeck = false; c.rel = null; c.modeT = 0; c.hooks = [null, null]; c.seat = null;
    if (opts.settled !== false) c.state = 'staged';
    return c;
  }

  // Foot load (N) of the ACM the arm holds, while it is a soil body; null otherwise (arm.js then uses the lift height).
  groundN() {
    const c = this.held();
    return c && c.mode === 'GROUND' ? c.soilN : null;
  }

  // ---- frames --------------------------------------------------------------------------------------------------
  _rover() { const R = this.sim.rover, t = R.body.translation(), r = R.body.rotation(); return { t, q: this._qR.set(r.x, r.y, r.z, r.w), R }; }
  // rover-local point -> world
  toWorld(p) { const { t, q } = this._rover(); const v = this._v.set(p.x, p.y, p.z).applyQuaternion(q); return { x: t.x + v.x, y: t.y + v.y, z: t.z + v.z }; }
  toLocal(w) { const { t, q } = this._rover(); const v = this._v.set(w.x - t.x, w.y - t.y, w.z - t.z).applyQuaternion(q.clone().invert()); return { x: v.x, y: v.y, z: v.z }; }
  // ACM trunnion-axis centre (the point the hooks grab), rover frame
  trunnionLocal(c) {
    const t = c.body.translation(), r = c.body.rotation();
    const v = new THREE.Vector3(0, c.geom.trunY, 0).applyQuaternion(this._qA.set(r.x, r.y, r.z, r.w));
    return this.toLocal({ x: t.x + v.x, y: t.y + v.y, z: t.z + v.z });
  }
  // ACM pose relative to the rover: position of the CG in the rover frame and the yaw about the rover's up axis
  relPose(c) {
    const { q } = this._rover(); const qi = q.clone().invert();
    const r = c.body.rotation(), qa = new THREE.Quaternion(r.x, r.y, r.z, r.w);
    const rel = qi.multiply(qa), fx = new THREE.Vector3(1, 0, 0).applyQuaternion(rel);
    const p = this.toLocal(c.body.translation());
    return { x: p.x, y: p.y, z: p.z, yaw: Math.atan2(-fx.z, fx.x), pitch: Math.asin(Math.max(-1, Math.min(1, fx.y))) };
  }
  // pose the ACM body (CG) from a rover-frame {x, y, z, yaw}
  _place(c, rel) {
    const { q } = this._rover();
    const w = this.toWorld(rel), qy = this._qy.setFromAxisAngle(new THREE.Vector3(0, 1, 0), rel.yaw);
    const qq = q.clone().multiply(qy);
    c.body.setNextKinematicTranslation({ x: w.x, y: w.y, z: w.z });
    c.body.setNextKinematicRotation({ x: qq.x, y: qq.y, z: qq.z, w: qq.w });
    return w;
  }

  // ---- state changes --------------------------------------------------------------------------------------------
  _setGroups(c, g) { for (const cc of c.colliders) cc.setCollisionGroups(g); }
  _toKinematic(c) {
    c._kinP = null;
    c.body.setBodyType(this.RAPIER.RigidBodyType.KinematicPositionBased, true);
    this._setGroups(c, G_ACM_ARM);
  }
  _toDynamic(c, vel = null) {
    c.body.setBodyType(this.RAPIER.RigidBodyType.Dynamic, true);
    c.body.setLinvel(vel || { x: 0, y: 0, z: 0 }, true); c.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this._setGroups(c, G_ACM);
    for (const f of c.feet) { f.hasPrev = false; f.anchored = false; }
    // soil memory of a body that was in the air: the feet forget their imprint (footStep does this at d <= 0)
  }
  _mode(c, m) {
    if (c.mode === m) return;
    this.sim.log(`${c.id}: ${c.mode} -> ${m}`, 'info');
    c.mode = m; c.modeT = 0;
  }

  // Sole clearance above the regolith (min over the feet), for the current body pose
  soleClearance(c) {
    const t = c.body.translation(), r = c.body.rotation(), q = this._qA.set(r.x, r.y, r.z, r.w);
    let m = Infinity;
    for (const f of c.geom.feet) {
      const v = this._v.set(f.x, c.geom.groundY, f.z).applyQuaternion(q);
      const px = t.x + v.x, pz = t.z + v.z;
      m = Math.min(m, t.y + v.y - heightAt(this.sim.terrain, px, pz));
    }
    return m;
  }

  // ---- arm coupling API (called from autonomy) ------------------------------------------------------------------
  // Hooks at the trunnions: validates the V-seat capture (lateral, yaw), seats the arms, registers the ACM as held.
  engage(c) {
    const arm = this.sim.rover.arm;
    if (c.mode !== 'GROUND') return { ok: false, reason: `ACM is ${c.mode}` };
    const tr = this.trunnionLocal(c), rp = this.relPose(c);
    const yawErr = wrapPi(rp.yaw - nearPi(rp.yaw));
    const out = { lat: tr.z, yaw: yawErr, x: tr.x, y: tr.y };
    if (Math.abs(tr.z) > ACM_PICK.latTol) return { ok: false, reason: `lateral ${(tr.z * 100).toFixed(0)} cm outside the hook cheeks (${(ACM_PICK.latTol * 100).toFixed(0)} cm)`, ...out };
    if (Math.abs(yawErr) > ACM_PICK.yawTolDeg * Math.PI / 180) return { ok: false, reason: `yaw ${(yawErr * 180 / Math.PI).toFixed(1)} deg outside the V-seat`, ...out };
    const r = arm.engage(c.T.mass, { x: tr.x, y: tr.y });
    if (!r.ok) return { ...r, ...out };
    c.held = true; c.hooks = [null, null]; c.holdYaw = nearPi(rp.yaw);
    this._mode(c, 'GROUND');
    return { ok: true, ...out, seated: r.seated };
  }
  // Open gates done: the ACM is no longer held. A GROUND body is simply free; CARRIED must be down first.
  release(c) {
    const arm = this.sim.rover.arm;
    if (c.mode === 'CARRIED') return { ok: false, reason: 'ACM is still suspended' };
    arm.release(); c.held = false; c.hooks = [null, null];
    if (c.mode === 'GROUND') c.state = 'placed';
    return { ok: true };
  }
  // CARRIED over the deck pins -> DECK. Pin capture: the carried pose must be within DECK capture radially and the soles
  // within seatedEps+capture of the deck; the body then blends onto the exact seated pose.
  deckSeat(c) {
    const arm = this.sim.rover.arm;
    if (c.mode !== 'CARRIED') return { ok: false, reason: `ACM is ${c.mode}` };
    const rp = this.relPose(c), dx = rp.x - arm.placeX, dz = rp.z;
    const yaw = wrapPi(rp.yaw - nearPi(rp.yaw));
    const goalY = ROVER.frame.deckY + c.T.cgY, dy = rp.y - goalY;
    const cap = this.deckCapture();
    const err = { dx, dy, dz, yaw, radial: Math.hypot(dx, dz) };
    // the tapered pins centre the body inside capture; the pin span makes the yaw tolerance (pinSpan/2 * yaw <= capture - 2 mm)
    const yawMax = (cap.radial - cap.eps) / cap.halfSpan;
    c.seat = { ...err, yawMax, capture: cap.radial };
    if (err.radial > cap.radial) return { ok: false, reason: `pins missed by ${(err.radial * 1000).toFixed(0)} mm (capture ${(cap.radial * 1000).toFixed(0)} mm)`, ...err };
    if (Math.abs(yaw) > yawMax) return { ok: false, reason: `yaw ${(yaw * 180 / Math.PI).toFixed(2)} deg beyond the pin taper (${(yawMax * 180 / Math.PI).toFixed(2)} deg)`, ...err };
    if (dy > cap.radial) return { ok: false, reason: `not down on the pins (${(dy * 1000).toFixed(0)} mm high)`, ...err };
    c.onDeck = true; c.state = 'held';
    if (arm.payload) arm.payload.onDeck = true;
    this._mode(c, 'DECK');
    return { ok: true, ...err };
  }
  // eps is the residual of the TAPERED PIN centring (2 mm of the 98 mm capture radius, plan s3.3): it sets the yaw the pin span can
  // take out, (radial - eps) / halfSpan. DECK.seatedEps (k(0.002), 32.6 mm) is the arm-side 'pre-seat error' window of the full-scale
  // pose and is a different quantity, deliberately not used here (it would shrink the pin yaw limit from 4.8 to 3.3 deg).
  deckCapture() { return { radial: 0.098, eps: 0.002, halfSpan: 1.141 }; }
  // DECK -> CARRIED (place-to-ground): arm hooks are engaged under the trunnions, deck latch open
  deckRelease(c) {
    const arm = this.sim.rover.arm;
    if (c.mode !== 'DECK') return { ok: false, reason: `ACM is ${c.mode}` };
    if (!arm.deckOpen) return { ok: false, reason: 'deck latch not open' };
    const tip = arm._trunnion(), want = { x: this.sim.rover.arm.placeX, y: ROVER.frame.deckY + c.T.trunnion.y };
    const r = arm.engage(c.T.mass, { x: want.x, y: want.y });
    if (!r.ok) return r;
    arm.latch(true);
    c.held = true; c.onDeck = false;
    if (arm.payload) { arm.payload.onDeck = false; arm.payload.state = 'carried'; arm.payload.lf = 1; }
    this._mode(c, 'CARRIED');
    void tip;
    return { ok: true };
  }

  // ---- per-step ----------------------------------------------------------------------------------------------------
  // Call once per physics step, after rover.step and before world.step.
  step(dt) {
    const sim = this.sim, R = sim.rover, arm = R.arm;
    let any = false;
    for (const c of this.list()) {
      c.modeT += dt;
      if (c.mode === 'GROUND') this._ground(c, dt, arm);
      else this._kin(c, dt, arm);
      if (c.mode !== 'GROUND') any = true;
    }
    // point loads for the rover: the links + the ACM, only while an ACM rides on the arms or the deck
    if (any) {
      const loads = [];
      const mp = arm.massProps(), pl = arm.payload;
      let lm = mp.m, lx = mp.x * mp.m, ly = mp.y * mp.m;
      if (pl) { const tr = arm._trunnion(), pm = pl.mass; lm -= pm; lx -= pm * tr.x; ly -= pm * (tr.y - mp.cgBelow); }
      if (lm > 1) { const w = this.toWorld({ x: lx / lm, y: ly / lm, z: 0 }); loads.push({ m: lm, ...w }); }
      for (const c of this.list()) if (c.mode !== 'GROUND') { const t = c.body.translation(); loads.push({ m: c.T.mass, x: t.x, y: t.y, z: t.z }); }
      R.extraLoads = loads;
    } else if (R.extraLoads.length) R.extraLoads = [];
  }

  _ground(c, dt, arm) {
    acmSoilStep(c, dt, this.sim);
    if (!c.held || !arm.payload) return;
    const R = this.sim.rover, tipLocal = arm.arms.map((a) => ARM_TIP(a));
    const body = c.body, t = body.translation(), r = body.rotation(), qa = this._qA.set(r.x, r.y, r.z, r.w);
    const kh = c.T.mass / 2 * (2 * Math.PI * ARM.hook.fh) ** 2, ch = A.hookC * Math.sqrt(kh * c.T.mass / 2);
    const lin = body.linvel(), ang = body.angvel(), rv = R.body.linvel(), ra = R.body.angvel(), rt = R.body.translation();
    for (let i = 0; i < 2; i++) {
      const a = arm.arms[i];
      if (!a.seated) continue;
      const side = a.side, W2 = c.geom.W / 2 + c.T.trunnion.len / 2;
      // anchor: the hook tip, in the plane of the trunnion end (the hook cheeks centre the ACM laterally)
      const anchor = this.toWorld({ x: tipLocal[i][0], y: tipLocal[i][1], z: side * W2 });
      const e = this._v.set(0, c.geom.trunY, side * W2).applyQuaternion(qa);
      const ex = t.x + e.x, ey = t.y + e.y, ez = t.z + e.z;
      const rx = ex - t.x, ry = ey - t.y, rz = ez - t.z;
      const ve = { x: lin.x + ang.y * rz - ang.z * ry, y: lin.y + ang.z * rx - ang.x * rz, z: lin.z + ang.x * ry - ang.y * rx };
      const prev = c.hooks[i]; c.hooks[i] = anchor;
      const va = prev ? { x: (anchor.x - prev.x) / dt, y: (anchor.y - prev.y) / dt, z: (anchor.z - prev.z) / dt } : { x: rv.x, y: rv.y, z: rv.z };
      void ra; void rt;
      let Fx = kh * (anchor.x - ex) + ch * (va.x - ve.x), Fy = kh * (anchor.y - ey) + ch * (va.y - ve.y), Fz = kh * (anchor.z - ez) + ch * (va.z - ve.z);
      if (Fy < 0) Fy = 0;
      const lim = 6 * c.T.mass * MOON.g;     // keep a bad first step from kicking the body
      const m = Math.hypot(Fx, Fy, Fz); if (m > lim) { const s = lim / m; Fx *= s; Fy *= s; Fz *= s; }
      body.applyImpulseAtPoint({ x: Fx * dt, y: Fy * dt, z: Fz * dt }, { x: ex, y: ey, z: ez }, true);
      R.body.applyImpulseAtPoint({ x: -Fx * dt, y: -Fy * dt, z: -Fz * dt }, anchor, true);
      c.hookF = c.hookF || [0, 0]; c.hookF[i] = Math.hypot(Fx, Fy, Fz);
    }
    // hand over to the kinematic carry once the arm reports the load lifted and the soles are clear
    if (arm.lifted && c.modeT > A.holdT && this.soleClearance(c) > A.liftClear) {
      this._toKinematic(c);
      const rp = this.relPose(c);
      c.rel = { x: rp.x, y: rp.y, z: rp.z, yaw: rp.yaw };
      c.state = 'held';
      this._mode(c, 'CARRIED');
    }
  }

  _kin(c, dt, arm) {
    // velocity of the kinematic body (finite difference of its pose), the launch velocity if it is dropped
    { const t = c.body.translation(), p = c._kinP; c._kinV = p ? { x: (t.x - p.x) / dt, y: (t.y - p.y) / dt, z: (t.z - p.z) / dt } : { x: 0, y: 0, z: 0 }; c._kinP = { x: t.x, y: t.y, z: t.z }; }
    // 'drop' fault (arm.js _checks: hook let go of the trunnion, payload cleared): the ACM is no longer held, so it falls as a dynamic
    // soil body with the velocity it had. The fault stays recorded on the arm (the safe-down path logs it, it does not wipe it).
    if (c.mode === 'CARRIED' && c.held && !arm.payload) {      // (the fault itself can already be cleared by the safe-down: the missing payload is the test)
      this.sim.log(`${c.id}: DROPPED (hook fault), falling`, 'err');
      this._toDynamic(c, c._kinV); c.rel = null; c.held = false; c.hooks = [null, null]; c._kinP = null; c.state = 'dropped';
      this._mode(c, 'GROUND');
      return;
    }
    // set-down: a carried ACM whose soles are near the regolith (and not yet seated on the deck) becomes a soil body again
    if (c.mode === 'CARRIED' && c.held && c.modeT > A.holdT && this._groundBelow(c)) {
      this._toDynamic(c); c.rel = null; c._kinP = null;
      this._mode(c, 'GROUND');
      return;
    }
    const pl = arm.payload;
    let tgt;
    if (c.mode === 'DECK') tgt = { x: arm.placeX, y: ROVER.frame.deckY + c.T.cgY, z: 0, yaw: nearPi(c.rel ? c.rel.yaw : 0) };
    else if (pl) { const tr = arm._trunnion(); tgt = { x: tr.x, y: tr.y - (c.T.trunnion.y - c.T.cgY), z: 0, yaw: nearPi(c.rel ? c.rel.yaw : 0) }; }
    else tgt = c.rel || this.relPose(c);
    if (!c.rel) { const rp = this.relPose(c); c.rel = { x: rp.x, y: rp.y, z: rp.z, yaw: rp.yaw }; }
    const k = A.blendTau, r = c.rel;
    r.x = smooth(r.x, tgt.x, dt, k); r.y = smooth(r.y, tgt.y, dt, k); r.z = smooth(r.z, tgt.z, dt, k); r.yaw = smooth(r.yaw, tgt.yaw, dt, k);
    this._place(c, r);
  }
  // a carried ACM being lowered onto the regolith: soles within setDownClear of the terrain, and the arm lowering it
  _groundBelow(c) {
    const R = this.sim.rover;
    if (R.arm.payload && R.arm.payload.onDeck) return false;
    const rp = c.rel;
    // never "set down" while it is over the deck: the deck is not regolith
    if (rp && rp.x < ROVER.corners[0][0] * 0.4 && Math.abs(rp.z) < 2.4) return false;
    return this.soleClearance(c) < A.setDownClear;
  }
}

const ARM_TIP = (a) => fk(a.q).tip;     // hook tip, rover frame (same FK as arm.js)
