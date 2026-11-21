// Arm-handled cargo (ACM) missions for the Autonomy executive (docs/ARM_CARGO_BUILD_PLAN.md s6.4). Mixed into Autonomy.prototype
// from autonomy.js (Object.assign), so these generators run with `this` = the Autonomy and follow its step/log/abort style.
//
//   acquireArm(c)        pick-up: stand-off -> square up -> park -> deploy -> pre-lift check -> grasp -> latch -> breakout
//                        -> swing -> place on the deck pins -> deck latch -> release -> stow
//   deliverArm(x, z)     the reverse: drive to the receiving point, unlatch, hook, lift, swing out, set down on the regolith, stow
//   driveAway(dist)      straight drive with the ACM on the deck, under the speed caps of arm.js / Rover.speedCap
//   missionPickupArm / missionPlaceArm   mission wrappers
//
// Close-range sensing (acmSense) stands in for the ToF pair + AprilTag face of the plan (s6.4 step 3): it reads the ACM pose
// relative to the rover from the simulation state and adds small seeded noise. Only close range (< 40 m) uses it; the far
// stand-off point comes from the offload report (cargoGuess-style error, 0.8 m / 2 deg).
import { ROVER, ARM, ACM_PICK, MOON } from './config.js';
import { ik } from './arm.js';
import { heightAt } from './terrain.js';

const DEG = Math.PI / 180;
const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
const wrap = (a) => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; };
const wrapHalf = (a) => a - Math.round(a / Math.PI) * Math.PI;     // reduce to (-pi/2, pi/2]: the ACM is symmetric end to end

// tolerances used by the state machine (capture ranges come from ACM_PICK / the deck pins, tighter by design margin)
export const ACM_TOL = {
  lat: 0.09,            // m, ACM centre off the rover axis at the grasp (hook cheeks capture ACM_PICK.latTol = 0.114 m)
  yaw: 2.5 * DEG,       // rad (V-seat / pin taper capture 3 deg)
  along: 0.45,          // m from ARM.pick.xBody (the grasp pose is re-solved for the measured trunnion, so this is slack)
  standoff: 4.0,        // m, ACM_PICK.standoff
};

export const armMissions = {
  // ---- helpers -----------------------------------------------------------------------------------------------------
  acmSense(c) {
    const R = this.sim.rover, p = R.pose(), t = c.body.translation(), r = c.body.rotation();
    // seeded LCG (glibc constants), Math.imul keeps the product exact in 32 bits (the plain float product overflowed 2^53)
    const nx = () => (this._acmN = (Math.imul(this._acmN ?? 12345, 1103515245) + 12345) & 0x7fffffff) / 0x7fffffff - 0.5;  // +-0.5
    const n = nx(), n2 = nx();
    const dx = t.x - p.x, dz = t.z - p.z, cs = Math.cos(p.yaw), sn = Math.sin(p.yaw);
    const qx = r.x, qy = r.y, qz = r.z, qw = r.w;
    const fx = 1 - 2 * (qy * qy + qz * qz), fz = 2 * (qx * qz - qw * qy);        // ACM +x axis in the world
    const ya = Math.atan2(-fz, fx);
    const f = dx * cs - dz * sn + 0.004 * n, rr = dx * sn + dz * cs + 0.004 * n2;
    const yawE = wrapHalf(wrap(ya - p.yaw)) + 0.0015 * n;
    const lat = f * Math.sin(yawE) + rr * Math.cos(yawE);                          // rover's offset from the ACM axis, as in _acquire
    return { f, r: rr, yawE, lat, along: f - ARM.pick.xBody };
  },
  *_armWait(cond, timeout, label, c = null) {
    const R = this.sim.rover, arm = R.arm, t0 = this.sim.time;
    while (!cond()) {
      this.stop();
      if (arm.faults.size) return { ok: false, why: `arm fault: ${[...arm.faults].join(',')}` };
      if (this.sim.time - t0 > timeout) return { ok: false, why: `timeout (${label})` };
      if (label) this.detail = label;
      yield;
    }
    return { ok: true };
  },
  // plan a move to a preset name or [q1,q2,q3]; slowAt = trunnion speed cap (m/s) over `dist` metres
  *_armMove(target, label, { dist = 0, v = 0 } = {}) {
    const arm = this.sim.rover.arm;
    if (arm.keepOut) {                       // hook-trunnion interference: never plan a path that crosses an un-seated trunnion from above
      const h = arm.pathHitsKeepOut(target, arm.keepOut);
      if (h.hit) return { ok: false, why: `arm refused ${label}: hook path crosses the trunnion (tip y ${h.tip[1].toFixed(2)} m at ${(h.at * 100).toFixed(0)} % of the move)` };
    }
    const m = arm.moveTo(target);
    if (!m.ok) return { ok: false, why: `arm refused ${label}: ${m.reason}` };
    if (dist > 0 && v > 0 && arm.plan) arm.plan.T = Math.max(arm.plan.T, dist / v);
    const T = arm.plan ? arm.plan.T : 1;
    this.detail = `arm: ${label}`;
    const w = yield* this._armWait(() => arm.done, T * 2 + 40, `arm: ${label}`);
    if (!w.ok) return w;
    for (let i = 0; i < 8; i++) { this.stop(); yield; }       // settle
    return { ok: true };
  },
  // After the gates are open the V-seat (opens upward) hooks leave the trunnion DOWN by k(0.03) and then slide clear (dir +1 toward the
  // front, -1 toward the rover): rising would lift the trunnion again. `tr` = the trunnion centre, rover frame. Leaves the arms stowed.
  *_hooksOff(tr, dir, label = 'hooks off the trunnions') {
    const arm = this.sim.rover.arm;
    arm.keepOut = { x: tr.x, y: tr.y };
    const low = this._qFor(tr.x, tr.y - ARM.releaseDrop), out = this._qFor(tr.x + dir * ARM.releaseSlide, tr.y - ARM.releaseDrop);
    if (!low || !out) return { ok: false, why: `${label}: release pose outside the arm envelope` };
    let r = yield* this._armMove(low, `${label}: down`, { dist: ARM.releaseDrop, v: ARM.pick.seatV * 4 });
    if (r.ok === false) return r;
    r = yield* this._armMove(out, `${label}: clear`);
    if (r.ok === false) return r;
    r = yield* this._armMove('stow', 'stowing the arms');
    arm.keepOut = null;
    return r;
  },
  _qFor(x, y) { const r = ik([x, y], 0, { branch: 'up' }); return r.ok ? r.q : null; },
  _trunnionGround(tx, tz) { // rover-frame trunnion height for soles on the regolith at world (tx, tz)
    const acm = this.sim.acm, h = heightAt(this.sim.terrain, tx, tz);
    return acm.toLocal({ x: tx, y: h, z: tz }).y;
  },

  // ---- pick-up -------------------------------------------------------------------------------------------------------
  *acquireArm(c, opts = {}) {
    const R = this.sim.rover;
    try { return yield* this._acquireArm(c, opts); } finally { R.crawlZone = false; }
  },
  *_acquireArm(c, opts = {}) {
    const sim = this.sim, R = sim.rover, arm = R.arm, acm = sim.acm;
    const T = c.T, W = T.mass * MOON.g;
    const st = (this.acmStats = { tipMax: 0, peakGround: 0, peakHook: 0, groundAtLift: null, latchedBeforeLift: false, retries: 0, t0: sim.time, seatErr: null, abort: null });
    this.log(`${c.id}: arm pick-up, ${T.mass} kg`, 'info');
    if (R.loaded || R.lift.cargo) return { ok: false, why: 'already carrying straddle cargo' };
    if (acm.onDeck || acm.carried) return { ok: false, why: 'an ACM is already on the arms or the deck' };
    if (c.mode !== 'GROUND') return { ok: false, why: `${c.id} is not on the ground` };
    if (R.lift.clamp !== 'open') return { ok: false, why: 'cradle ring jaws are not open' };
    if (arm.faults.size) return { ok: false, why: `arm fault: ${[...arm.faults].join(',')}` };
    if (!arm.stowed) { const r = yield* this._armMove('stow', 'stowing first'); if (r.ok === false) return r; }
    const abortSafe = function* (self, why) { st.abort = why; self.log(`${c.id}: pick-up aborted: ${why}`, 'err'); return yield* self._acmSafeDown(c, why); };

    // 1-2. stand-off and square-up on the trunnion axis (skid steer: drive, back up and re-approach if off)
    let squared = false;
    for (let attempt = 1; attempt <= 3 && !squared; attempt++) {
      yield* this._acmStandoff(c);
      this.phase = `ACM square up ${c.id}`;
      R.brake = false; R.crawlZone = true;
      squared = yield* this._acmSquareUp(c);
      if (!squared) {
        this.log(`${c.id}: not squared up (attempt ${attempt}), backing off to re-approach`, 'warn');
        st.retries++;
        yield* this.backUp(6);
      }
    }
    if (!squared) return { ok: false, why: `${c.id}: could not square up on the trunnion axis` };
    arm.keepOut = null;

    // 3. park
    this.phase = `ACM park ${c.id}`;
    this.stop(); R.brake = true;
    yield* this.wait(1.5, 'park brake set on the driven wheels');
    this.log(`${c.id}: squared up, park brake on`, 'ok');

    for (let attempt = 1; attempt <= 2; attempt++) {
      const tr = acm.trunnionLocal(c);
      const pre = this._qFor(tr.x - ARM.pick.preFwd, tr.y - ARM.pick.preBelow), gr = this._qFor(tr.x, tr.y);
      if (!pre || !gr) { return yield* abortSafe(this, `trunnion at x ${tr.x.toFixed(2)} m, y ${tr.y.toFixed(2)} m is outside the arm envelope`); }
      arm.keepOut = { x: tr.x, y: tr.y };
      // 4. deploy to pre-grasp
      this.phase = `ACM deploy ${c.id}`;
      let r = yield* this._armMove(pre, 'deploy to pre-grasp');
      if (r.ok === false) return yield* abortSafe(this, r.why);
      // 5. pre-lift check
      const chk = arm.canLift(T.mass, { marginF: R.tilt.marginF ?? R.tilt.margin, tip: { x: tr.x, y: tr.y } });
      this.log(`${c.id}: pre-lift check tip ratio ${chk.tipRatio.toFixed(3)} (limit ${ARM.tipRatioMax}), margin ${chk.marginF == null ? '-' : chk.marginF.toFixed(1)} deg, torque x${chk.torqueMargin.toFixed(2)}`, chk.ok ? 'ok' : 'err');
      if (!chk.ok) { this.phase = `ACM refused ${c.id}`; return yield* abortSafe(this, `pre-lift check refused: ${chk.reasons.join(', ')}`); }
      // 6. grasp: hooks to the trunnions, engage, latch
      this.phase = `ACM grasp ${c.id}`;
      r = yield* this._armMove(gr, 'hooks to the trunnions', { dist: ARM.pick.preFwd, v: ARM.pick.seatV * 4 });
      if (r.ok === false) return yield* abortSafe(this, r.why);
      const e = acm.engage(c);
      if (!e.ok) {
        this.log(`${c.id}: engage failed: ${e.reason}`, 'warn');
        if (attempt === 2) return yield* abortSafe(this, `engage failed: ${e.reason}`);
        st.retries++; yield* this._hooksOff(acm.trunnionLocal(c), -1, 'retract'); yield* this.backUp(6); R.brake = false; yield* this._acmSquareUp(c); R.brake = true; yield* this.wait(1, 'park');
        continue;
      }
      arm.keepOut = null;
      arm.latch(true);
      const lw = yield* this._armWait(() => arm.latched, 8, 'latch gates closing');
      if (!lw.ok || !arm.latched) {
        this.log(`${c.id}: latch not confirmed (${lw.why || 'switches disagree'}), retry ${attempt}/2`, 'warn');
        arm.latch(false); yield* this._armWait(() => arm.gatesOpen, 6, 'gates opening'); acm.release(c);
        if (attempt === 2) return yield* abortSafe(this, 'hook latch not confirmed');
        st.retries++; yield* this._hooksOff(acm.trunnionLocal(c), -1, 'retract'); yield* this.backUp(6); R.brake = false; yield* this._acmSquareUp(c); R.brake = true; yield* this.wait(1, 'park');
        continue;
      }
      st.latchedBeforeLift = arm.latched && arm.arms.every((a) => a.seated);
      this.log(`${c.id}: both hooks seated and latched (lat ${(e.lat * 100).toFixed(1)} cm, yaw ${(e.yaw / DEG).toFixed(2)} deg)`, 'ok');
      this._acmGrasp = { x: tr.x, y: tr.y };

      // 7. breakout lift at seatV, watching load split and tip ratio
      this.phase = `ACM breakout ${c.id}`;
      const lift = ARM.pick.clearance + 0.08, qTop = this._qFor(tr.x, tr.y + lift);
      if (!qTop) return yield* abortSafe(this, 'lift target outside the arm envelope');
      r = arm.moveTo(qTop);
      if (!r.ok) return yield* abortSafe(this, `lift refused: ${r.reason}${r.check ? ' ' + r.check.reasons.join(',') : ''}`);
      if (arm.plan) arm.plan.T = Math.max(arm.plan.T, lift / ARM.pick.seatV);
      const tL = sim.time;
      while (c.mode !== 'CARRIED') {
        this.stop();
        const tel = arm.telemetry();
        st.peakGround = Math.max(st.peakGround, (W - c.soilN) / W);
        st.peakHook = Math.max(st.peakHook, (tel.hookLoad[0] + tel.hookLoad[1]) / W);
        st.tipMax = Math.max(st.tipMax, arm.tip.ratio);
        this.detail = `breakout: feet ${(c.soilN / W * 100).toFixed(0)}% of W, hooks ${((tel.hookLoad[0] + tel.hookLoad[1]) / W * 100).toFixed(0)}%, tip ${arm.tip.ratio.toFixed(2)}`;
        if (arm.faults.size) return yield* abortSafe(this, `arm fault during breakout: ${[...arm.faults].join(',')}`);
        if (arm.tip.ratio > ARM.tipRatioMax) return yield* abortSafe(this, `tip ratio ${arm.tip.ratio.toFixed(2)} over ${ARM.tipRatioMax}`);
        if (st.peakHook > ACM_PICK.overloadK) return yield* abortSafe(this, `hook load ${st.peakHook.toFixed(2)} W over ${ACM_PICK.overloadK} W (soil suction)`);
        if (sim.time - tL > 150) return yield* abortSafe(this, 'breakout timeout');
        yield;
      }
      st.groundAtLift = c.soilN;
      this.log(`${c.id}: broke out of the regolith (peak soil load ${st.peakGround.toFixed(3)} W, hook peak ${st.peakHook.toFixed(3)} W), feet clear`, 'ok');
      break;
    }

    // 8. carry over: apex -> pre-place -> place, tip governor on
    const carry = function* (self, target, label, o) {
      const m = arm.moveTo(target);
      if (!m.ok) return { ok: false, why: `${label}: ${m.reason}` };
      if (o && arm.plan) arm.plan.T = Math.max(arm.plan.T, o.dist / o.v);
      const T2 = arm.plan ? arm.plan.T : 1, tc0 = sim.time;
      while (!arm.done) {
        self.stop(); st.tipMax = Math.max(st.tipMax, arm.tip.ratio);
        self.detail = `carry: ${label}, tip ${arm.tip.ratio.toFixed(2)}, margin ${(R.tilt.marginF ?? R.tilt.margin).toFixed(1)} deg`;
        if (arm.faults.size) return { ok: false, why: `arm fault: ${[...arm.faults].join(',')}` };
        if (arm.tip.ratio > ARM.tipRatioMax) return { ok: false, why: `tip ratio ${arm.tip.ratio.toFixed(2)} over ${ARM.tipRatioMax}` };
        if (sim.time - tc0 > 2 * T2 + 60) return { ok: false, why: `timeout (${label})` };
        yield;
      }
      for (let i = 0; i < 8; i++) { self.stop(); yield; }
      return { ok: true };
    };
    this.phase = `ACM swing ${c.id}`;
    let r = yield* carry(this, 'apex', 'to the apex');
    if (r.ok === false) return yield* abortSafe(this, r.why);
    r = yield* carry(this, 'prePlace', 'over the deck pins');
    if (r.ok === false) return yield* abortSafe(this, r.why);
    this.phase = `ACM place ${c.id}`;
    let seated = null;
    for (let attempt = 1; attempt <= 2 && !seated; attempt++) {
      r = yield* carry(this, 'place', 'onto the deck pins', { dist: this._prePlaceH(), v: ARM.pick.seatV });
      if (r.ok === false) return yield* abortSafe(this, r.why);
      yield* this.wait(1.5, 'seating on the pins');
      const s = acm.deckSeat(c);
      if (s.ok) seated = s;
      else {
        this.log(`${c.id}: PIN_TOP: ${s.reason}; lifting and retrying`, 'warn');
        st.retries++;
        r = yield* carry(this, 'prePlace', 'lifting off the pins');
        if (r.ok === false) return yield* abortSafe(this, r.why);
      }
    }
    if (!seated) return yield* abortSafe(this, 'PIN_TOP: could not seat on the deck pins');
    st.seatErr = { ...seated };
    this.log(`${c.id}: on the pins (dx ${(seated.dx * 1000).toFixed(1)} mm, dz ${(seated.dz * 1000).toFixed(1)} mm, dy ${(seated.dy * 1000).toFixed(1)} mm, yaw ${(seated.yaw / DEG).toFixed(2)} deg)`, 'ok');
    // 9. deck latch, release the hooks, stow
    this.phase = `ACM deck latch ${c.id}`;
    arm.deckLatch(true);
    let w = yield* this._armWait(() => arm.deckLatched, 6, 'deck latch closing');
    if (!w.ok) return yield* abortSafe(this, `deck latch: ${w.why}`);
    arm.latch(false);
    w = yield* this._armWait(() => arm.gatesOpen, 6, 'hook gates opening');
    if (!w.ok) return yield* abortSafe(this, `hook gates: ${w.why}`);
    acm.release(c);
    this.log(`${c.id}: deck latch closed, hooks released`, 'ok');
    this.phase = `ACM stow ${c.id}`;
    r = yield* this._hooksOff({ x: arm.placeX, y: ROVER.frame.deckY + c.T.trunnion.y }, +1);   // down k(0.03), then clear (never up through the trunnion)
    if (r.ok === false) return { ok: false, why: r.why };
    w = yield* this._armWait(() => arm.stowed, 20, 'waiting for stow');
    if (!w.ok) return { ok: false, why: w.why };
    st.duration = sim.time - st.t0;
    this.log(`${c.id}: on the deck and stowed in ${(st.duration / 60).toFixed(1)} min (max tip ratio ${st.tipMax.toFixed(2)})`, 'ok');
    R.brake = false;
    return { ok: true };
  },
  _prePlaceH() { return ARM.prePlaceH; },   // config ARM.prePlaceH: the last k(0.06) descended at seatV

  // Put the load down safely and stow (abort path). Anything that stops the arm (stall) leaves it for the operator.
  *_acmSafeDown(c, why) {
    const sim = this.sim, R = sim.rover, arm = R.arm, acm = sim.acm;
    this.phase = `ACM safe set-down ${c.id}`;
    if (arm.faults.has('stall') || arm.faults.has('overtemp')) return { ok: false, why: `${why}; arm servo fault, operator assistance required` };
    // a recorded fault must survive the safe-down (a 'drop' used to be wiped here): keep it in the stats and the log
    const had = [...arm.faults];
    if (this.acmStats) this.acmStats.faultsAtSafeDown = had;
    if (had.length) this.log(`${c.id}: safe-down with arm fault(s) ${had.join(',')} recorded`, 'warn');
    arm.clearFaults(); arm.keepOut = null;
    if (c.mode === 'GROUND' && had.includes('drop')) { /* it fell: nothing to lower, just stow clear of it below */ }
    else if (c.mode === 'GROUND' && arm.payload) {      // still hooked on the ground (stuck feet, overload, timeout): ease the hooks back down to the seat first
      const trg = acm.trunnionLocal(c), qs = this._qFor(trg.x, trg.y);
      if (qs) yield* this._armMove(qs, 'easing the hooks back to the seat', { dist: Math.max(0.1, arm._trunnion().y - trg.y), v: ARM.pick.seatV * 4 });
    } else if (c.mode === 'CARRIED') {
      const tr = arm._trunnion();
      if (tr.x > 2.5) {                                 // out front: lower it onto the regolith
        const wp = acm.toWorld({ x: tr.x, y: 0, z: 0 }), ty = this._trunnionGround(wp.x, wp.z) + c.T.trunnion.y - 0.02, q = this._qFor(tr.x, ty);
        if (q) {
          const r = yield* this._armMove(q, 'setting the load down', { dist: Math.max(0.1, tr.y - ty), v: ARM.pick.seatV * 2 });
          if (r.ok !== false) yield* this._armWait(() => c.mode === 'GROUND' && c.soilN > 0.9 * c.T.mass * MOON.g, 25, 'soil takes the weight', c);
        }
      } else {                                          // over the deck: seat it on the pins
        let r = yield* this._armMove('prePlace', 'over the pins');
        if (r.ok !== false) r = yield* this._armMove('place', 'onto the pins', { dist: this._prePlaceH(), v: ARM.pick.seatV * 2 });
        const s = acm.deckSeat(c);
        if (s.ok) { arm.deckLatch(true); yield* this._armWait(() => arm.deckLatched, 6, 'deck latch'); }
      }
    }
    arm.latch(false);
    yield* this._armWait(() => arm.gatesOpen, 6, 'gates opening');
    if (c.mode !== 'CARRIED') acm.release(c);
    if (arm.payload && c.mode === 'DECK') arm.release();
    if (arm.stowed) { /* refused before the arms moved */ }
    else if (c.mode === 'DECK') yield* this._hooksOff({ x: arm.placeX, y: ROVER.frame.deckY + c.T.trunnion.y }, +1);
    else if (c.mode === 'GROUND' && !had.includes('drop')) yield* this._hooksOff(acm.trunnionLocal(c), -1);
    else { yield* this._armMove('prePlace', 'hooks clear'); yield* this._armMove('stow', 'stowing the arms'); }
    arm.keepOut = null;
    R.brake = false;
    return { ok: false, why };
  },

  // ---- approach and square-up --------------------------------------------------------------------------------------
  *_acmStandoff(c) {
    const sim = this.sim, R = sim.rover;
    this.phase = `ACM approach ${c.id}`;
    const s = this.acmSense(c);
    const near = s.f > ARM.pick.xBody - 0.6 && s.f < ARM.pick.xBody + 16 && Math.abs(s.lat) < 1.2 && Math.abs(s.yawE) < 8 * DEG;
    if (near) return true;
    // the stand-off point on the ACM x axis: heading psi = ACM yaw or its opposite, the one whose point is nearer
    const t = c.body.translation(), p = R.pose();
    const q = c.body.rotation(), fx = 1 - 2 * (q.y * q.y + q.z * q.z), fz = 2 * (q.x * q.z - q.w * q.y), ya = Math.atan2(-fz, fx);
    const back = ARM.pick.xBody + ACM_TOL.standoff;
    let best = null;
    for (const psi of [ya, ya + Math.PI]) {
      const P = { x: t.x - Math.cos(psi) * back, z: t.z + Math.sin(psi) * back };
      const d = Math.hypot(P.x - p.x, P.z - p.z);
      if (!best || d < best.d) best = { P, psi, d };
    }
    // the offload report is good to about 0.8 m / 2 deg
    const goal = { x: best.P.x + 0.8, z: best.P.z - 0.6 };
    const e = this.toEst({ x: goal.x, y: p.y, z: goal.z }), yawE = this.dirToEst({ x: Math.cos(best.psi), z: -Math.sin(best.psi) });
    this.log(`${c.id}: stand-off point ${best.d.toFixed(0)} m away, heading ${(wrap(best.psi) / DEG).toFixed(0)} deg`, 'info');
    if (best.d > 30) {
      const r = yield* this.navTo({ x: e.x, z: e.z, name: `stand-off ${c.id}`, tol: 2.0, yaw: yawE, yawTol: 3 * DEG });
      if (r && r.ok === false) return false;
    } else {
      yield* this._acmGoto(e, yawE, 1.0, 20 * DEG);
    }
    return true;
  },
  // short local drive in the estimate frame: pivot toward the point, drive, pivot to the heading
  *_acmGoto(pt, yaw, tol = 0.5, yawTol = 2 * DEG) {
    const R = this.sim.rover;
    R.brake = false;
    let e = this.est();
    if (Math.hypot(pt.x - e.x, pt.z - e.z) > tol + 0.5) {
      yield* this.alignYaw(Math.atan2(-(pt.z - e.z), pt.x - e.x), 2 * DEG);
      const t0 = this.sim.time;
      while (this.sim.time - t0 < 120) {
        const rel = this.relBody(pt);
        if (rel.f < tol * 0.5) break;
        this.cmd(clamp(rel.f * 0.4, 0.15, 1.2), 0, clamp(-0.6 * Math.atan2(rel.r, Math.max(1, rel.f)), -0.08, 0.08));
        this.detail = `local drive: ${rel.f.toFixed(1)} m to go`;
        yield;
      }
      this.stop();
    }
    yield* this.alignYaw(yaw, yawTol);
  },
  // Final approach on the trunnion axis using the close-range sensing. Returns true when stopped squared up.
  *_acmSquareUp(c) {
    const R = this.sim.rover, T0 = this.sim.time;
    let s = this.acmSense(c);
    this._wzI = 0; this._trkI = 0;
    // a short run-up cannot take out a lateral / yaw error (skid steer cannot strafe): reverse to a 9 m run-up first
    const need = 4.5 + Math.abs(s.lat) / Math.tan(15 * DEG);       // run-up needed to intercept the axis at <= 15 deg
    if ((s.along < need || s.along < 6) && (Math.abs(s.lat) > 0.1 || Math.abs(s.r) > ACM_TOL.lat || Math.abs(s.yawE) > ACM_TOL.yaw)) {
      this.phase = `ACM run-up ${c.id}`;
      yield* this.backUp(Math.min(16, Math.max(need, 8) + 1 - s.along));
      this.phase = `ACM square up ${c.id}`; this._wzI = 0;
    }
    let ph = 'aim', lat0 = 0, along0 = s.along;
    while (this.sim.time - T0 < 240) {
      s = this.acmSense(c);
      const along = s.along;
      if (ph === 'aim') along0 = along;
      if (Math.abs(along) <= ACM_TOL.along && Math.abs(s.r) <= ACM_TOL.lat && Math.abs(s.yawE) <= ACM_TOL.yaw) {
        this.stop();
        for (let i = 0; i < 15; i++) yield;
        s = this.acmSense(c);
        if (Math.abs(s.r) <= ACM_TOL.lat && Math.abs(s.yawE) <= ACM_TOL.yaw) return true;
        continue;
      }
      if (along < -ACM_TOL.along - 0.3 && Math.abs(s.lat) < 0.3) { // overshot: creep back
        this.cmdCurv(-0.12, 0); this.detail = `squaring up: backing ${(-along).toFixed(2)} m`; yield; continue;
      }
      if (along < -2) return false;
      const v = along > 0 ? clamp(along * 0.3, 0.05, along > 6 ? 0.9 : 0.5) : 0.05;
      if (along <= ACM_TOL.along) {     // arrived: only a pivot can fix yaw now, and it swings the grasp point by ~xBody * dyaw.
        // Pivot only when the predicted lateral error after it is still inside the tolerance; otherwise back up and re-approach.
        if (Math.abs(s.yawE) <= ACM_TOL.yaw) { this.stop(); return false; }     // yaw fine, lateral off: nothing a pivot can fix
        const rAfter = s.f * Math.sin(s.yawE) + s.r * Math.cos(s.yawE);
        if (Math.abs(s.r) > ACM_TOL.lat * 1.5 || Math.abs(rAfter) > ACM_TOL.lat * 0.9) { this.stop(); return false; }
        this.cmd(0, 0, clamp(0.9 * s.yawE, -0.03, 0.03));
        this.detail = `squaring up: pivot, yaw ${(s.yawE / DEG).toFixed(2)} deg, lat ${(s.r * 100).toFixed(1)} cm`; yield; continue;
      }
      // Skid steer turns weakly while rolling (yaw efficiency ~0.2 at crawl speeds: arcs cannot take out a 1 m cross-track error in the
      // run-up), but it pivots well. A pivot about the body origin keeps the origin's offset `lat` from the axis, so: (1) pivot to
      // a heading that intercepts the axis ~3.5 m before the grasp, (2) drive straight with a heading hold until the origin is on
      // the axis, (3) pivot to the axis heading (far from the ACM, so the swing is harmless), (4) drive the last metres straight with a
      // light Stanley-style correction (kap = k_psi yawE - atan(k_e lat / v)).
      const lat = s.lat, yawAim = ph === 'aim' ? clamp(Math.atan2(lat, Math.max(along - 4, 1.5)), -15 * DEG, 15 * DEG) : 0;
      if (ph === 'aim' && Math.abs(lat) < 0.03) ph = 'align';
      if (ph === 'run' && (Math.abs(lat) < 0.03 || lat * lat0 < 0 || along < 3)) ph = 'align';
      if (ph === 'align' && Math.abs(s.yawE) < 0.3 * DEG) ph = 'final';
      if (ph === 'aim' || ph === 'align') {
        const tgt = ph === 'aim' ? yawAim : 0, err = s.yawE - tgt;
        if (ph === 'aim' && Math.abs(err) < 0.5 * DEG) { ph = 'run'; lat0 = lat; this.stop(); continue; }
        this.cmd(0, 0, clamp(0.9 * err, -0.04, 0.04));
        this.detail = `squaring up: pivot to ${(tgt / DEG).toFixed(1)} deg (yaw ${(s.yawE / DEG).toFixed(2)}), lat ${(lat * 100).toFixed(0)} cm`;
        yield; continue;
      }
      const tgtY = ph === 'run' ? clamp(Math.atan2(lat0, Math.max(along0 - 4, 1.5)), -15 * DEG, 15 * DEG) : 0;
      const vv = Math.min(v, along < 3 ? 0.3 : 9), vEff = Math.max(vv, 0.3);
      const kap = clamp(3 * (s.yawE - tgtY) - (ph === 'final' ? Math.atan(0.5 * lat / vEff) : 0), -0.1, 0.1);
      this.cmd(vv, 0, this.trackCurv(vv, kap));
      this.detail = `squaring up: along ${along.toFixed(2)} m, lat ${(s.r * 100).toFixed(1)} cm, yaw ${(s.yawE / DEG).toFixed(2)} deg (${ph})`;
      yield;
    }
    this.stop();
    return false;
  },

  // ---- drive away with the ACM on the deck -------------------------------------------------------------------------
  *driveAway(dist = 20, vmax = 1.0) {
    const sim = this.sim, R = sim.rover, acm = sim.acm;
    if (!acm.onDeck) return { ok: false, why: 'no ACM on the deck' };
    if (!R.arm.stowed) return { ok: false, why: 'arms not stowed' };
    this.phase = 'ACM drive away'; R.brake = false;
    const e0 = this.est(), t0 = sim.time; let stuck = 0;
    this._wzI = 0;
    for (;;) {
      const e = this.est(), d = Math.hypot(e.x - e0.x, e.z - e0.z);
      if (d >= dist) break;
      if (sim.time - t0 > dist / 0.1 + 60) { this.stop(); return { ok: false, why: 'drive-away timeout' }; }
      const v = Math.min(vmax, clamp((dist - d) * 0.4, 0.3, 9), 0.2 + 0.25 * (sim.time - t0));
      this.cmdCurv(v, clamp(1.0 * wrap(e0.yaw - e.yaw), -0.05, 0.05));
      stuck = R.telemetry().speed < 0.03 && sim.time - t0 > 15 ? stuck + this.dt : 0;
      if (stuck > 10) { this.stop(); return { ok: false, why: 'stalled while driving away' }; }
      this.detail = `driving away with the ACM on deck: ${d.toFixed(1)} / ${dist} m, tip margin ${R.tilt.margin.toFixed(0)} deg`;
      yield;
    }
    this.stop(); R.brake = true;
    for (let i = 0; i < 30; i++) yield;
    this.log(`Drove ${dist} m with the ACM on the deck (cap ${R.speedCap().toFixed(2)} m/s)`, 'ok');
    return { ok: true };
  },

  // ---- place on the ground (reverse of the pick) -------------------------------------------------------------------
  *deliverArm(x, z, opts = {}) {
    const R = this.sim.rover;
    try { return yield* this._deliverArm(x, z, opts); } finally { R.crawlZone = false; }
  },
  *_deliverArm(x, z, opts = {}) {
    const sim = this.sim, R = sim.rover, arm = R.arm, acm = sim.acm;
    const c = acm.onDeck;
    if (!c) return { ok: false, why: 'no ACM on the deck' };
    if (!arm.stowed) return { ok: false, why: 'arms not stowed' };
    if (arm.faults.size) return { ok: false, why: `arm fault: ${[...arm.faults].join(',')}` };
    const T = c.T, W = T.mass * MOON.g, st = (this.acmStats = { tipMax: 0, t0: sim.time, placeErr: null, abort: null });
    const psi = opts.yaw ?? R.pose().yaw;
    this.log(`${c.id}: delivering to (${x.toFixed(1)}, ${z.toFixed(1)})`, 'info');
    // 1. drive so the target is ARM.pick.xBody ahead on the heading psi
    this.phase = `ACM deliver ${c.id}: approach`;
    const back = ARM.pick.xBody, p = R.pose();
    const stand = { x: x - Math.cos(psi) * back, z: z + Math.sin(psi) * back };
    const e = this.toEst({ x: stand.x, y: p.y, z: stand.z }), yawE = this.dirToEst({ x: Math.cos(psi), z: -Math.sin(psi) });
    if (Math.hypot(stand.x - p.x, stand.z - p.z) > 30) { const r = yield* this.navTo({ x: e.x, z: e.z, name: 'receiving zone', tol: 1.5, yaw: yawE, yawTol: 2 * DEG }); if (r && r.ok === false) return r; }
    else yield* this._acmGoto(e, yawE, 0.25, 1 * DEG);
    // fine closing on the receiving point (odometry / map): creep to within 0.15 m along the axis
    { const t0 = sim.time;
      while (sim.time - t0 < 60) {
        const q = R.pose(), f = (stand.x - q.x) * Math.cos(q.yaw) - (stand.z - q.z) * Math.sin(q.yaw);
        if (Math.abs(f) < 0.15) break;
        this.cmdCurv(clamp(f * 0.3, -0.15, 0.4), 0); this.detail = `closing on the receiving point: ${f.toFixed(2)} m`; yield;
      } }
    this.phase = `ACM deliver ${c.id}: park`;
    this.stop(); R.brake = true;
    yield* this.wait(1.5, 'park brake set');
    // 2. deploy over the deck, unlatch, hook
    // re-hook FROM BELOW: the V-seat opens upward, so the hooks come up under the trunnions (down-and-out is the reverse of the release)
    const trD = { x: arm.placeX, y: ROVER.frame.deckY + T.trunnion.y };
    arm.keepOut = trD;
    const qLowIn = this._qFor(trD.x + ARM.releaseSlide, trD.y - ARM.releaseDrop), qLow = this._qFor(trD.x, trD.y - ARM.releaseDrop);
    if (!qLowIn || !qLow) return { ok: false, why: 'deck trunnion release pose outside the arm envelope' };
    let r = yield* this._armMove(qLowIn, 'beside and below the trunnions');
    if (r.ok === false) { arm.keepOut = null; return r; }
    r = yield* this._armMove(qLow, 'under the trunnions');
    if (r.ok === false) { arm.keepOut = null; return r; }
    r = yield* this._armMove('place', 'rising to the trunnions', { dist: ARM.releaseDrop, v: ARM.pick.seatV });
    if (r.ok === false) { arm.keepOut = null; return r; }
    arm.deckLatch(false);
    let w = yield* this._armWait(() => arm.deckOpen, 6, 'deck latch opening');
    if (!w.ok) return w;
    const dr = acm.deckRelease(c);
    if (!dr.ok) { arm.keepOut = null; return { ok: false, why: `unlatch: ${dr.reason}` }; }
    arm.keepOut = null;
    w = yield* this._armWait(() => arm.latched, 8, 'hook gates closing');
    if (!w.ok) return w;
    this.log(`${c.id}: hooks latched, lifting off the pins`, 'ok');
    // 3. lift, swing out
    const guard = function* (self, target, label, o) {
      const m = arm.moveTo(target); if (!m.ok) return { ok: false, why: `${label}: ${m.reason}` };
      if (o && arm.plan) arm.plan.T = Math.max(arm.plan.T, o.dist / o.v);
      const T2 = arm.plan ? arm.plan.T : 1, t1 = sim.time;
      while (!arm.done) {
        self.stop(); st.tipMax = Math.max(st.tipMax, arm.tip.ratio); self.detail = `deliver: ${label}, tip ${arm.tip.ratio.toFixed(2)}`;
        if (arm.faults.size) return { ok: false, why: `arm fault: ${[...arm.faults].join(',')}` };
        if (arm.tip.ratio > ARM.tipRatioMax) return { ok: false, why: `tip ratio ${arm.tip.ratio.toFixed(2)} over ${ARM.tipRatioMax}` };
        if (sim.time - t1 > 2 * T2 + 60) return { ok: false, why: `timeout (${label})` };
        yield;
      }
      for (let i = 0; i < 8; i++) { self.stop(); yield; }
      return { ok: true };
    };
    r = yield* guard(this, 'prePlace', 'lifting off the pins'); if (r.ok === false) return yield* this._acmSafeDown(c, r.why);
    r = yield* guard(this, 'apex', 'to the apex'); if (r.ok === false) return yield* this._acmSafeDown(c, r.why);
    // 4. over the receiving point, lower until the soil takes the weight
    const tw = acm.toWorld({ x: ARM.pick.xBody, y: 0, z: 0 });
    const gTrun = this._trunnionGround(tw.x, tw.z) + T.trunnion.y;
    const qAbove = this._qFor(ARM.pick.xBody, gTrun + 0.25), qDown = this._qFor(ARM.pick.xBody, gTrun - 0.02);
    if (!qAbove || !qDown) return yield* this._acmSafeDown(c, 'receiving point is outside the arm envelope');
    r = yield* guard(this, qAbove, 'over the receiving point'); if (r.ok === false) return yield* this._acmSafeDown(c, r.why);
    this.phase = `ACM deliver ${c.id}: set down`;
    r = yield* guard(this, qDown, 'lowering to the regolith', { dist: 0.27, v: ARM.pick.seatV }); if (r.ok === false) return yield* this._acmSafeDown(c, r.why);
    // the regolith sinks under the weight: keep easing the one-sided hooks below the trunnion until the feet carry it all
    const qDeep = this._qFor(ARM.pick.xBody, gTrun - 0.08);
    if (qDeep && !(c.mode === 'GROUND' && c.soilN > 0.97 * W)) { r = yield* guard(this, qDeep, 'easing the hooks off the trunnions', { dist: 0.06, v: ARM.pick.seatV }); if (r.ok === false) return yield* this._acmSafeDown(c, r.why); }
    w = yield* this._armWait(() => c.mode === 'GROUND' && c.soilN > 0.95 * W, 30, 'soil takes the weight');
    if (!w.ok) return yield* this._acmSafeDown(c, w.why);
    // 5. release and stow
    arm.latch(false);
    w = yield* this._armWait(() => arm.gatesOpen, 6, 'hook gates opening'); if (!w.ok) return w;
    acm.release(c);
    r = yield* this._hooksOff(acm.trunnionLocal(c), -1); if (r.ok === false) return r;
    yield* this._armWait(() => arm.stowed, 20, 'waiting for stow');
    yield* this.wait(3, 'ACM settling');
    const t = c.body.translation();
    st.placeErr = Math.hypot(t.x - x, t.z - z);
    st.duration = sim.time - st.t0;
    this.log(`${c.id}: on the ground ${st.placeErr.toFixed(2)} m from the target in ${(st.duration / 60).toFixed(1)} min`, 'ok');
    R.brake = false;
    return { ok: true };
  },

  // ---- missions ------------------------------------------------------------------------------------------------------
  *missionPickupArm(c, opts = {}) {
    const r = yield* this.acquireArm(c, opts);
    if (r.ok === false) return r;
    if (opts.driveAway) return yield* this.driveAway(opts.driveAway);
    return { ok: true };
  },
  *missionPlaceArm(x, z, opts = {}) { return yield* this.deliverArm(x, z, opts); },
};
