// AEGIS-V1 (team design) full-scale vehicle model on top of Rapier.
//  - Sprung chassis = Rapier rigid body (real collisions with rocks, structures, cargo)
//  - 6 rocker-bogie wheel stations, each with a 1-DOF unsprung mass on a long-stroke active strut
//    (the team's body_control.js: the links follow the slow ground shape, the struts take the rest)
//  - wheels: cylinder shape-casts for contact, radial tyre spring, Bekker/Janosi soil forces,
//    implicit spin dynamics, hub gearmotor with PI speed control, spring brakes, skid steer
//  - active levelling + kneel / rise ride control; the module is lifted by the deck V-saddles
//    (real contact) and locked by the two ring clamps (module_carry.js)
//  - battery power bus (charged at the AEGIS garage), motor thermal model
import * as THREE from 'three';
import { ROVER, K, MOON, REGOLITH, CARGO_TYPES } from './config.js';
export const VEHICLE = ROVER;
import { wheelSoil, shearFactor } from './terramechanics.js';
import { G_ROVER, G_CARGO, G_CARGO_HELD, Q_WHEEL, Q_TERRAIN, yawQuat } from './world.js';
import { heightAt } from './terrain.js';

const V = () => new THREE.Vector3();
const ROVER_DT = 1 / 120;
const UP = new THREE.Vector3(0, 1, 0);
const tmpQ = new THREE.Quaternion(), tmpQ2 = new THREE.Quaternion();
const qX90 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);

export class Rover {
  constructor(sim, pose) {
    this.sim = sim;
    const { RAPIER, world, terrain } = sim;
    this.RAPIER = RAPIER;
    const gy = heightAt(terrain, pose.x, pose.z);
    const desc = RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(pose.x, gy + ROVER.rideHeight + 0.04, pose.z)
      .setRotation(yawQuat(pose.yaw))
      .setAdditionalMassProperties(ROVER.sprungMass, ROVER.comLocal, ROVER.inertia, { x: 0, y: 0, z: 0, w: 1 })
      .setCanSleep(false).setLinearDamping(0.0).setAngularDamping(0.02);
    this.body = world.createRigidBody(desc);
    const F = ROVER.frame, SY = ROVER.seatY, MR = CARGO_TYPES.cyl40.radius;
    const col = (d) => world.createCollider(d.setDensity(0).setFriction(0.6).setCollisionGroups(G_ROVER), this.body);
    // belly tub + deck (team model: tub y 0.065-0.185, deck top 0.225, z +-0.146 at 1:16.3)
    col(RAPIER.ColliderDesc.cuboid(F.halfLen * 0.9, (F.deckY - F.tubY0) / 2, F.tubZ).setTranslation(0, (F.deckY + F.tubY0) / 2, 0));
    col(RAPIER.ColliderDesc.cuboid(F.halfLen * 0.95, 0.12, F.tubZ * 1.25).setTranslation(0, F.deckY - 0.12, 0));
    col(RAPIER.ColliderDesc.cuboid(1.38, 0.65, 1.63).setTranslation(3.08, F.deckY + 0.65, 0));                    // avionics pod (upperShell)
    for (const sz of [-1, 1]) col(RAPIER.ColliderDesc.cuboid(0.65, 0.5, 0.65).setTranslation(4.08, F.deckY + 0.5, sz * 2.45)); // LiDAR plinths
    // V saddles at the two rings: a flat seat and two 15 deg flanks tangent to the module bottom
    for (const rx of [-ROVER.ringX, ROVER.ringX]) {
      col(RAPIER.ColliderDesc.cuboid(0.41, (SY - F.deckY) / 2, 0.9).setTranslation(rx, (SY + F.deckY) / 2, 0));
      for (const sz of [-1, 1]) {
        const th = 0.26, zc = Math.sin(th) * (MR + 0.35), yc = SY + MR - Math.cos(th) * (MR + 0.35);
        const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -sz * th);
        col(RAPIER.ColliderDesc.cuboid(0.41, 0.35, 0.9).setTranslation(rx, yc, sz * zc).setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }));
      }
    }
    const B = ROVER.bogie;
    for (const [cx, cz] of ROVER.corners) col(RAPIER.ColliderDesc.ball(ROVER.wheel.radius * 0.8).setTranslation(cx, B.hardpointY - B.Lmin, cz)); // wheel-arch bump stops

    this.cyl = new RAPIER.Cylinder(ROVER.wheel.width / 2, ROVER.wheel.radius);
    const Ffree = ROVER.sprungMass * MOON.g / ROVER.corners.length / ROVER.susp.k;
    this.bogies = [];
    let idx = 0;
    for (const [cx, cz, name] of ROVER.corners) {
      const b = {
        name, x: cx, z: cz,
        L: B.Lnom, Ldot: 0, Lfree: B.Lnom + Ffree, level: 0, beamS: 0,
        steer: 0, steerTarget: 0, hpVelPrev: null, strutF: 0, Lslow: B.Lnom,
        wheels: [{
          id: idx++, off: 0, omega: 0, angle: 0, target: 0, integ: 0, torque: 0,
          slip: 0, alpha: 0, sink: 0, load: 0, contact: false, delta: 0, deltaPrev: 0,
          H: 0, Rc: 0, Fx: 0, Fy: 0, power: 0, temp: 15, fault: null,
          cp: V(), cn: V(), dragAccum: 0,
        }],
      };
      this.bogies.push(b);
    }
    this.wheels = this.bogies.flatMap((b) => b.wheels);
    this.sides = [this.bogies.filter((b) => b.z < 0), this.bogies.filter((b) => b.z > 0)]; // left / right rocker-bogie

    // command interface (body-frame velocity request)
    this.cmd = { vx: 0, vz: 0, wz: 0 };
    this.brake = true;           // parking brake engaged at power-up
    this.estop = false;
    this.powered = true;
    // Drive limits. rpm values are PROTOTYPE motor rpm (300 no-load, 180 normal cap); wheel rpm = proto / sqrt(K).
    // rpmMode 'auto' picks crawl 120 / cruise 180 / turbo 300 from the terrain (see _smartRpm).
    this.limits = { rpmCeil: ROVER.motor.protoNoLoadRpm, speed: 99, torquePct: 100 };
    this.rpm = { mode: 'auto', fixed: 180, tier: 'cruise', proto: 180, wheel: ROVER.motor.capRpmWheel, rough: 0, why: 'cruise' };
    this.turboOk = true; // set by the autonomy / driver when the way ahead is clear and straight
    this._smart = { rough: 0, turboT: 0 };
    this.leveling = true;
    this.rideOffset = 0;
    this.lights = true;
    this.previewK = null;

    this.lift = { pos: 0, target: 0, stageX: 0, stageXT: 0, stageZ: 0, stageZT: 0, clamp: 'open', clampT: 0, cargo: null, joint: null, base: null, trolleyX: 0, msg: '' };
    this.power = { soc: 0.92, wh: ROVER.battery.capacityWh * 0.92, solarW: 0, loadW: 0, netW: 0, driveW: 0, mastYaw: 0, sunlit: 1, energyUsedWh: 0, energyGenWh: 0, regenWh: 0 };
    this._shadowT = 0;
    this.odo = { dist: 0 };
    this.stats = { maxTilt: 0, minMargin: 99 };
    this.tilt = { roll: 0, pitch: 0, margin: 99 };
    this._prevPos = null;
    // scratch
    this._v = { pos: V(), lin: V(), ang: V(), com: V(), Yb: V(), Xb: V(), Zb: V(), Q: new THREE.Quaternion() };
  }

  get mass() { return ROVER.sprungMass + ROVER.corners.length * ROVER.unsprungMassPerBogie; }
  get loaded() { return !!(this.lift.cargo && this.lift.joint); }

  pose() {
    const t = this.body.translation(), r = this.body.rotation();
    const q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
    const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
    return { x: t.x, y: t.y, z: t.z, q, yaw: Math.atan2(-fwd.z, fwd.x), fwd };
  }

  // ---- kinematic command -> wheel speed targets. Skid steer (team design v6+): fixed wheels,
  // each side runs at v +- w * z; lateral velocity requests are ignored (no crab on skid steer).
  // top ground speed allowed right now (m/s): smart-rpm tier, the user speed limit, and the payload limit
  speedCap() {
    const v = this.rpm.wheel * 2 * Math.PI / 60 * ROVER.wheel.radius;
    return Math.min(v, this.limits.speed, this.loaded ? ROVER.speedLimitLoaded : 99);
  }

  // Smart rpm: the motors are 300 rpm units capped at 180 for normal work. Turbo (up to 300) is released
  // only on smooth, flat, grippy ground with nothing loaded and a stable chassis; crawl (120) on slopes,
  // slip, rough ground or a marginal tip angle, where torque control matters more than speed.
  _smartRpm(dt) {
    const M = ROVER.motor, k = 1 / Math.sqrt(K), S = this._smart, R = this.rpm;
    const av = this.body.angvel();
    const rate = Math.hypot(av.x, av.z);
    let strut = 0, slip = 0, n = 0, hot = 0;
    for (const w of this.wheels) { slip += Math.abs(w.slip); hot = Math.max(hot, w.temp); n++; }
    for (const b of this.bogies) strut += Math.abs(b.Ldot);
    strut /= this.bogies.length; slip /= n;
    const speed = this.body.linvel(); const v = Math.hypot(speed.x, speed.z);
    // roughness = chassis rocking + strut travel rate, normalised by speed
    const raw = (rate * 6 + strut) / Math.max(1, v);
    S.rough += (raw - S.rough) * Math.min(1, dt / 0.8);
    const slope = Math.max(Math.abs(this.tilt.pitch), Math.abs(this.tilt.roll)) * 180 / Math.PI;
    const mg = this.tilt.marginF ?? this.tilt.margin;
    const loaded = this.loaded;
    let tier = 'cruise', why = 'normal ground';
    if (slope > 12 || (loaded && slope > 7)) { tier = 'crawl'; why = `slope ${slope.toFixed(0)} deg`; }
    else if (slip > 0.22) { tier = 'crawl'; why = `wheel slip ${(slip * 100).toFixed(0)}%`; }
    else if (S.rough > 0.7) { tier = 'crawl'; why = 'rough ground'; }
    else if (mg < 14) { tier = 'crawl'; why = 'low tip margin'; }
    else if (!loaded && slope < 7 && S.rough < 0.5 && slip < 0.14 && mg > 26 && hot < 95 && this.power.soc > 0.15 && this.turboOk) {
      S.turboT += dt; why = 'flat, smooth, clear';
      if (S.turboT > 0.6) tier = 'turbo'; else why = 'turbo arming';
    }
    if (tier !== 'turbo') S.turboT = 0;
    let want = tier === 'turbo' ? M.protoNoLoadRpm : tier === 'crawl' ? 120 : M.protoCapRpm;
    if (R.mode === 'fixed') { want = R.fixed; tier = 'manual'; why = `locked at ${R.fixed} rpm`; }
    want = Math.min(want, this.limits.rpmCeil);
    // up slowly (60 rpm/s), down immediately
    R.proto = want < R.proto ? want : Math.min(want, R.proto + 150 * dt);
    R.wheel = R.proto * k;
    R.tier = tier; R.why = why; R.rough = S.rough;
  }

  _allocate(dt, stopping = false) {
    const r = ROVER.wheel.radius, A = ROVER.accel;
    let { vx, wz } = this.cmd;
    if (stopping) { vx = 0; wz = 0; }
    const lim = this.speedCap();
    if (Math.abs(vx) > lim) vx = Math.sign(vx) * lim;
    // tip-over governor: lateral acceleration v*w and longitudinal acceleration are limited so the
    // resultant stays inside the support polygon (loaded CoM ~7.8 m up on a 7 m x 7.7 m footprint)
    // dynamic tip margin: the static margin minus the lean the manoeuvre adds (atan(a/g)); keep 8 deg spare
    const mgn = this.tilt.marginF ?? this.tilt.margin;
    const aTip = MOON.g * Math.tan(Math.min(60, Math.max(0.3, mgn - 8)) * Math.PI / 180);
    const aLat = Math.min(this.loaded ? A.latLoaded : A.lat, aTip), aLon = Math.min(this.loaded ? A.lonLoaded : A.lon, aTip * 1.2) * (stopping ? A.estopFactor : 1);
    if (Math.abs(vx * wz) > aLat) vx = Math.sign(vx) * aLat / Math.abs(wz);
    const f = this._cmdF || (this._cmdF = { vx: 0, wz: 0 });
    f.vx += Math.max(-aLon * dt, Math.min(aLon * dt, vx - f.vx));
    f.wz += Math.max(-A.yaw * dt, Math.min(A.yaw * dt, wz - f.wz));
    vx = f.vx; wz = f.wz;
    const rpmOmega = this.rpm.wheel * 2 * Math.PI / 60;
    for (const b of this.bogies) {
      b.steerTarget = 0; b.steer = 0;
      const speed = vx + wz * b.z;
      for (const w of b.wheels) w.target = Math.max(-rpmOmega, Math.min(rpmOmega, speed / r));
    }
  }

  step(dt) {
    const { world } = this.sim;
    const sv = this._v;
    const t = this.body.translation(), r = this.body.rotation();
    sv.pos.set(t.x, t.y, t.z); sv.Q.set(r.x, r.y, r.z, r.w);
    const lv = this.body.linvel(), av = this.body.angvel(), c = this.body.worldCom();
    sv.lin.set(lv.x, lv.y, lv.z); sv.ang.set(av.x, av.y, av.z); sv.com.set(c.x, c.y, c.z);
    const Q = sv.Q, Yb = sv.Yb.set(0, 1, 0).applyQuaternion(Q), Xb = sv.Xb.set(1, 0, 0).applyQuaternion(Q);
    const B = ROVER.bogie, Wp = ROVER.wheel, S = ROVER.susp, M = ROVER.motor;
    const g = MOON.g, mu = ROVER.unsprungMassPerBogie;
    const totalMass = this.mass + (this.loaded ? this.lift.cargo.T.mass : 0);

    // E-STOP is a category-1 stop: the drives decelerate the vehicle at the tip-safe rate, then the
    // spring brakes set and drive power is removed (an instant lock-up would tip a loaded AEGIS).
    const speedNow = sv.lin.length();
    if (this.estop && !this._estopT0) this._estopT0 = this.sim.time;
    if (!this.estop) this._estopT0 = 0;
    const stopping = this.estop && this.powered && speedNow > 0.04 && this.sim.time - this._estopT0 < 12;
    const halt = (this.estop && !stopping) || !this.powered;
    if (halt || stopping) { this.cmd.vx = this.cmd.vz = this.cmd.wz = 0; }
    if (halt) this._cmdF = { vx: 0, wz: 0 };
    this._allocate(dt, stopping);
    const f = this._cmdF;
    const cmdZero = Math.abs(f.vx) + Math.abs(f.wz) < 1e-3 && Math.abs(this.cmd.vx) + Math.abs(this.cmd.wz) < 1e-4;
    const parkBrake = halt || this.brake || (cmdZero && speedNow < 0.03);

    const vel = V(), tmp = V(), tl = V(), tlat = V(), F = V(), hp = V(), wc = V(), dirW = V(), rel = V();
    let nContacts = 0;
    for (const b of this.bogies) for (const w of b.wheels) if (w.contact) nContacts++;
    const mEff = 0.6 * totalMass / Math.max(1, nContacts);
    const Lstart = B.Lmin - 0.32;
    const maxCast = B.Lmax + B.halfSpan * Math.sin(B.maxBeamAngle) + 0.12 - Lstart;
    const gvec = tmp.set(0, -g, 0).clone();
    let driveW = 0;
    const smax = Math.sin(B.maxBeamAngle);

    // ---- rocker-bogie: each side's rocker + bogie share the load equally across its three wheels
    // (rocker pivot at x=0, bogie pivot at x=-L/3: F_front = F_mid = F_rear for any wheel heights). The
    // mean spring force of the side is applied to every wheel; the differential bar to the other side
    // keeps a fraction `diff` of each wheel's own spring (that is what holds the body pitch).
    {
      const kr = totalMass / this.mass, S0 = ROVER.susp, RB = ROVER.rb;
      for (const side of this.sides) {
        let m = 0;
        for (const b of side) { b._own = S0.k * kr * (b.Lfree + b.level - b.L) - S0.c * kr * b.Ldot; m += b._own; }
        m /= side.length;
        for (const b of side) b._fs = (1 - RB.diff) * m + RB.diff * b._own;
      }
    }
    for (const b of this.bogies) {
      // hardpoint and its velocity
      hp.set(b.x, B.hardpointY, b.z).applyQuaternion(Q).add(sv.pos);
      rel.copy(hp).sub(sv.com);
      const vhp = V().copy(sv.ang).cross(rel).add(sv.lin);
      let aAlong = 0;
      if (b.hpVelPrev) aAlong = vhp.clone().sub(b.hpVelPrev).dot(Yb) / dt;
      b.hpVelPrev = vhp;
      const cs = Math.cos(b.steer), sn = Math.sin(b.steer);
      dirW.set(cs, 0, -sn).applyQuaternion(Q);
      tmpQ.setFromAxisAngle(UP, b.steer); tmpQ2.copy(Q).multiply(tmpQ).multiply(qX90);
      const rot = { x: tmpQ2.x, y: tmpQ2.y, z: tmpQ2.z, w: tmpQ2.w };
      const Lc = [Infinity, Infinity];
      const hits = [null, null];
      for (let j = 0; j < b.wheels.length; j++) {
        const w = b.wheels[j];
        wc.set(b.x + cs * w.off, B.hardpointY - Lstart - B.wheelBelowPivot, b.z - sn * w.off).applyQuaternion(Q).add(sv.pos);
        const hit = world.castShape({ x: wc.x, y: wc.y, z: wc.z }, rot, { x: -Yb.x, y: -Yb.y, z: -Yb.z }, this.cyl, 0, maxCast, true, undefined, Q_WHEEL, undefined, this.body);
        // Step-climb limit: a wheel can roll up an edge at most ~0.85 r above the ground it is on.
        // Taller faces are walls (the leg and wheel-guard colliders stop the vehicle).
        w.blocked = false;
        if (hit && w.groundY != null && hit.witness1.y - w.groundY > ROVER.climbMax && hit.time_of_impact > 1e-4) { w.blocked = true; }
        else if (hit) { Lc[j] = Lstart + hit.time_of_impact + w.sink; hits[j] = hit; }
      }
      // walking beam equalisation (quasi-static beam angle)
      let s;
      if (b.wheels.length === 1) s = 0;
      else if (Lc[0] === Infinity && Lc[1] === Infinity) s = b.beamS * 0.9;
      else if (Lc[0] === Infinity) s = -smax; else if (Lc[1] === Infinity) s = smax;
      else s = Math.max(-smax, Math.min(smax, (Lc[1] - Lc[0]) / (2 * B.halfSpan)));
      b.beamS = s;

      let FtireY = 0;
      for (let j = 0; j < b.wheels.length; j++) {
        const w = b.wheels[j];
        const sign = j === 0 ? -1 : 1; // wheel 0 is the forward wheel: L0 = L - a s
        const Li = b.L + sign * B.halfSpan * s;
        const delta = Lc[j] === Infinity ? -1 : Li - Lc[j];
        const hit = hits[j];
        const ddot = (delta - w.deltaPrev) / dt;
        w.deltaPrev = delta;
        let N = 0;
        if (delta > 0 && hit) N = Math.max(0, Math.min(400000, Wp.tireK * delta + Wp.tireC * ddot));
        w.contact = N > 0;
        w.load = N;
        // ---- motor torque (PI speed loop) ---------------------------------------
        const failed = w.fault === 'motor';
        const seized = w.fault === 'seized';
        const tAvail = M.peakTorque * (this.limits.torquePct / 100) * Math.max(0, Math.min(1, (M.noLoadSpeed - Math.abs(w.omega)) / (M.noLoadSpeed * 0.25))) * (w.temp > 110 ? Math.max(0.2, 1 - (w.temp - 110) / 30) : 1);
        let Tm = 0;
        if (parkBrake || failed || seized) w.integ = 0;
        if (!w.contact) {
          if (!parkBrake && !failed && !seized) {
            const e = w.target - w.omega;
            w.integ = Math.max(-0.8, Math.min(0.8, w.integ + e * dt));
            Tm = Math.max(-tAvail, Math.min(tAvail, M.kp * e + M.ki * w.integ));
          }
          w.torque = Tm;
          // free spinning wheel
          w.airT = (w.airT || 0) + dt; if (w.airT > 0.6 && !w.blocked) w.groundY = null;
          w.omega += dt * Tm / Wp.inertia;
          if (parkBrake || seized) w.omega = 0;
          w.Fx = w.Fy = w.H = w.Rc = 0; w.slip = 0; w.sink *= 0.9;
          w.angle += w.omega * dt;
          driveW += this._motorPower(Tm, w.omega);
          continue;
        }
        // ---- contact frame -----------------------------------------------------
        const n = V().set(hit.normal1.x, hit.normal1.y, hit.normal1.z);
        if (hit.time_of_impact <= 1e-5 || n.lengthSq() < 0.5) n.copy(Yb);
        w.cp.set(hit.witness1.x, hit.witness1.y, hit.witness1.z);
        w.groundY = w.groundY == null ? w.cp.y : w.groundY + (w.cp.y - w.groundY) * Math.min(1, dt * 6);
        w.airT = 0;
        w.cn.copy(n);
        tl.copy(dirW).addScaledVector(n, -dirW.dot(n)).normalize();
        tlat.copy(n).cross(tl).normalize();
        rel.copy(w.cp).sub(sv.com);
        vel.copy(sv.ang).cross(rel).add(sv.lin);
        const vL = vel.dot(tl), vLat = vel.dot(tlat);
        const rr = Wp.radius;
        const D = 2 * rr * Wp.effDiameterFactor;
        const den = Math.max(Math.abs(rr * w.omega), Math.abs(vL), 0.06);
        const slipOf = (om) => Math.max(-1, Math.min(1, (rr * om - vL) / Math.max(Math.abs(rr * om), Math.abs(vL), 0.06)));
        const tanA = vLat / Math.max(Math.abs(vL), 0.12);
        // Fully implicit wheel spin: solve I*(om' - om)/dt = T(om') - H(om')*r by bisection.
        // f(om') is monotonic (thrust rises and motor torque falls with om'), so this is
        // unconditionally stable even when the soil thrust saturates (scrubbing turns, stalls).
        const I = Wp.inertia;
        // contact patch + shear capacity from this step's load and last step's slip (sinkage)
        const base = wheelSoil(N, Wp.width, D, w.slip, tanA);
        const kRatio = base.l / REGOLITH.K;
        const Hof = (om) => { const sl = slipOf(om); return Math.sign(sl) * base.Hmax * shearFactor(Math.abs(sl) * kRatio); };
        const piT = (om) => {
          if (parkBrake || failed || seized) return 0;
          const e = w.target - om;
          const T = M.kp * e + M.ki * Math.max(-0.8, Math.min(0.8, w.integ + e * dt));
          return Math.max(-tAvail, Math.min(tAvail, T));
        };
        const f = (om, Textra) => I * (om - w.omega) / dt - piT(om) - Textra + Hof(om) * rr;
        const bisect = (Textra) => {
          let lo = w.omega - 8, hi = w.omega + 8;
          for (let it = 0; it < 16; it++) { const mid = 0.5 * (lo + hi); if (f(mid, Textra) > 0) hi = mid; else lo = mid; }
          return 0.5 * (lo + hi);
        };
        let omNew;
        if (parkBrake || seized) {
          // can the spring brake hold the wheel at zero this step?
          const need = I * (0 - w.omega) / dt + Hof(0) * rr;
          if (seized || Math.abs(need) <= M.brakeTorque) omNew = 0;
          else omNew = bisect(-Math.sign(w.omega || need) * M.brakeTorque);
        } else omNew = bisect(0);
        if (!(parkBrake || failed || seized)) {
          const e = w.target - omNew;
          const Tun = M.kp * e + M.ki * (w.integ + e * dt);
          if (Math.abs(Tun) < tAvail) w.integ = Math.max(-0.8, Math.min(0.8, w.integ + e * dt));
          Tm = piT(omNew);
        } else Tm = 0;
        w.torque = Tm;
        w.omega = omNew;
        w.angle += w.omega * dt;
        const soil = wheelSoil(N, Wp.width, D, slipOf(w.omega), tanA);
        w.slip = slipOf(w.omega);
        w.alpha = Math.atan(tanA);
        w.sink += (soil.z - w.sink) * Math.min(1, dt * 8);
        w.H = soil.H; w.Rc = soil.Rc;
        let Fx = soil.H - soil.Rc * Math.sign(vL) * Math.min(1, Math.abs(vL) / 0.05);
        let Fy = soil.Fy;
        // friction-ellipse saturation
        const mag = Math.hypot(Fx, Fy), cap = soil.Hmax * 1.02;
        if (mag > cap) { Fx *= cap / mag; Fy *= cap / mag; }
        // dead-beat guards (stability at very low speed): forces may stop, never reverse, motion
        const latCap = mEff * Math.abs(vLat) / dt;
        if (Math.abs(Fy) > latCap) Fy = Math.sign(Fy) * latCap;
        if (Math.abs(w.omega) < 1e-6) {
          const lonCap = mEff * Math.abs(vL) / dt;
          if (Math.abs(Fx) > lonCap) Fx = Math.sign(Fx) * lonCap;
        }
        // Static friction of a braked wheel (bristle / Dahl model): the locked tyre is anchored
        // to the regolith and resists with a stiff spring up to the soil shear capacity, so a
        // parked rover holds against slopes and docking-stage reaction loads without creep.
        if (Math.abs(w.omega) < 1e-6 && (parkBrake || seized) && Math.hypot(vL, vLat) < 0.06) {
          if (!w.anchor) w.anchor = w.cp.clone();
          const dx = V().subVectors(w.cp, w.anchor);
          const dL = dx.dot(tl), dT = dx.dot(tlat);
          let bx = -300000 * dL - 22000 * vL, by = -300000 * dT - 22000 * vLat;
          const capS = soil.Hmax + soil.Rc, m2 = Math.hypot(bx, by);
          if (m2 > capS) { // sliding: drag the anchor along so the bristle force stays at the limit
            bx *= capS / m2; by *= capS / m2;
            w.anchor.copy(w.cp).addScaledVector(tl, bx / 300000).addScaledVector(tlat, by / 300000);
          }
          Fx = bx; Fy = by;
        } else w.anchor = null;
        w.Fx = Fx; w.Fy = Fy;
        F.copy(n).multiplyScalar(N).addScaledVector(tl, Fx).addScaledVector(tlat, Fy);
        const fY = F.dot(Yb);
        FtireY += fY;
        F.addScaledVector(Yb, -fY); // tangential part goes straight into the chassis through the leg
        this.body.applyImpulseAtPoint({ x: F.x * dt, y: F.y * dt, z: F.z * dt }, { x: w.cp.x, y: w.cp.y, z: w.cp.z }, true);
        driveW += this._motorPower(Tm, w.omega);
        w.power = this._motorPower(Tm, w.omega);
        // thermal
        const loss = (Tm / M.peakTorque) ** 2 * 380 + Math.abs(Tm * w.omega) * (1 - M.efficiency);
        w.temp += dt * (loss - (w.temp + 20) / M.thermalR) / M.thermalC;
        w.dragAccum += Math.abs(vL) * dt;
      }
      // ---- strut / unsprung mass ------------------------------------------------
      // active leveling sets the spring seat (free length) on each corner
      // air struts: the levelling valves raise pressure with load, so stiffness (and the damping that
      // keeps the same damping ratio) scale with the supported mass; plus skyhook body-rate damping
      // (body_control.js fast loop) on pitch and roll.
      let Fs = b._fs;
      if (b.L < B.Lmin) Fs += S.bumpK * (B.Lmin - b.L) - S.bumpC * Math.min(0, b.Ldot);
      if (b.L > B.Lmax) Fs -= S.bumpK * (b.L - B.Lmax) + S.bumpC * Math.max(0, b.Ldot);
      b.strutF = Fs;
      const gAlong = g * Yb.y;              // weight of unsprung mass along strut extension
      const Lddot = (Fs + mu * gAlong + mu * aAlong - FtireY) / mu;
      b.Ldot += Lddot * dt;
      b.L += b.Ldot * dt;
      // body: strut force at hardpoint + the part of unsprung weight the strut can't carry
      const gPerp = V().copy(gvec).addScaledVector(Yb, -gvec.dot(Yb)).multiplyScalar(mu);
      F.copy(Yb).multiplyScalar(Fs).add(gPerp);
      this.body.applyImpulseAtPoint({ x: F.x * dt, y: F.y * dt, z: F.z * dt }, { x: hp.x, y: hp.y, z: hp.z }, true);
    }

    this._leveling(dt, Q);
    this._smartRpm(dt);
    this._liftStep(dt, Q, sv.pos);
    this._powerStep(dt, driveW, Q, sv.pos);
    this._stability(Q, sv.com);
    // odometer (true)
    if (this._prevPos) this.odo.dist += Math.hypot(sv.pos.x - this._prevPos.x, sv.pos.z - this._prevPos.z);
    this._prevPos = { x: sv.pos.x, z: sv.pos.z };
  }

  _motorPower(T, om) {
    const p = T * om, eff = ROVER.motor.efficiency;
    const copper = (T / ROVER.motor.peakTorque) ** 2 * 380;
    return (p >= 0 ? p / eff : p * eff * 0.7) + copper;
  }

  _leveling(dt, Q) {
    const B = ROVER.bogie, S = ROVER.susp;
    const fwd = V().set(1, 0, 0).applyQuaternion(Q), right = V().set(0, 0, 1).applyQuaternion(Q);
    const pitch = Math.asin(Math.max(-1, Math.min(1, fwd.y)));
    const roll = Math.asin(Math.max(-1, Math.min(1, right.y)));
    this.tilt.pitch = pitch; this.tilt.roll = roll;
    if (!this.leveling || this.estop) return;
    const Lavg = this.bogies.reduce((a, b) => a + b.L, 0) / this.bogies.length;
    const Ltarget = B.Lnom + this.rideOffset;
    for (const b of this.bogies) {
      // height of this corner relative to the plane through the centre
      const h = b.z * Math.sin(roll); // pitch is passive (rocker-bogie); only roll is trimmed
      const Ldes = b.L - h * 0.85 + (Ltarget - Lavg);
      const err = Math.max(B.Lmin + 0.06, Math.min(B.Lmax - 0.06, Ldes)) - b.L;
      const rate = Math.max(-S.levelRate, Math.min(S.levelRate, err * 0.6));
      b.level = Math.max(-S.levelRange, Math.min(S.levelRange, b.level + rate * dt));
    }
  }

  _stability(Q, com) {
    // Static tip-over margin (force-angle style, gravity only): for each edge of the wheel
    // support polygon, the angle the CoM must rotate about that edge before it passes over it.
    // a bogie counts as support if a wheel touched within the last 0.3 s (walking-beam rock-over
    // briefly unloads a corner without the vehicle actually pivoting off it)
    // Rollover margin uses every corner bogie: tipping about a diagonal only drops the vehicle onto
    // its 4th corner wheel (not a rollover). An unloaded corner uses its wheel's lowest point.
    const pts = [];
    const Yb = V().set(0, 1, 0).applyQuaternion(Q);
    const t = this.body.translation();
    for (const b of this.bogies) {
      let any = null;
      for (const w of b.wheels) if (w.contact) any = w.cp;
      if (!any) {
        const B = ROVER.bogie;
        any = V().set(b.x, B.hardpointY - b.L - B.wheelBelowPivot - ROVER.wheel.radius, b.z).applyQuaternion(Q).add(new THREE.Vector3(t.x, t.y, t.z));
      }
      pts.push(any);
    }
    void Yb;
    const c = V().copy(com);
    if (this.loaded) {
      const ct = this.lift.cargo.body.translation(), m1 = this.mass, m2 = this.lift.cargo.T.mass;
      c.multiplyScalar(m1).add(V().set(ct.x, ct.y, ct.z).multiplyScalar(m2)).divideScalar(m1 + m2);
    }
    this.comWorld = c.clone();
    // support polygon = convex hull of the wheel contacts (horizontal plane)
    const poly = convexHullXZ(pts.filter(Boolean));
    if (poly.length < 3) { this.tilt.margin = 0; return; }
    let area = 0;
    for (let i = 0; i < poly.length; i++) { const a = poly[i], b = poly[(i + 1) % poly.length]; area += a.x * b.z - b.x * a.z; }
    const orient = Math.sign(area) || 1;
    let minAng = Infinity;
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      let ex = b.x - a.x, ez = b.z - a.z; const el = Math.hypot(ex, ez) || 1; ex /= el; ez /= el;
      const rx = c.x - a.x, rz = c.z - a.z;
      const inward = (ex * rz - ez * rx) * orient;          // horizontal distance inside this edge
      const tpar = Math.max(0, Math.min(el, rx * ex + rz * ez));
      const edgeY = a.y + (b.y - a.y) * (tpar / el);
      const h = Math.max(0.1, c.y - edgeY);
      minAng = Math.min(minAng, Math.atan2(inward, h));
    }
    this.tilt.margin = minAng * 180 / Math.PI;
    // filtered margin (tau ~0.8 s) for the speed governor; raw value for display/stats
    this.tilt.marginF = this.tilt.marginF == null ? this.tilt.margin : this.tilt.marginF + (this.tilt.margin - this.tilt.marginF) * Math.min(1, ROVER_DT / 0.8);
    // ignore the settling transient after spawn/teleport (wheels not yet on the ground)
    if (this.sim.time - (this._spawnT || 0) > 3 && this.wheels.filter((w) => w.contact).length >= 6) this.stats.minMargin = Math.min(this.stats.minMargin, this.tilt.margin);
  }

  // ---- cradle hoist ------------------------------------------------------------
  findCargoUnder() {
    const p = this.pose();
    let best = null, bestD = Infinity;
    for (const c of this.sim.cargo) {
      if (c.state === 'delivered-locked') continue;
      const t = c.body.translation();
      const rel = V().set(t.x - p.x, t.y - p.y, t.z - p.z).applyQuaternion(p.q.clone().invert());
      const d = Math.hypot(rel.x, rel.z);
      if (d < bestD) { bestD = d; best = { c, rel }; }
    }
    return best;
  }

  // Ring-clamp capture window (module_carry.js CRADLE at 1:16.3): the V saddles catch +-0.41 m sideways
  // and +-0.06 rad of yaw; the 0.82 m clamp band must be within +-0.3 m axially; and the module must be
  // SEATED on the saddles (the rover has risen under it), not still on its stand.
  grappleCheck() {
    const f = this.findCargoUnder();
    if (!f) return { ok: false, why: 'no cargo over the saddles' };
    const { c, rel } = f;
    const p = this.pose();
    const r = c.body.rotation();
    const cq = new THREE.Quaternion(r.x, r.y, r.z, r.w);
    const cf = V().set(1, 0, 0).applyQuaternion(cq);
    let yawErr = Math.atan2(-cf.z, cf.x) - p.yaw;
    yawErr = Math.atan2(Math.sin(yawErr), Math.cos(yawErr));
    const T = c.T, half = T.adapterR || T.radius;
    const dxc = rel.x - (T.carryX || 0);
    const seatErr = rel.y - (ROVER.seatY + half);
    const res = { c, dx: dxc, dz: rel.z, yawErr, seatErr, ok: false, why: '' };
    if (Math.abs(dxc) > 0.3) res.why = `clamp band ${dxc.toFixed(2)} m off the rings (limit 0.30 m)`;
    else if (Math.abs(rel.z) > 0.41) res.why = `lateral offset ${rel.z.toFixed(2)} m > V-saddle capture 0.41 m`;
    else if (Math.abs(yawErr) > 0.06) res.why = `yaw error ${(yawErr * 180 / Math.PI).toFixed(1)} deg > 3.4 deg`;
    else if (Math.abs(seatErr) > 0.2) res.why = seatErr > 0 ? 'module not seated: rise under it first' : 'module below the saddles';
    else res.ok = true;
    return res;
  }

  clampClose() {
    const L = this.lift;
    if (L.clamp !== 'open') return { ok: false, why: `ring clamps are ${L.clamp}` };
    const chk = this.grappleCheck();
    if (!chk.ok) { L.msg = 'CLAMP REFUSED: ' + chk.why; return chk; }
    L.pending = chk.c;
    L.clamp = 'closing';
    L.msg = `Ring jaws closing on ${chk.c.id}`;
    return { ok: true };
  }

  // supported = lugs resting on stand posts / receiving jacks, or hard-docked to a port
  cargoSupported() {
    const L = this.lift;
    if (!L.cargo) return true;
    if (L.cargo.dockJoint) return true;
    const { world } = this.sim;
    const t = L.cargo.body.translation(), r = L.cargo.body.rotation();
    const q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
    const T = L.cargo.T;
    let ok = 0;
    for (const lx of T.lugXs) for (const sz of [-1, 1]) {
      const p = V().set(lx, 0, sz * T.lugZ).applyQuaternion(q).add(new THREE.Vector3(t.x, t.y, t.z));
      const ray = new this.RAPIER.Ray({ x: p.x, y: p.y, z: p.z }, { x: 0, y: -1, z: 0 });
      const hit = world.castRay(ray, 2.0, true, undefined, Q_TERRAIN);
      if (hit && hit.timeOfImpact < T.lugR + 0.15) ok++;
    }
    return ok >= 4;
  }

  clampOpen(force = false) {
    const L = this.lift;
    if (L.clamp === 'open' || L.clamp === 'opening') return { ok: true };
    const v = this.body.linvel();
    if (L.cargo && !force && Math.hypot(v.x, v.z) > 0.05 && !this.cargoSupported()) { L.msg = 'RELEASE INHIBITED: vehicle moving with an unsupported load'; return { ok: false, why: L.msg }; }
    this._detach();
    L.clamp = 'opening';
    L.msg = 'Ring jaws opening (the saddles still carry the module until the rover kneels)';
    return { ok: true };
  }

  _attach(c) {
    const { RAPIER, world } = this.sim;
    const p = this.pose();
    const t = c.body.translation(), r = c.body.rotation();
    const qinv = p.q.clone().invert();
    const rel = V().set(t.x - p.x, t.y - p.y, t.z - p.z).applyQuaternion(qinv);
    const qrel = qinv.clone().multiply(new THREE.Quaternion(r.x, r.y, r.z, r.w));
    const jd = RAPIER.JointData.fixed({ x: rel.x, y: rel.y, z: rel.z }, { x: qrel.x, y: qrel.y, z: qrel.z, w: qrel.w }, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0, w: 1 });
    const j = world.createImpulseJoint(jd, this.body, c.body, true);
    j.setContactsEnabled(false);
    for (const cc of c.colliders || [c.collider]) cc.setCollisionGroups(G_CARGO_HELD);
    this.lift.joint = j; this.lift.cargo = c; this.lift.base = rel.clone();
    c.state = 'held'; // ride set point is kept: the struts hold the module at carry height
  }

  _detach() {
    const L = this.lift;
    if (L.joint) { this.sim.world.removeImpulseJoint(L.joint, true); }
    if (L.cargo) { for (const cc of L.cargo.colliders || [L.cargo.collider]) cc.setCollisionGroups(G_CARGO); if (L.cargo.state === 'held') L.cargo.state = 'placed'; }
    L.joint = null; L.cargo = null; L.base = null;
  }

  // ring jaws + ride-height control (kneel / ride / carry). L.target is the ride set point (m),
  // L.pos the actual ride offset; the levelling struts follow it (rate-limited high-flow valves).
  _liftStep(dt, Q, pos) {
    const L = this.lift, LC = ROVER.lift;
    if (L.clamp === 'closing') {
      L.clampT = Math.min(1, L.clampT + dt / LC.jawS);
      if (L.clampT >= 1) {
        const chk = this.grappleCheck();
        if (chk.ok && chk.c === L.pending) { this._attach(L.pending); L.clamp = 'closed'; L.msg = `Ring clamps locked on ${L.pending.id} (worm drives self-locking, 0 W hold)`; }
        else { L.clamp = 'opening'; L.msg = 'CLAMP FAULT: module moved during closure, reopening'; }
        L.pending = null;
      }
    } else if (L.clamp === 'opening') {
      L.clampT = Math.max(0, L.clampT - dt / LC.jawS);
      if (L.clampT <= 0) { L.clamp = 'open'; L.msg = 'Ring jaws open'; }
    }
    if (this.estop) return; // air valves closed: ride held
    L.target = Math.max(ROVER.ride.kneel, Math.min(ROVER.ride.max, L.target));
    const prev = this.rideOffset;
    this.rideOffset += Math.max(-LC.rate * dt, Math.min(LC.rate * dt, L.target - this.rideOffset));
    L.pos = this.rideOffset;
    L.rateNow = (this.rideOffset - prev) / dt;
  }

  // ---- power ---------------------------------------------------------------------
  _powerStep(dt, driveW, Q, pos) {
    const P = this.power, BAT = ROVER.battery, SOL = ROVER.solar;
    const sun = this.sim.sunDir;
    // mast tracks the sun azimuth
    const fwd = V().set(1, 0, 0).applyQuaternion(Q);
    const bodyYaw = Math.atan2(-fwd.z, fwd.x);
    const sunYaw = Math.atan2(-sun.z, sun.x);
    let want = sunYaw - bodyYaw; want = Math.atan2(Math.sin(want), Math.cos(want));
    let d = want - P.mastYaw; d = Math.atan2(Math.sin(d), Math.cos(d));
    if (!this.estop) P.mastYaw += Math.max(-0.09 * dt, Math.min(0.09 * dt, d));
    const nrm = V().set(Math.cos(P.mastYaw), 0, -Math.sin(P.mastYaw)).applyQuaternion(Q);
    this._shadowT -= dt;
    if (this._shadowT <= 0) {
      this._shadowT = 1.0;
      const o = V().set(-1.2, 5.4, -2.75).applyQuaternion(Q).add(pos).addScaledVector(sun, 3);
      const ray = new this.RAPIER.Ray({ x: o.x, y: o.y, z: o.z }, { x: sun.x, y: sun.y, z: sun.z });
      const hit = this.sim.world.castRay(ray, 3000, true, undefined, Q_TERRAIN);
      P.sunlit = hit ? 0 : 1;
    }
    const cosI = Math.max(0, nrm.dot(sun));
    P.solarW = SOL.panels * SOL.panelArea * SOL.efficiency * 1361 * cosI * P.sunlit * (sun.y > -0.01 ? 1 : 0);
    let hotel = ROVER.avionicsW + ROVER.heaterW + ROVER.lidarW + ROVER.cameraW + ROVER.commsW + (this.lights ? 180 : 0);
    if (!this.powered) hotel = 60;
    const lifting = (this.mass + (this.lift.cargo ? this.lift.cargo.T.mass : 0)) * MOON.g * Math.max(0, this.lift.rateNow || 0) / 0.25; // air pump, eta 0.25 (suspension.js)
    const clampW = (this.lift.clamp === 'closing' || this.lift.clamp === 'opening') ? 450 : 0;
    const steerW = this.bogies.reduce((a, b) => a + (Math.abs(b.steerTarget - b.steer) > 0.002 ? 120 : 8), 0);
    P.driveW = driveW;
    P.loadW = hotel + driveW + lifting + clampW + steerW;
    P.netW = P.solarW - P.loadW;
    const dWh = P.netW * dt / 3600;
    P.wh = Math.max(0, Math.min(BAT.capacityWh, P.wh + dWh));
    if (P.netW < 0) P.energyUsedWh += -dWh; else P.energyGenWh += dWh;
    if (driveW < 0) P.regenWh += -driveW * dt / 3600;
    P.soc = P.wh / BAT.capacityWh;
    P.voltage = BAT.nominalV * (0.9 + 0.15 * P.soc) - (P.loadW - P.solarW) / BAT.nominalV * BAT.internalR;
    if (P.soc <= 0.005 && this.powered) { this.powered = false; this.sim.log('BATTERY DEPLETED - safe mode', 'err'); }
  }

  telemetry() {
    const p = this.pose();
    const v = this.body.linvel();
    const fwdSpeed = v.x * p.fwd.x + v.y * p.fwd.y + v.z * p.fwd.z;
    return {
      pose: p, speed: Math.hypot(v.x, v.y, v.z), fwdSpeed,
      roll: this.tilt.roll, pitch: this.tilt.pitch, margin: this.tilt.margin,
      wheels: this.wheels, bogies: this.bogies, power: this.power, lift: this.lift,
    };
  }

  teleport(x, z, yaw) {
    const gy = heightAt(this.sim.terrain, x, z);
    this._detach();
    this.body.setTranslation({ x, y: gy + ROVER.rideHeight + 0.3, z }, true);
    this.body.setRotation(yawQuat(yaw), true);
    this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    for (const b of this.bogies) { b.L = ROVER.bogie.Lnom; b.Ldot = 0; b.hpVelPrev = null; for (const w of b.wheels) { w.omega = 0; w.integ = 0; w.groundY = null; w.anchor = null; } }
    this._prevPos = null;
    this._spawnT = this.sim.time;
  }
}

function convexHullXZ(pts) {
  const P = pts.slice().sort((a, b) => a.x - b.x || a.z - b.z);
  if (P.length < 3) return P;
  const cr = (o, a, b) => (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
  const lo = [], up = [];
  for (const p of P) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = P.length - 1; i >= 0; i--) { const p = P[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  up.pop(); lo.pop();
  return lo.concat(up);
}
