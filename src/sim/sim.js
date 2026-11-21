// Simulation orchestrator: fixed-step physics, sensors, SLAM, autonomy, safety, faults.
import * as THREE from 'three';
import { PHYS, WORLD, ROVER, DOCK } from './config.js';
import { buildWorld, yawQuat, G_STRUCT } from './world.js';
import { Rover } from './rover.js';
import { Sensors } from './sensors.js';
import { Slam } from './slam.js';
import { Autonomy, dockAngles, DOCK_CAPTURE_MAX_ANGLE } from './autonomy.js';
import { heightAt } from './terrain.js';
import { Scout } from './scout.js';
import { Telemetry } from './telemetry.js';
import { PowerDock } from './powerDock.js';
import { Health } from './health.js';
import { Thermal } from './thermal.js';
import { AcmHandler } from './acmCarry.js';

// Dust-exclusion bellows contact points (docs/changes/C.md): the ring touches the port at BELLOWS_POINTS points spread round
// its rim, each carrying 1/N of DOCK.bellows k and c, so a module that is not square gets a restoring moment (the single
// centre point gave none). Rim radius: inside the port opening and the module end face.
const BELLOWS_POINTS = 4;
// Quiescent-hold step (s): while the autonomy holds the docked, braked rover for the vestibule leak check, stepOnce
// advances this much sim time per call without the physics solver (same size as the rails step warp.H).
const HOLD_STEP_S = 0.5;
// Transit-warp rails step: a fixed size per time scale, whole physics steps, about one rendered frame (1/60 s of wall time)
// of sim time, between PHYS.dt and warp.H. It used to follow the wall-clock backlog owed by the frame.
const RAILS_FRAME_S = 1 / 60;

export class Sim {
  constructor(RAPIER, opts = {}) {
    this.RAPIER = RAPIER;
    const w = buildWorld(RAPIER, opts.seed ?? WORLD.seed);
    Object.assign(this, w);
    this.time = 0;
    this.timeScale = 1;
    // (v1.7.0: minScale is PHYS.railsMinScale = 5, was 50. Full physics tops out near 3x real time on the team Mac mini, so 5x, 10x and
    // 20x could never be reached by stepping the physics faster; the cruise legs now use the same rails the 50x mode always used.)
    // Transit warp: at >= minScale the long autonomous cruise legs run kinematically (Rover.railsPlace) in H-second
    // steps; pickup, docking, charging, pivots, recoveries and anything the rails check refuses stay on full physics.
    this.warp = { enabled: opts.warp ?? true, minScale: PHYS.railsMinScale, H: 0.5, active: false, steps: 0, simS: 0, lastScanD: 0, lastStreamD: 0 };
    this.warping = false;   // true only inside warpStep, while the autonomy tick runs
    this.actualRate = 0;    // achieved sim seconds per wall second (EMA), for the HUD
    this.paused = false;
    this.logs = [];
    this.listeners = {};
    this.sunAz = opts.sunAz ?? 205;   // deg, from +X toward -Z (THREE yaw convention)
    this.sunEl = opts.sunEl ?? 9;
    this.sunDir = new THREE.Vector3();
    this.setSun(this.sunAz, this.sunEl);
    this.landerPos = { x: WORLD.pad.x, z: WORLD.pad.z };
    // Operator link. delayS = one-way signal delay: 0 for crew in the habitat 2 km away (default), ~1.3 s light time
    // plus ground-segment latency for an operator on Earth ("comms earth"). Commands and manual drive inputs reach the
    // rover delayS late; an Earth operator cannot joystick a moving vehicle, so manual drive is speed-capped there.
    this.comms = { delayS: 0, mode: 'local', queue: [], earthDelayS: 2.6, earthDriveCap: 0.3 };
    this.rover = new Rover(this, this.terrain.pad.roverStart);
    this.cargoForces = new Map();
    this.colliderOwner = new Map();
    for (const c of this.cargo) for (const cc of c.colliders || [c.collider]) this.colliderOwner.set(cc.handle, { kind: 'cargo', ref: c });
    for (const p of this.ports) this.colliderOwner.set(p.collider.handle, { kind: 'port', ref: p });
    this._streamT = 0;
    this.streamRocks([this.rover.pose(), ...this.cargo.map((c) => c.body.translation())]);
    this.modules = [];
    this.stepCount = 0;
    this.sensors = new Sensors(this);
    this.slam = new Slam(this, this.sensors);
    this.autonomy = new Autonomy(this, this.sensors, this.slam);
    if (opts.leakCheckS != null) this.autonomy.leakCheckS = opts.leakCheckS;
    // quiescent hold (berthing leak check): set by the autonomy to { until, gen }; Sim._holdStep runs it without physics
    this.quiescentHold = null;
    this.holdWarp = { enabled: opts.holdWarp ?? true, steps: 0, simS: 0 };
    this.addModule(this.sensors);
    this.addModule(this.slam);
    this.addModule(this.autonomy);
    // arm-handled cargo (ACM): states GROUND / CARRIED / DECK, soil feet, hook coupling (acmCarry.js). Spawned after the autonomy and
    // sensors so the straddle-cargo manifest and fiducial list stay as they were.
    this.acm = new AcmHandler(this);
    // no ACM is spawned by default (many tests and the logistics planner assume exactly the three straddle cargos in sim.cargo);
    // `acm demo` / `acm spawn [kg]` or a scenario adds them
    this.scout = new Scout(this);
    this.addModule(this.scout);
    this.powerDock = new PowerDock(this);
    this.addModule(this.powerDock);
    this.health = new Health(this); // FDIR, random failures, thermal, survival mode, keep-alive array
    this.addModule(this.health);
    // thermal environment, instrument nodes, random thermal scenarios (off headless unless asked) and the thermal brain
    this.thermal = new Thermal(this, { scenarios: opts.thermalScenarios ?? false });
    this.addModule(this.thermal);
    this.lunar = { on: false, periodS: 900, phase: 0.25, _t: 0 }; // demo lunar day/night cycle (off = fixed sun)
    this.telemetry = new Telemetry(this);
    this.addModule(this.telemetry);
    this.startup = null;
    this.log(`${ROVER.name} online at landing pad. Route pad -> habitat: ${Math.hypot(WORLD.habitat.x - WORLD.pad.x, WORLD.habitat.z - WORLD.pad.z).toFixed(0)} m`, 'ok');
  }

  setSun(azDeg, elDeg) {
    if (!this.lunar?.on) this.sunAz0 = azDeg;
    this.sunAz = azDeg; this.sunEl = elDeg;
    const a = azDeg * Math.PI / 180, e = elDeg * Math.PI / 180;
    this.sunDir.set(Math.cos(e) * Math.cos(a), Math.sin(e), -Math.cos(e) * Math.sin(a)).normalize();
    this.emit('sun');
  }

  on(ev, fn) { (this.listeners[ev] ||= []).push(fn); }
  emit(ev, data) { for (const fn of this.listeners[ev] || []) fn(data); }
  log(msg, level = 'info') {
    const e = { t: this.time, msg, level };
    this.logs.push(e); if (this.logs.length > 500) this.logs.shift();
    this.emit('log', e);
  }

  addModule(m) { this.modules.push(m); m._acc = 0; }

  // ---------------- safety ----------------
  emergencyStop(source = 'operator') {
    const R = this.rover;
    if (R.estop) return;
    const lv = R.body.linvel();
    const moving = Math.hypot(lv.x, lv.y, lv.z) > 0.04;
    R.estop = true;
    R.cmd.vx = R.cmd.vz = R.cmd.wz = 0;
    // Moving E-STOP enters Cat-1 tip-safe regenerative deceleration first (rover.js sets brake & powered=false once stopped).
    if (!moving) { R.brake = true; R.powered = false; } else { R.brake = false; }
    R.tilt.maxDeg = Math.max(R.tilt.maxDeg, Math.abs(R.tilt.roll * 180 / Math.PI), Math.abs(R.tilt.pitch * 180 / Math.PI));
    this.autonomy.paused = true; this.autonomy.mode = 'MANUAL';
    this.startup = null;
    if (this.scout && this.scout.enabled) { this.scout.enabled = false; this.scout.v = 0; this.scout.msg = 'halted by E-STOP (redeploy after restart)'; }
    this.log(`EMERGENCY STOP (${source}): drive torque removed, spring brakes set, actuators locked, SCOUT-1 halted. Reset + START required.`, 'err');
    this.emit('estop', true);
  }
  resetEstop() {
    const R = this.rover;
    if (!R.estop) { this.log('E-STOP is not latched', 'info'); return; }
    if (this.health.fx.estopLoop) { this.log('RESET refused: E-STOP loop is open circuit (repair it first: "repair estop")', 'err'); return; }
    R.estop = false; R.powered = false; R.brake = true;
    this.log('E-STOP released. Drive system still DISARMED: press START to run power-on self-test.', 'warn');
    this.emit('estop', false);
  }
  startUp() {
    const R = this.rover;
    if (R.estop) { this.log('START refused: E-STOP still latched (reset it first)', 'err'); return; }
    if (R.powered) { this.log('Drive system already armed', 'info'); return; }
    if (R.power.soc < 0.01) { this.log('START refused: battery depleted', 'err'); return; }
    const noArm = this.health.canArm();
    if (noArm) { this.log(`START refused: ${noArm} (see "health")`, 'err'); return; }
    if (this.health.survival.on) this.health.survival.on = false;
    this.startup = { t0: this.time, dur: 4 };
    this.log('Power-on self-test: motor controllers, brakes, steering, IMU, LiDAR, camera...', 'info');
  }
  _startupTick() {
    const s = this.startup; if (!s) return;
    if (this.time - s.t0 >= s.dur) {
      this.startup = null;
      const R = this.rover;
      R.powered = true; R.brake = true;
      for (const w of R.wheels) { w.integ = 0; }
      const faults = R.wheels.filter((w) => w.fault).length;
      this.log(`Self-test complete${faults ? `: ${faults} wheel fault(s) carried` : ''}. Drive ARMED, mode MANUAL (autonomy must be re-commanded).`, 'ok');
      this.emit('estop', false);
    }
  }

  // ---------------- float32 precision guard ----------------
  // Rapier stores positions in float32: ~800 m from the origin one ulp is 61 um, so a body creeping at
  // < 3 cm/s moves less than half an ulp per 1/120 s step and is rounded back to where it was (the
  // rover could not creep under a module or into a dock). Keep a float64 shadow position for the
  // bodies that matter and integrate it ourselves; contacts/joints that move a body more than a few
  // mm in a step reset the shadow to the solver's answer.
  _precisionPre() {
    if (!this._shadow) this._shadow = new Map();
    const bodies = [this.rover.body, ...this.cargo.map((c) => c.body)].filter((b) => b.isDynamic());
    for (const b of bodies) {
      const t = b.translation();
      let s = this._shadow.get(b);
      if (!s || Math.abs(s.x - t.x) > 1e-3 || Math.abs(s.y - t.y) > 1e-3 || Math.abs(s.z - t.z) > 1e-3) { s = { x: t.x, y: t.y, z: t.z }; this._shadow.set(b, s); }
      s.px = t.x; s.py = t.y; s.pz = t.z;
    }
  }
  _precisionPost(dt) {
    for (const [b, s] of this._shadow) {
      if (!b.isDynamic()) continue;   // kinematic ACM (carried / on deck) is placed by acmCarry.js
      const t = b.translation(), vc = b.linvel(), w = b.angvel(), c = b.worldCom();
      // velocity of the body ORIGIN (translation() is the origin, linvel() the centre of mass 2.25 m above it): v + w x r
      const rx = t.x - c.x, ry = t.y - c.y, rz = t.z - c.z;
      const v = { x: vc.x + w.y * rz - w.z * ry, y: vc.y + w.z * rx - w.x * rz, z: vc.z + w.x * ry - w.y * rx };
      // solver displacement beyond the plain v*dt (contact push-out) is taken as-is
      const ex = (t.x - s.px) - v.x * dt, ey = (t.y - s.py) - v.y * dt, ez = (t.z - s.pz) - v.z * dt;
      if (Math.abs(ex) > 2e-3 || Math.abs(ey) > 2e-3 || Math.abs(ez) > 2e-3) { s.x = t.x; s.y = t.y; s.z = t.z; continue; }
      s.x += v.x * dt; s.y += v.y * dt; s.z += v.z * dt;
      // Correct only up to PHYS.posGuardMaxSpeed. What is being corrected (measured, tests/scratch/dbg_quant.mjs): the solver moves
      // a body in steps of 2^-11 m = 0.49 mm per axis, 8 float32 ulps at these coordinates, so each step's displacement is v dt
      // rounded to that grid: up to 0.03 m/s of error per axis at ANY speed. At 0.12 m/s commanded straight the rover travelled
      // 6 deg to the left of its heading at 0.131 m/s; -4 deg at 0.3 m/s; under 1 deg above ~2.5 m/s.
      // History: the limit was 0.1 m/s ("above it one ulp is < 1.5 % of a step and unbiased": true of one ulp, but the grid is 8).
      // It was lowered to that after re-setting the position every step at driving speed left the rover's broadphase proxy stale and
      // the chassis drove through a 1.4 m boulder (tests/rock_climb.mjs). With the fresh-rock refresh in stepOnce that no longer
      // happens: tests/scratch/dbg_rock_shadow.mjs blocks at 0.3 and 0.6 m/s with the guard always on, and rock_climb runs at
      // 0.6 m/s, inside the new limit. Above the limit the rounding is left alone (it is small against the motion there).
      if (Math.hypot(vc.x, vc.z) > PHYS.posGuardMaxSpeed) { s.x = t.x; s.y = t.y; s.z = t.z; continue; }
      if (s.x !== t.x || s.y !== t.y || s.z !== t.z) b.setTranslation({ x: s.x, y: s.y, z: s.z }, false);
    }
  }

  // ---------------- docking hardware ----------------
  // Returns { ok, why }. The latch hardware has the same angular envelope as the autonomy's capture check: outside
  // DOCK_CAPTURE_MAX_ANGLE in yaw or pitch the ring cannot seat (a 1 deg yaw is 17 mm at a 1 m rim), so nothing is welded.
  dockCapture(c, port) {
    const an = dockAngles(c, port);
    if (Math.abs(an.yaw) >= DOCK_CAPTURE_MAX_ANGLE || Math.abs(an.pitch) >= DOCK_CAPTURE_MAX_ANGLE) {
      const why = `module not square to ${port.name}: yaw ${(an.yaw * 180 / Math.PI).toFixed(2)} deg, pitch ${(an.pitch * 180 / Math.PI).toFixed(2)} deg (limit ${(DOCK_CAPTURE_MAX_ANGLE * 180 / Math.PI).toFixed(2)})`;
      return { ok: false, why, yaw: an.yaw, pitch: an.pitch };
    }
    const t = c.body.translation(), r = c.body.rotation();
    const jd = this.RAPIER.JointData.fixed({ x: t.x, y: t.y, z: t.z }, r, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0, w: 1 });
    c.dockJoint = this.world.createImpulseJoint(jd, this.fixed, c.body, true);
    c.dockedPort = port; port.occupied = c;
    c.state = 'docked';
    return { ok: true, yaw: an.yaw, pitch: an.pitch };
  }
  // receiving stand at the port: four posts rise from the apron under the lug pins (same stand as the pad)
  extendJacks(port, k) {
    port.jackK = k;
    const c = port.occupied;
    if (k >= 0.999 && c && !port.jacks) {
      const T = c.T;
      const t = c.body.translation(), r = c.body.rotation();
      const q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
      const yaw = Math.atan2(-new THREE.Vector3(1, 0, 0).applyQuaternion(q).z, new THREE.Vector3(1, 0, 0).applyQuaternion(q).x);
      port.jacks = [];
      for (const lx of T.lugXs) for (const sz of [-1, 1]) {
        const p = new THREE.Vector3(lx, -T.lugR, sz * T.lugZ).applyQuaternion(q).add(new THREE.Vector3(t.x, t.y, t.z));
        const g0 = heightAt(this.terrain, p.x, p.z), hgt = p.y - g0;
        const d = this.RAPIER.ColliderDesc.cuboid(0.25, hgt / 2, 0.25).setTranslation(p.x, g0 + hgt / 2, p.z).setRotation(yawQuat(yaw)).setCollisionGroups(G_STRUCT);
        port.jacks.push({ x: p.x, z: p.z, h: hgt, g0, yaw, collider: this.world.createCollider(d, this.fixed) });
      }
      this.emit('jacks', port);
    }
  }
  openHatch(port) { port.hatchOpen = true; if (port.occupied) port.occupied.hatchOpen = true; this.emit('hatch', port); }

  // ---------------- fault injection (Mission 10) ----------------
  addBoulder(x, z, d = 3.0) {
    const r = { x, z, d, shape: 2, sy: 0.75, sx: 1.0, sz: 1.1, yaw: 0.7, tilt: 0.05, bury: 0.15, y: heightAt(this.terrain, x, z), injected: true };
    const i = this.addRock(r);
    this.streamRocks([this.rover.pose()]);
    this.emit('rocks', i);
    return i;
  }
  injectFault(kind, arg) {
    const R = this.rover, A = this.autonomy;
    switch (kind) {
      case 'block': {
        // drop a boulder field on the active path ~45 m ahead (or straight ahead)
        let p = null;
        if (A.path && A.path.length) {
          // path is in the SLAM frame; convert to world with the current estimate offset
          const i = Math.min(A.path.length - 1, A.pathIdx + 45), tp = R.pose(), e = this.slam.est;
          p = { x: A.path[i].x + (tp.x - e.x), z: A.path[i].z + (tp.z - e.z) };
        }
        if (!p) { const q = R.pose(); p = { x: q.x + q.fwd.x * 45, z: q.z + q.fwd.z * 45 }; }
        for (const [ox, oz, d] of [[0, 0, 3.4], [3.5, 2, 2.6], [-3.2, -1.5, 2.8], [1, -4, 2.2], [-1.5, 4.5, 2.4]]) this.addBoulder(p.x + ox, p.z + oz, d);
        this.log(`FAULT: route blocked by boulder field at (${p.x.toFixed(0)}, ${p.z.toFixed(0)})`, 'err');
        break;
      }
      case 'lidar': this.sensors.faults.lidar = true; this.log('FAULT: LiDAR returns lost (sensor failure)', 'err'); break;
      case 'camera': this.sensors.faults.camera = true; this.log('FAULT: camera / fiducial pipeline failed', 'err'); break;
      case 'comms': A.faults.comms = true; this.log('FAULT: comms loss with base. Rover continues autonomously; operator commands queued (E-STOP link independent).', 'err'); break;
      case 'battery': { const s = Math.max(0.01, Math.min(1, (arg ?? 15) / 100)); R.power.wh = s * ROVER.battery.capacityWh * (R.power.capK ?? 1); R.power.soc = s; /* s is the SoC of the strings on line */ this.log(`FAULT: battery state of charge forced to ${(s * 100).toFixed(0)}%`, 'err'); break; }
      case 'wheel': { const N = R.wheels.length, a = Math.round(Number(arg ?? 1)), n = Math.max(1, Math.min(N, Number.isFinite(a) ? a : 1)); R.wheels[n - 1].fault = 'motor'; this.log(`FAULT: wheel ${n} motor controller failed (wheel free-rolls). Torque redistributed to ${R.wheels.filter((w) => w.driven && w.fault !== 'motor' && w.fault !== 'seized').length} working drive wheels, speed derated 25% (autonomy.js).`, 'err'); break; }
      case 'shift': {
        if (!R.loaded) { this.log('No cargo to shift', 'warn'); return; }
        R.lift.base.z += 0.11; R.lift.shifted = true;
        this.log('FAULT: cargo shifted 11 cm in cradle (detected by cradle load-cell asymmetry)', 'err');
        break;
      }
      case 'seal': A.faults.seal = arg === 'hard' ? 'hard' : true; this.log(`FAULT: port seal fouled with regolith${arg === 'hard' ? ' (damaged: cleaning will not clear it)' : ''}`, 'err'); break;
      case 'port': { const n = arg ?? 1; A.faults.port[n] = true; this.log(`FAULT: PORT-${n} receiving location unavailable`, 'err'); break; }
      default:
        // the FDIR catalogue (src/sim/health.js): mc, seize, cool, enc, strut, bat, runaway, pdu, imd, fc, seu,
        // imu, sun, ldust, cdust, com, htr, louver, clamp, lcell, estop, coupler
        if (!this.health.inject(kind, arg)) { this.log(`Unknown fault "${kind}" ("fault list" shows the catalogue)`, 'warn'); return; }
        return;
    }
    this.emit('fault', kind);
  }
  clearFaults() {
    this.sensors.faults.lidar = this.sensors.faults.camera = false;
    this.autonomy.faults.comms = false; this.autonomy.faults.port = {}; this.autonomy.faults.seal = false;
    for (const w of this.rover.wheels) w.fault = null;
    this.health.clearAll();
    this.log('All faults cleared', 'ok');
    this.emit('fault', 'clear');
  }

  // True while the autonomy's quiescent hold applies: its own mission, rover braked and still, module latched.
  _holdActive() {
    const H = this.quiescentHold, R = this.rover;
    if (!H || !this.holdWarp.enabled) return false;
    if (H.gen !== this.autonomy.gen || this.time >= H.until - 1e-9) { this.quiescentHold = null; return false; }
    if (!R.brake || R.estop || this.autonomy.paused) return false;
    const v = R.body.linvel();
    return Math.hypot(v.x, v.y, v.z) < 0.01;
  }
  // One quiescent-hold step: everything but the rigid-body solver (rover idle power, FDIR, thermal, sensors, autonomy).
  _holdStep() {
    const H = this.quiescentHold, dt = PHYS.dt;
    const h = Math.max(dt, Math.min(HOLD_STEP_S, Math.floor((H.until - this.time) / dt + 1e-6) * dt));
    this._startupTick();
    if (this.comms.queue.length) { const q = this.comms.queue; while (q.length && q[0].t <= this.time) q.shift().fn(); }
    for (const m of this.modules) { m._acc += h; if (m._acc >= 1 / (m.rate || 120) - 1e-9) { m.update(m._acc); m._acc = 0; } }
    this._stepShiftRecovery(h);
    if (this.lunar.on) { const Ln = this.lunar; Ln.phase = (Ln.phase + h / Ln.periodS) % 1; this.setSun((this.sunAz0 ?? 205) + Ln.phase * 360, 10 * Math.sin(Ln.phase * 2 * Math.PI) - 1); }
    const p = this.rover.pose(); this.rover._powerStep(h, 0, p.q, p);
    this.cargoForces.clear();
    this.time += h; this.stepCount++; this.holdWarp.steps++; this.holdWarp.simS += h;
    return h;
  }

  // one fixed physics step (PHYS.dt), or one quiescent-hold step; returns the sim seconds advanced
  stepOnce(opts = {}) {
    if (this._holdActive()) return this._holdStep();
    const dt = PHYS.dt;
    this._startupTick();
    // operator link: release uplinked commands whose one-way light time has elapsed (commands.js, "comms")
    if (this.comms.queue.length) { const q = this.comms.queue; while (q.length && q[0].t <= this.time) q.shift().fn(); }
    for (const m of this.modules) {
      if (opts.skipAutonomy && m === this.autonomy) { m._acc = 0; continue; }
      m._acc += dt;
      const period = 1 / (m.rate || 120);
      if (m._acc >= period - 1e-9) { m.update(m._acc); m._acc = 0; }
    }
    this._stepShiftRecovery(dt);
    if (this.lunar.on) {
      const Ln = this.lunar; Ln.phase = (Ln.phase + dt / Ln.periodS) % 1; Ln._t -= dt;
      if (Ln._t <= 0) { Ln._t = 1; this.setSun((this.sunAz0 ?? 205) + Ln.phase * 360, 10 * Math.sin(Ln.phase * 2 * Math.PI) - 1); }
    }
    this._streamT -= dt;
    if (this._streamT <= 0) { this._streamT = 1.0; this.streamRocks([this.rover.pose()]); }
    this.rover.step(dt);
    this.acm.step(dt);
    this._precisionPre();
    this._bellowsStep(dt);
    this.world.step(this.eventQueue);
    if (this.freshRocks.length) { for (const c of this.freshRocks.splice(0)) if (c.isEnabled()) c.setTranslation(c.translation()); }
    this._precisionPost(dt);
    this.cargoForces.clear();
    this.eventQueue.drainContactForceEvents((e) => {
      const a = this.colliderOwner.get(e.collider1()), b = this.colliderOwner.get(e.collider2());
      const f = e.totalForceMagnitude();
      for (const [x, y] of [[a, b], [b, a]]) {
        if (x && x.kind === 'cargo') {
          const rec = this.cargoForces.get(x.ref) || { total: 0, port: 0, portRef: null };
          rec.total += f;
          if (y && y.kind === 'port') { rec.port += f; rec.portRef = y.ref; }
          this.cargoForces.set(x.ref, rec);
        }
      }
    });
    if (this._bellowsF) { // the coupling's push is a port contact force like the collar's
      const { c, port, F } = this._bellowsF, rec = this.cargoForces.get(c) || { total: 0, port: 0, portRef: null };
      rec.total += F; rec.port += F; rec.portRef = rec.portRef || port; this.cargoForces.set(c, rec);
    }
    this.time += dt;
    this.stepCount++;
    return dt;
  }

  // Port dust-exclusion coupling (DOCK.bellows): spring-damper between the held cargo's forward face and the port
  // plane, acting along the port normal while the face is within the bellows' free length and over the opening.
  // The ring meets the port at BELLOWS_POINTS rim points (top, sides, bottom), each a spring-damper of k/N, c/N on its own
  // gap and normal velocity: equal compression gives the old centre force, a tilted face gets a restoring moment.
  _bellowsStep(dt) {
    this._bellowsF = null;
    const B = DOCK.bellows, c = this.rover.lift.cargo;
    if (!B || !c || c.dockJoint) return;
    const t = c.body.translation(), r = c.body.rotation(), q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
    const f = new THREE.Vector3(c.T.length / 2, 0, 0).applyQuaternion(q).add(new THREE.Vector3(t.x, t.y, t.z));
    for (const p of this.ports) {
      if (p.occupied) continue;
      const dx = f.x - p.x, dz = f.z - p.z, gap = dx * p.nx + dz * p.nz;     // + = on the pad side of the port plane
      if (gap >= B.free + 0.02 || gap < -0.05) continue;
      if (Math.hypot(dx - gap * p.nx, dz - gap * p.nz, f.y - p.y) > DOCK.openingD / 2 + 0.6) continue;
      const faceR = c.T.shape === 'cylinder' ? c.T.radius : Math.min(c.T.width, c.T.height ?? c.T.width) / 2;
      const rimR = Math.max(0.2, Math.min(DOCK.openingD / 2 - 0.1, faceR - 0.05));
      const v = c.body.linvel(), w = c.body.angvel(), com = c.body.worldCom();
      const N = BELLOWS_POINTS, kN = B.k / N, cN = B.c / N;
      let Ft = 0;
      for (let i = 0; i < N; i++) {
        const a = (i / N) * 2 * Math.PI;
        const pt = new THREE.Vector3(c.T.length / 2, rimR * Math.cos(a), rimR * Math.sin(a)).applyQuaternion(q).add(new THREE.Vector3(t.x, t.y, t.z));
        const gi = (pt.x - p.x) * p.nx + (pt.z - p.z) * p.nz;
        if (gi >= B.free) continue;
        const rx = pt.x - com.x, ry = pt.y - com.y, rz = pt.z - com.z;   // point velocity v + w x r, along the port normal
        const vn = (v.x + w.y * rz - w.z * ry) * p.nx + (v.z + w.x * ry - w.y * rx) * p.nz;   // + = backing away
        const F = Math.max(0, kN * (B.free - gi) - cN * vn);
        if (F > 0) { c.body.applyImpulseAtPoint({ x: p.nx * F * dt, y: 0, z: p.nz * F * dt }, { x: pt.x, y: pt.y, z: pt.z }, true); Ft += F; }
      }
      if (Ft > 0) this._bellowsF = { c, port: p, F: Ft };
      return;
    }
  }

  // ---------------- transit warp ----------------
  _stepShiftRecovery(dt) {
    const L = this.rover.lift;
    if (L.shifted && L.base) {
      if (!this._shiftT) { this._shiftT = this.time; this.log('Cradle re-centring cargo (lateral actuator)', 'warn'); }
      L.base.z += Math.max(-0.01 * dt, Math.min(0.01 * dt, -L.base.z * 0.5));
      if (Math.abs(L.base.z) < 0.01) { L.shifted = false; this._shiftT = 0; this.log('Cargo re-centred, load path nominal', 'ok'); }
    }
  }
  warpWanted() { return this.warp.enabled && this.timeScale >= this.warp.minScale && !this.paused; }
  // One kinematic step of H seconds. Returns the sim time advanced (0 if the autonomy declined the rails this tick:
  // the caller then runs physics steps, and the autonomy keeps driving them until the rails check passes again).
  warpStep(h = this.warp.H) {
    const A = this.autonomy, R = this.rover, S = this.sensors, W = this.warp;
    if (this.comms.queue.length) return { dt: 0, autoRan: false }; // preserve delayed command execution in stepOnce
    if (!W.active) { W.active = true; R.railsBegin(); W.lastScanD = -1e9; W.lastStreamD = -1e9; }
    // camera at 1 Hz on rails: the fiducial beacons (habitat face, power dock, lander) must still be seen, or the SLAM
    // drift is never corrected on the warped approach to the base (the loaded rover then circled the lead-in point
    // 17-20 m off). Each frame is fused by this tick's autonomy update and then cleared, so it is never re-fused.
    S.detections = []; S.rockDetections = [];
    W.camT = (W.camT ?? 0) - h;
    if (W.camT <= 0) { W.camT = 1; if (S.cameraOn && !S.faults.camera && R.powered && this.thermal?.ok.camera !== false) S._doCamera(); }
    this.warping = true; A._railsMoved = false;
    try { A.update(h); } finally { this.warping = false; S.detections = []; S.rockDetections = []; }
    if (!A._railsMoved) return { dt: 0, autoRan: true };
    S.sunTick(h);
    this.slam.update(h);   // integrates the odometry the rails produced (est stays in its own SLAM frame)
    if (this.scout.enabled) this.scout.update(h);
    this._stepShiftRecovery(h);
    const d = S.odom.dist;
    if (d - W.lastStreamD > 30) { W.lastStreamD = d; this.streamRocks([R.pose()]); this.world.updateSceneQueries(); }
    if (d - W.lastScanD > 6) { W.lastScanD = d; S.warpScan(); }
    for (const m of [this.powerDock, this.health, this.thermal, this.telemetry]) { m._acc += h; if (m._acc >= 1 / (m.rate || 120) - 1e-9) { m.update(m._acc); m._acc = 0; } }
    this._startupTick();
    if (this.lunar.on) { const Ln = this.lunar; Ln.phase = (Ln.phase + h / Ln.periodS) % 1; this.setSun((this.sunAz0 ?? 205) + Ln.phase * 360, 10 * Math.sin(Ln.phase * 2 * Math.PI) - 1); }
    this.time += h; W.steps++; W.simS += h;
    return { dt: h, autoRan: true };
  }
  endWarp() {
    if (!this.warp.active) return;
    this.warp.active = false;
    this.rover.railsEnd();
    this.streamRocks([this.rover.pose()]);
    this._shadow = null; // float64 position shadows restart from the solver's positions
  }
  // Fixed rails step for the current time scale: whole physics steps, ~one frame of sim time (RAILS_FRAME_S x scale), clamped
  // to [PHYS.dt, warp.H]. It depends only on timeScale, never on the wall-clock backlog, so a warped leg integrates the same
  // way on a fast or a slow machine (5x: 0.083 s, 10x: 0.167 s, >= 30x: 0.5 s).
  railsStep() {
    const n = Math.round(RAILS_FRAME_S * Math.max(1, this.timeScale) / PHYS.dt);
    return Math.max(1, Math.min(Math.round(this.warp.H / PHYS.dt), n)) * PHYS.dt;
  }
  // one step of whichever kind applies; returns the sim seconds advanced. h (optional): rails step size, clamped to
  // [PHYS.dt, warp.H]; advance() always passes the fixed railsStep(), tests may pass their own.
  stepAuto(owed) {
    let skipAutonomy = false;
    if (this.warpWanted() && this.autonomy.railsOk) {
      const r = this.warpStep(owed == null ? this.warp.H : Math.max(PHYS.dt, Math.min(this.warp.H, owed)));
      if (r.dt > 0) return r.dt;
      skipAutonomy = r.autoRan;
    }
    this.endWarp();
    return this.stepOnce({ skipAutonomy }) || PHYS.dt;
  }

  // advance by wall-clock dt with time scaling, within a CPU budget per frame; returns the sim seconds advanced
  advance(realDt, budgetMs = 20) {
    if (this.paused) { this.actualRate = 0; return 0; }
    this._acc = (this._acc || 0) + Math.min(realDt, 0.1) * this.timeScale;
    // The budget grows with the time scale, but only to PHYS.warpBudgetMaxMs (50 ms): beyond that the frame rate collapses (160 ms
    // of physics per frame is 6 frames a second) for a few per cent more sim time. Sim speed above what the CPU can step comes from
    // the rails (stepAuto), not from a longer frame.
    const scale = Math.max(1, this.timeScale);
    const effectiveBudget = Math.min(budgetMs * Math.min(scale, 2.5), PHYS.warpBudgetMaxMs);
    const effectiveMaxSteps = Math.min(PHYS.maxSubSteps * scale, PHYS.warpMaxSteps);
    const t0 = performance.now(), simT0 = this.time;
    let steps = 0;
    while (this._acc >= PHYS.dt) {
      this._acc -= this.stepAuto(this.railsStep());   // fixed step, not the wall-clock backlog (it may overpay: _acc goes negative and the next frame waits)
      if (++steps >= effectiveMaxSteps || performance.now() - t0 > effectiveBudget) break;
    }
    // carry a little backlog between frames; beyond that the CPU cannot keep up with the time scale
    const cap = Math.max(0.25, this.timeScale * 0.05);
    if (this._acc > cap) { this._acc = cap; this._lagT = (this._lagT || 0) + realDt; } else this._lagT = Math.max(0, (this._lagT || 0) - realDt);
    this.lagging = this._lagT > 1;
    const adv = this.time - simT0;
    if (realDt > 0) this.actualRate += (adv / Math.min(realDt, 0.1) - this.actualRate) * Math.min(1, realDt / 1.5);
    return adv;
  }
}
