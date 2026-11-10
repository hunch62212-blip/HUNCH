// Sensor models: 16-channel spinning LiDAR, monocular camera with fiducial (AprilTag-style)
// detection and rock detection, IMU (gyro bias random walk), wheel odometry, and the cradle
// alignment sensor suite (lateral laser rangefinders + light curtains + dock gap sensor).
import * as THREE from 'three';
import { ROVER, CARGO_TYPES, DOCK } from './config.js';
import { Q_SENSOR, LANDER_SCALE, LANDER_TAG } from './world.js';
import { mulberry32, heightAt } from './terrain.js';
import { powerDockLayout } from './powerDock.js';
import { wheelSoil } from './terramechanics.js';

const rand = mulberry32(777);
export function gauss() { let u = 0, v = 0; while (u === 0) u = rand(); while (v === 0) v = rand(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

export class Sensors {
  constructor(sim) {
    this.sim = sim;
    this.rate = 20; // module update rate (Hz)
    const L = ROVER.lidar;
    this.lidarCfg = { channels: L.channels, hRes: L.hRes, vMin: L.vFovDeg[0], vMax: L.vFovDeg[1], range: L.range, rateHz: L.rateHz };
    this.lidarOn = true; this.cameraOn = true; this.lidarUnit = 'A'; this.lidarDead = {};
    this.faults = { lidar: false, camera: false, imu: false };
    this.scan = null; this.scanId = 0;
    this._lidarT = 0; this._camT = 0;
    this.ray = new sim.RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 0, y: -1, z: 0 });
    this.gyroBias = 0.000015 * gauss();              // rad/s (tactical-grade FOG/MEMS, ~3 deg/h)
    this.imu = { roll: 0, pitch: 0, yawRate: 0 };
    this.odom = { dx: 0, dz: 0, dy: 0, dyaw: 0, vx: 0, vz: 0, dist: 0 };  // accumulated increments, consumed by SLAM
    this._omPrev = new Map(); // last encoder speed per wheel (spin-up torque for the slip estimate)
    this.detections = []; this.rockDetections = [];
    this.cass = null;
    this._buildTargets();
    this._dirs = null; this._buildDirs();
    // pre-allocated scratch objects for _doCamera (called at 10 Hz)
    this._camV = new THREE.Vector3();
    this._camWpos = new THREE.Vector3();
    this._camWn = new THREE.Vector3();
  }

  slipFromTorque(T, W) {
    const r = ROVER.wheel.radius, b = ROVER.wheel.width, D = 2 * r * ROVER.wheel.effDiameterFactor;
    const Hm = T * (1 + 0.03 * gauss()) / r;
    const sign = Math.sign(Hm), H = Math.abs(Hm);
    if (H < 1) return 0;
    let lo = 0, hi = 1;
    if (wheelSoil(W, b, D, 1, 0).H <= H) return sign;
    for (let i = 0; i < 18; i++) { const mid = (lo + hi) / 2; if (wheelSoil(W, b, D, mid, 0).H < H) lo = mid; else hi = mid; }
    return sign * (lo + hi) / 2;
  }

  _buildDirs() {
    const { channels, hRes, vMin, vMax } = this.lidarCfg;
    this._dirs = new Float32Array(channels * hRes * 3);
    let k = 0;
    // sensor head pitched nose-down (about body Z) so the near field in front of the wheels is seen
    const tilt = (ROVER.lidar.tiltDeg || 0) * Math.PI / 180, ct = Math.cos(tilt), st = Math.sin(tilt);
    for (let c = 0; c < channels; c++) {
      const el = (vMin + (vMax - vMin) * c / Math.max(1, channels - 1)) * Math.PI / 180;
      for (let h = 0; h < hRes; h++) {
        const az = (h / hRes) * Math.PI * 2;
        const dx = Math.cos(el) * Math.cos(az), dy = Math.sin(el), dz = -Math.cos(el) * Math.sin(az);
        this._dirs[k++] = dx * ct - dy * st; this._dirs[k++] = dx * st + dy * ct; this._dirs[k++] = dz;
      }
    }
  }
  // Adaptive spin: the head turns faster when the rover is fast (fresher scans, ~constant ray budget, coarser azimuth),
  // and slower with finer azimuth when it creeps (docking, pickup). Applied only between scans.
  _lidarPolicy() {
    if (this.lidarAuto === false) return;
    const now = this.sim.time;
    if (now - (this._polT ?? -9) < 2) return;
    const v = Math.hypot(this.odom.vx, this.odom.vz);
    const tier = v < 0.5 ? [5, 240] : v < 2.5 ? [8, 150] : [12, 100];
    const C = this.lidarCfg;
    if (C.rateHz !== tier[0] || C.hRes !== tier[1]) {
      C.rateHz = tier[0]; C.hRes = tier[1]; this._buildDirs(); this._polT = now;
    }
  }
  // Two LiDAR heads sit on the front deck corners (lidar_mount.js: port unit A, starboard unit B, mirrored in z).
  // A is prime, B a cold spare: FDIR switches to it when A does not come back from a power cycle (health.js).
  switchLidar() {
    if (this.lidarUnit === 'B' || this.lidarDead?.B) return false;
    this.lidarDead = { ...(this.lidarDead || {}), A: true }; this.lidarUnit = 'B'; this._partial = null; this.faults.lidar = false;
    return true;
  }
  setLidar(channels, hRes) { this.lidarAuto = false; if (channels) this.lidarCfg.channels = channels; if (hRes) this.lidarCfg.hRes = hRes; this._buildDirs(); }

  _buildTargets() {
    // fiducials: local position + outward normal on each object
    this.targets = [];
    for (const c of this.sim.cargo) {
      const T = c.T;
      const h = T.shape === 'cylinder' ? T.radius : T.height / 2;
      this.targets.push({ id: `${c.id}/AFT`, obj: c, kind: 'cargo', local: new THREE.Vector3(-T.length / 2 - 0.01, -h * 0.62, 0), normal: new THREE.Vector3(-1, 0, 0), size: 0.8 }); // low on the aft cap so the rover camera keeps it in view while driving under
      this.targets.push({ id: `${c.id}/FWD`, obj: c, kind: 'cargo', local: new THREE.Vector3(T.length / 2 + 0.01, h * 0.62, 0), normal: new THREE.Vector3(1, 0, 0), size: 0.6 });
      for (const sz of [-1, 1]) this.targets.push({ id: `${c.id}/${sz < 0 ? 'L' : 'R'}`, obj: c, kind: 'cargo', local: new THREE.Vector3(-1.2, -0.3, sz * ((T.shape === 'cylinder' ? T.radius : T.width / 2) + 0.035)), normal: new THREE.Vector3(0, 0, sz), size: 0.8 });
    }
    for (const p of this.sim.ports) {
      const w = new THREE.Vector3(p.x + p.nx * 0.02, p.y - 1.5, p.z + p.nz * 0.02);
      this.targets.push({ id: `${p.name}`, obj: p, kind: 'port', world: w, normal: new THREE.Vector3(p.nx, 0, p.nz), size: 0.55, known: { x: w.x, z: w.z, nx: p.nx, nz: p.nz } });
    }
    // surveyed beacons on the lander's 4 sides, on the descent-stage wall. Same LANDER_SCALE as the lander's
    // colliders and model: unscaled, the tags sat 7 m inside the descent-stage collider and the camera's
    // line-of-sight ray always hit the hull first, so they were never seen.
    const LS = LANDER_SCALE, lx = this.sim.landerPos.x, lz = this.sim.landerPos.z, ly = this.sim.terrain.padH + LANDER_TAG.y * LS;
    for (let k = 0; k < 4; k++) {
      const a = k * Math.PI / 2, nx = Math.cos(a), nz = Math.sin(a);
      const w = new THREE.Vector3(lx + nx * LANDER_TAG.r * LS, ly, lz + nz * LANDER_TAG.r * LS);
      this.targets.push({ id: `LANDER-${'ENWS'[k]}`, obj: { id: 'lander' }, kind: 'beacon', world: w, normal: new THREE.Vector3(nx, 0, nz), size: LANDER_TAG.size * LS, known: { x: w.x, z: w.z, nx, nz } });
    }
    // surveyed beacon on the power-dock battery vault, facing the charge pad (1.0 m tag, ASSUMED): bounds the
    // SLAM error at the dock so the rover can park on the coupler from the map alone
    { const H = this.sim.terrain.hab, L = powerDockLayout(H), b = L.bank, nx = H.nx, nz = H.nz;
      const w = new THREE.Vector3(b.x + nx * (b.d / 2 + 0.02), heightAt(this.sim.terrain, b.x, b.z) + b.h * 0.6, b.z + nz * (b.d / 2 + 0.02));
      this.targets.push({ id: 'POWER-DOCK', obj: { id: 'powerdock' }, kind: 'beacon', world: w, normal: new THREE.Vector3(nx, 0, nz), size: 1.0, known: { x: w.x, z: w.z, nx, nz } }); }
    // surveyed beacon on the habitat supply building facade (1.2 m tag, ASSUMED): bounds
    // SLAM drift when approaching the habitat from the landing pad traverse
    { const H = this.sim.terrain.hab, f = H.face, nx = H.nx, nz = H.nz;
      const w = new THREE.Vector3(f.x + nx * 0.05, heightAt(this.sim.terrain, f.x, f.z) + 7.5, f.z + nz * 0.05);
      this.targets.push({ id: 'HAB-FACE', obj: { id: 'hab' }, kind: 'beacon', world: w, normal: new THREE.Vector3(nx, 0, nz), size: 1.2, known: { x: w.x, z: w.z, nx, nz } }); }
  }

  cameraPose() {
    const p = this.sim.rover.pose();
    const C = ROVER.camera;
    const pos = new THREE.Vector3(C.mountLocal.x, C.mountLocal.y, C.mountLocal.z).applyQuaternion(p.q).add(new THREE.Vector3(p.x, p.y, p.z));
    const q = p.q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), C.pitchDeg * Math.PI / 180));
    return { pos, q, fwd: new THREE.Vector3(1, 0, 0).applyQuaternion(q), up: new THREE.Vector3(0, 1, 0).applyQuaternion(q), right: new THREE.Vector3(0, 0, 1).applyQuaternion(q) };
  }

  _castRay(o, d, max, excludeBody) {
    const r = this.ray;
    r.origin.x = o.x; r.origin.y = o.y; r.origin.z = o.z; r.dir.x = d.x; r.dir.y = d.y; r.dir.z = d.z;
    return this.sim.world.castRay(r, max, true, undefined, Q_SENSOR, undefined, excludeBody || this.sim.rover.body);
  }

  update(dt) {
    const sim = this.sim, R = sim.rover;
    // --- IMU
    const p = R.pose();
    this.gyroBias += 0.000001 * gauss() * Math.sqrt(dt);
    const av = R.body.angvel();
    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(p.q);
    const yawRate = av.x * up.x + av.y * up.y + av.z * up.z;
    // imuFaultBias: bias jump of a failed IMU while it is still the one in use (health.js 'imu')
    this.imu.yawRate = yawRate + this.gyroBias + (this.imuFaultBias || 0) + 0.0004 * gauss();
    this.imu.roll = R.tilt.roll + 0.0009 * gauss();
    this.imu.pitch = R.tilt.pitch + 0.0009 * gauss();
    // --- wheel odometry (body velocity from the bogies' encoder speeds)
    // Model-based slip estimate: motor current -> torque -> soil thrust H = T/r; invert the Janosi thrust curve
    // with the strut load-cell wheel load to recover slip, then ground speed = r*omega*(1 - s). Torque that only
    // changes a wheel's own spin (I*domega/dt, from the encoder) never reaches the soil, so it is removed first;
    // otherwise every speed change reads as slip (CHANGES.md §7).
    // One body-level slip from the NET drive torque and mean load, not one per wheel: in a skid-steer turn the outer
    // wheels drive and the inner ones brake against each other (scrub). Those torques cancel across the vehicle,
    // but inverted wheel by wheel through the nonlinear thrust curve they did not, and the loaded rover's odometry
    // read 6-7 % short in every turn (1.4 % on straights): ~30 m of SLAM drift over a 1.5 km leg (CHANGES.md §14)
    let ux = 0, uz = 0, n = 0, Tnet = 0, Nsum = 0, nc = 0;
    for (const b of R.bogies) {
      let vsum = 0, k = 0;
      for (const w of b.wheels) {
        const omPrev = this._omPrev.get(w) ?? w.omega;
        this._omPrev.set(w, w.omega);
        if (w.contact && w.load > 50) {
          Tnet += w.torque - (dt > 0 ? ROVER.wheel.inertia * (w.omega - omPrev) / dt : 0);
          Nsum += w.load; nc++;
        }
        vsum += w.omega * ROVER.wheel.radius; k++;
      }
      const s = (vsum / k) * (1 + 0.004 * gauss());
      const st = b.steer + 0.002 * gauss();
      ux += s * Math.cos(st); uz += -s * Math.sin(st); n++;
    }
    ux /= n; uz /= n;
    if (nc) {
      const sEst = this.slipFromTorque(Tnet / nc, Nsum / nc), f = sEst >= 0 ? 1 - sEst : 1 / (1 - sEst);
      ux *= f; uz *= f;
    }
    const cp = Math.cos(this.imu.pitch);
    this.odom.vx = ux; this.odom.vz = uz;
    this.odom.dx += ux * cp * dt; this.odom.dz += uz * dt; this.odom.dy += ux * Math.sin(this.imu.pitch) * dt; {
      // skid-steer yaw scrub correction on the heading integral (ROVER.imu, CHANGES.md §17); zero for this gyro model
      const wz = this.imu.yawRate, K = ROVER.imu;
      const scrub = R.loaded && K.kScrub && Math.abs(wz) > K.scrubMinWz ? K.kScrub * Math.sign(wz) * Math.abs(ux * wz) : 0;
      this.odom.dyaw += (wz - scrub) * dt;
    }
    this.odom.dist += Math.hypot(ux, uz) * dt;

    // --- LiDAR: spinning head, one azimuth sector per sensor tick (rolling scan), de-skewed at scan end
    if (this.lidarOn && !this.faults.lidar && R.powered && this.sim.thermal?.ok.lidar !== false) { // thermal: protected OFF out of range
      if (!this._partial) this._lidarPolicy();
      const sectors = Math.max(1, Math.round(this.rate / this.lidarCfg.rateHz));
      if (!this._partial) this._beginScan(sectors);
      this._scanSector(p);
      if (this._partial.sector >= this._partial.sectors) this._finishScan(R.pose());
    } else this._partial = null;
    // --- Camera (fiducials + rock detection) at 10 Hz
    this._camT -= dt;
    if (this._camT <= 0) {
      this._camT += 0.1;
      if (this.cameraOn && !this.faults.camera && R.powered && this.sim.thermal?.ok.camera !== false) this._doCamera(); else { this.detections = []; this.rockDetections = []; }
    }
    this._cassN = (this._cassN || 0) + 1;
    if (this.sim.timeScale <= 3 || this._cassN % 2 === 0 || !this.cass) this.cass = this._doCASS(p);
    this.sunTick(dt, p);
  }
  // sun sensor (1 Hz): sun azimuth in the body frame -> absolute heading from ephemeris. Also run by Sim.warpStep:
  // without it FDIR saw "sun visible by ephemeris, sensor silent" during transit warp and raised a false failure.
  sunTick(dt, p = this.sim.rover.pose()) {
    const sim = this.sim, R = sim.rover;
    this._sunT = (this._sunT || 0) - dt;
    if (this._sunT <= 0) {
      this._sunT = 1;
      const sun = sim.sunDir;
      const qinv = p.q.clone().invert();
      const sb = sun.clone().applyQuaternion(qinv);
      this.sunSensor = R.power.sunlit && sun.y > 0 && !this.faults.sun ? { t: sim.time, azBody: Math.atan2(-sb.z, sb.x) + 0.004 * gauss() } : null;
    }
  }

  _beginScan(sectors) {
    const { channels, hRes } = this.lidarCfg;
    const N = channels * hRes;
    this._partial = { sectors, sector: 0, world: new Float32Array(N * 3), kind: new Uint8Array(N), m: 0 };
  }
  _scanSector(p) {
    const L = ROVER.lidar, P = this._partial;
    const o = new THREE.Vector3(L.mountLocal.x, L.mountLocal.y, L.mountLocal.z * (this.lidarUnit === 'B' ? -1 : 1)).applyQuaternion(p.q).add(new THREE.Vector3(p.x, p.y, p.z));
    const { channels, hRes, range } = this.lidarCfg;
    const h0 = Math.floor(P.sector * hRes / P.sectors), h1 = Math.floor((P.sector + 1) * hRes / P.sectors);
    const d = new THREE.Vector3(), q = p.q, dirs = this._dirs, colliderRock = this.sim.colliderRock;
    // above 3x time scale, fire every other azimuth to keep the simulation interactive
    const hs = this.sim.timeScale > 3 ? 2 : 1;
    this.decimated = hs > 1;
    for (let c = 0; c < channels; c++) for (let h = h0; h < h1; h += hs) {
      const i = c * hRes + h;
      d.set(dirs[i * 3], dirs[i * 3 + 1], dirs[i * 3 + 2]).applyQuaternion(q);
      const hit = this._castRay(o, d, range);
      if (!hit) continue;
      const t = hit.timeOfImpact + L.noise * gauss();
      if (t < 1.0) continue;
      const m = P.m++;
      P.world[m * 3] = o.x + d.x * t; P.world[m * 3 + 1] = o.y + d.y * t; P.world[m * 3 + 2] = o.z + d.z * t;
      const hc = hit.collider.handle;
      const owner = this.sim.colliderOwner.get(hc);
      P.kind[m] = hc === this.sim.terrainCollider.handle ? 0 : colliderRock.has(hc) ? 1 : owner && owner.kind === 'cargo' ? 3 : 2;
    }
    P.sector++;
  }
  _finishScan(p) {
    // motion de-skew: express every return in the body frame at the end of the sweep
    const P = this._partial; this._partial = null;
    const m = P.m, body = new Float32Array(m * 3);
    const qinv = p.q.clone().invert(), v = new THREE.Vector3();
    for (let i = 0; i < m; i++) {
      v.set(P.world[i * 3] - p.x, P.world[i * 3 + 1] - p.y, P.world[i * 3 + 2] - p.z).applyQuaternion(qinv);
      body[i * 3] = v.x; body[i * 3 + 1] = v.y; body[i * 3 + 2] = v.z;
    }
    this.scan = { t: this.sim.time, n: m, body, world: P.world.subarray(0, m * 3), kind: P.kind.subarray(0, m), roll: this.imu.roll, pitch: this.imu.pitch, id: ++this.scanId };
    this.sim.emit('scan', this.scan);
  }
  _doLidar(p) { this._beginScan(1); this._scanSector(p); this._finishScan(p); }
  // Transit warp (Sim.warpStep): one coarse full sweep from the current pose every few metres keeps the map ahead of
  // the rover filled. The pose comes from the rails, so scan matching is skipped (the estimate is already consistent).
  warpScan() {
    const R = this.sim.rover;
    this.detections = []; this.rockDetections = [];
    if (!this.lidarOn || this.faults.lidar || !R.powered) return;
    const C = this.lidarCfg;
    if (C.hRes !== 120) { C.hRes = 120; C.rateHz = 12; this._buildDirs(); this._polT = this.sim.time; }
    this._partial = null;
    const slam = this.sim.slam, m = slam.matching;
    slam.matching = false;
    try { this._doLidar(R.pose()); } finally { slam.matching = m; }
  }

  _doCamera() {
    const cam = this.cameraPose();
    const C = ROVER.camera;
    const hf = C.hFovDeg * Math.PI / 360, vf = Math.atan(Math.tan(hf) * 0.75);
    const out = [];
    const v = new THREE.Vector3();
    // Lighting (a camera, unlike the LiDAR, needs light and cannot look into the sun):
    //  - sun in the frame (low polar sun, no atmosphere to soften it): veiling glare halves the usable range, and
    //    nothing within glareDeg of the sun's direction can be decoded
    //  - night or the rover in shadow: cargo fiducials and rocks are seen only in the rover's own headlights
    //    (lampRange); port and surveyed beacons are floodlit by the base
    const sun = this.sim.sunDir, R0 = this.sim.rover, lit = sun.y > 0 && R0.power.sunlit;
    const sunFx = sun.dot(cam.fwd), sunInFrame = lit && sunFx > 0 && Math.abs(Math.atan2(sun.dot(cam.right), sunFx)) < hf && Math.abs(Math.atan2(sun.dot(cam.up), sunFx)) < vf;
    const glareCos = Math.cos(12 * Math.PI / 180), lampRange = 35, glareK = sunInFrame ? 0.5 : 1;
    this.camLight = { lit, sunInFrame, lamps: !!R0.lights };
    const visible = (dirx, diry, dirz, range, floodlit) => {
      if (lit) return !(dirx * sun.x + diry * sun.y + dirz * sun.z > glareCos);
      return floodlit || (R0.lights && range <= lampRange);
    };
    for (const tg of this.targets) {
      let wpos, wn;
      if (tg.kind === 'cargo') {
        const t = tg.obj.body.translation(), r = tg.obj.body.rotation();
        const q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
        wpos = this._camWpos.copy(tg.local).applyQuaternion(q);
        wpos.x += t.x; wpos.y += t.y; wpos.z += t.z;
        wn = this._camWn.copy(tg.normal).applyQuaternion(q);
      } else { wpos = tg.world; wn = tg.normal; }
      v.subVectors(wpos, cam.pos);
      const range = v.length();
      const maxR = tg.size * 85 * glareK;
      if (range > maxR || range < 0.8) continue;
      if (!visible(v.x / range, v.y / range, v.z / range, range, tg.kind !== 'cargo')) continue;
      const fx = v.dot(cam.fwd), rx = v.dot(cam.right), uy = v.dot(cam.up);
      if (fx <= 0) continue;
      const ax = Math.atan2(rx, fx), ay = Math.atan2(uy, fx);
      if (Math.abs(ax) > hf || Math.abs(ay) > vf) continue;
      const facing = -wn.dot(v) / range;
      if (facing < 0.25) continue; // too oblique to decode
      // line of sight
      const dir = v.clone().divideScalar(range);
      const hit = this._castRay(cam.pos, dir, range - 0.05);
      if (hit) {
        const own = this.sim.colliderOwner.get(hit.collider.handle);
        const isBeacon = tg.kind === 'beacon' && hit.timeOfImpact > range - 1.5;
        if (!(own && own.ref === tg.obj) && !isBeacon) continue;
      }
      const sigma = 0.004 + 0.0009 * range * range / tg.size;
      const meas = wpos.clone().add(this._camV.set(gauss() * sigma, gauss() * sigma * 0.6, gauss() * sigma));
      const yawNoise = (0.2 + 0.05 * range) * Math.PI / 180 * gauss();
      out.push({ id: tg.id, kind: tg.kind, obj: tg.obj, known: tg.known, range, pos: meas, normal: wn.clone(), yawNoise, u: 0.5 + Math.tan(ax) / Math.tan(hf) * 0.5, vv: 0.5 - Math.tan(ay) / Math.tan(vf) * 0.5, size: tg.size, confidence: Math.max(0.2, Math.min(1, facing * (1 - range / maxR) * 1.6)) });
    }
    this.detections = out;
    // rock detector (simulated learned detector: recall drops with range and size)
    const rocks = [];
    for (const [i] of this.sim.rockColliders) {
      const r = this.sim.terrain.rocks[i];
      if (r.d < 0.4) continue;
      v.set(r.x, r.top, r.z).sub(cam.pos);
      const range = v.length();
      if (range > C.range * (this.camRangeK ?? 1) * glareK) continue; // camRangeK < 1: dusty window (health.js 'cdust')
      if (!visible(v.x / range, v.y / range, v.z / range, range, false)) continue;
      const fx = v.dot(cam.fwd), rx = v.dot(cam.right), uy = v.dot(cam.up);
      if (fx <= 0) continue;
      const ax = Math.atan2(rx, fx), ay = Math.atan2(uy, fx);
      if (Math.abs(ax) > hf || Math.abs(ay) > vf) continue;
      if (r.d / range < 0.012) continue;
      const ang = Math.atan2(r.d * 0.6, range);
      rocks.push({ u: 0.5 + Math.tan(ax) / Math.tan(hf) * 0.5, v: 0.5 - Math.tan(ay) / Math.tan(vf) * 0.5, w: Math.tan(ang) / Math.tan(hf), h: Math.tan(ang * r.sy) / Math.tan(vf), d: r.d, range, conf: Math.min(0.99, 0.55 + r.d / range * 9) });
    }
    rocks.sort((a, b) => a.range - b.range);
    this.rockDetections = rocks.slice(0, 40);
  }

  // Cradle Alignment Sensor Suite (AEGIS-V1, under-module pickup): at each ring saddle two upward laser
  // rangefinders (z = +-1.4 m) measure the module's underside; with the known radius that gives the module's
  // lateral offset at that ring, and the two rings give yaw. Inductive band-edge sensors on the saddles see the
  // steel clamp bands (wear sleeves) and give the axial error. Valid only with the module overhead.
  _doCASS(p) {
    const R = this.sim.rover;
    const res = { lat: [], valid: false };
    const o = new THREE.Vector3(), d = new THREE.Vector3();
    const P = new THREE.Vector3(p.x, p.y, p.z);
    const F = ROVER.frame, zs = 1.4;
    const offs = [];
    for (const rx of [-ROVER.ringX, ROVER.ringX]) {
      const dist = {};
      let target = null;
      for (const sz of [-1, 1]) {
        o.set(rx, F.deckY + 0.05, sz * zs).applyQuaternion(p.q).add(P);
        d.set(0, 1, 0).applyQuaternion(p.q);
        const hit = this._castRay(o, d, 12);
        const own = hit && this.sim.colliderOwner.get(hit.collider.handle);
        dist[sz] = own && own.kind === 'cargo' ? hit.timeOfImpact + 0.004 * gauss() : null;
        if (own && own.kind === 'cargo') target = own.ref;
        res.lat.push({ x: rx, side: sz, d: dist[sz] });
      }
      if (dist[-1] != null && dist[1] != null && target) {
        // underside height h(z) = H - sqrt(R^2 - (z - c)^2); solve the axis offset c from the difference
        const Rm = target.T.adapterR || target.T.radius;
        const f = (c) => (Math.sqrt(Math.max(0, Rm * Rm - (zs - c) ** 2)) - Math.sqrt(Math.max(0, Rm * Rm - (-zs - c) ** 2))) - (dist[-1] - dist[1]);
        let lo = -2.5, hi = 2.5;
        for (let i = 0; i < 40; i++) { const mid = (lo + hi) / 2; if (f(lo) * f(mid) <= 0) hi = mid; else lo = mid; }
        offs.push({ x: rx, c: (lo + hi) / 2, target });
      }
    }
    if (offs.length === 2) {
      res.valid = true;
      res.lateral = (offs[0].c + offs[1].c) / 2;            // + => module axis to the right (+Z)
      res.yaw = Math.atan2(offs[1].c - offs[0].c, offs[1].x - offs[0].x);
      // band-edge sensors: axial position of the clamp bands relative to the rings
      const c = offs[0].target, t = c.body.translation();
      const rel = new THREE.Vector3(t.x - p.x, t.y - p.y, t.z - p.z).applyQuaternion(p.q.clone().invert());
      if (Math.abs(rel.x - (c.T.carryX || 0)) < 1.2) res.cargoX = rel.x + 0.02 * gauss();
      res.seatGap = Math.min(...res.lat.filter((l) => l.d != null).map((l) => l.d)) - (ROVER.seatY - F.deckY - 0.05);
    }
    const L = R.lift;
    if (L.cargo) {
      const t = L.cargo.body.translation(), r = L.cargo.body.rotation();
      const q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
      // dock gap sensor: from the held module's forward (hatch) end, straight ahead
      o.set(L.cargo.T.length / 2 + 0.02, 0, 0).applyQuaternion(q).add(new THREE.Vector3(t.x, t.y, t.z));
      d.set(1, 0, 0).applyQuaternion(q);
      const hit = this._castRay(o, d, 12, L.cargo.body);
      res.gap = hit ? hit.timeOfImpact + 0.002 * gauss() : null;
      // docking-ring close-range camera: offset of the nearest port centre from the cargo axis
      let bestP = null, bd = 12; // a 2 m port ring resolves to ~mm at 12 m with a narrow-field lens (ASSUMED)
      for (const port of this.sim.ports) {
        const dd = Math.hypot(port.x - o.x, port.z - o.z);
        if (dd < bd) { bd = dd; bestP = port; }
      }
      if (bestP && !this.faults.camera) {
        const rel = new THREE.Vector3(bestP.x - o.x, bestP.y - o.y, bestP.z - o.z).applyQuaternion(q.clone().invert());
        // relative yaw: the camera sees the ring as an ellipse, whose aspect / orientation gives its tilt
        // (+ = turn left to square the module axis onto the port axis; 0.03 deg noise, ASSUMED)
        const into = new THREE.Vector3(-bestP.nx, 0, -bestP.nz).applyQuaternion(q.clone().invert());
        res.ring = { port: bestP, along: rel.x, lateral: rel.z + 0.003 * gauss(), vertical: rel.y + 0.003 * gauss(), yaw: Math.atan2(-into.z, into.x) + 0.0005 * gauss() };
      }
    }
    return res;
  }
}
