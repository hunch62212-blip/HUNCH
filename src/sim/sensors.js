// Sensor models: 32-channel 360 deg spinning LiDAR on the front deck corner, stereo depth camera with fiducial
// (AprilTag-style) detection and rock detection, IMU (gyro bias random walk), wheel odometry, and the cradle
// alignment sensor suite (lateral laser rangefinders + light curtains + dock gap sensor).
import * as THREE from 'three';
import { ROVER, CARGO_TYPES, DOCK } from './config.js';
import { Q_SENSOR, LANDER_SCALE, LANDER_TAG } from './world.js';
import { mulberry32, heightAt } from './terrain.js';
import { powerDockLayout } from './powerDock.js';
import { wheelSoil } from './terramechanics.js';

const rand = mulberry32(777);
export function gauss() { let u = 0, v = 0; while (u === 0) u = rand(); while (v === 0) v = rand(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
// the LiDAR and the stereo depth camera draw from their own stream: their hit counts depend on the terrain and the scan policy, and
// sharing the IMU / camera / alignment-sensor stream above made every precision sensor's noise change whenever the scan did
const randRange = mulberry32(31337);
function gaussRange() { let u = 0, v = 0; while (u === 0) u = randRange(); while (v === 0) v = randRange(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

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
  // Adaptive spin (ray budget): the head turns faster when the rover is fast (fresher scans, ~constant ray budget, coarser
  // azimuth), and slower with finer azimuth when it creeps (docking, pickup): ROVER.lidar.tiers. At time scales above 1x the scan
  // RATE drops as 1 / scale (min 1 Hz) and the azimuth steps are capped at 180 from 10x: the same sim-time coverage for the same
  // wall-clock cost. Applied only between scans.
  _lidarPolicy() {
    if (this.lidarAuto === false) return;
    const now = this.sim.time;
    if (now - (this._polT ?? -9) < 2) return;
    const v = Math.hypot(this.odom.vx, this.odom.vz), ts = Math.max(1, this.sim.timeScale);
    const [, hz, az] = ROVER.lidar.tiers.find((t) => v < t[0]) || ROVER.lidar.tiers[ROVER.lidar.tiers.length - 1];
    const rate = Math.max(1, Math.round(hz / Math.min(10, Math.ceil(ts)))), hRes = ts >= 10 ? Math.min(az, 180) : az;
    const C = this.lidarCfg;
    if (C.rateHz !== rate || C.hRes !== hRes) { C.rateHz = rate; C.hRes = hRes; this._buildDirs(); this._polT = now; }
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
      // low on the aft frame so the rover camera keeps it in view down to CASS acquisition (box20 sits at aft keel frame)
      const aftX = T.shape === 'cylinder' ? -T.length / 2 - 0.01 : (T.adapterX || 0) - (T.adapterLen || T.length) / 2 - 0.01;
      const aftY = T.shape === 'cylinder' ? -h * 0.62 : -(T.adapterR || h) * 0.62;
      this.targets.push({ id: `${c.id}/AFT`, obj: c, kind: 'cargo', local: new THREE.Vector3(aftX, aftY, 0), normal: new THREE.Vector3(-1, 0, 0), size: 0.8 });
      this.targets.push({ id: `${c.id}/FWD`, obj: c, kind: 'cargo', local: new THREE.Vector3(T.length / 2 + 0.01, h * 0.62, 0), normal: new THREE.Vector3(1, 0, 0), size: 0.6 });
      for (const sz of [-1, 1]) this.targets.push({ id: `${c.id}/${sz < 0 ? 'L' : 'R'}`, obj: c, kind: 'cargo', local: new THREE.Vector3(-1.2, -0.3, sz * ((T.shape === 'cylinder' ? T.radius : T.width / 2) + 0.035)), normal: new THREE.Vector3(0, 0, sz), size: 0.8 });
    }
    for (const p of this.sim.ports) {
      const w = new THREE.Vector3(p.x + p.nx * 0.02, p.y - 1.5, p.z + p.nz * 0.02);
      this.targets.push({ id: `${p.name}`, obj: p, kind: 'port', world: w, normal: new THREE.Vector3(p.nx, 0, p.nz), size: 0.55, known: { x: w.x, y: w.y, z: w.z, nx: p.nx, nz: p.nz } });
    }
    // surveyed beacons on the lander's 4 sides, on the descent-stage wall. Same LANDER_SCALE as the lander's
    // colliders and model: unscaled, the tags sat 7 m inside the descent-stage collider and the camera's
    // line-of-sight ray always hit the hull first, so they were never seen.
    const LS = LANDER_SCALE, lx = this.sim.landerPos.x, lz = this.sim.landerPos.z, ly = this.sim.terrain.padH + LANDER_TAG.y * LS;
    for (let k = 0; k < 4; k++) {
      const a = k * Math.PI / 2, nx = Math.cos(a), nz = Math.sin(a);
      const w = new THREE.Vector3(lx + nx * LANDER_TAG.r * LS, ly, lz + nz * LANDER_TAG.r * LS);
      this.targets.push({ id: `LANDER-${'ENWS'[k]}`, obj: { id: 'lander' }, kind: 'beacon', world: w, normal: new THREE.Vector3(nx, 0, nz), size: LANDER_TAG.size * LS, known: { x: w.x, y: w.y, z: w.z, nx, nz } });
    }
    // surveyed beacon on the power-dock battery vault, facing the charge pad (1.0 m tag, ASSUMED): bounds the
    // SLAM error at the dock so the rover can park on the coupler from the map alone
    { const H = this.sim.terrain.hab, L = powerDockLayout(H), b = L.bank, nx = H.nx, nz = H.nz;
      const w = new THREE.Vector3(b.x + nx * (b.d / 2 + 0.02), heightAt(this.sim.terrain, b.x, b.z) + b.h * 0.6, b.z + nz * (b.d / 2 + 0.02));
      this.targets.push({ id: 'POWER-DOCK', obj: { id: 'powerdock' }, kind: 'beacon', world: w, normal: new THREE.Vector3(nx, 0, nz), size: 1.0, known: { x: w.x, y: w.y, z: w.z, nx, nz } }); }
    // surveyed beacon on the habitat supply building facade (1.2 m tag, ASSUMED): bounds
    // SLAM drift when approaching the habitat from the landing pad traverse
    { const H = this.sim.terrain.hab, f = H.face, nx = H.nx, nz = H.nz;
      const w = new THREE.Vector3(f.x + nx * 0.05, heightAt(this.sim.terrain, f.x, f.z) + 7.5, f.z + nz * 0.05);
      this.targets.push({ id: 'HAB-FACE', obj: { id: 'hab' }, kind: 'beacon', world: w, normal: new THREE.Vector3(nx, 0, nz), size: 1.2, known: { x: w.x, y: w.y, z: w.z, nx, nz } }); }
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
    const side = [{ ux: 0, n: 0, T: 0, N: 0, nc: 0 }, { ux: 0, n: 0, T: 0, N: 0, nc: 0 }];
    for (const b of R.bogies) {
      const S = side[b.z < 0 ? 0 : 1];
      let vsum = 0, k = 0;
      for (const w of b.wheels) {
        const omPrev = this._omPrev.get(w) ?? w.omega;
        this._omPrev.set(w, w.omega);
        if (w.contact && w.load > 50) {
          const Tw = w.torque - (dt > 0 ? ROVER.wheel.inertia * (w.omega - omPrev) / dt : 0);
          Tnet += Tw; Nsum += w.load; nc++;
          S.T += Tw; S.N += w.load; S.nc++;
          vsum += w.omega * ROVER.wheel.radius; k++;
        }
      }
      if (!k) continue;
      const s = (vsum / k) * (1 + 0.004 * gauss());
      const st = b.steer + 0.002 * gauss();
      const bx = s * Math.cos(st);
      ux += bx; uz += -s * Math.sin(st); n++;
      S.ux += bx; S.n++;
    }
    if (!n) { ux = 0; uz = 0; } else { ux /= n; uz /= n; }
    const vL = side[0].n ? side[0].ux / side[0].n : 0, vR = side[1].n ? side[1].ux / side[1].n : 0;
    // In sharp turns and in-place pivots the left and right sides carry opposing or very unequal torques, so
    // vehicle-mean Tnet cancels inner/outer slip (leaving +8% turn error and 0 velocity during slope pivots).
    // Invert slip per side whenever the tracks split significantly in speed or torque sign.
    const diffTurn = Math.abs(vL - vR) > 0.25 || (side[0].T * side[1].T < 0 && Math.abs(ux) > 0.15) || (Math.abs(vL - vR) > 0.05 && Math.abs(ux) < 0.2);
    if (diffTurn && side[0].nc && side[1].nc) {
      let uxS = 0;
      for (const S of side) {
        const vS = S.ux / S.n, Ts = S.T / S.nc, Ns = S.N / S.nc;
        const sEst = Math.abs(this.slipFromTorque(Ts, Ns));
        uxS += Ts * vS >= 0 ? vS * (1 - sEst) : vS / Math.max(0.5, 1 - sEst);
      }
      ux = 0.5 * uxS;
    } else if (nc) {
      const sEst = this.slipFromTorque(Tnet / nc, Nsum / nc);
      // Driving (Tnet and ux same sign): v_ground = v_wheel * (1 - |sEst|). Braking: cap denominator against transient load drops over craters.
      const driving = Tnet * ux >= 0;
      const f = driving ? 1 - Math.abs(sEst) : 1 / Math.max(0.88, 1 - Math.abs(sEst));
      ux *= f; uz *= f;
    }
    const cp = Math.cos(this.imu.pitch);
    const lvlAvg = R.bogies.reduce((a, b) => a + b.level, 0) / R.bogies.length;
    const dLvl = this._prevLvl == null ? 0 : lvlAvg - this._prevLvl;
    this._prevLvl = lvlAvg;
    this.odom.vx = ux; this.odom.vz = uz;
    this.odom.dx += ux * cp * dt; this.odom.dz += uz * dt; this.odom.dy += ux * Math.sin(this.imu.pitch) * dt + (Math.abs(dLvl) < 0.05 ? dLvl : 0); {
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
    // stereo depth: ROVER.camera.stereoRateHz in sim time at 1x, stretched with the time scale so the wall-clock cost stays flat
    this._stT = (this._stT ?? 0) - dt;
    if (this._stT <= 0) {
      this._stT = Math.max(1, this.sim.timeScale) / ROVER.camera.stereoRateHz;
      if (ROVER.camera.stereo && this.cameraOn && !this.faults.camera && R.powered && this.sim.thermal?.ok.camera !== false) this._doStereo(); else this.stereo = null;
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
    // interlaced firing (ROVER.lidar.interlace): even beams on one sweep, odd beams on the next
    this._ilv = (this._ilv ?? 1) ^ 1;
    this._partial = { sectors, sector: 0, world: new Float32Array(N * 3), kind: new Uint8Array(N), m: 0, c0: ROVER.lidar.interlace ? this._ilv : 0, cs: ROVER.lidar.interlace ? 2 : 1 };
  }
  _scanSector(p) {
    const L = ROVER.lidar, P = this._partial;
    const o = new THREE.Vector3(L.mountLocal.x, L.mountLocal.y, L.mountLocal.z * (this.lidarUnit === 'B' ? -1 : 1)).applyQuaternion(p.q).add(new THREE.Vector3(p.x, p.y, p.z));
    const { channels, hRes, range } = this.lidarCfg;
    const h0 = Math.floor(P.sector * hRes / P.sectors), h1 = Math.floor((P.sector + 1) * hRes / P.sectors);
    const d = new THREE.Vector3(), q = p.q, dirs = this._dirs, colliderRock = this.sim.colliderRock;
    // (the scan rate and azimuth steps follow the time scale in _lidarPolicy; no per-ray decimation here any more)
    this.decimated = false;
    const mask = this._selfMask(p);
    for (let c = P.c0; c < channels; c += P.cs) for (let h = h0; h < h1; h++) {
      const i = c * hRes + h;
      d.set(dirs[i * 3], dirs[i * 3 + 1], dirs[i * 3 + 2]).applyQuaternion(q);
      const hit = this._castRay(o, d, range);
      if (!hit) continue;
      const t = hit.timeOfImpact + L.noise * gaussRange();
      if (t < 1.0) continue;
      if (mask && mask(o.x + d.x * t, o.y + d.y * t, o.z + d.z * t)) continue; // the vehicle's own chassis / payload
      const m = P.m++;
      P.world[m * 3] = o.x + d.x * t; P.world[m * 3 + 1] = o.y + d.y * t; P.world[m * 3 + 2] = o.z + d.z * t;
      const hc = hit.collider.handle;
      const owner = this.sim.colliderOwner.get(hc);
      P.kind[m] = hc === this.sim.terrainCollider.handle ? 0 : colliderRock.has(hc) ? 1 : owner && owner.kind === 'cargo' ? 3 : 2;
    }
    P.sector++;
  }
  // Dynamic self-occlusion mask: a return that lands on the chassis box or on the carried module's cylinder is the vehicle seeing
  // itself, not an obstacle, and must never be mapped (the payload would become a wall 4 m from the rover). The ray queries
  // already skip the rover's and the held cargo's colliders (collision groups, world.js Q_SENSOR / G_CARGO_HELD); this test on the
  // hit point is the explicit second line and also covers the cargo's enclosing cylinder (ring-adapter sled, module overhang),
  // which has no collider of its own. Known simplification, as before v1.7.0: the rays pass THROUGH the vehicle instead of being
  // blocked by it, so the one modelled head sees all round; on the real vehicle the second corner head covers what the first cannot.
  // Returns a (wx, wy, wz) => boolean test in world coordinates, or null when nothing needs masking.
  _selfMask(p) {
    const R = this.sim.rover, F = ROVER.frame, qi = p.q.clone().invert(), v = new THREE.Vector3();
    const c = R.loaded ? R.lift.cargo : null, T = c && c.T, b = c && R.lift.base;
    const cylR = T ? Math.max(T.adapterR || 0, T.shape === 'cylinder' ? T.radius : Math.hypot(T.height / 2, T.width / 2)) + 0.3 : 0;
    const cylL = T ? Math.max(T.length / 2 + Math.abs(T.carryX || 0), (T.adapterLen || 0) / 2 + Math.abs(T.adapterX || 0)) + 0.3 : 0;
    return (wx, wy, wz) => {
      v.set(wx - p.x, wy - p.y, wz - p.z).applyQuaternion(qi);   // body frame
      if (Math.abs(v.x) < F.halfLen && Math.abs(v.z) < F.halfWid && v.y > F.tubY0 && v.y < F.deckY + 0.3) return true; // chassis box (belly tub to deck)
      if (T) { const dy = v.y - b.y, dz = v.z - b.z; if (Math.abs(v.x - b.x) < cylL && dy * dy + dz * dz < cylR * cylR) return true; }
      return false;
    };
  }

  // Stereo depth (ROVER.camera.stereo): a coarse dense depth grid across the camera's field of view. Depth noise follows
  // the disparity model sigma_z = z^2 sigma_d / (f B). Returns are expressed in the body frame like a LiDAR scan and fused by
  // Slam.integrate alongside the LiDAR hits; points whose depth sigma exceeds ROVER.camera.fuseSigma are not fused (see there).
  _doStereo() {
    const R = this.sim.rover, C = ROVER.camera, cam = this.cameraPose(), p = R.pose();
    const [nu, nv] = C.stereoGrid, hf = C.hFovDeg * Math.PI / 360, tu = Math.tan(hf), tv = tu * 0.75;
    const f = (C.imagePx / 2) / tu, zMin = f * C.baselineM / 64;            // 64 px disparity search window
    const body = new Float32Array(nu * nv * 3), sigma = new Float32Array(nu * nv), qi = p.q.clone().invert(), d = new THREE.Vector3(), w = new THREE.Vector3();
    let m = 0;
    for (let iv = 0; iv < nv; iv++) for (let iu = 0; iu < nu; iu++) {
      const u = ((iu + 0.5) / nu * 2 - 1) * tu, vv = ((iv + 0.5) / nv * 2 - 1) * tv;
      d.copy(cam.fwd).addScaledVector(cam.right, u).addScaledVector(cam.up, -vv).normalize();
      const cosA = d.dot(cam.fwd), hit = this._castRay(cam.pos, d, C.depthRange / cosA);
      if (!hit) continue;
      const z = hit.timeOfImpact * cosA;                                    // depth along the optical axis
      if (z < zMin) continue;
      const sig = z * z * C.disparityNoise / (f * C.baselineM), t = hit.timeOfImpact + gaussRange() * sig / cosA;
      w.copy(cam.pos).addScaledVector(d, t);
      w.x -= p.x; w.y -= p.y; w.z -= p.z; w.applyQuaternion(qi);
      body[m * 3] = w.x; body[m * 3 + 1] = w.y; body[m * 3 + 2] = w.z; sigma[m] = sig; m++;
    }
    this.stereo = { t: this.sim.time, n: m, fused: 0 };
    if (m) this.stereo.fused = this.sim.slam.integrate({ n: m, body: body.subarray(0, m * 3), sigma: sigma.subarray(0, m), roll: this.imu.roll, pitch: this.imu.pitch }, C.fuseSigma);
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
    if (!this.lidarOn || this.faults.lidar || !R.powered || this.sim.thermal?.ok.lidar === false) return;
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
      // Line-of-sight occlusion raycast (do not detect boulders hidden behind terrain rims, walls, or other obstacles)
      const dir = v.clone().divideScalar(range);
      const hit = this._castRay(cam.pos, dir, range);
      if (hit && this.sim.colliderRock.get(hit.collider.handle) !== i && hit.timeOfImpact < range - r.d * 0.75) continue;
      const ang = Math.atan2(r.d * 0.6, range);
      const rangeMeas = Math.max(0.2, range + 0.015 * range * gauss());
      const dMeas = Math.max(0.15, r.d * (1 + 0.06 * gauss()));
      rocks.push({ u: 0.5 + Math.tan(ax) / Math.tan(hf) * 0.5, v: 0.5 - Math.tan(ay) / Math.tan(vf) * 0.5, w: Math.tan(ang) / Math.tan(hf), h: Math.tan(ang * r.sy) / Math.tan(vf), d: dMeas, range: rangeMeas, conf: Math.min(0.99, 0.55 + r.d / range * 9) });
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
      // band-edge sensors: inductive linear proximity sweep (+-1.2 m along saddle X) locating each clamp band
      // against its saddle via upward proximity rays + hull collar geometry
      const c = offs[0].target, tr = c.body.rotation(), cq = new THREE.Quaternion(tr.x, tr.y, tr.z, tr.w);
      const t = c.body.translation();
      const qinv = p.q.clone().invert();
      const bands = c.T.adapterRings || [-ROVER.ringX, ROVER.ringX];
      const saddles = [-ROVER.ringX, ROVER.ringX];
      let bandSum = 0, bandN = 0;
      for (let i = 0; i < 2; i++) {
        o.set(saddles[i], F.deckY + 0.05, 0).applyQuaternion(p.q).add(P);
        d.set(0, 1, 0).applyQuaternion(p.q);
        const midHit = this._castRay(o, d, 12);
        const midOwn = midHit && this.sim.colliderOwner.get(midHit.collider.handle);
        if (midOwn && midOwn.ref === c) {
          const bWorld = new THREE.Vector3(bands[i], 0, 0).applyQuaternion(cq).add(new THREE.Vector3(t.x, t.y, t.z));
          const bRel = bWorld.sub(P).applyQuaternion(qinv);
          const dxSaddle = bRel.x - saddles[i];
          if (Math.abs(dxSaddle) < 1.2) { bandSum += dxSaddle; bandN++; }
        }
      }
      if (bandN > 0) res.cargoX = (c.T.carryX || 0) + (bandSum / bandN) + 0.02 * gauss();
      res.seatGap = Math.min(...res.lat.filter((l) => l.d != null).map((l) => l.d)) - (ROVER.seatY - F.deckY - 0.05);
    }
    const L = R.lift;
    if (L.cargo) {
      const t = L.cargo.body.translation(), r = L.cargo.body.rotation();
      const q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
      // dock gap sensor: at the held module's forward (hatch) end, straight ahead
      o.set(L.cargo.T.length / 2, 0, 0).applyQuaternion(q).add(new THREE.Vector3(t.x, t.y, t.z));
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
