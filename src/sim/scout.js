// Mission 9: SCOUT-1, a 3x scale-up of the team's 3D-printed tracked rover (tracked_lunar_rover.obj).
// It drives ahead of AEGIS-V1 on the planned route, maps terrain with its tilted 2D LiDAR into the
// shared SLAM map, keeps separation from the big rover and reports status over the local network.
import * as THREE from 'three';
import { MOON } from './config.js';
import { Q_SENSOR } from './world.js';
import { gauss } from './sensors.js';

export const SCOUT = {
  scale: 3, length: 1.05, width: 0.88, height: 0.8, mass: 24,
  vMax: 3.6, stepMax: 0.13, slopeMax: 25 * Math.PI / 180, // vMax: 3x prototype 0.7-0.9 m/s (speed scales with track size)
  lidarRange: 14, lidarH: 0.72, lead: 70, separation: 22,
};

export class Scout {
  constructor(sim) {
    this.sim = sim;
    this.rate = 20;
    const r = sim.rover.pose();
    this.x = r.x - r.fwd.x * 12 + r.fwd.z * 9; this.z = r.z - r.fwd.z * 12 - r.fwd.x * 9; this.yaw = r.yaw;
    this.y = 0; this.pitch = 0; this.roll = 0; this.v = 0; this.w = 0;
    this.enabled = false; this.mode = 'parked'; this.msg = 'parked beside AEGIS-V1';
    this.soc = 1.0; this.cellsMapped = 0; this.odo = 0; this.trackPhase = 0;
    this.ray = new sim.RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 0, y: -1, z: 0 });
    this._msgT = 0; this._scanT = 0;
    this.lastScan = null;
    this._ground();
  }

  groundAt(x, z) {
    const r = this.ray; r.origin.x = x; r.origin.y = 4000; r.origin.z = z; r.dir.x = 0; r.dir.y = -1; r.dir.z = 0;
    const hit = this.sim.world.castRay(r, 8000, true, undefined, Q_SENSOR, undefined, this.sim.rover.body);
    return hit ? 4000 - hit.timeOfImpact : null;
  }

  _ground() {
    // four track-corner probes give height, pitch and roll (the tracks conform to the ground)
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw), hl = SCOUT.length / 2, hw = SCOUT.width / 2;
    const p = (a, b) => this.groundAt(this.x + a * c + b * s, this.z - a * s + b * c);
    const fl = p(hl, -hw), fr = p(hl, hw), rl = p(-hl, -hw), rr = p(-hl, hw);
    if ([fl, fr, rl, rr].some((v) => v == null)) return;
    this.y = (fl + fr + rl + rr) / 4;
    this.pitch = Math.atan2((fl + fr) / 2 - (rl + rr) / 2, SCOUT.length);
    this.roll = Math.atan2((fr + rr) / 2 - (fl + rl) / 2, SCOUT.width);
  }

  // forward probes: is the ground ahead along heading h drivable for a 1 m tracked robot?
  clearAlong(h, dist) {
    const c = Math.cos(h), s = Math.sin(h);
    let prev = this.y;
    for (let d = 0.6; d <= dist; d += 0.4) {
      for (const lat of [-0.45, 0, 0.45]) {
        const x = this.x + c * d + s * lat, z = this.z - s * d + c * lat;
        const g = this.groundAt(x, z);
        if (g == null || Math.abs(g - prev) > SCOUT.stepMax + 0.4 * 0.45) return d - 0.6;
      }
      prev = this.groundAt(this.x + c * d, this.z - s * d) ?? prev;
    }
    // keep clear of AEGIS-V1's footprint
    const R = this.sim.rover.pose();
    const ax = this.x + c * dist, az = this.z - s * dist;
    if (Math.hypot(ax - R.x, az - R.z) < 9.5) return 0;
    return dist;
  }

  target() {
    const A = this.sim.autonomy, R = this.sim.rover.pose();
    if (!A.path || !A.gen || A.path.length < 2) return null;
    // the route is in SLAM coordinates; convert from SLAM est to world using both translation and heading offset
    const e = this.sim.slam.est;
    const wyaw = R.yaw - e.yaw, cw = Math.cos(wyaw), sw = Math.sin(wyaw);
    const i0 = A.pathIdx || 0;
    let acc = 0, i = i0;
    while (i < A.path.length - 1 && acc < SCOUT.lead) { acc += Math.hypot(A.path[i + 1].x - A.path[i].x, A.path[i + 1].z - A.path[i].z); i++; }
    const p = A.path[i];
    const dx = p.x - e.x, dz = p.z - e.z;
    return { x: R.x + dx * cw + dz * sw, z: R.z - dx * sw + dz * cw, ahead: acc };
  }

  update(dt) {
    const sim = this.sim, R = sim.rover.pose();
    if (!this.enabled) { this.v = 0; this.w = 0; this.mode = 'parked'; this._ground(); return; }
    let tgt = null;
    if (this.mode === 'recall') {
      tgt = { x: R.x - R.fwd.x * 14 + R.fwd.z * 8, z: R.z - R.fwd.z * 14 - R.fwd.x * 8 };
      this.msg = Math.hypot(tgt.x - this.x, tgt.z - this.z) < 2 ? 'docked beside AEGIS-V1 (recalled)' : 'returning to AEGIS-V1';   // stay in recall until redeployed
    } else {
      tgt = this.target();
      if (!tgt) { this.mode = 'escort'; tgt = { x: R.x + R.fwd.x * 25 + R.fwd.z * 10, z: R.z + R.fwd.z * 25 - R.fwd.x * 10 }; }
      else this.mode = 'scouting';
    }
    let dR = Math.hypot(this.x - R.x, this.z - R.z);
    // overtaking: while close to AEGIS-V1 and not yet ahead of it, run on a parallel line 14 m to
    // the side (whichever side the scout is on), then merge back onto the route ahead
    if (this.mode === 'scouting') {
      const ahead = (this.x - R.x) * R.fwd.x + (this.z - R.z) * R.fwd.z;
      if (ahead < 18 && dR < 45) {
        const side = ((this.x - R.x) * R.fwd.z - (this.z - R.z) * R.fwd.x) >= 0 ? 1 : -1; // +: left
        tgt = { ...tgt, x: R.x + R.fwd.x * 30 + R.fwd.z * 14 * side, z: R.z + R.fwd.z * 30 - R.fwd.x * 14 * side, ahead: tgt.ahead };
        this.msg = 'overtaking AEGIS-V1 on a parallel track';
        this.overtaking = true;
      } else this.overtaking = false;
    }
    const dT = Math.hypot(tgt.x - this.x, tgt.z - this.z);
    // the targets move with the convoy, so add AEGIS-V1's own speed to the closing term
    const vR = Math.hypot(sim.rover.body.linvel().x, sim.rover.body.linvel().z);
    let vDes = this.overtaking ? SCOUT.vMax : Math.min(SCOUT.vMax, vR + 0.2 + dT * 0.06);
    if (dT < 1.5) vDes = 0;
    if (this.soc < 0.1) { vDes = 0; this.msg = 'battery low: holding position'; }
    // heading choice: closest-to-target clear heading (local VFH-style search)
    const want = Math.atan2(-(tgt.z - this.z), tgt.x - this.x);
    let best = null;
    for (const dh of [0, 0.25, -0.25, 0.5, -0.5, 0.8, -0.8, 1.2, -1.2, 1.7, -1.7, 2.4, -2.4, 3.1]) {
      const h = want + dh;
      const free = this.clearAlong(h, 3.2);
      if (free >= 3.0) { best = h; break; }
    }
    if (best == null) { vDes = 0; this.w = 0.8; this.msg = 'boxed in: rotating to search for a gap'; }
    else {
      let e = best - this.yaw; e = Math.atan2(Math.sin(e), Math.cos(e));
      this.w = THREE.MathUtils.clamp(e * 1.5, -1.0, 1.0);
      if (Math.abs(e) > 0.8) vDes *= 0.2;
    }
    // separation from AEGIS-V1 (collision avoidance between the two robots)
    if (dR < 12 && this.mode === 'scouting') {
      const away = Math.atan2(-(this.z - R.z), this.x - R.x);
      let e = away - this.yaw; e = Math.atan2(Math.sin(e), Math.cos(e));
      if (Math.abs(e) > 1.3) vDes = Math.min(vDes, 0.3);
    }
    // don't run away from the convoy: hold if far beyond the lead point
    if (this.mode === 'scouting') { const ahead = (this.x - R.x) * R.fwd.x + (this.z - R.z) * R.fwd.z; if (ahead > SCOUT.lead + 45) vDes = 0; }
    // slope & ballistic crest limit (1/6 g: prevent launch on steep craters/bumps)
    if (Math.abs(this.pitch) > SCOUT.slopeMax) vDes = Math.min(vDes, 0.2);
    else if (Math.abs(this.pitch) > 0.14) vDes = Math.min(vDes, 1.8);
    // keep electrical draw accurate; apply cold-soak battery heater load when ambient sun is off
    this.v += THREE.MathUtils.clamp(vDes - this.v, -0.8 * dt, 0.5 * dt);
    this.yaw += this.w * dt;
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    this.x += c * this.v * dt; this.z += -s * this.v * dt;
    this.odo += this.v * dt; this.trackPhase += this.v * dt;
    this._ground();
    // power: brushless sealed gearmotors + LiDAR + compute + survival heater at night, small solar panel
    const heatW = sim.sunDir.y <= 0 ? 4 : 0;
    const p = 18 + heatW + Math.abs(this.v) * (SCOUT.mass * MOON.g * 0.25 + 8) + Math.abs(this.w) * 6;
    this.soc = Math.max(0, this.soc - (p - 9 * Math.max(0, sim.sunDir.y > 0 ? 1 : 0)) * dt / 3600 / 180);
    // mapping
    this._scanT -= dt;
    if (this._scanT <= 0) { this._scanT = 0.2; this._scan(); }
    this._msgT -= dt;
    if (this._msgT <= 0) {
      this._msgT = 15;
      if (this.mode === 'scouting' && tgt) this.msg = `scouting ${tgt.ahead ? tgt.ahead.toFixed(0) + ' m' : ''} ahead on route | mapped ${(this.cellsMapped * 0.25 / 1e4).toFixed(2)} ha | battery ${(this.soc * 100).toFixed(0)}%`;
    }
  }

  // tilted YDLIDAR (2D, 360 deg, ~12 m) sweeping the ground ahead; three tilt states emulate a nodding head
  _scan() {
    const sim = this.sim, slam = sim.slam;
    const R = sim.rover.pose(), e = slam.est;
    // scout is localised relative to AEGIS-V1 (UWB ranging + odometry): place its points in the SLAM frame
    const dyaw = e.yaw - R.yaw, cy = Math.cos(dyaw), sy = Math.sin(dyaw);
    const toEst = (x, z) => { const rx = x - R.x, rz = z - R.z; return { x: e.x + rx * cy + rz * sy, z: e.z - rx * sy + rz * cy }; };
    const o = { x: this.x, y: this.y + SCOUT.lidarH, z: this.z };
    const pts = [];
    const tilts = [-0.16, -0.28, -0.45];
    for (const tilt of tilts) for (let k = 0; k < 72; k++) {
      const az = this.yaw + (k / 72) * Math.PI * 2;
      const d = { x: Math.cos(tilt) * Math.cos(az), y: Math.sin(tilt), z: -Math.cos(tilt) * Math.sin(az) };
      const r = this.ray; r.origin.x = o.x; r.origin.y = o.y; r.origin.z = o.z; r.dir.x = d.x; r.dir.y = d.y; r.dir.z = d.z;
      const hit = sim.world.castRay(r, SCOUT.lidarRange, true, undefined, Q_SENSOR, undefined, sim.rover.body);
      if (!hit) continue;
      const t = hit.timeOfImpact + 0.015 * gauss();
      const wx = o.x + d.x * t, wy = o.y + d.y * t, wz = o.z + d.z * t;
      pts.push(wx, wy, wz);
      const pe = toEst(wx, wz);
      const before = slam.cellsKnown;
      slam.insert(pe.x, wy - R.y + e.y, pe.z);
      if (slam.cellsKnown > before) this.cellsMapped++;
    }
    slam.version++;
    this.lastScan = pts;
  }

  command(c) {
    if (c === 'on' || c === 'deploy') { this.enabled = true; this.mode = 'escort'; this.msg = 'deployed: will scout ahead whenever a route is active'; }
    else if (c === 'off' || c === 'park') { this.enabled = false; this.msg = 'parked'; }
    else if (c === 'recall') { this.enabled = true; this.mode = 'recall'; this.msg = 'recalled to AEGIS-V1'; }
    else return `unknown scout command "${c}" (use on | off | recall | status)`;
    return this.msg;
  }
}
