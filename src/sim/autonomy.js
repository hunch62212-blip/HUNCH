// Mission executive. Behaviours are generators: each tick the executive advances the active
// generator one step, so mission scripts read like the procedure they implement.
// All decisions use estimated (SLAM) state and sensor measurements, never ground truth.
import * as THREE from 'three';
import { ROVER, WORLD, DOCK, MOON, POWER_DOCK, MISSION, NAV } from './config.js';
import { Planner, LocalPlanner, LETHAL } from './planner.js';
import { OrbitalDEM, matchToDEM } from './orbitalMap.js';
const LETHAL_HALF = LETHAL * 0.5; // planner hard-inflation cost (obstacle margin)

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const DEG = Math.PI / 180;

export const PRIORITY = { 'life-support': 5, 'emergency-repair': 5, 'power': 4, 'habitat-supplies': 3, 'science': 2 };

export class Autonomy {
  constructor(sim, sensors, slam) {
    this.sim = sim; this.sensors = sensors; this.slam = slam;
    this.rate = 10;
    this.planner = new Planner(sim, slam);
    this.local = new LocalPlanner(slam);
    // surveyed thin posts and the vehicle's swept rectangle for the local planner's post check (planner.js LocalPlanner)
    { let posts = null; // built on first use: the power dock is created after the autonomy
      const build = () => {
        const out = [];
        for (const it of sim.terrain?.base || []) if (it.kind === 'mast' || it.kind === 'light' || it.kind === 'array') out.push({ x: it.x, z: it.z, r: it.kind === 'array' ? 0.2 : (it.r || 0.2) });
        for (const a of sim.powerDock?.layout?.arrays || []) out.push({ x: a.x, z: a.z, r: 0.45 });
        return out;
      };
      this.local.posts = () => (posts ??= build());
      this.local.extent = () => {
        const R = sim.rover, F = ROVER.frame, cg = R && R.loaded && R.lift.cargo;
        if (!cg) return { hl: F.halfLen, hw: F.halfWid };
        const T = cg.T, half = Math.max(T.adapterR || 0, T.shape === 'cylinder' ? T.radius : T.width / 2);
        return { hl: Math.max(F.halfLen, T.length / 2 + Math.abs(T.carryX || 0)), hw: Math.max(F.halfWid, half) };
      }; }
    this.dem = sim.terrain ? new OrbitalDEM(sim.terrain) : null; // orbital elevation model for map-relative fixes (orbitalMap.js)
    this.mrl = { on: true, dLast: 0, tLast: -1e9, fixes: 0, last: null };
    this.mode = 'MANUAL';      // MANUAL | AUTO
    this.gen = null; this.missionName = null;
    this.phase = 'Idle'; this.detail = '';
    this.path = null; this.pathIdx = 0; this.pathSource = '';
    this.goal = null;
    this.speedSetting = 99;    // autonomy cruise ceiling (m/s); 99 = automatic (smart-rpm tier decides)
    this.paused = false;
    this.faults = { comms: false, port: {}, seal: false };
    this.reserveSoc = 0.2;
    this.frozenS = 0;
    this.metrics = this._newMetrics();
    this.history = [];
    this.cargoEst = new Map();     // cargo id -> {x,z,y,yaw,t,src}
    this.portEst = new Map();
    this.manifest = sim.cargo.map((c, i) => ({ id: c.id, cargo: c, port: c.dest, category: c.category || (i === 0 ? 'life-support' : 'habitat-supplies'), priority: i === 0 ? 5 : 3, status: 'waiting' }));
    this.docking = null;
    this.lastTick = 0;
  }

  _newMetrics() { return { start: this.sim.time, dist0: this.sim.rover.odo.dist, e0: this.sim.rover.power.energyUsedWh, deliveries: 0, accuracy: [], legs: [] }; }
  log(m, l) { this.sim.log(m, l); }

  // ---------------- state helpers ----------------
  est() { const e = this.slam.est; return { x: e.x, z: e.z, y: e.y, yaw: e.yaw }; }
  cmd(vx, vz = 0, wz = 0) { const c = this.sim.rover.cmd; c.vx = vx; c.vz = vz; c.wz = wz; }
  stop() { this.cmd(0, 0, 0); }
  // Yaw-rate loop for precision legs: a loaded skid-steer rover scrubs, so a small open-loop yaw-rate command
  // gives almost no turn (docking trace: kap -0.02 commanded, -0.002 achieved) and large ones lag by metres.
  // PI on the gyro yaw-rate error makes the achieved curvature follow the commanded one.
  cmdCurv(v, kap, reset = false) {
    if (reset || this._wzI == null) this._wzI = 0;
    // authority scales with speed (<= 0.04 /m of extra curvature): at a creep the module's sway dominates the
    // gyro and an unscaled integrator wound up and steered the nose 4 cm off in the last metre
    const wDes = v * kap, err = wDes - this.sensors.imu.yawRate, lim = 0.04 * Math.abs(v) + 0.001;
    this._wzI = clamp(this._wzI + 1.2 * err * (this.dt || 0.1), -lim, lim);
    this.cmd(v, 0, wDes + clamp(2.0 * err + this._wzI, -lim, lim));
  }
  relBody(p, e = this.est()) { // world(est) point -> body (fwd, right)
    const dx = p.x - e.x, dz = p.z - e.z, c = Math.cos(e.yaw), s = Math.sin(e.yaw);
    return { f: dx * c - dz * s, r: dx * s + dz * c };
  }
  site(x, z) {
    const P = WORLD.pad, H = WORLD.habitat, S = this.sim.terrain.pad.staging;
    if (Math.hypot(x - S.x, z - S.z) < 60) return 'STAGING';
    if (Math.hypot(x - P.x, z - P.z) < 160) return 'PAD';
    if (Math.hypot(x - H.x, z - H.z) < 160) return 'HAB';
    return `WP(${Math.round(x)},${Math.round(z)})`;
  }
  // convert a camera detection (true-frame measurement) into the rover's estimated map frame
  toEst(worldPt) {
    const R = this.sim.rover.pose(), e = this.est();
    const rel = new THREE.Vector3(worldPt.x - R.x, worldPt.y - R.y, worldPt.z - R.z).applyQuaternion(R.q.clone().invert());
    const q = this.slam.attitudeQuat(e.yaw, this.sensors.imu.roll, this.sensors.imu.pitch);
    rel.applyQuaternion(q);
    return { x: e.x + rel.x, y: e.y + rel.y, z: e.z + rel.z };
  }
  dirToEst(worldDir) {
    const R = this.sim.rover.pose(), e = this.est();
    const yawT = Math.atan2(-worldDir.z, worldDir.x);
    return wrap(yawT - R.yaw + e.yaw);
  }

  // surveyed fiducials (lander, habitat ports) have known positions in the base frame:
  // each sighting pulls the SLAM estimate toward the surveyed truth (absolute localisation)
  // Surveyed beacons initialise the map frame. Once a map exists, snapping the estimate to the
  // survey would make it inconsistent with everything already mapped (and with remembered routes),
  // so afterwards the rover stays in its self-consistent SLAM frame (no pose-graph back-end here).
  beaconFix(d) {
    if (!d.known) return;
    if (this.slam.cellsKnown > 30000) { this.mapFix(d); return; }
    if (d.range > 45) return;
    const m = this.toEst(d.pos);
    const e = this.slam.est;
    const w = 0.12 * d.confidence;
    e.x += (d.known.x - m.x) * w; e.z += (d.known.z - m.z) * w;
    const nYaw = this.dirToEst(d.normal) + d.yawNoise;
    const dy = wrap(Math.atan2(-d.known.nz, d.known.nx) - nYaw);
    if (d.range < 30) e.yaw += dy * w * 0.5;
    this.lastBeacon = { id: d.id, t: this.sim.time, range: d.range };
  }

  // Once a map exists, fiducial sightings are averaged (weight 1/sigma^2, sigma from the camera's
  // published fiducial accuracy) and applied as one correction that moves the map, routes and plan with
  // the estimate (Slam.shiftFrame), so everything stays in one consistent frame. CHANGES.md §8.
  mapFix(d) {
    const now = this.sim.time;
    this.lastBeacon = { id: d.id, t: now, range: d.range };
    if (d.range > (d.size >= 1.0 ? 95 : 60)) return;
    // never move the frame under a precision manoeuvre: the pickup / docking scene is changing (lcSuspended),
    // and inside 25 m of a port the final approach is closing a 6 cm cone on the port estimate. A frame shift
    // there moved the target by 0.5-1.7 m mid-approach and every soft capture attempt failed.
    if (this.slam.lcSuspended || /Final approach|Dock /.test(this.phase)) { this._fixes = null; return; }
    if (/pre-dock/.test(this.phase)) { const e = this.est(); for (const p of this.sim.ports) { const g = this.portGuess(p); if (Math.hypot(g.x - e.x, g.z - e.z) < 25) { this._fixes = null; return; } } }
    const m = this.toEst(d.pos);
    const sigma = 0.004 + 0.0009 * d.range * d.range / d.size;
    const w = 1 / (sigma * sigma);
    // one accumulator per fiducial: a mean over different tags mixes their survey / mounting errors
    const FX = this._fixes || (this._fixes = new Map());
    let F = FX.get(d.id);
    if (!F || now - F.tLast > 3) { F = { tLast: now, sx: 0, sz: 0, sdy: 0, wy: 0, nyY: 0, w: 0, n: 0, sy: 0, syy: 0, ny: 0 }; FX.set(d.id, F); }
    F.sx += (d.known.x - m.x) * w; F.sz += (d.known.z - m.z) * w;
    // Vertical camera error grows sharply at long range (e.g. 88 m rays swing >2 m); restrict Y fixes to <= 45 m
    if (d.known.y != null && d.range <= 45) { F.sdy += (d.known.y - m.y) * w; F.wy += w; F.nyY++; }
    F.w += w; F.n++; F.tLast = now;
    // heading error from the tag's surveyed normal (CHANGES.md §17: the pose graph spreads it back along the drive)
    if (d.range < (d.size >= 1.0 ? 80 : 45) && d.known.nx != null) { const dy = wrap(Math.atan2(-d.known.nz, d.known.nx) - (this.dirToEst(d.normal) + d.yawNoise)); F.sy += dy; F.syy += dy * dy; F.ny++; }
    const dx = F.sx / F.w, dz = F.sz / F.w, off = Math.hypot(dx, dz);
    const dyH = F.nyY >= 12 ? F.sdy / F.wy : 0, seY = F.nyY >= 12 ? 1 / Math.sqrt(F.wy) : 0;
    // accept only a well-averaged mean: >= 15 sightings of this tag and a standard error under
    // max(0.3 m, 3 % of the offset). A large drift is fixed coarsely first and later sightings refine it.
    // The offset must also be significant: more than 3 standard errors (and 0.5 m). The old test applied any mean over
    // 0.5 m once the standard error reached 0.3 m; with a true offset of zero that happens by chance in ~25 % of the
    // averages (2-D Gaussian, exp(-0.5^2 / (2 x 0.3^2))), so on the habitat apron the map and the estimate were shifted
    // 0.5-1.6 m by camera noise every few seconds (tests/out/dock_cyl40.log). At 3 standard errors it is ~1 %.
    const se = 1 / Math.sqrt(F.w);
    if (F.n < 15 || se > Math.max(0.3, 0.03 * off)) return;
    if (off < Math.max(0.5, 3 * se) && Math.abs(dyH) < Math.max(0.35, 3 * seY)) {
      if (se <= 0.5 / 3) { FX.delete(d.id); this.slam.lastFixCheck = { t: now, id: d.id, off }; } // verified negligible drift
      return;                               // otherwise keep averaging this tag
    }
    this._fixes = null;
    // heading part of the fix: only from a well-averaged mean (standard error < 0.3 deg over >= 15 sightings), at most 4 deg
    let dyaw = 0;
    if (F.ny >= 15) { const mean = F.sy / F.ny, sd = Math.sqrt(Math.max(0, F.syy / F.ny - mean * mean)), se = sd / Math.sqrt(F.ny); if (se < 0.3 * DEG) dyaw = clamp(mean, -4 * DEG, 4 * DEG); }
    const A = this.slam._fixAnchor || { d: 0 };
    const limit = 5 + 0.08 * (this.sensors.odom.dist - A.d);
    if (off > limit) { this.log(`Fiducial ${d.id}: offset ${off.toFixed(1)} m exceeds plausible drift (${limit.toFixed(1)} m), ignored`, 'info'); return; }
    const dyFix = F.nyY >= 12 && Math.abs(dyH) > Math.max(0.26, 1.5 * seY) ? dyH : 0;
    const r = this.applyFrameShift(dx, dz, dyaw, dyFix);
    this.log(`SLAM fiducial fix (${d.id}): position corrected by ${off.toFixed(1)} m${dyFix ? ` / vert ${dyFix.toFixed(2)} m` : ''}${dyaw ? ` / heading ${(dyaw / DEG).toFixed(2)} deg` : ''}, ${r.cells} map cells re-placed (${r.nodes}-node graph, ${r.iters} sweeps)`, 'ok');
  }

  // Move the estimate, the map, routes and all plan state by one correction (pose graph: Slam.shiftFrame), then replan
  applyFrameShift(dx, dz, dyaw = 0, dy = 0) {
    const anchorT = this.slam._fixAnchor?.t ?? -1;
    const r = this.slam.shiftFrame(dx, dz, dyaw, dy);
    this.slam.est.yaw = wrap(this.slam.est.yaw);
    if (this._stuckPosRef) r.apply(this._stuckPosRef);
    // plan state lives in the same frame: move it with the correction at the current pose and replan
    // (remembered-route waypoints from at/before anchorT live in unshifted map tiles and must stay aligned with them)
    for (const p of this.path || []) if (!(p.t <= anchorT)) r.apply(p);
    for (const b of this.planner.blocked) if (!b.fault) r.apply(b);
    // landmark estimates move with the map they were seen in (Slam.shiftFrame warpAt), not rigidly about the current pose
    for (const p of this.portEst.values()) { const dy = r.warpAt(p); if (dy != null) p.nYaw = wrap(p.nYaw + dy); }
    for (const g of this.cargoEst.values()) if (g.src && g.src !== 'manifest') { const dy = r.warpAt(g); if (dy != null) g.yaw = wrap(g.yaw + dy); }
    this._replanNow = true;
    return r;
  }

  // Map-relative localization (orbitalMap.js): every ~40 m, correlate the local SLAM height map with the orbital DEM
  // and correct the drift it finds. Same precision-phase guards as the fiducial fixes.
  orbitalFix() {
    const M = this.mrl, dist = this.sensors.odom.dist;
    if (!M.on || !this.dem || dist - M.dLast < 40) return;
    M.dLast = dist;
    if (this.slam.lcSuspended || /Final approach|Dock |pre-dock|Pick|Clamp|Lift/.test(this.phase)) return;
    const epochT = Math.max(this.slam.lastFix?.t ?? -1e9, this.slam.lastLoop?.t ?? -1e9);
    if (this.sim.time - epochT < 30) return;   // a fiducial fix or loop closure just aligned the frame
    const e = this.est();
    // Skip patches dominated by already-anchored cells (t0 <= _fixAnchor.t): shiftFrame leaves those tiles
    // and remembered routes unmoved, so shifting est to DEM against them breaks local map/route consistency.
    if (this.slam.anchoredFraction(e.x, e.z, 45) > 0.35) { M.buf = []; return; }
    const m = matchToDEM(this.slam, this.dem, e.x, e.z);
    M.last = { t: this.sim.time, ...m };
    if (!m.ok) return;
    // One match is good to ~1.2-1.5 m (DEM noise and 4 m posting), and drift grows ~0.2 m per 40 m, so single matches
    // are averaged like the fiducial sightings: the 3 most recent agreeing matches (spread < 1.8 m) since the last frame
    // shift and within 400 m, applied only when the mean is clearly non-zero (> 1.5 m and > 2.5 standard errors).
    const fixT = Math.max(this.slam.lastFix?.t ?? -1, this.slam.lastLoop?.t ?? -1);
    M.buf = (M.buf || []).filter((q) => q.fixT === fixT && dist - q.d < 400);
    M.buf.push({ dx: m.dx, dz: m.dz, dy: m.dy || 0, d: dist, fixT });
    // the 3 most recent only: drift keeps growing, and older matches describe an older, smaller error
    if (M.buf.length > 3) M.buf = M.buf.slice(-3);
    const B = M.buf, k = B.length;
    if (k < 3) return;
    const mx = B.reduce((a, q) => a + q.dx, 0) / k, mz = B.reduce((a, q) => a + q.dz, 0) / k, my = B.reduce((a, q) => a + q.dy, 0) / k;
    const sd = Math.sqrt(B.reduce((a, q) => a + (q.dx - mx) ** 2 + (q.dz - mz) ** 2, 0) / (k - 1));
    const off = Math.hypot(mx, mz), se = sd / Math.sqrt(k);
    const applyXZ = off >= Math.max(1.5, 2.5 * se);
    const applyY = Math.abs(my) >= 0.4;
    if (sd > 1.8 || (!applyXZ && !applyY)) return;
    const A = this.slam._fixAnchor || { d: 0 };
    if (off > 5 + 0.08 * (dist - A.d)) { M.buf = []; return; }
    const r = this.applyFrameShift(applyXZ ? mx : 0, applyXZ ? mz : 0, 0, applyY ? my : 0);
    M.fixes++; M.tLast = this.sim.time; M.buf = [];
    const desc = [applyXZ ? `position corrected by ${off.toFixed(1)} m` : '', applyY ? `vert ${my.toFixed(2)} m` : ''].filter(Boolean).join(' / ');
    this.log(`SLAM orbital-map fix: local terrain matched the orbital elevation map (${k} agreeing matches, spread ${sd.toFixed(1)} m): ${desc}, ${r.cells} map cells re-placed`, 'ok');
  }

  updateEstimates() {
    for (const d of this.sensors.detections) {
      if (d.kind === 'cargo') {
        const c = d.obj, T = c.T;
        // the module on the rover's own cradle is in view of the cameras for the whole loaded leg: its tags say nothing about
        // where cargo is waiting, and fusing them dragged the pad estimate 2 km along with the rover ("disagreed ... restarting")
        if (c.state === 'held' || c.state === 'docked' || c.state === 'delivered') continue;
        const p = this.toEst(d.pos);
        const nYaw = this.dirToEst(d.normal) + d.yawNoise;
        let axisYaw, center;
        const ax = (y) => ({ x: Math.cos(y), z: -Math.sin(y) });
        const aftLever = T.shape === 'cylinder' ? T.length / 2 : (T.adapterLen || T.length) / 2 - (T.adapterX || 0);
        if (d.id.endsWith('/AFT')) { axisYaw = wrap(nYaw + Math.PI); const a = ax(axisYaw); center = { x: p.x + a.x * aftLever, z: p.z + a.z * aftLever }; }
        else if (d.id.endsWith('/FWD')) { axisYaw = nYaw; const a = ax(axisYaw); center = { x: p.x - a.x * T.length / 2, z: p.z - a.z * T.length / 2 }; }
        else {
          const half = T.shape === 'cylinder' ? T.radius : T.width / 2;
          const n = ax(nYaw); center = { x: p.x - n.x * half, z: p.z - n.z * half };
          axisYaw = wrap(nYaw + (d.id.endsWith('/L') ? -Math.PI / 2 : Math.PI / 2));
          const a = ax(axisYaw); center.x += a.x * 1.2; center.z += a.z * 1.2; // side tags sit 1.2 m aft of centre
        }
        // inverse-variance fusion (a static object: many sightings average the per-frame noise down)
        const sp = 0.004 + 0.0009 * d.range * d.range / d.size, sy = (0.2 + 0.05 * d.range) * DEG;
        const lever = d.id.endsWith('/AFT') ? aftLever : d.id.endsWith('/FWD') ? T.length / 2 : 1.2;
        const varP = sp * sp + (sy * lever) ** 2, varY = sy * sy;
        const prev = this.cargoEst.get(c.id);
        // innovation gate: a sighting far outside the combined uncertainty is not fused. If the sightings keep
        // disagreeing (~1 s at 10 Hz) the old estimate is the stale one (relocalisation, mis-association): restart.
        let fresh = !prev || prev.src === 'manifest';
        if (!fresh) {
          const innov = Math.hypot(center.x - prev.x, center.z - prev.z), gate = 4 * Math.sqrt((prev.varP ?? 1) + varP) + 0.5;
          if (innov > gate) {
            prev.miss = (prev.miss || 0) + 1;
            if (prev.miss < 10) continue;
            this.log(`${c.id}: cargo estimate disagreed with ${prev.miss} sightings by ${innov.toFixed(1)} m, restarting it`, 'warn');
            fresh = true;
          } else prev.miss = 0;
        }
        if (fresh) this.cargoEst.set(c.id, { ...center, yaw: axisYaw, t: this.sim.time, src: d.id, range: d.range, varP, varY, miss: 0 });
        else {
          prev.varP = (prev.varP ?? 1) + 3e-4; prev.varY = (prev.varY ?? 1) + 5e-7; // process noise: est-frame drift
          const wp = prev.varP / (prev.varP + varP), wy = prev.varY / (prev.varY + varY);
          prev.x += (center.x - prev.x) * wp; prev.z += (center.z - prev.z) * wp;
          prev.yaw = wrap(prev.yaw + wrap(axisYaw - prev.yaw) * wy);
          prev.varP *= 1 - wp; prev.varY *= 1 - wy;
          prev.t = this.sim.time; prev.src = d.id; prev.range = d.range;
        }
      } else if (d.kind === 'beacon') {
        this.beaconFix(d);
      } else if (d.kind === 'port') {
        this.beaconFix(d);
        const p = this.toEst(d.pos);
        const nYaw = this.dirToEst(d.normal) + d.yawNoise;
        const sp = 0.004 + 0.0009 * d.range * d.range / d.size, sy = (0.2 + 0.05 * d.range) * DEG;
        const varP = sp * sp, varY = sy * sy;
        const prev = this.portEst.get(d.obj.id);
        let fresh = !prev;
        if (!fresh) {
          const innov = Math.hypot(p.x - prev.x, p.z - prev.z), gate = 4 * Math.sqrt((prev.varP ?? 1) + varP) + 0.5;
          if (innov > gate) {
            prev.miss = (prev.miss || 0) + 1;
            if (prev.miss < 10) continue;
            fresh = true;
          } else prev.miss = 0;
        }
        if (fresh) this.portEst.set(d.obj.id, { x: p.x, z: p.z, y: p.y + 1.5, nYaw, t: this.sim.time, range: d.range, varP, varY, miss: 0 });
        else {
          prev.varP = (prev.varP ?? 1) + 1e-5; prev.varY = (prev.varY ?? 1) + 1e-8;
          const wp = prev.varP / (prev.varP + varP), wy = prev.varY / (prev.varY + varY);
          prev.x += (p.x - prev.x) * wp; prev.z += (p.z - prev.z) * wp; prev.y += (p.y + 1.5 - prev.y) * wp;
          prev.nYaw = wrap(prev.nYaw + wrap(nYaw - prev.nYaw) * wy);
          prev.varP *= 1 - wp; prev.varY *= 1 - wy;
          prev.t = this.sim.time; prev.range = d.range;
        }
      }
    }
  }

  // ---------------- executive ----------------
  start(name, gen) {
    if (this.sim.rover.estop) { this.log('Cannot start: E-STOP latched', 'warn'); return false; }
    if (!this.sim.rover.powered) { this.log('Cannot start: rover not powered (run START)', 'warn'); return false; }
    this.gen = gen; this.missionName = name; this.mode = 'AUTO'; this.paused = false;
    this.metrics = this._newMetrics();
    this.sim.rover.brake = false;
    this.log(`AUTO: ${name} started`, 'ok');
    this.sim.emit('mission', { name, state: 'start' });
    return true;
  }
  abort(reason = 'operator abort') {
    if (this.gen) this.log(`Mission "${this.missionName}" aborted: ${reason}`, 'warn');
    if (this.slam.recording) this.slam.endRecord(false);
    this.gen = null; this.missionName = null; this.mode = 'MANUAL'; this.phase = 'Idle'; this.detail = '';
    this.sim.rover.crawlZone = false; // an aborted mission must not leave the drive held at the crawl cap
    this.path = null; this.stop();
    this.sim.emit('mission', { state: 'abort' });
  }
  override() { // operator touched the controls
    if (this.mode === 'AUTO' && this.gen) { this.paused = true; this.mode = 'MANUAL'; this.log('Operator override: autonomy paused (type "resume" to continue)', 'warn'); }
  }
  resume() {
    if (!this.gen) { this.log('No mission to resume', 'warn'); return; }
    if (this.sim.rover.estop || !this.sim.rover.powered) { this.log('Cannot resume: rover not ready', 'warn'); return; }
    this.paused = false; this.mode = 'AUTO'; this.sim.rover.brake = false; this.log('Autonomy resumed', 'ok');
  }

  update(dt) {
    this.dt = dt;
    this.railsOk = false; // set again by navTo's cruise branch when the transit-warp check passes (Sim.stepAuto)
    this.updateEstimates();
    this.orbitalFix();
    const R = this.sim.rover;
    // frozenS: seconds a running mission has spent not executing (paused, powered down, FDIR hold). navTo takes it off
    // its watchdog clocks, so a hold or a survival-mode pause is not read as "no progress" when the mission picks up.
    if (R.estop || !R.powered) { if (this.gen && !this.paused) { this.paused = true; this.mode = 'MANUAL'; } if (this.gen) this.frozenS += dt; return; }
    if (this.mode !== 'AUTO' || !this.gen || this.paused) { if (this.gen) this.frozenS += dt; return; }
    // FDIR hold (health.js): computer switch-over, SEU reset, declutch, battery isolation, repair, array stow.
    // The executive is frozen (motor controllers see a zero command) and resumes where it was.
    if (this.holdUntil > this.sim.time) {
      R.cmd.vx = R.cmd.vz = R.cmd.wz = 0; this.frozenS += dt;
      if (this._holdLog !== this.holdWhy) { this._holdLog = this.holdWhy; this.detail = `FDIR hold: ${this.holdWhy}`; }
      return;
    }
    this._holdLog = null;
    try {
      const r = this.gen.next(dt);
      if (r.done) {
        const res = r.value || {};
        this.stop();
        this.log(`Mission "${this.missionName}" ${res.ok === false ? 'FAILED: ' + res.why : 'complete'}`, res.ok === false ? 'err' : 'ok');
        this.history.push({ name: this.missionName, ok: res.ok !== false, ...this.summary() });
        this.sim.emit('mission', { name: this.missionName, state: 'done', ok: res.ok !== false });
        this.gen = null; this.mode = 'MANUAL'; this.phase = 'Idle'; this.path = null;
        R.crawlZone = false;
        R.brake = true;
      }
    } catch (e) {
      console.error(e);
      this.abort('internal error: ' + e.message);
    }
  }

  summary() {
    const R = this.sim.rover, m = this.metrics;
    const acc = m.accuracy.length ? m.accuracy[m.accuracy.length - 1] : null;
    return { time: this.sim.time - m.start, dist: R.odo.dist - m.dist0, energyWh: R.power.energyUsedWh - m.e0, deliveries: m.deliveries, accuracy: acc };
  }

  // ---------------- primitive behaviours ----------------
  *wait(sec, label) { const t0 = this.sim.time; if (label) this.detail = label; while (this.sim.time - t0 < sec) { this.stop(); yield; } }

  *checkPower(label) {
    const R = this.sim.rover;
    if (R.power.soc >= this.reserveSoc) return;
    if (R.loaded) {
      // the dock can't charge a loaded rover (the coupler is under the module bay) and the rover has no arrays:
      // finish the delivery if the battery covers it with margin, else pause for the operator
      const need = this.deliveryEnergyWh(), spare = R.power.wh - need, floor = 0.05 * ROVER.battery.capacityWh;
      if (spare >= floor) {
        if (!this._lowLoadedT || this.sim.time - this._lowLoadedT > 120) {
          this._lowLoadedT = this.sim.time;
          this.log(`Low battery (${(R.power.soc * 100).toFixed(0)}%) while loaded: delivery needs ~${(need / 1000).toFixed(0)} kWh, ${(R.power.wh / 1000).toFixed(0)} kWh left - continuing to ${label}, charging after drop-off`, 'warn');
        }
        return;
      }
      yield* this.pauseForOperator(`Low battery while loaded: mission paused for operator (needs ~${(need / 1000).toFixed(0)} kWh, ${(R.power.wh / 1000).toFixed(0)} kWh left; "resume" to continue anyway)`);
      return;
    }
    this.log(`Low battery (${(R.power.soc * 100).toFixed(0)}%): going to the power dock before ${label}`, 'warn');
    const r = yield* this.chargeAtDock(0.9);
    if (r && r.ok === false) yield* this.pauseForOperator(`SoC ${(R.power.soc * 100).toFixed(1)}%: power dock unreachable (${r.why}) - mission paused for operator ("resume" to continue anyway)`);
  }

  // Energy (Wh) to finish carrying the held module to its port: drive at the loaded rolling resistance plus
  // hotel load, from the current estimate (same model as the logistics manager) and the berthing / back-out.
  deliveryEnergyWh() {
    const R = this.sim.rover, c = R.lift.cargo, e = this.est();
    if (!c) return 0;
    const port = this.sim.ports.find((p) => p.id === c.dest) || this.sim.ports[0], g = this.portGuess(port);
    const dist = Math.hypot(g.x - e.x, g.z - e.z) * 1.3 + 60;
    const driveWh = dist * (R.mass + c.T.mass) * MOON.g * 0.09 / 0.86 / 3600;
    const hotelWh = 1100 * (dist / 1.0 + 300) / 3600;
    return (driveWh + hotelWh) * 1.25;
  }

  // Stop, set the brakes and pause the mission with an explicit status; the operator's "resume" continues it.
  *pauseForOperator(msg) {
    const R = this.sim.rover;
    this.stop(); R.brake = true;
    this.phase = 'PAUSED: operator decision required'; this.detail = msg;
    this.log(msg, 'err');
    this.paused = true; this.mode = 'MANUAL';
    yield; // resumes here after "resume"
    R.brake = false;
  }

  // Drive onto the power dock pad, kneel onto the underbody coupler and charge (CC-CV) to `target` SoC.
  *chargeAtDock(target = 0.95) {
    const R = this.sim.rover, PD = this.sim.powerDock, L = PD.layout;
    if (R.loaded) return { ok: false, why: 'cannot charge while carrying cargo' };
    // goals are the surveyed layout: the map frame is the survey frame (initialised and drift-corrected by the
    // lander / port / power-dock beacons). Was the true-minus-estimate pose offset: a ground-truth leak.
    const off = () => ({ x: 0, z: 0 });
    this.phase = 'Power dock: approach';
    let o = off();
    let r = yield* this.navTo({ x: L.entry.x + o.x, z: L.entry.z + o.z, name: 'power dock entry', tol: 2.0, yaw: L.yaw, yawTol: 3 * DEG, toCharger: true });
    if (r && r.ok === false) return r;
    this.phase = 'Power dock: line up';
    yield* this.alignYaw(Math.atan2(-((L.pad.z + o.z) - this.est().z), (L.pad.x + o.x) - this.est().x), 1.5 * DEG);
    this.phase = 'Power dock: drive onto pad';
    let t0 = this.sim.time;
    while (this.sim.time - t0 < 90) {
      o = off();
      const rel = this.relBody({ x: L.pad.x + o.x, z: L.pad.z + o.z });
      if (rel.f < 0.2) break;
      const v = clamp(rel.f * 0.3, 0.12, 1.2);
      this.cmd(v, 0, v * clamp(-0.4 * Math.atan2(rel.r, Math.max(2, rel.f)), -1 / 8, 1 / 8));
      this.detail = `onto charge pad: ${rel.f.toFixed(1)} m, lateral ${rel.r.toFixed(2)} m`;
      yield;
    }
    this.stop(); R.brake = true;
    for (let i = 0; i < 30; i++) yield;
    this.phase = 'Power dock: couple';
    R.lift.target = ROVER.ride.kneel;
    t0 = this.sim.time;
    while (!PD.connected && this.sim.time - t0 < 20) { this.stop(); this.detail = `kneeling onto coupler: ride ${R.lift.pos.toFixed(2)} m`; yield; }
    if (!PD.connected) { R.lift.target = 0; R.brake = false; this.log('Power dock: coupler did not engage (not centred on the pad)', 'warn'); return { ok: false, why: 'coupler did not engage' }; }
    this.phase = 'Power dock: charging';
    const s0 = R.power.soc, ts = this.sim.time;
    while (R.power.soc < target && PD.connected) {
      this.stop();
      const rem = (target - R.power.soc) * ROVER.battery.capacityWh / Math.max(1, PD.chargeW);
      this.detail = `SoC ${(R.power.soc * 100).toFixed(1)}% -> ${(target * 100).toFixed(0)}% @ ${(PD.chargeW / 1000).toFixed(0)} kW | bank ${(PD.soc * 100).toFixed(1)}% | solar ${(PD.solarW / 1000).toFixed(0)} kW | ~${(rem * 60).toFixed(0)} min left`;
      if (!(PD.chargeW > 0) && PD.soc <= POWER_DOCK.bankFloor) this.detail += ' (bank at floor: waiting for sunrise)';
      yield;
    }
    this.log(`Charged ${(s0 * 100).toFixed(0)}% -> ${(R.power.soc * 100).toFixed(0)}% in ${((this.sim.time - ts) / 60).toFixed(1)} min (bank ${(PD.soc * 100).toFixed(0)}%)`, 'ok');
    this.phase = 'Power dock: undock';
    R.lift.target = 0;
    while (Math.abs(R.lift.pos) > 0.02) { this.stop(); this.detail = 'rising off the coupler'; yield; }
    R.brake = false;
    yield* this.backUp(18);
    return { ok: true };
  }
  *missionCharge(target = 0.95) { return yield* this.chargeAtDock(target); }

  *alignYaw(yaw, tol = 1.2 * DEG, opts = {}) {
    const R = this.sim.rover;
    let t0 = this.sim.time, noRate = 0, ok = true;
    while (true) {
      const e = this.est();
      const err = wrap(yaw - e.yaw);
      if (Math.abs(err) < tol && Math.abs(this.sensors.imu.yawRate) < 0.004) break;
      const wmax = R.loaded ? 0.07 : 0.11;
      // transit warp: a navigation pivot (sweep already checked clear by pivotSafe, rover empty) turns on rails at the
      // same rate the physics pivot runs; precision line-ups (pickup, docking, charging) never do
      if (opts.rails && !R.loaded && this.sim.warpWanted()) {
        this.railsOk = true;
        if (this.sim.warping) {
          const step = clamp(err, -wmax * this.dt, wmax * this.dt), T = R.pose();
          R.railsPlace(T.x, T.z, T.yaw + step, this.dt, 0);
          const o = this.sensors.odom; o.dyaw += step; o.dy += R.pose().y - T.y; o.vx = 0; o.vz = 0;
          this.sensors.imu.yawRate = Math.abs(err) - Math.abs(step) < tol ? 0 : step / this.dt; this.sensors.imu.roll = R.tilt.roll; this.sensors.imu.pitch = R.tilt.pitch;
          this._railsMoved = true;
          this.detail = `pivot turn (warp): heading error ${(err / DEG).toFixed(1)} deg`;
          yield; continue;
        }
      }
      const w = clamp(err * 0.9, -wmax, wmax);
      this.cmd(0, 0, Math.abs(w) < 0.012 ? Math.sign(w) * 0.012 : w);
      this.detail = `pivot turn: heading error ${(err / DEG).toFixed(1)} deg`;
      // stall: steering is set but the gyro sees no rotation -> something is blocking the sweep
      const steerDone = R.bogies.every((b) => Math.abs(b.steerTarget - b.steer) < 0.02);
      const rate = Math.abs(this.sensors.imu.yawRate);
      if (steerDone && Math.abs(w) > 0.02 && rate < 0.004) noRate += this.dt; else noRate = 0;
      if (noRate > 6) { this.log(`PIVOT STALL: commanded yaw ${w.toFixed(3)} rad/s, gyro ${(this.sensors.imu.yawRate).toFixed(4)} rad/s (sweep blocked)`, 'err'); ok = false; break; }
      if (this.sim.time - t0 > 150) { this.log('Pivot timeout', 'warn'); ok = false; break; }
      yield;
    }
    this.stop();
    for (let i = 0; i < 10; i++) yield;
    return ok;
  }

  nearestOnPath(p) {
    const P = this.path; if (!P) return 0;
    let best = this.pathIdx, bd = Infinity;
    for (let i = Math.max(0, this.pathIdx - 5); i < Math.min(P.length, this.pathIdx + 60); i++) {
      const d = (P[i].x - p.x) ** 2 + (P[i].z - p.z) ** 2;
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

  routeIsClear(pts) {
    this.planner.buildCostmap();
    return this.planner.pathBlocked(pts, 0, pts.length) < 0;
  }

  // Global planning with graceful fallbacks: normal -> drop temporary blocks -> tighter inflation.
  replan(goal, reason, quiet = false) {
    const e = this.est();
    const P = this.planner;
    P.unknownNearHazardCost = (this.sim.rover.loaded || this.surveyLimits) ? 12.0 : 2.5;
    const routes = this.slam.routes.filter((r) => r.ok);
    P.buildCostmap(routes);
    // straight-in legs (final lines onto a pose goal): drive the segment itself when it is clear of the hard
    // obstacle margin, so the arrival heading is the segment's and the grid search can't bend it
    if (goal.straight) {
      const d = Math.hypot(goal.x - e.x, goal.z - e.z), n = Math.max(2, Math.ceil(d));
      const pts = []; for (let i = 0; i <= n; i++) pts.push({ x: e.x + (goal.x - e.x) * i / n, z: e.z + (goal.z - e.z) * i / n });
      if (pts.every((p, i) => { const k = P.idx(p.x, p.z); return k >= 0 && (i < 3 || P.cost[k] < LETHAL_HALF); })) {
        this.path = pts; this.pathIdx = 0; this.pathTight = false; this.pathSource = 'direct line';
        this.sim.emit('path');
        return true;
      }
    }
    let res = P.plan(e.x, e.z, goal.x, goal.z);
    if (!res) res = P.plan(e.x, e.z, goal.x, goal.z, { escape: 35 });
    if (!res) {
      const inf = P.inflate; P.inflate = 5.0; P.buildCostmap(routes);
      res = P.plan(e.x, e.z, goal.x, goal.z, { escape: 35 });
      P.inflate = inf;
      if (res && !quiet) this.log('Planner: path found only with reduced obstacle margin (5.0 m) - proceeding slowly', 'warn');
      if (res) res.tight = true;
    }
    if (!res) {
      const nowB = this.sim.time, n = P.blocked.length;
      P.blocked = P.blocked.filter((b) => b.fault || (b.t != null && nowB - b.t < 180));
      if (P.blocked.length !== n) { P.buildCostmap(routes); res = P.plan(e.x, e.z, goal.x, goal.z, { escape: 35 }); }
    }
    if (!res) {
      const n = P.blocked.length; P.blocked = P.blocked.filter((b) => b.fault);
      if (n !== P.blocked.length) { P.buildCostmap(routes); res = P.plan(e.x, e.z, goal.x, goal.z, { escape: 35 }); }
    }
    if (!res) { if (!quiet) this.log(`Planner: no path to ${goal.name || 'goal'} (${reason})`, 'warn'); return false; }
    // plan hysteresis: on routine replans keep the current path unless it is blocked or the new
    // one is clearly (>15 %) cheaper - otherwise near-equal detours make the rover thrash
    if ((reason === 'periodic' || reason === 'local squeeze') && this.path && this.path.length > 2 && this.goal === goal) {
      const e2 = this.est(), i0 = this.nearestOnPath(e2), tail = this.path[this.path.length - 1];
      const offPath = Math.hypot(this.path[i0].x - e2.x, this.path[i0].z - e2.z) > 4.0;
      if (!offPath && Math.hypot(tail.x - goal.x, tail.z - goal.z) <= 1.5) {
        const cur = P.pathCost([{ x: e2.x, z: e2.z }, ...this.path.slice(i0)]);
        const neu = P.pathCost(res.pts);
        // Replan hysteresis (NAV): a routine replan only switches the path for a confirmed lethal obstacle within NAV.replanBlockM
        // path points, or once the fresh plan is cheaper than the path in hand by NAV.replanCostGain (30 %), and a path just
        // switched to is held for NAV.commitM of travel. Flickering map cells used to make the committed trajectory jump.
        const blockedNear = P.pathBlocked(this.path, i0, NAV.replanBlockM) >= 0;
        const sp = this._pathSwitchPos;
        if (!blockedNear) {
          if (Number.isFinite(cur) && sp && Math.hypot(e2.x - sp.x, e2.z - sp.z) < NAV.commitM) return true; // (a path with a blocked stretch further on has infinite cost: never held)
          if (Number.isFinite(cur) && neu > cur / (1 + NAV.replanCostGain)) return true;
        }
        // commitment (v1.5.0): for 20 s after a switch, a routine replan only replaces a path that has become blocked;
        // after that the new one must be >15 % AND > 12 m of typical ground cheaper (tiny relative gains on a 2 km route
        // used to flip the plan every few seconds)
        if (reason !== 'local squeeze' && Number.isFinite(cur) && this.sim.time - (this._pathSwitchT ?? -1e9) < 20) return true;
        if (Number.isFinite(cur) && (neu > cur * 0.85 || cur - neu < 12 * P.unkEff)) return true;
        // reversal guard: a routine replan that turns the rover back the way it came needs to be clearly (40 %)
        // cheaper. Newly mapped ground keeps raising the cost of the route ahead while the unmapped alternative
        // behind still looks cheap, and the loaded rover (no point turns) thrashed east-west for minutes (CHANGES.md §14)
        const dir = (pts, i) => { const a = pts[i] || pts[0], b = pts[Math.min(pts.length - 1, i + 15)]; return Math.atan2(b.z - a.z, b.x - a.x); };
        const dTurn = Math.abs(wrap(dir(res.pts, 0) - dir(this.path, i0)));
        if (Number.isFinite(cur) && dTurn > 100 * DEG && neu > cur * 0.6) return true;
        // a plan that swings the rover > 60 deg off its present line costs a long arc (or a stop and pivot): 30 % cheaper
        if (Number.isFinite(cur) && dTurn > 60 * DEG && neu > cur * 0.7) return true;
      }
    }
    if (this.path !== res.pts) { this._pathSwitchT = this.sim.time; this._pathSwitchPos = { x: e.x, z: e.z }; }
    this.path = res.pts; this.pathIdx = 0;
    this.pathTight = !!res.tight;
    this.pathSource = 'A* (explore)';
    this.lastPlan = { t: this.sim.time, ms: res.ms, expanded: res.expanded, len: res.pts.length, buildMs: P.buildMs };
    this.sim.emit('path');
    return true;
  }

  // Teach & repeat: take the portion of any proven route that runs between near-start and
  // near-goal (either direction), joined to the exact start/goal with short A* stubs.
  routeFromMemory(goal) {
    const e = this.est();
    const cands = [];
    // each route offers its straightened line and, behind it, the line it was actually driven on (r.rawPts): a straightened chord
    // crosses ground first seen from a distance, and closer scans can later show a rock on it, while the driven line stays proven
    for (const r of this.slam.routes) {
      if (!r.ok || r.pts.length < 20) continue;
      for (const [pts, alt] of [[r.pts, 0], ...(r.rawPts ? [[r.rawPts, 1]] : [])]) {
        let i0 = -1, d0 = Infinity, i1 = -1, d1 = Infinity;
        pts.forEach((p, i) => {
          const a = (p.x - e.x) ** 2 + (p.z - e.z) ** 2, b = (p.x - goal.x) ** 2 + (p.z - goal.z) ** 2;
          if (a < d0) { d0 = a; i0 = i; } if (b < d1) { d1 = b; i1 = i; }
        });
        d0 = Math.sqrt(d0); d1 = Math.sqrt(d1);
        if (d0 > 150 || d1 > 150 || Math.abs(i1 - i0) < 25) continue;
        const seg = i0 < i1 ? pts.slice(i0, i1 + 1) : pts.slice(i1, i0 + 1).reverse();
        const loadedNeed = this.sim.rover.loaded || !!this.surveyLimits;
        // (the driven line ranks after its own straightened line: +1 per route point of score, i.e. it is the fallback)
        cands.push({ r, seg, d0, d1, alt, score: (loadedNeed && !r.loadedOk ? 2000 : 0) + seg.length + 2 * (d0 + d1) + (alt ? 1 : 0) });
      }
    }
    if (!cands.length) return null;
    cands.sort((a, b) => a.score - b.score);
    const P = this.planner, okRoutes = this.slam.routes.filter((x) => x.ok);
    let why = '', bridged = false;
    const at = (p) => `(${p.x.toFixed(0)}, ${p.z.toFixed(0)})`;
    const repairSeg = (seg) => {
      let cur = seg, repairs = 0;
      // fallback (v1.7.0): a blocked stretch the local repair cannot get round (short rejoin, 320-point detour cap) is bridged by a
      // longer detour to a point further along where the route is clear again (up to 300 m on, larger search budget). Before, any
      // such stretch threw the whole 2 km route away and the loaded rover explored the crater field instead.
      const bridge = (bi, pre) => {
        bridged = true;
        for (let j = bi + 30; j < Math.min(cur.length - 15, bi + 300); j += 15) {
          if (P.pathBlocked(cur, j, Math.min(60, cur.length - j)) >= 0) continue;
          const d = P.plan(cur[pre].x, cur[pre].z, cur[j].x, cur[j].z, { maxExpand: 300000 });
          if (d && d.pts.length >= 2 && d.pts.length <= 900 && P.pathBlocked(d.pts, 0, d.pts.length) < 0) { why = `bridged a blocked stretch at ${at(cur[bi])}`; return [...cur.slice(0, pre), ...d.pts, ...cur.slice(j + 1)]; }
        }
        return null;
      };
      for (let bi; (bi = P.pathBlocked(cur, 0, cur.length)) >= 0;) {
        if (++repairs > 10) { why = `more than 10 blocked stretches (last at ${at(cur[bi])})`; return null; }
        let pre = Math.max(0, bi - 8);
        while (pre > 0 && P.pathBlocked(cur, pre, 1) >= 0 && bi - pre < 60) pre--;
        let post = Math.min(cur.length - 1, bi + 8);
        while (post < cur.length - 1 && post - bi < 120 && P.pathBlocked(cur, post, Math.min(6, cur.length - post)) >= 0) post++;
        if (P.pathBlocked(cur, pre, 1) >= 0) {
          if (pre === 0 && P.pathBlocked(cur, post, 1) < 0 && Math.hypot(cur[post].x - e.x, cur[post].z - e.z) <= 220) { cur = cur.slice(post); continue; }
          why = `blocked at ${at(cur[bi])}, no clear point before it`; return null;
        }
        if (P.pathBlocked(cur, post, 1) >= 0) {
          if (post === cur.length - 1 && Math.hypot(cur[pre].x - goal.x, cur[pre].z - goal.z) <= 220) { cur = cur.slice(0, pre + 1); continue; }
          const br = bridge(bi, pre); if (br) { cur = br; continue; }
          why = `blocked from ${at(cur[bi])} for more than 120 m`; return null;
        }
        const detour = P.plan(cur[pre].x, cur[pre].z, cur[post].x, cur[post].z, { maxExpand: 120000 });
        if (!detour || detour.pts.length < 2 || detour.pts.length > 320 || P.pathBlocked(detour.pts, 0, detour.pts.length) >= 0) {
          const br = bridge(bi, pre); if (br) { cur = br; continue; }
          why = `no local detour around ${at(cur[bi])}`; return null;
        }
        cur = [...cur.slice(0, pre), ...detour.pts, ...cur.slice(post + 1)];
      }
      if (cur.length < 15) { why = 'too little of it left after repairs'; return null; }
      return cur;
    };
    const tryAttach = (seg) => {
      const first = seg[0], last = seg[seg.length - 1];
      const d0 = Math.hypot(first.x - e.x, first.z - e.z), d1 = Math.hypot(goal.x - last.x, goal.z - last.z);
      const stubA = d0 > 6 ? (P.plan(e.x, e.z, first.x, first.z) || P.plan(e.x, e.z, first.x, first.z, { escape: 35 })) : { pts: [{ x: e.x, z: e.z }] };
      const stubB = d1 > 6 ? (P.plan(last.x, last.z, goal.x, goal.z) || P.plan(last.x, last.z, goal.x, goal.z, { escape: 35 })) : { pts: [last, { x: goal.x, z: goal.z }] };
      return stubA && stubB ? { pts: P.resample([...stubA.pts, ...seg, ...stubB.pts], 1.0), d0, d1 } : null;
    };
    P.buildCostmap(okRoutes);
    const repaired = [];
    for (const c of cands) {
      bridged = false;
      const seg = repairSeg(c.seg);
      if (seg) repaired.push({ c, seg, bridged });
    }
    // a candidate that needed no long bridging detour first (e.g. the driven line of a route whose straightened line is blocked)
    repaired.sort((a, b) => (a.bridged - b.bridged) || (a.c.score - b.c.score));
    for (const { c, seg } of repaired) {
      const res = tryAttach(seg);
      if (res) return { route: c.r, ...res };
    }
    if (repaired.length) {
      const inf = P.inflate;
      P.inflate = Math.min(inf, 5.0);
      try {
        P.buildCostmap(okRoutes);
        for (const { c, seg } of repaired) {
          const res = tryAttach(seg);
          if (res) return { route: c.r, ...res };
        }
      } finally { P.inflate = inf; }
    }
    if (repaired.length && !why) why = 'no A* link from here to the route or from the route to the goal';
    return { rejected: cands[0].r, why };
  }

  // navigate to a goal using the global/local planners and route memory.
  // Recovery and retry (v1.7.0): a leg the navigation gives up on (no progress for 4 min, boxed in after 4 unwinds,
  // moving away from the goal, steering not answering, pivot stall) used to end the whole mission, so one bad pocket on a
  // 2 km traverse cost all three deliveries. Now the leg is re-tried up to NAV.legRetries times after an autonomous
  // recovery (_retryPrep): back out of the pocket, keep what was learned about it, re-check the position against the
  // orbital map, and try the other route source (remembered route <-> fresh exploration). Only then is the operator asked.
  *navTo(goal, opts = {}) {
    const tries = opts.retries ?? (goal.toCharger ? 1 : NAV.legRetries);
    let r = null, noMemory = !!opts.noMemory;
    for (let a = 0; ; a++) {
      r = yield* this._navLeg(goal, { ...opts, noMemory, attempt: a });
      if (!r || r.ok !== false || !r.retry || a >= tries || !this.gen) return r;
      { const R = this.sim.rover, tilt = Math.max(Math.abs(R.tilt.pitch), Math.abs(R.tilt.roll)) * 180 / Math.PI;
        if (tilt > 12 || R.tilt.margin < (R.loaded ? 18 : 12)) { this.log(`No autonomous retry: the vehicle is on a steep tilt (${tilt.toFixed(0)} deg, tip margin ${R.tilt.margin.toFixed(0)} deg)`, 'warn'); return r; } }
      this.metrics.retries = (this.metrics.retries || 0) + 1;
      this.log(`Navigation to ${goal.name || 'goal'} stopped (${r.why}). Autonomous recovery and retry ${a + 1}/${tries}`, 'warn');
      yield* this._retryPrep(goal, r);
      // the route source that just failed is not the one to repeat: a remembered route that keeps boxing in is dropped,
      // an exploration that wandered gets the remembered route back
      noMemory = r.usedRoute ? true : false;
    }
  }

  *_retryPrep(goal, r) {
    const R = this.sim.rover, P = this.planner;
    const phase0 = this.phase;
    this.phase = `${phase0.replace(/ \((recovery|unwind|retry)\)$/, '')} (retry)`;
    this.stop();
    for (let i = 0; i < 30 && R.telemetry().speed > 0.05; i++) yield;
    // keep the obstacle marks the failed attempt learned (fail() clears them for the next, unrelated leg)
    const now = this.sim.time;
    for (const b of r.blocks || []) if (b.ttl == null || b.t == null || now - b.t < b.ttl) P.blocked.push(b);
    this.goal = goal;
    // steering / stall: a few metres straight back on a clear arc frees the tyres from their ruts.
    // boxed in / no progress / moving away: back out along the way in, which also marks the pocket lethal (unwindPath)
    // (not after "boxed in again after 4 reverse recoveries": it has just unwound four times out of this pocket, and the pocket is
    // already marked lethal; a fifth reverse took a loaded rover over a rim in a v1.7.0 test run)
    if (/steering|pivot/.test(r.why)) yield* this.recover(goal, 'retry');
    else if (!/boxed in/.test(r.why)) yield* this.unwindPath(R.loaded ? 20 : 25);
    // re-localise before re-planning: a drifted estimate puts the map's obstacles in the wrong place, and box-ins on long
    // legs came with SLAM errors of 5-13 m (CHANGES.md §27). Force an orbital-map check at the next tick.
    this.mrl.dLast = -1e9;
    if (this.mrl.buf) this.mrl.buf = [];
    yield* this.wait(3, 'retry: re-localising against the orbital map');
    this._noSteerT = 0;
    this.phase = phase0;
  }

  *_navLeg(goal, opts = {}) {
    const R = this.sim.rover;
    const tol = goal.tol ?? 2.0;
    const from = this.site(this.est().x, this.est().z), to = goal.site || this.site(goal.x, goal.z);
    this.goal = goal;
    this.local.last = null;
    // goal.toCharger: this leg IS the drive to the charger (chargeAtDock). Checking the reserve again here sent it to
    // chargeAtDock -> navTo -> checkPower without end: below the reserve, any unloaded leg died with a stack overflow.
    if (!goal.toCharger) yield* this.checkPower(goal.name);
    this.phase = `Navigate to ${goal.name || to}`;
    this.slam.setPayload(R.loaded || !!this.surveyLimits); // the first plan already needs this leg's limits
    this.planner.inflate = (R.loaded || this.surveyLimits) ? NAV.inflateLoaded : NAV.inflateEmpty; // 6.5 m hard margin around obstacles while carrying
    let usingRoute = false, routeProven = false, rejoins = 0;
    const mem = !opts.noMemory && Math.hypot(goal.x - this.est().x, goal.z - this.est().z) > 150 ? this.routeFromMemory(goal) : null;
    if (mem && mem.pts) {
      this.path = mem.pts; this.pathIdx = 0; usingRoute = true;
      mem.route.uses++;
      // a route this vehicle has already driven with this load (loaded routes are recorded under the loaded limits) is known
      // to be passable: a close obstacle beside it is not a reason to leave it (see the squeeze test below)
      routeProven = !!mem.route.loadedOk || !(R.loaded || this.surveyLimits);
      this.pathSource = `REPEAT ${mem.route.id} (${mem.route.from}->${mem.route.to}, used ${mem.route.uses}x)`;
      this.log(`Route memory: reusing proven route ${mem.route.id} (${mem.route.from}->${mem.route.to}); joins ${mem.d0.toFixed(0)} m from here, leaves ${mem.d1.toFixed(0)} m from goal`, 'ok');
      this.sim.emit('path');
    } else if (mem && mem.rejected) this.log(`Route memory: ${mem.rejected.id} is now blocked (${mem.why || 'unknown'}) - exploring a new route`, 'warn');
    if (!usingRoute && !this.replan(goal, 'initial')) {
      yield* this.wait(2, 'waiting for map data');
      if (!this.replan(goal, 'retry')) return { ok: false, why: 'no path to goal (check geofences / goal location)' };
    }
    if (from !== to) this.slam.beginRecord(from, to);
    let lastReplan = this.sim.time, lastSpliceT = -1e9, lastLoopSeen = this.slam.lastLoop?.t ?? -1e9, stuckT = this.sim.time, stuckPos = this.est();
    this._stuckPosRef = stuckPos;
    let blockedT = 0, episodes = 0, unwinds = 0, unwindBestD = Infinity, pivotFails = 0, backoffs = 0, bestD = Infinity, bestT = this.sim.time, lastPivotPos = null, bestTiles = this.slam.tiles.size, t00 = this.sim.time;
    let remRef = null, remCum = null, bestRouteRem = Infinity;
    const pathRemAt = (idx) => {
      if (!this.path || !this.path.length) return Infinity;
      if (this.path !== remRef) {
        remRef = this.path;
        remCum = new Float64Array(remRef.length);
        for (let i = remRef.length - 2; i >= 0; i--) {
          remCum[i] = remCum[i + 1] + Math.hypot(remRef[i + 1].x - remRef[i].x, remRef[i + 1].z - remRef[i].z);
        }
      }
      return remCum[Math.min(idx, remCum.length - 1)];
    };
    if (usingRoute) bestRouteRem = pathRemAt(0);
    const fail = (why) => {
      this._stuckPosRef = null;
      if (this.slam.recording) this.slam.endRecord(false);
      const blocks = this.planner.blocked.filter((b) => !b.fault);
      this.planner.blocked = this.planner.blocked.filter((b) => b.fault);
      this.stop();
      return { ok: false, why, retry: true, blocks, usedRoute: usingRoute || this.pathSource.startsWith('REPEAT') };
    };
    let frozen0 = this.frozenS, lidarDownSince = null, unknownRun = 0, deadband = false, lastPos = this.est(), pinRef = null, pins = 0;
    while (true) {
      // time the executive was frozen does not count against the progress / stuck watchdogs
      if (this.frozenS > frozen0) { const f = this.frozenS - frozen0; frozen0 = this.frozenS; stuckT += f; bestT += f; t00 += f; lastReplan += f; }
      const e = this.est();
      const dGoal = Math.hypot(goal.x - e.x, goal.z - e.z);
      // upset: past any drivable attitude (the tilt reflex fires at 14-17 deg). Nothing autonomous is safe from here
      { const tilt = Math.max(Math.abs(R.tilt.pitch), Math.abs(R.tilt.roll)) * 180 / Math.PI;
        if (tilt > NAV.upsetDeg) { const r = fail(`vehicle upset: tilt ${tilt.toFixed(0)} deg: all motion stopped, operator assistance required`); r.retry = false; R.brake = true; return r; } }
      if (dGoal < tol) break;
      this.pathIdx = this.nearestOnPath(e);
      if (usingRoute && !this.pathSource.startsWith('REPEAT')) usingRoute = false;
      // overall progress watchdog: 4 minutes without getting 5 m closer => ask for help
      if (dGoal < bestD - 5) {
        bestD = dGoal; bestT = this.sim.time; episodes = 0;
        if (unwinds > 0 && dGoal < unwindBestD - 300) { unwinds--; unwindBestD = dGoal; }
      }
      // a detour around a keep-out zone / crater field moves away from the goal while it maps new ground: that is progress,
      // but only while it stays a detour. (v1.5.0: unbounded, this let a rover that could no longer steer drive 1.3 km
      // away from PORT-1 "mapping new ground" without the watchdog ever firing)
      else if (this.slam.tiles.size > bestTiles + 6 && this.sim.time - t00 < 3600 && dGoal < bestD + Math.max(120, 0.18 * bestD)) { bestTiles = this.slam.tiles.size; bestT = this.sim.time; }
      else if (usingRoute) {
        const rem = pathRemAt(this.pathIdx);
        if (rem < bestRouteRem - 10 && dGoal < bestD + Math.max(180, 0.18 * bestD)) { bestRouteRem = rem; bestT = this.sim.time; }
      }
      if (this.sim.time - bestT > 240) return fail(`no progress toward ${goal.name || 'goal'} for 4 min: operator assistance required`);
      const awayLim = this.path && !this._noSteerT ? Math.max(240, 0.4 * bestD) : Math.max(150, 0.25 * bestD);
      if (Number.isFinite(bestD) && dGoal > bestD + awayLim) return fail(`moving away from ${goal.name || 'goal'} (${dGoal.toFixed(0)} m, was ${bestD.toFixed(0)} m): stopped, operator assistance required`);
      // steering-authority check: moving, asked to turn, and the gyro says the body is not turning
      // (v1.7.0: compared on 6 s averages, in the direction commanded, against NAV.noSteerRatio = 10 %. Measured: a healthy rover
      // achieves 40-55 % of the commanded yaw rate (skid-steer scrub), on five wheels with one free-rolling 15-25 %; 25 % stopped
      // the five-wheel traverse of tests/faults.mjs. The instantaneous test counted the normal lag of a loaded
      // skid-steer turn - the yaw rate builds over seconds while the command already reverses on an S-bend - as "no response",
      // and stopped a loaded leg 75 m from PORT-3 (CHANGES.md §29). A body that really does not yaw still trips it in ~15 s.)
      { const f = R._cmdF, yrS = this.sensors.imu.yawRate || 0, a = Math.min(1, this.dt / 6);
        const on = f && !(this.sim.warping && this.railsOk) && Math.abs(f.vx) > 0.5;
        const sg = f && f.wz ? Math.sign(f.wz) : 0;
        this._yawCmdAvg = (this._yawCmdAvg || 0) + a * ((on ? Math.abs(f.wz) : 0) - (this._yawCmdAvg || 0));
        this._yawActAvg = (this._yawActAvg || 0) + a * ((on ? yrS * sg : 0) - (this._yawActAvg || 0));
        if (on && this._yawCmdAvg > 0.04 && this._yawActAvg < NAV.noSteerRatio * this._yawCmdAvg) this._noSteerT = (this._noSteerT || 0) + this.dt;
        else this._noSteerT = Math.max(0, (this._noSteerT || 0) - 2 * this.dt);
        if (this._noSteerT > 15) { this._noSteerT = 0; return fail(`vehicle is not responding to steering (yaw rate < ${(NAV.noSteerRatio * 100).toFixed(0)} % of command for 15 s): stopped, operator assistance required`); } }
      const forceReplan = this._replanNow;
      if (this._replanNow) { this._replanNow = false; lastReplan = -1e9; bestD = Infinity; }
      if ((this.slam.lastLoop?.t ?? -1e9) > lastLoopSeen) {
        lastLoopSeen = this.slam.lastLoop.t;
        this.local.prevK = 0;
        lastReplan = -1e9;
      }
      // (in transit warp the map changes per metre, not per second: replanning every 5 sim-s would cost ~10 plans per wall second)
      const freeNow = this.local.last?.freeDist ?? 99;
      const closeObs = !this.sim.warp.active && freeNow < 14;
      if (this.sim.time - lastReplan > (this.sim.warp.active ? (freeNow < 14 ? 4 : 20) : closeObs ? 0.8 : usingRoute ? 8 : 5)) {
        lastReplan = this.sim.time;
        if (usingRoute) {
          this.planner.buildCostmap(this.slam.routes.filter((r) => r.ok));
          const tail = this.path?.[this.path.length - 1];
          if (tail && Math.hypot(tail.x - goal.x, tail.z - goal.z) > 1.5) {
            if (dGoal < 40) {
              usingRoute = false;
              this.replan(goal, 'frame shift approach', true);
            } else {
              // Frame shift moved un-timestamped stubB away from surveyed goal: re-anchor tail to goal.
              let cut = -1;
              for (let i = this.path.length - 1; i >= this.pathIdx; i--) if (this.path[i].t != null) { cut = i; break; }
              if (cut >= 0) {
                const a = this.path[cut];
                const tp = Math.hypot(goal.x - a.x, goal.z - a.z) > 6 ? this.planner.plan(a.x, a.z, goal.x, goal.z) : { pts: [a, { x: goal.x, z: goal.z }] };
                if (tp) { this.path = [...this.path.slice(0, cut + 1), ...this.planner.resample(tp.pts, 1.0)]; this.pathIdx = Math.min(this.pathIdx, this.path.length - 1); this.sim.emit('path'); }
                else { usingRoute = false; this.replan(goal, 'route tail blocked', true); }
              } else { usingRoute = false; this.replan(goal, 'frame shift approach', true); }
            }
          }
          const blk = usingRoute ? this.planner.pathBlocked(this.path, this.pathIdx, 150) >= 0 : false;
          // squeeze: a close obstacle along a route not proven for this load. On a proven one (v1.7.0) the local planner steers past
          // it on the route; leaving the route there sent the loaded rover exploring next to the obstacle, and it boxed in at the
          // same spot in every M6 run (CHANGES.md §27, §29, §31). A route that is actually blocked still splices / replans (blk).
          const squeeze = usingRoute && !routeProven && !blk && freeNow < 11 && this.pathIdx > 25 && dGoal > 60;
          if (usingRoute && (blk || squeeze)) {
            let spliced = false;
            if (blk && dGoal > 120) {
              let jJoin = Math.min(this.path.length - 1, this.pathIdx + 65);
              while (jJoin < this.path.length - 40 && this.planner.pathBlocked(this.path, jJoin, 20) >= 0) jJoin += 15;
              if (jJoin > this.pathIdx + 12 && this.planner.pathBlocked(this.path, jJoin, 40) < 0) {
                const jp = this.path[jJoin], jDet = this.planner.plan(e.x, e.z, jp.x, jp.z, { maxExpand: 60000 });
                if (jDet && jDet.pts.length >= 2 && jDet.pts.length <= 200) {
                  this.path = [...this.planner.resample(jDet.pts, 1.0), ...this.path.slice(jJoin + 1)];
                  this.pathIdx = 0; this.sim.emit('path'); spliced = true;
                  this.log('Remembered route obstructed ahead: splicing local detour', 'info');
                }
              }
            }
            // (v1.7.0) before giving the route memory up for exploration, ask it again from here: it now has the driven line behind
            // each straightened route and longer bridging detours. Dropping straight to exploration sent a loaded rover round the back
            // of the habitat into the power-dock yard, where it boxed in 25 m from PORT-2's pre-dock point
            if (!spliced && blk && rejoins < 3 && dGoal > 150) {
              rejoins++;
              const m2 = this.routeFromMemory(goal);
              if (m2 && m2.pts) {
                this.path = m2.pts; this.pathIdx = 0; spliced = true; this.sim.emit('path');
                this.pathSource = `REPEAT ${m2.route.id} (${m2.route.from}->${m2.route.to}, re-joined)`;
                this.log(`Remembered route obstructed ahead: re-joined ${m2.route.id} ${m2.d0.toFixed(0)} m from here`, 'info');
              }
            }
            if (!spliced) {
              this.log(blk ? 'Remembered route obstructed ahead: replanning around it' : 'Close obstacle along remembered route: replanning detour', 'info');
              usingRoute = false;
              if (!this.replan(goal, blk ? 'route blocked' : 'route clearance', true)) {
                this.log('Remembered route blocked and no detour found yet', 'warn');
                this.stop(); yield* this.wait(2, 'waiting for map / path');
              }
            }
          }
        } else {
          let rWhy = forceReplan ? 'forced' : 'periodic';
          if (!forceReplan && closeObs) {
            this.planner.buildCostmap(this.slam.routes.filter((r) => r.ok));
            if (this.planner.pathBlocked(this.path, this.pathIdx, 30) >= 0) rWhy = 'obstacle ahead';
            else if ((this.local.last?.freeDist ?? 99) < 10) rWhy = 'local squeeze';
          }
          this.replan(goal, rWhy, true);
        }
      }
      // lookahead target on the path
      let li = this.pathIdx;
      const cross = Math.hypot(this.path[li].x - e.x, this.path[li].z - e.z);
      // lookahead >= NAV.lookaheadMin (10 m): the pure-pursuit point sits far enough ahead that the delayed regolith shear
      // reaction of a skid-steer vehicle does not make it hunt left and right around a short-chord target
      let look = clamp(Math.max(NAV.lookaheadMin, R.telemetry().speed * NAV.lookaheadSpeedS), NAV.lookaheadMin, 25);
      if (cross > 2.5) look = Math.min(look, Math.max(5.5, 11 - cross));
      let acc = 0;
      while (li < this.path.length - 1 && acc < look) { acc += Math.hypot(this.path[li + 1].x - this.path[li].x, this.path[li + 1].z - this.path[li].z); li++; }
      // On sharp detours / U-turns around obstacles, don't look 20 m across a bend into the hazard: use the 8 m near tangent.
      if (look > 8.5) {
        let liS = this.pathIdx, accS = 0;
        while (liS < this.path.length - 1 && accS < 8) { accS += Math.hypot(this.path[liS + 1].x - this.path[liS].x, this.path[liS + 1].z - this.path[liS].z); liS++; }
        const pS = this.path[liS];
        if (Math.hypot(pS.x - e.x, pS.z - e.z) > 4 && Math.abs(wrap(Math.atan2(-(pS.z - e.z), pS.x - e.x) - e.yaw)) > 75 * DEG) li = liS;
      }
      const tgt = this.path[li];
      const rel = this.relBody(tgt, e);
      const headErr = Math.atan2(-rel.r, rel.f);
      // Turning (v1.5.0): heading errors up to 115 deg are taken on a moving arc (the local planner's curvature, slowed
      // for the turn below), so the rover no longer stops and spins for every bend. A point turn is kept only for a
      // target essentially behind it (> 115 deg; > 150 deg within 6 m of the last one), or when no forward arc is free.
      const sinceTurn = Math.hypot(e.x - (lastPivotPos?.x ?? 1e9), e.z - (lastPivotPos?.z ?? 1e9));
      const closeTurn = (this.local.last?.freeDist ?? 99) < 11 && sinceTurn >= 8;
      const pivotLim = sinceTurn < (usingRoute ? 15 : 6) ? 150 : usingRoute ? 60 : closeTurn ? 65 : 115;
      if (!R.loaded && Math.abs(headErr) > pivotLim * DEG && dGoal > tol + 0.8 && Math.hypot(rel.f, rel.r) > tol + 0.8) { // a loaded rover never point-turns: wheels would dig ruts and the CoM is 6.8 m up
        if (this.local.pivotSafe(e)) {
          lastPivotPos = { x: e.x, z: e.z };
          const tStop = this.sim.time;
          for (let i = 0; i < 5 || (R.telemetry().speed > 0.2 && this.sim.time - tStop < 4); i++) { this.stop(); yield; }
          const ok = yield* this.alignYaw(Math.atan2(-(tgt.z - e.z), tgt.x - e.x), 8 * DEG, { rails: true });
          if (!ok) { pivotFails++; if (pivotFails >= 3) return fail('vehicle stalled during pivot: operator assistance required'); yield* this.recover(goal, 'pivot stall'); }
          stuckT = this.sim.time; stuckPos = this.est();
          continue;
        }
      }
      // A loaded rover can't point-turn, so a goal inside its turning circle is unreachable going forward: it
      // orbits the goal (the arc tracker holds R ~ 9 m). Back straight up until the goal needs a gentler arc.
      if (R.loaded && dGoal < 15 && backoffs < 3) {
        const g = this.relBody(goal, e), kReq = 2 * Math.abs(g.r) / Math.max(1e-6, g.f * g.f + g.r * g.r), K_OK = 0.08; // 1 / 12.5 m
        // (no backoff when going straight on already passes within the arrival tolerance)
        if (kReq > 1.25 * K_OK && Math.abs(g.r) < 1 / K_OK && !(g.f > 0 && Math.abs(g.r) < 0.9 * tol)) {
          const b = clamp(Math.sqrt(Math.max(0, 2 * Math.abs(g.r) / K_OK - g.r * g.r)) - g.f, 3, 20);
          backoffs++;
          this.log(`${goal.name || 'goal'} is inside the loaded turning circle (${g.f.toFixed(1)} m ahead, ${g.r.toFixed(1)} m across): reversing ${b.toFixed(0)} m to re-approach`, 'info');
          yield* this.backUp(b);
          this.replan(goal, 'turning-circle backoff', true);
          stuckT = this.sim.time; stuckPos = this.est(); bestD = Infinity;
          continue;
        }
      }
      // pinned / spinning in place (v1.7.0): the wheels have turned > 2.5 m in 8 s but the LiDAR scene has not moved and the
      // heading has not changed. Wheel odometry would carry the estimate away (15 m in a minute when the rover was wedged under
      // the pod rack), so the estimate is put back where the scene says the vehicle is, the spot is marked and the rover recovers.
      if (!this.sim.warp.active && this.sensors.scanId !== pinRef?.id) {
        const now = this.sim.time;
        if (!pinRef || now - pinRef.t > 8) {
          const cur = this._scanSample(), o = this.sensors.odom;
          if (pinRef && o.dist - pinRef.d > 2.5 && Math.abs(wrap(e.yaw - pinRef.e.yaw)) < 3 * DEG && this._scanSame(pinRef.pts, cur)) {
            const lost = o.dist - pinRef.d;
            this.slam.est.x = pinRef.e.x; this.slam.est.z = pinRef.e.z;
            this.metrics.pinned = (this.metrics.pinned || 0) + 1;
            this.log(`Wheels turned ${lost.toFixed(1)} m but the LiDAR scene did not move: vehicle pinned or spinning in place. Position held, recovering`, 'warn');
            this.planner.blocked.push({ x: pinRef.e.x + Math.cos(e.yaw) * 7, z: pinRef.e.z - Math.sin(e.yaw) * 7, r: 3, t: now, ttl: 600 });
            if (++pins > 3) return fail('vehicle pinned: wheels turn but it does not move (4 times): operator assistance required');
            yield* this.recover(goal, 'pinned');
            pinRef = null; stuckT = this.sim.time; stuckPos = this.est(); continue;
          }
          pinRef = { t: now, id: this.sensors.scanId, d: o.dist, e: this.est(), pts: cur };
        }
      }
      // speed governor
      // a survey (teach) drive plans and checks the ground with the loaded limits, so the route it records is one a
      // loaded rover can repeat (point turns and empty speeds stay available: see surveyRoute)
      const loaded = R.loaded || !!this.surveyLimits;
      this.slam.setPayload(loaded);
      // tilt reflex: the ground ahead is steeper than the map said. Stop, mark it as a no-go, back out and replan.
      { const tiltDeg = Math.max(Math.abs(R.tilt.pitch), Math.abs(R.tilt.roll)) * 180 / Math.PI;
        if (tiltDeg > (loaded ? 14 : 17) && this.sim.time - (this._tiltT || -99) > 20) {
          this._tiltT = this.sim.time;
          this.log(`Tilt reflex: ${tiltDeg.toFixed(0)} deg (limit ${loaded ? 14 : 17}): steeper than mapped, marking slope, backing out`, 'warn');
          for (const f of [4, 9, 14]) this.planner.blocked.push({ x: e.x + Math.cos(e.yaw) * f, z: e.z - Math.sin(e.yaw) * f, r: 5, t: this.sim.time, ttl: 900 });
          this.stop(); yield* this.recover(goal, 'steep slope'); lastReplan = 0; continue;
        } }
      let v = Math.min(this.speedSetting, R.speedCap());
      // a LiDAR dropout only derates the leg once it has lasted NAV.dropoutHoldS: a transient one (power cycle, dust glitch) is
      // coasted through at a walking pace instead of stopping or aborting the leg
      if (this.sensors.faults.lidar) {
        lidarDownSince ??= this.sim.time;
        v = Math.min(v, this.sim.time - lidarDownSince > NAV.dropoutHoldS ? (usingRoute ? 0.3 : 0) : 1.0);
      } else lidarDownSince = null;
      { const mg = R.tilt.marginF ?? R.tilt.margin; if (mg < 22) v *= clamp((mg - 8) / 14, 0.3, 1); if (loaded) v = Math.min(v, 0.6 + Math.max(0, mg - 10) * 0.12); }
      if (R.wheels.some((w) => w.fault)) v *= 0.75;
      v *= this.sim.health ? this.sim.health.fx.speedK : 1; // FDIR derates (hub cooling, strut leak, battery string)
      // cargo shifted in the cradle (load-cell asymmetry): half speed until the cradle has re-centred it
      if (R.lift.shifted) {
        v *= 0.5;
        if (!this._shiftWarned) { this._shiftWarned = true; this.log('Cargo shift detected: speed derated 50% until the cradle re-centres the module', 'warn'); }
      } else if (this._shiftWarned) { this._shiftWarned = false; this.log('Cargo re-centred: speed derate lifted', 'ok'); }
      if (this.pathTight) v = Math.min(v, 0.6);
      // Tighten corners when tracking a proven route so sharp bends don't swing wide into unmapped terrain.
      if (usingRoute && !loaded && Math.abs(headErr) > 35 * DEG) v = Math.min(v, 1.0);
      v = Math.min(v, Math.max(0.3, Math.sqrt(2 * 0.12 * Math.max(0, dGoal - tol))));
      if (v <= 0) { this.stop(); this.detail = 'LiDAR failed off-route: holding for operator'; yield; continue; }
      // transit warp (>= 50x): a long cruise leg whose next 30 m are mapped and clear runs on rails (Sim.warpStep)
      { const vR = this._railsCheck(e, dGoal, v, loaded);
        this.railsOk = vR > 0;
        if (this.railsOk && this.sim.warping) {
          this._railsAdvance(vR, this.dt);
          this.detail = `TRANSIT WARP ${vR.toFixed(1)} m/s | ${this.pathSource} | ${dGoal.toFixed(0)} m to go`;
          stuckT = this.sim.time; stuckPos = this.est(); blockedT = 0;
          yield; continue;
        } }
      let lp = this.local.evaluate(e, tgt, v, loaded);
      // the gentle-turn arc set (|k| b <= 0.35-0.5) is the normal choice; in a tight spot try the full skid-steer arc
      // set at walking pace before declaring a block (v1.5.0: without this, the gentler arcs turned squeezes into
      // 25 m reverse "unwinds")
      if (!lp.ok) { const lp2 = this.local.evaluate(e, tgt, Math.min(v, 0.5), loaded, { kmax: loaded ? 0.11 : 0.2 }); if (lp2.ok) { lp = lp2; v = Math.min(v, 0.5); } }
      if (!lp.ok) { const lp3 = this.local.evaluate(e, tgt, Math.min(v, 0.35), loaded, { horizon: 6, kmax: loaded ? 0.13 : 0.25 }); if (lp3.ok) { lp = lp3; v = Math.min(v, 0.35); } }
      if (!lp.ok) {
        this.stop();
        const firstBlock = !blockedT;
        if (!blockedT) blockedT = this.sim.time;
        this.detail = 'local planner: no safe arc - learning obstacle and replanning';
        // put the actual obstacle points the arcs ran into into the global costmap
        if (!loaded) for (const h of lp.hits.slice(0, 30)) this.planner.blocked.push({ x: h.x, z: h.z, r: 1.6, t: this.sim.time, ttl: 180 });
        else if (firstBlock) {
          // loaded: the arcs sample only a narrow fan (~+-3 m at 7 m) and the 2.5 m costmap window reads a short
          // 11-12 deg face under the 10.5 deg limit, so A* side-stepped a few metres into the same face; and with a
          // 180 s memory it came back to it once the marks expired (east-west shuttling for minutes). Learn the hits
          // once per block for 15 min and sweep the wheel-scale hazard over the ground ahead (CHANGES.md §14)
          for (const h of lp.hits.slice(0, 30)) this.planner.blocked.push({ x: h.x, z: h.z, r: 1.6, t: this.sim.time, ttl: 900 });
          const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
          for (let f = 3; f <= 24; f += 1) for (let l = -15; l <= 15; l += 1) {
            const px = e.x + c * f + s * l, pz = e.z - s * f + c * l, wh = this.slam.wheelHazard(px, pz);
            if (wh && wh.lethal) this.planner.blocked.push({ x: px, z: pz, r: 0.9, t: this.sim.time, ttl: 900 });
          }
        }
        if (this.sim.time - blockedT > 1.0) {
          episodes++; blockedT = 0;
          if (episodes > 8) return fail('boxed in by obstacles: operator assistance required');
          if (episodes === 1) this.log('Local planner blocked: obstacle learned into the map, replanning', 'warn');
          if (episodes >= 2) {
            // boxed in: a replan alone cannot get out (or reverse arc is blocked), so back out along the way in
            if (++unwinds > 4) return fail('boxed in again after 4 reverse recoveries: operator assistance required');
            unwindBestD = Math.min(unwindBestD, dGoal);
            yield* this.unwindPath(25);
            episodes = 0; bestD = Infinity; bestT = this.sim.time; stuckT = this.sim.time; stuckPos = this.est(); usingRoute = false;
          } else {
            const okPlan = this.replan(goal, 'local block', true);
            let moved = false;
            if (okPlan && !R.loaded && this.local.pivotSafe(e)) {
              let ri = this.pathIdx, rAcc = 0;
              while (ri < this.path.length - 1 && rAcc < 12) { rAcc += Math.hypot(this.path[ri + 1].x - this.path[ri].x, this.path[ri + 1].z - this.path[ri].z); ri++; }
              const rp = this.path[ri];
              const rYaw = Math.atan2(-(rp.z - e.z), rp.x - e.x);
              if (Math.abs(wrap(rYaw - e.yaw)) > 35 * DEG) {
                this.stop(); for (let i = 0; i < 5; i++) yield;
                yield* this.alignYaw(rYaw, 10 * DEG, { rails: true });
                episodes = 0; moved = true;
              }
            }
            if (!moved) yield* this.recover(goal, 'blocked');
          }
          lastReplan = this.sim.time;
        }
        yield; continue;
      }
      blockedT = 0;
      // Two stopping-distance limits (braking at `dec` below, the same value the dynamics instruments use): stop 3 m
      // short of any known hazard, and never outrun the mapped ground - unseen terrain can hide a crater rim
      const dec = loaded ? 0.2 : 0.6; // loaded: under the ~0.4 m/s^2 nose-over limit of the passive pitch (CHANGES.md §14)
      // Unmapped (UNKNOWN) ground is not an obstacle: on flat ground (< NAV.flatDeg) the speed floor through the unmapped cells ahead
      // is NAV.unknownCoastSpeed for up to NAV.unknownCoastM metres while the next LiDAR sweeps fill them in, and only then the
      // 0.3 m/s creep. A mapped LETHAL / ROCK hit still brakes through freeDist exactly as before.
      unknownRun = lp.knownDist < 6 ? unknownRun + Math.hypot(e.x - lastPos.x, e.z - lastPos.z) : 0; lastPos = e;
      const tiltFlat = Math.max(Math.abs(R.tilt.pitch), Math.abs(R.tilt.roll)) * 180 / Math.PI < NAV.flatDeg;
      const coastFloor = tiltFlat && unknownRun < NAV.unknownCoastM && lp.freeDist >= lp.horizon - 1 ? NAV.unknownCoastSpeed : 0.3;
      let vCmd = Math.min(v, Math.max(0.3, Math.sqrt(2 * dec * Math.max(0, lp.freeDist - 3))));
      vCmd = Math.min(vCmd, Math.max(coastFloor, Math.sqrt(2 * dec * Math.max(0, lp.knownDist - 1.5))));
      { // tilt-aware speed: slow on tilted ground and so that the curve's lateral acceleration fits the remaining tip margin
        const mgn = R.tilt.marginF ?? R.tilt.margin, tiltDeg = Math.max(Math.abs(R.tilt.pitch), Math.abs(R.tilt.roll)) * 180 / Math.PI;
        const aOK = MOON.g * Math.tan(Math.min(60, Math.max(0.5, mgn - 9)) * DEG);
        vCmd = Math.min(vCmd, Math.max(0.35, Math.sqrt(aOK / Math.max(Math.abs(lp.k), 0.004))));
        // ride comfort / no skid in curves (v1.5.0): lateral acceleration <= 0.3 m/s^2 empty, 0.15 loaded (a skid-steer
        // turn scrubs the tyres sideways; slower = less scrub), and slow into a bend in proportion to the heading change
        vCmd = Math.min(vCmd, Math.max(0.5, Math.sqrt((loaded ? 0.15 : 0.3) / Math.max(Math.abs(lp.k), 0.002))));
        vCmd = Math.min(vCmd, Math.max(0.6, vCmd * (0.35 + 0.65 * Math.max(0, Math.cos(headErr)))));
        if (loaded) {
          // ground slope under the footprint from the map, not the body attitude: braking pitches the loaded body
          // forward (passive pitch), and slowing on that pitch braked it further into a nose-over (CHANGES.md §14)
          const c = Math.cos(e.yaw), s = Math.sin(e.yaw), H = (f, l) => this.slam.heightInterp(e.x + c * f + s * l, e.z - s * f + c * l);
          const gp = (H(4, 0) - H(-4, 0)) / 8, gr = (H(0, 3.5) - H(0, -3.5)) / 7;
          const gDeg = Number.isFinite(gp + gr) ? Math.atan(Math.hypot(gp, gr)) * 180 / Math.PI : tiltDeg;
          vCmd = Math.min(vCmd, Math.max(0.4, 3 - 0.22 * gDeg));
        }
      }
      // turbo (300 rpm) is only released on a straight, long, mapped, obstacle-free stretch
      R.turboOk = !loaded && !this.pathTight && Math.abs(lp.k) < 0.012 && lp.freeDist >= lp.horizon - 1 && lp.knownDist > 25 && dGoal > 50;
      // Yaw command shaping (smooth, no hunting): +-3 deg heading deadband (hysteresis to 4 deg) on a gentle path, damping of the
      // overshoot when nearly lined up but still rotating, and a yaw-acceleration limit of NAV.maxYawAccel on the command.
      let wzCmd = vCmd * lp.k;
      const hAbs = Math.abs(headErr);
      if (hAbs < NAV.headingDeadbandDeg * DEG) deadband = true; else if (hAbs > (NAV.headingDeadbandDeg + 1) * DEG) deadband = false;
      if (deadband && Math.abs(lp.k) < 0.02) wzCmd = 0;
      // damping only when it IS an overshoot in the making: nearly lined up on a straight stretch, and still rotating toward the
      // target faster than yawDampRate. (Without the direction and straight-path tests this also halved the yaw command on every
      // gentle curve, where the rover is supposed to be rotating: it under-steered, drifted past 5 deg and snapped back.)
      { const yr = this.sensors.imu.yawRate || 0;
        if (hAbs < NAV.yawDampErrDeg * DEG && Math.abs(lp.k) < 0.02 && Math.abs(yr) > NAV.yawDampRate && yr * headErr > 0) wzCmd *= NAV.yawDampGain; }
      { const w0 = R.cmd.wz, dw = NAV.maxYawAccel * this.dt; wzCmd = w0 + clamp(wzCmd - w0, -dw, dw); }
      this.cmd(vCmd, 0, wzCmd);
      this.detail = `${R.rpm.proto.toFixed(0)} rpm ${R.rpm.tier} | ${this.pathSource} | ${dGoal.toFixed(0)} m to go | v ${vCmd.toFixed(2)} m/s${lp.knownDist < 6 ? ' (creeping: unmapped ground ahead)' : ''} | ${Math.abs(lp.k) < 0.004 ? "straight" : `turn ${lp.k > 0 ? "L" : "R"} R=${Math.abs(1 / lp.k).toFixed(0)} m`}`;
      // stuck detection (embedding / slip / hidden obstacle)
      this._stuckPosRef = stuckPos;
      if (this.sim.time - stuckT > 20) {
        const moved = Math.hypot(e.x - stuckPos.x, e.z - stuckPos.z);
        if (moved < 1.0) {
          this.log('No progress for 20 s (slip or unseen obstacle): marking area and recovering', 'warn');
          this.planner.blocked.push({ x: e.x + Math.cos(e.yaw) * 9, z: e.z - Math.sin(e.yaw) * 9, r: 2.5, t: this.sim.time, ttl: 180 });
          yield* this.recover(goal, 'stuck');
        }
        stuckT = this.sim.time; stuckPos = this.est();
      }
      yield;
    }
    this._stuckPosRef = null;
    this.stop();
    if (this.slam.recording) {
      const r = this.slam.endRecord(true);
      if (r) {
        // straighten it over the ground this drive has mapped (Planner.straighten): later repeats skip the first drive's detours
        let note = '';
        if (NAV.straightenRoutes) {
          const P = this.planner;
          P.buildCostmap(this.slam.routes.filter((q) => q.ok && q !== r));
          const st = P.straighten(r.pts, R.loaded || !!this.surveyLimits);
          if (st && st.length < r.length - 20) {
            note = `, straightened to ${st.length.toFixed(0)} m (${st.cuts} shortcuts over mapped ground)`;
            r.raw = { length: r.length }; r.rawPts = r.pts; r.pts = st.pts; r.length = st.length;
          }
        }
        this.log(`Route memory: stored ${r.id} ${r.from}->${r.to} (${(r.raw?.length ?? r.length).toFixed(0)} m, ${(r.time / 60).toFixed(1)} min${note})`, 'ok');
      }
    }
    this.planner.blocked = this.planner.blocked.filter((b) => b.fault);
    if (goal.yaw != null) {
      // a loaded rover avoids point turns (CoM ~7 m up, wheels dig in); small heading errors are left to the
      // arc / line-following controller of the next leg
      const yErr = Math.abs(wrap(goal.yaw - this.est().yaw));
      if (!R.loaded || yErr > 6 * DEG) yield* this.alignYaw(goal.yaw, goal.yawTol ?? 1.5 * DEG);
    }
    return { ok: true };
  }

  // ---------------- transit warp (Sim.warpStep) ----------------
  // Cruise speed the rails may use, or 0 when this stretch must run on full physics: short of the goal, a tight path,
  // a shifted load, a low tip margin, scout out, or anything lethal / unmapped under the six wheel tracks in the
  // next 30 m of path. Cheap enough to run every tick (it is skipped below the warp time scale).
  _railsCheck(e, dGoal, v, loaded) {
    const S = this.sim, R = S.rover;
    if (!S.warpWanted()) return 0;
    if (dGoal < 45 || this.pathTight || !this.path || R.lift.shifted || R.estop || !R.powered || S.scout.enabled || this.sensors.faults.lidar) return 0;
    if ((R.tilt.marginF ?? R.tilt.margin) < 20) return 0;
    const P = this.path;
    let acc = Math.hypot(P[this.pathIdx].x - e.x, P[this.pathIdx].z - e.z);
    if (acc > 6) return 0; // off the path (recovering, or the plan just changed): let the tracker rejoin it
    const half = Math.abs(ROVER.corners[0][1]);
    let kapMax = 0;
    for (let i = this.pathIdx; i < P.length - 1 && acc < 30; i++) {
      const a = P[i], b = P[i + 1], L = Math.hypot(b.x - a.x, b.z - a.z);
      if (L < 1e-6) continue;
      const s = -(b.z - a.z) / L, c = (b.x - a.x) / L; // right vector of the segment = (sin yaw, cos yaw)
      if (i > this.pathIdx) {
        const p0 = P[i - 1], L0 = Math.hypot(a.x - p0.x, a.z - p0.z);
        if (L0 > 1e-4) {
          const dth = Math.abs(wrap(Math.atan2(-(b.z - a.z), b.x - a.x) - Math.atan2(-(a.z - p0.z), a.x - p0.x)));
          kapMax = Math.max(kapMax, dth / Math.max(0.5, 0.5 * (L + L0)));
        }
      }
      for (const off of [0, -half, half]) {
        const wh = this.slam.wheelHazard(a.x + s * off, a.z + c * off);
        if (!wh) { if (acc < 12 && !this.slam.pinhole(a.x + s * off, a.z + c * off)) return 0; continue; }
        if (wh.lethal) return 0;
      }
      acc += L;
    }
    const ssf = loaded ? 0.52 : 1.50;
    const vCurve = kapMax > 1e-4 ? Math.sqrt(MOON.g * 0.45 * ssf / kapMax) : 99;
    return clamp(v, 0, Math.min(loaded ? 2.4 : 4.2, vCurve));
  }
  // Move the estimate along the path by v*h and the true body by the same body-frame motion (so the SLAM frame
  // offset is preserved), feeding the odometry the SLAM filter integrates in Sim.warpStep.
  _railsAdvance(v, h) {
    const R = this.sim.rover, P = this.path, e = this.est();
    // projection of the estimate on the segment at pathIdx (or the one before it)
    let i = Math.min(this.pathIdx, P.length - 2), t = 0;
    for (const j of [i - 1, i]) {
      if (j < 0) continue;
      const a = P[j], b = P[j + 1], dx = b.x - a.x, dz = b.z - a.z, L2 = dx * dx + dz * dz;
      const tt = L2 > 0 ? clamp(((e.x - a.x) * dx + (e.z - a.z) * dz) / L2, 0, 1) : 0;
      if (j === i - 1 && tt < 1) { i = j; t = tt; break; }
      if (j === i) t = tt;
    }
    const walk = (i0, t0, d) => {
      let k = i0, x = P[k].x + (P[k + 1].x - P[k].x) * t0, z = P[k].z + (P[k + 1].z - P[k].z) * t0;
      while (d > 0 && k < P.length - 1) {
        const bx = P[k + 1].x, bz = P[k + 1].z, L = Math.hypot(bx - x, bz - z);
        if (L >= d) { x += (bx - x) * d / L; z += (bz - z) * d / L; d = 0; break; }
        d -= L; x = bx; z = bz; k++;
      }
      return { x, z, k: Math.min(k, P.length - 2) };
    };
    const n = walk(i, t, v * h), ahead = walk(n.k, 0, Math.hypot(P[n.k].x - n.x, P[n.k].z - n.z) + 4);
    const want = Math.atan2(-(ahead.z - n.z), ahead.x - n.x);
    // heading follows the path tangent, limited to the turning radius (loaded R >= 12 m, empty R >= 5 m)
    const maxTurn = v * h / (R.loaded ? 12 : 5);
    const yawN = e.yaw + clamp(Number.isFinite(want) ? wrap(want - e.yaw) : 0, -maxTurn, maxTurn);
    const dyaw = wrap(yawN - e.yaw), c = Math.cos(yawN), s = Math.sin(yawN);
    const Dx = n.x - e.x, Dz = n.z - e.z;
    const fx = Dx * c - Dz * s, rz = Dx * s + Dz * c;      // body-frame step (forward, right) in the new heading
    const T = R.pose(), yT = T.yaw + dyaw, cT = Math.cos(yT), sT = Math.sin(yT);
    R.railsPlace(T.x + fx * cT + rz * sT, T.z - fx * sT + rz * cT, yT, h, v);
    const o = this.sensors.odom, y1 = R.pose().y;
    o.dx += fx; o.dz += rz; o.dyaw += dyaw; o.dy += y1 - T.y; o.dist += Math.hypot(Dx, Dz); o.vx = v; o.vz = 0;
    const im = this.sensors.imu; im.yawRate = dyaw / h; im.roll = R.tilt.roll; im.pitch = R.tilt.pitch;
    this._railsMoved = true;
  }

  // Recovery: back out along the best collision-free reverse arc, then replan.
  *recover(goal, why) {
    const e = this.est();
    const target = { x: e.x - Math.cos(e.yaw) * 10, z: e.z + Math.sin(e.yaw) * 10 };
    let lp = this.local.evaluate(e, target, 0.3, this.sim.rover.loaded, { dir: -1, horizon: 8, kmax: 0.12, relaxed: true });
    if (!lp.ok) {
      for (const ang of [-0.35, 0.35, -0.7, 0.7]) {
        const y = e.yaw + ang;
        const tgt = { x: e.x - Math.cos(y) * 10, z: e.z + Math.sin(y) * 10 };
        const alt = this.local.evaluate(e, tgt, 0.3, this.sim.rover.loaded, { dir: -1, horizon: 8, kmax: 0.12, relaxed: true });
        if (alt.ok) { lp = alt; break; }
      }
    }
    const phase0 = this.phase.replace(/ \(recovery\)$/, '');
    this.phase = phase0 + ' (recovery)';
    if (lp.ok) {
      const s0 = this.est(), t0 = this.sim.time;
      while (Math.hypot(this.est().x - s0.x, this.est().z - s0.z) < Math.min(5, lp.freeDist - 1) && this.sim.time - t0 < 40) {
        this.cmd(-0.3, 0, -0.3 * lp.k); this.detail = `recovery (${why}): reversing on clear arc`; yield;
      }
      this.stop();
    } else this.log(`Recovery (${why}): no clear reverse arc either`, 'warn');
    for (let i = 0; i < 8; i++) yield;
    this.replan(goal, 'recovery', true);
    this.phase = phase0;
  }

  // The way back for an unwind: breadcrumbs from the newest backwards until `distance` m of path (only recent ones: older ones may
  // belong to another leg).
  _unwindTrace(e0, distance, now0) {
    const trace = [{ x: e0.x, z: e0.z }];
    let len = 0;
    // (v1.7.0) breadcrumbs recorded during an earlier unwind are the vehicle backing OUT; read backwards they lead back INTO that
    // pocket. A second box-in near the first one followed them back over the first pocket's rim, and a loaded rover tipped. So an
    // earlier unwind is jumped over, together with the stretch of the old entry path it already retraced: the trace resumes at the
    // pre-unwind breadcrumb nearest that unwind's end point, or stops if there is none.
    const traj = this.slam.trajectory, spans = (this._unwindSpans || []).filter((q) => now0 - q.t1 < 900);
    for (let i = traj.length - 1; i >= 0 && len < distance; i--) {
      const b = traj[i];
      if (now0 - b.t > 900) break;
      const sp = spans.find((q) => b.t >= q.t0 - 0.5 && b.t <= q.t1 + 0.5);
      if (sp) {
        let j = i; while (j >= 0 && traj[j].t >= sp.t0 - 0.5) j--;
        let best = -1, bd = 6;
        for (let q = j; q >= 0 && j - q < 120; q--) { const d = Math.hypot(traj[q].x - sp.end.x, traj[q].z - sp.end.z); if (d < bd) { bd = d; best = q; } }
        if (best < 0) break;
        i = best + 1; continue;
      }
      const a = trace[trace.length - 1], d = Math.hypot(b.x - a.x, b.z - a.z);
      if (d < 1.0) continue;
      trace.push({ x: b.x, z: b.z }); len += d;
    }
    return { trace, len };
  }

  // Reverse-recovery for a boxed-in rover (CHANGES.md §17). When the local planner cannot find a forward path
  // (e.g. boxed in by boulders or crater rims), the vehicle backs out along the breadcrumbs it drove in on (the proven
  // entry trajectory, which is clear because it just drove it), at 0.15 m/s. The rear arcs are checked every tick by
  // the local planner in reverse (the hazard-camera view astern); if one is blocked the unwind stops there. The corridor
  // ahead that boxed it in is marked lethal for 1200 s and the global planner is forced to replan from the new, open position.
  *unwindPath(distance = 25) {
    const R = this.sim.rover, P = this.planner, goal = this.goal;
    const phase0 = this.phase.replace(/ \((recovery|unwind)\)$/, '');
    this.phase = phase0 + ' (unwind)';
    this.unwindCount = (this.unwindCount || 0) + 1;
    const e0 = this.est(), TTL = 1200, now0 = this.sim.time;
    const repeatPocket = !!(this._lastUnwind && now0 - this._lastUnwind.t < 600 && Math.hypot(e0.x - this._lastUnwind.x, e0.z - this._lastUnwind.z) < 45);
    this._lastUnwind = { x: e0.x, z: e0.z, t: now0 };
    if (repeatPocket) distance = Math.max(distance, 35);
    // 1. the blocking corridor: the hit points the arcs ran into, plus a wide pocket around e0 and strip ahead
    for (const h of (this.local.last?.hits || []).slice(0, 30)) P.blocked.push({ x: h.x, z: h.z, r: 1.6, t: now0, ttl: TTL });
    const farGoal = !goal || Math.hypot(goal.x - e0.x, goal.z - e0.z) > 15;
    if (farGoal) {
      P.blocked.push({ x: e0.x, z: e0.z, r: repeatPocket ? 16.0 : 11.5, t: now0, ttl: TTL });
      if (repeatPocket) P.blocked.push({ x: e0.x - Math.cos(e0.yaw) * 10, z: e0.z + Math.sin(e0.yaw) * 10, r: 14.0, t: now0, ttl: TTL });
      for (let f = 4; f <= 30; f += 4) P.blocked.push({ x: e0.x + Math.cos(e0.yaw) * f, z: e0.z - Math.sin(e0.yaw) * f, r: 7.0, t: now0, ttl: TTL });
    } else {
      for (let f = 3; f <= 30; f += 3) P.blocked.push({ x: e0.x + Math.cos(e0.yaw) * f, z: e0.z - Math.sin(e0.yaw) * f, r: 4.5, t: now0, ttl: TTL });
    }
    // 2. the way in: breadcrumbs from the newest backwards until `distance` m of path (only recent ones: older ones
    // may belong to another leg)
    const { trace, len } = this._unwindTrace(e0, distance, now0);
    this.log(`Boxed in: unwinding ${Math.min(len, distance).toFixed(0)} m back along the entry trajectory at ${R.loaded ? 0.15 : 0.4} m/s, corridor ahead marked lethal for ${TTL} s`, 'warn');
    let moved = 0, ok = len >= 6, proven = 0;
    if (!ok) this.log('Unwind: no usable entry trajectory (breadcrumbs too short or too old)', 'warn');
    // empty: 0.4 m/s along its own fresh track (was 0.15: a 25 m unwind took ~3 min and dominated crater-field
    // exploration); loaded stays at 0.15 m/s (CoM 6.8 m up)
    const vU = R.loaded ? 0.15 : 0.4;
    const t0 = this.sim.time, tMax = Math.min(distance, len) / vU * 1.6 + 20;
    let ti = 1;
    const sampleElev = () => this._scanSample();
    let stallRef = { t: t0, e: e0, pts: sampleElev(), id: this.sensors.scanId };
    while (ok && moved < Math.min(distance, len) - 1 && this.sim.time - t0 < tMax) {
      const e = this.est();
      moved = Math.hypot(e.x - e0.x, e.z - e0.z);
      if (this.sim.time - stallRef.t > 18 && this.sensors.scanId !== stallRef.id) {
        const odomMove = Math.hypot(e.x - stallRef.e.x, e.z - stallRef.e.z);
        const curPts = sampleElev();
        if (odomMove > 1.8 && this._scanSame(stallRef.pts, curPts)) {
          this.log('Unwind: vehicle stationary against obstacle despite wheel rotation, stopping reverse', 'warn');
          break;
        }
        stallRef = { t: this.sim.time, e, pts: curPts, id: this.sensors.scanId };
      }
      // trace point to steer to: nearest one ahead of the progress so far, 5 m further back along the trace
      while (ti < trace.length - 1 && Math.hypot(trace[ti].x - e.x, trace[ti].z - e.z) < 5) ti++;
      const lp = this.local.evaluate(e, trace[ti], vU, R.loaded, { dir: -1, horizon: 8, kmax: 0.12, relaxed: true });
      let k = lp.k;
      if (!lp.ok) {
        // the rear arcs read blocked, but the trace is ground the wheels have just crossed: a crater rim mapped from
        // inside the pocket looks steeper than it drove (map 22-27 deg vs 14-17 deg true in tests/faults.mjs), so
        // follow the trace itself (pure pursuit, 0.15 m/s) while the tilt margin holds, and stop if it does not
        const c = Math.cos(e.yaw), s = Math.sin(e.yaw), tx = trace[ti].x - e.x, tz = trace[ti].z - e.z;
        const rT = tx * s + tz * c, d2 = Math.max(4, tx * tx + tz * tz);
        // raw tip margin, not the 0.8 s filtered one: reversing a loaded rover over a rim, the filtered value was still above 10 deg
        // when the vehicle had already gone over (v1.7.0 M6 run: tilt 36 deg, then 64 deg). Loaded stops at 15 deg (CoM 6.8 m up).
        const mgn = Math.min(R.tilt.margin, R.tilt.marginF ?? R.tilt.margin);
        if (mgn < (R.loaded ? 15 : 10)) { this.log('Unwind: tilt margin too small to keep reversing, stopping here', 'warn'); break; }
        k = Math.max(-0.12, Math.min(0.12, -2 * rT / d2));
        proven++;
      }
      this.cmd(-vU, 0, -vU * k);
      this.detail = `unwind: reversing ${moved.toFixed(1)} / ${Math.min(distance, len).toFixed(0)} m on the entry trajectory`;
      yield;
    }
    this.stop();
    { const eu = this.est(); (this._unwindSpans ||= []).push({ t0: now0, t1: this.sim.time, end: { x: eu.x, z: eu.z } }); if (this._unwindSpans.length > 20) this._unwindSpans.shift(); }
    for (let i = 0; i < 8; i++) yield;
    if (this.slam.recording) {
      const pts = this.slam.recording.pts, eu = this.est();
      let bestJ = -1, bestD = 12;
      for (let j = pts.length - 1; j >= 0; j--) {
        if (pts[j].t >= now0 - 2) continue;
        if (now0 - pts[j].t > 90 && bestJ >= 0) break;
        const d = Math.hypot(pts[j].x - eu.x, pts[j].z - eu.z);
        if (d < bestD) { bestD = d; bestJ = j; if (d < 4) break; }
      }
      if (bestJ >= 0) { pts.length = bestJ + 1; pts.push({ x: eu.x, z: eu.z, t: this.sim.time }); }
    }
    this._replanNow = true;
    if (goal) {
      this.replan(goal, 'unwind', true);
      if (!R.loaded && this.path?.length > 3) {
        const eu = this.est();
        if (this.local.pivotSafe(eu)) {
          let ri = 0, rAcc = 0;
          while (ri < this.path.length - 1 && rAcc < 12) { rAcc += Math.hypot(this.path[ri + 1].x - this.path[ri].x, this.path[ri + 1].z - this.path[ri].z); ri++; }
          const rp = this.path[ri], rYaw = Math.atan2(-(rp.z - eu.z), rp.x - eu.x);
          if (Math.abs(wrap(rYaw - eu.yaw)) > 25 * DEG) yield* this.alignYaw(rYaw, 10 * DEG, { rails: true });
        }
      }
    }
    this.phase = phase0;
  }

  // LiDAR scene sample (body frame x, z of raised returns 2-35 m out) and a same-scene test: >= 75 % of the earlier sample's
  // points still within 0.2 m. Used to tell a vehicle that is not moving although its wheels turn (pinned, spinning in place)
  _scanSample() {
    const sc = this.sensors.scan; if (!sc || !sc.n) return [];
    const pts = [], st = Math.max(1, Math.floor(sc.n / 180));
    for (let i = 0; i < sc.n; i += st) {
      const bx = sc.body[i * 3], by = sc.body[i * 3 + 1], bz = sc.body[i * 3 + 2];
      const r = Math.hypot(bx, bz);
      if (by > -0.2 && r > 2 && r < 35) pts.push(bx, bz);
    }
    return pts;
  }
  _scanSame(a, b) {
    if (!a || !b || a.length < 30 || b.length < 30) return false;
    let hit = 0; const tot = a.length / 2;
    for (let i = 0; i < a.length; i += 2) {
      const px = a[i], pz = a[i + 1];
      for (let j = 0; j < b.length; j += 2) { const dx = b[j] - px, dz = b[j + 1] - pz; if (dx * dx + dz * dz < 0.04) { hit++; break; } }
    }
    return hit >= 15 && hit / tot >= 0.75;
  }

  rearClear(e, ahead = 2.5, relaxed = false) {
    // check the strip swept by the rear of the vehicle before reversing
    // only the wheel tracks matter: the straddle bay passes over low objects (cargo, saddles)
    const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
    const hl = ROVER.frame.halfLen;
    for (let x = -hl; x >= -hl - ahead; x -= 0.5) for (const zz of [-4.4, -3.84, -3.3, 3.3, 3.84, 4.4]) {
      const wh = this.slam.wheelHazard(e.x + x * c + zz * s, e.z - x * s + zz * c, relaxed);
      if (wh && wh.lethal) return false;
    }
    return true;
  }
  // opts.relaxed: reversing out along the line the rover has just driven in on (the drive-under), so the tyres are retracing
  // their own tracks; the relaxed wheel-hazard limits (as unwindPath uses) apply. Returns the distance reversed.
  *backUp(dist, opts = {}) {
    const s0 = this.est(); const phase0 = this.phase.replace(/ \(recovery\)$/, ''); this.phase = phase0 + ' (recovery)';
    const t0 = this.sim.time, hold = opts.yaw;
    while (Math.hypot(this.est().x - s0.x, this.est().z - s0.z) < dist && this.sim.time - t0 < 60 + dist / 0.6) {
      if (!this.rearClear(this.est(), 2.5, !!opts.relaxed)) { this.stop(); this.log('Reverse stopped: obstacle behind the vehicle', 'warn'); break; }
      // (with a heading to hold: steer on it, so a long reverse out from under a module stays on the module axis)
      if (hold != null) this.cmdCurv(-0.6, -clamp(0.8 * wrap(hold - this.est().yaw), -0.05, 0.05));
      else this.cmd(-0.6, 0, 0);
      this.detail = 'recovery: reversing (rear strip clear)'; yield;
    }
    this.stop();
    this.phase = phase0;
    return Math.hypot(this.est().x - s0.x, this.est().z - s0.z);
  }

  // ---------------- cargo acquisition ----------------
  cargoGuess(c) {
    let g = this.cargoEst.get(c.id);
    if (!g) { // offload report from the lander (accurate to ~1.5 m / 3 deg)
      g = { x: c.initial.x + 0.9, z: c.initial.z - 0.7, yaw: c.initial.yaw + 0.03, t: -1, src: 'manifest' };
      this.cargoEst.set(c.id, g);
    }
    return g;
  }

  *acquire(c, opts = {}) {
    const R = this.sim.rover, T = c.T;
    try { return yield* this._acquire(c, opts); } finally { R.crawlZone = false; this.slam.lcSuspended = false; this.slam.matchSuspended = false; this.slam.mapping = true; }
  }
  // AEGIS-V1 pickup (module_carry.js): kneel, drive along the module axis under it until the clamp bands are
  // over the rings, RISE so the V-saddles lift it off the stand's lug notches, then close the ring jaws.
  *_acquire(c, opts = {}) {
    const R = this.sim.rover, T = c.T, RD = ROVER.ride;
    if (R.loaded) return { ok: false, why: 'already carrying cargo' };
    this._backedOut = true;
    const aft = Math.min(-T.length / 2, (T.adapterX || 0) - (T.adapterLen || 0) / 2);
    const carry = T.carryX || 0;
    // waits for the struts themselves (R.rideActual, strut position sensors), not only for the set point (R.lift.pos) to arrive;
    // 30 s limit so a strut that cannot reach its stroke (FDIR) fails the checks that follow instead of hanging the mission
    const waitRide = function* (self, target, label) {
      R.lift.target = target;
      const t0 = self.sim.time;
      while ((Math.abs(R.lift.pos - target) > 0.01 || Math.abs(R.rideActual - target) > 0.03) && self.sim.time - t0 < 30) { self.stop(); self.detail = `${label}: ride ${R.rideActual.toFixed(2)} -> ${target.toFixed(2)} m`; yield; }
      for (let i = 0; i < 20; i++) { self.stop(); yield; }
    };
    const backOut = function* (self) {
      const gc = self.cargoGuess(c);
      // clear overhead module returns from the 2.5D ground map (within +-4.38 m, leaving the +-5.63 m stand legs mapped)
      self.slam.clearFootprint(gc.x, gc.z, gc.yaw, -aft + 0.6, (T.adapterR || T.radius) + 0.4);
      // straight back out along the module axis, the line the drive-under came in on (v1.7.0: relaxed rear check and heading
      // held; the plain check stopped this reverse part-way under the pod rack, and the next attempt then raised the struts
      // and drove off wedged under the rack: no yaw response, spinning wheels, 15 m of odometry error)
      const need = -aft + ROVER.frame.halfLen + 4;
      let got = yield* self.backUp(need, { relaxed: true, yaw: gc.yaw });
      if (got < need - 1) { // still part-way under: try once more at a crawl after a pause (a transient return behind)
        yield* self.wait(2, 'backing out from under the module');
        got += yield* self.backUp(need - got, { relaxed: true, yaw: gc.yaw });
      }
      // clear of the module = the deck's front is behind the module's aft end (on its axis frame), whatever the reverse managed: an
      // attempt abandoned short of the module never had the deck under it, and must not stop the pick-up
      const e2 = self.est(), ax = Math.cos(gc.yaw), az = -Math.sin(gc.yaw);
      const front = (e2.x - gc.x) * ax + (e2.z - gc.z) * az + ROVER.frame.halfLen; // deck front on the module axis, from its centre
      const clear = front < aft - 0.3;
      if (!clear) self.log(`${c.id}: could only back out ${got.toFixed(1)} of ${need.toFixed(1)} m; deck front ${(front - aft).toFixed(1)} m past the module's aft end`, 'warn');
      return clear;
    };
    for (let attempt = 1; attempt <= 3; attempt++) {
      this.slam.lcSuspended = false; this.slam.matchSuspended = false; this.slam.mapping = true;
      R.crawlZone = false; // open-ground transit to the pre-grasp point runs at the smart-rpm ceiling
      // raise again only once the deck is out from under the module (the struts would press the deck into it)
      // (RD.cruise does not exist in ROVER.ride: the old test compared with undefined, never raised, and every retry re-approached
      // the module kneeled, 0.65 m lower, belly first. The driving ride height is 0.)
      if (attempt > 1 && Math.abs(R.lift.pos) > 0.02) {
        if (!this._backedOut) return { ok: false, why: `${c.id}: rover still under the module after a failed pickup: operator assistance required` };
        yield* waitRide(this, 0, 'raising to driving height');
      }
      let g = this.cargoGuess(c);
      // stand-off point on the module axis behind its aft end (rover front 14 m short of it)
      const pre = (gg, gap) => { const back = -aft + ROVER.frame.halfLen + gap - carry; return { x: gg.x - Math.cos(gg.yaw) * back, z: gg.z + Math.sin(gg.yaw) * back }; };
      this.phase = `M2 Approach ${c.id}`;
      const r = yield* this.navTo({ ...pre(g, 16), name: `pre-grasp ${c.id}`, tol: 2.0, site: 'PAD' });
      if (r && r.ok === false) return r;
      this.slam.lcSuspended = true; // the scene around the cargo is about to change (we move it)
      this.slam.matchSuspended = true; // avoid scan-to-map pulling pose onto shifted PAD/cargo cells during approach
      R.crawlZone = true; // the pre-grasp point is 16 m short of the module: everything from here (line-up, kneel, drive under, lift) is the 120 rpm crawl
      this.phase = `M2 Locate ${c.id} (fiducials)`;
      let t0 = this.sim.time;
      while (this.sim.time - t0 < 1.5) { this.stop(); this.detail = `cargo estimate from ${this.cargoGuess(c).src}`; yield; }
      g = this.cargoGuess(c);
      if (g.src === 'manifest') this.log(`${c.id}: no fiducial fix yet, using lander offload report`, 'warn');
      else this.detail = `cargo fix +-${Math.sqrt(g.varP).toFixed(2)} m, +-${(Math.sqrt(g.varY) / DEG).toFixed(2)} deg`;
      // line up on the axis (skid steer: turn, drive, turn)
      const p1 = pre(g, 12);
      this.phase = `M2 Line up on ${c.id}`;
      if (Math.hypot(p1.x - this.est().x, p1.z - this.est().z) > 0.6) {
        const e = this.est();
        yield* this.alignYaw(Math.atan2(-(p1.z - e.z), p1.x - e.x), 2 * DEG);
        const t1 = this.sim.time;
        while (this.sim.time - t1 < 120) {
          const rel = this.relBody(pre(this.cargoGuess(c), 12)); // re-aim as the fiducial fix sharpens
          if (rel.f < 0.15) break;
          this.cmd(clamp(rel.f * 0.5, 0.2, 1.8), 0, clamp(-Math.atan2(rel.r, Math.max(1, rel.f)) * 0.6, -0.08, 0.08));
          this.detail = `lining up on the module axis (${rel.f.toFixed(1)} m)`;
          yield;
        }
        this.stop();
      }
      this.phase = `M2 Square up to ${c.id}`;
      yield* this.alignYaw(this.cargoGuess(c).yaw, 0.6 * DEG);
      // kneel so the deck and saddles pass under the module on its stand (pause 2.5D mapping so the overhang doesn't paint a wall)
      this.phase = `M2 Kneel (${(RD.kneel).toFixed(2)} m)`;
      this.slam.mapping = false;
      yield* waitRide(this, RD.kneel, 'kneeling');
      // drive under: fiducials, then the saddle rangefinders + band-edge sensors
      this.phase = `M2 Drive under ${c.id}`;
      let ok = false; t0 = this.sim.time; let axF = null, cassLatF = null, cassYawF = null; this._wzI = 0;
      while (this.sim.time - t0 < 400) {
        g = this.cargoGuess(c);
        const e = this.est();
        const rel = this.relBody(g, e);
        // offsets from the module AXIS (not from its centre): rel is the centre in the rover frame, yawErr the axis
        // heading relative to the rover, so the centre's sideways offset alone hides along * sin(yawErr)
        let yawErr = wrap(g.yaw - e.yaw), src = 'camera';
        let lat = rel.f * Math.sin(yawErr) + rel.r * Math.cos(yawErr), along = rel.f * Math.cos(yawErr) - rel.r * Math.sin(yawErr) - carry;
        const cass = this.sensors.cass;
        if (cass && cass.valid) {
          const ac = clamp(this.dt / 0.4, 0, 1);
          cassLatF = cassLatF == null ? cass.lateral : cassLatF + (cass.lateral - cassLatF) * ac;
          cassYawF = cassYawF == null ? -cass.yaw : cassYawF + (-cass.yaw - cassYawF) * ac;
          lat = cassLatF; yawErr = cassYawF; src = 'saddle rangefinders';
          if (cass.cargoX != null) { axF = axF == null ? cass.cargoX : axF + (cass.cargoX - axF) * 0.3; along = axF - carry; src += ' + band sensors'; }
        }
        if (Math.abs(along) < 0.12 && Math.abs(lat) < 0.15 && Math.abs(yawErr) < 0.35 * DEG) {
          ok = true; break;
        }
        if (along < -0.5) break; // overshoot
        if (!(cass && cass.valid) && along < -aft + ROVER.frame.halfLen - carry + 0.5 && Math.abs(lat) > 0.25) { this.log(`${c.id}: lateral ${lat.toFixed(2)} m at the module end (tyre-to-post clearance ~0.9 m)`, 'warn'); break; }
        // (steering is curvature, i.e. per metre, so speed doesn't change the line the deck follows; the creep only
        // needs to be slow over the last ~0.5 m where the band sensors stop it inside +-0.12 m)
        const v = Math.min(along > 8 ? 1.5 : along > 2 ? 0.7 : clamp(along * 0.35, 0.06, 0.4), Math.abs(lat) > 0.12 ? 0.35 : 9, (cass && cass.valid && along < 1.5 && Math.abs(yawErr) > 0.35 * DEG) ? 0.12 : 9);
        // square the module cleanly on the V-saddles before clamping: once inside +-6 cm laterally and 2.2 m from
        // the band target, prioritize yaw alignment so saddle skew stays < 0.35 deg instead of twisting to chase cm
        const latTerm = (along < 2.2 && Math.abs(lat) < 0.06) ? 0.06 * lat : clamp(Math.atan(0.50 * lat), -0.14, 0.14);
        const kap = clamp(1.3 * yawErr - latTerm, -1 / 6, 1 / 6);
        this.cmdCurv(v, kap);
        this.detail = `[${src}] along ${along.toFixed(2)} m  lat ${(lat * 100).toFixed(1)} cm  yaw ${(yawErr / DEG).toFixed(2)} deg`;
        yield;
      }
      this.stop();
      for (let i = 0; i < 15; i++) yield;
      if (!ok) {
        this.log(`${c.id}: not centred under the module (attempt ${attempt}), backing out to retry`, 'warn');
        this._backedOut = yield* backOut(this);
        continue;
      }
      if (opts.approachOnly) { this.log(`${c.id}: rover centred under the module, ready to lift`, 'ok'); return { ok: true }; }
      // rise: saddles touch at the contact ride, weight transfers, the lugs lift clear of the notches
      this.phase = `M3 Rise under ${c.id}`;
      const y0 = c.body.translation().y;
      yield* waitRide(this, RD.carry, 'rising (roller-screw struts)');
      const lifted = c.body.translation().y - y0;
      if (lifted < 0.3) { this.log(`${c.id}: module did not lift off the stand (${lifted.toFixed(2)} m)`, 'warn'); yield* waitRide(this, RD.kneel, 'kneeling'); this._backedOut = yield* backOut(this); continue; }
      this.phase = `M3 Clamp ${c.id}`;
      const res = R.clampClose();
      if (!res.ok) { this.log(`Clamp attempt ${attempt} refused: ${res.why}`, 'warn'); yield* waitRide(this, RD.kneel, 'kneeling'); this._backedOut = yield* backOut(this); continue; }
      while (R.lift.clamp === 'closing') { this.detail = `ring jaws closing ${(R.lift.clampT * 100).toFixed(0)}%  (lugs ${lifted.toFixed(2)} m above the notches)`; yield; }
      // (was: retried straight away with the module still riding unclamped on the raised saddles, so the next attempt's
      // approach drive carried it off its stand. Set it back down and back out first, like the other failure paths.)
      if (R.lift.clamp !== 'closed') { this.log(`Clamp attempt ${attempt} failed: ${R.lift.msg}`, 'warn'); while (R.lift.clamp === 'opening') yield; yield* waitRide(this, RD.kneel, 'kneeling'); this._backedOut = yield* backOut(this); continue; }
      yield* this.wait(0.5, 'verifying load path');
      c.state = 'held';
      // the module is now part of the vehicle: forget its footprint in the terrain map
      { const e2 = this.est(); const half = (T.adapterR || T.radius) + 0.5;
        this.slam.clearFootprint(e2.x + Math.cos(e2.yaw) * carry, e2.z - Math.sin(e2.yaw) * carry, e2.yaw, -aft + 2.2, half);
        this.slam.mapping = true;
        this._needStandExit = { x: e2.x, z: e2.z, yaw: e2.yaw, dist: MISSION.standExitM }; }
      const st = this.manifest.find((m) => m.cargo === c); if (st) st.status = 'in transit';
      this.log(`${c.id} lifted ${lifted.toFixed(2)} m off its stand and clamped (${(T.mass * MOON.g / 1000).toFixed(1)} kN on the saddles)`, 'ok');
      return { ok: true };
    }
    return { ok: false, why: 'pickup failed after 3 attempts' };
  }

  // Mission 3 stability check: start / stop / turn with the load
  *stabilityCheck() {
    const R = this.sim.rover;
    this.phase = 'M3 Load stability check';
    // module position in the rover frame (cradle load cells / ring encoders): how far it moved during the test
    const inCradle = () => {
      const c = R.lift.cargo; if (!c) return null;
      const p = R.pose(), t = c.body.translation();
      return new THREE.Vector3(t.x - p.x, t.y - p.y, t.z - p.z).applyQuaternion(p.q.clone().invert());
    };
    const m0 = inCradle();
    const seq = [[0.5, 0, 0, 6], [0, 0, 0, 3], [0.5, 0, 0.04, 6], [0, 0, 0, 3], [-0.4, 0, 0, 5], [0, 0, 0, 3]];
    let minMargin = 99, maxShift = 0;
    for (const [vx, vz, wz, t] of seq) {
      const t0 = this.sim.time;
      while (this.sim.time - t0 < t) {
        this.cmd(vx, vz, wz); minMargin = Math.min(minMargin, R.tilt.margin);
        const m = inCradle(); if (m && m0) maxShift = Math.max(maxShift, m.distanceTo(m0));
        this.detail = `start/stop/turn test  tip margin ${R.tilt.margin.toFixed(1)} deg`; yield;
      }
    }
    this.stop();
    // (was: `R.lift.cargo ? 0 : 1`, a constant, never a measurement)
    const shift = m0 ? maxShift : NaN;
    this.log(`Load check: min tip margin ${minMargin.toFixed(1)} deg, cargo shift ${(shift * 100).toFixed(1)} cm - ${minMargin > 15 ? 'PASS' : 'MARGINAL'}`, minMargin > 15 ? 'ok' : 'warn');
    return { ok: true };
  }

  // ---------------- delivery & docking ----------------
  portGuess(port) {
    let g = this.portEst.get(port.id);
    if (!g) { g = { x: port.x, z: port.z, y: port.y, nYaw: Math.atan2(-port.nz, port.nx), t: -1, src: 'base map' }; }
    return g;
  }

  *deliver(portId) {
    try { return yield* this._deliver(portId); } finally { this.sim.rover.crawlZone = false; this.slam.lcSuspended = false; this.slam.matchSuspended = false; this.slam.mapping = true; }
  }
  *_deliver(portId) {
    const R = this.sim.rover;
    const port = this.sim.ports.find((p) => p.id === portId);
    if (!R.loaded) return { ok: false, why: 'no cargo held' };
    const c = R.lift.cargo, T = c.T;
    if (this._needStandExit) {
      const ex = this._needStandExit; this._needStandExit = null;
      const e0 = this.est(), RD = ROVER.ride, dir = MISSION.standExitDir;
      // metres travelled from the pick-up spot along the module axis, in the exit direction (+1 forward: toward the habitat, the
      // open end of the stand; see MISSION.standExitDir for why not reverse)
      const prog = () => { const e = this.est(); return dir * ((e.x - ex.x) * Math.cos(ex.yaw) - (e.z - ex.z) * Math.sin(ex.yaw)); };
      if (Math.hypot(e0.x - ex.x, e0.z - ex.z) < ex.dist + 2 && prog() < ex.dist) {
        this.phase = `Exit ${c.id} stand`;
        this.slam.lcSuspended = true;
        R.crawlZone = true;
        try {
          // 1. lift verification on the vehicle itself, not on the set point: struts at the carry stroke (position sensors), jaws
          //    locked. The 23 t vehicle does not move while the module's lugs can still be in the stand notches.
          const tV = this.sim.time;
          while (this.sim.time - tV < 20 && !(R.rideActual >= RD.carry - 0.04 && R.lift.clamp === 'closed')) {
            this.stop(); this.detail = `verifying lift before departure: struts ${R.rideActual.toFixed(2)} / ${RD.carry.toFixed(2)} m, jaws ${R.lift.clamp}`; yield;
          }
          // 2. straight out along the module axis with the heading held (curvature limited to MISSION.standExitKap: measured
          //    0.1 deg of heading change and 2 cm of lateral drift over the 12 m, 0 N of stand contact on the module). No turn
          //    starts inside the stand footprint; navTo takes over only once the rover centre is standExitM from the pick-up spot.
          const t0 = this.sim.time;
          while (prog() < ex.dist && this.sim.time - t0 < 90) {
            const e = this.est();
            this.cmdCurv(dir * MISSION.standExitSpeed, dir * clamp(0.8 * wrap(ex.yaw - e.yaw), -MISSION.standExitKap, MISSION.standExitKap)); // x dir: the yaw rate v * kap must correct the heading in reverse too
            this.detail = `leaving the stand, heading held (${Math.max(0, prog()).toFixed(1)} / ${ex.dist.toFixed(1)} m)`;
            yield;
          }
          this.stop();
        } finally { R.crawlZone = false; this.slam.lcSuspended = false; }
      }
    }
    if (this.faults.port[portId]) {
      this.log(`${port.name} reported UNAVAILABLE - holding at staging and requesting operator decision`, 'warn');
      const H = this.sim.terrain.hab;
      yield* this.navTo({ x: port.x + H.nx * 45, z: port.z + H.nz * 45, name: 'hab holding point', tol: 3 });
      this.phase = 'HOLD: receiving port unavailable';
      while (this.faults.port[portId] && c.dest === portId) { this.detail = `waiting: "clear port ${portId}" or "reroute ${c.id.slice(-1)} <port>"`; this.stop(); yield; }
      // operator redirected the module to another port while we held
      if (c.dest !== portId) { this.log(`${c.id} rerouted by operator: PORT-${portId} -> PORT-${c.dest}`, 'ok'); return yield* this._deliver(c.dest); }
    }
    const frontOff = () => R.lift.base.x + T.length / 2; // cargo front end ahead of rover centre
    const pg = this.portGuess(port);
    const n = { x: Math.cos(pg.nYaw), z: -Math.sin(pg.nYaw) };
    const heading = wrap(pg.nYaw + Math.PI);
    const pre = { x: pg.x + n.x * (frontOff() + 24), z: pg.z + n.z * (frontOff() + 24) };
    this.phase = `M4 Transport ${c.id} -> ${port.name}`;
    // lead-in point 14 m further out on the port axis: the grid planner ignores heading, so without it the path
    // reaches the pre-dock point from the side and a loaded rover (no point turns, R >= 9 m) can't line up there
    const lead = { x: pre.x + n.x * 14, z: pre.z + n.z * 14 };
    { // skip the lead-in only when already lined up on the axis between it and the pre-dock point
      const eL = this.est(), ax = (eL.x - pre.x) * n.x + (eL.z - pre.z) * n.z, off = Math.abs((eL.x - pre.x) * n.z - (eL.z - pre.z) * n.x);
      if (!(ax > 6 && ax < 16 && off < 2 && Math.abs(wrap(eL.yaw - heading)) < 10 * DEG)) {
        const rl = yield* this.navTo({ ...lead, name: `lead-in ${port.name}`, tol: 3.0, site: 'HAB' });
        if (rl && rl.ok === false) return rl;
      }
    }
    const r = yield* this.navTo({ ...pre, name: `pre-dock ${port.name}`, tol: 2.0, site: 'HAB', yaw: heading, yawTol: 1.0 * DEG, straight: true });
    if (r && r.ok === false) return r;
    // re-target on the port fiducial (the base-map position is offset by any SLAM drift)
    { const t0 = this.sim.time; while (this.sim.time - t0 < 1.5) { this.stop(); this.detail = 'acquiring port fiducial'; yield; }
      const g2 = this.portGuess(port);
      if (g2.t > 0) {
        const n2 = { x: Math.cos(g2.nYaw), z: -Math.sin(g2.nYaw) };
        const pre2 = { x: g2.x + n2.x * (frontOff() + 24), z: g2.z + n2.z * (frontOff() + 24) };
        const off = Math.hypot(pre2.x - this.est().x, pre2.z - this.est().z), eF = this.est();
        // the final approach line-follows the port axis from 24 m out; only a large cross-axis or heading
        // error needs a repositioning manoeuvre (a loaded rover has no point turns)
        const crossAx = Math.abs((eF.x - pre2.x) * n2.z - (eF.z - pre2.z) * n2.x), yawOff = Math.abs(wrap(wrap(g2.nYaw + Math.PI) - eF.yaw));
        // (the far-field leg steers on the rover's offset from the port axis and absorbs a few metres: an S-turn
        // at R 12 m needs ~2*sqrt(12*offset) of the 24 m leg; repositioning a 2.9 m offset put the goal inside
        // the loaded turning circle and the rover circled until the 4 min watchdog)
        if (off > 1.0 && (!R.loaded || crossAx > 3.5 || yawOff > 10 * DEG)) {
          this.log(`Port fiducial: pre-dock corrected by ${off.toFixed(1)} m (SLAM drift), repositioning`, 'info');
          const r2 = yield* this.navTo({ ...pre2, name: `pre-dock ${port.name} (fiducial)`, tol: 0.8, site: 'HAB', yaw: wrap(g2.nYaw + Math.PI), yawTol: 0.8 * DEG });
          if (r2 && r2.ok === false) return r2;
        }
      } else this.log(`${port.name} fiducial not seen yet: approaching on the base map`, 'warn');
    }
    // final approach: skid-steer line following on the port axis (port fiducial, then the docking-ring
    // camera inside 12 m). Lateral error is removed by heading while there is still distance to do it.
    this.slam.lcSuspended = true; // docking area changes as modules are berthed
    this.slam.matchSuspended = true; // avoid scan-to-map pulling pose onto shifted HAB/module cells during docking
    const RD = ROVER.ride;
    let captured = false, contactF = 0;
    this._dockPeakN = 0;
    for (let attempt = 1; attempt <= 3 && !captured; attempt++) {
      this._dockAttempts = attempt;
      this.phase = `M5 Final approach ${port.name}`;
      let t0 = this.sim.time, bad = false, vAppF = null, latA = null, lastSrc = null; this._wzI = 0;
      while (this.sim.time - t0 < 400) {
        const g = this.portGuess(port);
        const e = this.est();
        const ring = this.sensors.cass?.ring;
        const nn = { x: Math.cos(g.nYaw), z: -Math.sin(g.nYaw) };
        const target = { x: g.x + nn.x * frontOff(), z: g.z + nn.z * frontOff() };
        const rel = this.relBody(target, e);
        let lat = rel.r, along = rel.f, yawErr = wrap(wrap(g.nYaw + Math.PI) - e.yaw), src = 'port fiducial';
        R.crawlZone = along < MISSION.crawlDockM; // the last 20 m of vestibule entry run at the 120 rpm crawl cap
        if (ring && ring.port === port && ring.along < 12) {
          along = ring.along; src = 'docking-ring camera';
          if (ring.yaw != null) yawErr = ring.yaw; // module axis vs port axis, measured on the ring itself
          // ring.lateral is the port centre's offset in the MODULE frame (a bearing): it includes along*sin(yaw).
          // The nose's distance from the port AXIS is what the steering must null (driving in 1.8 deg off-axis
          // the two cancelled, the reading sat at -5 cm while the nose crossed the axis, and every attempt failed)
          lat = ring.lateral * Math.cos(yawErr) + ring.along * Math.sin(yawErr);
          // ride trim toward the ring height: filtered (~1 s) and slow (0.25 /s). Adding 8 % of the raw reading
          // every tick swung the ride 0.57-0.98 m, and the tilting module showed up as lateral drift at the nose
          vAppF = vAppF == null ? ring.vertical : vAppF + (ring.vertical - vAppF) * clamp(this.dt / 0.8, 0, 1);
          if (Math.abs(vAppF) > 0.002) R.lift.target = clamp(R.lift.target + vAppF * 0.45 * this.dt, RD.carry - 0.4, RD.max);
        }
        if (src !== lastSrc) { this._wzI = 0; latA = null; lastSrc = src; }
        // cone check on a ~1 s filtered lateral: the loaded module rocks (box20: +-0.6 deg pitch) and single
        // samples spiked past 6 cm while the mean sat within 2 cm, aborting every attempt
        latA = latA == null ? lat : latA + (lat - latA) * clamp(this.dt / 1.0, 0, 1);
        if (along < 1.2 && Math.abs(latA) > 0.06) { bad = true; break; }
        if (along < 0.22) break;
        const vx = along > 10 ? 1.2 : clamp((along - 0.15) * 0.25, 0.025, 0.8);
        // Inside the ring camera's range: lat is the nose's offset from the port axis (L = frontOff ahead of the
        // turning centre). Since d lat/ds = -(yaw + delta) + L*kap (delta = module-to-rover saddle skew) and
        // d yaw/ds = -kap, kap = -k*lat gives characteristic eq lambda^2 + k*L*lambda + k = 0 with critical
        // damping near k = (2/L)^2 (~0.10 /m^2, zeta ~ 0.92) AND zero steady-state lateral error regardless of delta.
        const near = src === 'docking-ring camera', Pfar = 0.25, L = frontOff(), pNear = Math.min(0.34, 2.6 / Math.max(L, 1));
        const hd = wrap(g.nYaw + Math.PI), ycFar = (target.x - e.x) * Math.sin(hd) + (target.z - e.z) * Math.cos(hd);
        const kap = near ? clamp(-pNear * pNear * (lat + L * yawErr) + 2 * 0.85 * pNear * yawErr, -1 / 12, 1 / 12)
          : clamp(2 * Pfar * yawErr - Pfar * Pfar * ycFar, -1 / 12, 1 / 12);
        this.cmdCurv(vx, kap);
        this.detail = `[${src}] range ${along.toFixed(2)} m  lat ${(lat * 100).toFixed(1)} cm  yaw ${(yawErr / DEG).toFixed(2)} deg  ride ${R.lift.pos.toFixed(2)} m`;
        yield;
      }
      if (bad) { this.stop(); for (let i = 0; i < 20; i++) yield; this.log(`${port.name}: lateral error too large for the coupling cone (attempt ${attempt}), backing off`, 'warn'); yield* this.backUp(14); continue; }
      // soft contact: creep at 1 cm/s with the ride trimmed to the ring until the coupling latches engage
      this.phase = `M5 Dock ${port.name}`;
      t0 = this.sim.time;
      // Continuous creep (never stopping: with the drive at zero the spring brakes set and the ~10 m high
      // module nose then sways 1-2 cm in yaw on the locked tyres without decaying). Ring lateral / vertical are
      // low-pass filtered (~1 s) against that sway; lateral is closed by heading over the remaining gap (a yaw
      // change moves the nose by frontOff x dyaw), vertical by trimming the ride while moving. The port face is
      // flat: the module end touches it rim-first unless square, and gentle push (< maxPushN) squares it up.
      let latF = null, vertF = null, yawF = null, lostT = 0; this._wzI = 0;
      // Force-impedance contact (CHANGES.md §17): inside 0.1 m the speed follows the contact force, v = clamp(Kf (Ft - F), -0.02, 0.015),
      // so the module settles on Ft = 1200 N (between softCaptureN 900 and maxPushN 6000) instead of creeping at a fixed rate.
      // First contact (> 200 N) ramps the speed target to zero within 0.1 s, so the tyres' kinetic energy cannot overshoot
      // maxPushN; the latches fire only after the ride trim has held the ring inside +-5 mm for 1.5 s.
      const IMP = { Ft: 1100, Kf: 0.000006, vMax: 0.012, vMin: -0.006, vTouch: 0.0008, touchN: 180, ramp: 0.12, tol: 0.005, dwell: 1.5 };
      let touchT = null, alignedT = 0, peakN = 0, fF = 0, spiked = false, pinT = 0, backoffUntil = 0;
      while (this.sim.time - t0 < 180) {
        const ring = this.sensors.cass?.ring;
        if (!ring) {
          // target lost (camera fault / occlusion): hold briefly, then abort instead of creeping blind
          this.stop(); lostT += this.dt; this.detail = `docking camera has no target (${lostT.toFixed(1)} s)`;
          if (lostT > 5) { this.log(`${port.name}: docking camera lost the port ring for 5 s, aborting docking`, 'err'); R.brake = false; yield* this.backUp(8); return { ok: false, why: 'docking visual sensor lost' }; }
          yield; continue;
        }
        lostT = 0;
        const a = clamp(this.dt / 0.45, 0, 1);
        const yNow = ring.lateral * Math.cos(ring.yaw ?? 0) + ring.along * Math.sin(ring.yaw ?? 0); // nose offset from the port axis
        latF = latF == null ? yNow : latF + (yNow - latF) * a;
        vertF = vertF == null ? ring.vertical : vertF + (ring.vertical - vertF) * a;
        yawF = yawF == null ? (ring.yaw ?? 0) : yawF + ((ring.yaw ?? 0) - yawF) * a;
        const f = this.sim.cargoForces.get(c);
        contactF = f ? f.port : 0;
        peakN = Math.max(peakN, contactF); this._dockPeakN = Math.max(this._dockPeakN || 0, peakN);
        fF += (contactF - fF) * clamp(this.dt / 0.25, 0, 1);   // 0.25 s low-pass (dt = 0.1 s would pass spikes unfiltered at tau = 0.1 s)
        // Freeze fine lift trim once in contact when already inside capture tolerance so vertical actuation does not bind into the vestibule wall
        if (Math.abs(vertF) > (fF > 150 ? 0.0048 : 0.002)) R.lift.target = clamp(R.lift.target + vertF * (fF > 150 ? 0.12 : 0.3) * this.dt, RD.carry - 0.4, RD.max);
        const now = this.sim.time;
        if (contactF > IMP.touchN && touchT == null) touchT = now;
        else if (touchT != null && fF < 0.5 * IMP.touchN) touchT = null;   // contact lost: the ramp re-arms for the next touch
        const aligned = Math.abs(vertF) < IMP.tol && Math.abs(latF) < IMP.tol;
        // If held off-center in sustained contact, back off a few cm to re-center free of wall friction
        if (fF > 150 && (Math.abs(vertF) > 0.006 || Math.abs(latF) > 0.006 || (fF > 650 && !aligned))) pinT += this.dt; else pinT = Math.max(0, pinT - 2 * this.dt);
        if (pinT > 3.5 && backoffUntil <= now) { backoffUntil = now + 3.5; pinT = 0; }
        const rimR = c.T.shape === 'cylinder' ? c.T.radius : c.T.width / 2;
        const rimGap = 0.022 + rimR * Math.sin(Math.abs(yawF || 0));
        let v, vCap = 0.025;
        if (backoffUntil > now && ring.along < 0.045) {
          v = -0.012;
          vCap = 0.025;
        } else if (ring.along > Math.max(0.08, rimGap + 0.04)) {
          // Smoothly taper below pitch-lunge speed so heavy payloads do not coast into the vestibule
          v = clamp(0.12 * (ring.along - rimGap), 0.008, 0.025);
        } else {
          // Taper down before first rim touch, pause gently if still off-center before contact, then taper with load so rigid-wall contacts stay < 2.2 kN
          const preAlign = ring.along > rimGap + 0.01 && fF < 80 && (Math.abs(latF) > 0.006 || Math.abs(vertF) > 0.006);
          const vContact = fF > 150 ? clamp(0.0009 * (1 - fF / 1200), 0.00025, 0.0009) : (touchT != null || fF > 80 ? 0.0009 : IMP.vTouch);
          const vGap = preAlign ? 0.002 : ring.along > rimGap ? Math.min(IMP.vMax, 0.22 * (ring.along - rimGap) + 0.0025) : vContact;
          vCap = vGap;
          const fCtl = Math.max(fF, 0.5 * contactF);
          v = clamp(IMP.Kf * (IMP.Ft - fCtl), -0.002, vGap);
          if (touchT != null && now - touchT < IMP.ramp) v *= clamp(1 - (now - touchT) / IMP.ramp, 0, 1);
        }
        if (contactF > 1600) v = Math.min(v, -0.001); // soft backstop well under the 3 kN limit without kicking off the port
        const kap = clamp(-0.10 * latF, -1 / 12, 1 / 12);
        if (ring.along > -0.05 && ring.along <= 0.40 && Math.abs(latF) > 0.0035 && (fF < IMP.touchN || (ring.along > 0.008 && fF < 550 && Math.abs(latF) > 0.0042))) {
          const wMax = ring.along < 0.05 ? 0.0003 : 0.0006;
          const wDes = clamp(-0.10 * latF / frontOff(), -wMax, wMax), err = wDes - this.sensors.imu.yawRate;
          const vA = backoffUntil > now && ring.along < 0.045 ? v : (ring.along > 0.12 ? Math.min(vCap, 0.012) : Math.max(2e-5, Math.min(v, vCap)));
          this._wzI = clamp((this._wzI || 0) * 0.992 + 0.3 * err * this.dt, -wMax, wMax);
          this.cmd(vA, 0, clamp(wDes + 1.5 * err + this._wzI, -wMax, wMax));
          v = vA;
        } else if ((fF >= 80 || backoffUntil > now || (ring.along > 0.025 && ring.along < 0.15)) && ring.along > 0.010 && Math.abs(yawF) > 0.22 * DEG && Math.abs(latF) <= 0.0042 && fF < 650) {
          // Square yaw before rim-first impact or during light pinches with tight lateral margin so nose swing does not break alignment
          const wMax = 0.0005;
          const wDes = clamp(0.22 * yawF, -wMax, wMax), err = wDes - this.sensors.imu.yawRate;
          this._wzI = clamp((this._wzI || 0) * 0.992 + 0.5 * err * this.dt, -wMax, wMax);
          const vHold = Math.abs(v) < 2e-5 ? 2e-5 : v;
          this.cmd(vHold, 0, clamp(wDes + 2.0 * err + this._wzI, -wMax, wMax));
        } else {
          const vHold = Math.abs(v) < 2e-5 ? 2e-5 : v;
          // Inside 8 cm of contact, do not apply the long-distance tail-swing curvature (it steers opposite yawF at latF ~ 0 and twists the pinned nose off-axis)
          if (ring.along < 0.08) this.cmd(vHold, 0, 0);
          else this.cmdCurv(vHold, kap);
        }
        this.detail = `gap ${ring.along.toFixed(3)} m | lat ${(latF * 1000).toFixed(1)} mm | vert ${(vertF * 1000).toFixed(1)} mm | yaw ${(yawF / DEG).toFixed(2)} deg | contact ${contactF.toFixed(0)} N (F ${fF.toFixed(0)} -> ${IMP.Ft}) v ${(v * 1000).toFixed(1)} mm/s`;
        // dwell: in contact at capture force and aligned, continuously
        if (contactF > 2500 && !spiked && (spiked = true)) this.log(`Dock contact spike ${contactF.toFixed(0)} N: gap ${ring.along.toFixed(3)} m, cmd ${(v * 1000).toFixed(1)} mm/s, filtered F ${fF.toFixed(0)} N, lat ${(latF * 1000).toFixed(1)} mm, wheel speed ${(R.telemetry().fwdSpeed * 1000).toFixed(1)} mm/s`, 'info');
        // (the 1.5 s count runs on the ring alignment alone: the contact force of a rigid port face bounces between 0 and the
        // target, and the latches then fire at the first sample above the soft-capture force once the dwell is over)
        // camera noise on the ring reading is ~3 mm: a single sample past the 5 mm limit used to restart the dwell, and with the
        // luck of the noise stream the dwell never completed (3 failed attempts). Hold the timer inside 1.6x the limit.
        const captureGapHold = (DOCK.captureGap ?? 0.015) + 0.01;
        const nearAligned = Math.abs(vertF) < 1.6 * IMP.tol && Math.abs(latF) < 1.6 * IMP.tol;
        const gapOk = ring.along < (DOCK.captureGap ?? 0.015);   // the bellows rests ~32 mm proud at the capture force, so a fixed 30 mm never held
        if (gapOk && aligned) alignedT += this.dt; else if (!(ring.along < captureGapHold && nearAligned)) alignedT = 0;
        if (alignedT >= IMP.dwell && ring.along < (DOCK.captureGap ?? 0.015) && fF > DOCK.softCaptureN && Math.abs(ring.lateral) < IMP.tol && Math.abs(ring.vertical) < IMP.tol) { captured = true; break; } // the latching sample itself is inside the envelope too
        yield;
      }
      this.stop(); R.brake = true;
      if (!captured) { this.log(`Soft capture attempt ${attempt} failed, backing off`, 'warn'); R.brake = false; yield* this.backUp(14); }
    }
    if (!captured) { this.log('Soft capture failed after 3 attempts', 'err'); return { ok: false, why: 'soft capture failed' }; }
    const ring = this.sensors.cass.ring;
    const acc = { port: port.name, lateralMm: ring.lateral * 1000, verticalMm: ring.vertical * 1000, yawDeg: 0, contactN: contactF, peakN: this._dockPeakN || 0, attempts: this._dockAttempts || 1 };
    // yaw error measured between cargo axis and port normal (from docking camera)
    { const rq = c.body.rotation(); const q = new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w); const ax = new THREE.Vector3(1, 0, 0).applyQuaternion(q); acc.yawDeg = wrap(Math.atan2(-ax.z, ax.x) - Math.atan2(port.nz, -port.nx)) / DEG; }
    this.metrics.accuracy.push(acc);
    this.sim.dockCapture(c, port);
    R.crawlZone = false; // soft capture done: clear of the approach zone
    this.log(`SOFT CAPTURE ${port.name}: lateral ${acc.lateralMm.toFixed(1)} mm, vertical ${acc.verticalMm.toFixed(1)} mm, yaw ${acc.yawDeg.toFixed(2)} deg`, 'ok');
    // berthing sequence: pressure boundary sealed before the module hatch opens
    // Order: the sealing faces are cleaned while they are still 30 mm apart (soft capture), THEN drawn together. (The
    // dust-shield pulse and sweep used to come after the seal was compressed, which traps regolith in the seal.)
    const steps = [
      ['Electrodynamic dust shield pulse + brush sweep of both seal faces (30 mm apart)', 2.5],
      ['Hard capture: latches draw the ring in, seal compressed 30 mm', 1.5],
      ['Dust-exclusion coupling: metal bellows shroud closed around the interface', 2],
      ['Leak check: vestibule pressure-decay test', 3],
      ['Equalising vestibule to 70.3 kPa', 3],
      ['Receiving cradle jacks extending under module', 2.5],
      ['Module hatch opening', 2],
    ];
    this.docking = { port, cargo: c, step: 0, steps: steps.map((s) => s[0]), pressure: 0, leakRate: 0 };
    let leakTries = 0;
    for (let i = 0; i < steps.length; i++) {
      this.docking.step = i;
      const t1 = this.sim.time;
      while (this.sim.time - t1 < steps[i][1]) {
        const k = (this.sim.time - t1) / steps[i][1];
        if (i === 3) { this.docking.pressure = 0.5; this.docking.leakRate = 0.0021 * (1 - k) + 0.0006; }
        if (i === 4) this.docking.pressure = 0.5 + k * 69.8;
        if (i === 5) this.sim.extendJacks(port, k);
        this.phase = `M5 Berthing ${port.name}`; this.detail = `${i + 1}/${steps.length} ${steps[i][0]}`;
        this.stop(); yield;
      }
      if (i === 3) {
        // The hatch interlock: no PASS, no equalisation and no hatch. A fouled seal (fault "seal") leaks; the response is
        // to back the seal off, clean and re-seat once, and after a second failure to hold with the hatch closed.
        if (this.faults.seal) this.docking.leakRate = 0.05;
        if (this.docking.leakRate > 0.01) {
          this.log(`Leak check FAIL: decay ${this.docking.leakRate.toFixed(4)} kPa/min (limit 0.01) - hatch stays closed`, 'err');
          this.docking.pressure = 0; leakTries++;
          if (leakTries >= 2) {
            yield* this.pauseForOperator(`${port.name}: seal leak after ${leakTries} attempts - module latched, vestibule vented, hatch closed. "clear seal" after service, then "resume" to re-seat and test again`);
            leakTries = 0;
          } else {
            this.log('Backing the seal off 30 mm: repeating the dust-shield pulse and sweep, then re-seating', 'warn');
            if (this.faults.seal !== 'hard') this.faults.seal = false;   // the second clean clears ordinary dust fouling
          }
          i = -1; continue;
        }
        this.log(`Leak check PASS: decay ${this.docking.leakRate.toFixed(4)} kPa/min (limit 0.01)`, 'ok');
        this.docking.leakPassT = this.sim.time;
      }
    }
    this.sim.openHatch(port);
    this.docking.step = steps.length;
    this.lastBerth = { port: port.name, cargo: c.id, leakPassT: this.docking.leakPassT, hatchT: this.sim.time, pressure: this.docking.pressure };
    // release and back out
    this.phase = `M5 Release ${c.id}`;
    const rel = R.clampOpen();
    if (!rel.ok) return { ok: false, why: rel.why };
    while (R.lift.clamp === 'opening') { this.detail = `cradle opening ${(R.lift.clampT * 100).toFixed(0)}%`; yield; }
    c.state = 'delivered'; c.deliveredAt = this.sim.time;
    port.occupied = c;
    const ms = this.manifest.find((m) => m.cargo === c); if (ms) ms.status = 'delivered';
    this.metrics.deliveries++;
    this.slam.setPayload(false);
    // Pause 2.5D mapping while kneeling and backing out under the berthed module + receiving jacks
    this.slam.mapping = false;
    this.phase = `M5 Kneel clear of ${c.id}`;
    R.lift.target = ROVER.ride.kneel;
    while (Math.abs(R.lift.pos - ROVER.ride.kneel) > 0.01) { this.stop(); this.detail = `kneeling: ride ${R.lift.pos.toFixed(2)} m (module on the receiving stand)`; yield; }
    R.brake = false;
    this.phase = 'Back out from dock';
    yield* this.backUp(frontOffSafe(T) + ROVER.frame.halfLen + 6);
    this.phase = `M5 Clear of ${port.name}`;
    R.lift.target = 0; // back to driving ride height
    while (Math.abs(R.lift.pos) > 0.02) { this.stop(); this.detail = `rising to drive height: ride ${R.lift.pos.toFixed(2)} m`; yield; }
    this.slam.mapping = true; this.slam.matchSuspended = false; this.slam.lcSuspended = false;
    this.docking = null;
    this.log(`${c.id} DELIVERED to ${port.name}`, 'ok');
    return { ok: true };
    function frontOffSafe(TT) { return TT.length / 2 + 6; }
  }

  // ---------------- Mission 11: AI logistics manager ----------------
  logisticsPlan() {
    const R = this.sim.rover, e = this.est();
    const items = this.manifest.filter((m) => m.status === 'waiting' && m.cargo.state === 'staged');
    const H = WORLD.habitat, P = WORLD.pad;
    const route = Math.hypot(H.x - P.x, H.z - P.z) * 1.15;
    const surveyNeed = (m) => {
      const port = this.sim.ports.find((p) => p.id === m.cargo.dest);
      return port && route >= 300 && !this.provenRoute(m.cargo.initial ?? e, port) ? (route * 2 * R.mass * MOON.g * 0.09 / 0.86 / 3600 + 1100 * (route * 2 / 2.5) / 3600) : 0;
    };
    const plan = items.map((m) => {
      const v = ROVER.speedLimitLoaded * 0.8;
      const toCargo = Math.hypot(m.cargo.initial.x - e.x, m.cargo.initial.z - e.z) * 1.2;
      const survWh = surveyNeed(m);
      const eta = (toCargo / 1.2 + route / v + 900) / 60;            // minutes incl. handling
      const energy = (route * (m.cargo.T.mass + R.mass) * MOON.g * 0.09 / 0.86 + toCargo * R.mass * MOON.g * 0.09 / 0.86) / 3600 + 1.1 * eta * 60 / 3600 * 1000 + survWh; // Wh
      const weight = PRIORITY[m.category] ?? m.priority;
      const score = weight * 100 - eta * 0.8 - energy / 500;
      return { ...m, eta, energy, score, feasible: energy < (R.power.wh - this.reserveSoc * ROVER.battery.capacityWh) };
    });
    plan.sort((a, b) => b.score - a.score);
    return plan;
  }

  // ---------------- mission scripts ----------------
  *missionLeaveLanding() {
    const S = this.sim.terrain.pad.staging, d = this.sim.terrain.pad.dir;
    this.phase = 'M1 Leave landing zone';
    const r = yield* this.navTo({ x: S.x, z: S.z, name: 'staging area', tol: 2.5, site: 'STAGING', yaw: Math.atan2(-d.z, d.x) });
    return r;
  }
  *missionApproach(c) { return yield* this.acquire(c, { approachOnly: true }); }
  *missionPickup(c) {
    const r = yield* this.acquire(c);
    if (r.ok === false) return r;
    return yield* this.stabilityCheck();
  }
  *missionTransport(portId) {
    const R = this.sim.rover;
    if (!R.loaded) return { ok: false, why: 'pick up cargo first (Mission 3)' };
    const port = this.sim.ports.find((p) => p.id === portId);
    const H = this.sim.terrain.hab;
    return yield* this.navTo({ x: port.x + H.nx * 40, z: port.z + H.nz * 40, name: `habitat approach ${port.name}`, tol: 3, site: 'HAB' });
  }
  *missionDeliver(portId) { return yield* this.deliver(portId); }
  // A proven route (teach & repeat) that joins within 150 m of `a` and of `b`
  provenRoute(a, b) {
    const near = (r, q) => r.pts.some((p) => Math.hypot(p.x - q.x, p.z - q.z) < 150);
    return this.slam.routes.find((r) => r.ok && r.pts.length >= 20 && near(r, a) && near(r, b)) || null;
  }

  // Survey (teach) drive. The first loaded leg to the habitat crosses ~2 km of unmapped crater field: a loaded rover
  // (no point turns, 10.5 deg slope limit) only finds each rim as it reaches it and got boxed in between rims until
  // the progress watchdog fired (CHANGES.md §14). So the empty rover drives the leg first, under the loaded
  // traversability limits, mapping it and recording a proven route; the loaded leg then repeats it (CHANGES.md §15).
  *surveyRoute(cargo) {
    const port = this.sim.ports.find((p) => p.id === cargo.dest);
    if (!port || this.sim.rover.loaded) return { ok: true };
    const pg = this.portGuess(port), n = { x: Math.cos(pg.nYaw), z: -Math.sin(pg.nYaw) };
    // end facing the port, so its fiducials are in the camera view for the drift fix below
    const goal = { x: pg.x + n.x * 45, z: pg.z + n.z * 45, name: `survey: ${port.name} approach`, tol: 4, site: 'HAB', yaw: wrap(pg.nYaw + Math.PI), yawTol: 2 * DEG };
    const start = this.est();
    if (Math.hypot(goal.x - start.x, goal.z - start.z) < 300 || this.provenRoute(cargo.initial ?? start, goal)) return { ok: true };
    const phase0 = this.phase;
    this.log(`Survey drive: mapping the route to ${port.name} empty, under the loaded limits, before carrying ${cargo.id}`, 'ok');
    this.surveyLimits = true;
    let r;
    try {
      r = yield* this.navTo(goal);
      if (r && r.ok === false) { this.log(`Survey drive to ${port.name} failed (${r.why}): the loaded leg will explore`, 'warn'); return { ok: true }; }
      // Hold on the port fiducials until the map-frame fix lands. It moves the map, the breadcrumbs and the route just
      // recorded back along the trajectory (Slam.shiftFrame), so they match the surveyed layout instead of carrying
      // the ~1 % odometry drift of the 4 km drive (CHANGES.md §16)
      // The port tags (0.55 m) are only readable inside ~47 m and the rover arrives with up to ~35 m of drift, so it may
      // be well outside that: after a short hold it creeps toward the port (never inside the 25 m fix exclusion) until
      // a fix lands, 60 s at most.
      const fix0 = this.slam.lastFix?.t ?? -1, tH = this.sim.time;
      const fixDone = () => (this.slam.lastFix?.t ?? -1) > fix0 || ((this.slam.lastFixCheck?.t ?? -1) > tH && (this.slam.lastFix?.t ?? -1) > tH - 60);
      while (this.sim.time - tH < 60 && !fixDone()) {
        const e = this.est(), g = this.portGuess(port), d = Math.hypot(g.x - e.x, g.z - e.z);
        if (this.sim.time - tH < 6 || d < 32) { this.stop(); this.detail = 'survey: holding on the port fiducials for a drift fix'; }
        else { this.cmd(0.6, 0, 0); this.detail = `survey: creeping toward ${port.name} to pick up its fiducials (${d.toFixed(0)} m)`; }
        yield;
      }
      this.stop();
      if ((this.slam.lastFix?.t ?? -1) > fix0) this.log(`Survey route corrected by the port fiducials: ${Math.hypot(this.slam.lastFix.dx, this.slam.lastFix.dz).toFixed(1)} m of drift removed along the route`, 'ok');
      else this.log('Survey: no fiducial fix at the port approach - the route keeps its odometry drift', 'warn');
      // the drive back is an ordinary empty drive (the pad apron itself reads lethal under the loaded limits)
      this.surveyLimits = false;
      const S = this.sim.terrain.pad.staging;
      r = yield* this.navTo({ x: S.x, z: S.z, name: 'survey: return to pad', tol: 4, site: 'STAGING' });
      if (r && r.ok === false) return r;
    } finally { this.surveyLimits = false; this.phase = phase0; }
    const rt = this.provenRoute(cargo.initial ?? start, goal);
    this.log(rt ? `Survey drive done: route ${rt.id} (${rt.length.toFixed(0)} m) proven for the loaded leg` : 'Survey drive done, but no route was recorded', rt ? 'ok' : 'warn');
    return { ok: true };
  }

  *missionCycle() {
    const plan0 = this.logisticsPlan();
    if (!plan0.length) return { ok: false, why: 'no cargo waiting at the pad' };
    this.log(`Logistics manager order: ${plan0.map((p) => `${p.id}[${p.category}]`).join(' -> ')}`, 'ok');
    while (true) {
      const plan = this.logisticsPlan();
      if (!plan.length) break;
      const item = plan[0];
      const legStart = { t: this.sim.time, d: this.sim.rover.odo.dist, e: this.sim.rover.power.energyUsedWh };
      // the plan's energy check: a leg it marks infeasible starts with a recharge (the dock can't charge a
      // loaded rover, so this is the last chance before the pick-up)
      if (!item.feasible && !this.sim.rover.loaded) {
        this.log(`${item.id}: leg needs ~${(item.energy / 1000).toFixed(0)} kWh above the ${(this.reserveSoc * 100).toFixed(0)}% reserve - recharging first`, 'warn');
        const rc = yield* this.chargeAtDock(0.9); if (rc && rc.ok === false) return rc;
      }
      // reserve check at the top of every leg: the next cargo is a drive back to the pad staging area and a loaded 2 km
      // transit, so a rover at or under the reserve recharges at the power dock first and then carries on by itself
      if (!this.sim.rover.loaded && this.sim.rover.power.soc <= this.reserveSoc) {
        this.log(`${item.id}: SoC ${(this.sim.rover.power.soc * 100).toFixed(0)}% is at the ${(this.reserveSoc * 100).toFixed(0)}% reserve - recharging at the power dock, then resuming`, 'warn');
        const rc = yield* this.chargeAtDock(0.9); if (rc && rc.ok === false) return rc;
      }
      // straight to the cargo (acquire: approach -> kneel -> rise -> clamp). The empty survey drive to the port and back
      // (surveyRoute) is opt-in: MISSION.preSurveyRoute
      let r = MISSION.preSurveyRoute ? yield* this.surveyRoute(item.cargo) : { ok: true }; if (r.ok === false) return r;
      r = yield* this.acquire(item.cargo); if (r.ok === false) return r;
      r = yield* this.deliver(item.cargo.dest); if (r.ok === false) return r; // dest, not the plan copy: "reroute" may change it
      this.metrics.legs.push({ id: item.id, time: this.sim.time - legStart.t, dist: this.sim.rover.odo.dist - legStart.d, energyWh: this.sim.rover.power.energyUsedWh - legStart.e });
    }
    const S = this.sim.terrain.pad.staging;
    if (!this.manifest.some((m) => m.status === 'waiting')) {
      // nothing left to carry: self-park in the power dock (HUNCH "examples of excellence": returns to a garage and recharges)
      this.phase = 'M6 Self-park at the power dock';
      const rc = yield* this.chargeAtDock(0.95);
      if (rc && rc.ok === false) this.log(`Self-park at the power dock failed: ${rc.why}`, 'warn');
      return { ok: true };
    }
    this.phase = 'M6 Return for next load';
    yield* this.navTo({ x: S.x, z: S.z, name: 'pad staging (ready for next load)', tol: 3, site: 'STAGING' });
    return { ok: true };
  }
  *missionGoto(x, z, name) {
    const res = yield* this.navTo({ x, z, name: name || `(${x.toFixed(0)}, ${z.toFixed(0)})`, tol: 2.5 });
    if (res && res.ok === false) return res;
    // Approaching habitat apron / base: ensure fiducials are acquired to collapse long-traverse odometry drift
    if (name === 'hab' || (name && name.toLowerCase().includes('hab')) || this.site(x, z) === 'HAB') {
      const H = this.sim.terrain.hab;
      const habYaw = wrap(Math.atan2(H.nz, -H.nx));
      if (!this.sim.rover.loaded) yield* this.alignYaw(habYaw, 4 * DEG);
      const fix0 = this.slam.lastFix?.t ?? -1, tH = this.sim.time;
      const fixDone = () => (this.slam.lastFix?.t ?? -1) > fix0 || ((this.slam.lastFixCheck?.t ?? -1) > tH && (this.slam.lastFix?.t ?? -1) > tH - 60);
      while (this.sim.time - tH < 60 && !fixDone()) {
        const e = this.est(), d = Math.hypot(WORLD.habitat.x - e.x, WORLD.habitat.z - e.z);
        if (this.sim.time - tH < 5 || d < 35) {
          this.stop(); this.detail = 'hab arrival: holding for fiducial fix';
        } else {
          this.cmd(0.6, 0, 0); this.detail = 'hab arrival: creeping to acquire habitat fiducials';
        }
        yield;
      }
      this.stop();
    }
    return { ok: true };
  }
}
