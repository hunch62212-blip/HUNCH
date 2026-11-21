// v1.7.0 checks that need no long drive. Each one measures the thing itself, not a proxy for it:
//   - ride height: the BODY's motion (strut extension), not the set point; and that the wheels are drawn where the physics has them
//   - LiDAR (CAD corner mount, 32 interlaced beams), self-occlusion mask, stereo depth cells (provisional, LiDAR overrides, no false hazards)
//   - time warp: budget logic on a fake clock, rails from 5x with frame-sized steps
//   - crawl-zone rpm cap, and that "Do Full Mission" goes straight to the cargo
import { check, atLeast, atMost, within, equal } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { Sim } from '../src/sim/sim.js';
import { ROVER, MISSION, PHYS, NAV, K, K_LEGACY } from '../src/sim/config.js';
import { CLS, RES, TILE } from '../src/sim/slam.js';
import { heightAt } from '../src/sim/terrain.js';
import { drawnRide, DRAWN_RIDE_RANGE } from '../src/render/aegis.js';
await RAPIER.init();
const sim = new Sim(RAPIER), R = sim.rover, S = sim.sensors, A = sim.autonomy, M = sim.slam;
{ const sp = sim.terrain.pad, p = R.pose(); R.teleport(p.x - sp.perp.x * 25, p.z - sp.perp.z * 25, Math.atan2(sp.perp.z, -sp.perp.x)); }
const run = (sec, fn) => { for (let i = 0; i < Math.round(sec * 120); i++) { sim.stepOnce(); fn && fn((i + 1) / 120); } };
run(4); R.brake = false;

// ---- 1. ride height: the body follows a smooth S-curve, and the drawn wheels follow the body
// drawn hub height (body frame, m) as aegis.js computes it on level ground, against the physical hub (hardpoint - strut length)
const drawErr = () => { const d = R.bogies.map((b) => (ROVER.bogie.Lnom + R.rideOffset - b.L) / K), vis = ROVER.wheel.radius - drawnRide(R.rideOffset, d) * K;
  let s = 0; for (const b of R.bogies) s += vis - (ROVER.bogie.hardpointY - b.L); return s / R.bogies.length; };
function stroke(target, sec = 9) {
  const y0 = R.rideActual; R.lift.target = target;
  const ys = [], dir = Math.sign(target - y0); let lag = 0, drawn = 0;
  run(sec, () => { ys.push(R.rideActual); lag = Math.max(lag, Math.abs(R.lift.pos - R.rideActual)); drawn = Math.max(drawn, Math.abs(drawErr())); });
  let settle = 0; for (let i = ys.length - 1; i >= 0; i--) if (Math.abs(ys[i] - target) > 0.02) { settle = (i + 1) / 120; break; }
  let aPk = 0, over = 0; const W = 12;
  for (let i = 2 * W; i < ys.length; i++) { aPk = Math.max(aPk, Math.abs(ys[i] - 2 * ys[i - W] + ys[i - 2 * W]) / 0.01); over = Math.max(over, dir * (ys[i] - target)); }
  return { settle, aPk, over, lag, drawn, v02: Math.abs(ys[23] - ys[11]) / 0.1 };
}
{
  const kn = stroke(ROVER.ride.kneel);
  // the S-curve duration is T = tFull x stroke / fullAt for strokes under fullAt (0.5 m): the 0 -> kneel stroke is now 0.326 m (k(0.040), was 0.65 m at K = 16.3), so T = 2.4 s and the 2 cm band (6 % of the stroke) is reached at ~0.77 T = 1.9 s
  within('squat-down 0 -> kneel: body settled within 2 cm (S-curve)', kn.settle, 1.5, 2.6, ' s', 2);
  atMost('squat-down: peak body acceleration (was a step in speed)', kn.aPk, 0.6, ' m/s2', 2);
  atMost('squat-down: body speed 0.2 s after the command (ease-in)', kn.v02, 0.05, ' m/s', 3);
  const up = stroke(ROVER.ride.carry);
  within('lift-up kneel -> carry: body settled within 2 cm (was 8.6 s)', up.settle, 3.0, 4.5, ' s', 2);
  atMost('lift-up: body overshoot', up.over, 0.02, ' m', 3);
  atMost('lift-up: peak body acceleration', up.aPk, 0.9, ' m/s2', 2);
  atMost('lift-up: body trails the set point by at most (was 0.9 m)', up.lag, 0.06, ' m', 3);
  atMost('lift-up: drawn wheel hub vs physical hub, worst instant (was 0.83 m)', up.drawn, 0.02, ' m', 3);
  const dn = stroke(ROVER.ride.kneel);
  within('squat-down carry -> kneel: body settled within 2 cm', dn.settle, 3.0, 4.5, ' s', 2);
  atMost('squat-down: drawn wheel hub vs physical hub, worst instant', dn.drawn, 0.02, ' m', 3);
  R.lift.target = 0; run(6);
  // a small trim (docking ride trim) must stay on the fast path
  const p0 = R.lift.pos; R.lift.target = p0 + 0.01; run(1);
  atMost('10 mm trim: set point there within 1 s (fast path)', Math.abs(R.lift.pos - (p0 + 0.01)), 0.0015, ' m', 4);
  R.lift.target = 0; run(3);
  // the drawn ride height is a real range clamp (the old cl(v, -0.045, 0.065) returned +0.045 for every input)
  const z = [0, 0, 0, 0, 0, 0];
  within('drawn ride at kneel', drawnRide(ROVER.ride.kneel, z) * K, ROVER.ride.kneel - 1e-6, ROVER.ride.kneel + 1e-6, ' m', 3);
  within('drawn ride at carry', drawnRide(ROVER.ride.carry, z) * K, ROVER.ride.carry - 1e-6, ROVER.ride.carry + 1e-6, ' m', 3);
  within('drawn ride at 0', drawnRide(0, z), -1e-9, 1e-9, '', 6);
  check('drawn ride clamps to the wheel-arch range', drawnRide(-5, z) === DRAWN_RIDE_RANGE[0] && drawnRide(5, z) === DRAWN_RIDE_RANGE[1]);
  within('drawn ride uses the struts, not the command (0.3 m of lag)', drawnRide(0.8, z.map(() => 0.3 / K)) * K, 0.499, 0.501, ' m', 3);
}

// ---- 2. LiDAR: 32 beams, 360 deg, on the team's CAD deck-corner mount, two interlaced sets of 16
equal('LiDAR channels', S.lidarCfg.channels, 32);
{ const m = ROVER.lidar.mountLocal;
  check('LiDAR on the CAD front-corner mount (lidar_mount.js option D), below the carried module', Math.abs(m.x - 0.25 * K) < 0.01 && Math.abs(Math.abs(m.z) - 0.15 * K) < 0.01 && m.y < ROVER.seatY, `x ${m.x} y ${m.y} z ${m.z}, module underside ${ROVER.seatY}`); }
S.lidarAuto = false; S.lidarCfg.rateHz = 5; S.setLidar(32, 240); S._partial = null;
{ const sets = []; const begin = S._beginScan.bind(S); S._beginScan = (n) => { begin(n); sets.push(S._partial.c0); }; run(1.2); S._beginScan = begin;
  check('interlaced firing: alternate sweeps fire the even and the odd beams', sets.length >= 4 && sets.every((c, i) => i === 0 || c !== sets[i - 1]), `first beam per sweep ${sets.join(',')}`); }
const sc = S.scan;
check('LiDAR produced a scan (16 of 32 beams per sweep)', !!sc && sc.n > 800 && sc.n <= 16 * 240, `${sc?.n} points`);
{ const q = [0, 0, 0, 0]; for (let i = 0; i < sc.n; i++) { const a = Math.atan2(sc.body[i * 3 + 2], sc.body[i * 3]); q[Math.floor(((a + Math.PI) / (2 * Math.PI)) * 4) % 4]++; }
  check('returns from all four 90 deg sectors (360 deg coverage)', q.every((n) => n > 60), `quadrants ${q.join(' / ')}`); }
S.lidarAuto = true;

// ---- 3. self-occlusion mask: chassis box and the carried module's cylinder are masked, terrain is not
{
  const p = R.pose(), W = (bx, by, bz) => new THREE.Vector3(bx, by, bz).applyQuaternion(p.q).add(new THREE.Vector3(p.x, p.y, p.z));
  const m0 = S._selfMask(p), pt = (v) => m0(v.x, v.y, v.z);
  // a point inside the chassis box, from the frame geometry (belly tub to deck), not hard-coded metres (the frame is k(0.30) x k(0.302) = 2.45 m half extents at K = 8.16)
  { const F = ROVER.frame; check('empty: chassis box point masked', pt(W(0.8 * F.halfLen, (F.tubY0 + F.deckY) / 2, 0.4 * F.halfWid)) === true); }
  check('empty: terrain 8 m ahead not masked', pt(W(8, -0.2, 0)) === false);
  check('empty: terrain under the chassis not masked', pt(W(0, -0.3, 0)) === false);
  const c = sim.cargo[0], T = c.T; const cargo0 = R.lift.cargo, joint0 = R.lift.joint, base0 = R.lift.base;
  R.lift.cargo = c; R.lift.joint = {}; R.lift.base = new THREE.Vector3(0, ROVER.seatY + T.radius, 0);
  const m1 = S._selfMask(p), q1 = (v) => m1(v.x, v.y, v.z);
  check('loaded: point inside the carried cylinder masked', q1(W(0, ROVER.seatY + T.radius, 1)) === true);
  check('loaded: point past the module end not masked', q1(W(T.length / 2 + 1.5, ROVER.seatY + T.radius, 0)) === false);
  check('loaded: terrain beside the vehicle not masked', q1(W(0, -0.3, 9)) === false);
  R.lift.cargo = cargo0; R.lift.joint = joint0; R.lift.base = base0;
}

// ---- 4. stereo depth: provisional cells, averaged, replaced by the LiDAR, never overriding it; no false hazards on its own
{
  equal('stereo enabled', ROVER.camera.stereo, true);
  equal('camera HFOV', ROVER.camera.hFovDeg, 90);
  M.clear(); M.relocalize(R.pose());
  const e = M.est, c = Math.cos(e.yaw), s = Math.sin(e.yaw);
  const body = (wx, wz, y) => { const dx = wx - e.x, dz = wz - e.z; return new Float32Array([dx * c - dz * s, y - e.y, dx * s + dz * c]); }; // world -> body (level)
  const wx = e.x + 40, wz = e.z + 40, cell = () => { const ix = Math.floor(wx / RES), iz = Math.floor(wz / RES), t = M.getTile(Math.floor(ix / TILE), Math.floor(iz / TILE), true), i = (iz - Math.floor(iz / TILE) * TILE) * TILE + (ix - Math.floor(ix / TILE) * TILE); return { h: t.h[i], n: t.n[i], st: t.st[i] }; };
  const sig = (v) => new Float32Array([v]);
  M.integrate({ n: 1, body: body(wx, wz, 5.0), sigma: sig(0.5), roll: 0, pitch: 0 });
  equal('stereo: a point over the sigma gate is not fused', cell().n, 0);
  M.integrate({ n: 1, body: body(wx, wz, 5.0), sigma: sig(0.05), roll: 0, pitch: 0 });
  check('stereo: first point makes a provisional cell (n 1, st 1)', cell().n === 1 && cell().st === 1 && Math.abs(cell().h - 5.0) < 1e-4, JSON.stringify(cell()));
  M.integrate({ n: 1, body: body(wx, wz, 5.2), sigma: sig(0.05), roll: 0, pitch: 0 });
  check('stereo: second point averages in, still provisional and n stays 1 (never a scan-matching reference)', Math.abs(cell().h - 5.1) < 1e-3 && cell().st === 2 && cell().n === 1, JSON.stringify(cell()));
  M.insert(wx, 7.0, wz);
  check('LiDAR return replaces a provisional cell outright', Math.abs(cell().h - 7.0) < 1e-5 && cell().st === 0 && cell().n === 1, JSON.stringify(cell()));
  M.integrate({ n: 1, body: body(wx, wz, 5.0), sigma: sig(0.01), roll: 0, pitch: 0 });
  check('stereo never changes a LiDAR-measured cell', Math.abs(cell().h - 7.0) < 1e-5 && cell().st === 0, JSON.stringify(cell()));
  // stereo alone (LiDAR off), 20 s of driving: the map it builds must be clean ground, not noise read as rocks and slopes
  M.clear(); M.relocalize(R.pose()); S.lidarOn = false; S._partial = null;
  let frames = 0, lastT = -1; R.cmd.vx = 1.5; run(20, () => { if (S.stereo && S.stereo.t !== lastT) { lastT = S.stereo.t; frames++; } }); R.cmd.vx = 0; run(2);
  const off = { x: M.est.x - R.pose().x, z: M.est.z - R.pose().z, y: M.est.y - R.pose().y }; let prov = 0, bad = 0, e2 = 0;
  for (const t of M.tiles.values()) { if (t.dirty) M.computeHazard(t);
    for (let i = 0; i < t.n.length; i++) { if (!t.n[i] || !t.st[i]) continue; prov++; if ([CLS.ROCK, CLS.SLOPE, CLS.LETHAL, CLS.CRATER].includes(t.cls[i])) bad++;
      const x = (t.tx * TILE + (i % TILE) + 0.5) * RES - off.x, z = (t.tz * TILE + Math.floor(i / TILE) + 0.5) * RES - off.z; e2 += (t.h[i] - off.y - heightAt(sim.terrain, x, z)) ** 2; } }
  S.lidarOn = true;
  atLeast('stereo depth frames in 20 s', frames, 40);
  atLeast('stereo-only map: cells mapped', prov, 1000);
  atMost('stereo-only map: cells read as rock / slope / lethal / crater on open ground (false hazards)', bad / Math.max(1, prov), 0.005, '', 4);
  atMost('stereo-only map: height error RMS against the true terrain', Math.sqrt(e2 / Math.max(1, prov)), 0.10, ' m', 3);
  run(3);
}

// ---- 5. time warp (deterministic: a fake clock charges 2 ms per physics step, so this tests the budget logic, not this machine's CPU)
{
  const realNow = performance.now.bind(performance), realStep = sim.stepAuto.bind(sim); let fake = 0;
  performance.now = () => fake; sim.stepAuto = (o) => { fake += 2; return realStep(o); };
  const owed = (scale) => { const sx = sim.time; sim.timeScale = scale; sim._acc = 0; sim.advance(0.1, 20); return sim.time - sx; };
  S.lidarOn = false;
  const t1 = owed(1), t2 = owed(2), t20 = owed(20);
  performance.now = realNow; sim.stepAuto = realStep; sim.timeScale = 1; S.lidarOn = true;
  // the frame owes 0.1 s x scale; a flat 20 ms budget at 2 ms / step paid 10 steps = 0.083 s whatever the scale
  within('1x: frame is fully paid (0.1 s owed)', t1, 0.08, 0.13, ' s', 3);
  within('2x: budget doubles, frame fully paid (0.2 s owed)', t2, 0.16, 0.22, ' s', 3);
  within(`20x: budget capped at ${PHYS.warpBudgetMaxMs} ms = 25 steps (frame rate kept; 2 s owed)`, t20, 0.19, 0.23, ' s', 3);
  equal('transit-warp rails available from 5x (was 50x)', sim.warp.minScale, 5);
  // rails steps are sized to what the frame owes, between one physics step and warp.H
  const hs = [], realWarp = sim.warpStep.bind(sim), ok0 = Object.getOwnPropertyDescriptor(A, 'railsOk');
  sim.warpStep = (h) => { hs.push(h); return { dt: h, autoRan: true }; }; sim.timeScale = 10; A.railsOk = true;
  sim.stepAuto(0.1); A.railsOk = true; sim.stepAuto(2); A.railsOk = true; sim.stepAuto(0.0001); A.railsOk = true; sim.stepAuto();
  sim.warpStep = realWarp; sim.timeScale = 1; A.railsOk = false; sim.warp.active = false;
  check('rails step = clamp(owed, one physics step, 0.5 s)', Math.abs(hs[0] - 0.1) < 1e-9 && hs[1] === sim.warp.H && Math.abs(hs[2] - PHYS.dt) < 1e-9 && hs[3] === sim.warp.H, hs.map((h) => h.toFixed(4)).join(', '));
  sim.timeScale = 3; A.railsOk = true; const before = sim.stepCount; sim.stepAuto(0.1); sim.timeScale = 1; A.railsOk = false;
  equal('below 5x the full physics always runs', sim.stepCount - before, 1);
}

// ---- 6. crawl zone caps the smart-rpm at 120, and releases
{
  R.cmd.vx = 1.5; run(6); R.crawlZone = true; run(6);
  within('crawl zone: rpm ceiling', R.rpm.want, 119, 121, ' rpm', 1);
  R.crawlZone = false; run(10); atLeast('crawl zone released: rpm rises again', R.rpm.proto, 130, ' rpm', 0);
  R.cmd.vx = 0; run(2);
}

// ---- 7. "Do Full Mission": pre-survey drive (on since the review pass: loaded tip margin too low to explore) or straight to the cargo
{
  equal('pre-survey drive is on (loaded trips use routes driven empty first)', MISSION.preSurveyRoute, true);
  equal('stand exit is forward (toward the habitat)', MISSION.standExitDir, 1);
  A.start('cycle', A.missionCycle());
  let seen = ''; for (let i = 0; i < 120 * 20 && A.gen; i++) { sim.stepOnce(); if (A.goal && !seen) seen = A.goal.name || ''; }
  if (MISSION.preSurveyRoute) check('first navigation goal is the survey approach to the port (route driven empty)', /survey/i.test(seen) || /survey/i.test(A.phase), `goal "${seen}", phase "${A.phase}"`);
  else {
    check('first navigation goal is the cargo pre-grasp point', /pre-grasp/.test(seen), `goal "${seen}"`);
    check('no survey phase', !/survey/i.test(A.phase) && !/survey/i.test(seen), `phase "${A.phase}"`);
  }
  A.abort('test over');
}
// ---- 8. navigation constants (smoothing)
check('nav constants', NAV.lookaheadMin >= 10 && NAV.headingDeadbandDeg === 3 && NAV.maxYawAccel === 0.1 && NAV.inflateLoaded === 6.5 && NAV.inflateEmpty === +(5.8 * K / K_LEGACY).toFixed(3)); // empty inflation is the legacy 5.8 m x KS (loaded 6.5 m keeps the 12.19 m module)
