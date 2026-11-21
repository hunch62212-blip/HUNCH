// P3a: autonomous two-arm ACM pick-up end to end (docs/ARM_CARGO_BUILD_PLAN.md s6.4, s6.6), headless.
//   1. ACM on regolith 5-8 m past the rover's nose with a seeded small lateral offset and yaw -> square up, park, deploy, pre-lift
//      check, grasp, latch, breakout, swing, place on the deck pins, deck latch, release, stow, drive 20 m with it on the deck.
//   2. Negative: an 8000 kg ACM is refused before any lift, the arms stow, the ACM is untouched.
//   3. Place-to-ground: deliverArm puts it back down at a receiving point.
// Precision tolerance on the deck (what the pins allow): the pre-seat error must be inside the pin capture (98 mm radial,
// yaw inside the taper (pin span)), and after seating the body sits on the exact seated pose: <= 5 mm and <= 0.2 deg
// measured from the rigid-body poses (Rapier float32 grid is ~0.5 mm at these coordinates).
import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { check, within, atMost, atLeast } from './lib/check.mjs';
import { Sim } from '../src/sim/sim.js';
import { ROVER, MOON, ARM, ACM_PICK, ACM_SOIL } from '../src/sim/config.js';
await RAPIER.init();
const DEG = Math.PI / 180, VERBOSE = !!process.env.VERBOSE, ONLY = process.env.ONLY;     // ONLY=pick|heavy|stuck|drop runs one section
let seed = Number(process.env.SEED || 7);
const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };

function setup(mass, label) {
  const sim = new Sim(RAPIER), R = sim.rover, A = sim.autonomy;
  sim.on('log', (e) => { if (VERBOSE || /ACM|arm|Mission/.test(e.msg)) console.log(`   [${label} ${e.t.toFixed(0).padStart(5)} s] ${e.msg}`); });
  // rover on the flat pad area, heading along the pad axis; the ACM 5-8 m past the nose, +-0.6 m off the axis, +-4 deg yaw
  const P = sim.terrain.pad.roverStart, yaw = P.yaw;
  R.teleport(P.x, P.z, yaw); for (let i = 0; i < 480; i++) sim.stepOnce();
  const gap = 5 + 3 * rnd(), off = (rnd() - 0.5) * 1.2, dyaw = (rnd() - 0.5) * 8 * DEG;
  const fwd = { x: Math.cos(yaw), z: -Math.sin(yaw) }, rgt = { x: Math.sin(yaw), z: Math.cos(yaw) };
  const d = ROVER.frame.halfLen + gap;
  const c = sim.acm.spawn(P.x + fwd.x * d + rgt.x * off, P.z + fwd.z * d + rgt.z * off, yaw + dyaw, { mass });
  for (let i = 0; i < 360; i++) sim.stepOnce();
  R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose());
  return { sim, R, A, c, gap, off, dyaw };
}
const runGen = (sim, A, name, gen, maxS) => {
  A.start(name, gen); const t0 = sim.time;
  while (A.gen && sim.time - t0 < maxS) sim.stepOnce();
  return { done: !A.gen, dur: sim.time - t0, last: A.history[A.history.length - 1] };
};

// ---------------- 1. full pick ----------------
if (!ONLY || ONLY === 'pick') {
  const { sim, R, A, c, gap, off, dyaw } = setup(2400, 'pick');
  console.log(`  setup: ACM ${gap.toFixed(1)} m past the nose, lateral ${(off * 100).toFixed(0)} cm, yaw ${(dyaw / DEG).toFixed(1)} deg, ${c.T.mass} kg`);
  const W = c.T.mass * MOON.g, arm = R.arm;
  let maxTip = 0, minMargin = 99, faults = new Set(), carriedSeen = false, maxExtra = 0;
  const watch = () => { maxTip = Math.max(maxTip, arm.tip.ratio); if (arm.mode === 'CARRY') minMargin = Math.min(minMargin, R.tilt.marginF ?? 99); for (const f of arm.faults) faults.add(f); if (c.mode === 'CARRIED') carriedSeen = true; maxExtra = Math.max(maxExtra, R.extraMass); };
  A.start('ACM pick', A.missionPickupArm(c)); const t0 = sim.time;
  let tDeck = null;
  while (A.gen && sim.time - t0 < 900) { sim.stepOnce(); watch(); if (tDeck == null && c.mode === 'DECK') tDeck = sim.time - t0; }
  const res = A.history[A.history.length - 1], st = A.acmStats;
  check('pick mission completes OK', !A.gen && res && res.ok, res ? JSON.stringify({ ok: res.ok, time: Math.round(res.time) }) : 'did not finish');
  if (!(res && res.ok)) { console.log('   last:', A.phase, '|', A.detail); process.exit(1); }
  const dur = tDeck;
  atMost('pick completes within 600 s of sim time (to the pins)', dur, 600, ' s', 0);
  console.log(`  pick duration to the pins ${dur.toFixed(0)} s, whole mission ${res.time.toFixed(0)} s`);
  check('both hooks latched and seated before the lift', st.latchedBeforeLift);
  check('ACM was carried (CARRIED state seen) and ends on the deck', carriedSeen && c.mode === 'DECK' && c.onDeck);
  check('no arm faults', faults.size === 0 && arm.faults.size === 0, [...faults].join(','));
  check('no hook-trunnion interference during the pick (hooks never pass through an un-seated trunnion)', arm.strikes === 0, `strikes ${arm.strikes}`);
  atMost('max tip ratio during the pick', maxTip, 0.75, '', 3);
  check('breakout: soil load reached ~W and fell to 0 at lift-off', st.peakGround > 0.9 && Math.abs(st.groundAtLift) < 1e-6, `peak ${st.peakGround.toFixed(3)} W, at lift-off ${st.groundAtLift} N`);
  atMost('breakout: hook peak (soil suction + hook dynamics) / W', st.peakHook, 1.25, '', 3);
  atLeast('min tip margin while carrying (deg)', minMargin, ARM.minMarginDeg, ' deg', 1);
  check('ACM weight reached the rover (extra mass) while carried', maxExtra > c.T.mass, `${maxExtra.toFixed(0)} kg`);
  // placement on the pins
  const se = st.seatErr;
  atMost('pre-seat error inside the pin capture (radial)', se.radial * 1000, 98, ' mm', 1);
  atMost('pre-seat yaw inside the pin taper', Math.abs(se.yaw) / DEG, (c.seat?.yawMax ?? 0.0841) / DEG, ' deg', 2);
  const rel = sim.acm.relPose(c), goalY = ROVER.frame.deckY + c.T.cgY;
  const ex = (rel.x - arm.placeX) * 1000, ey = (rel.y - goalY) * 1000, ez = rel.z * 1000, eyaw = Math.abs(rel.yaw - Math.round(rel.yaw / Math.PI) * Math.PI) / DEG;
  atMost('final deck position error (xz) after seating', Math.hypot(ex, ez), 5, ' mm', 2);
  atMost('final deck height error', Math.abs(ey), 5, ' mm', 2);
  atMost('final deck yaw error', eyaw, 0.2, ' deg', 3);
  check('deck latch closed, hooks open, arms stowed', arm.deckLatched && arm.gatesOpen && arm.stowed);
  console.log(`  placement: pre-seat ${(se.radial * 1000).toFixed(1)} mm / ${(se.yaw / DEG).toFixed(2)} deg; final ${Math.hypot(ex, ez).toFixed(2)} mm xz, ${ey.toFixed(2)} mm y, ${eyaw.toFixed(3)} deg; max tip ${maxTip.toFixed(3)}; breakout soil ${st.peakGround.toFixed(3)} W`);
  // drive 20 m with it on the deck
  const p0 = R.pose(), relBefore = sim.acm.relPose(c);
  const r2 = runGen(sim, A, 'ACM drive away', A.driveAway(20), 400);
  const p1 = R.pose(), dist = Math.hypot(p1.x - p0.x, p1.z - p0.z), relAfter = sim.acm.relPose(c);
  check('drive-away mission OK', r2.done && r2.last && r2.last.ok, r2.last ? `ok=${r2.last.ok}` : 'unfinished');
  atLeast('distance driven with the ACM on the deck', dist, 19.5, ' m', 1);
  check('ACM still on the deck after the drive (latched, same pose)', c.mode === 'DECK' && arm.deckLatched && Math.hypot(relAfter.x - relBefore.x, relAfter.z - relBefore.z) < 0.005, `d ${(Math.hypot(relAfter.x - relBefore.x, relAfter.z - relBefore.z) * 1000).toFixed(2)} mm`);
  // ---------------- 3. place to the ground ----------------
  const q = R.pose(), tx = q.x + Math.cos(q.yaw) * 14, tz = q.z - Math.sin(q.yaw) * 14;
  for (let i = 0; i < 240; i++) sim.stepOnce();
  const r3 = runGen(sim, A, 'ACM place', A.missionPlaceArm(tx, tz), 900);
  check('place-to-ground mission OK', r3.done && r3.last && r3.last.ok, r3.last ? `ok=${r3.last.ok}` : `unfinished: ${A.phase} | ${A.detail}`);
  if (r3.last && r3.last.ok) {
    for (let i = 0; i < 600; i++) sim.stepOnce();
    const t = c.body.translation();
    const err = Math.hypot(t.x - tx, t.z - tz);
    atMost('ACM on the ground within 0.5 m of the receiving point', err, 0.5, ' m', 2);
    check('ACM is a soil body again carrying its weight', c.mode === 'GROUND' && !c.onDeck && Math.abs(c.soilN / (c.T.mass * MOON.g) - 1) < 0.05, `${(c.soilN / (c.T.mass * MOON.g)).toFixed(3)} W`);
    check('rover mass back to unloaded, arms stowed, no faults', R.extraMass === 0 && arm.stowed && arm.faults.size === 0);
    check('no hook-trunnion interference over the whole pick + place', arm.strikes === 0, `strikes ${arm.strikes}`);
    console.log(`  place-to-ground: ${err.toFixed(2)} m off target, ${(A.acmStats.duration).toFixed(0)} s, max tip ${A.acmStats.tipMax.toFixed(3)}`);
  }
}

// ---------------- 2. overweight refusal ----------------
if (!ONLY || ONLY === 'heavy') {
  const { sim, R, A, c } = setup(8000, 'heavy');
  const t0 = c.body.translation(), p0 = { x: t0.x, y: t0.y, z: t0.z };
  const r = runGen(sim, A, 'ACM pick (overweight)', A.missionPickupArm(c), 600);
  check('8000 kg ACM: mission ends FAILED (refused)', r.done && r.last && r.last.ok === false, r.last ? `ok=${r.last.ok}` : `unfinished: ${A.phase}`);
  check('refused by the pre-lift check', /pre-lift check refused/.test(A.acmStats?.abort || ''), A.acmStats?.abort);
  check('never lifted: ACM still on the ground, not held', c.mode === 'GROUND' && !c.held && !R.arm.payload);
  const t1 = c.body.translation();
  atMost('ACM did not move (m)', Math.hypot(t1.x - p0.x, t1.z - p0.z), 0.3, ' m', 3);
  check('clean abort: arms stowed, no faults, no load on the rover', R.arm.stowed && R.arm.faults.size === 0 && R.extraMass === 0 && !R.arm.payload);
}

// ---------------- 4. stuck foot: soil suction beyond ACM_PICK.overloadK aborts the lift (M8) ----------------
if (!ONLY || ONLY === 'stuck') {
  const cAdh0 = ACM_SOIL.cAdhK; ACM_SOIL.cAdhK = Number(process.env.STUCK_ADH || 400);   // very sticky breakout adhesion on the feet
  const { sim, R, A, c } = setup(2400, 'stuck');
  const r = runGen(sim, A, 'ACM pick (stuck foot)', A.missionPickupArm(c), 700);
  ACM_SOIL.cAdhK = cAdh0;
  check('stuck foot: mission ends FAILED', r.done && r.last && r.last.ok === false, r.last ? `ok=${r.last.ok}` : `unfinished: ${A.phase}`);
  check(`stuck foot: aborted on hook overload (> ${ACM_PICK.overloadK} W)`, /hook load/.test(A.acmStats?.abort || ''), `${A.acmStats?.abort} (peak ${A.acmStats?.peakHook?.toFixed(3)} W)`);
  check('stuck foot: clean abort, ACM on the ground and free, arms stowed', c.mode === 'GROUND' && !c.held && R.arm.stowed && R.extraMass === 0, `mode ${c.mode}`);
  check('stuck foot: no hook-trunnion interference on the abort path', R.arm.strikes === 0, `strikes ${R.arm.strikes}`);
}

// ---------------- 5. drop fault: the ACM falls to the regolith, the fault stays recorded (M2) ----------------
if (!ONLY || ONLY === 'drop') {
  const { sim, R, A, c } = setup(2400, 'drop');
  const arm = R.arm; let injected = false, dropMode = null, faultSeen = false;
  A.start('ACM pick (drop)', A.missionPickupArm(c)); const t0 = sim.time;
  while (A.gen && sim.time - t0 < 700) {
    sim.stepOnce();
    if (!injected && c.mode === 'CARRIED' && arm.latched && c.modeT > 3) { injected = true; arm.faults.add('drop'); arm.payload = null; for (const a of arm.arms) { a.hookF = 0; a.seated = false; } }
    if (injected && dropMode == null && c.mode === 'GROUND') dropMode = sim.time;
    if (arm.faults.has('drop')) faultSeen = true;
  }
  check('drop: injected while carried', injected);
  check('drop: the ACM left CARRIED (kinematic) and is a soil body again', c.mode === 'GROUND' && dropMode != null, `mode ${c.mode}`);
  check('drop: the fault was recorded by the safe-down path (acmStats) and the mission failed', A.acmStats?.faultsAtSafeDown?.includes('drop') && A.history[A.history.length - 1]?.ok === false, JSON.stringify(A.acmStats?.faultsAtSafeDown));
  check('drop: ACM is on the regolith, not in mid-air', c.body.translation().y < sim.rover.pose().y + 6 && sim.acm.soleClearance(c) < 0.2, `sole clearance ${sim.acm.soleClearance(c).toFixed(3)} m`);
  void faultSeen;
}

// ---------------- 6. square-up only, several seeds (first attempt, tests/arm_squareup.mjs; SEEDS="a-b", default 2-8 here) ----------------
if (!ONLY) { process.env.SEEDS ||= '2-8'; await import('./arm_squareup.mjs'); }
