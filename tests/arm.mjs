// Arm physics (plan s4, s5.3, s5.4, s5.6, P2a): kinematics, limits, gravity torque, stall, lift margin, tip governor,
// load split, latch timing, determinism, and a pick -> lift -> place trajectory. Headless, no Rapier.
import { check, within, atLeast, atMost } from './lib/check.mjs';
import { ARM, ACT, CARGO_TYPES, MOON, ROVER, K, K_LEGACY } from '../src/sim/config.js';
import { ArmSystem, fk, ik, gravityTorques, computeDeckPlaceX, inLimits } from '../src/sim/arm.js';

const g = MOON.g, DT = 1 / 120, ACM = CARGO_TYPES.acm, LK = ARM.links;
const KS = K / K_LEGACY; // the arm links (0.2 x K = 1.63 m) are half the legacy 1:16.3 ones, so the legacy-metre test points below scale by KS
const run = (a, secs, ctx = {}) => { for (let i = 0; i < Math.round(secs / DT); i++) a.step(DT, ctx); };
const place = (a, q) => { a.wake(); for (const s of a.arms) { s.q = q.slice(); s.ref = q.slice(); s.qd = [0, 0, 0]; s.servos.forEach((sv, j) => sv.reset(q[j])); } };
const waitDone = (a, ctx = {}, maxS = 400) => { let t = 0; while (!a.done && t < maxS) { a.step(DT, ctx); t += DT; } run(a, 2, ctx); return t; };

// 1. FK / IK round trip
{
  const A = new ArmSystem();
  let worst = 0, n = 0;
  for (const [name, p] of Object.entries(A.poses)) {
    if (name === 'stow') continue;
    const r = fk(p.q).tip; worst = Math.max(worst, Math.hypot(r[0] - p.p[0], r[1] - p.p[1])); n++;
  }
  for (const [x, y, psi] of [[5, 3, 0.3], [1, 7, -0.5], [6.5, 1.5, 0.1], [2.5, 8, 0]].map(([x, y, a]) => [x * KS, y * KS, a])) { // legacy metres x KS (reach 2 L = 3.26 m now)
    for (const br of ['up', 'down']) {
      const r = ik([x, y], psi, { branch: br }), f = fk(r.q);
      worst = Math.max(worst, Math.hypot(f.tip[0] - x, f.tip[1] - y), Math.abs(Math.atan2(Math.sin(f.psi - psi), Math.cos(f.psi - psi)))); n++;
    }
  }
  atMost(`FK(IK(p)) error over ${n} cases`, worst, 1e-9, ' m');
  check('elbow-up branch has the higher elbow', fk(ik([6.854 * KS, 1.469 * KS], 0, { branch: 'up' }).q).elbow[1] > fk(ik([6.854 * KS, 1.469 * KS], 0, { branch: 'down' }).q).elbow[1]);
  check('all presets reachable inside limits', Object.values(A.poses).every((p) => p.ok), Object.entries(A.poses).filter(([, p]) => !p.ok).map(([n]) => n).join());
}

// 2. joint limits and reach
{
  const A = new ArmSystem();
  check('IK flags an unreachable point', ik([20, 1], 0).ok === false && ik([20, 1], 0).reasons.includes('reach'));
  check('moveTo refuses q1 beyond the limit', A.moveTo([3.2, -1, 0]).ok === false);
  check('moveTo refuses q3 beyond the limit', A.moveTo([1, -1, 3]).ok === false);
  check('stow pose inside limits', inLimits(ARM.stow.q));
  // hard stop: a reference at the limit never lets q pass it
  place(A, [ARM.limits.q1[1] - 0.001, -1, 0]);
  A.arms.forEach((s) => { s.ref[0] = 3.0; });
  run(A, 5);
  atMost('q1 held at its upper stop', A.arms[0].q[0], ARM.limits.q1[1] + 1e-9, ' rad');
}

// 3. gravity torque against hand calculation (lunar g, 2400 kg ACM, 1200 kg per arm)
{
  const mt = LK.mWrist + LK.mHook + ACM.mass / 2;
  const L1 = ARM.L1, L2 = ARM.L2;
  const hand1 = g * (LK.m1 * LK.lc1 + LK.mElbow * L1 + LK.m2 * (L1 + LK.lc2) + mt * (L1 + L2));   // arm stretched forward, horizontal
  const hand2 = g * (LK.m2 * LK.lc2 + mt * L2);
  const G = gravityTorques([0, 0, 0], mt, g);
  within('shoulder gravity torque, arm horizontal', G[0], hand1 * 0.999, hand1 * 1.001, ' N*m', 1);
  within('elbow gravity torque, arm horizontal', G[1], hand2 * 0.999, hand2 * 1.001, ' N*m', 1);
  // independent check at the grasp pose: numerical derivative of the potential energy
  const A = new ArmSystem(), q = A.poses.grasp.q;
  const U = (qq) => {
    const c1 = Math.cos(qq[0]), s1 = Math.sin(qq[0]), c12 = Math.cos(qq[0] + qq[1]), s12 = Math.sin(qq[0] + qq[1]);
    const ey = ARM.shoulder.y + L1 * s1;
    return g * (LK.m1 * (ARM.shoulder.y + LK.lc1 * s1) + LK.mElbow * ey + LK.m2 * (ey + LK.lc2 * s12) + mt * (ey + L2 * s12));
  };
  const e = 1e-6, Gp = gravityTorques(q, mt, g);
  const d1 = (U([q[0] + e, q[1]]) - U([q[0] - e, q[1]])) / (2 * e), d2 = (U([q[0], q[1] + e]) - U([q[0], q[1] - e])) / (2 * e);
  within('shoulder torque at grasp = dU/dq1', Gp[0], d1 * 0.9999, d1 * 1.0001, ' N*m', 1);
  within('elbow torque at grasp = dU/dq2', Gp[1], d2 * 0.9999, d2 * 1.0001, ' N*m', 1);
  console.log(`INFO grasp pose gravity torques: shoulder ${Gp[0].toFixed(0)} N*m, elbow ${Gp[1].toFixed(0)} N*m (2400 kg ACM, 1200 kg per arm)`);
}

// 4. lift margin at the grasp pose and servo stall
{
  const A = new ArmSystem();
  const m24 = A.liftMargin(2400), m30 = A.liftMargin(3000);
  atLeast('2400 kg lift margin at grasp (avail / required, worst joint)', m24.margin, 1.5);
  atLeast('3000 kg (rated) lift margin at grasp', m30.margin, 1.25);
  atMost('2400 kg shoulder demand <= 0.5 x stall (continuous rule)', m24.joints[0].ratioStall, 0.5);
  console.log(`INFO margin 2400 kg: shoulder ${m24.joints[0].margin.toFixed(2)} (${m24.joints[0].req.toFixed(0)}/${m24.joints[0].avail.toFixed(0)} N*m), elbow ${m24.joints[1].margin.toFixed(2)}; 3000 kg: shoulder ${m30.joints[0].margin.toFixed(2)}, elbow ${m30.joints[1].margin.toFixed(2)}`);
  // closed loop: hold 2400 kg at the grasp pose with the soil load removed
  place(A, A.poses.grasp.q);
  check('engage seats both hooks at the trunnion', A.engage(2400, { x: A.poses.grasp.p[0], y: A.poses.grasp.p[1] }).ok);
  A.latch(true); run(A, 1, { groundN: 0 });
  const q0 = A.arms[0].q.slice();
  run(A, 6, { groundN: 0 });
  check('2400 kg held with no stall fault', !A.faults.has('stall'), [...A.faults].join());
  atMost('2400 kg hold: shoulder drift', Math.abs(A.arms[0].q[0] - q0[0]), 0.01, ' rad', 4);
  const tl = A.telemetry();
  within('2400 kg hook load = weight', tl.hookLoad[0] + tl.hookLoad[1], 2400 * g * 0.99, 2400 * g * 1.01, ' N', 0);
  // overload: 12 t payload exceeds the servo torque -> arm sags, stall fault
  const B = new ArmSystem();
  place(B, B.poses.grasp.q);
  B.engage(12000, { x: B.poses.grasp.p[0], y: B.poses.grasp.p[1] }); B.latch(true);
  run(B, 90, { groundN: 0 });
  check('12 t payload: stall fault raised', B.faults.has('stall'), [...B.faults].join());
  atLeast('12 t payload: shoulder sagged', Math.abs(B.arms[0].q[0] - B.poses.grasp.q[0]), 0.05, ' rad');
  atMost('12 t payload: lift margin below 1', B.liftMargin(12000).margin, 1);
}

// 5. tip-ratio governor
{
  const A = new ArmSystem();
  const pick = { x: A.poses.grasp.p[0], y: A.poses.grasp.p[1] };
  const ok = A.canLift(2400, { marginF: 20, tip: pick });
  atMost('tip ratio, 2400 kg x 1.25 at the pick', ok.tipRatio, ARM.tipRatioMax);
  check('2400 kg lift allowed', ok.ok, ok.reasons.join());
  console.log(`INFO tip ratio at pick: 2400 kg x1.25 ${ok.tipRatio.toFixed(3)}, 3000 kg x1.25 ${A.canLift(3000, { tip: pick }).tipRatio.toFixed(3)}, 6500 kg x1.25 ${A.canLift(6500, { tip: pick }).tipRatio.toFixed(3)}`);
  const over = A.canLift(8000, { tip: pick });
  check('8000 kg over-reach lift refused for tip ratio', !over.ok && over.reasons.includes('tip'), `ratio ${over.tipRatio.toFixed(3)} ${over.reasons.join()}`);
  check('low marginF refuses an otherwise fine lift', A.canLift(2400, { marginF: 5, tip: pick }).reasons.includes('margin'));
  // the move command itself refuses
  place(A, A.poses.grasp.q); A.engage(6500, pick); A.latch(true);
  const mv = A.moveTo('apex');
  check('moveTo refuses to lift a tipping payload', mv.ok === false && /tip/.test(mv.reason), mv.reason);
  const C = new ArmSystem(); place(C, C.poses.grasp.q); C.engage(2400, pick); C.latch(true);
  check('moveTo accepts the 2400 kg lift', C.moveTo('apex').ok === true);
}

// 6. load split
{
  const A = new ArmSystem(), pick = { x: A.poses.grasp.p[0], y: A.poses.grasp.p[1] };
  place(A, A.poses.grasp.q); A.engage(2400, pick); A.latch(true); run(A, 1, { groundN: 0 });
  const s = A.telemetry();
  within('symmetric: left share', s.hookLoad[0] / (2400 * g), 0.49, 0.51);
  atMost('symmetric: asymmetry', s.asym, 0.01);
  check('symmetric: not flagged', !s.asymFlag);
  const B = new ArmSystem(); place(B, B.poses.grasp.q);
  // ~3.3 mm of hook height at L2 = 1.63 m (0.001 rad at the legacy 3.26 m link): the hook spring does not scale with length
  B.arms[1].q[1] += 0.001 / KS; B.engage(2400, pick); B.latch(true); run(B, 0.1, { groundN: 0 });
  const t = B.telemetry();
  check('asymmetric (~4 mm hook height error): flagged', t.asymFlag, `asym ${t.asym.toFixed(2)}`);
  check('asymmetric: higher arm carries more', t.hookLoad[1] > t.hookLoad[0], `${t.hookLoad[0].toFixed(0)} / ${t.hookLoad[1].toFixed(0)} N`);
  within('asymmetric: loads sum to weight', t.hookLoad[0] + t.hookLoad[1], 2400 * g * 0.999, 2400 * g * 1.001, ' N', 0);
  const C = new ArmSystem(); place(C, C.poses.grasp.q); C.engage(2400, pick);
  C.arms[0].seated = false; C.latch(true); run(C, 1, { groundN: 0 });
  const u = C.telemetry();
  check('one hook seated: that arm carries the whole load and it is flagged', u.hookLoad[0] === 0 && u.hookLoad[1] > 2400 * g * 0.99 && u.asymFlag);
  // lift-off from the ground: soil load decays -> state carried
  const D = new ArmSystem(); place(D, D.poses.grasp.q); D.engage(2400, pick); D.latch(true);
  D.step(DT, { groundN: 2400 * g });
  check('on the ground: engaged, hooks carry nothing', D.telemetry().payload.state === 'engaged' && D._hookLoad() < 1);
  D.step(DT, { groundN: 0 });
  check('soil load gone but clearance not reached: still on the ground', !D.lifted, D.payload.state);
  D.payload.restY = pick.y - ARM.pick.clearance - 0.01; D.step(DT, { groundN: 0 });
  check('lift-off from the ground detected (load on hooks, clearance reached)', D.lifted, D.payload && D.payload.state);
}

// 7. latch timing
{
  const A = new ArmSystem();
  const time = (cmd, cond, deck) => { cmd(); let t = 0; while (!cond() && t < 3) { A.step(DT, {}); t += DT; } return t; };
  const tc = time(() => A.latch(true), () => A.latched);
  within('latch gate close time', tc, 0.3, 0.6, ' s');
  const to = time(() => A.latch(false), () => A.gatesOpen);
  within('latch gate open time', to, 0.3, 0.6, ' s');
  const td = time(() => A.deckLatch(true), () => A.deckLatched);
  within('deck latch time (DECK latchTime 0.6 s)', td, 0.3, 0.7, ' s');
}

// 8. determinism
{
  const trace = () => {
    const A = new ArmSystem(), pick = { x: A.poses.grasp.p[0], y: A.poses.grasp.p[1] };
    A.moveTo('preGrasp'); run(A, 30); A.moveTo('grasp'); run(A, 30);
    A.engage(2400, pick); A.latch(true); run(A, 1); A.moveTo('apex'); run(A, 30);
    return JSON.stringify(A.telemetry());
  };
  check('two identical runs give identical telemetry', trace() === trace());
}

// 8b. startup power state and wake-up
{
  const A = new ArmSystem();
  run(A, 5);
  check('power-up: arms cradled, servos unpowered', A.cradled && A.armW === 0);
  within('power-up: arm power', A.powerW, 0, 5, ' W');
  A.moveTo('preGrasp'); run(A, 5);
  atLeast('moving: hotel + servos powered', A.powerW, ARM.hotelW, ' W');
  check('moving: not cradled', !A.cradled);
}

// 9. pick -> lift -> place through the presets
{
  const A = new ArmSystem();
  const placeX = computeDeckPlaceX();
  // Legacy -0.734 m (6200 kg rover, K 16.3). At K = 8.16 the stow CG offset halves (x KS) and the rover is 5000 kg against the same 2400 kg
  // ACM, so x = -cg M / m_acm = -0.347 m; the loaded-CG = 0 check below is the actual rule.
  within('DECK place x from the balance rule (stow CG from FK)', placeX, -0.39, -0.30, ' m');
  const T = ACM, deckTrun = { x: placeX, y: ROVER.frame.deckY + T.trunnion.y };
  const cgL = A.cg({ mass: T.mass, tip: deckTrun }), cgE = A.cg({ mass: 0 });
  const half = 0.02 * 2 * ROVER.corners[0][0];   // 0.02 x wheelbase (7.83 m)
  within('loaded CG x (plan s3 rule: 0 +- 0.02 wheelbase)', cgL.x, -half, half, ' m');
  within('loaded front-axle share (plan: 33 %)', (cgL.x + ROVER.corners[0][0] * 0.5) / (ROVER.corners[0][0] * 1.5), 0.30, 0.37);
  within('unloaded stowed CG x', cgE.x, 0, 0.35, ' m');
  within('stowed arm tip clear of the LiDAR plinths (x < 3.43 m)', fk(ARM.stow.q).tip[0], -10, 3.4, ' m');
  atLeast('stowed arm links above the deck (min elbow/tip y)', Math.min(fk(ARM.stow.q).elbow[1], fk(ARM.stow.q).tip[1]), ROVER.frame.deckY + 0.5, ' m');
  console.log(`INFO CG x unloaded ${cgE.x.toFixed(3)} (front ${((cgE.x + 1.958) / 5.875).toFixed(3)}), loaded ${cgL.x.toFixed(3)} (front ${((cgL.x + 1.958) / 5.875).toFixed(3)}), deck place x ${placeX.toFixed(3)}`);
  let maxTip = 0, maxPct = 0, tStart = 0;
  const watch = (a) => { const t = a.telemetry(); maxTip = Math.max(maxTip, t.tipRatio); maxPct = Math.max(maxPct, ...t.pctStall.flat()); };
  const leg = (name) => { const r = A.moveTo(name); if (!r.ok) { check(`leg ${name}`, false, r.reason); return; } let t = 0; while (!A.done && t < 400) { A.step(DT, { marginF: 20 }); watch(A); t += DT; } for (let i = 0; i < 240; i++) { A.step(DT, { marginF: 20 }); watch(A); } };
  leg('preGrasp'); leg('grasp');
  const tip = fk(A.arms[0].q).tip, gp = A.poses.grasp.p;
  atMost('at grasp: hook position error', Math.hypot(tip[0] - gp[0], tip[1] - gp[1]), 0.05, ' m');
  check('engage at the trunnion', A.engage(2400, { x: gp[0], y: gp[1] }).ok);
  A.latch(true); run(A, 1, { marginF: 20 });
  check('latched before lift', A.latched);
  leg('apex');
  check('payload lifted off the ground (liftoff event)', A.lifted && A.events.includes('liftoff'), A.payload && A.payload.state);
  leg('prePlace'); leg('place');
  const pt = A._trunnion(), pp = A.poses.place.p;
  within('ACM trunnion x at DECK place x', pt.x, placeX - 0.05, placeX + 0.05, ' m');
  atMost('ACM trunnion height error at the deck', Math.abs(pt.y - pp[1]), 0.05, ' m');
  A.deckLatch(true); run(A, 1); A.release(); leg('stow');
  check('deck latch closed and ACM released', A.deckLatched && A.payload === null);
  check('no faults over the whole trajectory', A.faults.size === 0, [...A.faults].join());
  atMost('max tip ratio over the trajectory', maxTip, ARM.tipRatioMax);
  atMost('max joint torque / stall over the trajectory', maxPct, 1);
  check('returned to stow', A.stowed);
  check('stowed arms power down into the cradles', A.cradled);
  within('stowed power draw (standby only, no servo hold current)', A.telemetry().armW + A.powerW, 0, 5, ' W');
  A.step(DT, { powered: false });
  check('rover in survival: arm draws 0 W', A.powerW === 0);
  console.log(`INFO max tip ratio ${maxTip.toFixed(3)}, max torque/stall ${maxPct.toFixed(2)}, energy ${A.energyWh.toFixed(2)} Wh`);
}

// 8. release / re-hook geometry for the up-opening V-seat (plan s6.4): hooks leave DOWN k(0.03), come back UP from below
{
  const A = new ArmSystem(), trD = { x: A.placeX, y: A.poses.place.p[1] };
  const low = ik([trD.x, trD.y - ARM.releaseDrop], 0, { branch: 'up' }), out = ik([trD.x + ARM.releaseSlide, trD.y - ARM.releaseDrop], 0, { branch: 'up' });
  check('release pose (down k(0.03)) and the slide-clear pose are reachable', low.ok && out.ok, `${low.reasons} ${out.reasons}`);
  place(A, A.poses.stow.q);
  place(A, A.poses.stow.q);
  const hAbove = (ARM.keepOutMargin + ARM.hook.mouth / 2) / 2; // between the margin and the half-disc radius (0.15 m at the legacy 0.245 m radius; 0.075 m now)
  const above = ik([trD.x, trD.y + hAbove], 0, { branch: 'up' });
  // the OLD re-hook (hooks at prePlace, 0.98 m over the trunnion, then down onto it) passes the hook through the trunnion from above
  place(A, A.poses.prePlace.q);
  check('keep-out: the old prePlace -> place descent (from above) is flagged', A.pathHitsKeepOut('place', trD).hit === true || A.pathHitsKeepOut(above.q, trD).hit === true);
  check('keep-out: a hook tip above the trunnion, between the margin and the half-disc radius, is inside the half-disc', ArmSystem.keepOutHit(fk(above.q).tip, trD));
  place(A, low.q);
  check('keep-out: rising from below to the seat (target y = trunnion y) is clear', A.pathHitsKeepOut(A.poses.place.q, trD).hit === false);
  place(A, out.q);
  check('keep-out: slide-clear pose to the stow path is clear of the column', A.pathHitsKeepOut('stow', trD).hit === false);
  // runtime watch counts a hook tip above an un-seated trunnion
  place(A, above.q); A.keepOut = trD; run(A, 0.1);
  check('runtime watch counts the strike (hook above the un-seated trunnion)', A.strikes === 1, `strikes ${A.strikes}`);
  place(A, A.poses.place.q); A.keepOut = trD; A.strikes = 0; A._koIn = false; run(A, 0.1);
  check('no strike when the hook sits at the seat height', A.strikes === 0, `strikes ${A.strikes}`);
}

// 9. manual pose commands are gated per payload state (no stow through the deck while holding the ACM)
{
  const A = new ArmSystem(), pick = { x: A.poses.grasp.p[0], y: A.poses.grasp.p[1] };
  place(A, A.poses.grasp.q); A.engage(2400, pick); A.latch(true);
  const r1 = A.moveTo('stow');
  check("engaged on the ground: 'stow' is refused", r1.ok === false && /not allowed/.test(r1.reason), r1.reason);
  check("engaged on the ground: 'apex' is allowed", A.moveTo('apex').ok === true);
  A.payload.state = 'carried';
  check("carried: 'stow' and 'grasp' are refused, 'prePlace' allowed", A.moveTo('stow').ok === false && A.moveTo('grasp').ok === false && A.moveTo('prePlace').ok === true);
}
