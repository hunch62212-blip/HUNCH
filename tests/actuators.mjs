// Actuator models (src/sim/actuators.js, plan s5.1/s5.2): DC gearmotor and servo.
//  1. 37D 30:1: R, k, stall torque + current, no-load speed, gearbox limit flag
//  2. torque-speed line, saturation (I <= Ilim), regen clipping, power/energy sign sanity
//  3. DS3218 60 deg step at 6.8 V vs 5 V (TsV table)
//  4. backlash dead zone / hysteresis = b
//  5. rail sag, thermal heating + derate, brake, full-scale ACT specs
import { check, within, atMost, atLeast } from './lib/check.mjs';
import { ACT } from '../src/sim/config.js';
import { DCMotor, Servo, Rail, dz, fromSpec } from '../src/sim/actuators.js';

const D = Math.PI / 180;

// 1. 37D 30:1
{
  const m = new DCMotor(ACT.proto.gm37d30);
  within('37D R', m.R, 2.17, 2.19, ' ohm');
  within('37D k', m.k, 0.33, 0.338, ' V*s/rad');
  const st = m.eval(1, 0, 12);
  within('37D stall torque', st.tau, 1.37 * 0.98, 1.37 * 1.02, ' N*m');
  within('37D stall current', st.I, 5.5 * 0.98, 5.5 * 1.02, ' A');
  const nl = m.eval(1, 34.6, 12);
  within('37D no-load current', nl.I, 0.19, 0.21, ' A');
  within('37D no-load torque ~0', nl.tau, -0.01, 0.01, ' N*m');
  // no-load speed: speed where torque crosses zero is w0 +- 2 %
  let lo = 0, hi = 60;
  for (let i = 0; i < 60; i++) { const mid = (lo + hi) / 2; (m.eval(1, mid, 12).tau > 0 ? (lo = mid) : (hi = mid)); }
  within('37D no-load speed', lo, 34.6 * 0.98, 34.6 * 1.02, ' rad/s');
  check('37D stall exceeds gearbox continuous limit flag', m.step(1, 0, 12, 0.01).gearOver === true);
  check('37D half-torque point inside gearbox limit', m.step(0.3, 0, 12, 0.01).gearOver === false);
}

// 2. torque-speed line, saturation, power sign
{
  const m = new DCMotor(ACT.proto.gm37d30);
  const t0 = m.eval(1, 0, 12).tau, t1 = m.eval(1, 17.3, 12).tau;
  within('torque-speed: half speed ~ half torque', t1 / t0, 0.45, 0.55);
  const m2 = new DCMotor({ ...ACT.proto.gm37d30, Ilim: 3 });
  within('saturation: I <= Ilim at stall', m2.eval(1, 0, 12).I, 2.99, 3.0001, ' A');
  within('saturation: I >= -Ilim when plugged', m2.eval(-1, 30, 12).I, -3.0001, -2.99, ' A');
  atMost('saturation: |u|>1 clamped', Math.abs(m.step(5, 0, 12, 0.01).I), 5.51, ' A');
  // motoring: electrical in > mechanical out, both >= 0
  const r = m.step(1, 17, 12, 0.01);
  atLeast('motoring P_el > 0', r.P_el, 1e-6, ' W');
  atLeast('motoring P_mech > 0', r.P_mech, 1e-6, ' W');
  check('motoring P_el > P_mech (losses positive)', r.P_el > r.P_mech, `${r.P_el.toFixed(2)} > ${r.P_mech.toFixed(2)}`);
  // braking (u = 0 while spinning): torque opposes, regen clipped to 0 in prototype mode
  const b = m.step(0, 20, 12, 0.01);
  check('braking torque opposes motion', b.tau < 0, `tau ${b.tau.toFixed(3)}`);
  check('regen clipped (P_el >= 0)', b.P_el >= 0, `P_el ${b.P_el.toFixed(3)}`);
  const mr = new DCMotor(ACT.proto.gm37d30, { regen: 0.5 });
  const br = mr.step(0.3, 20, 12, 0.01);
  check('full-scale regen returns power (P_el < 0)', br.P_el < 0, `P_el ${br.P_el.toFixed(3)}`);
  m.E_el = 0; m.E_mech = 0;
  for (let i = 0; i < 200; i++) m.step(0.8, 20, 12, 0.005);
  check('energy: electrical >= mechanical over a run', m.E_el > m.E_mech && m.E_mech > 0, `${m.E_el.toFixed(1)} J vs ${m.E_mech.toFixed(1)} J`);
}

// 3. DS3218 60 deg step: closed loop with a light inertial load (servo only, no external torque)
function step60(Vbus, spec = ACT.proto.ds3218) {
  const s = new Servo({ ...spec, backlash: 0 });
  s.reset(0);
  let q = 0, qd = 0, t = 0;
  const dt = 1 / 960, JL = 1e-3;
  while (t < 1.5) {
    const r = s.step(60 * D, q, qd, Vbus, dt);
    qd += (r.tau / JL) * dt; q += qd * dt; t += dt;
    if (q >= 58 * D) return t;
  }
  return Infinity;
}
{
  const t68 = step60(6.8), t5 = step60(5);
  atMost('DS3218 60 deg step at 6.8 V', t68, 0.20, ' s');
  check('DS3218 slower at 5 V (TsV)', t5 > t68, `${t5.toFixed(3)} s > ${t68.toFixed(3)} s`);
  const s = new Servo(ACT.proto.ds3218, { brake: false });
  within('DS3218 stall torque (6.8 V)', s.eval(1, 0, 6.8).tau, 2.11 * 0.98, 2.11 * 1.02, ' N*m');
  within('DS3218 stall torque (5 V, TsV table)', s.eval(1, 0, 5).tau, 1.77 * 0.97, 1.77 * 1.03, ' N*m');
}

// 4. backlash
{
  check('dz', dz(0.3, 0.1) === 0.3 - 0.1 && dz(-0.3, 0.1) === -0.3 + 0.1 && dz(0.05, 0.1) === 0);
  const spec = { ...ACT.proto.big60, I0: 0.001, fs: 12 };
  const b = spec.backlash;
  const s = new Servo(spec, { brake: false, ki: 0 });
  s.reset(0);
  const dt = 1 / 960, v = 0.005;
  let q = 0, lagUp = 0, lagDn = 0;
  // ref tracks q so the loop commands ~0; gear coasts and is pushed by the contact
  for (let i = 0; i < 12000; i++) { q += v * dt; s.step(q, q, v, 7.4, dt); }
  lagUp = s.thG - q;
  for (let i = 0; i < 12000; i++) { q -= v * dt; s.step(q, q, -v, 7.4, dt); }
  lagDn = s.thG - q;
  within('backlash hysteresis = b', lagDn - lagUp, b * 0.95, b * 1.05, ' rad', 5);
  check('backlash: gear lags when moving up, leads when moving down', lagUp < 0 && lagDn > 0);
  // with real motor friction the hysteresis is >= b
  const sf = new Servo(ACT.proto.big60, { brake: false, ki: 0 }); sf.reset(0);
  let qf = 0;
  for (let i = 0; i < 12000; i++) { qf += v * dt; sf.step(qf, qf, v, 7.4, dt); }
  const up = sf.thG - qf;
  for (let i = 0; i < 12000; i++) { qf -= v * dt; sf.step(qf, qf, -v, 7.4, dt); }
  atLeast('backlash with friction >= b', (sf.thG - qf) - up, b * 0.95, ' rad', 5);
}

// 5. rail sag, thermal, brake, full-scale specs
{
  const rail = new Rail({ Vsrc: 12, Rsrc: 0.1 });
  const ms = [new DCMotor(ACT.proto.gm37d30), new DCMotor(ACT.proto.gm37d30), new DCMotor(ACT.proto.gm37d30)];
  let V = 12, sumI = 0;
  for (let it = 0; it < 20; it++) { sumI = ms.reduce((a, m) => a + Math.abs(m.eval(1, 0, V).I), 0); V = rail.bus(sumI); }
  within('rail sag dV = sum(I)*R_src', 12 - V, sumI * 0.1 - 1e-9, sumI * 0.1 + 1e-9, ' V');
  atMost('3 stalled motors sag the rail', V, 11, ' V');
  const sag = new DCMotor(ACT.proto.gm37d30);
  check('stall torque falls with sag', sag.eval(1, 0, V).tau < sag.eval(1, 0, 12).tau);

  const sh = fromSpec(ACT.shoulder);
  check('fromSpec: ACT.shoulder -> Servo', sh instanceof Servo);
  check('fromSpec: gm37d30 -> DCMotor', fromSpec(ACT.proto.gm37d30) instanceof DCMotor && !(fromSpec(ACT.proto.gm37d30) instanceof Servo));
  const st = sh.eval(1, 0, 400);
  atMost('shoulder stall current limited to Ilim', st.I, ACT.shoulder.Ilim + 1e-9, ' A');
  atMost('shoulder stall torque <= Ts', st.tau, ACT.shoulder.Ts, ' N*m');
  // thermal: stall heating raises T_w, derating cuts torque
  const th = new DCMotor(ACT.shoulder);
  const t0 = th.eval(1, 0, 400).tau;
  for (let i = 0; i < 3000; i++) th.step(1, 0, 400, 0.1);
  atLeast('thermal: winding heats at stall', th.T_w, 40, ' C');
  check('thermal: torque derates when hot', th.derate() < 1 ? th.eval(1, 0, 400).tau < t0 : true, `derate ${th.derate().toFixed(2)}`);
  // brake: engages after 0.5 s still, zero current, still holds a load
  const br = new Servo(ACT.shoulder, { q0: 1 });
  let r;
  for (let i = 0; i < 960 * 1; i++) r = br.step(1, 1, 0, 400, 1 / 960);
  check('brake engaged after standstill', br.braked === true);
  check('brake: zero holding current', r.I === 0 && r.P_el === 0, `I ${r.I}`);
}
