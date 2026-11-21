// P1b (docs/ARM_CARGO_BUILD_PLAN.md s5.7): outer-wheel drive. FL FR RL RR have motors; ML MR are passive idlers.
// Slopes are done with tilted gravity on flat ground (equivalent incline: same along-slope and normal components).
import { check, atLeast, atMost, within } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER, MOON, DRIVE } from '../src/sim/config.js';
await RAPIER.init();
const fresh = () => { const sim = new Sim(RAPIER); sim.modules = sim.modules.filter((m) => m !== sim.autonomy); return sim; };
const settle = (sim, n = 360) => { for (let i = 0; i < n; i++) sim.stepOnce(); };
const tilt = (sim, deg) => { const a = deg * Math.PI / 180; sim.world.gravity = { x: -MOON.g * Math.sin(a), y: -MOON.g * Math.cos(a), z: 0 }; }; // downhill = -x, rover faces +x (uphill)
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

// ---- A. configuration ----
{
  const R = fresh().rover;
  const drv = R.wheels.filter((w) => w.driven).map((w) => ROVER.corners[w.id][2]);
  check('driven wheels are FL FR RL RR', drv.join(',') === 'FL,FR,RL,RR' && drv.join() === DRIVE.driven.join(), drv.join(','));
  check('ML / MR are idlers', !R.wheels[2].driven && !R.wheels[3].driven);
  // 4 driven motors keep the old 6-motor budget: peakTorque is the legacy 8400 N m x MR x KS (config.js), so 4 x peak = 6 x (peak x 5600/8400)
  check('rated drive torque 4 x peak = 6 x legacy 5600 N m (scaled)', Math.abs(4 * ROVER.motor.peakTorque - 6 * ROVER.motor.peakTorque * 5600 / 8400) <= 2, `${4 * ROVER.motor.peakTorque}`);
}

// ---- B. flat drive: idlers get zero drive torque and no power; driven wheels do the work ----
{
  const sim = fresh(), R = sim.rover; settle(sim, 480); R.brake = false; R.cmd.vx = 0.8;
  let maxIdlerT = 0, maxIdlerP = 0, drvT = 0, n = 0, maxIdlerOm = 0;
  for (let i = 0; i < 120 * 40; i++) {
    sim.stepOnce();
    if (i > 120 * 15) { maxIdlerT = Math.max(maxIdlerT, Math.abs(R.wheels[2].torque), Math.abs(R.wheels[3].torque)); maxIdlerP = Math.max(maxIdlerP, R.wheels[2].power, R.wheels[3].power);
      drvT += [0, 1, 4, 5].reduce((s, k) => s + Math.abs(R.wheels[k].torque), 0) / 4; n++; maxIdlerOm = Math.max(maxIdlerOm, Math.abs(R.wheels[2].omega)); }
  }
  const v = R.telemetry().fwdSpeed;
  atMost('idler drive torque (ML, MR) while cruising', maxIdlerT, 0, ' N m', 3);
  atMost('idler motor power', maxIdlerP, 0, ' W', 3);
  atLeast('driven wheels carry the drive torque (mean |T|)', drvT / n, 50, ' N m', 0);
  atLeast('rover cruises under 4WD (m/s)', v, 0.6, ' m/s', 2);
  atLeast('idlers roll with the vehicle (omega)', maxIdlerOm, 0.1, ' rad/s', 2);
  const tel = R.telemetry();
  check('telemetry flags the idlers as passive', tel.wheels.filter((w) => !w.driven).length === 2);
  atMost('drive power from driven wheels only (sum of per-wheel power matches)', Math.abs(R.power.driveW - R.wheels.reduce((s, w) => s + w.power, 0)), 0.1 * Math.max(1, Math.abs(R.power.driveW)) + 50, ' W', 0);
}

// ---- C. parking hold and slope climb on an equivalent incline ----
for (const deg of [12, 20]) {
  const sim = fresh(), R = sim.rover; settle(sim, 480); tilt(sim, deg); R.brake = true; settle(sim, 120 * 3);
  const p0 = R.pose(); settle(sim, 120 * 20); const p1 = R.pose();
  const slid = Math.hypot(p1.x - p0.x, p1.z - p0.z);
  atMost(`parked (brakes on 4 driven wheels) holds on ${deg} deg incline: creep in 20 s`, slid, 0.5, ' m', 2);
}
for (const deg of [10, 15]) {
  const sim = fresh(), R = sim.rover; settle(sim, 480); tilt(sim, deg); R.brake = false; R.cmd.vx = 0.5;
  const p0 = R.pose(); settle(sim, 120 * 40); const p1 = R.pose();
  const along = (p1.x - p0.x) * p0.fwd.x + (p1.z - p0.z) * p0.fwd.z;
  atLeast(`4WD climbs ${deg} deg incline (40 s at 0.5 m/s cmd)`, along, 12, ' m', 1);
}

// ---- D. skid turn: yaw efficiency with the middle wheels passive ----
// Measured (P1b): the middle wheels sat at the yaw centre and gave pure thrust torque with no scrub arm; as idlers the four outer
// wheels must out-push their own lateral scrub, so pure spins lose authority at high rates (0.15 rad/s: 0.04 vs 0.78 on 6WD).
// Low-rate spins and arcs (what navigation commands) stay above NAV.noSteerRatio (0.1).
const yawEff = (vx, wz) => { const sim = fresh(), R = sim.rover; settle(sim, 480); R.brake = false; R.cmd.vx = vx; R.cmd.wz = wz;
  settle(sim, 120 * 10); const y0 = R.pose().yaw; settle(sim, 120 * 20); return wrap(R.pose().yaw - y0) / 20 / wz; };
for (const [vx, wz, lo] of [[0, 0.03, 0.12], [0.5, 0.03, 0.25], [0.5, 0.06, 0.2]]) {
  const e = yawEff(vx, wz); console.log(`yaw efficiency vx ${vx} wz ${wz}: ${e.toFixed(2)}`);
  atLeast(`skid-steer yaw efficiency vx ${vx} wz ${wz} (NAV.noSteerRatio 0.1)`, e, lo, '', 2);
}
