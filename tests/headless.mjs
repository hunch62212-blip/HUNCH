// Headless physics checks for the AEGIS-V1 vehicle model (run: npm run test:physics)
// drive / turn / stop / pivot / reverse / park / top speed / kneel / carry ride, unloaded.
import { check, within, atLeast, atMost } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER, MOON } from '../src/sim/config.js';
await RAPIER.init();

const t0 = Date.now();
const sim = new Sim(RAPIER);
const R = sim.rover;
console.log('world build ms', Date.now() - t0, 'rocks', sim.terrain.rocks.length, 'craters', sim.terrain.craters.length, 'active rock colliders', sim.rockColliders.size);
// move 15 m off the pad edge onto open ground
{ const sp = sim.terrain.pad, p = R.pose(); R.teleport(p.x - sp.perp.x * 15, p.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); }
const run = (sec, fn) => { const n = Math.round(sec * 120); for (let i = 0; i < n; i++) { fn && fn(i / 120); sim.stepOnce(); } };
const deg = (r) => r * 180 / Math.PI;
const dyaw = (a, b) => Math.abs(deg(Math.atan2(Math.sin(b - a), Math.cos(b - a))));
const st = (label) => {
  const p = R.pose(), tl = R.telemetry();
  console.log(`${label.padEnd(14)} t=${sim.time.toFixed(1)} pos=(${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}) yaw=${deg(p.yaw).toFixed(1)} v=${tl.fwdSpeed.toFixed(2)} roll=${deg(tl.roll).toFixed(1)} pitch=${deg(tl.pitch).toFixed(1)} margin=${tl.margin.toFixed(1)} loads kN=[${R.wheels.map((w) => (w.load / 1000).toFixed(2)).join(' ')}] P=${(R.power.loadW / 1000).toFixed(1)}kW`);
  return { p, tl };
};

run(4); st('settle');
// 1. static load balance: wheel loads carry the rover's lunar weight (5 % tolerance for strut settling)
const wsum = R.wheels.reduce((a, w) => a + w.load, 0), wexp = R.mass * MOON.g;
within('static wheel load / lunar weight', wsum / wexp, 0.95, 1.05);
equal6();
function equal6() { check('wheel count matches config corners', R.wheels.length === ROVER.corners.length, `${R.wheels.length} wheels, ${ROVER.corners.length} corners`); }

R.brake = false;
R.cmd.vx = 1.5; run(10); let s = st('drive 1.5');
within('speed tracking at 1.5 m/s', s.tl.fwdSpeed, 1.35, 1.65, ' m/s');

let y0 = R.pose().yaw;
R.cmd.vx = 1.5; R.cmd.wz = 1.5 / 14; run(8); s = st('turn');
atLeast('skid-steer turn: heading change in 8 s at 0.107 rad/s cmd', dyaw(y0, s.p.yaw), 20, ' deg', 1);

R.cmd.wz = 0; R.cmd.vx = 0; run(6); s = st('stop');
atMost('stop from 1.5 m/s within 6 s', Math.abs(s.tl.fwdSpeed), 0.05, ' m/s');

y0 = R.pose().yaw; const pv0 = R.pose();
R.cmd.vz = 0; R.cmd.wz = 0.08; run(10); s = st('pivot');
atLeast('pivot: heading change in 10 s', dyaw(y0, s.p.yaw), 20, ' deg', 1);
atMost('pivot: stays in place', Math.hypot(s.p.x - pv0.x, s.p.z - pv0.z), 0.5, ' m');
R.cmd.wz = 0; run(4); st('stop2');

R.cmd.vx = -0.8; run(6); s = st('reverse');
within('reverse speed tracking at -0.8 m/s', s.tl.fwdSpeed, -0.9, -0.7, ' m/s');

R.cmd.vx = 0; run(4); st('park');
const pp = R.pose(); run(10); const pq = R.pose();
atMost('park drift over 10 s (brakes set)', Math.hypot(pq.x - pp.x, pq.z - pp.z), 0.05, ' m', 4);
atLeast('min tip margin, unloaded manoeuvres (autonomy slow-down limit 25 deg)', R.stats.minMargin, 25, ' deg', 1);
console.log('dist', R.odo.dist.toFixed(1), 'energy Wh', R.power.energyUsedWh.toFixed(1));

R.brake = false; R.cmd.vx = 5; run(25); s = st('top speed');
within('top speed, unloaded, 5 m/s command: held at speedLimitEmpty', s.tl.fwdSpeed, 2.7, 3.1, ' m/s', 2);
atMost('upright at top speed (|roll|)', Math.abs(deg(s.tl.roll)), 10, ' deg', 1);
R.cmd.vx = 0; run(10); st('stop3');

// 2. ride height range (kneel for pickup, carry ride)
R.lift.target = 0; run(6); const yRide0 = R.pose().y;
R.lift.target = ROVER.ride.kneel; run(6); const yKneel = st('kneel').p.y;
R.lift.target = ROVER.ride.carry; run(8); const yCarry = st('carry ride').p.y;
R.lift.target = 0; run(6); st('ride 0');
atLeast('kneel lowers body', yRide0 - yKneel, 0.5, ' m');
atLeast('carry ride raises body', yCarry - yRide0, 0.5, ' m');
console.log('sim ms per step', ((Date.now() - t0) / sim.stepCount).toFixed(3));
