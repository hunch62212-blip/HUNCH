// Headless physics checks for the AEGIS-V1 vehicle model (run: npm run test:physics)
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER } from '../src/sim/config.js';
await RAPIER.init();

const t0 = Date.now();
const sim = new Sim(RAPIER);
const R = sim.rover;
console.log('world build ms', Date.now() - t0, 'rocks', sim.terrain.rocks.length, 'craters', sim.terrain.craters.length, 'active rock colliders', sim.rockColliders.size);
const sp=sim.terrain.pad; R0: { const p=sim.rover.pose(); sim.rover.teleport(p.x - sp.perp.x*15, p.z - sp.perp.z*15, Math.atan2(sp.perp.z, -sp.perp.x)); }
const run = (sec, fn) => { const n = Math.round(sec * 120); for (let i = 0; i < n; i++) { fn && fn(i / 120); sim.stepOnce(); } };
const st = (label) => {
  const p = R.pose(), tl = R.telemetry();
  const L = R.bogies.map((b) => b.L.toFixed(3)).join(' ');
  const loads = R.wheels.map((w) => (w.load / 1000).toFixed(2)).join(' ');
  console.log(`${label.padEnd(14)} t=${sim.time.toFixed(1)} pos=(${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}) yaw=${(p.yaw * 57.3).toFixed(1)} v=${tl.fwdSpeed.toFixed(2)} roll=${(tl.roll * 57.3).toFixed(1)} pitch=${(tl.pitch * 57.3).toFixed(1)} margin=${tl.margin.toFixed(1)} L=[${L}] loads kN=[${loads}] slip=${R.wheels.map((w) => w.slip.toFixed(2)).join(' ')} P=${(R.power.loadW / 1000).toFixed(1)}kW`);
};
run(4); st('settle');
const p0 = R.pose();
const wsum = R.wheels.reduce((a, w) => a + w.load, 0);
console.log('total wheel load kN', (wsum / 1000).toFixed(2), 'expected', (R.mass * 1.62 / 1000).toFixed(2));
R.brake = false;
R.cmd.vx = 1.5; run(10); st('drive 1.5');
R.cmd.vx = 1.5; R.cmd.wz = 1.5 / 14; run(8); st('turn');
R.cmd.wz = 0; R.cmd.vx = 0; run(6); st('stop');
R.cmd.vz = 0; R.cmd.wz = 0.08; run(10); st('pivot');
R.cmd.wz = 0; run(4); st('stop2');
R.cmd.vx = -0.8; run(6); st('reverse');
R.cmd.vx = 0; run(4); st('park');
const pp = R.pose(); run(10); const pq = R.pose();
console.log('park drift m', Math.hypot(pq.x - pp.x, pq.z - pp.z).toFixed(4));
console.log('dist', R.odo.dist.toFixed(1), 'energy Wh', R.power.energyUsedWh.toFixed(1), 'min margin', R.stats.minMargin.toFixed(1));

R.brake = false; R.cmd.vx = 5; run(25); st('top speed');
R.cmd.vx = 0; run(10); st('stop3');
// ride height range
R.lift.target = ROVER.ride.kneel; run(6); st('kneel');
R.lift.target = ROVER.ride.carry; run(8); st('carry ride');
R.lift.target = 0; run(6); st('ride 0');
console.log('sim ms per step', ((Date.now() - t0) / sim.stepCount).toFixed(3));
