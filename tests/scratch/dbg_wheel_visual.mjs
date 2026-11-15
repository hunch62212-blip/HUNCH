// How far the DRAWN wheel hub is from the PHYSICAL one during ride-height moves (flat ground, so the rocker-bogie pose is neutral).
// aegis.js draws the hub at  wheelRadius - clamp(rideOffset)  (the COMMANDED ride); the physics has it at  hardpointY - L  (the strut).
// err < 0: the drawn wheel is below the real one (sinks into the ground);  err > 0: it floats.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER, K } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER), R = sim.rover, B = ROVER.bogie;
const run = (sec, fn) => { for (let i = 0; i < sec * 120; i++) { sim.stepOnce(); fn && fn(i / 120); } };
run(5);
const err = () => { const vis = ROVER.wheel.radius - Math.max(-0.045 * K, Math.min(0.065 * K, R.rideOffset)); let s = 0; for (const b of R.bogies) s += vis - (B.hardpointY - b.L); return s / R.bogies.length; };
const phase = (name, target, sec) => { R.lift.target = target; let lo = 0, hi = 0, tLo = 0, tHi = 0, tCmd = null, y0 = R.pose().y;
  run(sec, (t) => { const e = err(); if (e < lo) { lo = e; tLo = t; } if (e > hi) { hi = e; tHi = t; } if (tCmd == null && Math.abs(R.lift.pos - target) < 0.002) tCmd = t; });
  console.log(`${name.padEnd(22)} command done ${tCmd?.toFixed(2)} s | max SINK ${(-lo * 100).toFixed(0)} cm at ${tLo.toFixed(1)} s | max FLOAT ${(hi * 100).toFixed(0)} cm at ${tHi.toFixed(1)} s | settled err ${(err() * 100).toFixed(1)} cm | body moved ${(R.pose().y - y0).toFixed(2)} m`); };
console.log(`at rest, ride 0: err ${(err() * 100).toFixed(1)} cm`);
phase('kneel (0 -> -0.65)', ROVER.ride.kneel, 12);
phase('rise (kneel -> carry)', ROVER.ride.carry, 12);
phase('squat (carry -> kneel)', ROVER.ride.kneel, 12);
phase('back to 0', 0, 10);
