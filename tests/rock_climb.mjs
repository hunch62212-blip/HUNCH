// A wheel track meets a boulder: the rover must climb it (or be stopped by it), never pass through.
// Climb limit: ROVER.climbMax (0.85 x wheel radius). A 0.57 m rock must be climbed, a 1.40 m rock must block.
import { check, atLeast, atMost } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
const BOULDER_AT = 12; // m ahead
for (const d of [0.9, 2.2]) {
  const sim = new Sim(RAPIER); sim.modules = sim.modules.filter((m) => m !== sim.autonomy);
  const R = sim.rover, sp = sim.terrain.pad;
  const p0 = R.pose();
  R.teleport(p0.x - sp.perp.x * 15, p0.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); for (let i = 0; i < 240; i++) sim.stepOnce();
  const p = R.pose();
  const right = { x: -p.fwd.z, z: p.fwd.x };
  // boulder 12 m ahead, under the right wheel track
  const i = sim.addBoulder(p.x + p.fwd.x * BOULDER_AT + right.x * 2.75, p.z + p.fwd.z * BOULDER_AT + right.z * 2.75, d);
  const rock = sim.terrain.rocks[i], h = rock.top - rock.y;
  R.brake = false; R.cmd.vx = 0.6;
  let maxRoll = 0, maxPitch = 0, minMargin = 99;
  for (let k = 0; k < 120 * 40; k++) { sim.stepOnce(); maxRoll = Math.max(maxRoll, Math.abs(R.tilt.roll)); maxPitch = Math.max(maxPitch, Math.abs(R.tilt.pitch)); minMargin = Math.min(minMargin, R.tilt.margin); }
  const q = R.pose();
  const along = (q.x - p.x) * p.fwd.x + (q.z - p.z) * p.fwd.z;
  console.log(`boulder d=${d} m (top ${h.toFixed(2)} m): travelled ${along.toFixed(1)} m of 24, max roll ${(maxRoll * 57.3).toFixed(1)} deg, max pitch ${(maxPitch * 57.3).toFixed(1)} deg, min tip margin ${minMargin.toFixed(1)} deg`);
  if (d < 1) atLeast(`climbs ${h.toFixed(2)} m boulder (distance in 40 s at 0.6 m/s)`, along, 22, ' m', 1);
  else atMost(`blocked by ${h.toFixed(2)} m boulder at ${BOULDER_AT} m (does not pass)`, along, BOULDER_AT, ' m', 1);
  check(`upright after ${h.toFixed(2)} m boulder`, Math.abs(R.tilt.roll) < 0.5 && maxRoll * 57.3 < 30, `max roll ${(maxRoll * 57.3).toFixed(1)} deg (want < 30)`);
}
