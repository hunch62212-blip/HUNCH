// A wheel track meets a boulder: the rover must climb it (or be stopped by it), never pass through.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
await RAPIER.init();
for (const d of [0.9, 2.2]) {
  const sim = new Sim(RAPIER); sim.modules = sim.modules.filter((m) => m !== sim.autonomy);
  const R = sim.rover, sp = sim.terrain.pad;
  const p0 = R.pose();
  const x0 = p0.x - sp.perp.x * 15, z0 = p0.z - sp.perp.z * 15, yaw = Math.atan2(sp.perp.z, -sp.perp.x);
  R.teleport(x0, z0, yaw); for (let i = 0; i < 240; i++) sim.stepOnce();
  const p = R.pose();
  const right = { x: -p.fwd.z, z: p.fwd.x };
  // boulder 12 m ahead, under the right wheel track
  const bx = p.x + p.fwd.x * 12 + right.x * 2.75, bz = p.z + p.fwd.z * 12 + right.z * 2.75;
  const i = sim.addBoulder(bx, bz, d);
  const rock = sim.terrain.rocks[i];
  R.brake = false; R.cmd.vx = 0.6;
  let maxRoll = 0, maxPitch = 0, minMargin = 99;
  for (let k = 0; k < 120 * 40; k++) { sim.stepOnce(); maxRoll = Math.max(maxRoll, Math.abs(R.tilt.roll)); maxPitch = Math.max(maxPitch, Math.abs(R.tilt.pitch)); minMargin = Math.min(minMargin, R.tilt.margin); }
  const q = R.pose();
  const along = (q.x - p.x) * p.fwd.x + (q.z - p.z) * p.fwd.z;
  console.log(`boulder d=${d} m (top ${(rock.top - rock.y).toFixed(2)} m above ground): travelled ${along.toFixed(1)} m of 24, max roll ${(maxRoll * 57.3).toFixed(1)} deg, max pitch ${(maxPitch * 57.3).toFixed(1)} deg, min tip margin ${minMargin.toFixed(1)} deg, upright ${Math.abs(R.tilt.roll) < 0.5}`);
}
