// Regression: driving into the cargo lander (screenshot bug 2026-11-09, seed 866201175). The lander's colliders
// were 2.6x the drawn model, and wheel casts treated the leg struts as ground, so the rover climbed a leg to
// 64-70 deg pitch and wedged on its side. Also checks the camera now sees the lander's surveyed tags and the
// engineering quantities (src/sim/dynamics.js) are sane.
import { check, atMost, within, atLeast } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { WORLD } from '../src/sim/config.js';
import { LANDER_SCALE } from '../src/sim/world.js';
import { vehicleDynamics } from '../src/sim/dynamics.js';
await RAPIER.init();

const sim = new Sim(RAPIER, { seed: 866201175 });
const R = sim.rover, deg = (r) => Math.abs(r * 180 / Math.PI);
const run = (s, cb) => { for (let i = 0; i < s * 120; i++) { sim.stepOnce(); cb && cb(); } };
run(3);
// 1. lander tags: the rover starts ~58 m from the lander, facing it on this seed
let tags = new Set();
run(2, () => { for (const d of sim.sensors.detections) if (d.id.startsWith('LANDER')) tags.add(d.id); });
atLeast('camera sees lander tags at the start', tags.size, 1, ' tags');

// 2. drive into a leg and into the lander centre at 3 m/s, forward and back
const P = WORLD.pad, LS = LANDER_SCALE, p0 = R.pose();
for (const [name, tx, tz] of [['leg', P.x + Math.cos(Math.PI / 4) * 5.4 * LS, P.z + Math.sin(Math.PI / 4) * 5.4 * LS], ['centre', P.x, P.z]]) {
  R.teleport(p0.x, p0.z, Math.atan2(-(tz - p0.z), tx - p0.x)); run(2);
  let maxPitch = 0, maxRoll = 0, maxRise = 0;
  const y0 = R.pose().y;
  const track = () => { const t = R.telemetry(); maxPitch = Math.max(maxPitch, deg(t.pitch)); maxRoll = Math.max(maxRoll, deg(t.roll)); maxRise = Math.max(maxRise, R.pose().y - y0); };
  R.brake = false; R.cmd.vx = 3; run(36, track);
  atMost(`ram lander ${name}: max pitch driving in`, maxPitch, 15, ' deg', 1);
  atMost(`ram lander ${name}: body rise driving in`, maxRise, 1.5, ' m', 2);
  R.cmd.vx = -2; run(20, track);
  atMost(`ram lander ${name}: max roll (in + back out)`, maxRoll, 40, ' deg', 1);
  R.cmd.vx = 0; run(4);
  const t = R.telemetry();
  check(`ram lander ${name}: ends upright on its wheels`, deg(t.pitch) < 10 && deg(t.roll) < 10 && R.wheels.filter((w) => w.contact).length >= 5,
    `pitch ${deg(t.pitch).toFixed(1)} roll ${deg(t.roll).toFixed(1)} contacts ${R.wheels.filter((w) => w.contact).length}`);
}

// 3. dynamics quantities on open ground at 2 m/s
{ const sp = sim.terrain.pad; R.teleport(p0.x - sp.perp.x * 15, p0.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); }
run(3); R.brake = false; R.cmd.vx = 2; run(10);
const d = vehicleDynamics(R);
within('dynamics: static stability factor (empty, flat)', d.SSF, 1.5, 2.5, '', 2);
within('dynamics: stopping distance at 2 m/s = v t_r + v^2/2a', d.stopDist, 4.0, 4.6, ' m', 2);
within('dynamics: cost of transport', d.cot, 0.01, 0.2, '', 3);
within('dynamics: wheel load sum / weight', d.Nsum / (d.m * d.g), 0.9, 1.1, '', 3);
