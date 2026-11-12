// Mission 11 (AI logistics manager) and Mission 12 (own capability: active load levelling).
// M11 pass rules: the plan lists every staged cargo with a finite ETA and energy; the life-support module is first;
//   after "priority" swaps the categories the order flips; a battery below the reserve marks every leg infeasible.
// M12 pass rules: parked across a 5-9 deg side slope, the levelling struts take out at least half of the body roll
//   (the controller trims 85 % of the error each pass, limited by strut travel) and costs at most 2 deg of static tip margin.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER, WORLD } from '../src/sim/config.js';
import { heightAt } from '../src/sim/terrain.js';
import { runCommand } from '../src/sim/commands.js';
import { check, atLeast, atMost, equal } from './lib/check.mjs';
await RAPIER.init();
const DEG = Math.PI / 180;

{ // ---- M11 ----
  const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover;
  const p0 = A.logisticsPlan();
  equal('M11: plan lists every staged cargo', p0.length, sim.cargo.length);
  check('M11: every leg has a finite ETA and energy', p0.every((m) => Number.isFinite(m.eta) && m.eta > 0 && Number.isFinite(m.energy) && m.energy > 0), p0.map((m) => `${m.id} ${m.eta.toFixed(0)} min ${(m.energy / 1000).toFixed(1)} kWh`).join(', '));
  check('M11: every leg feasible on a full battery', p0.every((m) => m.feasible));
  equal('M11: life-support module is delivered first', p0[0].category, 'life-support');
  equal('M11: three requests in the plan, by category weight', p0.map((m) => `${m.id.slice(-1)}:${m.category}`).join(' > '), 'A:life-support > C:power > B:habitat-supplies');
  runCommand(sim, {}, 'priority A science'); runCommand(sim, {}, 'priority B emergency-repair');
  const p1 = A.logisticsPlan();
  equal('M11: "priority" re-orders the plan', p1.map((m) => m.id.slice(-1)).join(' > '), 'B > C > A');
  atLeast('M11: score gap follows the category weights', Math.min(p1[0].score - p1[1].score, p1[1].score - p1[2].score), 60, '', 0);
  equal('M11: each request goes to its own port', [...new Set(p1.map((m) => m.port))].length, sim.cargo.length);
  R.power.wh = 0.1 * ROVER.battery.capacityWh; R.power.soc = 0.1;
  check('M11: below the battery reserve no leg is feasible', A.logisticsPlan().every((m) => !m.feasible));
}

{ // ---- M12: active levelling on a side slope ----
  const sim = new Sim(RAPIER), R = sim.rover, T = sim.terrain;
  // look for an even 5-9 deg slope, clear of rocks, the pad and the base
  let site = null;
  const lim = WORLD.size / 2 - 150;
  search: for (let x = -lim; x <= lim; x += 15) for (let z = -lim; z <= lim; z += 15) {
    if (Math.hypot(x - WORLD.pad.x, z - WORLD.pad.z) < 250 || Math.hypot(x - WORLD.habitat.x, z - WORLD.habitat.z) < 300) continue;
    const h = (dx, dz) => heightAt(T, x + dx, z + dz), d = 6;
    const gx = (h(d, 0) - h(-d, 0)) / (2 * d), gz = (h(0, d) - h(0, -d)) / (2 * d), slope = Math.atan(Math.hypot(gx, gz)) / DEG;
    if (slope < 5 || slope > 9) continue;
    let dev = 0; const h0 = h(0, 0);
    for (const [dx, dz] of [[d, d], [d, -d], [-d, d], [-d, -d], [d, 0], [-d, 0], [0, d], [0, -d], [3, 3], [-3, -3], [3, -3], [-3, 3]]) dev = Math.max(dev, Math.abs(h(dx, dz) - (h0 + gx * dx + gz * dz)));
    if (dev > 0.12) continue;
    if (T.rocks.some((r) => !r.pebble && Math.hypot(r.x - x, r.z - z) < 14)) continue;
    const L = Math.hypot(gx, gz), fx = -gz / L, fz = gx / L;       // heading along the contour: the slope is all roll
    site = { x, z, yaw: Math.atan2(-fz, fx), slope }; break search;
  }
  if (!check('M12: found an even 5-9 deg side slope on the test map', !!site, site ? `(${site.x}, ${site.z}), ${site.slope.toFixed(1)} deg` : 'none')) process.exit(1);
  const settle = (s) => { for (let i = 0; i < s * 120; i++) sim.stepOnce(); };
  R.teleport(site.x, site.z, site.yaw); sim.streamRocks([{ x: site.x, z: site.z }]); R.brake = true;
  runCommand(sim, {}, 'level off'); settle(12);
  const rollOff = Math.abs(R.tilt.roll) / DEG, marginOff = R.tilt.margin;
  runCommand(sim, {}, 'level on'); settle(30);
  const rollOn = Math.abs(R.tilt.roll) / DEG, marginOn = R.tilt.margin;
  atLeast('M12: body roll on the slope with levelling off', rollOff, 3.5, ' deg', 2);
  atMost('M12: body roll with levelling on (fraction of the unlevelled roll)', rollOn / rollOff, 0.5, '', 2);
  // the nearest tip edge here is fore-aft, which roll trim does not move; the check is that levelling costs no real margin
  check('M12: static tip margin not reduced by more than 2 deg', marginOn - marginOff >= -2, `${marginOff.toFixed(1)} deg off -> ${marginOn.toFixed(1)} deg on`);
  const p = R.pose();
  atMost('M12: rover held its place on the slope while levelling', Math.hypot(p.x - site.x, p.z - site.z), 1.0, ' m', 2);
}

{ // ---- solar + battery: parked idle in sunlight the rover opens the keep-alive array and charges; stows before driving ----
  const sim = new Sim(RAPIER), H = sim.health, R = sim.rover;
  sim.idleSolar = true;
  runCommand(sim, {}, 'time noon');
  for (let i = 0; i < 60 * 40; i++) sim.stepOnce();
  check('solar: idle in sunlight -> array deploying/deployed', H.array.state === 'deploying' || H.array.state === 'deployed', H.array.state);
  for (let i = 0; i < 60 * 40; i++) sim.stepOnce();
  for (let i = 0; i < 60 * 150 && H.array.state !== 'deployed'; i++) sim.stepOnce();
  for (let i = 0; i < 120; i++) sim.stepOnce();
  check('solar: array deployed', H.array.state === 'deployed', H.array.state);
  atLeast('solar: array output when deployed', R.power.auxSolarW, 100, ' W', 0);
  sim.idleSolar = false; H.array.state = 'stowed'; H.array.k = 0;
  for (let i = 0; i < 60 * 40; i++) sim.stepOnce();
  equal('solar: off by default (headless tests unchanged)', H.array.state, 'stowed');
}
{ // ---- saved maps from an older pad/port layout are rejected ----
  const sim = new Sim(RAPIER), d = sim.slam.serialize();
  check('map: current layout loads', sim.slam.deserialize(d));
  check('map: older layout rejected', !sim.slam.deserialize({ ...d, layout: 1 }));
  check('map: unversioned (pre-3-port) map rejected', !sim.slam.deserialize({ ...d, layout: undefined }));
}
