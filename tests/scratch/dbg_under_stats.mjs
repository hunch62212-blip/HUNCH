// Drive-under statistics for one cargo: TRUE lateral offset from the module axis (cm) and heading (deg) at set distances to go,
// the ground slope across the stand, and the outcome. argv: cargo index, noise skip.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
import { gauss } from '../../src/sim/sensors.js';
import { heightAt } from '../../src/sim/terrain.js';
await RAPIER.init();
const ci = Number(process.argv[2] || 2), skip = Number(process.argv[3] || 0);
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover, c = sim.cargo[ci], T = c.T, ct = c.body.translation();
for (let i = 0; i < skip; i++) gauss();
const cy = Math.cos(c.yaw), sy = Math.sin(c.yaw);
// cross slope at the stand: height 4 m to the right minus 4 m to the left of the module axis (right = +lat)
const hR = heightAt(sim.terrain, ct.x + sy * 4, ct.z + cy * 4), hL = heightAt(sim.terrain, ct.x - sy * 4, ct.z - cy * 4);
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - cy * back, ct.z + sy * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const marks = [12, 6, 3, 1.5, 0.8, 0.3], got = {}; let fail = '';
sim.on('log', (e) => { if (/not centred/.test(e.msg) && !fail) fail = 'RETRY'; });
const t0 = sim.time; A.start('pickup', A.acquire(c));
while (A.gen && sim.time - t0 < 110 && !fail) { sim.stepOnce(); if (!/Drive under/.test(A.phase)) continue;
  const p = R.pose(), dx = p.x - ct.x, dz = p.z - ct.z, along = -(dx * cy - dz * sy) - (T.carryX || 0), lat = dx * sy + dz * cy;
  for (const m of marks) if (got[m] == null && along <= m) got[m] = `${(lat * 100).toFixed(0)}/${(wrap(p.yaw - c.yaw) * 57.3).toFixed(1)}`; }
console.log(`${c.id} skip ${String(skip).padStart(4)} | cross slope ${(Math.atan2(hR - hL, 8) * 57.3).toFixed(2)} deg (+ = right side higher) | lat cm / yaw deg at ${marks.map((m) => `${m} m: ${got[m] ?? '-'}`).join('  ')} | ${fail || (R.loaded || /Rise|Clamp/.test(A.phase) ? 'OK first attempt' : 'running')}`);
