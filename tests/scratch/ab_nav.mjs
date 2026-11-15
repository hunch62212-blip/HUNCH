// A/B: navigation smoothness, empty rover, pad -> habitat, first N sim-minutes (default 10). Truth sampled at 20 Hz.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover;
const MIN = Number(process.argv[2] || 10);
const warns = []; let paths = 0; sim.on('log', (e) => { if (e.level === 'warn' || e.level === 'err') warns.push(`[${(e.t / 60).toFixed(1)} min] ${e.msg.slice(0, 100)}`); }); sim.on('path', () => paths++);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false;
const H = sim.terrain.hab, g = { x: WORLD.habitat.x + H.nx * 60, z: WORLD.habitat.z + H.nz * 60 };
A.start('goto', A.navTo({ x: g.x, z: g.z, name: 'hab approach', tol: 4 }));
const t0 = sim.time, d0 = R.odo.dist, wall0 = performance.now();
let n = 0, stops = 0, stopT = 0, slowT = 0, wz2 = 0, rev = 0, lastSign = 0, cmdRev = 0, lastCmdSign = 0, switches = 0, lastSw = A._pathSwitchT, pivots = 0, inPivot = false, lat2 = 0, moveN = 0, i = 0;
while (A.gen && sim.time - t0 < MIN * 60) {
  sim.stepOnce(); if (++i % 6) continue; n++;
  const v = R.telemetry().speed, wz = R.body.angvel().y, t = sim.time - t0;
  if (A._pathSwitchT !== lastSw) { lastSw = A._pathSwitchT; switches++; }
  const piv = /pivot/.test(A.detail || ''); if (piv && !inPivot) pivots++; inPivot = piv;
  if (t > 15 && !piv) {
    if (v < 0.05) { stopT += 0.05; if (stopT > 1 && stopT - 0.05 <= 1) stops++; } else stopT = 0;
    if (v < 0.4) slowT += 0.05;
    if (v > 0.5) { moveN++; wz2 += wz * wz; lat2 += (v * wz) ** 2;
      const s = Math.abs(wz) > 0.01 ? Math.sign(wz) : 0; if (s && lastSign && s !== lastSign) rev++; if (s) lastSign = s;
      const cs = Math.abs(R.cmd.wz) > 0.005 ? Math.sign(R.cmd.wz) : 0; if (cs && lastCmdSign && cs !== lastCmdSign) cmdRev++; if (cs) lastCmdSign = cs; }
  }
}
const T = (sim.time - t0) / 60, dist = R.odo.dist - d0, wall = (performance.now() - wall0) / 1000;
console.log(`${T.toFixed(1)} sim-min: ${dist.toFixed(0)} m, avg ${(dist / (T * 60)).toFixed(2)} m/s | stops ${stops} | time under 0.4 m/s ${slowT.toFixed(0)} s | pivots ${pivots}`);
console.log(`yaw rate RMS ${(Math.sqrt(wz2 / Math.max(1, moveN)) * 57.3).toFixed(2)} deg/s | yaw-rate reversals ${(rev / T).toFixed(1)} /min | yaw-command reversals ${(cmdRev / T).toFixed(1)} /min | lateral accel RMS ${Math.sqrt(lat2 / Math.max(1, moveN)).toFixed(3)} m/s2`);
console.log(`path switches ${switches} | path events ${paths} | warnings ${warns.length} | arrived ${!A.gen} | sim speed ${(T * 60 / wall).toFixed(1)}x real time (wall ${wall.toFixed(0)} s)`);
for (const w of warns) console.log('  ' + w);
