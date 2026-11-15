// Wall-clock cost of the LiDAR: sim seconds per wall second at several channel x hRes x rate settings (no mission, rover idle).
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER), S = sim.sensors;
for (let i = 0; i < 240; i++) sim.stepOnce();
const cfgs = process.argv.slice(2).map((a) => a.split(',').map(Number));
for (const [ch, hr, hz] of cfgs.length ? cfgs : [[16, 180, 5], [32, 360, 10]]) {
  S.lidarAuto = false; S.lidarCfg.rateHz = hz; S.setLidar(ch, hr);
  S._partial = null;
  const t0 = performance.now(); let scans0 = S.scanId;
  for (let i = 0; i < 120 * 6; i++) sim.stepOnce();
  const ms = performance.now() - t0;
  console.log(`${ch}ch x ${hr} @ ${hz} Hz: ${(ms / 6).toFixed(0)} ms wall per sim second, ${S.scanId - scans0} scans/6 s, last scan ${S.scan?.n} pts`);
}
S.lidarOn = false; S._partial = null;
{ const t0 = performance.now(); for (let i = 0; i < 720; i++) sim.stepOnce(); console.log(`LiDAR off: ${((performance.now() - t0) / 6).toFixed(0)} ms wall per sim second`); }
