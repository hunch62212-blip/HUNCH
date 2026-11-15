// Where the LiDAR time goes: ray casting (_scanSector) vs SLAM scan processing (onScan) vs the rest.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER), S = sim.sensors;
for (let i = 0; i < 240; i++) sim.stepOnce();
const acc = { ray: 0, slam: 0, fin: 0 };
const wrap = (o, k, key) => { const f = o[k].bind(o); o[k] = (...a) => { const t = performance.now(); const r = f(...a); acc[key] += performance.now() - t; return r; }; };
wrap(S, '_scanSector', 'ray'); wrap(sim.slam, 'onScan', 'slam'); wrap(S, '_finishScan', 'fin');
for (const [ch, hr, hz] of process.argv.slice(2).map((a) => a.split(',').map(Number))) {
  S.lidarAuto = false; S.lidarCfg.rateHz = hz; S.setLidar(ch, hr); S._partial = null;
  acc.ray = acc.slam = acc.fin = 0;
  const t0 = performance.now(); for (let i = 0; i < 720; i++) sim.stepOnce(); const tot = performance.now() - t0;
  console.log(`${ch}ch x ${hr} @ ${hz} Hz: total ${(tot / 6).toFixed(0)} ms/s | rays ${(acc.ray / 6).toFixed(0)} | slam.onScan ${(acc.slam / 6).toFixed(0)} (incl. finish ${(acc.fin / 6).toFixed(0)})`);
}
