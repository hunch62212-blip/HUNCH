// SLAM hazard classes + per-cell sample count around the rover (debug aid). usage: node dbg_cls.mjs <simS> <out.ppm> [halfSpan]
import RAPIER from '@dimforge/rapier3d-compat';
import { writeFileSync } from 'node:fs';
import { Sim } from '../../src/sim/sim.js';
import { CLS } from '../../src/sim/slam.js';
import { heightAt } from '../../src/sim/terrain.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy;
if (process.env.WARP) sim.timeScale = 50;
A.start('cycle', A.missionCycle());
while (sim.time < +process.argv[2] && A.gen) process.env.WARP ? sim.stepAuto() : sim.stepOnce();
const S = sim.slam, e = S.est, span = +(process.argv[4] || 120), W = 480, s = 2 * span / W;
const p = sim.rover.pose(), ox = p.x - e.x, oz = p.z - e.z;
const col = { [CLS.FREE]: [70, 120, 80], [CLS.ROUGH]: [130, 130, 70], [CLS.ROCK]: [235, 120, 40], [CLS.SLOPE]: [200, 160, 60], [CLS.CRATER]: [190, 60, 200], [CLS.LETHAL]: [235, 50, 40] };
const img = Buffer.alloc(W * W * 3); const hist = {}; let errs = [];
for (let j = 0; j < W; j++) for (let i = 0; i < W; i++) {
  const x = e.x - span + i * s, z = e.z - span + j * s, inf = S.infoAt(x, z), o = (j * W + i) * 3;
  let c = [20, 20, 28];
  if (inf && inf.n) { c = col[inf.cls] || [90, 90, 90]; hist[inf.cls] = (hist[inf.cls] || 0) + 1; if (i % 4 === 0 && j % 4 === 0) errs.push(inf.h - heightAt(sim.terrain, x + ox, z + oz)); }
  img[o] = c[0]; img[o + 1] = c[1]; img[o + 2] = c[2];
}
writeFileSync(process.argv[3], Buffer.concat([Buffer.from(`P6 ${W} ${W} 255\n`), img]));
errs.sort((a, b) => a - b); const q = (f) => errs[Math.floor(f * (errs.length - 1))].toFixed(2);
console.log('t', sim.time.toFixed(0), 'cls hist', JSON.stringify(hist), 'map-truth height err p5/p50/p95', q(0.05), q(0.5), q(0.95), 'slamErr', Math.hypot(ox, oz).toFixed(2), 'limits', JSON.stringify(S.limits));
