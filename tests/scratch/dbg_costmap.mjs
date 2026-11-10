// Render the planner costmap + current path around the rover to a PPM (debug aid). usage: node dbg_costmap.mjs <simSeconds> <out.ppm> [halfSpanM]
import RAPIER from '@dimforge/rapier3d-compat';
import { writeFileSync } from 'node:fs';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER, process.env.SEED ? { seed: +process.env.SEED } : {}); const A = sim.autonomy;
if (process.env.WARP) sim.timeScale = 50;
A.start('cycle', A.missionCycle());
const T = +process.argv[2];
while (sim.time < T && A.gen) process.env.WARP ? sim.stepAuto() : sim.stepOnce();
const P = A.planner; P.unknownNearHazardCost = 12; P.buildCostmap(sim.slam.routes);
const e = sim.slam.est, span = +(process.argv[4] || 400), W = 400, s = 2 * span / W;
const img = Buffer.alloc(W * W * 3);
for (let j = 0; j < W; j++) for (let i = 0; i < W; i++) {
  const x = e.x - span + i * s, z = e.z - span + j * s, k = P.idx(x, z), o = (j * W + i) * 3;
  const c = k < 0 ? 1e9 : P.cost[k];
  let col = c >= 1e9 ? [220, 40, 40] : c >= 5e8 ? [140, 30, 30] : P.knownMask[k] ? [40, Math.min(255, 60 + c * 25), 60] : (c > 5 ? [90, 60, 120] : [25, 25, 35]);
  img[o] = col[0]; img[o + 1] = col[1]; img[o + 2] = col[2];
}
const dot = (x, z, col, r = 1) => { const i = Math.round((x - e.x + span) / s), j = Math.round((z - e.z + span) / s); for (let a = -r; a <= r; a++) for (let b = -r; b <= r; b++) { const ii = i + a, jj = j + b; if (ii < 0 || jj < 0 || ii >= W || jj >= W) continue; const o = (jj * W + ii) * 3; img[o] = col[0]; img[o + 1] = col[1]; img[o + 2] = col[2]; } };
for (const p of A.path || []) dot(p.x, p.z, [60, 200, 255], 0);
for (const p of sim.slam.trajectory) dot(p.x, p.z, [255, 255, 255], 0);
dot(e.x, e.z, [255, 255, 0], 3);
const g = A.goal; if (g) dot(g.x, g.z, [255, 140, 0], 4);
for (const r of sim.terrain.restricted) for (let a = 0; a < 6.28; a += 0.02) dot(r.x + Math.cos(a) * r.r, r.z + Math.sin(a) * r.r, [255, 100, 200], 0);
writeFileSync(process.argv[3], Buffer.concat([Buffer.from(`P6 ${W} ${W} 255\n`), img]));
console.log('t', sim.time.toFixed(0), 'est', e.x.toFixed(0), e.z.toFixed(0), 'goal', g && g.x.toFixed(0), g && g.z.toFixed(0), 'phase', A.phase, '|', A.detail, 'lethal', P.lethalCount, 'unknownInflated', P.unknownInflated, 'blocked', P.blocked.length);
