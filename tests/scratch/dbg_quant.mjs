// Per-step position increments of the rover body at creep speed, in units of the float32 ulp at that coordinate.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
await RAPIER.init();
const sim = new Sim(RAPIER), R = sim.rover;
const run = (n, fn) => { for (let i = 0; i < n; i++) { sim.stepOnce(); fn && fn(); } };
run(480); R.brake = false; R.cmd.vx = Number(process.argv[2] || 0.12); run(600);
const ulp = (x) => Math.pow(2, Math.floor(Math.log2(Math.abs(x))) - 23);
let prev = R.body.translation(); const hx = {}, hz = {}; let sx = 0, sz = 0, svx = 0, svz = 0, n = 0;
run(240, () => { const t = R.body.translation(), v = R.body.linvel(); const dx = Math.round((t.x - prev.x) / ulp(t.x)), dz = Math.round((t.z - prev.z) / ulp(t.z)); hx[dx] = (hx[dx] || 0) + 1; hz[dz] = (hz[dz] || 0) + 1; sx += t.x - prev.x; sz += t.z - prev.z; svx += v.x / 120; svz += v.z / 120; n++; prev = t; });
const t = R.body.translation();
console.log(`pos (${t.x.toFixed(2)}, ${t.z.toFixed(2)}) ulp x ${ulp(t.x).toExponential(2)} z ${ulp(t.z).toExponential(2)} | per-step dx in ulps ${JSON.stringify(hx)} dz ${JSON.stringify(hz)}`);
console.log(`mean per-step displacement: position (${(sx / n * 1e3).toFixed(4)}, ${(sz / n * 1e3).toFixed(4)}) mm vs velocity x dt (${(svx / n * 1e3).toFixed(4)}, ${(svz / n * 1e3).toFixed(4)}) mm | v*dt in ulps (${(svx / n / ulp(t.x)).toFixed(2)}, ${(svz / n / ulp(t.z)).toFixed(2)})`);
