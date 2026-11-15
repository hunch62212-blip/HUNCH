// what is near a world point on a given site seed: node tests/scratch/dbg_site_near.mjs <seed> <x> <z> [r]
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { heightAt } from '../../src/sim/terrain.js';
await RAPIER.init();
const [seed, X, Z, Rr] = process.argv.slice(2).map(Number), R = Rr || 40;
const sim = new Sim(RAPIER, { seed });
const T = sim.terrain, H = T.hab;
console.log('habitat face', H.face, 'normal', { nx: H.nx, nz: H.nz });
for (const p of sim.ports) console.log('port', p.id, p.name, p.x.toFixed(1), p.z.toFixed(1), 'dist', Math.hypot(p.x - X, p.z - Z).toFixed(1));
const PD = sim.powerDock && sim.powerDock.layout; if (PD) console.log('power dock entry', PD.entry, 'bank', PD.bank, 'arrays', PD.arrays.length, PD.arrays.map((a) => `(${a.x.toFixed(0)},${a.z.toFixed(0)})`).join(' '));
for (const it of T.base || []) { const d = Math.hypot(it.x - X, it.z - Z); if (d < R + 20) console.log('base', it.kind, it.x.toFixed(1), it.z.toFixed(1), 'r', it.r, 'len', it.len, 'd', d.toFixed(1)); }
const rocks = T.rocks.filter((r) => Math.hypot(r.x - X, r.z - Z) < R).map((r) => `(${r.x.toFixed(0)},${r.z.toFixed(0)}) r${(r.r ?? r.radius ?? 0).toFixed(1)}`);
console.log('rocks within', R, 'm:', rocks.length, rocks.slice(0, 20).join(' '));
for (const z of T.restricted) if (Math.hypot(z.x - X, z.z - Z) < R + z.r) console.log('restricted', z);
let lo = Infinity, hi = -Infinity; for (let dx = -10; dx <= 10; dx += 1) for (let dz = -10; dz <= 10; dz += 1) { const h = heightAt(T, X + dx, Z + dz); lo = Math.min(lo, h); hi = Math.max(hi, h); }
console.log('terrain height range within 10 m', lo.toFixed(2), hi.toFixed(2));
