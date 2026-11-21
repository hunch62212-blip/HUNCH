// Camera / stereo lighting: shadow ray toward the sun (docs/changes/B.md). Mock sim, no Rapier.
import { Sensors } from '../src/sim/sensors.js';
let fail = 0; const ok = (c, m) => { console.log(`${c ? 'PASS' : 'FAIL'} ${m}`); if (!c) fail++; };
const e = Math.PI / 20, sun = { x: Math.cos(e), y: Math.sin(e), z: 0 };
let blockerX = null, calls = 0;   // a wall at x = blockerX (taller than everything) casts the shadow
const world = { castRay(r, max) { calls++; if (blockerX == null) return null; const t = (blockerX - r.origin.x) / r.dir.x; return t > 0 && t <= max ? { toi: t } : null; } };
const self = { sim: { sunDir: sun, world, rover: { body: {} } }, ray: { origin: {}, dir: {} } };
const lit = (x) => Sensors.prototype._sunlitAt.call(self, x, 0, 0, 0, 1, 0);
self._castRay = Sensors.prototype._castRay.bind(self);
ok(lit(0) === true, 'open ground in sun: lit');
blockerX = 40; ok(lit(0) === false, 'wall 40 m sunward: point in its shadow');
ok(lit(60) === true, 'point past the wall (sun side): lit');
blockerX = 200; ok(lit(0) === true, `blocker beyond shadowRayM: ignored (1 ray each, ${calls} rays for 4 checks)`);
self.sim.sunDir = { x: 1, y: -0.05, z: 0 }; const c0 = calls; ok(lit(0) === false && calls === c0, 'night: dark with no ray cast');
console.log(fail ? `${fail} FAIL` : 'all PASS'); process.exit(fail ? 1 : 0);
