// cycle: second-by-second trace before the loaded rollover (tilt, speed, command, true vs mapped slope).
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { Q_TERRAIN } from '../../src/sim/world.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover, S = sim.slam;
A.start('cycle', A.missionCycle());
const t0 = +(process.argv[2] || 285), t1 = +(process.argv[3] || 325);
const ray = new RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 0, y: -1, z: 0 });
const trueH = (x, z) => { ray.origin.x = x; ray.origin.y = 400; ray.origin.z = z; const h = sim.world.castRay(ray, 800, true, undefined, Q_TERRAIN, undefined, R.body); return h ? 400 - h.timeOfImpact : NaN; };
const slope = (hf, x, z, d = 2) => Math.atan(Math.hypot((hf(x + d, z) - hf(x - d, z)) / (2 * d), (hf(x, z + d) - hf(x, z - d)) / (2 * d))) * 57.3;
sim.on('log', (e) => { if (e.t > t0 - 20 && e.level !== 'info') console.log(`[${e.t.toFixed(1)}] ${e.level} ${e.msg}`); });
let last = 0;
while (A.gen && sim.time < t1) {
  sim.stepOnce();
  if (sim.time >= t0 && sim.time - last >= 1) { last = sim.time; const p = R.pose(), e = S.est, tl = R.telemetry();
    console.log(`t=${sim.time.toFixed(0)} pitch ${(R.tilt.pitch * 57.3).toFixed(1)} roll ${(R.tilt.roll * 57.3).toFixed(1)} margin ${(R.tilt.marginF ?? R.tilt.margin)?.toFixed(1)} | v ${tl.fwdSpeed.toFixed(2)} cmd ${R.cmd.vx.toFixed(2)}/${(R.cmd.wz ?? R.cmd.yawRate ?? 0).toFixed(3)} | true slope ${slope(trueH, p.x, p.z).toFixed(1)} map slope ${slope((x, z) => S.heightAt(x, z), e.x, e.z).toFixed(1)} | ${A.detail}`); }
}
