// Does stereo fusion add false hazards? Drive 60 m straight on open ground with stereo on / off (argv[2]); count hazard classes in the
// mapped corridor and measure the height error of the provisional stereo cells against the true terrain.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
import { CLS, RES, TILE } from '../../src/sim/slam.js';
import { heightAt } from '../../src/sim/terrain.js';
await RAPIER.init();
ROVER.camera.stereo = process.argv[2] !== 'off'; if (process.argv[3]) ROVER.camera.fuseSigma = +process.argv[3];
const NOLIDAR = process.argv[4] === 'nolidar';
const sim = new Sim(RAPIER), R = sim.rover, S = sim.sensors, M = sim.slam;
{ const sp = sim.terrain.pad, p = R.pose(); R.teleport(p.x - sp.perp.x * 15, p.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); }
const run = (sec, fn) => { for (let i = 0; i < sec * 120; i++) { sim.stepOnce(); fn && fn(); } };
run(4); R.brake = false; M.clear(); M.relocalize(R.pose()); if (NOLIDAR) { S.lidarOn = false; S._partial = null; }
let fused = 0, frames = 0, provPeak = 0; const tick = () => { if (S.stereo && S.stereo.t !== tick.t && (tick.t = S.stereo.t)) { fused += S.stereo.fused; frames++; } };
const count = () => { let n = 0; for (const t of M.tiles.values()) if (t.st) for (let i = 0; i < t.st.length; i++) if (t.st[i] && t.n[i]) n++; return n; };
R.cmd.vx = 1.5; run(40, () => { tick(); if (sim.stepCount % 120 === 0) provPeak = Math.max(provPeak, count()); }); R.cmd.vx = 0; run(2);
const off = { x: M.est.x - R.pose().x, z: M.est.z - R.pose().z, y: M.est.y - R.pose().y };
const cls = {}; let known = 0, prov = 0, e2 = 0, eMax = 0, l2 = 0, ln = 0;
for (const t of M.tiles.values()) { if (t.dirty) M.computeHazard(t);
  for (let i = 0; i < t.n.length; i++) { if (!t.n[i]) continue; known++; const c = Object.keys(CLS).find((k) => CLS[k] === t.cls[i]); cls[c] = (cls[c] || 0) + 1;
    const wx = (t.tx * TILE + (i % TILE) + 0.5) * RES - off.x, wz = (t.tz * TILE + Math.floor(i / TILE) + 0.5) * RES - off.z, err = t.h[i] - off.y - heightAt(sim.terrain, wx, wz);
    if (t.st && t.st[i]) { prov++; e2 += err * err; eMax = Math.max(eMax, Math.abs(err)); } else if (Math.abs(err) < 1) { l2 += err * err; ln++; } } }
console.log(`${NOLIDAR ? 'LiDAR OFF, ' : ''}stereo ${ROVER.camera.stereo ? 'ON (fuseSigma ' + ROVER.camera.fuseSigma + ')' : 'OFF'}: known cells ${known} | classes ${JSON.stringify(cls)} | stereo frames ${frames}, points fused ${fused}, provisional cells peak ${provPeak}, left ${prov}${prov ? `, height error RMS ${(Math.sqrt(e2 / prov) * 100).toFixed(1)} cm max ${(eMax * 100).toFixed(0)} cm` : ''} | LiDAR cells height error RMS ${(Math.sqrt(l2 / Math.max(1, ln)) * 100).toFixed(1)} cm | SLAM pos err ${Math.hypot(off.x, off.z).toFixed(2)} m`);
