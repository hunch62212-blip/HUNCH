// Physical body motion during ride-height moves: the body's height (truth), not the commanded lift.pos.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER), R = sim.rover;
const run = (sec, fn) => { for (let i = 0; i < sec * 120; i++) { sim.stepOnce(); fn && fn((i + 1) / 120); } };
run(5);
function move(name, target, sec = 16) {
  const y0 = R.pose().y, d = target - R.lift.pos; R.lift.target = target;
  const ys = []; let cmdT = null;
  run(sec, (t) => { ys.push(R.pose().y - y0); if (cmdT == null && Math.abs(R.lift.pos - target) < 0.002) cmdT = t; });
  const fin = ys[ys.length - 1], frac = (f) => { const i = ys.findIndex((y) => Math.abs(y) >= Math.abs(fin) * f); return (i + 1) / 120; };
  // settle: last time the body was more than 2 cm from its final height
  let settle = 0; for (let i = ys.length - 1; i >= 0; i--) if (Math.abs(ys[i] - fin) > 0.02) { settle = (i + 1) / 120; break; }
  let vPk = 0, aPk = 0, over = 0; const W = 12; // 0.1 s windows
  for (let i = W; i < ys.length; i++) { const v = (ys[i] - ys[i - W]) / 0.1; vPk = Math.max(vPk, Math.abs(v)); if (i >= 2 * W) { const v0 = (ys[i - W] - ys[i - 2 * W]) / 0.1; aPk = Math.max(aPk, Math.abs(v - v0) / 0.1); } over = Math.max(over, Math.sign(d) * (ys[i] - fin)); }
  const v1 = Math.abs(ys[23] - ys[11]) / 0.1; // body speed 0.1-0.2 s after the command starts
  console.log(`${name.padEnd(24)} cmd ${cmdT?.toFixed(2)} s | body: 10% ${frac(0.1).toFixed(2)} s  90% ${frac(0.9).toFixed(2)} s  within 2 cm ${settle.toFixed(2)} s | moved ${fin.toFixed(2)} m | peak v ${vPk.toFixed(2)} m/s  peak a ${aPk.toFixed(2)} m/s2 | v at 0.2 s ${v1.toFixed(2)} | overshoot ${(over * 100).toFixed(1)} cm`);
}
move('kneel 0 -> -0.65', ROVER.ride.kneel);
move('rise kneel -> carry', ROVER.ride.carry);
move('squat carry -> kneel', ROVER.ride.kneel);
