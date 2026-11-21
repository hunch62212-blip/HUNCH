// Unit checks of the single mass-properties path (rover.js massProps / syncMass). run: node tests/unit_mass.mjs
//  1. free fall: the system CoM (sprung + unsprung at their actual stroke + held loads) drops g t^2 / 2 = 0.81 m in 1 s
//  2. static: sum of the tyre normal loads = total mass x g (flat pad, time average)
//  3. coast-down on a 10 deg ramp with free-rolling wheels: a = g (sin t - mu_r cos t) x M / (M + sum I_w / r^2)
//     (mu_r = measured sum Rc / sum N; the wheel spin inertia factor is printed separately)
import { check, within } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { ROVER, MOON } from '../src/sim/config.js';
import { G_TERRAIN } from '../src/sim/world.js';
import * as THREE from 'three';
await RAPIER.init();
const sim = new Sim(RAPIER), R = sim.rover, g = MOON.g, B = ROVER.bogie, mu = ROVER.unsprungMassPerBogie;
const run = (sec, fn) => { const n = Math.round(sec * 120); for (let i = 0; i < n; i++) { fn && fn(); sim.stepOnce(); } };
const resetStruts = () => { for (const b of R.bogies) { b.L = B.Lnom; b.Ldot = 0; b.hpVelPrev = null; for (const w of b.wheels) { w.omega = 0; w.integ = 0; w.groundY = null; w.anchor = null; w.deltaPrev = null; w.sink = 0; } } };
// system CoM: Rapier CoM (unsprung at nominal stroke) corrected for the actual strut lengths
const sysCom = () => {
  const c = R.body.worldCom(), r = R.body.rotation(), q = new THREE.Quaternion(r.x, r.y, r.z, r.w), M = R.totalMass;
  const v = new THREE.Vector3(c.x, c.y, c.z);
  for (const b of R.bogies) v.add(new THREE.Vector3(0, -(b.L - B.Lnom) * mu / M, 0).applyQuaternion(q));
  return v;
};

// ---- 1. static load on the pad --------------------------------------------------------------------------------------
run(4);
let sN = 0, n = 0;
run(4, () => { sN += R.wheels.reduce((a, w) => a + w.load, 0); n++; });
const Mtot = R.totalMass;
within('mass path: Rapier body mass = massProps total', R.body.mass() / Mtot, 0.999, 1.001);
within('static: sum N / (M g), 4 s mean on the pad', sN / n / (Mtot * g), 0.995, 1.005, '', 4);

// ---- 2. free fall ---------------------------------------------------------------------------------------------------
{ const p = R.body.translation();
  R.body.setTranslation({ x: p.x, y: p.y + 300, z: p.z }, true); R.body.setRotation({ x: 0, y: 0, z: 0, w: 1 }, true);
  R.body.setLinvel({ x: 0, y: 0, z: 0 }, true); R.body.setAngvel({ x: 0, y: 0, z: 0 }, true); resetStruts();
  sim.stepOnce(); // let Rapier pick up the pose
  R.body.setLinvel({ x: 0, y: 0, z: 0 }, true); R.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
  for (const b of R.bogies) b.Ldot = 0;
  const y0 = sysCom().y; run(1); const drop = y0 - sysCom().y;
  within('free fall: system CoM drop in 1 s (g/2 = 0.810 m, +-1 cm)', drop, 0.800, 0.820, ' m', 4); }

// ---- 3. 10 deg coast-down ramp --------------------------------------------------------------------------------------
{ const th = 10 * Math.PI / 180, qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -th);
  const n3 = new THREE.Vector3(0, 1, 0).applyQuaternion(qz), down = new THREE.Vector3(1, 0, 0).applyQuaternion(qz);
  const p = R.body.translation(), C = new THREE.Vector3(p.x, p.y + 800, p.z);
  // the ramp is a heightfield like the terrain's (on a cuboid face the wheel loads chatter: pre-existing, also with the old rover.js), sloping down to +x
  const L = 600, W = 600, hx = (x) => -Math.tan(th) * x, ramp = sim.world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(C.x, C.y, C.z));
  // Rapier heightfield: heights[i + (NG + 1) j], i along z (rows), j along x (columns)
  const NG = 120, h = new Float32Array((NG + 1) ** 2); for (let i = 0; i <= NG; i++) for (let j = 0; j <= NG; j++) h[i + (NG + 1) * j] = hx((j / NG - 0.5) * L);
  sim.world.createCollider(RAPIER.ColliderDesc.heightfield(NG, NG, h, { x: L, y: 1, z: W }).setFriction(0.8).setCollisionGroups(G_TERRAIN), ramp);
  const S = C.clone().addScaledVector(down, -150 / Math.cos(th)), P0 = S.clone().addScaledVector(n3, 0.35);
  R.body.setTranslation(P0, true); R.body.setRotation(qz, true); R.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
  R.body.setLinvel({ x: down.x * 1.0, y: down.y * 1.0, z: 0 }, true); resetStruts();
  R.brake = false; for (const w of R.wheels) { w.fault = 'motor'; w.omega = 1.0 / ROVER.wheel.radius; }
  run(3, () => {}); // settle onto the ramp while rolling
  const v0 = R.body.linvel(), s0 = v0.x * down.x + v0.y * down.y; let sRc = 0, sN3 = 0, sFx = 0, k = 0;
  run(4, () => { for (const w of R.wheels) { sRc += w.Rc || 0; sN3 += w.load; sFx += w.Fx || 0; } k++; });
  { const r = R.body.rotation(), f = new THREE.Vector3(1, 0, 0).applyQuaternion(new THREE.Quaternion(r.x, r.y, r.z, r.w));
    within('rover pitch on the ramp (nose down, deg)', Math.asin(f.y) * 180 / Math.PI, -10.5, -9.5, ' deg', 2); }
  const v1 = R.body.linvel(), s1 = v1.x * down.x + v1.y * down.y, a = (s1 - s0) / 4;
  // mu_r: the tyres' measured longitudinal resistance (Bekker compaction Rc + the shear that spins the wheels up + idler drag)
  // over their normal load, i.e. the effective rolling-resistance coefficient the model produced in this run
  const muR = -sFx / sN3, muRc = sRc / sN3, M = R.totalMass;
  const aPred = g * (Math.sin(th) - muR * Math.cos(th));
  console.log(`coast: v ${s0.toFixed(3)} -> ${s1.toFixed(3)} m/s, a ${a.toFixed(4)} m/s^2, mu_r ${muR.toFixed(4)} (Bekker Rc only ${muRc.toFixed(4)}), g(sin t - mu_r cos t) ${aPred.toFixed(4)} m/s^2, sum N / (M g cos t) ${(sN3 / k / (M * g * Math.cos(th))).toFixed(4)}`);
  within('coast-down 10 deg: a / g(sin t - mu_r cos t) (+-3 %)', a / aPred, 0.97, 1.03, '', 4);
  check('coast-down rover stayed on the ramp', R.wheels.every((w) => w.contact), `${R.wheels.filter((w) => w.contact).length}/6 in contact`); }
