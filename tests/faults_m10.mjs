// Mission 10, the six "something goes wrong" cases that tests/faults.mjs does not cover (docs/REQUIREMENTS.md):
// LiDAR loss, camera loss, comms loss, low battery, cargo shift and an unavailable port.
// Pass rules (one group per fault):
//   lidar   unmapped ground, so the rover must not drive blind: stopped (<= 0.1 m/s) within 8 s of the fault and for as
//           long as it has no LiDAR; FDIR isolates the head and switches to the spare; driving again within 90 s.
//   camera  the traverse does not need the camera: FDIR raises the alarm, the mission keeps running and covers ground.
//   comms   an operator command is queued and not applied; the mission keeps running; "clear comms" delivers the queue.
//   battery below the reserve at the start of a leg: the rover goes to the power dock first and takes charge.
//   shift   loaded: speed derate is announced, the cradle re-centres within 60 s, derate lifted, module still held.
//   port    loaded, PORT-1 unavailable: holds short of the port with the module still clamped and asks the operator;
//           after "clear port 1" it delivers.
//   lidar on a proven route   crawls at <= 0.35 m/s (not stopped) on the remembered route until the spare head is on.
//   battery runs down mid-drive (5 %, 1.8 km from the dock)   survival mode stops the rover, the keep-alive array
//           deploys once it has stopped and the pack gains charge; above 30 % the mission resumes by itself.
//   camera dead at the port   docking needs the ring camera: the rover must refuse the berthing, never touch the port,
//           keep the module clamped and report the reason.
//   seal fouled with regolith   the leak check fails: hatch stays closed, the seal is backed off, cleaned and re-seated,
//           the second check passes and only then does the hatch open. A damaged seal ("hard") holds for the operator.
import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { Sim } from '../src/sim/sim.js';
import { ROVER, WORLD } from '../src/sim/config.js';
import { runCommand } from '../src/sim/commands.js';
import { check, atMost, atLeast, equal } from './lib/check.mjs';
await RAPIER.init();

function newSim() {
  const sim = new Sim(RAPIER); sim.logs = [];
  sim.on('log', (e) => { sim.logs.push(e); if (!/Local planner blocked/.test(e.msg)) console.log(`[${e.t.toFixed(0).padStart(5)} s] ${e.level.padEnd(4)} ${e.msg}`); });
  return sim;
}
const speed = (R) => { const v = R.body.linvel(); return Math.hypot(v.x, v.z); };
const said = (sim, re, t0 = 0) => sim.logs.some((e) => e.t >= t0 && re.test(e.msg));
function run(sim, T, each) { const t0 = sim.time; while (sim.time - t0 < T) { sim.stepOnce(); if (each && each(sim.time - t0) === false) break; } }

// ---- 1-3: LiDAR, camera and comms loss on the unmapped pad -> habitat traverse ----
{
  const sim = newSim(), A = sim.autonomy, R = sim.rover, S = sim.sensors;
  runCommand(sim, {}, 'goto hab');
  run(sim, 400, () => !(sim.time > 45 && speed(R) > 1.5));   // past the slow start beside the lander
  const tL = sim.time; runCommand(sim, {}, 'fault lidar');
  let blindMax = 0, tStop = null, tBack = null;
  run(sim, 90, (t) => {
    const v = speed(R);
    if (S.faults.lidar) { if (t > 8) blindMax = Math.max(blindMax, v); if (tStop == null && v <= 0.1) tStop = t; }
    else if (tBack == null && v > 1) tBack = t;
    return tBack == null;
  });
  check('lidar: FDIR alarm names the LiDAR head', said(sim, /FDIR.*LiDAR/i, tL));
  atMost('lidar: time to stop after losing the LiDAR', tStop ?? Infinity, 8, ' s', 1);
  atMost('lidar: fastest speed while blind (after 8 s)', blindMax, 0.1, ' m/s', 2);
  check('lidar: switched to the spare head', S.lidarUnit === 'B' && !S.faults.lidar, `unit ${S.lidarUnit}, fault ${S.faults.lidar}`);
  atMost('lidar: driving again after', tBack ?? Infinity, 90, ' s', 0);
  check('lidar: mission still running', !!A.gen, A.phase);

  const tC = sim.time, d0 = R.odo.dist; runCommand(sim, {}, 'fault camera');
  run(sim, 60);
  check('camera: FDIR alarm names the camera', said(sim, /FDIR.*camera/i, tC));
  atLeast('camera: distance covered in the next 60 s', R.odo.dist - d0, 40, ' m', 0);
  check('camera: mission still running', !!A.gen, A.phase);

  const s0 = A.speedSetting, d1 = R.odo.dist; runCommand(sim, {}, 'fault comms'); runCommand(sim, {}, 'speed 1');
  run(sim, 20);
  equal('comms: operator command queued, not applied', A.speedSetting, s0);
  equal('comms: commands waiting', (A.queuedCommands || []).length, 1);
  atLeast('comms: distance covered while out of contact (20 s)', R.odo.dist - d1, 3, ' m', 0);
  runCommand(sim, {}, 'clear comms');
  equal('comms: queued command delivered when the link returns', A.speedSetting, 1);
  runCommand(sim, {}, 'fault comms'); runCommand(sim, {}, 'estop');
  check('comms: E-STOP still accepted during comms loss', R.estop === true, `estop ${R.estop}`);
}

// ---- 4: low battery at the start of a leg, beside the power dock ----
{
  const sim = newSim(), A = sim.autonomy, R = sim.rover, D = sim.powerDock, L = D.layout;
  R.teleport(L.entry.x + 8, L.entry.z + 8, L.yaw); for (let i = 0; i < 480; i++) sim.stepOnce(); R.brake = false;
  sim.slam.clear(); sim.slam.relocalize(R.pose()); sim.streamRocks([R.pose()]);
  runCommand(sim, {}, 'fault battery 12');
  const t0 = sim.time; runCommand(sim, {}, 'goto pad');
  let socMin = 1, tConn = null;
  run(sim, 1500, (t) => { socMin = Math.min(socMin, R.power.soc); if (tConn == null && D.connected && D.chargeW > 0) tConn = t; return !(tConn != null && t > tConn + 120); });
  check('battery: low-battery diversion announced', said(sim, /Low battery.*power dock/i, t0));
  check('battery: coupled to the power dock and charging', tConn != null, tConn != null ? `after ${tConn.toFixed(0)} s, ${D.chargeA.toFixed(2)} A` : 'never connected');
  atLeast('battery: lowest state of charge on the way', socMin * 100, 10, ' %', 1);
  atLeast('battery: state of charge 2 min into the charge', R.power.soc * 100, 12, ' %', 2);
}

// autonomous pickup of the 40-ft module from 22 m behind it (same setup as dock.mjs)
function loadedSim() {
  const sim = newSim(), A = sim.autonomy, R = sim.rover, c = sim.cargo[0];
  const ct = c.body.translation(), back = c.T.length / 2 + ROVER.frame.halfLen + 22 - (c.T.carryX || 0);
  R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw); for (let i = 0; i < 480; i++) sim.stepOnce();
  R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
  A.start('setup pickup', A.acquire(c)); const tp = sim.time;
  while (A.gen && sim.time - tp < 1500) sim.stepOnce();
  if (!check(`setup: ${c.id} picked up`, R.loaded, `cargo state ${c.state}`)) process.exit(1);
  return sim;
}
// rigidly move rover + module to 60 m in front of the module's port, facing it (same as dock.mjs)
function placeAtPort(sim) {
  const R = sim.rover, c = sim.cargo[0], port = sim.ports[c.dest - 1], yaw = Math.atan2(port.nz, -port.nx);
  const tx = port.x + port.nx * 60, tz = port.z + port.nz * 60;
  const p = R.pose(), dq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw).multiply(p.q.clone().invert());
  for (const b of [R.body, c.body]) {
    const t = b.translation(), r = b.rotation();
    const rel = new THREE.Vector3(t.x - p.x, t.y - p.y, t.z - p.z).applyQuaternion(dq);
    b.setTranslation({ x: tx + rel.x, y: sim.terrain.habH + 3.05 + rel.y, z: tz + rel.z }, true);
    b.setRotation(dq.clone().multiply(new THREE.Quaternion(r.x, r.y, r.z, r.w)), true);
    b.setLinvel({ x: 0, y: 0, z: 0 }, true); b.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }
  for (const bg of R.bogies) { bg.hpVelPrev = null; for (const w of bg.wheels) { w.groundY = null; w.anchor = null; } }
  sim.streamRocks([{ x: tx, z: tz }]);
  for (let i = 0; i < 1800; i++) { sim.stepOnce(); const v = R.body.linvel(), w = R.body.angvel(); if (i >= 240 && Math.abs(v.y) < 0.01 && Math.hypot(v.x, v.z) < 0.02 && Math.hypot(w.x, w.z) < 0.003) break; }
  sim.slam.clear(); sim.slam.relocalize(R.pose());
  return port;
}

// ---- 5-6: cargo shift and port unavailable, with the 40-ft module on board ----
{
  const sim = loadedSim(), A = sim.autonomy, R = sim.rover, c = sim.cargo[0];
  runCommand(sim, {}, 'goto hab');
  run(sim, 200, () => !(speed(R) > 0.8));
  const tS = sim.time; runCommand(sim, {}, 'fault shift');
  let tCentred = null;
  run(sim, 60, (t) => { if (tCentred == null && !R.lift.shifted) tCentred = t; return !(tCentred != null && t > tCentred + 3); });
  check('shift: speed derate announced', said(sim, /Cargo shift detected: speed derated/i, tS));
  atMost('shift: time to re-centre the module', tCentred ?? Infinity, 60, ' s', 1);
  check('shift: derate lifted after re-centring', said(sim, /Cargo re-centred: speed derate lifted/i, tS));
  check('shift: module still clamped', R.loaded && c.state === 'held', `state ${c.state}`);
  A.abort('test: next scenario'); run(sim, 15);

  const port = placeAtPort(sim);
  runCommand(sim, {}, 'fault port 1');
  const tP = sim.time; A.start('M5 with port fault', A.missionDeliver(c.dest));
  run(sim, 900, () => !/HOLD: receiving port unavailable/.test(A.phase));
  run(sim, 20);
  const q = R.pose(), dPort = Math.hypot(q.x - port.x, q.z - port.z);
  check('port: holding and asking the operator', /HOLD: receiving port unavailable/.test(A.phase) && said(sim, /UNAVAILABLE.*operator/i, tP), A.phase);
  atMost('port: speed while holding', speed(R), 0.05, ' m/s', 2);
  atLeast('port: distance kept from the unavailable port', dPort, 30, ' m', 1);
  check('port: module still clamped while holding', R.loaded && c.state === 'held' && !port.occupied, `state ${c.state}`);
  runCommand(sim, {}, 'clear port 1');
  run(sim, 2400, () => !!A.gen);
  const rec = A.history[A.history.length - 1];
  check('port: delivery completes after "clear port 1"', !!(rec && rec.ok) && ['delivered', 'docked'].includes(c.state) && !!port.occupied, `ok=${rec && rec.ok}, state ${c.state}`);
}

// ---- 7: LiDAR loss on a proven route (out 260 m, back, then repeat the leg) ----
{
  const sim = newSim(), A = sim.autonomy, R = sim.rover, S = sim.sensors;
  const p = R.pose(), H = WORLD.habitat, L = Math.hypot(H.x - p.x, H.z - p.z);
  const wp = `${Math.round(p.x + (H.x - p.x) / L * 260)} ${Math.round(p.z + (H.z - p.z) / L * 260)}`;
  for (const cmd of [`goto ${wp}`, 'goto pad']) { runCommand(sim, {}, cmd); run(sim, 900, () => !!A.gen); }
  runCommand(sim, {}, `goto ${wp}`);
  run(sim, 200, () => !(speed(R) > 1 && /^REPEAT/.test(A.pathSource)));
  check('lidar on route: following the remembered route', /^REPEAT/.test(A.pathSource), A.pathSource);
  runCommand(sim, {}, 'fault lidar');
  let vMax = 0, vMin = Infinity, tBack = null; const d0 = R.odo.dist;
  run(sim, 90, (t) => { const v = speed(R); if (S.faults.lidar) { if (t > 8) { vMax = Math.max(vMax, v); vMin = Math.min(vMin, v); } } else if (tBack == null && v > 1) tBack = t; return tBack == null; });
  atMost('lidar on route: fastest speed without LiDAR (after 8 s)', vMax, 0.35, ' m/s', 2);
  atLeast('lidar on route: still creeping along the route', vMin, 0.15, ' m/s', 2);
  check('lidar on route: spare head on and back to speed', S.lidarUnit === 'B' && tBack != null, `unit ${S.lidarUnit}, after ${tBack == null ? '-' : tBack.toFixed(0)} s`);
}

// ---- 8: battery runs down mid-drive, far from the power dock ----
{
  const sim = newSim(), A = sim.autonomy, R = sim.rover, Hh = sim.health;
  runCommand(sim, {}, 'goto hab'); run(sim, 150);
  runCommand(sim, {}, 'fault battery 5');
  let tStop = null, tDep = null;
  run(sim, 200, (t) => { if (tStop == null && speed(R) < 0.05) tStop = t; if (tDep == null && Hh.array.state === 'deployed') tDep = t; return tDep == null; });
  check('battery mid-drive: survival mode entered, mission paused (not aborted)', Hh.survival.on && !!A.gen && A.paused, `survival ${Hh.survival.on}, paused ${A.paused}`);
  atMost('battery mid-drive: time to stop', tStop ?? Infinity, 30, ' s', 1);
  atMost('battery mid-drive: keep-alive array deployed after', tDep ?? Infinity, 150, ' s', 0);
  const wh0 = R.power.wh; run(sim, 600);
  atLeast('battery mid-drive: array output', Hh.array.W, 2000, ' W', 0);
  atLeast('battery mid-drive: energy gained in 10 min parked', R.power.wh - wh0, 100, ' Wh', 0);
  R.power.wh = 0.32 * ROVER.battery.capacityWh * (R.power.capK ?? 1); R.power.soc = 0.32;   // skip the ~22 h recharge
  let tMove = null; run(sim, 300, (t) => { if (speed(R) > 0.5) tMove = t; return tMove == null; });
  check('battery mid-drive: mission resumes by itself above 30 %', !Hh.survival.on && !!A.gen && !A.paused && tMove != null, `moving after ${tMove == null ? '-' : tMove.toFixed(0)} s`);
  equal('battery mid-drive: array stowed before driving', Hh.array.state, 'stowed');
}

// ---- 9: camera dead at the port (hard failure: forced on every step) ----
{
  const sim = loadedSim(), A = sim.autonomy, R = sim.rover, c = sim.cargo[0];
  const port = placeAtPort(sim);
  A.start('M5 without camera', A.missionDeliver(c.dest));
  let dMin = Infinity, vTouch = 0;
  run(sim, 1500, () => { sim.sensors.faults.camera = true; const q = R.pose(); dMin = Math.min(dMin, Math.hypot(q.x - port.x, q.z - port.z)); return !!A.gen; });
  const rec = A.history[A.history.length - 1];
  check('camera at port: berthing refused with the reason', !A.gen && rec && rec.ok === false && said(sim, /docking (visual sensor|camera) lost/i), rec ? `ok=${rec.ok}` : 'no record');
  atMost('camera at port: peak contact force on the port', A._dockPeakN || 0, 100, ' N', 0);
  check('camera at port: module still clamped, port empty', R.loaded && c.state === 'held' && !port.occupied, `state ${c.state}`);
  run(sim, 10); atMost('camera at port: rover at rest afterwards', speed(R), 0.05, ' m/s', 2);
}

// ---- 10: port seal fouled with regolith (leak check fails) ----
{
  const sim = loadedSim(), A = sim.autonomy, R = sim.rover, c = sim.cargo[0];
  const port = placeAtPort(sim);
  runCommand(sim, {}, 'fault seal');
  let hatchT = null, failT = null; sim.on('hatch', () => { hatchT = sim.time; });
  A.start('M5 with a fouled seal', A.missionDeliver(c.dest));
  run(sim, 1500, () => { if (failT == null && said(sim, /Leak check FAIL/)) failT = sim.time; return !!A.gen; });
  const rec = A.history[A.history.length - 1];
  check('seal: leak detected, hatch kept closed', failT != null && (hatchT == null || hatchT > failT), `fail at ${failT?.toFixed(0)} s, hatch at ${hatchT?.toFixed(0)} s`);
  check('seal: cleaned and re-seated', said(sim, /repeating the dust-shield pulse and sweep/i));
  check('seal: second leak check passed before the hatch opened', A.lastBerth?.leakPassT != null && hatchT != null && A.lastBerth.leakPassT > failT && A.lastBerth.leakPassT < hatchT);
  check('seal: delivery completes', !!(rec && rec.ok) && c.state === 'delivered', `ok=${rec && rec.ok}, state ${c.state}`);
}
{
  const sim = loadedSim(), A = sim.autonomy, c = sim.cargo[0];
  const port = placeAtPort(sim);
  runCommand(sim, {}, 'fault seal hard');
  A.start('M5 with a damaged seal', A.missionDeliver(c.dest));
  run(sim, 1500, () => !/PAUSED: operator decision/.test(A.phase));
  run(sim, 30);
  check('seal (damaged): holds for the operator with the hatch closed', /PAUSED: operator decision/.test(A.phase) && !port.hatchOpen && !!A.gen, A.phase);
  check('seal (damaged): module stays latched to the port', !!c.dockJoint || c.state === 'docked', `state ${c.state}`);
  runCommand(sim, {}, 'clear seal'); runCommand(sim, {}, 'resume');
  run(sim, 1500, () => !!A.gen);
  check('seal (damaged): after service and "resume" the hatch opens and the delivery completes', port.hatchOpen && c.state === 'delivered', `hatch ${port.hatchOpen}, state ${c.state}`);
}
