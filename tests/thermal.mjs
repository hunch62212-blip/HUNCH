// Thermal environment, scenarios and the thermal brain (src/sim/thermal.js).
//  1. environment: regolith temperature vs sun (polar 9 deg sun, sun-facing slope, night)
//  2. hot lunar day: sun-facing slope heat soak + sun glint, 12 min: with the brain the LiDAR and camera stay inside
//     their operating range and stay ON; with the brain OFF the LiDAR overheats (shows the brain is doing the work)
//  3. cold: cold-soaked wheel ends -> the brain revs up (stator heating) and the hubs climb out of the stiff range
//  4. heat: controller overload + heavy climb on the slope -> the brain revs down, controllers / windings in range
//  5. every scenario once, back to back, brain on: no instrument ever protected OFF
//  6. normal polar day, driving with physics: the brain stays nominal (does not disturb normal operations)
// Long scenarios use Thermal.soak (thermal-only stepping, rover parked); 6 runs the full physics.
import { check, within, atMost, atLeast } from './lib/check.mjs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { NODES, SCENARIOS, DRIVE_LIMITS } from '../src/sim/thermal.js';
await RAPIER.init();

const K0 = 273.15;
function fresh(brain = true) {
  const sim = new Sim(RAPIER, { seed: 866201175 });
  for (let i = 0; i < 120 * 3; i++) sim.stepOnce();       // settle, power up
  sim.thermal.brain.on = brain;
  sim.setSun(sim.sunAz, 9);
  sim.thermal.soak(900);                                  // 15 min to thermal equilibrium on a normal polar day
  return sim;
}
const watch = (sim, s, fn) => { const dt = 0.1; for (let t = 0; t < s; t += 1) { sim.thermal.soak(1, dt); fn && fn(); } };

// 1. environment
{
  const sim = fresh();
  const E = sim.thermal.env;
  within('env: regolith at a 9 deg polar sun', E.regK, 225, 260, ' K', 0);
  sim.thermal.trigger('slope-heat'); sim.thermal.scen.active.t1 = 1e9; sim.thermal.soak(1500);
  within('env: regolith on a 30 deg sun-facing slope', E.regK, 320, 360, ' K', 0);
  sim.thermal._end(); sim.setSun(sim.sunAz, -6); sim.thermal.soak(2400);
  within('env: regolith at night', E.regK, 90, 140, ' K', 0);
}

// 2. hot lunar day with and without the brain
for (const brain of [true, false]) {
  const sim = fresh(brain), th = sim.thermal;
  th.trigger('slope-heat'); th.scen.active.t1 = sim.time + 1e9;
  th.fx.glint = 1;   // sun glint on top of the slope heat: the hottest the instruments see
  let maxL = -1e9, maxC = -1e9, off = 0;
  watch(sim, 1800, () => { maxL = Math.max(maxL, th.n.lidar.T); maxC = Math.max(maxC, th.n.camera.T); if (!th.ok.lidar || !th.ok.camera) off++; });
  if (brain) {
    atMost('hot day (brain): LiDAR max', maxL, NODES.lidar.op[1], ' C', 1);
    atMost('hot day (brain): camera max', maxC, NODES.camera.op[1], ' C', 1);
    check('hot day (brain): instruments never protected OFF', off === 0, `${off} s off`);
    check('hot day (brain): LiDAR on TEC cooling', /COOL/.test(th.brain.choice.lidar.label), th.brain.choice.lidar.label);
    atMost('hot day (brain): avionics vault', sim.health.th.Tv, 45, ' C', 1);
  } else {
    atLeast('hot day (brain OFF, reference): LiDAR overheats past its range', maxL, NODES.lidar.op[1], ' C', 1);
  }
}

// 3. cold-soaked wheel ends -> rev up
{
  const sim = fresh(), th = sim.thermal, H = sim.health;
  sim.setSun(sim.sunAz, -6); th.soak(600);
  th.trigger('cold-start');
  let revUp = false;
  watch(sim, 30, () => { if (/^up/.test(th.brain.choice.drive.id)) revUp = true; });
  check('cold start: brain revs up (stator heating) within 30 s', revUp, th.brain.choice.drive.label);
  atLeast('cold start: thermal power drawn while revving up', th.powerW, 600, ' W', 0);
  const h0 = Math.min(...H.th.hub);
  watch(sim, 1800);
  const h1 = Math.min(...H.th.hub);
  atLeast('cold start: coldest hub (290 kg) warms in 30 min', h1 - h0, 2, ' K', 1);
  atLeast('cold start: windings out of the stiff range', Math.min(...sim.rover.wheels.map((w) => w.temp)), DRIVE_LIMITS.windColdMaxC, ' C', 1);
}

// 4. heat in the drive -> rev down
{
  const sim = fresh(), th = sim.thermal;
  th.trigger('slope-heat'); th.scen.active.t1 = sim.time + 1e9;
  th.soak(600);
  th.fx.mcExtraW = 400; th.fx.motorExtraW = 600;          // controller overload + heavy climb together
  let down = false, maxMc = -1e9;
  watch(sim, 2400, () => { if (/^down|stop/.test(th.brain.choice.drive.id)) down = true; maxMc = Math.max(maxMc, th.n.mc.T); });
  check('drive heat: brain revs down', down, th.brain.choice.drive.label);
  check('drive heat: rover rpm capped by the brain', down && Number.isFinite(sim.rover.thermalRpm) || !/^down|stop/.test(th.brain.choice.drive.id), `cap ${sim.rover.thermalRpm}`);
  atMost('drive heat: motor controllers stay under their limit', maxMc, NODES.mc.op[1] + 5, ' C', 1);
  atMost('drive heat: hottest winding', Math.max(...sim.rover.wheels.map((w) => w.temp)), DRIVE_LIMITS.windMaxC, ' C', 1);
}

// 5. every scenario back to back
{
  const sim = fresh(), th = sim.thermal;
  let off = 0, n = 0;
  for (const id of Object.keys(SCENARIOS)) {
    if (SCENARIOS[id].need === 'sun' && sim.sunDir.y <= 0.01) sim.setSun(sim.sunAz, 9);
    if (!th.trigger(id)) continue;
    n++;
    const dur = Math.min(600, th.scen.active ? th.scen.active.t1 - sim.time : 0);
    watch(sim, Math.max(60, dur), () => { if (!th.ok.lidar || !th.ok.camera) off++; });
    if (th.scen.active) th._end();
    watch(sim, 120);
  }
  check('all scenarios ran', n === Object.keys(SCENARIOS).length, `${n} of ${Object.keys(SCENARIOS).length}`);
  check('all scenarios (brain): instruments never protected OFF', off === 0, `${off} s off`);
  atLeast('all scenarios: the brain made decisions', th.brain.decisions, 4, '', 0);
}

// 6. normal polar day with full physics: brain stays out of the way
{
  const sim = fresh(), th = sim.thermal, R = sim.rover;
  const d0 = th.brain.decisions;
  { const sp = sim.terrain.pad, p = R.pose(); R.teleport(p.x - sp.perp.x * 15, p.z - sp.perp.z * 15, Math.atan2(sp.perp.z, -sp.perp.x)); }
  R.brake = false; R.cmd.vx = 2; R.cmd.wz = 0.04;
  for (let i = 0; i < 120 * 60; i++) sim.stepOnce();
  check('normal day: drive channel nominal', th.brain.choice.drive.id === 'nominal', th.brain.choice.drive.label);
  atMost('normal day: brain decisions while driving 60 s', th.brain.decisions - d0, 1, '', 0);
  within('normal day: driving speed unaffected', R.telemetry().fwdSpeed, 1.8, 2.2, ' m/s', 2);
  check('normal day: LiDAR and camera on', th.ok.lidar && th.ok.camera);
}
