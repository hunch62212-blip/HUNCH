// Operator command console. Every capability of the rover is reachable from here.
import { ROVER, WORLD, K, FAILURES } from './config.js';
import { PRIORITY } from './autonomy.js';
import { MODES } from './health.js';

export const HELP = [
  ['help [topic]', 'this list'],
  ['estop | reset | start', 'emergency stop; release latch; power-on self-test (all 3 needed to restart)'],
  ['status', 'one-line vehicle status'],
  ['mission <1-6|cycle> [A|B|C]', 'run a NASA mission (A = 40-ft module CM-40A, B = 20-ft container CC-20B, C = pod rack CP-36C)'],
  ['abort | resume', 'abort mission | resume after operator override'],
  ['goto <x> <z> | goto <pad|hab|staging|port1|port2|port3|A|B|C>', 'autonomous drive to a point / site'],
  ['speed <m/s>', 'autonomy cruise speed'],
  ['limit rpm <n> | limit speed <m/s> | limit torque <%>', 'hard limits (rpm = prototype motor rpm, 300 max)'],
  ['rpm auto|<n> | turbo on|off|auto', 'smart rpm (continuous 120..280, turbo 300) or lock the drive'],
  ['drive <vx> [wz] | stop', 'direct command: forward speed (m/s) and yaw rate (rad/s, + = left); skid steer has no sideways motion'],
  ['brake on|off', 'parking brake'],
  ['lift kneel|drive|carry|<m> | clamp open|close', 'ride height (kneel / carry) and ring jaws'],
  ['level on|off | ride <m>', 'active leveling, ride-height offset'],
  ['lights on|off', 'work lights'],
  ['timescale <x> | pause | ff', 'simulation speed (1-50x) / pause toggle / fast forward 50x on-off'],
  ['warp on|off', 'transit warp: at >= 50x, long autonomous cruise legs run kinematically (physics for everything else)'],
  ['newmap', 'reload with a new random crater / boulder layout'],
  ['lidar on|off | lidar channels <n> | lidar res <n>', 'LiDAR power / resolution'],
  ['camera on|off', 'camera power'],
  ['comms earth | comms local | comms delay <s>', 'operator link: Earth (1.3 s one-way delay) or habitat crew (no delay)'],
  ['fault block|lidar|camera|comms|battery <%>|wheel <1-6>|shift|port <1|2>', 'Mission 10 fault injection'],
  ['fault list | fault <id> [unit]', 'hardware failure catalogue (mc 3, seize 5, bat 2, fc A, imu B, ldust, louver L, ...)'],
  ['failures on|off [realistic|100x|1000x|demo]', 'RANDOM FAILURE switch (off by default; demo = one every few minutes)'],
  ['health | procedure <id> | repair <id>|all|abort', 'FDIR alarms + redundancy, procedure card, field / dock repair'],
  ['survival on|off|auto | array deploy|stow', 'survival mode (night / dead battery) and the keep-alive solar blanket'],
  ['thermal | brain on|off', 'thermal state, the thermal brain\'s decisions; brain off = thermostats only (docs/THERMAL_BRAIN.md)'],
  ['scenarios on|off|often|rare | scenario <id>|random|end', 'random thermal scenarios; ids: slope-heat psr-cold eclipse dust-fins climb glint heater-stuck mc-overload cold-start'],
  ['clear faults | clear comms | clear port <n>', 'remove faults (clearing comms runs the queued commands)'],
  ['reroute <A|B|C> <1|2|3>', 'send a module to the other receiving port (e.g. when its port is faulted)'],
  ['map clear|save|load|forget | routes', 'SLAM map & route memory (forget = erase the stored copy too)'],
  ['memory', 'rover memory store: where the map and routes are kept on disk, and which sites are remembered (?site=last reopens the newest)'],
  ['plan | priority <A|B|C> <category>', 'Mission 11 logistics plan / set request category'],
  ['reserve <%>', 'battery reserve: below it an empty rover goes to the power dock before its next leg'],
  ['sun <az> <el>', 'sun azimuth / elevation in degrees'],
  ['charge [%] | dock status', 'drive to the solar power dock and charge (default 95 %) / dock power report'],
  ['day | night | lunar on|off [minutes per lunar day]', 'sun up / sun down / run the lunar day-night cycle'],
  ['view chase|orbit|top|cam|free', 'camera view'],
  ['restricted on|off', 'enforce geofenced zones in the planner'],
  ['scout on|off|recall|status', 'Mission 9: SCOUT-1 tracked robot maps the route ahead of AEGIS-V1'],
];

const num = (s, d) => { const v = parseFloat(s); return Number.isFinite(v) ? v : d; };

export function runCommand(sim, ui, line, _uplinked = false) {
  const out = [];
  const say = (m, l = 'info') => out.push({ msg: m, level: l });
  const raw = line.trim();
  if (!raw) return out;
  const [cmd0, ...a] = raw.split(/\s+/);
  const cmd = cmd0.toLowerCase();
  const R = sim.rover, A = sim.autonomy;
  // signal delay (Earth operator): everything that acts on the rover arrives one light time late, E-STOP included.
  // Simulator and display commands are local to the console and run at once.
  if (sim.comms.delayS > 0 && !_uplinked && !['comms', 'clear', 'help', '?', 'view', 'timescale', 'ts', 'pause', 'ff', 'fastforward', 'warp', 'newmap', 'sun', 'day', 'night', 'fault', 'failures', 'random'].includes(cmd)) {
    sim.comms.queue.push({ t: sim.time + sim.comms.delayS, fn: () => runCommand(sim, ui, raw, true) });
    say(`uplink: "${raw}" reaches the rover in ${sim.comms.delayS.toFixed(1)} s`, 'info');
    for (const o of out) sim.log(o.msg, o.level);
    return out;
  }
  // comms loss: only the hard-wired E-stop link, fault clearing and local UI commands get through; the rest is
  // queued at the ground station and sent, in order, when the link is restored ("clear faults" / "clear comms")
  if (A.faults.comms && !['estop', 'e-stop', 'clear', 'help', '?', 'status', 'view', 'timescale', 'ts', 'pause', 'ff', 'fastforward', 'warp', 'newmap', 'health', 'fdir', 'procedure', 'proc', 'repair', 'failures', 'random', 'fault'].includes(cmd)) { // repair = local crew / service robot
    (A.queuedCommands = A.queuedCommands || []).push(raw);
    say(`COMMS LOSS: "${raw}" queued (${A.queuedCommands.length} waiting; rover continuing autonomously). Use "clear comms" to restore the link.`, 'err');
    for (const o of out) sim.log(o.msg, o.level);
    return out;
  }
  const cargoBy = (k) => { k = (k || 'A').toUpperCase(); return sim.cargo.find((c) => c.id.endsWith(k)) || sim.cargo.find((c) => c.id.toUpperCase() === k) || sim.cargo[0]; }; // A / B / C = last letter of the id
  const site = (name) => {
    const n = (name || '').toLowerCase();
    const P = sim.terrain.pad, H = sim.terrain.hab;
    if (n === 'pad') return { x: WORLD.pad.x - P.dir.x * 45, z: WORLD.pad.z - P.dir.z * 45, name: 'landing pad' };
    if (n === 'dock') { const L = sim.powerDock.layout; return { ...L.entry, name: 'power dock' }; }
    if (n === 'staging') return { ...P.staging, name: 'staging area' };
    if (n === 'hab') return { x: WORLD.habitat.x + H.nx * 45, z: WORLD.habitat.z + H.nz * 45, name: 'habitat apron' };
    if (/^port[1-9]$/.test(n) && sim.ports[+n[4] - 1]) { const p = sim.ports[+n[4] - 1]; return { x: p.x + H.nx * 35, z: p.z + H.nz * 35, name: p.name + ' approach' }; }
    if (n.length === 1 && sim.cargo.some((c) => c.id.toLowerCase().endsWith(n))) { const c = cargoBy(n); const T = c.T; return { x: c.initial.x - Math.cos(c.yaw) * (T.length / 2 + 16), z: c.initial.z + Math.sin(c.yaw) * (T.length / 2 + 16), name: c.id + ' pre-grasp' }; }
    return null;
  };
  switch (cmd) {
    case 'help': case '?':
      for (const [c, d] of HELP) say(`${c.padEnd(52)} ${d}`);
      say('Keys: W/S drive, A/D turn, SPACE brake, R/F lift, G clamp, ESC = E-STOP, C camera, V lidar, H help');
      break;
    case 'estop': case 'e-stop': sim.emergencyStop('console'); break;
    case 'reset': sim.resetEstop(); break;
    case 'start': sim.startUp(); break;
    case 'status': {
      const t = R.telemetry();
      say(`${R.estop ? 'E-STOP' : R.powered ? 'ARMED' : 'DISARMED'} | ${A.mode}${A.missionName ? ' ' + A.missionName : ''} | v ${t.speed.toFixed(2)} m/s | SoC ${(R.power.soc * 100).toFixed(1)}% | tip margin ${t.margin.toFixed(1)} deg | load ${R.loaded ? R.lift.cargo.id : 'none'} | rpm ${R.rpm.proto.toFixed(0)} (${R.rpm.tier}: ${R.rpm.why}; ceiling ${R.limits.rpmCeil}) speed limit ${R.limits.speed >= 99 ? 'none' : R.limits.speed + ' m/s'} torque ${R.limits.torquePct}%`);
      break;
    }
    case 'mission': {
      const m = (a[0] || '').toLowerCase();
      const c = cargoBy(a[1]);
      const port = c.dest;
      const map = {
        1: ['Mission 1: Leave the landing zone', () => A.missionLeaveLanding()],
        2: [`Mission 2: Find & approach ${c.id}`, () => A.missionApproach(c)],
        3: [`Mission 3: Pick up & secure ${c.id}`, () => A.missionPickup(c)],
        4: [`Mission 4: Transport to habitat`, () => A.missionTransport(R.loaded ? R.lift.cargo.dest : port)],
        5: [`Mission 5: Deliver & place`, () => A.missionDeliver(R.loaded ? R.lift.cargo.dest : port)],
        6: ['Mission 6: Full logistics cycle', () => A.missionCycle()],
        cycle: ['Mission 6: Full logistics cycle', () => A.missionCycle()],
      };
      const e = map[m];
      if (!e) { say('usage: mission <1-6|cycle> [A|B|C]', 'warn'); break; }
      if (A.gen) A.abort('new mission');
      A.start(e[0], e[1]());
      break;
    }
    case 'abort': A.abort(); break;
    case 'resume': A.resume(); break;
    case 'goto': {
      let g = null;
      if (a.length >= 2 && Number.isFinite(parseFloat(a[0]))) g = { x: num(a[0]), z: num(a[1]), name: `(${a[0]}, ${a[1]})` };
      else g = site(a[0]);
      if (!g) { say('usage: goto <x> <z> | goto pad|hab|staging|port1|port2|port3|A|B|C', 'warn'); break; }
      if (A.gen) A.abort('new goto');
      A.start(`Goto ${g.name}`, A.missionGoto(g.x, g.z, g.name));
      break;
    }
    case 'speed': { if ((a[0] || '') === 'auto') { A.speedSetting = 99; say('Autonomy cruise speed: AUTO (smart rpm decides)', 'ok'); } else { A.speedSetting = Math.max(0.1, Math.min(9, num(a[0], 1.2))); say(`Autonomy cruise speed ceiling ${A.speedSetting.toFixed(2)} m/s`, 'ok'); } break; }
    case 'rpm': case 'turbo': {
      const c = cmd;
      // rpm are PROTOTYPE motor rpm: 300 no-load units capped at 280; wheel rpm = proto / sqrt(K)
      const k = (a[0] || '').toLowerCase();
      if (c === 'turbo') {
        if (k === 'off') { R.rpm.mode = 'fixed'; R.rpm.fixed = ROVER.motor.protoCapRpm; say(`Turbo disabled: locked at ${ROVER.motor.protoCapRpm} rpm`, 'ok'); }
        else if (k === 'on') { R.rpm.mode = 'fixed'; R.rpm.fixed = 300; say('TURBO locked at 300 rpm (terrain governor bypassed; tip governor still active)', 'warn'); }
        else { R.rpm.mode = 'auto'; say('Smart rpm AUTO: continuous 120..280 rpm set by slope, slip, roughness, tip margin, turn and temperature (turbo 300 on clear straights)', 'ok'); }
      } else if (k === 'auto' || k === '') { R.rpm.mode = 'auto'; say('Smart rpm AUTO: continuous 120..280 rpm set by slope, slip, roughness, tip margin, turn and temperature (turbo 300 on clear straights)', 'ok'); }
      else { const v = num(a[0]); if (v == null) say('usage: rpm auto | rpm <30..300>', 'warn'); else { R.rpm.mode = 'fixed'; R.rpm.fixed = Math.max(30, Math.min(300, v)); say(`Drive locked at ${R.rpm.fixed} rpm (= ${(R.rpm.fixed / Math.sqrt(K) * 2 * Math.PI / 60 * ROVER.wheel.radius).toFixed(2)} m/s)`, 'ok'); } }
      break;
    }
    case 'limit': {
      const k = (a[0] || '').toLowerCase(), v = num(a[1]);
      if (k === 'rpm' && v != null) { R.limits.rpmCeil = Math.max(20, Math.min(300, v)); say(`Motor rpm ceiling ${R.limits.rpmCeil} (prototype rpm; 300 = no-load, 280 = normal cap) = ${(R.limits.rpmCeil / Math.sqrt(K) * 2 * Math.PI / 60 * ROVER.wheel.radius).toFixed(2)} m/s`, 'ok'); }
      else if (k === 'speed' && v != null) { R.limits.speed = Math.max(0.05, Math.min(99, v)); say(`Ground speed limit ${R.limits.speed} m/s`, 'ok'); }
      else if (k === 'torque' && v != null) { R.limits.torquePct = Math.max(5, Math.min(100, v)); say(`Motor torque limit ${R.limits.torquePct}%`, 'ok'); }
      else say('usage: limit rpm <n> | limit speed <m/s> | limit torque <%>', 'warn');
      break;
    }
    case 'drive': {
      if (A.mode === 'AUTO') A.override();
      R.brake = false; ui.manualHold = true;
      R.cmd.vx = num(a[0], 0); R.cmd.wz = num(a[1], 0); R.cmd.vz = 0; // no crab on a skid-steer vehicle (a third number used to be accepted and silently ignored)
      say(`Direct command: forward ${R.cmd.vx} m/s, yaw rate ${R.cmd.wz} rad/s (type "stop" to end)`, 'ok');
      break;
    }
    case 'stop': R.cmd.vx = R.cmd.vz = R.cmd.wz = 0; ui.manualHold = false; if (A.mode === 'AUTO') A.override(); say('Stopped', 'ok'); break;
    case 'steer': say('AEGIS is skid-steer (fixed wheels): steering is a yaw rate; no Ackermann or crab modes', 'info'); break;
    case 'brake': R.brake = (a[0] || 'on').toLowerCase() !== 'off'; say(`Parking brake ${R.brake ? 'SET' : 'released'}`, 'ok'); break;
    case 'lift': {
      const k = (a[0] || '').toLowerCase();
      if (k === 'up' || k === 'carry') R.lift.target = ROVER.ride.carry; else if (k === 'down' || k === 'kneel') R.lift.target = ROVER.ride.kneel; else if (k === 'drive') R.lift.target = 0; else R.lift.target = num(a[0], R.lift.target);
      say(`Ride height target ${R.lift.target.toFixed(2)} m (kneel ${ROVER.ride.kneel.toFixed(2)} .. carry ${ROVER.ride.carry.toFixed(2)})`, 'ok'); break;
    }
    case 'clamp': case 'grapple': {
      const k = (a[0] || '').toLowerCase();
      const r = k === 'open' || k === 'release' ? R.clampOpen() : R.clampClose();
      say(r.ok ? R.lift.msg : `Refused: ${r.why}`, r.ok ? 'ok' : 'warn'); break;
    }
    case 'level': R.leveling = (a[0] || 'on').toLowerCase() !== 'off'; if (!R.leveling) for (const b of R.bogies) b.level = 0; say(`Active leveling ${R.leveling ? 'ON' : 'OFF'}`, 'ok'); break;
    // (was: wrote R.rideOffset directly, which the lift controller pulled straight back to lift.target next step)
    case 'ride': R.lift.target = Math.max(ROVER.ride.kneel, Math.min(ROVER.ride.max, num(a[0], 0))); say(`Ride height set point ${R.lift.target.toFixed(2)} m`, 'ok'); break;
    case 'lights': R.lights = (a[0] || 'on').toLowerCase() !== 'off'; say(`Lights ${R.lights ? 'ON' : 'OFF'}`, 'ok'); break;
    case 'timescale': case 'ts': sim.timeScale = Math.max(0.1, Math.min(50, num(a[0], 1))); say(`Time scale ${sim.timeScale}x`, 'ok'); break;
    case 'pause': sim.paused = !sim.paused; say(sim.paused ? 'Simulation paused' : 'Simulation running', 'ok'); break;
    case 'ff': case 'fastforward': sim.paused = false; sim.timeScale = sim.timeScale >= 50 ? 1 : 50; say(sim.timeScale >= 50 ? 'FAST FORWARD 50x (sim time; long drives use transit warp)' : 'Time scale 1x', 'ok'); break;
    case 'warp': {
      const k = (a[0] || '').toLowerCase();
      if (k === 'on' || k === 'off') sim.warp.enabled = k === 'on';
      say(`Transit warp ${sim.warp.enabled ? 'ON' : 'OFF'} (used at >= ${sim.warp.minScale}x on long autonomous cruise legs; ${sim.warp.steps} warp steps, ${(sim.warp.simS / 60).toFixed(1)} sim min so far)`, 'ok');
      break;
    }
    case 'newmap': case 'newterrain':
      if (typeof location === 'undefined') { say('newmap: browser only', 'warn'); break; }
      say('Loading a new random terrain…', 'ok'); setTimeout(() => { location.href = location.pathname; }, 50); break;
    case 'lidar': {
      const k = (a[0] || '').toLowerCase(), S = sim.sensors;
      if (k === 'on' || k === 'off') { S.lidarOn = k === 'on'; say(`LiDAR ${k.toUpperCase()}`, 'ok'); }
      else if (k === 'channels') { S.setLidar(Math.max(4, Math.min(64, num(a[1], 16))), null); say(`LiDAR channels ${S.lidarCfg.channels}`, 'ok'); }
      else if (k === 'res') { S.setLidar(null, Math.max(60, Math.min(720, num(a[1], 180)))); say(`LiDAR horizontal samples ${S.lidarCfg.hRes}`, 'ok'); }
      else if (k === 'auto') { S.lidarAuto = a[1] !== 'off'; say(`LiDAR adaptive spin ${S.lidarAuto ? 'ON (5-12 Hz by speed)' : 'OFF'}`, 'ok'); }
      else say(`LiDAR ${S.lidarOn ? 'ON' : 'OFF'} ${S.lidarCfg.channels}ch x ${S.lidarCfg.hRes} @ ${S.lidarCfg.rateHz} Hz, range ${S.lidarCfg.range} m`);
      break;
    }
    case 'camera': sim.sensors.cameraOn = (a[0] || 'on').toLowerCase() !== 'off'; say(`Camera ${sim.sensors.cameraOn ? 'ON' : 'OFF'}`, 'ok'); break;
    case 'fault': {
      const k = (a[0] || '').toLowerCase();
      if (k === 'list' || !k) {
        say('Fault catalogue (fault <id> [unit]): legacy block | lidar | camera | comms | battery <%> | wheel <1-6> | shift | port <1|2|3> | seal [hard]', 'info');
        for (const m of MODES) say(`  ${m.id.padEnd(8)} ${m.inst[0] == null ? '' : `[${m.inst.join('|')}] `}${m.name} (${m.sev}, ${m.fidelity})`, 'info');
        break;
      }
      sim.injectFault(k, a[1] != null ? (Number.isFinite(+a[1]) ? +a[1] : a[1]) : undefined); break;
    }
    case 'failures': case 'random': {
      const H = sim.health, k = (a[0] || '').toLowerCase();
      if (k === 'on' || k === 'off') H.setRandom(k === 'on', a[1]);
      else if (k) H.setRandom(true, k);
      else say(`Random failures ${H.random.on ? 'ON' : 'OFF'} at ${H.random.preset} (${H.random.rateX}x MTBF rate). Usage: failures on|off [realistic|100x|1000x|demo|<n>]`, 'info');
      break;
    }
    case 'thermal': {
      const T = sim.thermal, b = T.brain;
      say(`THERMAL: ${T.summary()}`, 'info');
      for (const [k, n] of [['drive', 'drive'], ['rad', 'radiators'], ['lidar', 'LiDAR'], ['camera', 'camera']]) if (b.reason[k] && !/nominal|auto/.test(b.choice[k].id)) say(`  ${n}: ${b.choice[k].label} - ${b.reason[k]}`, 'warn');
      break;
    }
    case 'brain': {
      const v = (a[0] || '').toLowerCase();
      if (v === 'on' || v === 'off') sim.thermal.brain.on = v === 'on';
      say(`Thermal brain ${sim.thermal.brain.on ? 'ON' : 'OFF (thermostats only)'}`, sim.thermal.brain.on ? 'ok' : 'warn');
      break;
    }
    case 'scenarios': {
      const v = (a[0] || '').toLowerCase(), T = sim.thermal;
      if (v === 'on' || v === 'off') T.setScenarios(v === 'on');
      else if (v === 'often') T.setScenarios(true, 120);
      else if (v === 'rare') T.setScenarios(true, 600);
      else say(`Thermal scenarios ${T.scen.on ? 'ON' : 'OFF'}, mean gap ${Math.round(T.scen.meanGapS / 60)} min${T.scen.active ? `, active: ${T.scen.active.name}` : ''}`, 'info');
      break;
    }
    case 'scenario': {
      const v = (a[0] || 'random').toLowerCase(), T = sim.thermal;
      if (v === 'end') { if (T.scen.active) T._end(); else say('No thermal scenario active', 'info'); }
      else if (!T.trigger(v)) say(`Unknown or unavailable scenario "${v}" (help lists them)`, 'warn');
      break;
    }
    case 'health': case 'fdir': {
      const H = sim.health, st = H.stats;
      say(`FDIR: ${H.alarms.size} alarm(s), ${H.active.size} failure(s) in the vehicle. Injected ${st.injected} (${st.random} random), detected ${st.detected}${st.detected ? `, mean latency ${(st.latSum / st.detected).toFixed(1)} s` : ''}, spurious ${st.spurious}, repaired ${st.repaired}`, 'info');
      say(`Redundancy: FC-${H.redund.fc} | IMU-${H.redund.imu} | radio-${H.redund.com} | heaters ${H.redund.htr} | strings ${4 - H.outStrings.size}/4 | vault ${H.th.Tv.toFixed(1)} C battery ${H.th.Tb.toFixed(1)} C heaters ${H.heaterW.toFixed(0)} W (hubs ${Math.min(...H.th.hub).toFixed(0)} C, ${H.th.hubW.toFixed(0)} W)`, 'info');
      for (const s of H.summary()) say('  ' + s, 'warn');
      if (H.alarms.size) say('"procedure <id>" for the procedure card, "repair <id>" or "repair all" to fix', 'info');
      break;
    }
    case 'procedure': case 'proc': {
      const g = (a[0] || '').toLowerCase(); const p = sim.health.procedure(g.split(/[:\s-]/)[0]);
      if (!p) { say(`usage: procedure <${MODES.map((m) => m.id).join('|')}>`, 'warn'); break; }
      for (const l of p.lines) say(l, 'info');
      break;
    }
    case 'repair': sim.health.startRepair(a.join(' ')); break;
    case 'survival': {
      const k = (a[0] || '').toLowerCase(), H = sim.health;
      if (k === 'auto') { H.survival.auto = a[1] !== 'off'; say(`Automatic survival mode ${H.survival.auto ? 'ON' : 'OFF'} (enter < ${(FAILURES.survivalEnterSoc * 100).toFixed(0)} % away from the dock)`, 'ok'); }
      else H.setSurvival(k !== 'off', 'operator');
      break;
    }
    case 'array': sim.health.deployArray((a[0] || 'deploy').toLowerCase() !== 'stow'); break;
    case 'clear': {
      const k = (a[0] || '').toLowerCase();
      if (k === 'port') { delete A.faults.port[num(a[1], 1)]; say(`PORT-${num(a[1], 1)} available`, 'ok'); }
      else if (k === 'comms') { A.faults.comms = false; say('Comms link restored', 'ok'); }
      else if (k === 'seal') { A.faults.seal = false; say('Port seal serviced', 'ok'); }
      else if (k === 'faults' || k === 'all' || !k) sim.clearFaults();
      else { say('usage: clear faults | clear comms | clear port <n> | clear seal', 'warn'); break; } // (anything unrecognised used to clear every fault)
      // link back: send what the operator queued during the outage, in order
      if (!A.faults.comms && A.queuedCommands && A.queuedCommands.length) {
        const q = A.queuedCommands; A.queuedCommands = [];
        say(`Sending ${q.length} queued command${q.length > 1 ? 's' : ''}`, 'ok');
        for (const o of out) sim.log(o.msg, o.level);
        out.length = 0;
        for (const line of q) out.push(...runCommand(sim, ui, line));
        return out;
      }
      break;
    }
    case 'reroute': {
      const c = cargoBy(a[0]), id = Math.round(num(a[1], NaN));
      if (!a[0] || !sim.ports.some((p) => p.id === id)) { say(`usage: reroute <A|B|C> <${sim.ports.map((p) => p.id).join('|')}>`, 'warn'); break; }
      if (c.state === 'delivered') { say(`${c.id} is already delivered`, 'warn'); break; }
      const other = sim.cargo.find((x) => x !== c && x.dest === id && x.state !== 'delivered');
      if (other) say(`Note: ${other.id} is also bound for PORT-${id}`, 'warn');
      c.dest = id; const m = A.manifest.find((x) => x.cargo === c); if (m) m.port = id;
      say(`${c.id} rerouted to PORT-${id}`, 'ok');
      break;
    }
    case 'memory': { ui.memoryStatus?.(); break; }
    case 'map': {
      const k = (a[0] || '').toLowerCase();
      if (k === 'clear') { sim.slam.clear(); say('SLAM map and route memory cleared', 'ok'); }
      else if (k === 'save') ui.saveMap?.();
      else if (k === 'load') ui.loadMap?.();
      else if (k === 'forget') ui.forgetMap?.();
      else say(`SLAM map: ${sim.slam.tiles.size} tiles, ${sim.slam.cellsKnown} cells (${(sim.slam.cellsKnown * 0.25 / 1e4).toFixed(2)} ha)`);
      break;
    }
    case 'routes':
      if (!sim.slam.routes.length) say('No routes stored yet (complete a leg between sites to teach one)');
      for (const r of sim.slam.routes) say(`${r.id}: ${r.from} -> ${r.to}  ${r.length.toFixed(0)} m  ${(r.time / 60).toFixed(1)} min  ${r.energyWh.toFixed(0)} Wh  used ${r.uses}x`);
      break;
    case 'plan': {
      const p = A.logisticsPlan();
      if (!p.length) say('No cargo requests waiting');
      p.forEach((x, i) => say(`${i + 1}. ${x.id} [${x.category}, P${PRIORITY[x.category] ?? x.priority}] -> PORT-${x.port}  ETA ${x.eta.toFixed(0)} min  energy ${(x.energy / 1000).toFixed(1)} kWh  ${x.feasible ? 'feasible' : 'NEEDS RECHARGE'}`));
      break;
    }
    case 'priority': {
      const c = cargoBy(a[0]); const m = A.manifest.find((x) => x.cargo === c); const cat = (a[1] || '').toLowerCase();
      if (!m || !(cat in PRIORITY)) { say(`usage: priority <A|B|C> <${Object.keys(PRIORITY).join('|')}>`, 'warn'); break; }
      m.category = cat; say(`${m.id} category -> ${cat}`, 'ok'); break;
    }
    case 'reserve': A.reserveSoc = Math.max(0.05, Math.min(0.8, num(a[0], 20) / 100)); say(`Battery reserve ${(A.reserveSoc * 100).toFixed(0)}%`, 'ok'); break;
    case 'charge': {
      const tgt = Math.max(0.3, Math.min(1, num(a[0], 95) / 100));
      if (A.gen) A.abort('charge');
      A.start(`Charge at power dock to ${(tgt * 100).toFixed(0)}%`, A.missionCharge(tgt));
      break;
    }
    case 'dock': {
      const D = sim.powerDock;
      say(`POWER DOCK ${D.status} | bank ${(D.soc * 100).toFixed(1)}% (${(D.bankWh / 1e6).toFixed(2)} MWh) | solar ${(D.solarW / 1000).toFixed(1)} kW | dock load ${(D.loadW / 1000).toFixed(1)} kW | to rover ${(D.chargeW / 1000).toFixed(0)} kW | night endurance ${D.nightHours().toFixed(0)} h (lunar night 354 h)`);
      break;
    }
    case 'day': sim.lunar.on = false; sim.setSun(sim.sunAz, 9); say('Sun up (el 9 deg)', 'ok'); break;
    case 'night': sim.lunar.on = false; sim.setSun(sim.sunAz, -6); say('Lunar night: sun below the horizon, dock bank supplying', 'ok'); break;
    case 'lunar': {
      const on = (a[0] || 'on').toLowerCase() !== 'off';
      sim.lunar.on = on; if (a[1]) sim.lunar.periodS = Math.max(60, num(a[1], 15) * 60);
      say(on ? `Lunar day/night cycle ON: one lunar day = ${(sim.lunar.periodS / 60).toFixed(0)} sim min` : 'Lunar cycle OFF', 'ok');
      break;
    }
    case 'comms': {
      const C = sim.comms, k = (a[0] || '').toLowerCase();
      if (k === 'earth') { C.mode = 'earth'; C.delayS = C.earthDelayS; }
      else if (k === 'local') { C.mode = 'local'; C.delayS = 0; }
      else if (k === 'delay') { C.delayS = Math.max(0, num(a[1], 0)); C.mode = C.delayS > 0 ? 'remote' : 'local'; }
      say(`Operator link: ${C.mode}, one-way delay ${C.delayS.toFixed(1)} s${C.delayS > 0 ? ` (commands and E-STOP arrive late; manual drive capped at ${C.earthDriveCap} m/s)` : ' (crew in the habitat)'}`, 'ok');
      break;
    }
    case 'sun': sim.setSun(num(a[0], sim.sunAz), num(a[1], sim.sunEl)); say(`Sun az ${sim.sunAz} el ${sim.sunEl}`, 'ok'); break;
    case 'view': ui.setView?.((a[0] || 'chase').toLowerCase()); break;
    case 'scout': {
      const k = (a[0] || 'status').toLowerCase();
      if (k === 'status') say(`SCOUT-1: ${sim.scout.enabled ? sim.scout.mode : 'parked'} | ${sim.scout.msg} | battery ${(sim.scout.soc * 100).toFixed(0)}% | odometer ${sim.scout.odo.toFixed(0)} m`);
      else say('SCOUT-1: ' + sim.scout.command(k), 'ok');
      break;
    }
    case 'restricted': A.planner.useRestricted = (a[0] || 'on').toLowerCase() !== 'off'; say(`Geofenced zones ${A.planner.useRestricted ? 'enforced' : 'ignored'}`, 'ok'); break;
    case 'teleport': { const x = num(a[0]), z = num(a[1]), y = num(a[2], 0) * Math.PI / 180; if (x == null || z == null) { say('usage: teleport <x> <z> [yawDeg]', 'warn'); break; } R.teleport(x, z, y); sim.slam.relocalize({ ...R.pose(), x, z, yaw: y }); say('Teleported (SLAM relocalised)', 'warn'); break; }
    default: say(`Unknown command "${cmd}". Type "help".`, 'warn');
  }
  for (const o of out) sim.log(o.msg, o.level);
  return out;
}
