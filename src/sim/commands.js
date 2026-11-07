// Operator command console. Every capability of the rover is reachable from here.
import { ROVER, WORLD, K } from './config.js';
import { PRIORITY } from './autonomy.js';

export const HELP = [
  ['help [topic]', 'this list'],
  ['estop | reset | start', 'emergency stop; release latch; power-on self-test (all 3 needed to restart)'],
  ['status', 'one-line vehicle status'],
  ['mission <1-6|cycle> [A|B]', 'run a NASA mission (A = 40-ft module CM-40A, B = 20-ft container CC-20B)'],
  ['abort | resume', 'abort mission | resume after operator override'],
  ['goto <x> <z> | goto <pad|hab|staging|port1|port2|A|B>', 'autonomous drive to a point / site'],
  ['speed <m/s>', 'autonomy cruise speed'],
  ['limit rpm <n> | limit speed <m/s> | limit torque <%>', 'hard limits (rpm = prototype motor rpm, 300 max)'],
  ['rpm auto|<n> | turbo on|off|auto', 'smart rpm 120/180/300 or lock the drive'],
  ['drive <vx> [wz] [vz] | stop', 'direct body-velocity command (m/s, rad/s)'],
  ['steer ackermann|crab|pivot', 'manual steering mode'],
  ['brake on|off', 'parking brake'],
  ['lift kneel|drive|carry|<m> | clamp open|close', 'ride height (kneel / carry) and ring jaws'],
  ['level on|off | ride <m>', 'active leveling, ride-height offset'],
  ['lights on|off', 'work lights'],
  ['timescale <x> | pause', 'simulation speed (1-50x) / pause toggle'],
  ['lidar on|off | lidar channels <n> | lidar res <n>', 'LiDAR power / resolution'],
  ['camera on|off', 'camera power'],
  ['fault block|lidar|camera|comms|battery <%>|wheel <1-6>|shift|port <1|2>', 'Mission 10 fault injection'],
  ['clear faults | clear port <n>', 'remove faults'],
  ['map clear|save|load|forget | routes', 'SLAM map & route memory (forget = erase the stored copy too)'],
  ['plan | priority <A|B> <category>', 'Mission 11 logistics plan / set request category'],
  ['reserve <%>', 'battery reserve that triggers solar recharge'],
  ['sun <az> <el>', 'sun azimuth / elevation in degrees'],
  ['charge [%] | dock status', 'drive to the solar power dock and charge (default 95 %) / dock power report'],
  ['day | night | lunar on|off [minutes per lunar day]', 'sun up / sun down / run the lunar day-night cycle'],
  ['view chase|orbit|top|cam|free', 'camera view'],
  ['restricted on|off', 'enforce geofenced zones in the planner'],
  ['scout on|off|recall|status', 'Mission 9: SCOUT-1 tracked robot maps the route ahead of AEGIS-V1'],
];

const num = (s, d) => { const v = parseFloat(s); return Number.isFinite(v) ? v : d; };

export function runCommand(sim, ui, line) {
  const out = [];
  const say = (m, l = 'info') => out.push({ msg: m, level: l });
  const raw = line.trim();
  if (!raw) return out;
  const [cmd0, ...a] = raw.split(/\s+/);
  const cmd = cmd0.toLowerCase();
  const R = sim.rover, A = sim.autonomy;
  // comms loss: only the hard-wired E-stop link and fault clearing get through
  if (A.faults.comms && !['estop', 'clear', 'help', 'status', 'view', 'timescale', 'pause'].includes(cmd)) {
    say('COMMS LOSS: command not delivered (rover continuing autonomously). Use "clear faults" to restore link.', 'err');
    return out;
  }
  const cargoBy = (k) => { k = (k || 'A').toUpperCase(); return sim.cargo.find((c) => c.id.includes(k === 'A' ? '40A' : k === 'B' ? '20B' : k)) || sim.cargo[0]; };
  const site = (name) => {
    const n = (name || '').toLowerCase();
    const P = sim.terrain.pad, H = sim.terrain.hab;
    if (n === 'pad') return { x: WORLD.pad.x - P.dir.x * 45, z: WORLD.pad.z - P.dir.z * 45, name: 'landing pad' };
    if (n === 'dock') { const L = sim.powerDock.layout; return { ...L.entry, name: 'power dock' }; }
    if (n === 'staging') return { ...P.staging, name: 'staging area' };
    if (n === 'hab') return { x: WORLD.habitat.x + H.nx * 45, z: WORLD.habitat.z + H.nz * 45, name: 'habitat apron' };
    if (n === 'port1' || n === 'port2') { const p = sim.ports[n === 'port1' ? 0 : 1]; return { x: p.x + H.nx * 35, z: p.z + H.nz * 35, name: p.name + ' approach' }; }
    if (n === 'a' || n === 'b') { const c = cargoBy(n); const T = c.T; return { x: c.initial.x - Math.cos(c.yaw) * (T.length / 2 + 16), z: c.initial.z + Math.sin(c.yaw) * (T.length / 2 + 16), name: c.id + ' pre-grasp' }; }
    return null;
  };
  switch (cmd) {
    case 'help': case '?':
      for (const [c, d] of HELP) say(`${c.padEnd(52)} ${d}`);
      say('Keys: W/S drive, A/D steer, Q/E crab, 1/2/3 steer mode, SPACE brake, R/F lift, G grapple, T/Y stage, ESC = E-STOP, C camera, V lidar, H help');
      break;
    case 'estop': case 'e-stop': sim.emergencyStop('console'); break;
    case 'reset': sim.resetEstop(); break;
    case 'start': sim.startUp(); break;
    case 'status': {
      const t = R.telemetry();
      say(`${R.estop ? 'E-STOP' : R.powered ? 'ARMED' : 'DISARMED'} | ${A.mode}${A.missionName ? ' ' + A.missionName : ''} | v ${t.speed.toFixed(2)} m/s | SoC ${(R.power.soc * 100).toFixed(1)}% | tip margin ${t.margin.toFixed(1)} deg | load ${R.loaded ? R.lift.cargo.id : 'none'} | rpm ${R.rpm.proto.toFixed(0)} (${R.rpm.tier}: ${R.rpm.why}; ceiling ${R.limits.rpmCeil}) speed ${R.limits.speed} torque ${R.limits.torquePct}%`);
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
      if (!e) { say('usage: mission <1-6|cycle> [A|B]', 'warn'); break; }
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
      if (!g) { say('usage: goto <x> <z> | goto pad|hab|staging|port1|port2|A|B', 'warn'); break; }
      if (A.gen) A.abort('new goto');
      A.start(`Goto ${g.name}`, A.missionGoto(g.x, g.z, g.name));
      break;
    }
    case 'speed': { if ((a[0] || '') === 'auto') { A.speedSetting = 99; say('Autonomy cruise speed: AUTO (smart rpm decides)', 'ok'); } else { A.speedSetting = Math.max(0.1, Math.min(9, num(a[0], 1.2))); say(`Autonomy cruise speed ceiling ${A.speedSetting.toFixed(2)} m/s`, 'ok'); } break; }
    case 'rpm': case 'turbo': {
      const c = cmd;
      // rpm are PROTOTYPE motor rpm: 300 no-load units capped at 180; wheel rpm = proto / sqrt(K)
      const k = (a[0] || '').toLowerCase();
      if (c === 'turbo') {
        if (k === 'off') { R.rpm.mode = 'fixed'; R.rpm.fixed = 180; say('Turbo disabled: locked at 180 rpm', 'ok'); }
        else if (k === 'on') { R.rpm.mode = 'fixed'; R.rpm.fixed = 300; say('TURBO locked at 300 rpm (terrain governor bypassed; tip governor still active)', 'warn'); }
        else { R.rpm.mode = 'auto'; say('Smart rpm AUTO: 120 crawl / 180 cruise / 300 turbo chosen from terrain', 'ok'); }
      } else if (k === 'auto' || k === '') { R.rpm.mode = 'auto'; say('Smart rpm AUTO: 120 crawl / 180 cruise / 300 turbo chosen from terrain', 'ok'); }
      else { const v = num(a[0]); if (v == null) say('usage: rpm auto | rpm <60..300>', 'warn'); else { R.rpm.mode = 'fixed'; R.rpm.fixed = Math.max(30, Math.min(300, v)); say(`Drive locked at ${R.rpm.fixed} rpm (= ${(R.rpm.fixed / Math.sqrt(K) * 2 * Math.PI / 60 * ROVER.wheel.radius).toFixed(2)} m/s)`, 'ok'); } }
      break;
    }
    case 'limit': {
      const k = (a[0] || '').toLowerCase(), v = num(a[1]);
      if (k === 'rpm' && v != null) { R.limits.rpmCeil = Math.max(20, Math.min(300, v)); say(`Motor rpm ceiling ${R.limits.rpmCeil} (prototype rpm; 300 = no-load, 180 = normal cap) = ${(R.limits.rpmCeil / Math.sqrt(K) * 2 * Math.PI / 60 * ROVER.wheel.radius).toFixed(2)} m/s`, 'ok'); }
      else if (k === 'speed' && v != null) { R.limits.speed = Math.max(0.05, Math.min(99, v)); say(`Ground speed limit ${R.limits.speed} m/s`, 'ok'); }
      else if (k === 'torque' && v != null) { R.limits.torquePct = Math.max(5, Math.min(100, v)); say(`Motor torque limit ${R.limits.torquePct}%`, 'ok'); }
      else say('usage: limit rpm <n> | limit speed <m/s> | limit torque <%>', 'warn');
      break;
    }
    case 'drive': {
      if (A.mode === 'AUTO') A.override();
      R.brake = false; ui.manualHold = true;
      R.cmd.vx = num(a[0], 0); R.cmd.wz = num(a[1], 0); R.cmd.vz = num(a[2], 0);
      say(`Direct command vx ${R.cmd.vx} m/s, wz ${R.cmd.wz} rad/s, vz ${R.cmd.vz} m/s (type "stop" to end)`, 'ok');
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
    case 'ride': R.rideOffset = Math.max(-0.25, Math.min(0.25, num(a[0], 0))); say(`Ride height offset ${R.rideOffset.toFixed(2)} m`, 'ok'); break;
    case 'lights': R.lights = (a[0] || 'on').toLowerCase() !== 'off'; say(`Lights ${R.lights ? 'ON' : 'OFF'}`, 'ok'); break;
    case 'timescale': case 'ts': sim.timeScale = Math.max(0.1, Math.min(50, num(a[0], 1))); say(`Time scale ${sim.timeScale}x`, 'ok'); break;
    case 'pause': sim.paused = !sim.paused; say(sim.paused ? 'Simulation paused' : 'Simulation running', 'ok'); break;
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
    case 'fault': sim.injectFault((a[0] || '').toLowerCase(), a[1] != null ? num(a[1]) : undefined); break;
    case 'clear':
      if ((a[0] || '').toLowerCase() === 'port') { delete A.faults.port[num(a[1], 1)]; say(`PORT-${num(a[1], 1)} available`, 'ok'); }
      else sim.clearFaults();
      break;
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
      if (!m || !(cat in PRIORITY)) { say(`usage: priority <A|B> <${Object.keys(PRIORITY).join('|')}>`, 'warn'); break; }
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
