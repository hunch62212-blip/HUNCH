// AEGIS-V1 vehicle health: fault detection, isolation and recovery (FDIR), random failures, thermal control,
// survival mode and the emergency keep-alive array.
//
//  * Catalogue: every line-replaceable unit (LRU) with the service bay it lives in, how you reach it, spares and
//    the repair procedure. Every failure mode lists the sensors that see it and the rule that isolates it.
//  * Detection works on sensor channels (heartbeats, phase currents, resolvers, thermistors, string voltages,
//    louver encoders...), not on the injected failure: legacy "fault ..." commands are detected the same way, and a
//    detector that fires with no real failure behind it is counted as a spurious alarm.
//  * Random failures (off by default): each mode fails at rateX / MTBF per instance, from a seeded generator.
//  * Repairs: "repair <alarm>" safes the rover, walks the procedure steps in sim time and consumes a spare from
//    the on-board kit or the dock stores.
//
// Effect fidelity per mode: 'physics' = the failure changes the vehicle model (wheel free-rolls, capacity lost,
// sensor blind, heater off...); 'annunciation' = the failure only changes its sensor channel and the alarm
// (the physical consequence is not modelled). Listed in the catalogue so nothing is overstated.
import { ROVER, THERMAL, KEEPALIVE, FAILURES, RADIATOR } from './config.js';

const WN = ROVER.corners.map((c) => c[2]); // FL FR ML MR RL RR (wheel n = index n-1; odd n = port side)
const KNEEL = ROVER.ride.kneel;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

function mulberry32(a) {
  return () => { a |= 0; a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}
function gaussFrom(rng) { let u = 0, v = 0; while (!u) u = rng(); while (!v) v = rng(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

// ---------------------------------------------------------------------------------------------------------
// Service bays. pos = body frame (m, +x fwd, +y up from the ground at ride 0, +z starboard). Working height is
// with the rover knelt (ride -0.65 m), which drops every bay to between knee and shoulder height.
// Design rules (all bays): front-access only, no ORU behind another ORU, captive fasteners, one tool (7/16 in
// EVA hex driver), blind-mate connectors on slide rails, every ORU <= 25 kg unless dock-serviced, BIT LED on
// each ORU face, door interlock opens the bay's solid-state breaker (no live 400 V with the door open).
export const BAYS = {
  AV: { name: 'Bay AV - avionics card cage', where: 'nose of the belly tub, door on the front face', pos: [4.45, 2.2, 0], door: 'front door hinges down to a work shelf, 6 captive quarter-turn fasteners' },
  MCL: { name: 'Bay MC-L - port motor controllers', where: 'port tub wall, in the gap between the front and middle wheels', pos: [1.95, 1.9, -2.4], door: 'side door, 1.6 x 1.0 m, 4 quarter-turn fasteners; reach in from outboard' },
  MCR: { name: 'Bay MC-R - starboard motor controllers', where: 'starboard tub wall, between the front and middle wheels', pos: [1.95, 1.9, 2.4], door: 'side door, 1.6 x 1.0 m, 4 quarter-turn fasteners' },
  BTL: { name: 'Bay BT-L - battery strings 1-2', where: 'port tub wall, between the middle and rear wheels', pos: [-1.95, 1.7, -2.4], door: 'side door; string drawers roll out on rails onto the dock exchange cart' },
  BTR: { name: 'Bay BT-R - battery strings 3-4', where: 'starboard tub wall, between the middle and rear wheels', pos: [-1.95, 1.7, 2.4], door: 'side door; string drawers roll out on rails onto the dock exchange cart' },
  SNS: { name: 'Sensor bar', where: 'front of the avionics pod (LiDAR, camera, sun sensor heads)', pos: [4.2, 4.6, 0], workH: 1.6, door: 'the bar pivots forward and down on a service hinge (pin release, gas strut) to 1.6 m; heads on kinematic quick-release mounts, one connector each' },
  RAD: { name: 'Radiators', where: 'vertical louvered panels on the port and starboard tub walls, under the deck overhang', pos: [0, RADIATOR.y, RADIATOR.z], workH: 2.0, door: 'louver actuator boxes at the lower front corner of each panel, 2 fasteners' },
  CRD: { name: 'Cradle', where: 'ring-clamp jaw drives and load cells at the two V-saddles', pos: [0, 4.6, 0], workH: 1.5, door: 'module off: from the fold-out deck walkway (rear ladder); load cells sit in the saddle feet, jaw drives are dock-gantry work' },
  WHL: { name: 'Wheel end', where: 'hub of each wheel (motor, gearbox, spring brake, resolver, declutch)', pos: [0, 1.06, 0], door: 'strut lifts the corner clear (3-legged stance); 12-bolt hub flange' },
  BLY: { name: 'Belly', where: 'dock charge coupler under the tub centre', pos: [0, 1.0, 0], door: 'reached from the dock service pit' },
};

// Line-replaceable units. where: 'field' = crew EVA or base service robot, anywhere, rover safed;
// 'dock' = power-dock service pad (gantry hoist, battery exchange cart). min = hands-on repair time.
const SAFE = ['Park on < 5 deg slope, set spring brakes, autonomy hold', 'Kneel (ride -0.65 m): the bays drop to working height', 'Open the bay door: interlock opens its solid-state breaker; check 0 V at the bay test point'];
const CHECKOUT = ['Close the door (interlock re-arms the breaker)', 'Run the unit built-in test (BIT) and a 10 s functional check; confirm the alarm clears'];
export const LRUS = {
  FC: { name: 'Flight computer (radiation-tolerant, A/B hot standby)', bay: 'AV', qty: 2, kg: 6, where: 'field', min: 20, proto: 'Raspberry Pi 5', steps: ['Confirm the healthy computer is in command', 'Pull the failed FC card (2 wedge-locks)', 'Insert spare, load flight software image from the healthy side', 'Cross-strap check A/B'] },
  PDU: { name: 'Power distribution unit (400 V -> 28 V converter + solid-state breakers), A/B', bay: 'AV', qty: 2, kg: 9, where: 'field', min: 25, proto: 'YRDZXG buck', steps: ['Confirm the bus is on the healthy PDU (diode-OR)', 'Unplug the failed converter (blind-mate)', 'Insert spare converter', 'Load-share check A/B'] },
  MC: { name: 'Hub motor controller drawer (3-phase inverter, resolver interface, brake driver)', bay: 'MC', qty: 6, kg: 11, where: 'field', min: 15, proto: 'Cytron MDD10A', steps: ['Pull the drawer by its handle (4 quarter-turn fasteners)', 'Slide in the spare (blind-mate power + CAN)', 'Spin the wheel 1 rev off load: resolver and phase currents'] },
  WHL: { name: 'Wheel-end assembly (hub motor, planetary gearbox, spring brake, resolver, declutch)', bay: 'WHL', qty: 6, kg: 290, where: 'dock', min: 90, proto: '12 V 300 rpm gearmotor', steps: ['Drive onto the dock service pad', 'Strut lifts the corner (3-legged stance on the other five)', 'Gantry hoist takes the wheel; remove 12 hub bolts and the harness connector', 'Fit spare wheel end, torque bolts, reconnect', 'Lower corner, re-engage declutch, brake release test'] },
  ENC: { name: 'Wheel resolver cartridge', bay: 'WHL', qty: 6, kg: 1.2, where: 'field', min: 20, proto: 'gearmotor Hall encoder', steps: ['Strut lifts the corner a few cm', 'Remove the hub end cap (3 fasteners) and the cartridge', 'Fit spare, align the index mark'] },
  STRUT: { name: 'Roller-screw strut actuator (BLDC servo + metal bellows)', bay: 'WHL', qty: 6, kg: 60, where: 'dock', min: 60, proto: 'linear servo actuator', steps: ['Drive onto the dock service pad, gantry takes the corner weight', 'Safe the servo drive, set the screw brake', 'Swap the actuator cartridge (roller screw, servo, bellows) and the power harness', 'Re-home the screw to its end stop, levelling self-calibration'] },
  BAT: { name: 'Battery string drawer (105 kWh, BMS slave, contactors, fuse)', bay: 'BT', qty: 4, kg: 540, where: 'dock', min: 45, proto: '12.8 V 10 Ah LiFePO4 hot-swap pack', steps: ['Drive onto the dock service pad, dock charger off', 'String contactors confirmed open; dock exchange cart docks to the bay rails', 'Roll out the failed drawer onto the cart, roll in the spare', 'BMS adopts the new string, balance to pack voltage'] },
  HAR: { name: '400 V harness branch / connector', bay: 'AV', qty: 8, kg: 3, where: 'field', min: 40, proto: 'harness', steps: ['Isolate the branch named by the insulation test', 'Inspect the connector and run (dust, chafe)', 'Replace the branch jumper', 'Insulation re-test > 1 MOhm'] },
  IMU: { name: 'Inertial measurement unit, A/B', bay: 'AV', qty: 2, kg: 4, where: 'field', min: 15, proto: 'BNO085', steps: ['Unplug and remove the failed IMU (4 captive screws, kinematic seat)', 'Fit spare', '60 s stationary gyro-bias calibration'] },
  SUN: { name: 'Sun sensor head', bay: 'SNS', qty: 1, kg: 0.5, where: 'field', min: 10, proto: 'n/a', steps: ['Pivot the sensor bar down to the service position (1.6 m)','Swap the head (quick-release)', 'Heading cross-check against scan matching'] },
  LID: { name: 'LiDAR head (16 ch) with electrodynamic dust-shield window', bay: 'SNS', qty: 1, kg: 4, where: 'field', min: 10, proto: 'YDLIDAR X2L', steps: ['Pivot the sensor bar down to the service position (1.6 m)','Release the kinematic mount, unplug, fit spare', 'Extrinsic check against the deck calibration target'] },
  CAM: { name: 'Camera head with electrodynamic dust-shield window', bay: 'SNS', qty: 1, kg: 1.5, where: 'field', min: 10, proto: 'Arducam OV5647', steps: ['Pivot the sensor bar down to the service position (1.6 m)','Swap the head (quick-release)', 'Fiducial calibration check'] },
  WIN: { name: 'Sensor window cleaning (no spare)', bay: 'SNS', qty: 0, kg: 0, where: 'field', min: 10, proto: '-', steps: ['Pivot the sensor bar down to the service position (1.6 m)','Brush and wipe the window (dust tool kit)', 'Run the EDS self-test'] },
  COM: { name: 'Radio: surface-network (primary) / UHF (backup)', bay: 'AV', qty: 2, kg: 3, where: 'field', min: 15, proto: 'n/a', steps: ['Swap the radio card', 'Link test with the base on both radios'] },
  HTR: { name: 'Heater zone controller A/B (film heaters on vault and battery)', bay: 'AV', qty: 2, kg: 2, where: 'field', min: 20, proto: 'n/a', steps: ['Swap the controller card', 'Heater current BIT on every zone'] },
  LOUV: { name: 'Radiator louver actuator', bay: 'RAD', qty: 2, kg: 5, where: 'field', min: 25, proto: 'n/a', steps: ['Open the actuator box at the panel end (2 fasteners)', 'Swap actuator, free the louver blades (dust)', 'Cycle louvers full open / closed'] },
  CLD: { name: 'Ring-clamp jaw gearmotor (dual winding)', bay: 'CRD', qty: 4, kg: 14, where: 'dock', min: 40, proto: 'worm gearmotor', steps: ['Module off the rover (or held by the dock jacks)', 'Gantry reaches the saddle; jaw locked by its worm', 'Swap the gearmotor, both windings checked', 'Open / close cycle'] },
  LDC: { name: 'Cradle load cell', bay: 'CRD', qty: 4, kg: 1, where: 'field', min: 20, proto: 'strut load cell', steps: ['Unload the cradle', 'Swap the load cell', 'Zero and span check'] },
  ESL: { name: 'E-STOP hard-wired loop (harness, connectors, relay)', bay: 'AV', qty: 1, kg: 1, where: 'field', min: 20, proto: 'E-stop loop', steps: ['Loop continuity check by segment', 'Replace the open segment or connector', 'Loop current check, then RESET + START'] },
  CPL: { name: 'Dock charge coupler (rover side)', bay: 'BLY', qty: 1, kg: 18, where: 'dock', min: 30, proto: 'dock charge pad', steps: ['Rover on the dock pad, raised to carry height', 'Swap the coupler from the service pit', 'Pilot-signal and insulation check'] },
};

// ---------------------------------------------------------------------------------------------------------
// Failure modes. inst: instance numbers (wheels 1-6, strings 1-4, A/B ...) or [null] for a single unit.
// sig(h, n) returns the sensor reading that trips the detector, or null. persist: seconds it must hold.
const W = (h, n) => h.sim.rover.wheels[n - 1];
const wheelBay = (n) => (n % 2 ? 'MCL' : 'MCR');
const strBay = (n) => (n <= 2 ? 'BTL' : 'BTR');
const other = (x) => ({ A: 'B', B: 'A', P: 'B' }[x] || 'A');

export const MODES = [
  // ---- mobility ----
  {
    id: 'mc', lru: 'MC', bay: wheelBay, name: 'Hub motor controller dropout', sev: 'warning', inst: [1, 2, 3, 4, 5, 6], mtbfH: 60000, fidelity: 'physics',
    cause: 'inverter gate-driver latch-up or DC-link capacitor failure',
    sensors: ['CAN heartbeat from MC-n (50 Hz)', 'phase current sensor', 'wheel resolver'],
    detect: 'heartbeat missing for 0.3 s with the bus up', persist: 0.3,
    response: 'drive inhibit on wheel n (free-rolls), torque re-allocated to the other 5 wheels, autonomy speed derated 25 %',
    operator: ['Continue the mission derated; avoid slopes > 12 deg loaded', 'Swap the MC drawer at the next stop (field, 15 min, kit spare)'],
    apply: (h, n) => { W(h, n).fault = 'motor'; W(h, n).declutched = false; },
    clear: (h, n) => { if (W(h, n).fault === 'motor' && !W(h, n).declutched) W(h, n).fault = null; },
    truth: (h, n) => W(h, n).fault === 'motor' && !W(h, n).declutched,
    sig: (h, n) => (h.sim.rover.powered && !h.ch.mcAlive(n) ? `no heartbeat MC-${n}, phase I ${h.ch.phaseI(n).toFixed(2)} pu` : null),
  },
  {
    id: 'seize', lru: 'WHL', bay: () => 'WHL', name: 'Wheel-end seized (gearbox / bearing / brake fails to release)', sev: 'warning', inst: [1, 2, 3, 4, 5, 6], mtbfH: 150000, fidelity: 'physics',
    cause: 'regolith past the hub seal, gearbox bearing failure, or spring brake stuck applied',
    sensors: ['phase current sensor', 'wheel resolver', 'hub thermistor', 'brake solenoid current'],
    detect: 'phase current at the limit while the resolver shows < 0.05 rad/s and the wheel is commanded > 0.3 rad/s, for 1.5 s', persist: 1.5,
    response: 'stop, retract the declutch pin so the wheel free-rolls (LRV / Lunokhod heritage), continue on 5 driven wheels derated 25 %',
    operator: ['Confirm the declutch with a 1 m creep (wheel turns freely)', 'Wheel-end swap at the dock (90 min, gantry)'],
    apply: (h, n) => { W(h, n).fault = 'seized'; },
    clear: (h, n) => { W(h, n).fault = null; W(h, n).declutched = false; },
    truth: (h, n) => W(h, n).fault === 'seized' || (W(h, n).fault === 'motor' && W(h, n).declutched),
    // a seized wheel stops while the others keep turning; when every wheel is at the current limit and none turns,
    // the vehicle is stalled against something (common mode), not six wheels seizing at once (v1.3.0: four false
    // "seized" isolations against the building wall left 2 driven wheels and the rover ran away from the port)
    sig: (h, n) => { const w = W(h, n); let turning = 0; for (let m = 1; m <= 6; m++) if (m !== n && Math.abs(h.ch.encW(m)) > 0.15) turning++;
      return turning >= 3 && h.ch.phaseI(n) > 0.95 && Math.abs(h.ch.encW(n)) < 0.05 && Math.abs(w.target) > 0.3 ? `phase I ${h.ch.phaseI(n).toFixed(2)} pu, resolver ${h.ch.encW(n).toFixed(2)} rad/s, cmd ${w.target.toFixed(2)} rad/s` : null; },
    respond: (h, n) => { h.hold(3, `declutching wheel ${n}`); h.after(3, () => { const w = W(h, n); if (w.fault === 'seized') { w.fault = 'motor'; w.declutched = true; h.sim.log(`FDIR: wheel ${n} (${WN[n - 1]}) declutched: free-rolling, 5 driven wheels`, 'warn'); } }); },
  },
  {
    id: 'cool', lru: 'WHL', bay: () => 'WHL', name: 'Hub motor cooling path degraded', sev: 'caution', inst: [1, 2, 3, 4, 5, 6], mtbfH: 80000, fidelity: 'physics',
    cause: 'dust on the hub radiator fins or a cracked thermal strap',
    sensors: ['winding thermistor (each hub)', 'comparison with the other 5 hubs'],
    detect: 'winding > 40 C and > 20 C above the median of the other hubs, for 10 s', persist: 10, derate: 0.7,
    response: 'autonomy speed derated 30 % to cut copper loss; existing torque derate above 110 C still applies',
    operator: ['Prefer flat routes; rest 20 min if the winding passes 100 C', 'Clean fins / replace strap (field 30 min) or wheel end at the dock'],
    apply: (h, n) => { W(h, n).extraHeat = 4000; }, // W of heat the hub can no longer reject
    clear: (h, n) => { W(h, n).extraHeat = 0; },
    sig: (h, n) => { const t = h.ch.windT(n), o = [1, 2, 3, 4, 5, 6].filter((k) => k !== n).map((k) => h.ch.windT(k)).sort((a, b) => a - b), med = (o[2] + o[3]) / 2; return t > 40 && t - med > 20 ? `hub ${n} ${t.toFixed(0)} C vs median ${med.toFixed(0)} C` : null; },
  },
  {
    id: 'enc', lru: 'ENC', bay: () => 'WHL', name: 'Wheel resolver failure', sev: 'caution', inst: [1, 2, 3, 4, 5, 6], mtbfH: 100000, fidelity: 'annunciation',
    cause: 'resolver winding open or connector fretting',
    sensors: ['wheel resolver', 'back-EMF speed estimate from the motor controller'],
    detect: 'resolver reads 0 while back-EMF speed > 0.5 rad/s, for 1 s (latent while parked)', persist: 1,
    response: 'MC-n switches to sensorless (back-EMF) speed control; odometry drops wheel n from the least-squares fit',
    operator: ['No mission impact; swap the resolver cartridge at the next stop (field 20 min)'],
    sig: (h, n) => (Math.abs(h.ch.bemfW(n)) > 0.5 && Math.abs(h.ch.encW(n)) < 0.05 ? `resolver ${h.ch.encW(n).toFixed(2)} vs back-EMF ${h.ch.bemfW(n).toFixed(2)} rad/s` : null),
  },
  {
    id: 'strut', lru: 'STRUT', bay: () => 'WHL', name: 'Roller-screw actuator jam / motor winding open circuit', sev: 'caution', inst: [1, 2, 3, 4, 5, 6], mtbfH: 70000, fidelity: 'annunciation',
    cause: 'regolith past the bellows into the roller screw (jam), or an open servo winding / connector (CHANGES.md §17)',
    sensors: ['servo phase-current sensor', 'actuator LVDT (stroke)', 'screw resolver'],
    detect: 'actuator force response < 85 % of the value the load model expects, for 5 s', persist: 5, derate: 0.85,
    response: 'servo drive derates the corner, screw brake held between levelling moves, speed derated 15 %',
    operator: ['If the response keeps falling below 65 %, kneel is inhibited: finish the leg and go to the dock', 'Actuator cartridge swap at the dock (60 min)'],
    apply: (h, n, f) => { f.p = 1; },
    tick: (h, n, f, dt) => { f.p = Math.max(0.62, f.p - dt / 900); },
    sig: (h, n) => { const r = h.ch.strutRatio(n); return r < 0.85 ? `strut ${n} actuator response ${(r * 100).toFixed(0)} % of expected` : null; },
  },
  // ---- power ----
  {
    id: 'bat', lru: 'BAT', bay: strBay, name: 'Battery string fault (cell short / BMS trip)', sev: 'warning', inst: [1, 2, 3, 4], mtbfH: 120000, fidelity: 'physics',
    cause: 'internal cell short or BMS slave failure',
    sensors: ['string voltage (BMS)', 'string current shunt', 'cell-group voltages'],
    detect: 'string voltage > 3 % below the pack mean, for 2 s', persist: 2, derate: 0.85,
    response: 'open the string contactors; run on 3 strings (-25 % capacity, -25 % peak power), speed derated 15 %',
    operator: ['Re-plan: logistics manager re-checks energy for the remaining requests', 'String drawer swap at the dock (45 min, exchange cart)'],
    apply: (h, n) => h.stringOut(n, true),
    clear: (h, n) => h.stringOut(n, false),
    sig: (h, n) => { const d = h.ch.stringDev(n); return d < -0.03 ? `string ${n} at ${(d * 100).toFixed(1)} % vs pack mean` : null; },
  },
  {
    id: 'runaway', lru: 'BAT', bay: strBay, name: 'Battery cell thermal-runaway precursor', sev: 'critical', inst: [1, 2, 3, 4], mtbfH: 1000000, fidelity: 'physics',
    cause: 'internal short heating a cell group',
    sensors: ['cell-group thermistors (BMS)', 'vent-gas sensor (H2 / CO) in the battery bay', 'string voltage'],
    detect: 'string > 6 C above the pack mean and rising > 1 C/min, with vent gas', persist: 1,
    response: 'open the string contactors, stop the rover, open the battery louvers; hold 2 min to confirm the temperature is falling, then continue derated',
    operator: ['If the string keeps heating after isolation: keep personnel and cargo > 30 m away', 'String drawer swap at the dock (45 min)'],
    apply: (h, n, f) => { f.dT = 0; },
    tick: (h, n, f, dt) => { f.dT = h.isOut(n) ? Math.max(0, f.dT - dt * 0.004) : f.dT + dt * 0.08; },
    clear: (h, n) => h.stringOut(n, false),
    sig: (h, n) => { const f = h.active.get(`runaway:${n}`); const dT = f ? f.dT : 0; return dT > 6 ? `string ${n} +${dT.toFixed(1)} C, vent gas detected` : null; },
    respond: (h, n) => { h.stringOut(n, true); h.hold(120, `battery string ${n} isolated: confirming temperature trend`); h.after(120, () => { const f = h.active.get(`runaway:${n}`); h.sim.log(f && f.dT > 6.5 ? `FDIR CRITICAL: string ${n} still heating after isolation - keep clear, operator decision` : `FDIR: string ${n} cooling after isolation; continuing on 3 strings`, f && f.dT > 6.5 ? 'err' : 'warn'); }); },
  },
  {
    id: 'pdu', lru: 'PDU', bay: () => 'AV', name: 'PDU converter failure', sev: 'warning', inst: ['A', 'B'], mtbfH: 90000, fidelity: 'physics',
    cause: 'converter switch or controller failure',
    sensors: ['converter output voltage', 'converter temperature', 'bus voltage'],
    detect: 'converter output < 24 V (28 V nominal) for 0.2 s', persist: 0.2,
    response: 'diode-OR: the other converter carries the 28 V bus (no interruption). Both lost: drive disarmed, safe mode',
    operator: ['Redundancy lost: swap the converter at the next stop (field 25 min)'],
    apply: (h, x) => { if (h.isFailed('pdu', other(x))) h.safeMode('both PDU converters failed: 28 V bus lost'); },
    sig: (h, x) => (h.isFailed('pdu', x) ? `PDU-${x} output 0.0 V` : null),
  },
  {
    id: 'imd', lru: 'HAR', bay: () => 'AV', name: '400 V insulation fault (bus to chassis)', sev: 'caution', inst: [null], mtbfH: 200000, fidelity: 'annunciation',
    cause: 'chafed harness or conductive dust in a connector',
    sensors: ['insulation monitoring device (IMD) on the 400 V bus'],
    detect: 'insulation resistance < 100 kOhm for 2 s', persist: 2,
    response: 'automatic branch isolation test (30 s): each 400 V branch is opened in turn until the IMD reading recovers, naming the branch',
    operator: ['Single fault is safe (IT earthing); a second fault would be a short. Repair before the next EVA near the rover', 'Replace the named harness branch (field 40 min)'],
    apply: (h, x, f) => { f.branch = `wheel ${1 + Math.floor(h.rngOut() * 6)} motor feeder`; },
    sig: (h) => { const r = h.ch.insulationKOhm(); return r < 100 ? `IMD ${r.toFixed(0)} kOhm` : null; },
    respond: (h) => { h.after(30, () => { const f = h.active.get('imd'); if (f) h.sim.log(`FDIR: insulation test isolated the fault to the ${f.branch}`, 'warn'); }); },
  },
  // ---- avionics ----
  {
    id: 'fc', lru: 'FC', bay: () => 'AV', name: 'Flight computer failure', sev: 'warning', inst: ['A', 'B'], mtbfH: 50000, fidelity: 'physics',
    cause: 'radiation latch-up, memory failure or power supply fault',
    sensors: ['hardware watchdog (heartbeat, 10 Hz)', 'cross-channel data link', 'board current'],
    detect: 'watchdog heartbeat missing 0.5 s', persist: 0.5,
    response: 'the standby computer takes command from its last checkpoint (8 s autonomy hold; motor controllers stop on command timeout). Both lost: safe mode',
    operator: ['Redundancy lost: swap the card (field 20 min)'],
    apply: (h, x) => { if (h.redund.fc === x) { h.hold(1e9, `FC-${x} lost: motor controllers on command timeout`); } if (h.isFailed('fc', other(x))) h.safeMode('both flight computers failed'); },
    sig: (h, x) => (h.isFailed('fc', x) ? `FC-${x} watchdog timeout` : null),
    respond: (h, x) => { if (h.redund.fc === x && !h.isFailed('fc', other(x))) { h.redund.fc = other(x); h.sim.autonomy.holdUntil = h.sim.time + 8; h.sim.autonomy.holdWhy = `FC-${other(x)} taking command`; h.sim.log(`FDIR: FC-${other(x)} in command from checkpoint`, 'warn'); } },
    clear: (h, x) => { h.releaseHold(); },
  },
  {
    id: 'seu', lru: null, bay: () => 'AV', name: 'Single-event upset (radiation) - uncorrectable memory error', sev: 'caution', inst: [null], mtbfH: 8000, fidelity: 'physics', transient: 4,
    cause: 'galactic cosmic ray / solar particle hit in flight-computer memory',
    sensors: ['EDAC memory controller (multi-bit error flag)', 'watchdog'],
    detect: 'EDAC multi-bit flag', persist: 0,
    response: 'watchdog reset of the computer in command and memory scrub (4 s autonomy hold); clears itself',
    operator: ['None. Repeated upsets: check the space-weather alert, consider parking (solar particle event)'],
    apply: (h) => h.hold(4, 'SEU reset'),
    sig: (h) => (h.active.has('seu') ? `EDAC multi-bit error on FC-${h.redund.fc}` : null),
  },
  {
    id: 'imu', lru: 'IMU', bay: () => 'AV', name: 'IMU gyro failure (bias jump)', sev: 'warning', inst: ['A', 'B'], mtbfH: 100000, fidelity: 'physics',
    cause: 'gyro bias shift or laser/fiber degradation',
    sensors: ['IMU-A gyro', 'IMU-B gyro', 'MEMS gyro in the LiDAR head (third voter)'],
    detect: 'the two IMUs differ by > 0.004 rad/s for 2 s; the third gyro votes out the bad one', persist: 2,
    response: 'navigation switches to the healthy IMU; SLAM heading re-anchors on the next sun-sensor fix / scan match',
    operator: ['Redundancy lost: swap the IMU (field 15 min)'],
    apply: (h, x) => { if (h.redund.imu === x) h.sim.sensors.imuFaultBias = 0.012; },
    clear: (h) => { h.sim.sensors.imuFaultBias = 0; },
    sig: (h, x) => { const a = h.ch.gyro('A'), b = h.ch.gyro('B'), c = h.ch.gyro('C'); if (Math.abs(a - b) < 0.004) return null; const bad = Math.abs(a - c) > Math.abs(b - c) ? 'A' : 'B'; return bad === x ? `IMU-A ${a.toFixed(4)} / IMU-B ${b.toFixed(4)} / voter ${c.toFixed(4)} rad/s` : null; },
    respond: (h, x) => { if (h.redund.imu === x) { h.redund.imu = other(x); h.sim.sensors.imuFaultBias = 0; h.sim.log(`FDIR: navigation on IMU-${other(x)}`, 'warn'); } },
  },
  {
    id: 'sun', lru: 'SUN', bay: () => 'SNS', name: 'Sun sensor failure', sev: 'caution', inst: [null], mtbfH: 150000, fidelity: 'physics',
    cause: 'detector or electronics failure',
    sensors: ['sun sensor output', 'ephemeris + shadow ray (sun should be visible)'],
    detect: 'no output for 5 s while the ephemeris says the sun is above the horizon and unshadowed', persist: 5,
    response: 'heading held by gyro + LiDAR scan matching + surveyed beacons (no absolute sun fix)',
    operator: ['Expect slower heading drift correction on long legs; swap the head at the next stop (field 10 min)'],
    apply: (h) => { h.sim.sensors.faults.sun = true; },
    clear: (h) => { h.sim.sensors.faults.sun = false; },
    truth: (h) => !!h.sim.sensors.faults.sun,
    sig: (h) => { const R = h.sim.rover; return R.powered && R.power.sunlit && h.sim.sunDir.y > 0.02 && !h.sim.sensors.sunSensor ? 'sun visible by ephemeris, sensor silent' : null; },
  },
  {
    id: 'lidar', lru: 'LID', bay: () => 'SNS', name: 'LiDAR head failure', sev: 'warning', inst: [null], mtbfH: 40000, fidelity: 'physics',
    cause: 'laser driver, spindle motor or Ethernet PHY failure',
    sensors: ['LiDAR packet rate', 'spindle encoder'],
    detect: 'no packets for 1 s while powered', persist: 1,
    response: 'power-cycle the head (20 s, clears latch-ups about half the time). If hard-failed: proven route at 0.3 m/s, else hold for the operator',
    operator: ['Swap the LiDAR head (field 10 min, sensor bar in service position)'],
    apply: (h) => { h.sim.sensors.faults.lidar = true; },
    clear: (h) => { h.sim.sensors.faults.lidar = false; },
    truth: (h) => !!h.sim.sensors.faults.lidar,
    sig: (h) => { const S = h.sim.sensors; return S.lidarOn && h.sim.rover.powered && S.faults.lidar ? 'packet rate 0 Hz' : null; },
    respond: (h) => h.powerCycle('lidar', () => { h.sim.sensors.faults.lidar = false; }),
  },
  {
    id: 'ldust', lru: 'WIN', bay: () => 'SNS', name: 'Dust on the LiDAR window', sev: 'caution', inst: [null], mtbfH: 6000, fidelity: 'physics',
    cause: 'electrostatically charged regolith (worst near the pad and in wheel spray)',
    sensors: ['LiDAR return intensity from the deck calibration target'],
    detect: 'calibration-target intensity < 75 % of the clean value for 3 s', persist: 3,
    response: 'electrodynamic dust shield (EDS) pulse on the window, 30 s; repeated up to 3 times; range limited meanwhile',
    operator: ['If EDS fails 3 times: wipe the window (field 10 min)'],
    apply: (h, x, f) => { f.k = 1; },
    tick: (h, x, f, dt) => { if (!f.eds) f.k = Math.max(0.35, f.k - dt / 300); h.sim.sensors.lidarCfg.range = h.lidarRange0 * f.k; },
    clear: (h) => { h.sim.sensors.lidarCfg.range = h.lidarRange0; },
    sig: (h) => { const f = h.active.get('ldust'); const k = f ? f.k : 1; return k < 0.75 ? `cal-target intensity ${(k * 100).toFixed(0)} %` : null; },
    respond: (h) => h.eds('ldust', 'LiDAR window'),
  },
  {
    id: 'cam', lru: 'CAM', bay: () => 'SNS', name: 'Camera failure', sev: 'warning', inst: [null], mtbfH: 40000, fidelity: 'physics',
    cause: 'image sensor or link failure',
    sensors: ['frame counter', 'link CRC'],
    detect: 'frame counter stalled for 1 s while powered', persist: 1,
    response: 'power-cycle (20 s). If hard-failed: fiducial docking unavailable, terminal guidance on cradle sensors only; pickups wait for the operator',
    operator: ['Swap the camera head (field 10 min)'],
    apply: (h) => { h.sim.sensors.faults.camera = true; },
    clear: (h) => { h.sim.sensors.faults.camera = false; },
    truth: (h) => !!h.sim.sensors.faults.camera,
    sig: (h) => { const S = h.sim.sensors; return S.cameraOn && h.sim.rover.powered && S.faults.camera ? 'frame counter stalled' : null; },
    respond: (h) => h.powerCycle('camera', () => { h.sim.sensors.faults.camera = false; }),
  },
  {
    id: 'cdust', lru: 'WIN', bay: () => 'SNS', name: 'Dust on the camera window', sev: 'caution', inst: [null], mtbfH: 6000, fidelity: 'physics',
    cause: 'charged regolith',
    sensors: ['image contrast metric on the deck fiducial'],
    detect: 'contrast < 70 % of clean for 3 s', persist: 3,
    response: 'EDS pulse (30 s, up to 3 times); rock-detection range reduced meanwhile',
    operator: ['If EDS fails 3 times: wipe the window (field 10 min)'],
    apply: (h, x, f) => { f.k = 1; },
    tick: (h, x, f, dt) => { if (!f.eds) f.k = Math.max(0.3, f.k - dt / 300); h.sim.sensors.camRangeK = f.k; },
    clear: (h) => { h.sim.sensors.camRangeK = 1; },
    sig: (h) => { const f = h.active.get('cdust'); const k = f ? f.k : 1; return k < 0.7 ? `contrast ${(k * 100).toFixed(0)} %` : null; },
    respond: (h) => h.eds('cdust', 'camera window'),
  },
  // ---- communications ----
  {
    id: 'com', lru: 'COM', bay: () => 'AV', name: 'Radio failure', sev: 'warning', inst: ['P', 'B'], mtbfH: 70000, fidelity: 'physics',
    cause: 'power amplifier or synthesiser failure',
    sensors: ['radio BIT (forward power, lock detect)', 'base heartbeat age'],
    detect: 'radio BIT: forward power 0 / synthesiser unlocked for 1 s', persist: 1,
    response: 'switch to the other radio after 20 s (UHF backup: 64 kbps, telemetry + commands, no video). Both lost: continue autonomously, commands queued at the base; the E-STOP link is separate',
    operator: ['Swap the radio card (field 15 min)'],
    apply: (h, x) => { if (h.redund.com === x || h.isFailed('com', other(x))) h.sim.autonomy.faults.comms = true; },
    clear: (h, x) => { if (!h.isFailed('com', other(x))) h.sim.autonomy.faults.comms = false; },
    sig: (h, x) => (h.isFailed('com', x) ? `radio-${x} forward power 0 W` : null),
    respond: (h, x) => { const y = other(x); if (h.redund.com === x && !h.isFailed('com', y)) h.after(20, () => { if (h.isFailed('com', y)) return; h.redund.com = y; h.sim.autonomy.faults.comms = false; h.sim.log(`FDIR: link restored on radio-${y}${y === 'B' ? ' (UHF backup, 64 kbps)' : ''}. Queued commands: "clear comms" sends them`, 'warn'); }); },
  },
  // ---- thermal ----
  {
    id: 'htr', lru: 'HTR', bay: () => 'AV', name: 'Heater zone controller fails off', sev: 'caution', inst: ['A', 'B'], mtbfH: 80000, fidelity: 'physics', latent: true,
    cause: 'controller relay or driver failure',
    sensors: ['heater current per zone', 'zone thermistors', 'periodic heater BIT (10 min)'],
    detect: 'heater current < 10 % of the thermostat demand for 20 s, or the periodic BIT', persist: 1,
    response: 'zones switch to controller B (A) automatically',
    operator: ['Redundancy lost before the lunar night: swap the controller (field 20 min) before sunset'],
    sig: (h, x) => { if (!h.isFailed('htr', x)) return null; if (h.bitNow) return `heater BIT: controller ${x} no current`; return h.redund.htr === x && h.th.demandW > 30 ? `demand ${h.th.demandW.toFixed(0)} W, current 0 A` : null; },
    respond: (h, x) => { if (h.redund.htr === x && !h.isFailed('htr', other(x))) { h.redund.htr = other(x); h.sim.log(`FDIR: heater zones on controller ${other(x)}`, 'warn'); } },
  },
  {
    id: 'louver', lru: 'LOUV', bay: () => 'RAD', name: 'Radiator louver actuator stuck', sev: 'caution', inst: ['L', 'R'], mtbfH: 60000, fidelity: 'physics', latent: true,
    cause: 'dust in the blade pivots or actuator failure',
    sensors: ['louver position encoder', 'vault thermistors', 'periodic louver-cycle BIT (10 min)'],
    detect: 'position differs from the thermostat command by > 0.25 for 20 s, or the BIT wiggle shows no motion', persist: 20,
    response: 'the other radiator carries the load (each side sized for the full day load). Both stuck closed with the vault > 45 C: park and shed loads',
    operator: ['Swap the actuator and clean the blades (field 25 min)'],
    apply: (h, s) => { h.th.stuck[s] = h.th.louver[s]; },
    clear: (h, s) => { delete h.th.stuck[s]; },
    sig: (h, s) => { if (h.th.stuck[s] == null) return null; if (h.bitNow) return `BIT: louver ${s} did not move`; return Math.abs(h.th.louverCmd - h.th.louver[s]) > 0.25 ? `louver ${s} at ${(h.th.louver[s] * 100).toFixed(0)} %, command ${(h.th.louverCmd * 100).toFixed(0)} %` : null; },
  },
  // ---- cargo ----
  {
    id: 'clamp', lru: 'CLD', bay: () => 'CRD', name: 'Ring-clamp jaw motor primary winding open', sev: 'caution', inst: [1, 2], mtbfH: 100000, fidelity: 'physics', latent: true,
    cause: 'winding open circuit',
    sensors: ['jaw position encoder', 'winding current', 'periodic winding-continuity BIT'],
    detect: 'jaw commanded but no current on the primary winding, or the BIT', persist: 0.5,
    response: 'switch to the secondary winding (half speed: clamp / release take twice as long). Worm drive is self-locking: a held module stays locked',
    operator: ['Swap the gearmotor at the dock with the module off (40 min)'],
    apply: () => {}, // effect applied via h.clampSlow() below
    sig: (h, n) => { if (!h.active.has(`clamp:${n}`)) return null; const L = h.sim.rover.lift; return h.bitNow ? `BIT: jaw ${n} primary winding open` : (L.clamp === 'closing' || L.clamp === 'opening') ? `jaw ${n} commanded, primary current 0 A` : null; },
  },
  {
    id: 'lcell', lru: 'LDC', bay: () => 'CRD', name: 'Cradle load cell failure', sev: 'caution', inst: [1, 2, 3, 4], mtbfH: 90000, fidelity: 'annunciation', latent: true,
    cause: 'strain-gauge bond or amplifier failure',
    sensors: ['cradle load cells (4)', 'strut pressures (independent load estimate)'],
    detect: 'loaded: one cell reads < 40 % of the others for 2 s; unloaded: zero-offset BIT', persist: 2,
    response: 'cargo-shift detection and tip margin use the strut-pressure load estimate instead',
    operator: ['Swap the load cell with the cradle unloaded (field 20 min)'],
    sig: (h, n) => { if (!h.active.has(`lcell:${n}`)) return null; return h.sim.rover.loaded ? `cell ${n} 0.0 kN vs others ${(h.sim.rover.lift.cargo.T.mass * 1.62 / 4000).toFixed(1)} kN` : h.bitNow ? `BIT: cell ${n} zero offset out of range` : null; },
  },
  // ---- safety / dock ----
  {
    id: 'estop', lru: 'ESL', bay: () => 'AV', name: 'E-STOP loop open circuit', sev: 'critical', inst: [null], mtbfH: 300000, fidelity: 'physics',
    cause: 'connector backed out or harness chafe in the hard-wired loop',
    sensors: ['loop current monitor'],
    detect: 'loop current 0 (the loop is normally closed, so an open circuit is itself a stop: fail-safe)', persist: 0,
    response: 'emergency stop (spring brakes, drive torque off). RESET is refused until the loop is repaired',
    operator: ['Segment continuity check, repair (field 20 min), then RESET + START'],
    apply: (h) => { h.fx.estopLoop = true; h.sim.emergencyStop('E-STOP loop open circuit (fail-safe)'); },
    clear: (h) => { h.fx.estopLoop = false; },
    sig: (h) => (h.fx.estopLoop ? 'loop current 0 mA' : null),
  },
  {
    id: 'coupler', lru: 'CPL', bay: () => 'BLY', name: 'Dock charge coupler will not mate', sev: 'caution', inst: [null], mtbfH: 50000, fidelity: 'physics', latent: true,
    cause: 'dust on the contacts or a bent guide pin',
    sensors: ['coupler pilot signal', 'contact resistance', 'pad position fiducials'],
    detect: 'rover knelt on the dock pad but no pilot signal for 3 s', persist: 3,
    response: 'lift and re-kneel once to re-seat; if still no pilot: request service (dock bank cannot charge the rover)',
    operator: ['Swap the coupler from the service pit (dock 30 min)'],
    apply: (h) => { h.fx.coupler = true; },
    clear: (h) => { h.fx.coupler = false; },
    sig: (h) => { const R = h.sim.rover; return h.fx.coupler && h.sim.powerDock.roverOnPad() && R.lift.pos < KNEEL + 0.12 ? 'knelt on pad, pilot signal absent' : null; },
  },
];
export const MODE = Object.fromEntries(MODES.map((m) => [m.id, m]));
const SEV_LEVEL = { caution: 'warn', warning: 'warn', critical: 'err' };

const DOCK_SPARES = { FC: 1, PDU: 1, MC: 2, WHL: 1, ENC: 2, STRUT: 1, BAT: 1, HAR: 2, IMU: 1, SUN: 1, LID: 1, CAM: 1, COM: 1, HTR: 1, LOUV: 1, CLD: 2, LDC: 2, ESL: 1, CPL: 1 };
const KIT_SPARES = { MC: 1, ENC: 1, HAR: 1, LDC: 1, ESL: 1 };

export function bayOf(mode, n) { return BAYS[mode.bay(n)] ? mode.bay(n) : 'AV'; }
// body-frame location of the part (wheel ends, strings and radiator sides resolve to their own spot)
export function partPos(mode, n) {
  const b = bayOf(mode, n);
  if (b === 'WHL' && typeof n === 'number') { const c = ROVER.corners[n - 1]; return [c[0], ROVER.wheel.radius, c[1]]; }
  if (b === 'RAD') return [0, RADIATOR.y, n === 'L' ? -RADIATOR.z : RADIATOR.z];
  if ((b === 'BTL' || b === 'BTR') && typeof n === 'number') { const p = BAYS[b].pos; return [p[0] + (n % 2 ? 0.45 : -0.45), p[1], p[2]]; }
  return BAYS[b].pos.slice();
}
export const unitLabel = (mode, n) => `${mode.lru || 'FC'}${n == null ? '' : '-' + n}${typeof n === 'number' && mode.inst.length === 6 ? ` (${WN[n - 1]})` : ''}`;

export class Health {
  constructor(sim) {
    this.sim = sim;
    this.rate = 10;
    this.active = new Map();   // key -> injected failure (truth): { id, n, t0, random, ...mode state }
    this.alarms = new Map();   // key -> { id, n, tDet, reading, spurious, latency }
    this.persist = new Map();
    this.inhibit = new Set();  // detectors waiting to re-arm after a clear / repair
    this.history = [];
    this.stats = { injected: 0, random: 0, detected: 0, latSum: 0, spurious: 0, repaired: 0 };
    this.random = { on: false, preset: FAILURES.defaultPreset, rateX: FAILURES.presets[FAILURES.defaultPreset], max: FAILURES.maxActiveRandom };
    this.rngFail = mulberry32(FAILURES.seed);
    this.rngOut = mulberry32(FAILURES.seed ^ 0x9e3779b9);
    this.rngNoise = mulberry32(17);
    this.redund = { fc: 'A', imu: 'A', com: 'P', htr: 'A' };
    this.fx = { speedK: 1, coupler: false, estopLoop: false };
    this.th = { Tv: 20, Tb: 18, louver: { L: 0.3, R: 0.3 }, louverCmd: 0.3, louverB: 0.06, stuck: {}, demandW: 0, vaultW: 0, battW: 0, rejW: 0,
      hub: new Array(6).fill(THERMAL.hub.initC), hubOn: new Array(6).fill(false), hubW: 0 };
    this.heaterW = 0;
    this.survival = { on: false, auto: true, t0: 0, why: '' };
    this.array = { state: 'stowed', k: 0, W: 0, yaw: 0 };
    this.job = null; this.jobQueue = [];
    this.spares = { kit: { ...KIT_SPARES }, dock: { ...DOCK_SPARES } };
    this.timers = [];
    this.bitT = FAILURES.bitPeriodS; this.bitNow = false;
    this.lidarRange0 = sim.sensors.lidarCfg.range;
    this.outStrings = new Set();
    this.selected = null;
    this._holdTag = 0;
    const self = this;
    // sensor channels (what the FDIR software can see). Each derives from the vehicle state the sensor measures.
    this.ch = {
      mcAlive: (n) => { const w = W(self, n); return !(w.fault === 'motor' && !w.declutched); },
      phaseI: (n) => { const w = W(self, n); if (w.fault === 'seized' && Math.abs(w.target) > 0.05) return 1.0; return self.ch.mcAlive(n) ? Math.min(1, Math.abs(w.torque) / ROVER.motor.peakTorque) : 0; },
      encW: (n) => (self.active.has(`enc:${n}`) || W(self, n).fault === 'seized' ? 0 : W(self, n).omega + 0.005 * self.noise()),
      bemfW: (n) => (self.ch.mcAlive(n) ? W(self, n).omega + 0.02 * self.noise() : 0),
      windT: (n) => W(self, n).temp + 0.3 * self.noise(),
      strutRatio: (n) => { const f = self.active.get(`strut:${n}`); return (f ? f.p : 1) + 0.01 * self.noise(); },
      stringDev: (n) => (self.outStrings.has(n) && (self.active.has(`bat:${n}`)) ? -0.045 : 0.003 * self.noise()),
      insulationKOhm: () => (self.active.has('imd') ? 40 : 2500) * (1 + 0.02 * self.noise()),
      gyro: (x) => { const t = self.sim.sensors.imu.yawRate - (self.sim.sensors.gyroBias || 0) - (self.sim.sensors.imuFaultBias || 0); const bias = self.isFailed('imu', x) ? 0.012 : 0; return t + bias + (x === 'C' ? 0.0008 : 0.0003) * self.noise(); },
    };
  }

  noise() { return gaussFrom(this.rngNoise); }
  key(id, n) { return n == null ? id : `${id}:${n}`; }
  isFailed(id, n) { return this.active.has(this.key(id, n)); }
  isOut(n) { return this.outStrings.has(n); }
  after(s, fn) { this.timers.push({ t: this.sim.time + s, fn }); }
  log(m, l = 'info') { this.sim.log(m, l); }

  // ---- holds the autonomy executive (motor command timeout / FDIR safing) ----
  hold(s, why) { const A = this.sim.autonomy; A.holdUntil = Math.max(A.holdUntil || 0, this.sim.time + s); A.holdWhy = why; }
  releaseHold() { const A = this.sim.autonomy; A.holdUntil = 0; A.holdWhy = ''; }

  safeMode(why) {
    const R = this.sim.rover, A = this.sim.autonomy;
    R.powered = false; R.brake = true; R.cmd.vx = R.cmd.vz = R.cmd.wz = 0;
    if (A.gen) { A.paused = true; A.mode = 'MANUAL'; }
    this.log(`SAFE MODE: ${why}. Drive disarmed, beacon on. Repair, then START.`, 'err');
  }
  canArm() {
    if (this.isFailed('fc', 'A') && this.isFailed('fc', 'B')) return 'both flight computers failed';
    if (this.isFailed('pdu', 'A') && this.isFailed('pdu', 'B')) return 'both PDU converters failed';
    if (this.fx.estopLoop) return 'E-STOP loop open';
    return null;
  }

  stringOut(n, out) {
    const P = this.sim.rover.power;
    const was = this.outStrings.has(n);
    if (out && !was) { const k0 = 1 - this.outStrings.size * 0.25; this.outStrings.add(n); P.wh -= P.wh * 0.25 / Math.max(0.25, k0); }
    if (!out && was && !this.active.has(`bat:${n}`) && !this.active.has(`runaway:${n}`)) this.outStrings.delete(n);
    if (!out && was && (this.active.has(`bat:${n}`) || this.active.has(`runaway:${n}`))) { /* still failed under the other mode */ }
    P.capK = 1 - this.outStrings.size * 0.25;
  }

  powerCycle(what, fix) {
    this.log(`FDIR: power-cycling the ${what} (20 s)`, 'warn');
    this.after(20, () => {
      const id = what === 'lidar' ? 'lidar' : 'cam';
      if (!this.alarms.has(id)) return;
      if (this.rngOut() < 0.5) { fix(); this.active.delete(id); this.alarms.delete(id); this.log(`FDIR: ${what} recovered after power cycle (latch-up cleared)`, 'ok'); }
      else this.log(`FDIR: ${what} hard failure after power cycle - replace the head (field 10 min). Autonomy degraded.`, 'err');
    });
  }
  eds(id, label, attempt = 1) {
    const f = this.active.get(id); if (!f) return;
    f.eds = true;
    this.log(`FDIR: electrodynamic dust shield pulse on the ${label} (attempt ${attempt}/3, 30 s)`, 'warn');
    this.after(30, () => {
      const g = this.active.get(id); if (!g) return; g.eds = false;
      if (this.rngOut() < 0.85) { MODE[id].clear(this, null); this.active.delete(id); this.alarms.delete(id); this.log(`FDIR: ${label} clean (EDS)`, 'ok'); }
      else if (attempt < 3) this.eds(id, label, attempt + 1);
      else this.log(`FDIR: EDS could not clear the ${label}: manual wipe needed ("repair ${id}")`, 'err');
    });
  }

  // ---- injection (operator "fault ..." and the random generator) ----
  inject(id, n, random = false) {
    const m = MODE[id]; if (!m) return false;
    if (m.inst[0] != null) {
      if (n == null || n === '') n = m.inst[0];
      if (typeof m.inst[0] === 'number') n = Number(n); else n = String(n).toUpperCase();
      if (!m.inst.includes(n)) { this.log(`fault ${id}: unit must be one of ${m.inst.join(', ')}`, 'warn'); return true; }
    } else n = null;
    const k = this.key(id, n);
    if (this.active.has(k)) { this.log(`${k} already failed`, 'info'); return true; }
    const f = { id, n, t0: this.sim.time, random };
    this.active.set(k, f);
    this.inhibit.delete(k);  // a new failure re-arms its detector at once
    m.apply?.(this, n, f);
    if (id === 'clamp') this.sim.rover.lift.clampSlow = 2;
    this.stats.injected++; if (random) this.stats.random++;
    this.history.push({ t: this.sim.time, ev: 'fail', k, random });
    this.log(`SIM: ${random ? 'random failure' : 'injected'} - ${m.name} on ${unitLabel(m, n)} (hidden from FDIR; it must find it)`, 'info');
    this.sim.emit('fault', id);
    return true;
  }

  _clearUnit(id, n) {
    const m = MODE[id], k = this.key(id, n);
    this.active.delete(k);
    m.clear?.(this, n);
    if (id === 'clamp' && ![1, 2].some((j) => this.active.has(`clamp:${j}`))) this.sim.rover.lift.clampSlow = 1;
    if ((id === 'bat' || id === 'runaway') && typeof n === 'number') this.stringOut(n, false);
    if (['fc', 'imu', 'com', 'htr'].includes(id) && this.redund[id] === n && !this.isFailed(id, other(n))) { /* stays on the backup; both healthy now */ }
    this.alarms.delete(k); this.persist.delete(k); this.inhibit.add(k);
  }
  clearAll() {
    for (const k of [...this.active.keys()]) { const f = this.active.get(k); this._clearUnit(f.id, f.n); }
    for (const k of [...this.alarms.keys()]) { const a = this.alarms.get(k); MODE[a.id].clear?.(this, a.n); this.alarms.delete(k); this.inhibit.add(k); }
    this.redund = { fc: 'A', imu: 'A', com: 'P', htr: 'A' };
    this.sim.sensors.imuFaultBias = 0; this.sim.sensors.faults.sun = false; this.sim.sensors.camRangeK = 1;
    this.sim.sensors.lidarCfg.range = this.lidarRange0;
    this.outStrings.clear(); this.sim.rover.power.capK = 1;
    this.fx.coupler = this.fx.estopLoop = false; this.th.stuck = {};
    this.sim.rover.lift.clampSlow = 1;
    for (const w of this.sim.rover.wheels) { w.extraHeat = 0; w.declutched = false; }
    this.job = null; this.jobQueue = []; this.releaseHold(); this.timers = [];
  }

  // ---- random failures ----
  setRandom(on, preset) {
    if (preset != null) {
      const x = FAILURES.presets[preset] ?? parseFloat(preset);
      if (!(x > 0)) { this.log(`random failures: unknown rate "${preset}" (use ${Object.keys(FAILURES.presets).join(', ')} or a number)`, 'warn'); return; }
      this.random.preset = FAILURES.presets[preset] ? preset : `${x}x`; this.random.rateX = x;
    }
    this.random.on = on;
    const lam = MODES.reduce((s, m) => s + m.inst.length / m.mtbfH, 0) * this.random.rateX;
    this.log(`Random failures ${on ? 'ON' : 'OFF'} (rate ${this.random.preset}: expect one every ~${(1 / lam).toFixed(1)} h sim${lam * 3600 > 1 ? '' : ''}${1 / lam < 1 ? ` = ${(60 / lam).toFixed(1)} min` : ''}; at most ${this.random.max} at once)`, on ? 'warn' : 'ok');
    this.sim.emit('fault', 'random');
  }
  _random(dt) {
    if (!this.random.on) return;
    let n = 0; for (const f of this.active.values()) if (f.random) n++;
    if (n >= this.random.max) return;
    for (const m of MODES) for (const u of m.inst) {
      const k = this.key(m.id, u);
      if (this.active.has(k) || this.alarms.has(k)) continue;
      if (this.rngFail() < this.random.rateX * dt / (m.mtbfH * 3600)) { this.inject(m.id, u, true); return; }
    }
  }

  // ---- detection ----
  _detect(dt) {
    for (const m of MODES) for (const u of m.inst) {
      const k = this.key(m.id, u);
      if (this.alarms.has(k)) continue;
      const r = m.sig(this, u);
      // re-arm: after a repair / clear the detector stays inhibited until its signature has gone away once
      // (a repaired hub is still hot for a while; that is not a new failure)
      if (this.inhibit.has(k)) { if (!r) this.inhibit.delete(k); continue; }
      if (!r) { this.persist.delete(k); continue; }
      const p = (this.persist.get(k) || 0) + dt; this.persist.set(k, p);
      // a built-in test is one deliberate measurement: no persistence filter needed
      if (p + 1e-9 < m.persist && !(this.bitNow && m.latent)) continue;
      this.persist.delete(k);
      this._raise(m, u, r);
    }
  }
  _raise(m, n, reading) {
    const k = this.key(m.id, n), f = this.active.get(k);
    const real = m.truth ? m.truth(this, n) : !!f;
    const a = { id: m.id, n, key: k, tDet: this.sim.time, reading, spurious: !real, latency: f ? this.sim.time - f.t0 : null };
    this.alarms.set(k, a);
    if (a.spurious) this.stats.spurious++; else { this.stats.detected++; if (a.latency != null) this.stats.latSum += a.latency; }
    this.history.push({ t: this.sim.time, ev: 'detect', k, latency: a.latency });
    const b = BAYS[bayOf(m, n)];
    this.log(`FDIR ${m.sev.toUpperCase()}: ${m.name} - ${unitLabel(m, n)} isolated, ${b.name} (${b.where}). Sensor: ${reading}. Auto: ${m.response}`, SEV_LEVEL[m.sev]);
    if (!this.selected) this.selected = k;
    m.respond?.(this, n, a);
    if (m.transient) this.after(m.transient, () => { this.active.delete(k); this.alarms.delete(k); this.log(`FDIR: ${m.name} cleared (transient)`, 'ok'); });
    this.sim.emit('fault', m.id);
  }

  // ---- repair ----
  startRepair(arg) {
    const R = this.sim.rover;
    if ((arg || '').toLowerCase() === 'abort') { if (this.job) { this.log(`Repair of ${this.job.k} aborted`, 'warn'); this.job = null; this.jobQueue = []; this.releaseHold(); } return; }
    let keys;
    if (!arg || arg === 'all') keys = [...this.alarms.keys()].filter((k) => !MODE[this.alarms.get(k).id].transient);
    else {
      const k = this.parseKey(arg);
      keys = k && this.alarms.has(k) ? [k] : [];
      if (!keys.length && k && this.active.has(k)) { this.log(`${k}: not detected yet - FDIR has to isolate it before a repair can be planned`, 'warn'); return; }
    }
    if (!keys.length) { this.log(arg ? `No alarm "${arg}" (see "health")` : 'No alarms to repair', 'info'); return; }
    const speed = R.body.linvel(); if (Math.hypot(speed.x, speed.z) > 0.05) { this.log('Repair refused: rover moving. Stop first.', 'warn'); return; }
    this.jobQueue = keys.slice(1); this._beginJob(keys[0]);
  }
  // "mc 3", "mc:3", "mc-3", "imu a" -> "mc:3" / "imu:A"
  parseKey(s) {
    const g = String(s).trim().match(/^([a-z]+)[\s:-]*([0-9a-z])?$/i); if (!g) return null;
    const m = MODE[g[1].toLowerCase()]; if (!m) return null;
    if (m.inst[0] == null) return m.id;
    const n = g[2] == null ? m.inst[0] : typeof m.inst[0] === 'number' ? Number(g[2]) : g[2].toUpperCase();
    return this.key(m.id, n);
  }
  atDock() { const PD = this.sim.powerDock, p = this.sim.rover.pose(), e = PD.layout.entry; return PD.roverOnPad() || Math.hypot(p.x - e.x, p.z - e.z) < 80; }
  _beginJob(k) {
    const a = this.alarms.get(k); if (!a) { this._next(); return; }
    const m = MODE[a.id], L = LRUS[m.lru];
    if (!L) { this._next(); return; }
    const dock = this.atDock();
    if (L.where === 'dock' && !dock) { this.log(`Repair of ${unitLabel(m, a.n)} needs the dock service pad (${L.name}): "charge" or "goto dock" first`, 'warn'); this._next(); return; }
    let src = null;
    if (L.qty > 0) {
      if (this.spares.kit[m.lru] > 0) src = 'kit'; else if (dock && this.spares.dock[m.lru] > 0) src = 'dock';
      if (!src) { this.log(`Repair of ${unitLabel(m, a.n)}: no ${m.lru} spare ${dock ? 'in the dock stores' : 'in the on-board kit (dock has ' + (this.spares.dock[m.lru] || 0) + ')'}`, 'err'); this._next(); return; }
    }
    const steps = [...SAFE, ...L.steps, ...CHECKOUT];
    this.job = { k, id: a.id, n: a.n, steps, i: 0, t0: this.sim.time, dur: L.min * 60, src, who: L.where === 'dock' ? 'dock service gantry' : 'crew EVA / base service robot' };
    const R = this.sim.rover; R.cmd.vx = R.cmd.vz = R.cmd.wz = 0; R.brake = true;
    this.hold(1e9, `repair ${k}`);
    this.log(`REPAIR started: ${L.name} on ${unitLabel(m, a.n)} by ${this.job.who}, ${L.min} min, spare from ${src || 'none needed'}`, 'warn');
  }
  _next() { this.job = null; if (this.jobQueue.length) this._beginJob(this.jobQueue.shift()); else this.releaseHold(); }
  _repairTick() {
    const j = this.job; if (!j) return;
    const R = this.sim.rover; R.cmd.vx = R.cmd.vz = R.cmd.wz = 0; R.brake = true;
    const frac = (this.sim.time - j.t0) / j.dur;
    const i = Math.min(j.steps.length - 1, Math.floor(frac * j.steps.length));
    if (i !== j.i) { j.i = i; this.log(`REPAIR ${j.k} step ${i + 1}/${j.steps.length}: ${j.steps[i]}`, 'info'); }
    if (frac >= 1) {
      const m = MODE[j.id];
      if (j.src) this.spares[j.src][m.lru]--;
      this._clearUnit(j.id, j.n);
      if (j.id === 'seize') { this._clearUnit('mc', j.n); }
      this.stats.repaired++;
      this.history.push({ t: this.sim.time, ev: 'repair', k: j.k });
      this.log(`REPAIR complete: ${unitLabel(m, j.n)} - BIT pass, alarm cleared${j.src ? ` (${j.src} ${m.lru} spares left: ${this.spares[j.src][m.lru]})` : ''}`, 'ok');
      if (j.id === 'estop') this.log('E-STOP loop closed: RESET then START to re-arm', 'warn');
      else if (!R.powered && !R.estop && !this.survival.on && !this.canArm()) this.log('Redundancy restored: START to re-arm the drive', 'warn');
      this._next();
    }
  }

  // ---- thermal (vault + battery, louvered radiators, thermostatic heaters) ----
  _thermal(dt) {
    const T = THERMAL, R = this.sim.rover, P = R.power, th = this.th, sun = this.sim.sunDir;
    const day = P.sunlit && sun.y > 0;
    const Ts4 = T.skyViewFraction * (day ? T.sinkDayK : T.sinkNightK) ** 4;
    const survival = !R.powered;
    const V = T.vault, B = T.battery;
    const htrOK = !this.isFailed('htr', this.redund.htr);
    // vault
    const qV = R.powered ? ROVER.avionicsW + ROVER.lidarW + ROVER.cameraW + ROVER.commsW : 60;
    th.louverCmd = clamp((th.Tv - V.louverOpenC) / (V.louverFullC - V.louverOpenC), V.louverMin, 1);
    let rej = 0;
    for (const s of ['L', 'R']) {
      const want = th.stuck[s] != null ? th.stuck[s] : th.louverCmd;
      th.louver[s] += clamp(want - th.louver[s], -0.02 * dt, 0.02 * dt);
      rej += th.louver[s] * V.emis * T.sigma * (V.radArea / 2) * ((th.Tv + 273.15) ** 4 - Ts4);
    }
    th.demandW = clamp(T.heaterGainWperK * ((survival ? V.survivalSetC : V.setC) - th.Tv), 0, V.heaterMaxW);
    th.vaultW = htrOK ? th.demandW : 0;
    th.Tv += dt * (qV + th.vaultW - rej) / V.C;
    th.rejW = rej;
    // battery
    const I = P.loadW / ROVER.battery.nominalV;
    const qB = I * I * ROVER.battery.internalR;
    th.louverB = clamp((th.Tb - B.louverOpenC) / (B.louverFullC - B.louverOpenC), B.louverMin, 1);
    const rejB = th.louverB * B.emis * T.sigma * B.radArea * ((th.Tb + 273.15) ** 4 - Ts4);
    const dB = clamp(T.heaterGainWperK * ((survival ? B.survivalSetC : B.setC) - th.Tb), 0, B.heaterMaxW);
    th.demandW += dB;
    th.battW = htrOK ? dB : 0;
    th.Tb += dt * (qB + th.battW - rejB) / B.C;
    // wheel-end freeze protection (CHANGES.md §17): each hub is a lumped, MLI-wrapped mass. The thermostat has a dead band
    // (on below hubFreezeC, off hubHystK above) and runs only at night or in survival; by day the sun and the regolith keep
    // the hubs well above the grease glass transition. The heaters share the zone controllers' health ('htr').
    const Hb = T.hub, night = !day, protect = (survival || this.survival.on || night) && htrOK;
    let hubW = 0;
    for (let i = 0; i < 6; i++) {
      const Th = th.hub[i];
      const on = protect && (Th < T.hubFreezeC || (th.hubOn[i] && Th < T.hubFreezeC + T.hubHystK));
      th.hubOn[i] = on;
      const qh = on ? T.hubHeaterW : 0; hubW += qh;
      const rad = Hb.emis * T.sigma * Hb.radArea * ((Th + 273.15) ** 4 - Ts4);
      th.hub[i] = Th + dt * (qh + (day ? Hb.solarW : 0) + (R.powered ? Hb.motorW : 0) - rad) / Hb.C;
    }
    th.hubW = hubW;
    this.heaterW = th.vaultW + th.battW + hubW;
  }

  // ---- emergency keep-alive array ----
  deployArray(on) {
    const a = this.array, R = this.sim.rover, v = R.body.linvel();
    if (on) {
      if (Math.hypot(v.x, v.z) > 0.05) { this.log('Keep-alive array: deploy refused while moving', 'warn'); return; }
      if (a.state === 'stowed' || a.state === 'stowing') { a.state = 'deploying'; R.brake = true; this.log(`Keep-alive array deploying (${KEEPALIVE.area} m2 on a ${KEEPALIVE.mastH} m mast, ${KEEPALIVE.deployS} s)`, 'info'); }
    } else if (a.state !== 'stowed') { a.state = 'stowing'; this.log('Keep-alive array stowing', 'info'); }
  }
  _array(dt) {
    const a = this.array, R = this.sim.rover, sun = this.sim.sunDir, K = KEEPALIVE;
    const wantsMove = Math.abs(R.cmd.vx) > 0.01 || Math.abs(R.cmd.wz) > 0.01;
    if (wantsMove && (a.state === 'deployed' || a.state === 'deploying')) { a.state = 'stowing'; this.log('Keep-alive array stowing before the drive (autonomy holds)', 'info'); }
    if (a.state === 'stowing') this.hold(a.k * K.deployS + 0.5, 'stowing keep-alive array');
    if (a.state === 'deploying') { a.k = Math.min(1, a.k + dt / K.deployS); if (a.k >= 1) { a.state = 'deployed'; this.log('Keep-alive array deployed, tracking the sun', 'ok'); } }
    if (a.state === 'stowing') { a.k = Math.max(0, a.k - dt / K.deployS); if (a.k <= 0) { a.state = 'stowed'; this.log('Keep-alive array stowed', 'info'); } }
    a.yaw = Math.atan2(-sun.z, sun.x);
    a.W = a.state === 'deployed' ? K.area * K.efficiency * 1361 * Math.sqrt(Math.max(0, 1 - sun.y * sun.y)) * R.power.sunlit * (sun.y > -0.01 ? 1 : 0) : 0;
    R.power.auxSolarW = a.W;
  }

  // ---- survival mode (lunar night / dead battery away from the dock) ----
  setSurvival(on, why = 'operator') {
    const S = this.survival, R = this.sim.rover, A = this.sim.autonomy;
    if (on && !S.on) {
      S.on = true; S.t0 = this.sim.time; S.why = why;
      R.cmd.vx = R.cmd.vz = R.cmd.wz = 0; R.brake = true; R.powered = false;
      if (A.gen) { A.paused = true; A.mode = 'MANUAL'; }
      this.log(`SURVIVAL MODE (${why}): drive, LiDAR and camera off; FC-${this.redund.fc} at minimum, heaters to survival set points, beacon on`, 'err');
      if (R.power.sunlit && this.sim.sunDir.y > 0) this.deployArray(true);
    } else if (!on && S.on) {
      S.on = false;
      this.log('Leaving survival mode: power-on self-test', 'warn');
      if (!R.estop && !this.canArm()) this.sim.startUp();
    }
  }
  _survivalAuto() {
    const S = this.survival, P = this.sim.rover.power;
    if (!S.auto) return;
    if (!S.on && P.soc < FAILURES.survivalEnterSoc && !this.sim.powerDock.connected && this.sim.rover.powered) this.setSurvival(true, `battery ${(P.soc * 100).toFixed(1)} %`);
    else if (S.on && P.soc > FAILURES.survivalExitSoc && /battery/.test(S.why)) this.setSurvival(false);
  }

  update(dt) {
    this._thermal(dt);
    this._array(dt);
    for (const f of this.active.values()) MODE[f.id].tick?.(this, f.n, f, dt);
    this.bitT -= dt;
    this.bitNow = this.bitT <= 0;
    if (this.bitNow) this.bitT = FAILURES.bitPeriodS;
    this._random(dt);
    this._detect(dt);
    this.bitNow = false;
    if (this.timers.length) { const now = this.sim.time, due = this.timers.filter((t) => t.t <= now); this.timers = this.timers.filter((t) => t.t > now); for (const t of due) t.fn(); }
    this._repairTick();
    this._survivalAuto();
    let k = 1;
    for (const a of this.alarms.values()) { const m = MODE[a.id]; if (m.derate && !a.spurious) k *= m.derate; }
    this.fx.speedK = k;
    // vault over-temperature (both louvers stuck closed, or a hot day with radiators dusted): shed sensor loads
    if (this.th.Tv > THERMAL.vault.hotC && !this._hotWarned) { this._hotWarned = true; this.log(`THERMAL: avionics vault ${this.th.Tv.toFixed(0)} C > ${THERMAL.vault.hotC} C - park, open bay vents, shed non-essential loads`, 'err'); }
    if (this.th.Tv < THERMAL.vault.hotC - 5) this._hotWarned = false;
  }

  // ---- reports ----
  procedure(id) {
    const m = MODE[id]; if (!m) return null;
    const L = LRUS[m.lru];
    return {
      m, L,
      lines: [
        `${m.name} [${m.sev}] - unit ${m.lru || 'none (transient)'}, MTBF ${m.mtbfH.toLocaleString()} h per unit, effect ${m.fidelity}`,
        `Cause: ${m.cause}`,
        `Sensors: ${m.sensors.join('; ')}`,
        `Detection: ${m.detect}`,
        `Automatic response: ${m.response}`,
        ...m.operator.map((o) => `Operator: ${o}`),
        ...(L ? [`Repair (${L.where}, ${L.min} min, ${L.kg} kg): ${[...SAFE, ...L.steps, ...CHECKOUT].map((s, i) => `${i + 1}. ${s}`).join(' ')}`] : []),
      ],
    };
  }
  summary() {
    const out = [];
    for (const a of this.alarms.values()) { const m = MODE[a.id]; out.push(`${a.key}: ${m.name} (${m.sev}${a.spurious ? ', spurious' : ''}) since ${(a.tDet / 60).toFixed(1)} min - ${a.reading}`); }
    return out;
  }
}
