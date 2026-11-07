// Engineering constants for the LLASO-P2-XPORT-2026 simulation.
// Units are SI throughout (m, kg, s, N, W, rad). Body frame: +X forward, +Y up, +Z right.
//
// Provenance tags used on each group below:
//   [CAD]     team design files in public/team/js/ (1:16.3 prototype), scaled 1:1 by K
//   [WONG]    J.Y. Wong, Theory of Ground Vehicles; Apollo soil mechanics (Carrier, Mitchell)
//   [PHYS]    standard physical constant
//   [ASSUMED] engineering estimate or sim tuning value, not traced to a source; treat as open

// [PHYS] lunar surface gravity and mean radius; solar constant at 1 AU.
export const MOON = {
  g: 1.62,                 // m/s^2, lunar surface gravity
  radius: 1737400,         // m, used for horizon curvature in the far terrain
  solarConstant: 1361,     // W/m^2 at 1 AU, no atmosphere
};

// [WONG] Lunar regolith terramechanics, Bekker-Wong / Janosi-Hanamoto parameters (n, kc, kphi, c, phi, K).
// Source values: Wong, "Theory of Ground Vehicles"; Apollo soil mechanics (Carrier, Mitchell).
// bulk: [ASSUMED] near-surface value in the range Carrier et al. report for the top 15 cm.
export const REGOLITH = {
  n: 1.0,                  // sinkage exponent
  kc: 1400,                // N/m^(n+1) cohesive modulus
  kphi: 820000,            // N/m^(n+2) frictional modulus
  c: 170,                  // Pa, cohesion
  phi: 35 * Math.PI / 180, // internal friction angle
  K: 0.018,                // m, shear deformation modulus
  bulk: 1500,              // kg/m^3, near-surface bulk density
};

// [ASSUMED] procedural site. Pad-habitat separation set to ~2 km per the LLASO-P2 storyboard; seed, plume
// and blast radii are sim choices.
export const WORLD = {
  size: 2048,              // m, square physics terrain, centred on origin
  res: 1,                  // m, heightfield cell size
  seed: 20262027,
  // True 2 km separation between landing pad and habitat, per the storyboard.
  pad: { x: -850, z: -520, radius: 34 },
  habitat: { x: 830, z: 560 },
  plumeHazardRadius: 190,  // m, ejecta hazard ring around the pad
  blastCoreRadius: 32,     // m, no-go core (pad surface + lander)
};

// AEGIS-V1 (the team's design: public/team/js/fusion_rover.js + rocker_bogie.js + module_carry.js; the
// suspension.js and body_control.js files the strut model follows are not in this repo) at full scale. The team's
// module is built at 1:16.3 (0.747 m model of the 12.192 m / 40 ft module), so every prototype dimension is x K.
// Provenance by field:
//   [CAD]     corners, frame, wheel radius/width, ride set points, seatY, ringX, lidar/camera mounts, motor rpm
//             (rocker_bogie.js GEO, module_carry.js CRADLE, lidar_mount.js, rover_bom.js; dims x K, rpm / sqrt(K))
//   [ASSUMED] sprungMass, unsprungMassPerBogie, comLocal, inertia, tyre/suspension stiffness and damping,
//             motor torque/gains/thermal, speedLimitLoaded, accel limits, battery, hotel loads, lift rates,
//             lidar channels/range/noise, camera FOV/range. Full-scale masses are NOT the BOM mass x K^3.
//   [ASSUMED] solar = 0 panels: charging is at the dock station (dock_station.js); no on-board array.
export const K = 12.192 / 0.747; // 16.32
const k = (v) => +(v * K).toFixed(3);
export const ROVER = {
  name: 'AEGIS-V1',
  longName: 'AEGIS-V1 rocker-bogie cargo rover (team design, 1:1 of the 1:16.3 prototype)',
  scale: K,
  // body origin at the ground plane at ride 0 (the team model's root frame), +x forward, +y up
  sprungMass: 6200,        // kg (structure, struts, 6 x hub motors, pack, avionics, ring clamps)
  unsprungMassPerBogie: 300, // kg per corner: 2.1 m sand tyre + hub gearmotor + strut lower
  comLocal: { x: 0.0, y: 2.25, z: 0.0 },
  inertia: { x: 54000, y: 99000, z: 54000 },
  rideHeight: 0,
  // six corners (rocker-bogie wheel stations, rocker_bogie.js GEO: x +0.215 / 0 / -0.215, z +-0.235)
  corners: [[k(0.215), -k(0.235), 'FL'], [k(0.215), k(0.235), 'FR'], [0, -k(0.235), 'ML'], [0, k(0.235), 'MR'], [-k(0.215), -k(0.235), 'RL'], [-k(0.215), k(0.235), 'RR']],
  frame: { halfLen: k(0.30), halfWid: k(0.302), deckY: k(0.225), tubY0: k(0.065), tubZ: k(0.146) },
  bogie: {
    hardpointY: 3.0,       // strut top (body frame); the active strut stands in for the slow rocker links + fast struts (body_control.js)
    // travel = ride range (kneel -0.65 .. carry +0.82) plus the rocker-bogie articulation (+-1.1 m of wheel height difference)
    Lnom: +(3.0 - k(0.065)).toFixed(3), Lmin: +(3.0 - k(0.065) - k(0.040) - 1.1).toFixed(3), Lmax: +(3.0 - k(0.065) + k(0.060) + 1.1).toFixed(3),
    wheelBelowPivot: 0, halfSpan: 0, maxBeamAngle: 0,
  },
  wheel: {
    radius: k(0.065), width: k(0.075), mass: 180, inertia: 120,
    tireK: 420000, tireC: 7000,   // woven steel hoop tyre with chevron cleats (rover_bom.js), scaled
    effDiameterFactor: 2.0,
  },
  susp: { k: 42000, c: 9500, sky: 0, bumpK: 1600000, bumpC: 60000, levelRange: 1.75, levelRate: 0.30 },
  // ride set points (module_carry.js CRADLE: kneelExt -40 mm, carryExt +50 mm at 1:16.3)
  ride: { kneel: -k(0.040), carry: k(0.050), max: k(0.060), contact: k(0.310 - 0.318) }, // max = carry + docking trim
  seatY: k(0.318),         // module bottom when seated (rover frame, ride 0)
  ringX: k(0.085),         // two ring clamps at +-x
  // Team drive: 300 rpm gearmotors capped at 180 rpm on 130 mm wheels (1.22 m/s on the bench). Froude scaling
  // (same gravity-to-inertia ratio) maps the 1:16.3 prototype to full size: speed x sqrt(K), rpm / sqrt(K).
  motor: {
    protoNoLoadRpm: 300, protoCapRpm: 180, turboRpmWheel: +(300 / Math.sqrt(K)).toFixed(2), crawlRpmWheel: +(120 / Math.sqrt(K)).toFixed(2),
    noLoadSpeed: 300 / Math.sqrt(K) * 2 * Math.PI / 60, capRpmWheel: +(180 / Math.sqrt(K)).toFixed(2),
    peakTorque: 5600, efficiency: 0.86,
    kp: 9000, ki: 5000, brakeTorque: 9000, thermalR: 0.05, thermalC: 30000,
  },
  rb: { diff: 0.3 }, // rocker-bogie: fraction of independent spring kept (differential-bar pitch stiffness)
  steer: { maxAngle: 0, rate: 0, skid: true }, // v6+ skid steer: the wheels do not steer
  speedLimitLoaded: 3.8, // carrying the module (CoM 6.8 m up); smart-rpm and the tip governor trim it further
  // tip-safe acceleration limits (m/s^2, rad/s^2). Loaded static tip threshold is ~0.45 g_moon fore-aft.
  accel: { lon: 1.6, lonLoaded: 0.5, lat: 1.2, latLoaded: 0.35, yaw: 0.14, estopFactor: 2.0 },
  battery: { capacityWh: 420000, nominalV: 400, internalR: 0.02 },
  solar: { panelArea: 0, panels: 0, efficiency: 0.30, mastHeight: 0 }, // charged at the AEGIS garage (dock_station.js)
  avionicsW: 900, heaterW: 600, lidarW: 22, cameraW: 6, commsW: 90,
  lift: { travel: 0, rate: 0.5, capacityN: 60000, stageMax: 0, stageRate: 0, lateralMax: 0, jawS: 2.5 },
  // one LiDAR (front deck corner, lidar_mount.js unit FS) and one camera on the avionics pod
  lidar: { mountLocal: { x: k(0.25), y: k(0.266), z: -k(0.15) }, channels: 16, vFovDeg: [-30, 10], tiltDeg: -10, hRes: 180, range: 90, rateHz: 5, noise: 0.02 },
  camera: { mountLocal: { x: k(0.275), y: k(0.29), z: 0 }, hFovDeg: 70, pitchDeg: -12, range: 70 },
  climbMax: 0.85 * k(0.065), // edge height a driven 2.1 m wheel can roll up
};

// Cargo catalogue (Mission 8).
// cyl40: [CAD] module_models.js / module_carry.js geometry x K; mass [ASSUMED].
// box20: [ASSUMED] ISO 668 20-ft external dimensions (6.06 x 2.44 x 2.59 m); mass, sled and lug layout are sim choices.
export const CARGO_TYPES = {
  cyl40: {
    label: '40-ft cargo module (team design, 12.19 x 7.96 m)', shape: 'cylinder',
    length: k(0.747), radius: k(0.244), mass: 14940, color: 0xd8dde3,
    standY: k(0.310), lugXs: [-k(0.21), k(0.21)], lugZ: k(0.345), lugR: k(0.010), bands: [-k(0.085), k(0.085)], bandW: k(0.050), carryX: 0,
    lugX: k(0.21),
  },
  // 20 ft ISO container in a ring-adapter sled, so the same ring clamps carry it (Mission 8)
  box20: {
    label: '20-ft ISO container in ring-adapter sled', shape: 'box',
    length: 6.06, height: 2.59, width: 2.44, mass: 8500, color: 0xc9a227,
    adapterR: k(0.244), adapterRings: [-k(0.085) - 3.1, k(0.085) - 3.1], adapterLen: 8.4, adapterX: -1.0,
    standY: k(0.310), lugXs: [-5.2, 2.2], lugZ: k(0.345), lugR: k(0.010), carryX: 3.1, lugX: 3.7,
  },
};

// Docking interface: the module's berthing ring meets the supply-building port at the carried axis height.
// portCenterH derived from [CAD] values; opening, vestibule and force limits [ASSUMED].
export const DOCK = {
  portCenterH: +(ROVER.seatY + CARGO_TYPES.cyl40.radius + ROVER.ride.carry).toFixed(3),
  openingD: 2.0,
  vestibuleLen: 1.2, vestibuleW: 6.0, vestibuleH: 13.5,
  maxPushN: 6000, softCaptureN: 900,
};

// AEGIS power dock beside the supply building (positions in the habitat frame: u along the building face,
// v out from it toward the pad). Sized for the lunar night: ~354 h with zero solar input.
//  - night survival load (vault heaters + avionics) 2.2 kW x 354 h = 0.78 MWh
//  - plus 6 full rover recharges per night (6 x 0.42 MWh / 0.95) = 2.65 MWh  ->  bank 4.0 MWh with 15 % floor
//  - arrays: 4 vertical sun-tracking masts x 120 m^2, 30 % cells -> ~196 kW at a low polar sun; refills the
//    bank in ~1 day and still covers a 350 kW fast-charge with the bank's help.
export const POWER_DOCK = {
  padT: 44, padV: 22, couplerTol: 3.0,
  arrays: [[60, 8], [60, 28], [70, 12], [70, 32]], arrayArea: 120, efficiency: 0.30, mastH: 22, panelW: 8, panelH: 15,
  bank: { w: 14, d: 7, h: 4.2 }, bankWh: 4.0e6, bankFloor: 0.15, bankEff: 0.95, bankMaxDischargeW: 400000,
  chargeMaxW: 350000, chargerEff: 0.95,
  hotelW: 700, nightHeaterW: 1500, dayHeaterW: 200,
};

// [ASSUMED] integrator step (120 Hz) and catch-up limit; collision-group bitmasks are code conventions.
export const PHYS = { dt: 1 / 120, maxSubSteps: 40 };

export const GROUPS = { TERRAIN: 1, ROCK: 2, STRUCT: 4, ROVER: 8, CARGO: 16 };
export function groups(member, filter) { return ((member & 0xffff) << 16) | (filter & 0xffff); }
