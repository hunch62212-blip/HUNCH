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
  // [ASSUMED] spatial variation: real regolith is not uniform (loose crater-rim and ejecta soil, compacted inter-crater
  // plains). Stiffness (kc, kphi) and cohesion scale by 1 +- amp, friction angle by +- phiDeg, over patches ~scale m
  // across (terrain.js soilAt). The on-board slip estimator only knows the nominal values above.
  vary: { amp: 0.3, phiDeg: 3, scale: 45 },
  zMax: 0.5,               // m, sinkage limit of the model (about half the tyre radius: beyond this the hub is buried)
};

// [ASSUMED] procedural site. Pad-habitat separation is exactly 2 km per the LLASO-P2 storyboard; seed, plume
// and blast radii are sim choices. The browser picks a random seed on every load (crater / boulder placement
// changes each time; ?seed=N repeats a map); the headless tests use the fixed default seed below.
const ROUTE_LEN = 2000;                       // m, landing pad -> habitat
const ROUTE_DIR = (() => { const x = 1680, z = 1080, L = Math.hypot(x, z); return { x: x / L, z: z / L }; })(); // unit vector pad -> habitat
export const WORLD = {
  size: 2500,              // m, square physics terrain (2.5 x 2.5 km), centred on origin
  res: 1,                  // m, heightfield cell size
  seed: 20262027,
  routeLen: ROUTE_LEN,
  // pad and habitat sit symmetrically about the map centre, exactly ROUTE_LEN apart
  pad: { x: -ROUTE_DIR.x * ROUTE_LEN / 2, z: -ROUTE_DIR.z * ROUTE_LEN / 2, radius: 34 },
  habitat: { x: ROUTE_DIR.x * ROUTE_LEN / 2, z: ROUTE_DIR.z * ROUTE_LEN / 2 },
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
  // six corners (rocker-bogie wheel stations, rocker_bogie.js GEO: x +0.24 / 0 / -0.24, z +-0.235; wheelbase 7.83 m, was 7.02 m, widened for the loaded tip margin)
  corners: [[k(0.24), -k(0.235), 'FL'], [k(0.24), k(0.235), 'FR'], [0, -k(0.235), 'ML'], [0, k(0.235), 'MR'], [-k(0.24), -k(0.235), 'RL'], [-k(0.24), k(0.235), 'RR']],
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
  // Strut = roller-screw electromechanical actuator (CHANGES.md §17): brushless DC servomotor driving a planetary
  // roller screw, welded metal-bellows seal against regolith, impedance-controlled so it presents the same spring and
  // damper as before (k, c, bumpK, bumpC unchanged: the stiffness scheduling that the air strut got from its levelling
  // valve is now a servo gain schedule). No gas, no vented volume: lunar-vacuum, thermal-cycle and dust qualified
  // components at TRL 6+ (roller-screw actuators and bellows seals have flown on lunar and Mars mechanisms).
  //   k, c, sky, bumpK, bumpC, levelRange, levelRate  [ASSUMED] sim tuning, as before
  //   levelTd [ASSUMED] roll-rate damping time, s (CHANGES.md §9)
  //   eff     [ASSUMED] lift efficiency, electrical to mechanical (roller screw ~0.85 x servo + drive ~0.8, derated);
  //           the air strut's pump was 0.25
  //   screwLeadMm / peakServoW / backdriveBrake  [ASSUMED] actuator descriptors for the report; not used by the physics
  susp: {
    k: 42000, c: 9500, sky: 0, bumpK: 1600000, bumpC: 60000, levelRange: 1.75, levelRate: 0.30, levelTd: 6,
    kind: 'roller-screw EMA (BLDC servo, metal bellows seal)', eff: 0.6, screwLeadMm: 10, peakServoW: 1800, backdriveBrake: true,
  },
  // ride set points (module_carry.js CRADLE: kneelExt -40 mm, carryExt +50 mm at 1:16.3)
  ride: { kneel: -k(0.040), carry: k(0.050), max: k(0.060), contact: k(0.310 - 0.318) }, // max = carry + docking trim
  seatY: k(0.318),         // module bottom when seated (rover frame, ride 0)
  ringX: k(0.085),         // two ring clamps at +-x
  // Team drive: 300 rpm gearmotors on 130 mm wheels. The normal cap was 180 rpm (1.22 m/s on the bench); v1.5.0 raises
  // it to 280 (operator request, v1.5.0). The torque-speed curve keeps ~27 % of peak torque at 280 (rover.js tAvail), so the
  // top of the range only opens on flat, firm, straight ground (rover.js _smartRpm). Froude scaling (same
  // gravity-to-inertia ratio) maps the 1:16.3 prototype to full size: speed x sqrt(K), rpm / sqrt(K).
  motor: {
    protoNoLoadRpm: 300, protoCapRpm: 280, protoCrawlRpm: 120, turboRpmWheel: +(300 / Math.sqrt(K)).toFixed(2), crawlRpmWheel: +(120 / Math.sqrt(K)).toFixed(2),
    noLoadSpeed: 300 / Math.sqrt(K) * 2 * Math.PI / 60, capRpmWheel: +(280 / Math.sqrt(K)).toFixed(2),
    peakTorque: 5600, efficiency: 0.86,
    kp: 9000, ki: 5000, brakeTorque: 9000, thermalR: 0.05, thermalC: 30000,
  },
  // [ASSUMED] gyro skid-scrub model (CHANGES.md §17): yaw_rate_est = yaw_rate_imu - kScrub sgn(wz) |vx wz| while turning loaded.
  // kScrub is in (rad/s) per (m/s x rad/s). Fitted with tests/scratch/gyro_turn.mjs: the sim gyro reads the true body
  // rate, and over 55-77 deg constant-curvature turns loaded and empty the IMU heading integral / true heading change is
  // 0.998-1.005, so the fit is 0. The ~1 % turn error of nav_bench is in the odometry translation (CHANGES.md §15), not in heading.
  imu: { kScrub: 0, scrubMinWz: 0.005 },
  rb: { diff: 0.45 }, // rocker-bogie: fraction of independent spring kept (differential-bar pitch stiffness)
  steer: { maxAngle: 0, rate: 0, skid: true }, // v6+ skid steer: the wheels do not steer
  // Operating speed limits [ASSUMED]. The Froude-scaled motor caps (4.95 m/s cruise, 8.2 m/s no-load) only show that the
  // prototype is dynamically similar; they are not safe operating speeds. In 1.62 m/s^2 gravity the soil can brake the
  // vehicle at ~1.1 m/s^2 at most (tan 35 deg x g), obstacles launch a fast vehicle, and the crewed Apollo LRV peaked
  // near 5 m/s. Operator decision (v1.5.0, between the 280 rpm request and this realism pass): empty 5.0 m/s (LRV class;
  // stops in ~14 m at 0.9 m/s^2, so the autonomy's stopping-distance governor needs a clear, mapped 17 m+ ahead to
  // use it); loaded 2.5 m/s (stops in ~16 m at the 0.2 m/s^2 nose-over limit, same governor). Smart-rpm, the curve
  // speed limit and the tip governor trim these further.
  speedLimitEmpty: 5.0,
  speedLimitLoaded: 2.5,
  // tip-safe acceleration limits (m/s^2, rad/s^2). Loaded static tip threshold is ~0.45 g_moon fore-aft, but the
  // passive (rocker-bogie) pitch lets the body nose over at ~0.4 m/s^2 of braking (less downhill): lonLoaded 0.2 (CHANGES.md §14)
  // Traction bound: regolith gives at most c A + W tan(phi) ~ 0.70 W of shear, so no command may ask for more than
  // ~0.7 x 1.62 = 1.13 m/s^2 in any direction. lon 0.9 / lat 0.8 keep the resultant inside that circle with margin;
  // estopFactor 1.2 puts an empty E-STOP at 1.08 m/s^2 (was lon 1.6, lat 1.2, E-STOP 3.2 m/s^2 = 2 g_moon: impossible).
  accel: { lon: 0.9, lonLoaded: 0.2, lat: 0.8, latLoaded: 0.35, yaw: 0.14, estopFactor: 1.2 },
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
    adapterR: k(0.244), adapterRings: [-k(0.085) - 0.15, k(0.085) - 0.15], adapterLen: 8.4, adapterX: -1.0,
    standY: k(0.310), lugXs: [-5.2, 2.2], lugZ: k(0.345), lugR: k(0.010), carryX: 0.15, lugX: 3.7,
  },
};

// Astrobotic-class cargo pods (LLASO-P2 Missions 3 and 8: "4-ft x 2-ft cargo pod"): 1.22 m long x 0.61 m pods, 36 to a
// 20-ft flat rack (4 along x 3 across x 3 tiers). The rack has the 20-ft ISO footprint and rides in the same ring-adapter
// sled as the 20-ft container, so the rover, the stands and the ports see box20's geometry; only the mass and the look
// differ. That is the point of Mission 8: one ring interface, three cargo systems (40-ft module, 20-ft container, pods).
// [ASSUMED] pod 110 kg loaded (36 = 3,960 kg), rack + sled 1,940 kg.
CARGO_TYPES.pod4 = {
  ...CARGO_TYPES.box20,
  label: '36 x 4-ft cargo pods on a 20-ft flat rack, ring-adapter sled', mass: 5900, color: 0xe8e8e2,
  visual: 'podrack', pods: { length: 1.22, dia: 0.61, along: 4, across: 3, tiers: 3, massEach: 110 },
};

// Docking interface: the module's berthing ring meets the supply-building port at the carried axis height.
// portCenterH derived from [CAD] values; opening, vestibule and force limits [ASSUMED].
export const DOCK = {
  portCenterH: +(ROVER.seatY + CARGO_TYPES.cyl40.radius + ROVER.ride.carry).toFixed(3),
  openingD: 2.0,
  // vestibule depth 2.4 m (was 1.2): the 20-ft container rides 1.75 m behind the rover's front (box face 3.18 m,
  // frame front 4.89 m from the rover centre), so with 1.2 m the rover hit the building wall 0.5 m short of the port
  vestibuleLen: 2.4, vestibuleW: 6.0, vestibuleH: 13.5,
  maxPushN: 6000, softCaptureN: 750,
  // Dust-exclusion coupling: a welded metal-bellows soft-capture ring that stands `free` m proud of the rigid collar
  // (drawn on every port, props.js). It is what the module actually pushes against: a spring-damper, so the 23 t
  // vehicle settles on the push force instead of ringing on a rigid wall (rigid-wall contact gave single-step force
  // spikes of 2-6 kN that changed from run to run). The latches fire with the ring compressed to captureGap.
  //   free m, k N/m, c N s/m [ASSUMED]: 1,100 N at 35 mm of compression; damping ratio ~0.4 with the loaded rover
  bellows: { free: 0.06, k: 31000, c: 20000 }, captureGap: 0.036,
};

// AEGIS power dock beside the supply building (positions in the habitat frame: u along the building face,
// v out from it toward the pad). Sized for the lunar night: ~354 h with zero solar input.
//  - night survival load (vault heaters + avionics) 2.2 kW x 354 h = 0.78 MWh
//  - rover parked on the coupler all night in survival mode (~0.3 kW heaters, tests/fdir.mjs) = 0.11 MWh
//  - 20 night delivery legs at ~6 kWh each (2 km loaded + return, tests) = 0.12 MWh, / 0.95 charger
//    -> 1.03 MWh needed; bank 1.5 MWh with the 15 % floor leaves ~20 % margin (was 4.0 MWh, sized for six full
//    0.42 MWh rover recharges a night that no mission needs: ~20 t of cells to land instead of ~7.5 t)
//  - arrays: 4 vertical sun-tracking masts x 120 m^2, 30 % cells x 0.85 derate (dust, cell temperature, wiring,
//    ageing) -> ~165 kW unshaded at a low sun; the masts shade each other when the sun lines up with a row
//    (powerDock.js). Refills the bank in well under a day.
//  - charger 60 kW (0.14 C): the limit is heat. Charge current I^2 R ends up in the pack, and the pack can only
//    radiate ~1 kW (THERMAL.battery); 350 kW (875 A) put ~15 kW into it. At 60 kW (150 A) it is 0.45 kW.
//    No charging below 0 C cell temperature (lithium plating): the heaters warm the pack first.
export const POWER_DOCK = {
  padT: 44, padV: 22, couplerTol: 3.0,
  arrays: [[60, 8], [60, 28], [70, 12], [70, 32]], arrayArea: 120, efficiency: 0.30, mastH: 22, panelW: 8, panelH: 15,
  bank: { w: 14, d: 7, h: 4.2 }, bankWh: 1.5e6, bankFloor: 0.15, bankEff: 0.95, bankMaxDischargeW: 80000,
  arrayDerate: 0.85,
  chargeMaxW: 60000, chargerEff: 0.95, chargeMinCellC: 0,
  hotelW: 700, nightHeaterW: 1500, dayHeaterW: 200,
};

// [ASSUMED] AEGIS-V1 thermal control (src/sim/health.js), after the VIPER "warm box" layout: avionics on a
// heat-spreader vault in the belly-tub nose, heat carried by loop heat pipes to two vertical louvered radiators
// hung under the deck edges (port + starboard). Radiators are on the sides, not the top, because the carried
// module (7.96 m dia.) covers the whole deck and would block a zenith view. Battery strings have their own
// louvered radiator. Thermostatic film heaters (zone controllers A and B) hold the set points.
//  - day, avionics ~1.0 kW, sink 230 K terrain / 3 K sky: louvers ~30 % open, vault ~19 C, heaters ~0 W
//  - night or survival (drive off, 60 W in the vault): heaters ~0.15-0.2 kW in all, so a 354 h lunar night
//    costs well under a quarter of the pack with no solar input (tests/fdir.mjs measures it)
export const THERMAL = {
  sigma: 5.670e-8,
  vault: { C: 120000, radArea: 12, emis: 0.85, louverMin: 0.12, louverOpenC: 15, louverFullC: 30, setC: 5, survivalSetC: -20, heaterMaxW: 800, hotC: 45 },
  battery: { C: 2.4e6, radArea: 3, emis: 0.85, louverMin: 0.12, louverOpenC: 25, louverFullC: 35, setC: 5, survivalSetC: -10, heaterMaxW: 1500, hotC: 50 },
  heaterGainWperK: 200,
  sinkDayK: 230, sinkNightK: 100, skyViewFraction: 0.5,
  // louverMin 0.12: closed-louver effective emittance fraction (flight louvers close to ~0.10-0.15 of open; 0.06 was
  // optimistic). leakWperK: parasitic conduction through MLI seams, mounts and harness to the cold structure, per box.
  leakWperK: { vault: 0.25, battery: 0.4 },
  // battery internal resistance rises in the cold: R(T) = R0 x (1 + coldRperK x (refC - T)) below refC
  coldRperK: 0.03, coldRrefC: 20,
  // Wheel-end freeze protection (CHANGES.md §17). The hub gearbox lubricant (perfluoropolyether grease) must stay
  // above its glass-transition region or the planetary gearbox seizes on the first drive after the night. A 25 W film
  // heater on each of the six hubs switches on below hubFreezeC at night or in survival and off hubHystK above it.
  // hub: lumped thermal state of one MLI-wrapped wheel end (health.js _thermal). All [ASSUMED].
  //   C       J/K, hub motor + gearbox + knuckle (WHL LRU 290 kg of Al / steel, ~0.4 kJ/kg/K)
  //   radArea m^2 radiating surface;  emis effective emissivity of the MLI-wrapped hub (blanket + open drive face)
  //   solarW  W absorbed in daylight (sun + regolith IR);  motorW W dissipated while the drive is powered
  hubHeaterW: 25, hubFreezeC: -65, hubHystK: 5,
  // The drive motor's losses are real heat (hundreds of W per hub when loaded) and there is no air to carry it away.
  // Path: winding (rover.js w.temp, ROVER.motor.thermalC) -> conduction motor.thermalR -> this hub mass -> radiation.
  // The hub rejects through a radiator ring on its outboard face behind a passive thermal switch (paraffin actuator,
  // [ASSUMED]): effective emissivity emis (MLI-like) below switchLoC, rising to emisHot at switchHiC. Cold = insulated
  // (the 25 W freeze heater holds it), hot = radiating. motorW: controller idle dissipation while the drive is powered.
  hub: { C: 120000, radArea: 1.2, emis: 0.05, emisHot: 0.8, switchLoC: 20, switchHiC: 60, solarW: 5, motorW: 8, initC: 10 },
};

// [ASSUMED] vault radiators (THERMAL.vault.radArea 12 m^2 = 2 x 7.2 x 0.85 m). Measured on the team body: the belly-tub
// side walls are at z = +-2.23..2.33 m between y 1.6 and 3.4 m, and the 7.35 m deck overhangs them to z = +-3.09 m at
// y 3.4..3.67 m. Each radiator is bolted to its tub wall on two rails, tucked under that overhang (which shades it from
// sun and falling ejecta) and inboard of the rocker beams (z +-2.95 m) and the fenders (inner edge z +-3.09 m).
// (Earlier versions put them at z = +-4.86 m, sized from frame.halfWid; nothing on the body reaches out there.)
//   len, h m panel size;  y m panel centre height;  z m panel centre off the centreline (|z|, port is -z)
export const RADIATOR = { len: 7.2, h: 0.85, y: 2.95, z: 2.36 };

// [ASSUMED] wheel fenders (CHANGES.md §17, §20). A tyre throws regolith up and back (a "rooster tail", ballistic at
// 1.62 m/s^2 gravity: a 1.2 m/s tread speed lofts it several metres) and sideways in turns, and it coats the radiator and
// louvers (the dust-fouled radiator failure mode in health.js). Each wheel gets an arched fender that rides with the wheel
// on the rocker-bogie, and each radiator an ejecta skirt: a plate hinged to the tub wall along the radiator's lower edge,
// sloping outboard and down, so regolith thrown up from the wheels and rocker beams cannot reach the louvers.
//   clearance m gap tyre to fender;  arcDeg arc covered, centred on the top and biased aft;  width m (tyre width 1.22 m + margin)
//   flapH m trailing flap;  deflector: len m, depth m (hinge to outer edge), angleDeg below horizontal, y m hinge height
export const FENDER = {
  clearance: 0.28, arcDeg: 150, aftBiasDeg: 15, width: 1.5, thickness: 0.03, flapH: 0.45,
  deflector: { len: 7.2, depth: 0.45, angleDeg: 40, y: 2.5 },
};

// [ASSUMED] emergency keep-alive solar blanket (Lunokhod / Yutu-2 lesson: a rover with no on-board generation
// cannot recover from a dead battery away from the dock). A 2 x 6 m flexible array on a telescoping mast, deployed
// only while parked. Vertical, sun-tracking in azimuth. The mast must stand outside the carried module's plan
// footprint (radius 3.98 m), so its canister sits at the end of an outrigger arm from the port-aft deck corner.
// armY is the arm centre height: high enough to clear the rear fender at full kneel plus full bogie lift (~4.15 m).
export const KEEPALIVE = { area: 12, efficiency: 0.30, deployS: 90, mastH: 6.5, massKg: 55, corner: { x: -4.6, z: -4.75 }, armY: 4.35 };

// [ASSUMED] random-failure generator: each catalogue MTBF is divided by rateX (realistic = 1, so a failure
// every few thousand hours; demo = one every few minutes of sim time). Off by default.
export const FAILURES = {
  presets: { realistic: 1, '100x': 100, '1000x': 1000, demo: 20000 },
  defaultPreset: 'demo', maxActiveRandom: 3, seed: 4242, bitPeriodS: 600,
  survivalEnterSoc: 0.06, survivalExitSoc: 0.30,
};

// [ASSUMED] integrator step (120 Hz) and catch-up limit; collision-group bitmasks are code conventions.
export const PHYS = { dt: 1 / 120, maxSubSteps: 40 };

export const GROUPS = { TERRAIN: 1, ROCK: 2, STRUCT: 4, ROVER: 8, CARGO: 16 };
export function groups(member, filter) { return ((member & 0xffff) << 16) | (filter & 0xffff); }
