import { froude } from './scaling.js';

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
// prototype is printed at 1:8.16 (USER DECISION, Nov 2026: K = 8.16 for every print; the 12.192 m x 3.0 m module prints as
// 1.494 m x 0.368 m), so every prototype dimension is x K. K_LEGACY (16.32) is the old 0.747 m module print: the full-scale
// values below that were tuned at that scale (hard-coded metres, masses, stiffnesses, torques) are re-derived from it with
// KS = K / K_LEGACY (lengths x KS, inertia ~ m L^2, torque ~ m L) so K stays the single source.
// Provenance by field:
//   [CAD]     corners, frame, wheel radius/width, ride set points, seatY, ringX, lidar/camera mounts, motor rpm
//             (rocker_bogie.js GEO, module_carry.js CRADLE, lidar_mount.js, rover_bom.js; dims x K, rpm x sqrt(gR / K))
//   [ASSUMED] sprungMass, unsprungMassPerBogie, comLocal, inertia, tyre/suspension stiffness and damping,
//             motor torque/gains/thermal, speedLimitLoaded, accel limits, battery, hotel loads, lift rates,
//             lidar channels/range/noise, camera FOV/range. Full-scale masses are NOT the BOM mass x K^3.
//   [ASSUMED] solar = 0 panels: charging is at the dock station (dock_station.js); no on-board array.
export const K = 8.16;                       // USER DECISION: 1:8.16 prototype for all prints
export const SCALE = froude(K, MOON.g);        // Froude sheet incl. gravity (scaling.js, docs/SCALING.md): SCALE.rpm = sqrt(gR / K), not 1 / sqrt(K)
export const K_LEGACY = 12.192 / 0.747;      // 16.32: the old module print; only used to re-derive the values tuned at that scale
const KS = K / K_LEGACY;                      // 0.5: legacy full-scale metres -> current full-scale metres
const k = (v) => +(v * K).toFixed(3);
const ks = (v) => +(v * KS).toFixed(3);       // a full-scale length tuned at K_LEGACY, re-scaled to K
// Mass budget [ASSUMED]: ~5 t full-scale rover (USER DECISION, Nov 2026: about 4.9 m and 5 t). The legacy 6200 + 6 x 300 kg
// = 8000 kg rover sets the tuning reference: MR = mass ratio, so stiffness ~ MR (same ride frequency), torque ~ MR x KS.
const SPRUNG_KG = 4100, UNSPRUNG_KG = 150, LEGACY_KG = 8000;
const MR = (SPRUNG_KG + 6 * UNSPRUNG_KG) / LEGACY_KG;     // 0.625
const MRs = SPRUNG_KG / 6200;                               // sprung-mass ratio (strut stiffness, body inertia)
// Driven wheels (plan s0, s5.7: FL/FR/RL/RR; ML/MR passive idlers). Only these brake, so they bound the deceleration.
const DRIVEN = ['FL', 'FR', 'RL', 'RR'];
// [ASSUMED] regolith traction coefficient (tan 35 deg = 0.70 friction + ~0.09 cohesion share c A / W at this wheel load).
// Only the driven wheels brake and they carry 4/6 of the weight: a_max = mu x (4 / 6) x g = 0.79 x 0.667 x 1.62 = 0.85 m/s^2.
const TRACTION_MU = 0.79;
const BRAKE_MAX = +(TRACTION_MU * DRIVEN.length / 6 * 1.62).toFixed(3); // 0.853 m/s^2, E-STOP ceiling
export const ROVER = {
  name: 'AEGIS-V1',
  longName: 'AEGIS-V1 rocker-bogie cargo rover (team design, 1:1 of the 1:16.3 prototype)',
  scale: K,
  // body origin at the ground plane at ride 0 (the team model's root frame), +x forward, +y up
  sprungMass: SPRUNG_KG,   // kg (structure, struts, 4 x hub motors, pack, avionics, ring clamps) [ASSUMED]
  unsprungMassPerBogie: UNSPRUNG_KG, // kg per corner: 1.06 m sand tyre + hub gearmotor + strut lower [ASSUMED]
  comLocal: { x: 0.0, y: ks(2.25), z: 0.0 },
  // sprung-body inertia about its own CoM ~ m L^2 (legacy 54000 / 99000 / 54000 kg m^2 at 6200 kg, K_LEGACY). The Rapier body gets
  // the combined sprung + unsprung + held-load properties from Rover.massProps() (rover.js), not this alone.
  inertia: { x: Math.round(54000 * MRs * KS * KS), y: Math.round(99000 * MRs * KS * KS), z: Math.round(54000 * MRs * KS * KS) },
  rideHeight: 0,
  // six corners (rocker-bogie wheel stations, rocker_bogie.js GEO: x +0.24 / 0 / -0.24, z +-0.235; wheelbase 7.83 m, was 7.02 m, widened for the loaded tip margin)
  corners: [[k(0.24), -k(0.235), 'FL'], [k(0.24), k(0.235), 'FR'], [0, -k(0.235), 'ML'], [0, k(0.235), 'MR'], [-k(0.24), -k(0.235), 'RL'], [-k(0.24), k(0.235), 'RR']],
  frame: { halfLen: k(0.30), halfWid: k(0.302), deckY: k(0.225), tubY0: k(0.065), tubZ: k(0.146) },
  bogie: {
    hardpointY: ks(3.0),   // strut top (body frame); the active strut stands in for the slow rocker links + fast struts (body_control.js)
    // travel = ride range (kneel -0.65 .. carry +0.82) plus the rocker-bogie articulation (+-1.1 m of wheel height difference)
    Lnom: +(ks(3.0) - k(0.065)).toFixed(3), Lmin: +(ks(3.0) - k(0.065) - k(0.040) - ks(1.1)).toFixed(3), Lmax: +(ks(3.0) - k(0.065) + k(0.060) + ks(1.1)).toFixed(3),
    wheelBelowPivot: 0, halfSpan: 0, maxBeamAngle: 0,
  },
  wheel: {
    // mass ~ unsprung ratio, inertia ~ m r^2; tyre: load ~ MR, deflection ~ KS so k ~ MR / KS, c keeps the legacy damping ratio
    radius: k(0.065), width: k(0.075), mass: Math.round(180 * UNSPRUNG_KG / 300), inertia: +(120 * (UNSPRUNG_KG / 300) * KS * KS).toFixed(2),
    tireK: Math.round(420000 * UNSPRUNG_KG / 300), tireC: Math.round(7000 * UNSPRUNG_KG / 300), // woven steel hoop tyre with chevron cleats (rover_bom.js). Scaled with the unsprung mass so the wheel-hop mode keeps its legacy 6 Hz and damping ratio 0.31 (omega dt 0.31 at 120 Hz; scaling by mass / length gave 9.4 Hz and chattered)
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
    // k, c ~ sprung mass (legacy 42000 / 9500 at 6200 kg): keeps the ~1.0 Hz ride and the damping ratio. bumpK, bumpC ~ mass too;
    // the bump stop is integrated implicitly in rover.js (its explicit c dt / m was ~2, at the stability limit).
    k: Math.round(42000 * MRs), c: Math.round(9500 * MRs), sky: 0, bumpK: Math.round(1600000 * MRs), bumpC: Math.round(60000 * MRs), levelRange: ks(1.75), levelRate: +(0.30 * KS).toFixed(3), levelTd: 6,
    kind: 'roller-screw EMA (BLDC servo, metal bellows seal)', eff: 0.6, screwLeadMm: 10, peakServoW: 1800, backdriveBrake: true,
  },
  // ride set points (module_carry.js CRADLE: kneelExt -40 mm, carryExt +50 mm at 1:16.3)
  ride: { kneel: -k(0.040), carry: k(0.050), max: k(0.060), contact: k(0.310 - 0.318) }, // max = carry + docking trim
  seatY: k(0.318),         // module bottom when seated (rover frame, ride 0)
  ringX: k(0.085),         // two ring clamps at +-x
  // Team drive: 300 rpm gearmotors on 130 mm wheels. The normal cap was 180 rpm (1.22 m/s on the bench); v1.5.0 raises
  // it to 280 (operator request, v1.5.0). The torque-speed curve keeps ~27 % of peak torque at 280 (rover.js tAvail), so the
  // top of the range only opens on flat, firm, straight ground (rover.js _smartRpm). Froude scaling (same
  // gravity-to-inertia ratio) maps the 1:K prototype (Earth g) to full size (lunar g): speed x sqrt(K gR), wheel rpm x sqrt(gR / K)
  // with gR = 1.62 / 9.807 (scaling.js). Until the review pass this was rpm / sqrt(K), which ignored the gravity change and made
  // the full-scale rover 2.5x too fast (5.83 m/s at 300 rpm instead of 2.37 m/s).
  motor: {
    protoNoLoadRpm: 300, protoCapRpm: 280, protoCrawlRpm: 120, turboRpmWheel: +(300 * SCALE.rpm).toFixed(2), crawlRpmWheel: +(120 * SCALE.rpm).toFixed(2),
    noLoadSpeed: 300 * SCALE.rpm * 2 * Math.PI / 60, capRpmWheel: +(280 * SCALE.rpm).toFixed(2),
    // v1.8.0 (ARM plan s0/s6.1): 4 driven wheels keep the old 6-motor budget: peakTorque 5600 -> 8400, brakeTorque 9000 -> 13500 (x1.5).
    // NOTE: until the P1b rover.js drive change, the two middle (idler) wheels still read these values too.
    // Torques ~ m L (same acceleration capability per kg at wheel radius ~ KS): legacy 8400 / 13500 N m at 8000 kg, K_LEGACY.
    // kp / ki scale the same way so the PI saturates at the same speed error (rad/s).
    peakTorque: Math.round(8400 * MR * KS), efficiency: 0.86,
    kp: Math.round(9000 * MR * KS), ki: Math.round(5000 * MR * KS), brakeTorque: Math.round(13500 * MR * KS), thermalR: 0.05, thermalC: 30000,
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
  // Review pass (item 6): with Froude-correct rpm scaling the wheels top out at 2.37 m/s (300 rpm, turbo) / 2.21 m/s (280 cap) in
  // lunar gravity, so these are no longer the binding limits (the old 5.0 m/s was only reachable because rpm ignored gravity).
  // speedLimitEmpty = 2.4 sits just above the motor top speed so the rpm governor decides.
  speedLimitEmpty: 2.4,
  speedLimitLoaded: 2.5,
  // tip-safe acceleration limits (m/s^2, rad/s^2). Loaded static tip threshold is ~0.45 g_moon fore-aft, but the
  // passive (rocker-bogie) pitch lets the body nose over at ~0.4 m/s^2 of braking (less downhill): lonLoaded 0.2 (CHANGES.md §14)
  // Traction bound: only the 4 driven wheels drive and brake (ML/MR are idlers), and they carry 4/6 of the weight, so the
  // longitudinal limit is BRAKE_MAX = TRACTION_MU x 4/6 x g = 0.85 m/s^2 (was derived for six braked wheels: 0.7 x g = 1.13).
  // lon = min(0.75, BRAKE_MAX) for normal driving; estopFactor puts the E-STOP exactly at BRAKE_MAX (lon x estopFactor <= 0.85).
  // lat 0.8 uses all six tyres' side force.
  accel: { lon: Math.min(0.75, BRAKE_MAX), lonLoaded: 0.2, lat: 0.8, latLoaded: 0.35, yaw: 0.14, estopFactor: +(BRAKE_MAX / Math.min(0.75, BRAKE_MAX)).toFixed(3), brakeMax: BRAKE_MAX },
  // USER DECISION (Nov 2026): 150 kWh pack. dockMaxW [ASSUMED]: full-scale charge power cap (rover.js _powerStep); a realistic
  // surface charger is 10-20 kW, and 0.14 C of 150 kWh is 21 kW.
  battery: { capacityWh: 150000, nominalV: 400, internalR: 0.02, dockMaxW: 20000 },
  solar: { panelArea: 0, panels: 0, efficiency: 0.30, mastHeight: 0 }, // charged at the AEGIS garage (dock_station.js)
  avionicsW: 900, heaterW: 600, lidarW: 22, cameraW: 6, commsW: 90,
  // ride-height stroke: `rate` is the fast path for small trims. A move longer than trimBand follows a minimum-jerk (quintic) S-curve,
  // ease-in and ease-out, of duration max(tMin, 1.875 x stroke / vPeak, tFull x min(1, stroke / fullAt)): any stroke of fullAt (0.5 m) or
  // more takes at least tFull, so the 0.65 m squat-down takes 3.7 s (peak 0.33 m/s, 0.27 m/s^2) and the 1.47 m kneel-to-carry lift 3.9 s
  // (peak 0.70 m/s, 0.55 m/s^2 = 0.34 lunar g). The levelling servo feeds the stroke rate forward (rover.js _leveling), so this is the
  // motion of the body itself, not only of the set point. Soft seat: an upward stroke that would run the saddles into a module resting
  // overhead stops seatMargin m under it (duration x seatK: the rover moves alone), then lifts from rest (rover.js _liftStep).
  // [ASSUMED] sim tuning; real roller-screw EMAs are jerk-limited over seconds.
  lift: { travel: 0, rate: 0.5, capacityN: 60000, stageMax: 0, stageRate: 0, lateralMax: 0, jawS: 2.5, sCurve: { vPeak: 0.7, tMin: 0.5, tFull: 3.7, fullAt: 0.5, trimBand: 0.02, seatMargin: 0.03, seatK: 0.65 } },
  // one LiDAR (front deck corner, lidar_mount.js unit FS) and one camera on the avionics pod
  // v1.7.0 sensor suite. LiDAR mount [CAD]: the team's chosen option D of the lidar_mount.js trade study, a fixed scanner on a front
  // deck corner under the module (unit FS; unit FP, mirrored in z, is the cold spare). The spec for v1.7.0 asked for a centre mast at
  // y = k(0.35) = 5.7 m: that point is inside the carried module (underside 5.2 m at ride 0) and the study rejected a mast for that
  // reason ("stowed head sat 24 mm inside the module swing"), so the mount stays where the CAD and the drawn model have it.
  // Scanner [ASSUMED]: 32 beams over -30..+15 deg (was 16 over -30..+10), 360 deg sweep, head pitched 10 deg nose-down as before.
  //   rateHz / hRes  the head's rated 10 Hz x 360 azimuth steps = 115 000 rays/s, 6x what the old scanner cast and the dominant CPU
  //                  cost of the whole simulation. What it runs is `tiers` (sensors.js _lidarPolicy): [speed limit m/s, scan Hz,
  //                  azimuth steps], the old scanner's own schedule ...
  //   interlace      ... with the 32 beams fired as two interleaved sets of 16 on alternate sweeps: every sweep costs what a 16-beam
  //                  one did (19 200 rays/s), and the map gets all 32 rings over two sweeps (the rover moves < 0.5 m between them).
  // Camera [ASSUMED]: a stereo depth pair on the avionics pod, 90 deg HFOV (was 70 mono), 0.18 m baseline, dense depth to depthRange m.
  //   disparityNoise  sub-pixel disparity sigma (px, imagePx-wide image): depth sigma = z^2 x disparityNoise / (f x baseline),
  //                   3 cm at 8 m, 10 cm at 14 m, 0.6 m at 35 m;  stereoRateHz / stereoGrid  depth frame rate and its columns x rows
  //   fuseSigma       m: only depth points at or under this sigma are fused into the map (slam.js integrate)
  lidar: { mountLocal: { x: k(0.25), y: k(0.266), z: -k(0.15) }, channels: 32, vFovDeg: [-30, 15], tiltDeg: -10, hRes: 360, range: 90, rateHz: 10, noise: 0.02,
    interlace: true, tiers: [[0.5, 5, 240], [2.5, 8, 150], [Infinity, 12, 100]] },
  camera: {
    mountLocal: { x: k(0.275), y: k(0.29), z: 0 }, hFovDeg: 90, pitchDeg: -12, range: 70,
    stereo: true, baselineM: 0.18, depthRange: 35, disparityNoise: 0.03, stereoRateHz: 4, stereoGrid: [32, 18], imagePx: 640, fuseSigma: 0.10,
  },
  climbMax: 0.85 * k(0.065), // edge height a driven 2.1 m wheel can roll up
};

// Cargo catalogue (Mission 8).
// cyl40: [CAD] module_models.js / module_carry.js geometry x K; mass [ASSUMED].
// box20: [ASSUMED] ISO 668 20-ft external dimensions (6.06 x 2.44 x 2.59 m); mass, sled and lug layout are sim choices.
export const CARGO_TYPES = {
  cyl40: {
    // USER DECISION (Nov 2026): 12.192 m x 3.0 m diameter (prints at 1:8.16 as 1.494 x 0.368 m). Lugs keep their fraction of
    // the length (legacy +-0.21 of 0.747); bands sit on the rover ring clamps (ROVER.ringX); mass [ASSUMED], unchanged.
    label: '40-ft cargo module (team design, 12.19 x 3.0 m)', shape: 'cylinder',
    handling: 'straddle', // ARM plan s6.1
    length: 12.192, radius: 1.5, mass: 14940, color: 0xd8dde3,
    standY: k(0.310), lugXs: [-+(12.192 * 0.21 / 0.747).toFixed(3), +(12.192 * 0.21 / 0.747).toFixed(3)], lugZ: k(0.345), lugR: k(0.010), bands: [-k(0.085), k(0.085)], bandW: k(0.050), carryX: 0,
    lugX: +(12.192 * 0.21 / 0.747).toFixed(3),
  },
  // 20 ft ISO container in a ring-adapter sled, so the same ring clamps carry it (Mission 8)
  box20: {
    label: '20-ft ISO container in ring-adapter sled', shape: 'box', handling: 'straddle', // ARM plan s6.1
    length: 6.06, height: 2.59, width: 2.44, mass: 8500, color: 0xc9a227,
    adapterR: 1.5, adapterRings: [-k(0.085) - 0.15, k(0.085) - 0.15], adapterLen: 8.4, adapterX: -1.0,
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

// ---------------------------------------------------------------------------------------------------------------
// ARM-HANDLED CARGO + OUTER-WHEEL DRIVE (docs/ARM_CARGO_BUILD_PLAN.md, v1.8.0 / CHANGES.md s33). Full-scale SI (Moon g).
// Prototype (Earth bench, 1:16.32) values are in ACT.proto. [ASSUMED] = no source. [VERIFY] = placeholder size/datasheet.
// ---------------------------------------------------------------------------------------------------------------

// Plan s0, s5.7: FL/FR/RL/RR driven; ML/MR passive idlers (Tm = 0, never braked). ROVER.motor torque/brake raised x1.5 above.
export const DRIVE = {
  driven: DRIVEN,
  idlerDragNm: 15,   // [ASSUMED] bearing Coulomb drag per idler wheel, N*m
  degYawGain: 2.0,   // [ASSUMED] degraded-drive (dead driven motor) yaw-rate integral gain on commanded wz, 1/s per (rad/s*s)
  idlerViscous: 4,   // [ASSUMED] bearing viscous drag, N*m*s/rad
};

// Plan s5.1/s5.2/s6.1: actuator model parameters. Torques are joint-output. Full-scale values scale the prototype
// shoulder/elbow/wrist/latch stalls from the s4.3 torque table; all [ASSUMED] unless noted.
export const ACT = {
  shoulder: { V: 400, Ts: 26000, w0: 0.35, Is: 40, I0: 0.8, Ilim: 32, backlash: 0.0017, fs: 4, Rth: 0.6, Cth: 9000, Tmax: 120, brake: true }, // [ASSUMED] s4.3: 10.39 kN*m static = 40 % of stall
  elbow:    { V: 400, Ts: 18000, w0: 0.35, Is: 28, I0: 0.6, Ilim: 22, backlash: 0.0017, fs: 4, Rth: 0.8, Cth: 6000, Tmax: 120, brake: true }, // [ASSUMED]
  wrist:    { V: 400, Ts: 1200,  w0: 0.6,  Is: 4,  I0: 0.2, Ilim: 3.2, backlash: 0.003, fs: 6, brake: true },                                // [ASSUMED]
  latch:    { V: 28,  Ts: 300,   w0: 1.5,  Is: 6,  I0: 0.3, Ilim: 5 },                                                                       // [ASSUMED]
  // Earth bench values used by tests and the calculator (s1.5 datasheets, s5.2)
  proto: {
    big60:   { V: 7.4, Ts: 5.39, w0: 6.0,  Is: 5.0, I0: 0.25, backlash: 0.0087 },                       // [VERIFY datasheet] 60 kg*cm-class HV servo (~55 kg*cm at 7.4 V)
    ds3218:  { V: 6.8, Ts: 2.11, w0: 7.48, Is: 2.5, I0: 0.15, TsV: [[5, 1.77], [6.8, 2.11]] },          // DS3218 datasheet (s1.5)
    mg996r:  { V: 6.0, Ts: 1.08, w0: 7.48, Is: 2.5, I0: 0.17 },                                         // MG996R datasheet (s1.5); I0 [ASSUMED]
    mg90s:   { V: 6.0, Ts: 0.216, w0: 13.1, Is: 0.7, I0: 0.1 },                                         // MG90S 2.2 kg*cm; I0 [ASSUMED]
    gm37d30: { V: 12,  Ts: 1.37, w0: 34.6, Is: 5.5, I0: 0.2, gearLimitNm: 0.98 },                       // Pololu 37D 30:1 (s1.5); I0 [ASSUMED]
  },
};

// Plan s4, s5.3, s6.1: two mirrored 3-DOF arms (shoulder/elbow/wrist pitch). Prototype mm x K = full scale.
export const ARM = {
  enabled: true, arms: 2,
  shoulder: { x: k(0.15), y: k(0.275), z: k(0.175) },   // [VERIFY] +-z per side; prototype (150, 275, +-175) mm
  L1: k(0.20), L2: k(0.20),                              // [VERIFY] prototype 200 mm links
  links: { m1: 120, lc1: k(0.10), I1: 110, mElbow: 110, m2: 120, lc2: k(0.10), I2: 110, mWrist: 40, mHook: 40 }, // [ASSUMED] s4.2 masses x ~K^3-ish budget, kg / kg*m^2
  fixedMassPerArm: 210,                                  // [ASSUMED] s3.3 (tower + shoulder servo), kg; added to rover sprung mass
  limits: { q1: [-1.05, 3.0], q2: [-3.0, 2.4], q3: [-2.8, 2.8] }, // [ASSUMED] rad. q2 > 0 is the place-side elbow-up branch (arm reaching back over the shoulder); q2 < 0 the pick/apex branch
  rateMax: 0.12, accMax: 0.05, substeps: 8,              // s4.3 full-scale rates (rad/s, rad/s^2); dt = 1/960 s
  stow: { q: [2.8, -2.8, 1.2] }, apex: { dy: k(0.24) },// s4.1 [VERIFY] stow: link 1 folded back over the deck, link 2 forward above it, hook tip clear of the LiDAR plinths and avionics pod
  pick: { xBody: k(0.42), clearance: k(0.02), preBelow: k(0.02), preFwd: k(0.03), seatV: 0.02 }, // s4.1 / s6.4 (seatV m/s)
  prePlaceH: k(0.06),                                    // plan s6.4 step 10: the last k(0.06) above the pins is descended at seatV [ASSUMED] (no source for the value in the CAD)
  releaseDrop: k(0.03), releaseSlide: k(0.03),           // plan s6.4: after the gates open the V-seat hooks go DOWN k(0.03) (it opens upward: rising would lift the trunnion again), then slide clear by k(0.03) [ASSUMED slide]
  keepOutMargin: ks(0.06),                               // m (legacy 0.06 x KS: it scales with the hook mouth, half-disc radius mouth / 2 = 0.12 m), hook tip may not rise more than this above an un-seated trunnion centre (hook-trunnion interference check) [ASSUMED]
  hook: { mouth: k(0.030), seatHalf: 0.785, mu: 0.3, fh: 3, zeta: 0.5, latchMaxN: 2.5e4 }, // s5.4: V-seat 45 deg, f_h 3 Hz; mu, zeta, latchMaxN [ASSUMED]
  ctrl: { kp: 1 / 0.087, ki: 0.2, kd: 0.05, deadband: 0.0017, gravityFF: true }, // s5.2: kp = 1/theta_sat (5 deg) [ASSUMED]; ki, kd [ASSUMED]
  tipRatioMax: 0.75, minMarginDeg: 8, ratedKg: 3000, peakW: 6000, hotelW: 120,   // s0 / s4.3 (ASME B30.5-style 75 %); peakW, hotelW [ASSUMED]
};

// Plan s6.4: acquireArm thresholds (full scale). [ASSUMED] unless noted.
export const ACM_PICK = { standoff: k(0.25), creepV: 0.05, latTol: k(0.007), yawTolDeg: 3, alongTol: k(0.04),
  retries: 2, overloadK: 1.15, seatTimeout: 20 }; // overloadK was 1.6 and could never trip (hook load peaks at 1.006 W nominal, 1.13 W for the hook spring at seatV); 1.15 W trips on stuck feet [ASSUMED]

// Plan s3.3, s5.4: deck interface. placeX is computed at sim init from the balance rule (loaded CG -> x = 0; about -0.347 m at K = 8.16, was -0.734 m at 1:16.3).
export const DECK = { placeX: null /* computed at init */, capture: k(0.006),
  pinSpan: k(0.14), latchTime: 0.6, seatedEps: k(0.002) };                // [VERIFY] pin geometry; latchTime [ASSUMED]

// Plan s5.5: ACM foot-regolith contact. Ku = 10x secant stiffness, creep = 1.15-1 settling, tauSettle s, cAdhK x REGOLITH cohesion.
export const ACM_SOIL = { foot: k(0.03), nFeet: 4, Ku: 10, zeta: 0.7, creep: 0.15, tauSettle: 120, cAdhK: 0.3, bristleK: 3e5 }; // [ASSUMED] Ku, creep, tauSettle, cAdhK; bristleK = parked-brake value

// Plan s0, s6.1: ACM = Arm-handled Cargo Module (sits on regolith; prototype 180 x 240 x 140 mm, 1.0 kg).
CARGO_TYPES.acm = {
  label: 'ACM arm-handled cargo module', name: 'ACM arm-handled cargo module', handling: 'arm',
  dims: [k(0.18), k(0.14), k(0.24)],                                  // [VERIFY with user] x, y, z (m)
  mass: 2400,                                                         // [ASSUMED] default ACM, kg (rated 3,000 kg)
  trunnion: { r: k(0.006), len: k(0.02), y: k(0.09) },                // s0: 2 x D12 mm trunnions on the +-z faces [VERIFY]
  cgY: k(0.07),                                                       // 20 mm below trunnion (pendulum, s4.3)
  twistMax: 2e4, stand: false,                                        // [ASSUMED] twistMax N*m
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

// LUNAR POWER DOCK, FULL SCALE (review item 6; CHANGES.md s37, docs/SCALING.md, docs/LUNAR_CHARGING.md). Until the review pass the
// dock was modelled at prototype scale (24 V, 100 Ah, 126 W of array) and every watt was multiplied by 150 kWh / 128 Wh = 1172 on the
// way to the rover pack. That made the full-scale dock a 148 kW array and a 2.8 MWh bank, while the masts drawn in the world are the
// prototype masts at 1:K and give about 8 kW. The sim dock is now sized for the full-scale job in real units (V, A, W, Wh); the
// prototype bench dock keeps its own spec below (POWER_DOCK_PROTO) and is not scaled up.
//  - rover pack: 125S LiFePO4, 400 V nominal, 375 Ah = 150 kWh (ROVER.battery); charged CC-CV at 0.14 C = 52 A (about 21 kW, capped at the
//    20 kW charger, ROVER.battery.dockMaxW): 30 -> 90 % in about 4.5-5 h.
//  - array: 2 vertical azimuth-tracking masts (NASA VSAT-style), each a panelW x panelH wing at 1:K of the prototype mast
//    (3.43 x 3.92 m = 13.4 m^2), 30 % triple-junction cells: 2 x 13.4 m^2 x 1361 W/m^2 x 0.30 x 0.80 x 0.96 = 8.4 kW AM0 at normal incidence.
//    A 20 kW rover charge therefore comes mostly from the bank; the bank refills from the 10 % floor in about 28 h of full sun
//    (7 % of a 354 h lunar day).
//  - bank: 250 kWh (125S, 400 V, 625 Ah, about 1.9 t of LFP cells at ~130 Wh/kg) in a heated, bermed vault. [ASSUMED] vault: ~13 m^2 of
//    MLI-wrapped skin, G = 4 sigma T^3 x eps 0.02 x 13 m^2 = 0.9 W/K, electric heaters only (rhuW 0; set 8 W x n for RHU units).
//    Sized in review item 7 (docs/THERMAL_BUDGET.md): 200 kWh left the rover only 1.09x its 354 h survival need; 250 kWh gives 1.5x.
//  - night: avionics 120 W + vault heater 0.9 W/K x 55 K = 50 W = 170 W; x 354 h / 0.97 = 62 kWh = 25 % of the bank.
//    nightReserveSoc 0.40 = 0.10 floor + 0.25 + 0.05 margin. Above it the rover may draw (1 - 0.40) x 250 kWh x 0.97 x 0.85 = 124 kWh
//    through the link: 1.5x its steady-state 354 h survival load (82 kWh, docs/THERMAL_BUDGET.md).
// [ASSUMED] everything in this block except the geometry keys and cellEff: no 20 kW lunar inductive link or 250 kWh lunar bank exists
// to cite; the commercial lunar-qualified wireless-charger reference is the 125 W Astrobotic / WiBotic UFPC (kept in POWER_DOCK_PROTO).
export const PACK = {
  chem: 'LiFePO4 125S', cells: 125, Ah: ROVER.battery.capacityWh / ROVER.battery.nominalV, Vnom: ROVER.battery.nominalV, Wh: ROVER.battery.capacityWh,
  cvCell: 3.65,  // LiFePO4 CV limit, V/cell (typical datasheet)
  R: ROVER.battery.internalR,
};
export const LFP_OCV = [[0, 2.50], [0.05, 3.00], [0.10, 3.20], [0.20, 3.25], [0.50, 3.30], [0.80, 3.33], [0.90, 3.35], [1, 3.40]]; // [ASSUMED] typical LiFePO4 rest OCV per cell
const DOCK_PANEL_W = k(0.42), DOCK_PANEL_H = k(0.48);
export const POWER_DOCK = {
  padT: 44, padV: 22, couplerTol: 3.0,
  arrays: [[60, 10], [70, 30]],                        // [ASSUMED] 2 masts (habitat u, v)
  mastH: k(1.35), panelW: DOCK_PANEL_W, panelH: DOCK_PANEL_H, // [ASSUMED] prototype 1.35 m mast and 0.42 x 0.48 m wing x K (world 11.0 m / 3.43 x 3.92 m)
  bank: { w: k(0.56), d: k(0.34), h: k(0.26) },        // drawn vault (visual); the thermal skin area below is the physical one
  panelArea: +(DOCK_PANEL_W * DOCK_PANEL_H).toFixed(3),// m^2 per mast, full scale
  cellEff: 0.30,                                       // triple-junction AM0 BOL (Spectrolab XTJ-class ~30 %)
  arrayDerate: 0.80,                                   // [ASSUMED] dust 0.93 x cell temp 0.90 x wiring/mismatch 0.97 x ageing 0.98
  mpptEff: 0.96,                                       // [ASSUMED]
  nightSolarW: 130,                                    // [ASSUMED] ~1.5 % of the array: below this the dock is in night mode
  bankV: 400, bankAh: 625, bankWh: 250000,
  bankCells: 125,
  bankR: 0.05,                                         // [ASSUMED] ohm (about 2 % drop at 0.5 C)
  bankEff: 0.97,                                       // [ASSUMED] each way
  bankFloor: 0.10,                                     // [ASSUMED] BMS low-voltage cutoff margin
  nightReserveSoc: 0.40,                               // derived above
  bankMaxDischargeW: 125000,                           // [ASSUMED] 0.5 C BMS limit
  bankMaxChargeA: 312,                                 // [ASSUMED] 0.5 C
  chargeMinBankC: 0, dischargeMinBankC: -20,           // LiFePO4 typical datasheet
  hotelW: 120,                                         // [ASSUMED] BMS + MPPT + controller + beacon + habitat link relay, sleep
  padStandbyW: 30,                                     // [ASSUMED] inductive pilot / foreign-object detection while a rover is on the pad
  vault: { C: 2.2e6, G: 0.9, rhuW: 0, setC: 5, heaterGain: 50, heaterMaxW: 400, sinkDayC: -10, sinkNightC: -50, initC: 10 },
  //   C: [ASSUMED] J/K, ~1.9 t of cells + box x ~1.2 kJ/kg K;  G: [ASSUMED] W/K, MLI box under a regolith berm (4 sigma T^3 x eps 0.02 x ~13 m^2)
  //   rhuW: electric-only; a radioisotope option (8 W GPHS-class units, Lunokhod / Yutu precedent for small ones) would offset G x 55 K
  //   sinks: [ASSUMED] bermed-vault sink temperatures, day / night
  chargeCC_A: 52.5,                                    // 0.14 C of 375 Ah (about 21.6 kW at 412 V, capped at ROVER.battery.dockMaxW = 20 kW)
  linkEff: 0.85,                                       // [ASSUMED] inductive DC-DC at centre
  linkOffsetLoss: 0.25,                                // [ASSUMED] efficiency loss fraction at full couplerTol, quadratic
  linkMaxW: 30000,                                     // [ASSUMED] 20 kW delivered / 0.85 = 23.5 kW in, plus margin
  chargeMinCellC: 0,                                   // no charge below 0 C cell temperature (lithium plating): the pack heaters warm it first
};

// PROTOTYPE BENCH DOCK (not used by the sim; docs/PROTOTYPE_POWER_BUDGET.md, docs/LUNAR_CHARGING.md, the Fusion CAD): 24 V 100 Ah
// bank, 2 x 0.2016 m^2 of cells = 126 W, 1.4 A CC into the 12.8 V 10 Ah (128 Wh) prototype pack, 125 W link limit. Bench units.
export const PROTO_PACK = {
  chem: 'LiFePO4 4S', cells: 4, Ah: 10, Vnom: 12.8, Wh: 128, // public/team/js/rover_bom.js BATTERY; ARM_CARGO_BUILD_PLAN BOM #7
  cvCell: 3.65, R: 0.04,
};
export const POWER_DOCK_PROTO = {
  panelAreaProto: 0.42 * 0.48, cellEff: 0.30, arrayDerate: 0.80, mpptEff: 0.96, nightSolarW: 2,
  bankV: 24, bankAh: 100, bankWh: 2400, bankCells: 8, bankR: 0.02, bankEff: 0.97, bankFloor: 0.10, nightReserveSoc: 0.50,
  bankMaxDischargeW: 1200, bankMaxChargeA: 50, hotelW: 0.8, padStandbyW: 0.5,
  vault: { C: 25000, G: 0.04, rhuW: 1.0, setC: 5, heaterGain: 2, heaterMaxW: 10, sinkDayC: -10, sinkNightC: -50, initC: 10 },
  chargeCC_A: 1.4, linkEff: 0.85, linkOffsetLoss: 0.25, linkMaxW: 125,
};

// [ASSUMED] vault radiators (THERMAL.vault.radArea = 2 x len x h = 3.06 m^2 at K = 8.16; 12 m^2 = 2 x 7.2 x 0.85 m was the K_LEGACY size). Measured on the team body: the belly-tub
// side walls are at z = +-2.23..2.33 m between y 1.6 and 3.4 m, and the 7.35 m deck overhangs them to z = +-3.09 m at
// y 3.4..3.67 m. Each radiator is bolted to its tub wall on two rails, tucked under that overhang (which keeps falling
// ejecta off it but does NOT shade it: at the ~9 deg polar sun the overhang casts only ~0.12 m, so a panel facing the sun is
// almost fully lit; health.js _thermal applies the solar load, docs/changes/B.md) and inboard of the rocker beams (z +-2.95 m) and the fenders (inner edge z +-3.09 m).
// (Earlier versions put them at z = +-4.86 m, sized from frame.halfWid; nothing on the body reaches out there.)
//   len, h m panel size;  y m panel centre height;  z m panel centre off the centreline (|z|, port is -z)
// Legacy-scale metres (above) x KS: the radiators follow the body, which scales with K.
export const RADIATOR = { len: ks(7.2), h: ks(0.85), y: ks(2.95), z: ks(2.36) };

// Pack thermal mass and radiating area (review item 7). The 2.4e6 J/K and 3 m^2 were tuned for the old 420 kWh pack; the pack is now 150 kWh.
// Mass = capacity / system-level specific energy (the old numbers imply 175 Wh/kg: 2.4e6 J/K / 1000 J/kg K = 2400 kg = 420 kWh / 175 Wh/kg).
// Area scales as mass^(2/3) (same pack shape). [ASSUMED] specific energy, cp.
const PACK_WH_PER_KG = 175, PACK_CP = 1000;                   // J/kg K, LFP cells + structure + cold plate (about 1 kJ/kg K)
const PACK_MASS_KG = Math.round(ROVER.battery.capacityWh / PACK_WH_PER_KG);   // 857 kg
const OLD_PACK_MASS_KG = Math.round(420000 / PACK_WH_PER_KG), BATT_AREA_OLD = 3;
// [ASSUMED] AEGIS-V1 thermal control (src/sim/health.js), after the VIPER "warm box" layout: avionics on a
// heat-spreader vault in the belly-tub nose, heat carried by loop heat pipes to two vertical louvered radiators
// hung under the deck edges (port + starboard). Radiators are on the sides, not the top, because the carried
// module (3.0 m dia.) covers most of the deck and would block a zenith view. Battery strings have their own
// louvered radiator. Thermostatic film heaters (zone controllers A and B) hold the set points.
//  - day, avionics ~1.0 kW, sink 230 K terrain / 3 K sky: louvers ~30 % open, vault ~19 C, heaters ~0 W
//  - night or survival (drive off, 60 W in the vault): heaters ~0.15-0.2 kW in all. Measured (Nov 2026, tests/fdir.mjs E
//    procedure): ~154 W survival load -> a 354 h night costs ~55 kWh = ~36 % of the 150 kWh pack, with no solar input. That
//    passes, but NOT "under a quarter" (it was, with the old larger pack): tests/fdir.mjs E asserts <= 25 % (docs/changes/A.md).
export const THERMAL = {
  sigma: 5.670e-8,
  vault: { C: 120000, radArea: +(2 * RADIATOR.len * RADIATOR.h).toFixed(2), emis: 0.85, louverMin: 0.12, louverOpenC: 15, louverFullC: 30, setC: 5, survivalSetC: -20, heaterMaxW: 800, hotC: 45 },
  battery: { C: PACK_MASS_KG * PACK_CP, radArea: +(BATT_AREA_OLD * (PACK_MASS_KG / OLD_PACK_MASS_KG) ** (2 / 3)).toFixed(2), emis: 0.85, louverMin: 0.12, louverOpenC: 25, louverFullC: 35, setC: 5, survivalSetC: -10, heaterMaxW: 1500, hotC: 50 },
  heaterGainWperK: 200,
  sinkDayK: 230, sinkNightK: 100, skyViewFraction: 0.5,
  // louverMin 0.12: closed-louver effective emittance fraction (flight louvers close to ~0.10-0.15 of open; 0.06 was
  // optimistic). leakWperK: parasitic conduction through MLI seams, mounts and harness to the cold structure, per box.
  leakWperK: { vault: 0.25, battery: +(0.4 * (PACK_MASS_KG / OLD_PACK_MASS_KG) ** (2 / 3)).toFixed(3) }, // battery: per-box leak scales with the box area, like its radiator
  // battery internal resistance rises in the cold: R(T) = R0 x (1 + coldRperK x (refC - T)) below refC
  coldRperK: 0.03, coldRrefC: 20,
  // Wheel-end freeze protection (CHANGES.md §17). The hub gearbox lubricant (perfluoropolyether grease) must stay
  // above its glass-transition region or the planetary gearbox seizes on the first drive after the night. A 25 W film
  // heater on each of the six hubs switches on below hubFreezeC at night or in survival and off hubHystK above it.
  // hub: lumped thermal state of one MLI-wrapped wheel end (health.js _thermal). All [ASSUMED].
  //   C       J/K, hub motor + gearbox + knuckle (WHL LRU 290 kg of Al / steel, ~0.4 kJ/kg/K)
  //   radArea m^2 radiating surface;  emis effective emissivity of the MLI-wrapped hub (blanket + open drive face)
  //   motorW  W dissipated while the drive is powered (the hub's sun load is computed in health.js _thermal, not a constant)
  hubHeaterW: 25, hubFreezeC: -65, hubHystK: 5,
  // The drive motor's losses are real heat (hundreds of W per hub when loaded) and there is no air to carry it away.
  // Path: winding (rover.js w.temp, ROVER.motor.thermalC) -> conduction motor.thermalR -> this hub mass -> radiation.
  // The hub rejects through a radiator ring on its outboard face behind a passive thermal switch (paraffin actuator,
  // [ASSUMED]): effective emissivity emis (MLI-like) below switchLoC, rising to emisHot at switchHiC. Cold = insulated
  // (the 25 W freeze heater holds it), hot = radiating. motorW: controller idle dissipation while the drive is powered.
  hub: { C: 120000, radArea: 1.2, emis: 0.05, emisHot: 0.8, switchLoC: 20, switchHiC: 60, motorW: 8, initC: 10 },
};


// [ASSUMED] wheel fenders (CHANGES.md §17, §20). A tyre throws regolith up and back (a "rooster tail", ballistic at
// 1.62 m/s^2 gravity: a 1.2 m/s tread speed lofts it several metres) and sideways in turns, and it coats the radiator and
// louvers (the dust-fouled radiator failure mode in health.js). Each wheel gets an arched fender that rides with the wheel
// on the rocker-bogie, and each radiator an ejecta skirt: a plate hinged to the tub wall along the radiator's lower edge,
// sloping outboard and down, so regolith thrown up from the wheels and rocker beams cannot reach the louvers.
//   clearance m gap tyre to fender;  arcDeg arc covered, centred on the top and biased aft;  width m (tyre width 1.22 m + margin)
//   flapH m trailing flap;  deflector: len m, depth m (hinge to outer edge), angleDeg below horizontal, y m hinge height
export const FENDER = {
  clearance: ks(0.28), arcDeg: 150, aftBiasDeg: 15, width: ks(1.5), thickness: 0.03, flapH: ks(0.45),  // legacy metres x KS
  deflector: { len: ks(7.2), depth: ks(0.45), angleDeg: 40, y: ks(2.5) },
};

// [ASSUMED] emergency keep-alive solar blanket (Lunokhod / Yutu-2 lesson: a rover with no on-board generation
// cannot recover from a dead battery away from the dock). A 2 x 6 m flexible array on a telescoping mast, deployed
// only while parked. Vertical, sun-tracking in azimuth. The mast must stand outside the carried module's plan
// footprint (radius 3.98 m), so its canister sits at the end of an outrigger arm from the port-aft deck corner.
// armY is the arm centre height: high enough to clear the rear fender at full kneel plus full bogie lift (~4.15 m).
export const KEEPALIVE = { area: 12, efficiency: 0.30, deployS: 90, mastH: 6.5, massKg: 55, corner: { x: ks(-4.6), z: ks(-4.75) }, armY: ks(4.35) }; // geometry: legacy metres x KS

// [ASSUMED] random-failure generator: each catalogue MTBF is divided by rateX (realistic = 1, so a failure
// every few thousand hours; demo = one every few minutes of sim time). Off by default.
export const FAILURES = {
  presets: { realistic: 1, '100x': 100, '1000x': 1000, demo: 20000 },
  defaultPreset: 'demo', maxActiveRandom: 3, seed: 4242, bitPeriodS: 600,
  survivalEnterSoc: 0.06, survivalExitSoc: 0.30,
};

// [ASSUMED] mission-executive tuning (autonomy.js). All sim choices, none traced to a source.
//   preSurveyRoute   drive the empty rover to the port and back before the first pick-up (surveyRoute). ON since the
//                    review pass (2026-11-21): the loaded rover's tip margin is only ~26-27 deg in manoeuvres (static ~28 deg),
//                    so autonomy stops it at 14 deg tilt; a loaded trip must therefore only use a route driven empty first
//                    (empty limit 17 deg). Turn it off again only when stability.js reports >= 35 deg on the design slope
//                    (review item 1, branch review/04-stability).
//   crawlApproachM / crawlDockM  m from the cargo / the port inside which the drive is held to the 120 rpm crawl cap
//                    (ROVER.motor.protoCrawlRpm): cargo approach + kneel + lift, and habitat vestibule entry + soft capture
//   standExitM       m of straight departure from the loading stand along the module axis, heading held (autonomy.js _deliver); the
//                    transit arc (planner fillet, R >= 18 m loaded) starts only after it
//   standExitSpeed   m/s of that departure;  standExitKap  1/m, the most curvature the heading hold may use inside the stand
//   standExitDir     +1 forward (default), -1 reverse. Every stand on the pad points its module at the habitat, so forward is the
//                    direction of travel and the far end of the stand is open. Reverse was measured (tests/scratch/ab_departure.mjs):
//                    the loaded rover, which cannot point-turn, is left behind its own stand facing it, and it did not get 30 m away
//                    in 25 min (4 boxed-in recoveries). Forward: 200 m clear in 3.4 min, 0 N stand contact, 2 cm lateral drift.
export const MISSION = { preSurveyRoute: true, crawlApproachM: 15, crawlDockM: 20, standExitM: 12, standExitSpeed: 0.5, standExitKap: 0.05, standExitDir: 1 };

// [ASSUMED] navigation smoothing (autonomy.js navTo / replan): all sim tuning values.
//   headingDeadbandDeg  inside +-this heading error to the lookahead point, on a gentle path (|k| < 0.02 /m), wz is commanded 0
//                       (released again past 4 deg: hysteresis); lookaheadMin / lookaheadSpeedS  lookahead = max(min, v x s) m
//   yawDampErrDeg / yawDampRate / yawDampGain  inside this heading error on a straight stretch, while still rotating TOWARD the target
//                       faster than yawDampRate rad/s (an overshoot in the making), the commanded wz is cut by gain
//   maxYawAccel         rad/s^2 slew limit on the navigation's yaw command (the rover's own limiter, ROVER.accel.yaw, still applies)
//   unknownCoastSpeed / unknownCoastM / flatDeg  on ground tilted < flatDeg, unmapped cells ahead (LiDAR still integrating) hold at
//                       least this speed for up to this many metres instead of the 0.3 m/s creep
//   dropoutHoldS        a LiDAR dropout derates the leg only once it has lasted this long (transient dropouts are ignored)
//   inflateEmpty / inflateLoaded  planner hard inflation, set by navTo from the payload state
//   replanBlockM / replanCostGain / commitM  routine replans switch the path only for a lethal obstacle within replanBlockM
//                       path points, or when the fresh plan is cheaper by replanCostGain (30 %); a switched path is held for commitM metres
export const NAV = {
  headingDeadbandDeg: 3, lookaheadMin: 10, lookaheadSpeedS: 3.5, yawDampErrDeg: 5, yawDampRate: 0.03, yawDampGain: 0.5, maxYawAccel: 0.1,
  unknownCoastSpeed: 0.5, unknownCoastM: 12, flatDeg: 5, dropoutHoldS: 2.5, replanBlockM: 25, replanCostGain: 0.3, commitM: 15,
  inflateEmpty: ks(5.8), inflateLoaded: 6.5, // m, planner hard obstacle inflation (AEGIS-V1 half width 2.45 m + margin, legacy 5.8 x KS; loaded keeps 6.5: the 12.19 m module did not shrink); loaded = 6.5 for a swinging 23 t module
  straightenRoutes: true, // straighten stored routes over mapped ground (planner.js straighten)
  noSteerRatio: 0.1, // steering-authority check: 6 s mean yaw rate below this share of the command for 15 s = not steering (healthy 0.4-0.55, one wheel free-rolling 0.15-0.25)
  upsetDeg: 25, // [ASSUMED] pitch or roll beyond this = vehicle upset: navigation stops, no autonomous retry
  // Slip-aware navigation (CHANGES.md §32). A skid-steer rover on regolith achieves ~0.5 of the commanded yaw rate (measured 0.51 in turns on the 6WD rover; the 4WD rover with free-rolling middle idlers differs, re-measure before enabling).
  slip: {
    enabled: typeof process !== 'undefined' && !!process.env && process.env.SLIPNAV === '1', // [ASSUMED] master switch, OFF by default until it passes faults_m10 (env SLIPNAV=1 = on, for A/B benches)
    etaInit: 0.6, etaMin: 0.3, etaTau: 4, etaPlanMin: 0.6, etaFFMin: 0.75, // [ASSUMED] yaw-efficiency estimate: start, floor (a free-rolling wheel reads ~0.2), low-pass s
    trackKp: 1.5, trackKi: 0.6, trackLimK: 0.04, // [ASSUMED] gyro PI on the yaw-rate error; authority 0.04 /m x speed (as cmdCurv)
  },
  legRetries: 2, // [ASSUMED] autonomous recover-and-retry attempts per navigation leg before the operator is asked (autonomy.js navTo)
};

// [ASSUMED] integrator step (120 Hz) and catch-up limit; collision-group bitmasks are code conventions.
//   warpBudgetMaxMs / warpMaxSteps: at time scale s the per-frame physics budget is min(budget x min(s, 2.5), warpBudgetMaxMs) and at most
//   min(maxSubSteps x s, warpMaxSteps) steps are taken per frame (sim.js advance).
//   railsMinScale: from this time scale up, long cruise legs whose next 30 m are mapped and clear run on the kinematic rails
//   (sim.js warpStep); pick-up, docking, turns in tight ground and unmapped ground always run the full physics.
//   posGuardMaxSpeed: m/s, the float64 position guard (sim.js _precisionPost) corrects the solver's 0.49 mm position grid up to this
//   speed: it covers every precision manoeuvre (line-up, drive-under, stand exit, port approach). Was 0.1.
export const PHYS = { dt: 1 / 120, maxSubSteps: 40, warpBudgetMaxMs: 50, warpMaxSteps: 240, railsMinScale: 5, posGuardMaxSpeed: 0.8 };

export const GROUPS = { TERRAIN: 1, ROCK: 2, STRUCT: 4, ROVER: 8, CARGO: 16 };
export function groups(member, filter) { return ((member & 0xffff) << 16) | (filter & 0xffff); }
