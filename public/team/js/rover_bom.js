/**
 * LLASO-P2-XPORT-2026 rover bill of materials & dynamic thermal subsystem.
 * High-sensitivity aggressive thermal physics responding dynamically to load, slip, and fan RPM.
 */
(function (global) {
  // Earth-demo planform from the bay-balance board: 4 ft long × 2 ft wide.
  // Front avionics bay; 5 Ah hot-swap pack + ballast plate under the aft deck for rear-axle grip. Cargo bay is aft.
  // v6 planform: rocker-bogie, skid steer; body, wheels, scanners and stowed arm fit a 24 in cube (cradle rails extend the length).
  var ROVER_FOOTPRINT = {
    lengthFt: 2,
    widthFt: 2,
    lengthM: 0.6096,
    widthM: 0.6096,
    lengthIn: 24,
    widthIn: 24,
    refLengthM: 0.6096,
    refWidthM: 0.6096
  };

  // Masses: vendor listings where a part is bought (cited in review/docs/rover-v4-design.md), estimates
  // (estimate: true) for printed / machined parts. pos = centre of mass in the rover frame (x fwd, y up from
  // ground, z port), m. Six identical items on symmetric corners sit at their common centroid.
  var PARTS = [
    { id: 'chassis', name: 'Belly tub + deck (7075-T6 / 6061-T6 Al extrusion & PEEK-CF30 panels, cryo-rated)', count: 1, spare: 0, massKg: 1.55, heatW: 0, size: [0.58, 0.14, 0.52], pos: [0.0, 0.160, 0], estimate: true, onVehicle: true },
    { id: 'wheel', name: '130 x 75 mm lunar wheel (NiTi superelastic mesh hoop, 20 Ti-6Al-4V grousers, 7075-T6 hub)', count: 6, spare: 1, massKg: 0.135, heatW: 0, size: [0.13, 0.075, 0.13], pos: [0, 0.065, 0], estimate: true, onVehicle: true },
    // v6 rocker-bogie (js/rocker_bogie.js): machined 7075-T6 / Ti-6Al-4V lunar links
    { id: 'rocker', name: 'Rocker link, machined 7075-T6 Al, bent (front wheel + bogie pivot)', count: 2, spare: 1, massKg: 0.115, heatW: 0, size: [0.33, 0.018, 0.012], pos: [0.03, 0.150, 0], estimate: true, onVehicle: true },
    { id: 'bogie', name: 'Bogie link, machined 7075-T6 Al (middle + rear wheel)', count: 2, spare: 1, massKg: 0.062, heatW: 0, size: [0.255, 0.018, 0.012], pos: [-0.12, 0.105, 0], estimate: true, onVehicle: true },
    { id: 'pivot', name: 'Pivot pins, Ti-6Al-4V shafts, MoS2 dry-lubed bearings and axle stubs', count: 1, spare: 0, massKg: 0.12, heatW: 0, size: [0.05, 0.05, 0.40], pos: [-0.03, 0.120, 0], estimate: true, onVehicle: true },
    { id: 'diff', name: 'Bevel-gear differential + transverse pivot shaft (body pitch = mean of the rockers)', count: 1, spare: 0, massKg: 0.16, heatW: 0, size: [0.03, 0.03, 0.36], pos: [0, 0.194, 0], estimate: true, onVehicle: true },
    { id: 'motor', name: 'Hermetically sealed brushless hub actuator x4 (FL/FR/RL/RR driven; the two middle wheels are passive idlers on 608-2RS bearings) (resolver + harmonic drive, space-rated)', count: 4, spare: 0, massKg: 0.210, heatW: 0, size: [0.074, 0.037, 0.037], pos: [0, 0.065, 0], onVehicle: true },
    { id: 'battery', name: '12.8V 10Ah MLI-insulated pack w/ kapton film heaters + belly blind-mate bus', count: 1, spare: 1, massKg: 1.65, heatW: 0.45, size: [0.155, 0.100, 0.100], pos: [-0.056, 0.130, 0.000], onVehicle: true, wh: 128, dimsMm: [155, 100, 100] },
    { id: 'imu', name: 'IMU tip sensor (BNO085)', count: 1, spare: 0, massKg: 0.003, heatW: 0.05, size: [0.025, 0.004, 0.020], pos: [0.0, 0.17, 0.0], onVehicle: true },
    { id: 'pi5', name: 'Raspberry Pi 5 (4 GB) + Cu heat-pipe cold plate (vacuum conductive strap to OSR)', count: 1, spare: 0, massKg: 0.075, heatW: 7.0, size: [0.085, 0.020, 0.056], piBoardSize: [0.085, 0.006, 0.056], pos: [0.165, 0.155, 0.07], onVehicle: true, dimsMm: [85, 56, 20] },
    { id: 'uno', name: 'Arduino Mega 2560 (6 drive + 4 clamp channels, encoders)', count: 1, spare: 0, massKg: 0.037, heatW: 0.5, size: [0.101, 0.015, 0.053], pos: [0.165, 0.155, -0.07], onVehicle: true },
    { id: 'buck', name: 'YRDZXG 12V to 5V 5A buck', count: 1, spare: 0, massKg: 0.055, heatW: 1.5, size: [0.066, 0.018, 0.036], pos: [0.11, 0.15, 0.105], onVehicle: true },
    { id: 'driver', name: 'Space-derated 3-phase FET bridge driver', count: 3, spare: 0, massKg: 0.055, heatW: 3.0, size: [0.095, 0.022, 0.065], pos: [0.205, 0.095, 0.0], onVehicle: true },
    { id: 'fan', name: 'Zenith Z-93 OSR radiator panel + bimetal louver vane + Cu heat pipe (vacuum-rated, 0 W fan)', count: 1, spare: 0, massKg: 0.045, heatW: 0.0, size: [0.155, 0.012, 0.180], pos: [0.18, 0.285, 0], onVehicle: true },
    { id: 'camera', name: 'Arducam OV5647 (stereo pair on the pod nose, Au/SiO2 shield + ITO heater)', count: 2, spare: 0, massKg: 0.006, heatW: 0.25, size: [0.025, 0.024, 0.017], pos: [0.270, 0.275, 0], onVehicle: true },
    { id: 'lidar', name: 'YDLIDAR X2L (Phi60.5 x 50 mm), front deck corners, dry-lubed & ITO-heated sapphire window', count: 2, spare: 0, massKg: 0.19, heatW: 2.0, size: [0.0605, 0.050, 0.0605], pos: [0, 0.262, 0], onVehicle: true },   // front port + front starboard
    { id: 'ring', name: 'Ring clamp (120 deg Al-7075 saddle, dual 60 deg jaws, +/-3.4 mm delta-CTE slide, Vespel SP-21 flexure pads)', count: 2, spare: 0, massKg: 0.44, heatW: 0, size: [0.05, 0.42, 0.56], pos: [0, 0.31, 0], estimate: true, onVehicle: true },
    { id: 'lift', name: 'Roller-screw jacking suspension actuator block + welded 321SS bellows (vacuum-rated, no air bag)', count: 1, spare: 0, massKg: 0.09, heatW: 0, size: [0.06, 0.03, 0.04], pos: [-0.05, 0.15, 0], estimate: true, onVehicle: true },
    { id: 'clampdrv', name: 'Cytron MD13S motor driver (jaws, 4 channels)', count: 2, spare: 0, massKg: 0.03, heatW: 0.4, size: [0.062, 0.015, 0.045], pos: [0.11, 0.15, -0.105], estimate: true, onVehicle: true },
    // two-arm ACM handling (docs/ARM_CARGO_BUILD_PLAN.md s2 / s4.2; masses per arm x2, arm plane at z = +-0.175, so the centroid sits on z = 0)
    { id: 'arm_servo_big', name: '60 kg-cm-class HV servo (shoulder + elbow, 7.4 V) with cradle', count: 4, spare: 1, massKg: 0.18, heatW: 0, size: [0.065, 0.030, 0.048], pos: [0.15, 0.275, 0], estimate: true, onVehicle: true },
    { id: 'arm_servo_wrist', name: 'MG996R servo (wrist x2, deck latch x1) + bracket', count: 3, spare: 1, massKg: 0.07, heatW: 0, size: [0.040, 0.020, 0.043], pos: [0.30, 0.30, 0], estimate: true, onVehicle: true },
    { id: 'arm_link1', name: 'Arm link 1, twin 5 x 28 mm PETG plates', count: 2, spare: 0, massKg: 0.06, heatW: 0, size: [0.20, 0.028, 0.012], pos: [0.20, 0.32, 0], estimate: true, onVehicle: true },
    { id: 'arm_link2', name: 'Arm link 2, twin 5 x 28 mm PETG plates', count: 2, spare: 0, massKg: 0.05, heatW: 0, size: [0.20, 0.028, 0.012], pos: [0.30, 0.40, 0], estimate: true, onVehicle: true },
    { id: 'arm_hook', name: 'V-seat trunnion hook (PA-CF) + MG90S latch gate servo', count: 2, spare: 1, massKg: 0.04, heatW: 0, size: [0.050, 0.040, 0.030], pos: [0.35, 0.42, 0], estimate: true, onVehicle: true },
    { id: 'deck_pins', name: 'Deck tapered locating pins (round + diamond), 6061, at DECK.pinSpan', count: 2, spare: 0, massKg: 0.01, heatW: 0, size: [0.012, 0.025, 0.012], pos: [-0.10, 0.31, 0], estimate: true, onVehicle: true },
    { id: 'deck_latch', name: 'Rotary deck latch (two claws on one shaft, MG996R) + keeper bar', count: 1, spare: 0, massKg: 0.04, heatW: 0, size: [0.060, 0.030, 0.050], pos: [-0.18, 0.31, 0], estimate: true, onVehicle: true },
    { id: 'avionics', name: 'Avionics warm vault (MLI + OSR lid; belly shuttered dock charge receptacle at x 0, y 0.058)', count: 1, spare: 0, massKg: 0.16, heatW: 0, size: [0.155, 0.060, 0.20], pos: [0.19, 0.26, 0], estimate: true, onVehicle: true },
    { id: 'harness', name: 'Kapton/FEP harness, Ti-6Al-4V fasteners & carbon-bronze ESD grounding sweepers', count: 1, spare: 0, massKg: 0.21, heatW: 0, size: [0.10, 0.02, 0.05], pos: [0.05, 0.15, 0], estimate: true, onVehicle: true },
    { id: 'solar', name: 'HQST 100W 12V panel', count: 1, spare: 0, massKg: 6.5, heatW: 0, size: [0.808, 0.030, 0.698], pos: null, onVehicle: false, powerW: 100 },
    { id: 'kapton_peek_kit', name: 'PEEK-CF30 / Kapton MLI repair kit (flight cryo-safe)', count: 1, spare: 0, massKg: 0.8, heatW: 0, size: [0.20, 0.07, 0.20], pos: null, onVehicle: false, estimate: true }
  ];

  var BAY_HEAT_IDS = { pi5: 1, uno: 1, buck: 1, driver: 1, fan: 1 };

  function byId(id) {
    for (var i = 0; i < PARTS.length; i++) if (PARTS[i].id === id) return PARTS[i];
    return null;
  }

  function dX(legacyX) {
    return legacyX * (ROVER_FOOTPRINT.lengthM / ROVER_FOOTPRINT.refLengthM);
  }

  function dZ(legacyZ) {
    return legacyZ * (ROVER_FOOTPRINT.widthM / ROVER_FOOTPRINT.refWidthM);
  }

  // v5: the curb mass is the bottom-up sum of the BOM (it used to be scaled to a 15 lb target). With the 10 Ah pack
  // (1.6 kg), the 40 ft module cradle, four twistlocks and the tug, the honest sum is above 15 lb; scaling every part
  // down to hit 15 lb would shrink the battery and the cradle on paper only.
  var LB_KG = 0.45359237;
  function rawCurbKg() {
    var m = 0;
    for (var i = 0; i < PARTS.length; i++) {
      var p = PARTS[i];
      if (p.onVehicle) m += p.massKg * p.count;
    }
    return m;
  }
  var MASS_SCALE = 1;                                   // bottom-up, unscaled

  /** Scaled mass of one BOM line (all units of it), kg. */
  function partKg(p) {
    return p.massKg * p.count * MASS_SCALE;
  }

  function curbKg() {
    return rawCurbKg();
  }

  /** Where the 15 lb sits: every on-vehicle BOM line with its scaled mass, rover-local position and share. */
  function massBreakdown() {
    var rows = [];
    var total = curbKg();
    for (var i = 0; i < PARTS.length; i++) {
      var p = PARTS[i];
      if (!p.onVehicle) continue;
      var kg = partKg(p);
      rows.push({ id: p.id, name: p.name, kg: kg, lb: kg / LB_KG, share: kg / total, pos: p.pos ? p.pos.slice() : null });
    }
    rows.sort(function (a, b) { return b.kg - a.kg; });
    return rows;
  }

  function com() {
    var m = 0, x = 0, y = 0, z = 0;
    for (var i = 0; i < PARTS.length; i++) {
      var p = PARTS[i];
      if (!p.onVehicle || !p.pos) continue;
      var w = partKg(p);
      m += w;
      x += p.pos[0] * w;
      y += p.pos[1] * w;
      z += p.pos[2] * w;
    }
    if (m <= 0) return { x: 0, y: 0, z: 0, kg: 0 };
    return { x: x / m, y: y / m, z: z / m, kg: m };
  }

  var G_LUNAR = 1.625;
  var G_EARTH = 9.806;

  /** Bogie axle X (m): front, mid, rear — matches fusion_rover axle tubes. */
  function axleXM() {
    return [0.24, 0, -0.24];          // wheel rows (rocker_bogie.js GEO.xF / xM / xR)
  }

  /**
   * Static pitch-plane load split (front vs rear bogie pairs) for grip budgeting.
   * Optional cargo mass at cargoXM (aft bay center by default).
   */
  function tractionSnapshot(opts) {
    opts = opts || {};
    var grav = opts.g != null ? opts.g : G_LUNAR;
    var cargoKg = Math.max(0, opts.cargoKg || 0);
    var cargoX = opts.cargoX != null ? opts.cargoX : -0.134;      // midway between the two canister cradles
    var m0 = curbKg();
    var c0 = com();
    var massKg = m0 + cargoKg;
    var cx = cargoKg > 0 ? (c0.x * m0 + cargoX * cargoKg) / massKg : c0.x;
    var ax = axleXM();
    var xF = ax[0];
    var xR = ax[2];
    var span = xF - xR;
    var rearFrac = span > 0.01 ? Math.max(0.08, Math.min(0.92, (xF - cx) / span)) : 0.5;
    var frontFrac = 1 - rearFrac;
    var weightN = massKg * grav;
    return {
      g: grav,
      massKg: +massKg.toFixed(3),
      comX: +cx.toFixed(4),
      comY: +c0.y.toFixed(4),
      rearBogieN: +(weightN * rearFrac).toFixed(2),
      frontBogieN: +(weightN * frontFrac).toFixed(2),
      rearWheelN: +(weightN * rearFrac / 4).toFixed(2),
      frontWheelN: +(weightN * frontFrac / 4).toFixed(2),
      rearFrac: +rearFrac.toFixed(3),
      cargoKg: cargoKg
    };
  }

  function bayHeatW() {
    var w = 0;
    for (var i = 0; i < PARTS.length; i++) {
      var p = PARTS[i];
      if (BAY_HEAT_IDS[p.id]) w += p.heatW * p.count;
    }
    return w;
  }

  // 12.8 V 10 Ah LiFePO4 hot-swap pack (128 Wh); a second pack charges in the dock.
  var BATTERY = {
    ah: 10,
    vNom: 12.8,
    wh: 128,
    socReserve: 0.18,
    name: '12.8V 10Ah LiFePO4 hot-swap pack (x2: rover + dock spare)'
  };

  var powerModeId = 'balance';   // default (decided 2026-10-28): lowest-risk mode that still saves energy
  var powerEcoMode = true;

  /** Four energy profiles for the 10 Ah / 128 Wh pack (LLASO cargo rover); whPerRoundTrip is per full 4 km sortie. */
  var POWER_MODES = {
    range: {
      id: 'range',
      label: 'Range',
      hint: 'Max trips — slow cruise, sensors sleep when idle',
      cruiseRpmCap: 58,
      transitSpeedCap: 0.48,
      fanNominalRpm: 2200,
      fanWarmRpm: 3200,
      fanHotRpm: 4500,
      lidarMovingW: 1.6,
      lidarIdleW: 0.22,
      lidarSpinMoving: 10,
      lidarSpinIdle: 2,
      camActiveW: 0.55,
      camSleepW: 0.08,
      piBase: 4.0,
      piTransitTrim: 0.45,
      driverScale: 5.2,
      motorIdle: 0.34,
      motorLoadScale: 0.88,
      armW: 8.0,
      slopeScale: 11,
      slipDriverAdd: 2.0,
      whPerRoundTrip: 51.0 // Wh per full 2 km + 2 km sortie with the 11.6 kg module, lunar g (tools/energy_runtime_sim.js)
    },
    eco: {
      id: 'eco',
      label: 'Eco',
      hint: 'Default logistics — balanced savings',
      cruiseRpmCap: 72,
      transitSpeedCap: 0.62,
      fanNominalRpm: 2800,
      fanWarmRpm: 3600,
      fanHotRpm: 4500,
      lidarMovingW: 2.0,
      lidarIdleW: 0.35,
      lidarSpinMoving: 12,
      lidarSpinIdle: 4,
      camActiveW: 0.65,
      camSleepW: 0.12,
      piBase: 4.2,
      piTransitTrim: 0.35,
      driverScale: 5.8,
      motorIdle: 0.38,
      motorLoadScale: 1.0,
      armW: 8.5,
      slopeScale: 13,
      slipDriverAdd: 2.5,
      whPerRoundTrip: 44.0 // Wh per full 2 km + 2 km sortie with the 11.6 kg module, lunar g (tools/energy_runtime_sim.js)
    },
    balance: {
      id: 'balance',
      label: 'Balance',
      hint: 'Faster legs — moderate sensor + fan use',
      cruiseRpmCap: 88,
      transitSpeedCap: 0.78,
      fanNominalRpm: 3600,
      fanWarmRpm: 4200,
      fanHotRpm: 5500,
      lidarMovingW: 2.2,
      lidarIdleW: 0.85,
      lidarSpinMoving: 12,
      lidarSpinIdle: 8,
      camActiveW: 0.72,
      camSleepW: 0.28,
      piBase: 4.6,
      piTransitTrim: 0.15,
      driverScale: 6.2,
      motorIdle: 0.4,
      motorLoadScale: 1.08,
      armW: 9.5,
      slopeScale: 14.5,
      slipDriverAdd: 3.2,
      whPerRoundTrip: 39.2 // Wh per full 2 km + 2 km sortie with the 11.6 kg module, lunar g (tools/energy_runtime_sim.js)
    },
    boost: {
      id: 'boost',
      label: 'Boost',
      hint: 'Full torque & sensors — shortest time',
      cruiseRpmCap: 100,
      transitSpeedCap: 1.0,
      fanNominalRpm: 4500,
      fanWarmRpm: 5500,
      fanHotRpm: 7000,
      lidarMovingW: 2.4,
      lidarIdleW: 1.2,
      lidarSpinMoving: 12,
      lidarSpinIdle: 12,
      camActiveW: 0.8,
      camSleepW: 0.4,
      piBase: 4.8,
      piTransitTrim: 0,
      driverScale: 6.5,
      motorIdle: 0.42,
      motorLoadScale: 1.18,
      armW: 11.0,
      slopeScale: 16,
      slipDriverAdd: 4.0,
      whPerRoundTrip: 34.2 // Wh per full 2 km + 2 km sortie with the 11.6 kg module, lunar g (tools/energy_runtime_sim.js)
    }
  };

  function getPowerMode() {
    return POWER_MODES[powerModeId] || POWER_MODES.eco;
  }

  function setPowerMode(id) {
    if (!POWER_MODES[id]) id = 'eco';
    powerModeId = id;
    powerEcoMode = id !== 'boost';
    if (global.NASA_RoverBOM) {
      global.NASA_RoverBOM.powerModeId = powerModeId;
      global.NASA_RoverBOM.powerEcoMode = powerEcoMode;
    }
    return getPowerMode();
  }

  function setPowerEcoMode(on) {
    return setPowerMode(on ? 'eco' : 'boost');
  }

  /** Fan RPM from active energy mode + thermal status. */
  function ecoFanTargetRpm(thermalState) {
    thermalState = thermalState || thermal;
    var m = getPowerMode();
    var st = thermalState.status || 'NOMINAL';
    if (st === 'THROTTLING' || st === 'HOT') return m.fanHotRpm;
    if (st === 'WARM') return m.fanWarmRpm;
    return m.fanNominalRpm;
  }

  /**
   * Electrical draw (W) from the on-rover BOM + drive state.
   * Motors: Greartisan 12V ×6 via Cytron MDD10A; avionics on YRDZXG 5 V buck.
   */
  function computePowerDraw(state) {
    state = state || {};
    var mode = state.powerModeId && POWER_MODES[state.powerModeId]
      ? POWER_MODES[state.powerModeId]
      : getPowerMode();
    var throttle = Math.abs(state.throttle || 0);
    var vel = Math.abs(state.vel || 0);
    var massRatio = Math.max(1, state.massRatio || 1);
    var fanFrac = state.fanRpmFrac != null ? state.fanRpmFrac : (thermal.fanRpm / thermal.fanNominalRpm);
    var moving = vel > 0.05 || throttle > 0.08;
    var transit = !!state.logisticsTransit;

    // Preferred path: the subsystem power model (js/power_model.js), which returns only watts actually drawn at the
    // live operating point. The legacy block below is the fallback when that module is not loaded.
    if (global.NASA_PowerModel) {
      var pmState = {};
      for (var pk in state) pmState[pk] = state[pk];
      if (pmState.massKg == null) pmState.massKg = 6.80 * massRatio;
      var pm = global.NASA_PowerModel.draw(pmState, mode);
      return { totalW: pm.totalW, powerModeId: mode.id, eco: mode.id !== 'boost', battery: BATTERY, parts: pm.parts, breakdown: pm.breakdown, drive: pm.drive };
    }

    var pi = byId('pi5');
    var uno = byId('uno');
    var cam = byId('camera');
    var fan = byId('fan');
    var buck = byId('buck');
    var driver = byId('driver');
    var motor = byId('motor');
    var bat = byId('battery');

    var wPi = mode.piBase + (state.neuralOn ? 3.0 : 0.45) + (state.visionActive ? 1.4 : 0.2);
    if (transit && !state.neuralOn) wPi -= mode.piTransitTrim;
    var wUno = (uno && uno.heatW) ? uno.heatW + 0.05 : 0.45;
    var wCam = state.visionActive || state.armMoving ? mode.camActiveW : mode.camSleepW;
    var wLidar = (moving || state.navActive ? mode.lidarMovingW : mode.lidarIdleW) * byId('lidar').count;   // per scanner, two fitted
    var wFans = 0.0; // vacuum conductive heat-pipe + passive bimetal OSR louver (replaces 3007 fans)
    var wBuck = 0.85 + wPi * 0.13;
    var wBms = (bat && bat.heatW) ? bat.heatW + 0.15 : 0.45;
    var avionicsW = wPi + wUno + wCam + wLidar + wFans + wBuck + wBms;

    // Drive: physical DC-motor model (js/drive_power.js). Grade and rolling resistance come from the force balance,
    // so there is no separate slope term. Falls back to the legacy heuristic only if the module is not loaded.
    var DP = global.NASA_DrivePower;
    var massKg = state.massKg != null ? state.massKg : 6.80 * massRatio;
    var dp = DP ? DP.draw({ massKg: massKg, vel: state.vel || 0, accel: state.accel || 0, slopeSin: state.slopeSin || 0 }) : null;

    var wDriver = dp ? dp.driverW : (0.75 + (driver ? driver.heatW * 0.35 : 1.0) + mode.driverScale * Math.pow(throttle, 1.65));
    if (state.slip) wDriver += mode.slipDriverAdd;

    var perMotorLoad = (2.2 * throttle + 5.2 * throttle * Math.min(1, vel / 0.52)) * mode.motorLoadScale;
    var wMotors = dp ? dp.motorsW : (motor ? motor.count : 6) * (mode.motorIdle + perMotorLoad) * Math.pow(massRatio, 0.8);
    if (!dp && !moving) wMotors = (motor ? motor.count : 6) * mode.motorIdle;

    var wArm = state.armMoving ? mode.armW : 0.35;
    var slopeW = dp ? 0 : Math.abs(state.slopeSin || 0) * mode.slopeScale * massRatio;

    var totalW = avionicsW + wDriver + wMotors + wArm + slopeW;
    return {
      totalW: totalW,
      powerModeId: mode.id,
      eco: mode.id !== 'boost',
      battery: BATTERY,
      breakdown: {
        pi: +wPi.toFixed(2),
        uno: +wUno.toFixed(2),
        cameras: +wCam.toFixed(2),
        lidar: +wLidar.toFixed(2),
        fans: +wFans.toFixed(2),
        buck: +wBuck.toFixed(2),
        bms: +wBms.toFixed(2),
        avionics: +avionicsW.toFixed(2),
        driver: +wDriver.toFixed(2),
        motors: +wMotors.toFixed(2),
        arm: +wArm.toFixed(2),
        slope: +slopeW.toFixed(2)
      }
    };
  }

  function benchmarkPowerScenarios() {
    var scenarios = [
      { name: 'idle_hangar', state: { throttle: 0, vel: 0, massRatio: 1 } },
      { name: 'idle_eco', state: { throttle: 0, vel: 0, massRatio: 1 }, ecoOnly: true },
      { name: 'transit_eco', state: { throttle: 0.55, vel: 0.26, massRatio: 1.06, logisticsTransit: true, navActive: true } },
      { name: 'transit_full', state: { throttle: 0.72, vel: 0.38, massRatio: 1.08, navActive: true } },
      { name: 'arm_pick', state: { throttle: 0.15, vel: 0.05, massRatio: 1.04, armMoving: true, visionActive: true } },
      { name: 'slip_recovery', state: { throttle: 0.85, vel: 0.12, massRatio: 1.1, slip: true } }
    ];
    var rows = [];
    for (var i = 0; i < scenarios.length; i++) {
      var sc = scenarios[i];
      var prev = powerModeId;
      var off = sc.ecoOnly ? null : (setPowerMode('boost'), computePowerDraw(sc.state));
      setPowerMode('eco');
      var on = computePowerDraw(sc.state);
      setPowerMode(prev);
      rows.push({
        name: sc.name,
        wattsFull: off ? +off.totalW.toFixed(2) : null,
        wattsEco: +on.totalW.toFixed(2),
        savedW: off ? +(off.totalW - on.totalW).toFixed(2) : 0,
        ecoBreakdown: on.breakdown
      });
    }
    var modeSummary = [];
    var ids = ['range', 'eco', 'balance', 'boost'];
    for (var mi = 0; mi < ids.length; mi++) {
      var mid = ids[mi];
      setPowerMode(mid);
      var m = getPowerMode();
      var cruise = computePowerDraw({
        throttle: 0.55,
        vel: m.transitSpeedCap * 0.42,
        massRatio: 1.05,
        logisticsTransit: true,
        navActive: true
      });
      modeSummary.push({
        id: mid,
        label: m.label,
        cruiseW: +cruise.totalW.toFixed(2),
        whPerRoundTrip: m.whPerRoundTrip,
        tripsAt90: estimateRoundTrips(90, m.whPerRoundTrip).trips
      });
    }
    setPowerMode('eco');
    return {
      scenarios: rows,
      modes: modeSummary,
      modelWhPerRoundTrip: POWER_MODES.eco.whPerRoundTrip,
      tripsAt90pctEco: estimateRoundTrips(90, POWER_MODES.eco.whPerRoundTrip).trips
    };
  }

  function whUsableFromSoc(socPct, reservePct) {
    var cap = BATTERY.wh;
    var reserve = reservePct != null ? reservePct : (BATTERY.socReserve * 100);
    var floor = cap * (reserve / 100);
    var avail = cap * (Math.max(0, Math.min(100, socPct)) / 100) - floor;
    return Math.max(0, avail);
  }

  function estimateRoundTrips(socStartPct, whPerRoundTrip) {
    var usable = whUsableFromSoc(socStartPct);
    if (!whPerRoundTrip || whPerRoundTrip <= 0) return { usableWh: usable, trips: 0 };
    return { usableWh: +usable.toFixed(1), trips: Math.floor(usable / whPerRoundTrip) };
  }

  // =========================================================================
  // HIGH-SENSITIVITY AGGRESSIVE THERMAL PHYSICS ENGINE
  // =========================================================================
  // Radiative + conductive vacuum thermal model (lunar sink 95 K night / 245 K polar sun, epsilon = 0.88 OSR)
  var air = { ambientC: -28.0, sinkK: 180.0, epsilon: 0.88, radiatorAreaM2: 0.048, rho: 0, cp: 0, cfmEach: 0, intakeCount: 0 };

  var thermal = {
    ambientC: -28.0,
    fanRpm: 0,
    fanNominalRpm: 4500,
    fanSpinRad: 0,
    intakeC: -28.0,
    exhaustC: 24.0,
    flowLs: 0.0,
    heatW: 9.8,

    // Core Component Temperatures in °C
    piChipC: 48.0,     // Raspberry Pi SoC on Cu/heat-pipe cold plate
    chamberC: 18.0,    // MLI-insulated avionics warm box
    driverC: 25.0,     // 3-phase motor bridge on cold plate
    buckC: 23.0,       // 12V -> 5V buck regulator
    batteryC: 16.0,    // LiFePO4 pack (MLI + Kapton heater, belly bay)

    // 6 Sealed Hub Actuators
    motorNames: ['FL', 'FR', 'ML', 'MR', 'BL', 'BR'],
    motorsC: [18.0, 18.0, 18.0, 18.0, 18.0, 18.0],
    hottestMotor: { index: 0, name: 'FL', tempC: 18.0 },

    status: 'NOMINAL', // NOMINAL, WARM, HOT, THROTTLING
    throttling: false,
    throttleFactor: 1.0,
    note: 'Lunar vacuum heat-pipe + zenith Z-93 OSR radiator physics (no airflow in vacuum).'
  };

  var airState = {
    intakeC: -28.0,
    exhaustC: 24.0,
    flowLs: 0.0,
    heatW: 9.8,
    riseC: 0.0,
    spinRad: 0,
    note: 'Vacuum conductive strap + zenith OSR louvers (0 airflow).'
  };

  function setFanRPM(rpm) {
    // Compatibility shim: maps louver/radiator command (0 in vacuum) cleanly
    rpm = Math.max(0, Math.min(8000, Number(rpm) || 0));
    thermal.fanRpm = rpm;
    return rpm;
  }

  function stepThermal(dt, vehicleState) {
    dt = Math.max(0.001, Math.min(0.1, dt || 0.016));
    vehicleState = vehicleState || {};
    var throttle = Math.abs(vehicleState.throttle || 0);
    var vel = Math.abs(vehicleState.vel || 0);
    var steer = Math.abs(vehicleState.steer || 0);
    var slip = !!vehicleState.slip;
    var neuralOn = !!vehicleState.neuralOn;
    var visionActive = !!vehicleState.visionActive;
    var armMoving = !!vehicleState.armMoving;

    // Lunar vacuum: zero convective airflow; rejection is via Stefan-Boltzmann zenith OSR radiation + frame conduction
    var rpmFrac = thermal.fanRpm > 0 ? Math.min(1.5, thermal.fanRpm / thermal.fanNominalRpm) : 1.0;
    thermal.flowLs = 0.0;
    thermal.fanSpinRad = 0;

    // 1. Internal Avionics Chamber Heat Generation
    var qPi = 4.8 + (neuralOn ? 3.2 : 0.5) + (visionActive ? 1.8 : 0.3) + throttle * 1.6;
    var qDriver = 1.0 + 5.5 * Math.pow(throttle, 1.8) + (slip ? 4.5 : 0);
    var qBuck = 1.2 + qPi * 0.15;
    var qBat = 0.35 + 1.4 * Math.pow(throttle, 2.0);
    var totalBayHeat = qPi + qDriver + qBuck + (armMoving ? 2.5 : 0);
    thermal.heatW = totalBayHeat;

    // Stefan-Boltzmann radiation + chassis conduction (G_rad linearized around ~300 K + bimodal louver opening 0.15..1.0)
    var louverFrac = thermal.chamberC < 5 ? 0.15 : (thermal.chamberC > 25 ? 1.0 : 0.15 + 0.85 * (thermal.chamberC - 5) / 20);
    var SIGMA = 5.67037e-8, TchamK = thermal.chamberC + 273.15, TsinkK = air.sinkK;
    var gRad = Math.max(0.12, 4 * SIGMA * air.epsilon * air.radiatorAreaM2 * louverFrac * Math.pow((TchamK + TsinkK) * 0.5, 3));
    var kChassis = 0.38;
    var tSinkC = TsinkK - 273.15;
    var tChamberTarget = Math.max(0, tSinkC + totalBayHeat / (gRad + kChassis));
    thermal.chamberC += (tChamberTarget - thermal.chamberC) * (1 - Math.exp(-dt / 8.0));

    // Radiator cold-plate surface estimate (no air exhaust)
    thermal.exhaustC = thermal.chamberC - Math.min(6, totalBayHeat * 0.25);

    // 2. SoC Junction Temperature via Cu Heat Pipe to Cold Plate
    var thetaDie = 2.6;
    var thetaHeatPipe = 1.4 + 0.8 / Math.max(0.5, rpmFrac);
    var tPiTarget = thermal.chamberC + qPi * (thetaDie + thetaHeatPipe);
    thermal.piChipC += (tPiTarget - thermal.piChipC) * (1 - Math.exp(-dt / 2.2));
    thermal.piChipC = Math.max(thermal.chamberC + 16.0, thermal.piChipC);

    // 3. Space-rated 3-Phase Motor Driver
    var thetaDriver = 1.1 + 1.0 / Math.max(0.5, rpmFrac);
    var tDriverTarget = thermal.chamberC + qDriver * thetaDriver;
    thermal.driverC += (tDriverTarget - thermal.driverC) * (1 - Math.exp(-dt / 2.5));
    thermal.driverC = Math.max(thermal.chamberC, thermal.driverC);

    // 4. 6 Wheel Motors (conduction through Ti-6Al-4V spokes + radiation to regolith sink, no vacuum air cooling)
    var maxMotTemp = -999;
    var maxMotIdx = 0;
    for (var mi = 0; mi < 6; mi++) {
      var isCorner = (mi === 0 || mi === 1 || mi === 4 || mi === 5);
      var isFront = (mi === 0 || mi === 1);
      var loadFactor = throttle * (isFront ? 1.15 : 1.0) + (isCorner ? steer * 0.75 : 0) + (slip ? 1.2 : 0);
      var qMotor = 0.5 + 11.0 * Math.pow(loadFactor, 1.8);
      var gMotorCond = 0.34 + 0.06 * Math.min(2, vel); // conductive wheel/strut sinking into regolith
      var tMotorTarget = Math.max(thermal.ambientC, tSinkC + qMotor / gMotorCond);
      thermal.motorsC[mi] += (tMotorTarget - thermal.motorsC[mi]) * (1 - Math.exp(-dt / 4.0));
      if (thermal.motorsC[mi] > maxMotTemp) {
        maxMotTemp = thermal.motorsC[mi];
        maxMotIdx = mi;
      }
    }
    thermal.hottestMotor = {
      index: maxMotIdx,
      name: thermal.motorNames[maxMotIdx],
      tempC: maxMotTemp
    };

    var tBatTarget = thermal.ambientC + 6 + qBat * 4.2 + throttle * 3.5;
    thermal.batteryC += (tBatTarget - thermal.batteryC) * (1 - Math.exp(-dt / 6.5));
    thermal.batteryC = Math.max(thermal.ambientC, thermal.batteryC);

    // 5. Overall Thermal Status & Hardware Throttling
    var isThrottled = (thermal.piChipC >= 80.0 || maxMotTemp >= 78.0 || thermal.chamberC >= 60.0);
    thermal.throttling = isThrottled;
    thermal.throttleFactor = isThrottled ? Math.max(0.40, 1.0 - (Math.max(thermal.piChipC - 80, maxMotTemp - 78, 0)) * 0.05) : 1.0;

    if (isThrottled) {
      thermal.status = 'THROTTLING';
    } else if (thermal.piChipC >= 74 || maxMotTemp >= 70 || thermal.chamberC >= 50) {
      thermal.status = 'HOT';
    } else if (thermal.piChipC >= 62 || maxMotTemp >= 48 || thermal.chamberC >= 36) {
      thermal.status = 'WARM';
    } else {
      thermal.status = 'NOMINAL';
    }

    airState.intakeC = thermal.intakeC;
    airState.exhaustC = thermal.exhaustC;
    airState.flowLs = thermal.flowLs;
    airState.heatW = thermal.heatW;
    airState.riseC = thermal.exhaustC - thermal.intakeC;
    airState.spinRad = thermal.fanSpinRad;

    return thermal;
  }

  function stepAir() {
    return stepThermal(0.016, {});
  }

  global.NASA_RoverBOM = {
    batteryZ: byId('battery').pos[2],          // v13: sled on the centreline now the two scanners are a balanced diagonal pair (fusion_rover reads it)
    batteryX: byId('battery').pos[0],          // v13: sled home 56 mm aft of centre: with the mast gone (the two scanners are a centred pair) this puts the empty and the loaded CoM within 0.2 mm of the base centre
    parts: PARTS,
    roverFootprint: ROVER_FOOTPRINT,
    dX: dX,
    dZ: dZ,
    byId: byId,
    curbKg: curbKg,
    massBreakdown: massBreakdown,
    partKg: partKg,
    MASS_SCALE: MASS_SCALE,
    LB_KG: LB_KG,
    com: com,
    G_LUNAR: G_LUNAR,
    G_EARTH: G_EARTH,
    axleXM: axleXM,
    tractionSnapshot: tractionSnapshot,
    bayHeatW: bayHeatW,
    battery: BATTERY,
    powerModeId: powerModeId,
    powerEcoMode: powerEcoMode,
    POWER_MODES: POWER_MODES,
    getPowerMode: getPowerMode,
    setPowerMode: setPowerMode,
    setPowerEcoMode: setPowerEcoMode,
    ecoFanTargetRpm: ecoFanTargetRpm,
    computePowerDraw: computePowerDraw,
    benchmarkPowerScenarios: benchmarkPowerScenarios,
    whUsableFromSoc: whUsableFromSoc,
    estimateRoundTrips: estimateRoundTrips,
    air: air,
    thermal: thermal,
    setFanRPM: setFanRPM,
    stepThermal: stepThermal,
    stepAir: stepAir,
    airState: function () { return airState; },
    solar: byId('solar'),
    pod: {
      scale: 4,
      fullM: [1.04, 0.60, 0.60],
      lengthM: 0.26,
      widthM: 0.15,
      heightM: 0.15,
      shape: 'canister',
      note: 'Cargo canister (1.04 m x 0.60 m dia flight article) at 1:4: domed ends, top grapple knob, side trunnions'
    },
    airlock: {
      fullOpeningM: 1.5,
      scale: 4,
      openingM: 1.5 / 4
    }
  };
})(typeof window !== 'undefined' ? window : globalThis);
