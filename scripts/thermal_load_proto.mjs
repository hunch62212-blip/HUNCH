// Thermal load estimate for the Earth prototype (docs/ARM_CARGO_BUILD_PLAN.md BOM).
// Run: node scripts/thermal_load_proto.mjs
// Hand model, not a measurement. Every thermal resistance / capacitance is [ASSUMED] (no datasheet value);
// electrical values come from the plan (s1.5, s5.1) and vendor pages. Replace with bench data (see report s7).

const g = 9.81;
const M = { curb: 8.30, loaded: 9.30 };             // kg, plan s3.1
const r = 0.065;                                     // wheel radius m (params.csv wh_r)
const track = 0.47;                                  // 2 x tr_z
const xAxle = 0.24;                                  // wb_x

// Pololu 37D 30:1 12 V (plan s5.1 check values)
const MOT = { V: 12, Is: 5.5, I0: 0.2, Ts: 1.37, w0: 34.6, gearLimit: 0.98 };
MOT.R25 = MOT.V / MOT.Is;                            // 2.18 ohm
MOT.k = (MOT.V - MOT.I0 * MOT.R25) / MOT.w0;         // 0.334 V s/rad (output-referred)
MOT.Kt = MOT.Ts / (MOT.Is - MOT.I0);                 // 0.258 N m/A at the output (gearbox loss folded in)
const ALPHA_CU = 0.0039;

// Two-node motor thermal model [ASSUMED]: winding -> can/gearbox -> air.
const TH = {
  Rwh: 2.5, Cw: 12,          // winding->housing K/W, winding J/K  (tau ~30 s)
  RhaStill: 9, RhaMoving: 6, // housing->air K/W (still air / driving airflow); 14 if boxed in a PETG pod
  Ch: 100,                   // whole gearmotor ~0.21 kg steel/brass (tau ~15 min)
};

const VBAT = { full: 14.4, nominal: 13.2, low: 12.4 }; // LiFePO4 4S resting/charged voltages

function wheelTorque(scn, mass) {
  const W = mass * g;
  const crr = scn.crr;
  const slope = (scn.slopeDeg || 0) * Math.PI / 180;
  let Ftot = W * Math.sin(slope) + crr * W * Math.cos(slope);
  let perMotor = Ftot / 4 * r;
  if (scn.pivot) {
    // skid-steer pivot: lateral scrub on front + rear axles (middle wheels sit on the yaw centre),
    // driven axles carry 2/3 of the weight at x_cg = 0 (plan s3.2)
    const Mr = scn.muT * W * (2 / 3) * xAxle;
    const Fside = Mr / track;
    perMotor = (Fside / 2 + crr * W / 4) * r;
  }
  return perMotor;
}

function motorOperatingPoint(tau, w, Tw) {
  const R = MOT.R25 * (1 + ALPHA_CU * (Tw - 25));
  const I = MOT.I0 + tau / MOT.Kt;
  const Vm = I * R + MOT.k * w;
  const Pcu = I * I * R;
  const Pmech = Math.max(0, MOT.k * w * I - tau * w); // gear + brush + bearing loss
  return { I, Vm, Pcu, Pmech, P: Pcu + Pmech, R };
}

function stallPower(V, Tw) {
  const R = MOT.R25 * (1 + ALPHA_CU * (Tw - 25));
  const I = V / R;
  return { I, P: V * I, R };
}

// Transient: segments [{dur, kind:'load', tau, w} | {dur, kind:'stall', V} | {dur, kind:'off'}]
function simulateMotor(segments, Ta, Rha = TH.RhaStill, repeat = 1) {
  let Tw = Ta, Th = Ta, t = 0, peakW = Ta, peakH = Ta;
  const dt = 0.05;
  const log = [];
  for (let k = 0; k < repeat; k++) {
    for (const s of segments) {
      for (let tt = 0; tt < s.dur; tt += dt) {
        let Pw = 0, Ph = 0;
        if (s.kind === 'load') { const op = motorOperatingPoint(s.tau, s.w, Tw); Pw = op.Pcu; Ph = op.Pmech; }
        else if (s.kind === 'stall') { Pw = stallPower(s.V, Tw).P; }
        const qwh = (Tw - Th) / TH.Rwh;
        const rha = s.w > 5 ? Math.min(Rha, TH.RhaMoving) : Rha;
        const qha = (Th - Ta) / rha;
        Tw += (Pw - qwh) / TH.Cw * dt;
        Th += (qwh + Ph - qha) / TH.Ch * dt;
        t += dt;
        peakW = Math.max(peakW, Tw); peakH = Math.max(peakH, Th);
      }
      log.push({ t: +t.toFixed(0), seg: s.name || s.kind, Tw: +Tw.toFixed(1), Th: +Th.toFixed(1) });
    }
  }
  return { peakW, peakH, endW: Tw, endH: Th, t, log };
}

function timeToWinding(seg, Ta, limit, Rha = TH.RhaStill) {
  let Tw = Ta, Th = Ta, t = 0; const dt = 0.05;
  while (t < 3600) {
    let Pw = 0, Ph = 0;
    if (seg.kind === 'load') { const op = motorOperatingPoint(seg.tau, seg.w, Tw); Pw = op.Pcu; Ph = op.Pmech; }
    else { Pw = stallPower(seg.V, Tw).P; }
    const qwh = (Tw - Th) / TH.Rwh, qha = (Th - Ta) / Rha;
    Tw += (Pw - qwh) / TH.Cw * dt; Th += (qwh + Ph - qha) / TH.Ch * dt; t += dt;
    if (Tw >= limit) return { t, Th };
  }
  return { t: Infinity, Th };
}

// ---------------------------------------------------------------- scenarios
const scenarios = [
  { name: 'Cruise, flat floor/carpet',          crr: 0.05, w: 15 },
  { name: 'Cruise, packed sand / gravel',       crr: 0.15, w: 12 },
  { name: '10 deg ramp, loaded',                crr: 0.10, slopeDeg: 10, w: 10 },
  { name: '20 deg ramp, loaded',                crr: 0.10, slopeDeg: 20, w: 8 },
  { name: 'Pivot turn, low-grip (mu_t 0.4)',    crr: 0.08, pivot: true, muT: 0.4, w: 5 },
  { name: 'Pivot turn, rubber-on-concrete (0.8)', crr: 0.08, pivot: true, muT: 0.8, w: 5 },
];

const out = { motor: { R25: +MOT.R25.toFixed(3), k: +MOT.k.toFixed(3), Kt: +MOT.Kt.toFixed(3) }, steady: [], stall: [], mission: {}, electronics: {} };

console.log('=== Pololu 37D 30:1, per motor (loaded rover 9.3 kg, 4 driven) ===');
console.log('R25 %s ohm, Kt(out) %s N m/A, k %s V s/rad', out.motor.R25, out.motor.Kt, out.motor.k);
for (const s of scenarios) {
  const tau = wheelTorque(s, M.loaded);
  const op = motorOperatingPoint(tau, s.w, 60);
  const Rtot = TH.Rwh + (s.w > 5 ? TH.RhaMoving : TH.RhaStill);
  const ssRise = op.Pcu * Rtot + op.Pmech * (s.w > 5 ? TH.RhaMoving : TH.RhaStill);
  const ttl = timeToWinding({ kind: 'load', tau, w: s.w }, 25, 100);
  const row = {
    scenario: s.name, tauNm: +tau.toFixed(3), pctGearLimit: Math.round(100 * tau / MOT.gearLimit),
    I: +op.I.toFixed(2), heatW: +op.P.toFixed(1), ssWindingC_25amb: Math.round(25 + ssRise), ssWindingC_40amb: Math.round(40 + ssRise),
    secTo100C: isFinite(ttl.t) ? Math.round(ttl.t) : 'never',
  };
  out.steady.push(row);
  console.log(JSON.stringify(row));
}

console.log('\n=== Stall (MDD10A has no current limit; full PWM) ===');
for (const [lbl, V] of Object.entries(VBAT)) {
  const sp = stallPower(V, 25);
  const t100 = timeToWinding({ kind: 'stall', V }, 25, 100);
  const t130 = timeToWinding({ kind: 'stall', V }, 25, 130);
  const t130h = timeToWinding({ kind: 'stall', V }, 40, 130);
  const row = { vbat: `${lbl} ${V} V`, I: +sp.I.toFixed(1), W: Math.round(sp.P), secTo100C: +t100.t.toFixed(1), secTo130C: +t130.t.toFixed(1), secTo130C_40amb: +t130h.t.toFixed(1) };
  out.stall.push(row);
  console.log(JSON.stringify(row));
}
const fourStallA = 4 * VBAT.full / MOT.R25;
console.log('4 motors stalled at full charge: %s A (30 A fuse; BMS rating [VERIFY])', fourStallA.toFixed(1));

// Demo mission loop: 60 s flat sand cruise, 3 x (3 s pivot hi-grip), 15 s 20 deg ramp, 30 s arm pick (motors idle), repeated 20 min.
const sand = wheelTorque(scenarios[1], M.loaded), ramp = wheelTorque(scenarios[3], M.loaded), piv = wheelTorque(scenarios[5], M.loaded);
const loop = [
  { name: 'cruise', kind: 'load', tau: sand, w: 12, dur: 60 },
  { name: 'pivot', kind: 'load', tau: piv, w: 5, dur: 9 },
  { name: 'ramp', kind: 'load', tau: ramp, w: 8, dur: 15 },
  { name: 'arm', kind: 'off', w: 0, dur: 30 },
];
const loopLen = loop.reduce((a, s) => a + s.dur, 0);
const reps = Math.ceil(1200 / loopLen);
for (const Ta of [25, 40]) {
  const r1 = simulateMotor(loop, Ta, TH.RhaStill, reps);
  const r2 = simulateMotor(loop, Ta, 14, reps);
  out.mission[`amb${Ta}`] = { loopSec: loopLen, reps, openPod: { peakWinding: Math.round(r1.peakW), peakHousing: Math.round(r1.peakH) }, boxedPETGpod: { peakWinding: Math.round(r2.peakW), peakHousing: Math.round(r2.peakH) } };
}
// Mission with one 5 s stall (wheel jammed on a rock) followed by watchdog cut vs no cut
for (const [lbl, stallDur] of [['stall5s_thenCut', 5], ['stall30s_noDetection', 30]]) {
  const seq = [{ name: 'cruise', kind: 'load', tau: sand, w: 12, dur: 120 }, { name: 'stall', kind: 'stall', V: VBAT.nominal, w: 0, dur: stallDur }, { name: 'cool', kind: 'off', w: 0, dur: 120 }];
  const rr = simulateMotor(seq, 25);
  out.mission[lbl] = { peakWinding: Math.round(rr.peakW), peakHousing: Math.round(rr.peakH) };
}
console.log('\n=== 20-min demo loop + stall cases ===');
console.log(JSON.stringify(out.mission, null, 1));

// ---------------------------------------------------------------- electronics bay
// Heat inside the tub (420 x 292 x 156 mm, closed by the deck). Motors are outboard in the pods.
const bay = {
  pi5: { idle: 2.7, ros: 5.5, max: 8.0 },         // board W; Pi 5 measured ~2.7 idle, ~7-8 W stress (Raspberry Pi / community data)
  buck5v: (loadW) => loadW / 0.88 - loadW,        // YRDZXG-class buck ~88 %
  esp32: 0.5, pca9685: 0.1, imu: 0.05,
  lidarUsb: 2 * 1.75,                             // 2 x X2L on 5 V [VERIFY]
  cams: 2 * 0.3,
  mdd10a: (I) => 2 * 0.02 * I * I,                // per channel: two FETs in path, ~10 mohm hot each [ASSUMED]
  ubec: (W) => W / 0.9 - W,
};
function bayHeat(state) {
  const pi = bay.pi5[state.pi];
  const fiveV = pi + bay.esp32 + bay.lidarUsb + bay.cams + bay.imu + bay.pca9685;
  const buck = bay.buck5v(fiveV);
  const mdd = 4 * bay.mdd10a(state.motorI);
  const ubec = bay.ubec(state.servoW);
  const bat = (state.packA ** 2) * 0.04;          // 4S 10 Ah LiFePO4 ~40 mohm [ASSUMED]
  return { pi, other5V: +(fiveV - pi).toFixed(2), buck: +buck.toFixed(2), mdd: +mdd.toFixed(2), ubec: +ubec.toFixed(2), battery: +bat.toFixed(2),
    total: +(fiveV + buck + mdd + ubec + bat).toFixed(1), fiveVloadA: +(fiveV / 5).toFixed(2) };
}
const bayStates = {
  idle: { pi: 'idle', motorI: 0, servoW: 0, packA: 1 },
  drive_sand: { pi: 'ros', motorI: motorOperatingPoint(sand, 12, 60).I, servoW: 2, packA: 5 },
  arm_hold_at_reach: { pi: 'ros', motorI: 0, servoW: 34, packA: 4.5 },
  worst: { pi: 'max', motorI: motorOperatingPoint(piv, 5, 60).I, servoW: 89, packA: 25 },
};
// Box: 4 mm PETG/plywood walls, natural convection both sides. U ~ 1/(1/5 + t/k + 1/6) W/m2K; exposed area ~0.40 m2.
const U = 1 / (1 / 5 + 0.004 / 0.2 + 1 / 6), A = 0.40;
const fanG = (cfm) => cfm * 0.000472 * 1.2 * 1005; // W/K carried by airflow (m3/s * rho * cp)
console.log('\n=== Electronics bay (tub) ===');
for (const [k, s] of Object.entries(bayStates)) {
  const h = bayHeat(s);
  const riseSealed = h.total / (U * A);
  const riseFan = h.total / (U * A + fanG(4));       // one 40 mm fan ~4 CFM effective through a filtered tub
  out.electronics[k] = { ...h, airRiseSealedK: +riseSealed.toFixed(1), airRiseWith40mmFanK: +riseFan.toFixed(1) };
  console.log(k, JSON.stringify(out.electronics[k]));
}

// Pi 5 SoC: T = Tair + P_soc * Rth. SoC share ~80 % of board power. Throttle 80 C (soft), 85 C hard.
const piCool = { bare: 9.5, passiveCase: 6.5, activeCooler: 4.5 }; // K/W [ASSUMED from public stress-test curves]
console.log('\n=== Pi 5 SoC steady temp (ROS + 2 LiDAR + SLAM, 5.5 W board) ===');
const piTable = [];
for (const [cool, Rth] of Object.entries(piCool)) {
  for (const [lbl, air] of [['bench 25C open', 25], ['sealed tub, 25C room', 25 + out.electronics.drive_sand.airRiseSealedK], ['sealed tub, 40C outdoor', 40 + out.electronics.drive_sand.airRiseSealedK], ['tub + fan, 40C outdoor', 40 + out.electronics.drive_sand.airRiseWith40mmFanK], ['sealed tub + sun on dark deck', 40 + out.electronics.drive_sand.airRiseSealedK + 15]]) {
    const ros = air + 0.8 * bay.pi5.ros * Rth;
    const max = air + 0.8 * bay.pi5.max * Rth;
    piTable.push({ cooling: cool, case: lbl, airC: Math.round(air), socRosC: Math.round(ros), socMaxC: Math.round(max), throttles: max >= 80 ? (ros >= 80 ? 'yes, always' : 'under full load') : 'no' });
  }
}
console.table(piTable);
out.pi = piTable;

if (process.argv.includes('--json')) console.log(JSON.stringify(out));

// ---------------------------------------------------------------- fix comparison (same 20-min loop)
console.log('\n=== Fix comparison: 20-min loop peak winding / housing (C) ===');
function loopWith(opts) {
  const gear = opts.gear50 ? 50 / 30 : 1;              // 50:1 option: same motor, 1.67x torque per amp (plan s1.5: 21 kg cm stall)
  const kt = MOT.Kt * gear;
  const save = MOT.Kt, saveK = MOT.k; MOT.Kt = kt; MOT.k = saveK * gear; // output-referred constants scale with the ratio; wheel speed unchanged
  const pivTau = opts.arcTurn ? piv * 0.45 : piv;      // arc turn at R ~ 1 track width: scrub moment roughly halves [ASSUMED]
  const pivDur = opts.arcTurn ? 12 : 9;
  const L = [
    { kind: 'load', tau: sand, w: 12, dur: 60 },
    { kind: 'load', tau: pivTau, w: 5, dur: pivDur },
    { kind: 'load', tau: ramp, w: 8, dur: 15 },
    { kind: 'off', w: 0, dur: opts.cooldown ? 60 : 30 },
  ];
  const res = {};
  for (const Ta of [25, 40]) { const s = simulateMotor(L, Ta, opts.alMount ? 6 : TH.RhaStill, reps); res[Ta] = `${Math.round(s.peakW)} / ${Math.round(s.peakH)}`; }
  MOT.Kt = save; MOT.k = saveK;
  return res;
}
const fixes = {
  'baseline (pivot on hi-grip, 30:1)': {},
  'arc turns instead of pivots': { arcTurn: true },
  '50:1 gearmotors': { gear50: true },
  'aluminium motor mount (heat sink)': { alMount: true },
  'arc turns + 50:1 + Al mount': { arcTurn: true, gear50: true, alMount: true },
};
const ft = [];
for (const [k, o] of Object.entries(fixes)) { const r = loopWith(o); ft.push({ fix: k, '25C amb': r[25], '40C amb': r[40] }); }
console.table(ft);
