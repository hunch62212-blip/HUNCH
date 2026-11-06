// 1 Hz engineering telemetry recorder (exportable as CSV for reports and analysis).
export const TELEMETRY_FIELDS = [
  ['t', 's'], ['x', 'm'], ['z', 'm'], ['y', 'm'], ['est_x', 'm'], ['est_z', 'm'], ['slam_err', 'm'],
  ['heading', 'deg'], ['speed', 'm/s'], ['roll', 'deg'], ['pitch', 'deg'], ['tip_margin', 'deg'],
  ['soc', '%'], ['solar', 'W'], ['load', 'W'], ['drive', 'W'], ['energy_used', 'Wh'],
  ['slip_mean', '%'], ['sinkage_mean', 'mm'], ['wheel_load_max', 'kN'], ['motor_temp_max', 'C'],
  ['hoist', 'm'], ['payload', ''], ['mode', ''], ['phase', ''],
];

export class Telemetry {
  constructor(sim) {
    this.sim = sim; this.rate = 1; this.rows = []; this.max = 40000;
  }
  update() {
    const s = this.sim, R = s.rover, p = R.pose(), e = s.slam.est, v = R.body.linvel();
    const cw = R.wheels.filter((w) => w.contact);
    const mean = (f) => cw.length ? cw.reduce((a, w) => a + f(w), 0) / cw.length : 0;
    this.rows.push([
      s.time, p.x, p.z, p.y, e.x, e.z, Math.hypot(e.x - p.x, e.z - p.z),
      ((90 - p.yaw * 180 / Math.PI) % 360 + 360) % 360, Math.hypot(v.x, v.y, v.z), R.tilt.roll * 57.3, R.tilt.pitch * 57.3, R.tilt.margin,
      R.power.soc * 100, R.power.solarW, R.power.loadW, R.power.driveW, R.power.energyUsedWh,
      mean((w) => Math.abs(w.slip)) * 100, mean((w) => w.sink) * 1000, Math.max(0, ...R.wheels.map((w) => w.load)) / 1000, Math.max(...R.wheels.map((w) => w.temp)),
      R.lift.pos, R.loaded ? R.lift.cargo.id : '', s.autonomy.mode, (s.autonomy.missionName || '') + ' ' + s.autonomy.phase,
    ]);
    if (this.rows.length > this.max) this.rows.shift();
  }
  series(i, seconds = 300) {
    const t1 = this.sim.time, out = [];
    for (let k = this.rows.length - 1; k >= 0; k--) { const r = this.rows[k]; if (t1 - r[0] > seconds) break; out.push([r[0], r[i]]); }
    return out.reverse();
  }
  csv() {
    const head = TELEMETRY_FIELDS.map(([n, u]) => (u ? `${n}_${u}` : n)).join(',');
    const esc = (v) => (typeof v === 'number' ? (Number.isInteger(v) ? v : v.toFixed(4)) : `"${String(v).replace(/"/g, "'")}"`);
    return [head, ...this.rows.map((r) => r.map(esc).join(','))].join('\n');
  }
}
