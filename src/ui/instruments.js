// Vehicle dynamics instruments (game / flight-sim style, fed by src/sim/dynamics.js):
//  * attitude indicator: horizon tilted by roll and shifted by pitch, roll scale with the static rollover
//    threshold atan(SSF) marked red and the governor's slow-down band amber (an inclinometer that knows
//    where the vehicle actually tips, not a fixed 30 deg mark)
//  * stability plan view (crane load-moment-indicator style): support polygon in body axes with the CoM
//    projection, the critical tip edge in red, and a utilisation bar (100 % = wheels lift)
//  * physics readouts, each with its formula in the tooltip
// Canvas drawing is cheap (two 300 x 300 canvases) and runs every frame so the needle moves smoothly.
import { vehicleDynamics } from '../sim/dynamics.js';

const DEG = 180 / Math.PI;
const fmt = (v, d = 1) => (v == null || !Number.isFinite(v) ? '—' : v.toFixed(d));
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

export const INSTRUMENT_ROWS = [
  ['iSSF', 'Static stability factor', 'SSF = T_eff / 2h (T_eff = 2 x CoM-to-nearest-side-edge, h = CoM height above the contact plane). Rollover tilt = atan(SSF).'],
  ['iRollLim', 'Rollover / pitch-over tilt', 'atan(T_eff / 2h) and atan(L_eff / 2h): static tilt at which the CoM passes the support-polygon edge.'],
  ['iMargin', 'Force-angle margin', 'Papadopoulos & Rey: smallest angle the gravity vector through the CoM must rotate to cross a tip axis.'],
  ['iLat', 'Lateral accel / limit', 'a_y = v x yaw rate; static limit a = g x SSF (lunar g = 1.62 m/s^2).'],
  ['iStop', 'Stopping distance', 'd = v t_r + v^2 / 2a, t_r = 0.5 s, a = 0.6 m/s^2 empty, 0.2 m/s^2 loaded (autonomy braking limits).'],
  ['iDP', 'Drawbar pull', 'DP = sum(H) - sum(R_c): Bekker-Wong soil thrust minus compaction resistance over the six wheels.'],
  ['iCoef', 'Traction / resistance coef.', 'sum(H) / sum(N) and sum(R_c) / sum(N), N = wheel normal loads.'],
  ['iSlip', 'Slip (load weighted)', 's = (r omega - v) / max(r omega, v), weighted by wheel load.'],
  ['iPow', 'Drive power mech / elec', 'P_mech = sum(T omega) at the wheels; P_elec from the motor model; efficiency = ratio.'],
  ['iCOT', 'Energy per km / COT', 'E/d = P_elec / v; cost of transport = P_elec / (m g v).'],
  ['iFr', 'Froude number', 'Fr = v^2 / (g L), L = wheelbase: equal Fr = dynamically similar to the 1:16.3 prototype.'],
  ['iKE', 'Kinetic energy / grade force', 'KE = m v^2 / 2; grade force = m g sin(pitch).'],
];

export function instrumentsTemplate() {
  return `<div id="instPanel" class="panel"><h3>Dynamics instruments <span class="small" style="text-transform:none;letter-spacing:0">I toggles · hover a value for its formula</span><span class="tog">▾</span></h3><div class="body">
    <div class="instRow">
      <div class="instGauge"><canvas id="adi" width="300" height="300"></canvas><div class="cap">Attitude · rollover limit</div></div>
      <div class="instGauge"><canvas id="lmi" width="300" height="300"></canvas><div class="cap">Stability plan · CoM</div></div>
      <div class="kv small instKv">${INSTRUMENT_ROWS.map(([id, k, f]) => `<span class="k" title="${f}">${k}</span><span class="v" id="${id}" title="${f}">—</span>`).join('')}</div>
    </div></div></div>`;
}

export class Instruments {
  constructor(root) {
    this.panel = root.querySelector('#instPanel');
    this.adi = root.querySelector('#adi').getContext('2d');
    this.lmi = root.querySelector('#lmi').getContext('2d');
    this.el = Object.fromEntries(INSTRUMENT_ROWS.map(([id]) => [id, root.querySelector('#' + id)]));
    this._tText = 0;
    this.colors = { ok: css('--ok') || '#3ddc84', warn: css('--warn') || '#ffb020', err: css('--err') || '#ff4d3a', accent: css('--accent') || '#35d0ff', dim: css('--dim') || '#8796a8', text: css('--text') || '#dfe6ee' };
  }

  get visible() { return this.panel.offsetParent !== null && !this.panel.classList.contains('collapsed'); } // CSS may hide it (phones)

  update(R, dt, show) {
    this.panel.style.display = show ? '' : 'none';
    if (!this.visible) return;
    const d = vehicleDynamics(R);
    this.drawADI(d, R);
    this.drawLMI(d, R);
    this._tText -= dt;
    if (this._tText <= 0) { this._tText = 0.2; this.text(d); }
  }

  text(d) {
    const e = this.el, set = (id, txt, cls) => { e[id].textContent = txt; e[id].className = 'v' + (cls ? ' ' + cls : ''); };
    const util = d.rolloverUtil;
    set('iSSF', fmt(d.SSF, 2), d.SSF < 0.6 ? 'err' : d.SSF < 0.9 ? 'warn' : '');
    set('iRollLim', `${fmt(d.rollLimitDeg, 0)}° / ${fmt(d.pitchLimitDeg, 0)}°`);
    set('iMargin', `${fmt(d.marginDeg, 1)}°`, d.marginDeg < 8 ? 'err' : d.marginDeg < 25 ? 'warn' : '');
    set('iLat', `${fmt(d.aLat, 2)} / ${fmt(d.aRoll, 2)} m/s²`, util > 0.8 ? 'err' : util > 0.5 ? 'warn' : '');
    set('iStop', `${fmt(d.stopDist, 1)} m`);
    set('iDP', `${fmt(d.drawbar / 1000, 2)} kN`, d.drawbar < -500 && Math.abs(d.v) < 0.05 && d.Pelec > 2000 ? 'warn' : '');
    set('iCoef', `${fmt(d.tractionCoef, 3)} / ${fmt(d.resistCoef, 3)}`);
    set('iSlip', `${fmt(d.slip * 100, 1)} %`, d.slip > 0.4 ? 'err' : d.slip > 0.2 ? 'warn' : '');
    set('iPow', `${fmt(d.Pmech / 1000, 1)} / ${fmt(d.Pelec / 1000, 1)} kW${Number.isFinite(d.eta) ? ` (η ${fmt(d.eta * 100, 0)} %)` : ''}`);
    set('iCOT', `${fmt(d.whPerKm, 0)} Wh/km · ${fmt(d.cot, 3)}`);
    set('iFr', fmt(d.froude, 3));
    set('iKE', `${fmt(d.ke / 1000, 1)} kJ · ${fmt(d.gradeF / 1000, 2)} kN`);
  }

  // ---- attitude indicator --------------------------------------------------------------------------------
  drawADI(d, R) {
    const g = this.adi, W = 300, C = W / 2, Rr = 128, C0 = this.colors;
    g.clearRect(0, 0, W, W);
    const roll = d.roll, pitchDeg = d.pitch * DEG, ppd = 3.2; // px per degree of pitch
    g.save();
    g.beginPath(); g.arc(C, C, Rr, 0, Math.PI * 2); g.clip();
    g.translate(C, C); g.rotate(-roll);
    const off = pitchDeg * ppd;
    g.fillStyle = '#0b1424'; g.fillRect(-W, -W * 2 + off, W * 2, W * 2);          // sky (lunar black-blue)
    g.fillStyle = '#5d564b'; g.fillRect(-W, off, W * 2, W * 2);                    // regolith
    g.strokeStyle = '#e8e2d4'; g.lineWidth = 2.5; g.beginPath(); g.moveTo(-W, off); g.lineTo(W, off); g.stroke();
    g.fillStyle = g.strokeStyle = 'rgba(232,226,212,0.85)'; g.lineWidth = 1.5; g.font = '600 15px "SF Mono", Menlo, monospace'; g.textAlign = 'left'; g.textBaseline = 'middle';
    for (let p = -40; p <= 40; p += 5) {
      if (!p) continue;
      const y = off - p * ppd, w = p % 10 ? 18 : 36;
      if (Math.abs(y) > Rr) continue;
      g.beginPath(); g.moveTo(-w, y); g.lineTo(w, y); g.stroke();
      if (!(p % 10)) g.fillText(String(Math.abs(p)), w + 6, y);
    }
    g.restore();
    // roll scale (fixed to the case), pointer rotates with roll
    g.save(); g.translate(C, C);
    const lim = Number.isFinite(d.rollLimitDeg) ? d.rollLimitDeg : 60;
    const band = (a0, a1, color) => { g.strokeStyle = color; g.lineWidth = 7; g.beginPath(); g.arc(0, 0, Rr + 7, -Math.PI / 2 + a0 / DEG, -Math.PI / 2 + a1 / DEG); g.stroke(); };
    const amber = Math.max(0, lim - 25); // autonomy slows when the force-angle margin is below 25 deg
    for (const s of [-1, 1]) {
      band(s > 0 ? 0 : -amber, s > 0 ? amber : 0, 'rgba(61,220,132,0.55)');
      band(s > 0 ? amber : -lim, s > 0 ? lim : -amber, C0.warn);
      band(s > 0 ? lim : -80, s > 0 ? 80 : -lim, C0.err);
    }
    g.strokeStyle = C0.text; g.lineWidth = 2;
    for (const a of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
      const t = -Math.PI / 2 + a / DEG, r0 = Rr + 2, r1 = Rr + (a % 30 ? 10 : 14);
      g.beginPath(); g.moveTo(Math.cos(t) * r0, Math.sin(t) * r0); g.lineTo(Math.cos(t) * r1, Math.sin(t) * r1); g.stroke();
    }
    g.rotate(-roll);
    g.fillStyle = C0.text; g.beginPath(); g.moveTo(0, -Rr + 2); g.lineTo(-9, -Rr + 18); g.lineTo(9, -Rr + 18); g.closePath(); g.fill();
    g.restore();
    // fixed vehicle symbol
    g.strokeStyle = '#ffd23d'; g.lineWidth = 4; g.lineCap = 'round';
    g.beginPath(); g.moveTo(C - 62, C); g.lineTo(C - 22, C); g.lineTo(C - 12, C + 10); g.moveTo(C + 62, C); g.lineTo(C + 22, C); g.lineTo(C + 12, C + 10); g.stroke();
    g.fillStyle = '#ffd23d'; g.beginPath(); g.arc(C, C, 4, 0, Math.PI * 2); g.fill();
    // readout
    g.fillStyle = 'rgba(10,13,18,0.8)'; g.fillRect(C - 92, W - 34, 184, 28);
    g.fillStyle = Math.abs(roll * DEG) > lim - 25 || Math.abs(pitchDeg) > (d.pitchLimitDeg || 60) - 25 ? C0.warn : C0.text;
    g.font = '600 16px "SF Mono", Menlo, monospace'; g.textAlign = 'center';
    g.fillText(`R ${fmt(roll * DEG, 1)}°  P ${fmt(pitchDeg, 1)}°`, C, W - 19);
  }

  // ---- stability plan view (load moment indicator) ----------------------------------------------------------
  drawLMI(d, R) {
    const g = this.lmi, W = 300, C = W / 2, C0 = this.colors;
    g.clearRect(0, 0, W, W);
    g.fillStyle = 'rgba(5,7,10,0.85)'; g.beginPath(); g.roundRect(4, 4, W - 8, W - 8, 10); g.fill();
    const poly = R.tilt?.poly, com = R.comWorld;
    const util = Math.max(d.rolloverUtil || 0, d.pitchUtil || 0);
    if (poly && poly.length >= 3 && com) {
      const p = R.pose(), fx = p.fwd.x, fz = p.fwd.z, fl = Math.hypot(fx, fz) || 1;
      const ux = fx / fl, uz = fz / fl, rx = -uz, rz = ux;                       // body forward / right (horizontal)
      const toS = (x, z) => { const dx = x - com.x, dz = z - com.z; return [dx * rx + dz * rz, -(dx * ux + dz * uz)]; };
      let ext = 1;
      for (const q of poly) { const [a, b] = toS(q.x, q.z); ext = Math.max(ext, Math.abs(a), Math.abs(b)); }
      const s = 105 / ext;
      // polygon, margin-coloured
      const col = d.marginDeg < 8 ? C0.err : d.marginDeg < 25 ? C0.warn : C0.ok;
      g.beginPath();
      poly.forEach((q, i) => { const [a, b] = toS(q.x, q.z); i ? g.lineTo(C + a * s, C - 10 + b * s) : g.moveTo(C + a * s, C - 10 + b * s); });
      g.closePath(); g.fillStyle = col + '22'; g.fill(); g.strokeStyle = col; g.lineWidth = 2.5; g.stroke();
      // critical edge
      const e0 = poly[R.tilt.crit % poly.length], e1 = poly[(R.tilt.crit + 1) % poly.length];
      { const [a0, b0] = toS(e0.x, e0.z), [a1, b1] = toS(e1.x, e1.z); g.strokeStyle = C0.err; g.lineWidth = 5; g.beginPath(); g.moveTo(C + a0 * s, C - 10 + b0 * s); g.lineTo(C + a1 * s, C - 10 + b1 * s); g.stroke(); }
      // nose marker
      g.fillStyle = C0.dim; g.font = '600 13px "SF Mono", Menlo, monospace'; g.textAlign = 'center'; g.fillText('FWD ▲', C, 22);
      // CoM (centre) and where the resultant would go under the current lateral accel: shift = h * a_y / g
      const shift = Number.isFinite(d.h) ? d.h * d.aLat / d.g : 0;
      g.strokeStyle = C0.accent; g.lineWidth = 2; g.beginPath(); g.moveTo(C, C - 10); g.lineTo(C + shift * s, C - 10); g.stroke();
      g.fillStyle = C0.accent; g.beginPath(); g.arc(C + shift * s, C - 10, 5, 0, Math.PI * 2); g.fill();
      g.fillStyle = '#ffd23d'; g.beginPath(); g.arc(C, C - 10, 7, 0, Math.PI * 2); g.fill();
      g.strokeStyle = '#000'; g.lineWidth = 1.5; g.beginPath(); g.moveTo(C - 7, C - 10); g.lineTo(C + 7, C - 10); g.moveTo(C, C - 17); g.lineTo(C, C - 3); g.stroke();
    } else {
      g.fillStyle = C0.err; g.font = '600 15px "SF Mono", Menlo, monospace'; g.textAlign = 'center'; g.fillText('NO SUPPORT POLYGON', C, C);
    }
    // utilisation bar (LMI): 100 % = the static rollover / pitch-over limit
    const bx = 22, by = W - 36, bw = W - 44, bh = 14, u = Math.max(0, Math.min(1.2, util || 0));
    g.fillStyle = 'rgba(255,255,255,0.08)'; g.fillRect(bx, by, bw, bh);
    g.fillStyle = u > 0.8 ? C0.err : u > 0.5 ? C0.warn : C0.ok; g.fillRect(bx, by, bw * Math.min(1, u), bh);
    g.fillStyle = C0.text; g.font = '600 13px "SF Mono", Menlo, monospace'; g.textAlign = 'left';
    g.fillText(`tip use ${fmt(u * 100, 0)} %`, bx, by - 8);
    g.textAlign = 'right'; g.fillText(`margin ${fmt(d.marginDeg, 1)}°`, bx + bw, by - 8);
  }
}
