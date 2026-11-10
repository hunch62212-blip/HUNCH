// Thermal brain panel: environment, every thermal node against its range, the brain's current decision per
// channel with its reason, the active scenario, and the controls (brain on/off, scenarios on/off, trigger one).
import { NODES, SCENARIOS, DRIVE_LIMITS } from '../sim/thermal.js';
import { THERMAL } from '../sim/config.js';

const fmt = (v, d = 1) => (v == null || !Number.isFinite(v) ? '—' : v.toFixed(d));
const K0 = 273.15;

export function thermalTemplate() {
  const opts = Object.entries(SCENARIOS).map(([k, s]) => `<option value="${k}">${s.name}</option>`).join('');
  return `<div id="thermPanel" class="panel"><h3>Thermal brain <span class="small" style="text-transform:none;letter-spacing:0">T toggles</span><span class="tog">▾</span></h3><div class="body">
    <div class="mono small" id="thEnv"></div>
    <div id="thScn" class="thScn"></div>
    <table class="wheels thermT"><thead><tr><th style="text-align:left">part</th><th>°C</th><th style="width:96px">range</th><th>K/min</th></tr></thead><tbody id="thRows"></tbody></table>
    <div class="thBrain" id="thBrain"></div>
    <div class="btnrow" style="margin-top:6px">
      <button class="btn" id="thBrainBtn" title="Thermal brain on / off (off = thermostats only)">brain ON</button>
      <button class="btn" id="thScnBtn" title="Random preplanned thermal scenarios">scenarios ON</button>
      <select class="btn" id="thScnSel"><option value="random">random scenario</option>${opts}</select>
      <button class="btn" id="thGo">trigger</button>
    </div>
  </div></div>`;
}

export class ThermalPanel {
  constructor(root, sim) {
    this.sim = sim; this.root = root;
    const $ = (s) => root.querySelector(s);
    this.panel = $('#thermPanel'); this.env = $('#thEnv'); this.scn = $('#thScn'); this.rows = $('#thRows'); this.brain = $('#thBrain');
    this.bBtn = $('#thBrainBtn'); this.sBtn = $('#thScnBtn');
    this.bBtn.addEventListener('click', () => { const b = sim.thermal.brain; b.on = !b.on; sim.log(`Thermal brain ${b.on ? 'ON' : 'OFF (thermostats only)'}`, b.on ? 'ok' : 'warn'); });
    this.sBtn.addEventListener('click', () => sim.thermal.setScenarios(!sim.thermal.scen.on));
    $('#thGo').addEventListener('click', () => sim.thermal.trigger($('#thScnSel').value));
    this._t = 0;
  }

  // overall status for the top-bar pill: [text, class]
  status() {
    const th = this.sim.thermal; if (!th) return ['THERMAL', ''];
    if (!th.ok.lidar || !th.ok.camera) return ['THERMAL: SENSOR OFF', 'err'];
    const d = th.brain.choice.drive?.id;
    if (d === 'stop') return ['THERMAL: COOL-DOWN', 'err'];
    if (d === 'down1' || d === 'down2') return ['THERMAL: REV DOWN', 'warn'];
    if (d === 'up1' || d === 'up2') return ['THERMAL: REV UP', 'info'];
    if (th.scen.active) return ['THERMAL: EVENT', 'warn'];
    return ['THERMAL OK', 'ok'];
  }

  update(dt) {
    const th = this.sim.thermal; if (!th) return;
    this._t -= dt; if (this._t > 0) return; this._t = 0.25;
    if (this.panel.classList.contains('collapsed') || this.panel.offsetParent === null) return;
    const E = th.env, H = this.sim.health, R = this.sim.rover, b = th.brain;
    this.env.textContent = `sun ${fmt(E.sunEl, 1)}° ${E.sunlit ? 'sunlit' : 'shadow'} · regolith ${fmt(E.regK - K0, 0)} °C (${fmt(E.regK, 0)} K) · radiator sink ${fmt(E.sinkK, 0)} K · thermal power ${fmt(th.powerW, 0)} W`;
    const a = th.scen.active;
    this.scn.innerHTML = a ? `<b>${a.name}</b> · ${a.t1 - this.sim.time > 86400 ? 'until cleared' : fmt((a.t1 - this.sim.time) / 60, 1) + ' min left'}` : `<span class="dim">no scenario active${th.scen.on ? ` · next in ~${fmt(Math.max(0, th.scen.next - this.sim.time) / 60, 1)} min` : ' · scenarios off'}</span>`;
    this.scn.className = 'thScn' + (a ? ' on' : '');
    const ws = R.wheels, wmax = Math.max(...ws.map((w) => w.temp)), wrate = Math.max(...ws.map((w) => w.tRate || 0));
    const hubs = H?.th?.hub || [0], hmin = Math.min(...hubs);
    const L = DRIVE_LIMITS;
    const rows = [
      ['LiDAR head', th.n.lidar.T, th.n.lidar.rate, NODES.lidar.surv, NODES.lidar.op, th.ok.lidar ? b.choice.lidar.label : 'PROTECTED OFF'],
      ['Camera', th.n.camera.T, th.n.camera.rate, NODES.camera.surv, NODES.camera.op, th.ok.camera ? b.choice.camera.label : 'PROTECTED OFF'],
      ['Motor controllers', th.n.mc.T, th.n.mc.rate, NODES.mc.surv, NODES.mc.op, ''],
      ['Motor windings (max)', wmax, wrate, [-80, 150], [L.windColdMaxC, L.windMaxC], ''],
      ['Wheel hubs (min)', hmin, null, [-90, 120], [L.hubColdMaxC, 100], ''],
      ['Avionics vault', H?.th?.Tv, null, [-40, 70], [-10, THERMAL.vault.hotC], ''],
      ['Battery', H?.th?.Tb, null, [-30, 70], [0, THERMAL.battery.hotC], ''],
    ];
    this.rows.innerHTML = rows.map(([name, T, rate, sv, op, act]) => {
      const pos = Math.max(0, Math.min(100, ((T - sv[0]) / (sv[1] - sv[0])) * 100));
      const lo = ((op[0] - sv[0]) / (sv[1] - sv[0])) * 100, hi = ((op[1] - sv[0]) / (sv[1] - sv[0])) * 100;
      const cls = T > op[1] ? 'err' : T < op[0] ? 'cold' : T > op[1] - 0.15 * (op[1] - op[0]) ? 'warn' : '';
      return `<tr title="${name}: operating ${op[0]}..${op[1]} C${act ? ' · ' + act : ''}"><td style="text-align:left">${name}</td><td class="${cls}">${fmt(T, 1)}</td>
        <td><div class="tbar"><span style="left:${lo}%;width:${hi - lo}%"></span><i class="${cls}" style="left:${pos}%"></i></div></td><td>${rate == null ? '' : fmt(rate, 2)}</td></tr>`;
    }).join('');
    const ch = [['drive', 'Drive'], ['rad', 'Radiators'], ['lidar', 'LiDAR'], ['camera', 'Camera']];
    this.brain.innerHTML = ch.map(([k, n]) => {
      const c = b.choice[k], id = c.id;
      const cls = /stop|up2|shed/.test(id) ? 'err' : id === 'nominal' || id === 'auto' ? 'ok' : id === 'off' ? 'dim' : 'warn';
      return `<div class="thRow"><span class="k">${n}</span><span class="v ${cls}">${c.label}</span>${b.on && id !== 'nominal' && id !== 'auto' && b.reason[k] ? `<div class="why">${b.reason[k]}</div>` : ''}</div>`;
    }).join('');
    this.bBtn.textContent = `brain ${b.on ? 'ON' : 'OFF'}`; this.bBtn.classList.toggle('on', b.on);
    this.sBtn.textContent = `scenarios ${th.scen.on ? 'ON' : 'OFF'}`; this.sBtn.classList.toggle('on', th.scen.on);
  }
}
