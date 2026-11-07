// Operator HUD: telemetry, SLAM map, camera feed overlay, missions, faults, console, E-STOP.
import { ROVER, WORLD, MOON, K } from '../sim/config.js';
import { CLS, RES, TILE } from '../sim/slam.js';
import { runCommand } from '../sim/commands.js';
import { PRIORITY } from '../sim/autonomy.js';
import P1 from '../sim/lunar_logistics_manifest.json';

const $ = (sel, el = document) => el.querySelector(sel);
const fmt = (v, d = 1) => (v == null || !Number.isFinite(v) ? '—' : v.toFixed(d));
const fmtT = (s) => { s = Math.max(0, s); const h = Math.floor(s / 3600), m = Math.floor(s / 60) % 60, x = Math.floor(s % 60); return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(x).padStart(2, '0')}`; };

const CLS_COLOR = {
  [CLS.FREE]: [70, 92, 80], [CLS.ROUGH]: [120, 120, 70], [CLS.ROCK]: [235, 120, 40], [CLS.SLOPE]: [200, 160, 60],
  [CLS.CRATER]: [190, 60, 200], [CLS.LETHAL]: [235, 50, 40],
};

export class HUD {
  constructor(sim, ui) {
    this.sim = sim; this.ui = ui;
    this.root = $('#hud');
    this.root.innerHTML = this.template();
    this.mapZoom = 1.6; // px per metre
    this.mapCanvas = $('#map'); this.mapCtx = this.mapCanvas.getContext('2d');
    this.mapCanvas.width = 636; this.mapCanvas.height = 636;
    this.tileCache = new Map();
    this.camOverlay = $('#camOverlay'); this.camCtx = this.camOverlay.getContext('2d');
    this.camOverlay.width = 636; this.camOverlay.height = 476;
    this.history = []; this.hIdx = -1;
    this.bind();
    this.fillManifest();
    sim.on('log', (e) => this.addLog(e));
    for (const e of sim.logs) this.addLog(e);
    this._t = 0;
  }

  template() {
    const cargoOpts = this.sim.cargo.map((c, i) => `<option value="${i === 0 ? 'A' : 'B'}">${c.id} · ${c.T.label}</option>`).join('');
    return `
    <div id="topbar" class="panel">
      <div class="brand">AEGIS-V1 · LLASO-P2-XPORT-2026<small>Rocker-bogie skid-steer cargo carrier · lunar south-pole logistics</small></div>
      <span class="pill mono" id="clock">MET 00:00:00</span>
      <span class="pill" id="pMode">MANUAL</span>
      <span class="pill" id="pDrive">ARMED</span>
      <span class="pill" id="pComms">COMMS</span>
      <span class="pill" id="pLidar">LIDAR</span>
      <span class="pill" id="pCam">CAM</span>
      <span class="pill" id="pSteer">SKID STEER</span>
      <div class="grow"></div>
      <div class="btnrow timebtns" id="timeBtns">
        ${[0, 1, 2, 5, 10, 20].map((x) => `<button class="btn" data-ts="${x}">${x === 0 ? '❚❚' : x + '×'}</button>`).join('')}
      </div>
      <div class="btnrow" id="viewBtns">
        ${['chase', 'orbit', 'top', 'cam', 'free'].map((v) => `<button class="btn" data-view="${v}">${v === 'cam' ? 'onboard' : v}</button>`).join('')}
      </div>
      <select class="btn" id="qualitySel" title="Graphics quality"><option value="auto">GFX auto</option><option value="low">GFX low</option><option value="medium">GFX medium</option><option value="high">GFX high</option><option value="ultra">GFX ultra</option></select>
      <button class="btn" id="helpBtn">? help</button>
      <div id="restart"><button class="btn danger" id="resetBtn" title="Release the E-STOP latch">RESET</button><button class="btn" id="startBtn" title="Power-on self-test and arm drive">START</button></div>
      <button id="estop" title="EMERGENCY STOP (Esc)">E-STOP</button>
    </div>
    <div id="vehicle" class="panel"><h3>Vehicle state <span class="tog">▾</span></h3><div class="body">
      <div class="kv">
        <span class="k">Ground speed</span><span class="v" id="vSpeed">0</span>
        <span class="k">Heading (true / SLAM)</span><span class="v" id="vHdg">0</span>
        <span class="k">Roll / pitch</span><span class="v" id="vAtt">0</span>
        <span class="k">Tip-over margin</span><span class="v" id="vMargin">0</span>
      </div>
      <div class="bar"><i id="bMargin"></i></div>
      <div class="kv">
        <span class="k">Battery</span><span class="v" id="vSoc">0</span>
      </div>
      <div class="bar"><i id="bSoc"></i></div>
      <div class="kv">
        <span class="k">Dock charge / load</span><span class="v" id="vPow">0</span>
        <span class="k">Power dock</span><span class="v" id="vDock">-</span>
        <span class="k">Drive power</span><span class="v" id="vDrive">0</span>
        <span class="k">Odometer / energy</span><span class="v" id="vOdo">0</span>
        <span class="k">Payload</span><span class="v" id="vLoad">none</span>
        <span class="k">Ride height (kneel -0.65 / carry +0.82)</span><span class="v" id="vLift">0</span>
        <span class="k">Cradle</span><span class="v" id="vClamp">open</span>
        <span class="k">Smart rpm · why</span><span class="v" id="vLim">0</span>
      </div>
      <table class="wheels"><thead><tr><th>whl</th><th>mtr rpm</th><th>T Nm</th><th>slip</th><th>sink mm</th><th>load kN</th><th>°C</th></tr></thead><tbody id="wheelRows"></tbody></table>
      <div class="sect"><div class="t">Live trends (last 5 min)</div><canvas id="trend" width="560" height="200" style="width:280px;height:100px;display:block"></canvas>
        <div class="btnrow" style="margin-top:4px"><button class="btn" id="csvBtn">export telemetry CSV</button><button class="btn" id="repBtn">mission report</button></div></div>
      <div class="sect"><div class="t">Rocker-bogie wheel stations (height above axle mm)</div><div class="mono small" id="bogieRow"></div></div>
      <div class="sect"><div class="t">Terramechanics</div><div class="mono small" id="soilRow"></div></div>
    </div></div>

    <div id="mapPanel" class="panel"><h3>SLAM map · elevation + hazards <span class="tog">▾</span></h3><div class="body">
      <canvas id="map"></canvas>
      <div class="legend"><span><i style="background:#465c50"></i>free</span><span><i style="background:#787846"></i>rough</span><span><i style="background:#c8a03c"></i>slope</span><span><i style="background:#eb7828"></i>rock</span><span><i style="background:#be3cc8"></i>crater</span><span><i style="background:#eb3228"></i>lethal</span><span><i style="background:#35d0ff"></i>plan</span><span><i style="background:#ffd23d"></i>remembered route</span></div>
      <div class="kv small" style="margin-top:6px">
        <span class="k">SLAM error (vs truth)</span><span class="v" id="sErr">0</span>
        <span class="k">Scan match</span><span class="v" id="sMatch">—</span>
        <span class="k">Mapped area</span><span class="v" id="sArea">0</span>
        <span class="k">Planner</span><span class="v" id="sPlan">—</span>
      </div>
      <div class="btnrow" style="margin-top:6px"><button class="btn" id="zIn">zoom +</button><button class="btn" id="zOut">zoom −</button><button class="btn" id="mapSave">save map</button><button class="btn" id="mapLoad">load map</button><button class="btn danger" id="mapClear">clear map</button></div>
    </div></div>

    <div id="camPanel" class="panel"><h3>Camera · fiducial + hazard detection <span class="tog">▾</span></h3><div class="body">
      <div id="camFrame"><canvas id="camOverlay"></canvas></div>
      <div class="small mono" id="camInfo" style="margin-top:4px">—</div>
    </div></div>

    <div id="missions" class="panel"><h3>Missions <span class="tog">▾</span></h3><div class="body">
      <div class="btnrow" style="margin-bottom:6px">
        <select class="btn" id="cargoSel">${cargoOpts}</select>
      </div>
      <div class="btnrow">
        <button class="btn" data-m="1" title="Leave the landing zone">M1 Leave LZ</button>
        <button class="btn" data-m="2" title="Find & approach cargo">M2 Approach</button>
        <button class="btn" data-m="3" title="Pick up & secure">M3 Pick up</button>
        <button class="btn" data-m="4" title="Transport to habitat">M4 Transport</button>
        <button class="btn" data-m="5" title="Deliver & place (dock)">M5 Deliver</button>
        <button class="btn on" data-m="6" title="Full logistics cycle">M6 Full cycle</button>
      </div>
      <div class="btnrow" style="margin-top:6px"><button class="btn" id="resumeBtn">resume</button><button class="btn danger" id="abortBtn">abort</button><button class="btn" data-c="goto hab">goto hab</button><button class="btn" data-c="goto pad">goto pad</button><button class="btn" data-c="charge">charge</button><button class="btn" data-c="lunar on">lunar cycle</button><button class="btn" data-c="night">night</button><button class="btn" data-c="day">day</button></div>
      <div class="sect"><div class="t">M9 · SCOUT-1 (team tracked rover)</div>
        <div class="btnrow"><button class="btn" data-c="scout on">deploy</button><button class="btn" data-c="scout recall">recall</button><button class="btn" data-c="scout off">park</button></div>
        <div class="small mono" id="scoutBox" style="margin-top:4px">parked</div></div>
      <div class="sect"><div class="t">M10 · inject a fault</div>
        <div class="btnrow">
          <button class="btn" data-c="fault block">block road</button><button class="btn" data-c="fault lidar">LiDAR fail</button><button class="btn" data-c="fault camera">camera fail</button>
          <button class="btn" data-c="fault comms">comms loss</button><button class="btn" data-c="fault battery 18">low battery</button><button class="btn" data-c="fault wheel 3">wheel 3 fail</button>
          <button class="btn" data-c="fault shift">cargo shift</button><button class="btn" data-c="fault port 1">port 1 down</button><button class="btn" data-c="clear faults">clear all</button>
        </div></div>
      <div class="sect"><div class="t">M11 · logistics manager plan</div><div id="planBox" class="small mono"></div></div>
      <div class="sect"><div class="t">Mission metrics</div><div class="metric" id="metrics"></div></div>
      <div class="sect"><div class="t">Overlays</div><div class="btnrow">
        <button class="btn on" data-ov="lidar">LiDAR</button><button class="btn on" data-ov="path">path</button><button class="btn" data-ov="arcs">arcs</button><button class="btn on" data-ov="routes">routes</button><button class="btn on" data-ov="fences">geofences</button><button class="btn on" data-ov="dust">dust</button><button class="btn" data-ov="xray">X-ray cargo</button>
      </div></div>
    </div></div>

    <div id="autoPanel" class="panel"><span class="pill" id="autoMode">MANUAL</span><span class="phase" id="autoPhase">Idle</span><span class="detail" id="autoDetail"></span></div>
    <div id="console" class="panel"><div id="log"></div><div id="cmdline"><span>&gt;</span><input id="cmd" autocomplete="off" spellcheck="false" placeholder='type a command, e.g. "limit rpm 20", "mission cycle", "help"' /></div></div>
    <div id="dockPanel" class="panel"><h3>Berthing sequence</h3><div class="body"><ol id="dockSteps"></ol><div class="kv small" style="margin-top:6px"><span class="k">Vestibule pressure</span><span class="v" id="dockP">0</span><span class="k">Leak rate</span><span class="v" id="dockL">—</span></div></div></div>
    <div id="manifestPanel" class="panel"><h3>CM-40A interior · LLASO-P1 manifest <span class="tog">▾</span></h3><div class="body">
      <div class="small" id="manSum"></div>
      <table class="wheels" id="manTable"><thead><tr><th style="text-align:left">item</th><th>day</th><th>bay</th><th>kg</th><th style="text-align:left">actor</th></tr></thead><tbody></tbody></table>
    </div></div>
    <div id="overrideBanner" class="panel">OPERATOR OVERRIDE — autonomy paused · type "resume" or press the resume button</div>
    <div id="estopBanner" class="panel"><div class="big">E-STOP LATCHED</div><div style="margin-top:6px">Drive torque removed · spring brakes set · actuators locked</div><div class="small" style="margin-top:6px">Manual restart required: press <b>RESET</b>, then <b>START</b> (power-on self-test)</div></div>
    <div id="helpPanel" class="panel"><h3>Controls & commands <span class="tog" id="helpClose">✕</span></h3><div class="body">
      <div><b>Driving (manual)</b><br><kbd>W</kbd>/<kbd>S</kbd> forward / reverse (hold)<br><kbd>A</kbd>/<kbd>D</kbd> steer (skid steer: curves while moving, slow point-turn when stopped and unloaded)<br><kbd>Space</kbd> brake · <kbd>Shift</kbd> slow / precise<br>Smart rpm picks 120 / 180 / 300 itself; hold full throttle straight on flat ground for TURBO<br>Gamepad: left stick drive/steer</div>
      <div><b>Cargo handling</b><br><kbd>G</kbd> close / open ring jaws<br><kbd>R</kbd>/<kbd>F</kbd> ride height up / down (kneel under a module, rise to lift it)</div>
      <div><b>Safety</b><br><kbd>Esc</kbd> E-STOP (latching). Restart = RESET then START.<br>Touching the drive keys during a mission pauses autonomy (operator override).</div>
      <div><b>View</b><br><kbd>C</kbd> cycle camera · mouse drag orbit · wheel zoom<br><kbd>V</kbd> LiDAR overlay · <kbd>P</kbd> pause · <kbd>H</kbd> this help<br><kbd>Enter</kbd> focus console · <kbd>↑</kbd> history</div>
      <div><b>Useful commands</b><br>mission cycle · mission 3 B · goto hab<br>limit rpm 120 · rpm 180 · turbo on/off/auto<br>drive 0.5 0 0.2 · stop<br>lift kneel/carry · clamp close<br>fault block · fault wheel 2 · clear faults<br>map save · routes · plan · timescale 10</div>
    </div></div>`;
  }

  bind() {
    const sim = this.sim, ui = this.ui;
    this.root.querySelectorAll('.panel h3').forEach((h) => h.addEventListener('click', (e) => { if (e.target.id === 'helpClose') return; h.parentElement.classList.toggle('collapsed'); }));
    $('#estop').addEventListener('click', () => sim.emergencyStop('HUD button'));
    $('#resetBtn').addEventListener('click', () => sim.resetEstop());
    $('#startBtn').addEventListener('click', () => sim.startUp());
    $('#helpBtn').addEventListener('click', () => this.toggleHelp());
    $('#qualitySel').addEventListener('change', (e) => { ui.setQuality?.(e.target.value); e.target.blur(); });
    setTimeout(() => { if (ui.quality) $('#qualitySel').value = ui.quality; }, 500);
    $('#helpClose').addEventListener('click', () => this.toggleHelp(false));
    this.root.querySelectorAll('[data-ts]').forEach((b) => b.addEventListener('click', () => { const v = +b.dataset.ts; if (v === 0) sim.paused = !sim.paused; else { sim.paused = false; sim.timeScale = v; } }));
    this.root.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => ui.setView(b.dataset.view)));
    this.root.querySelectorAll('[data-m]').forEach((b) => b.addEventListener('click', () => this.exec(`mission ${b.dataset.m} ${$('#cargoSel').value}`)));
    this.root.querySelectorAll('[data-c]').forEach((b) => b.addEventListener('click', () => this.exec(b.dataset.c)));
    this.root.querySelectorAll('[data-ov]').forEach((b) => b.addEventListener('click', () => { b.classList.toggle('on'); ui.overlays[b.dataset.ov] = b.classList.contains('on'); }));
    $('#resumeBtn').addEventListener('click', () => this.exec('resume'));
    $('#abortBtn').addEventListener('click', () => this.exec('abort'));
    $('#zIn').addEventListener('click', () => { this.mapZoom = Math.min(12, this.mapZoom * 1.5); });
    $('#zOut').addEventListener('click', () => { this.mapZoom = Math.max(0.12, this.mapZoom / 1.5); });
    $('#mapSave').addEventListener('click', () => this.exec('map save'));
    $('#mapLoad').addEventListener('click', () => this.exec('map load'));
    $('#mapClear').addEventListener('click', () => this.exec('map clear'));
    const download = (name, text, type) => { const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text], { type })); a.download = name; document.body.appendChild(a); a.click(); setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000); };
    $('#csvBtn').addEventListener('click', () => { download(`aegis-v1_telemetry_${Math.round(sim.time)}s.csv`, sim.telemetry.csv(), 'text/csv'); sim.log(`Telemetry exported (${sim.telemetry.rows.length} rows @ 1 Hz)`, 'ok'); });
    $('#repBtn').addEventListener('click', () => {
      const A = sim.autonomy, R = sim.rover;
      const rep = { generated: new Date().toISOString(), sim_time_s: sim.time, vehicle: 'AEGIS-V1', missions: A.history, current: A.summary(), docking_accuracy: A.metrics.accuracy,
        routes: sim.slam.routes.map((r) => ({ id: r.id, from: r.from, to: r.to, length_m: r.length, time_s: r.time, energy_Wh: r.energyWh, uses: r.uses })),
        odometer_m: R.odo.dist, energy_used_Wh: R.power.energyUsedWh, energy_generated_Wh: R.power.energyGenWh, min_tip_margin_deg: R.stats.minMargin,
        manifest: A.manifest.map((m) => ({ id: m.id, category: m.category, port: m.port, status: m.status })), scout: { odometer_m: sim.scout.odo, mapped_cells: sim.scout.cellsMapped } };
      download(`aegis-v1_mission_report_${Math.round(sim.time)}s.json`, JSON.stringify(rep, null, 2), 'application/json');
      sim.log('Mission report exported', 'ok');
    });
    this.mapCanvas.addEventListener('wheel', (e) => { e.preventDefault(); this.mapZoom = Math.max(0.12, Math.min(12, this.mapZoom * (e.deltaY < 0 ? 1.2 : 1 / 1.2))); }, { passive: false });
    // click on the map: autonomous goto that point
    this.mapCanvas.addEventListener('dblclick', (e) => {
      const r = this.mapCanvas.getBoundingClientRect();
      const px = (e.clientX - r.left) / r.width * this.mapCanvas.width, py = (e.clientY - r.top) / r.height * this.mapCanvas.height;
      const e0 = sim.slam.est, z = this.mapZoom * 2;
      const x = e0.x + (px - this.mapCanvas.width / 2) / z, zz = e0.z + (py - this.mapCanvas.height / 2) / z;
      this.exec(`goto ${x.toFixed(1)} ${zz.toFixed(1)}`);
    });
    const cmd = $('#cmd');
    cmd.addEventListener('keydown', (e) => {
      e.stopPropagation();
      if (e.key === 'Enter') { const v = cmd.value; cmd.value = ''; if (v.trim()) { this.history.unshift(v); this.hIdx = -1; this.exec(v); } }
      else if (e.key === 'ArrowUp') { this.hIdx = Math.min(this.history.length - 1, this.hIdx + 1); cmd.value = this.history[this.hIdx] || ''; e.preventDefault(); }
      else if (e.key === 'ArrowDown') { this.hIdx = Math.max(-1, this.hIdx - 1); cmd.value = this.hIdx >= 0 ? this.history[this.hIdx] : ''; e.preventDefault(); }
      else if (e.key === 'Escape') { if (cmd.value) cmd.value = ''; else { cmd.blur(); sim.emergencyStop('keyboard (Esc)'); } }
    });
    cmd.addEventListener('keyup', (e) => e.stopPropagation());
  }

  fillManifest() {
    const m = P1.manifest || [];
    $('#manSum').innerHTML = `${P1.total_cargo_items} items · ${P1.total_payload_mass_kg} kg (${(P1.total_payload_mass_kg * 1.62).toFixed(0)} N lunar) · CoM x ${P1.center_of_mass_vector_m.x_longitudinal} m · ${P1.stability_status}`;
    $('#manTable tbody').innerHTML = m.slice().sort((a, b) => a.Day_Number - b.Day_Number).map((it) => `<tr><td style="text-align:left;font-family:var(--sans)">${it.Item}</td><td>${it.Day_Number}</td><td>${(it.Location || '').replace(/, Tier.*/, '')}</td><td>${it.Mass_kg}</td><td style="text-align:left;font-family:var(--sans)">${it.Responsible_Actor}</td></tr>`).join('');
  }

  exec(line) {
    this.addLog({ t: this.sim.time, msg: '> ' + line, level: 'cmd' });
    runCommand(this.sim, this.ui, line);
  }
  toggleHelp(v) { const p = $('#helpPanel'); const show = v ?? p.style.display !== 'block'; p.style.display = show ? 'block' : 'none'; }

  addLog(e) {
    const log = $('#log');
    const d = document.createElement('div');
    d.className = e.level;
    d.innerHTML = `<span class="ts">${fmtT(e.t)}</span>`;
    d.appendChild(document.createTextNode(e.msg));
    log.appendChild(d);
    while (log.children.length > 300) log.removeChild(log.firstChild);
    log.scrollTop = log.scrollHeight;
  }

  camRect() { const r = $('#camFrame').getBoundingClientRect(); return $('#camPanel').classList.contains('collapsed') || getComputedStyle($('#camPanel')).display === 'none' ? null : r; }

  update(dt) {
    this._t -= dt; if (this._t > 0) return; this._t = 0.1;
    const sim = this.sim, R = sim.rover, A = sim.autonomy, S = sim.sensors, tl = R.telemetry();
    $('#camPanel').style.top = ($('#mapPanel').getBoundingClientRect().bottom + 8) + 'px';
    $('#clock').textContent = `MET ${fmtT(sim.time)}${sim.paused ? ' ❚❚' : ''} ${sim.timeScale !== 1 ? '· ' + sim.timeScale + '×' : ''}${sim.lagging ? ' (CPU-limited)' : ''}${S.decimated ? ' · sensors decimated' : ''}`;
    const pill = (id, txt, cls) => { const e = $(id); e.textContent = txt; e.className = 'pill ' + cls; };
    pill('#pMode', A.mode === 'AUTO' ? 'AUTO' : A.paused && A.gen ? 'OVERRIDE' : 'MANUAL', A.mode === 'AUTO' ? 'info' : A.paused && A.gen ? 'warn' : '');
    pill('#pDrive', R.estop ? 'E-STOP' : sim.startup ? 'SELF-TEST' : R.powered ? 'DRIVE ARMED' : 'DISARMED', R.estop ? 'err' : R.powered ? 'ok' : 'warn');
    pill('#pComms', A.faults.comms ? 'COMMS LOS' : 'COMMS OK', A.faults.comms ? 'err' : 'ok');
    pill('#pLidar', S.faults.lidar ? 'LIDAR FAIL' : S.lidarOn ? 'LIDAR' : 'LIDAR OFF', S.faults.lidar ? 'err' : S.lidarOn ? 'ok' : 'warn');
    pill('#pCam', S.faults.camera ? 'CAM FAIL' : S.cameraOn ? 'CAM' : 'CAM OFF', S.faults.camera ? 'err' : S.cameraOn ? 'ok' : 'warn');
    pill('#pSteer', `${R.rpm.proto.toFixed(0)} RPM ${R.rpm.tier.toUpperCase()}`, R.rpm.tier === 'turbo' ? 'warn' : R.rpm.tier === 'crawl' ? 'info' : '');
    $('#estop').classList.toggle('latched', R.estop);
    $('#resetBtn').disabled = !R.estop; $('#startBtn').disabled = R.estop || R.powered || !!sim.startup;
    $('#estopBanner').style.display = R.estop ? 'block' : 'none';
    $('#overrideBanner').style.display = A.paused && A.gen && !R.estop ? 'block' : 'none';
    this.root.querySelectorAll('[data-ts]').forEach((b) => b.classList.toggle('on', (+b.dataset.ts === 0 && sim.paused) || (!sim.paused && +b.dataset.ts === sim.timeScale)));
    this.root.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('on', b.dataset.view === this.ui.view));

    const e = sim.slam.est, p = tl.pose;
    const hdg = (y) => ((90 - y * 180 / Math.PI) % 360 + 360) % 360;
    $('#vSpeed').textContent = `${fmt(tl.fwdSpeed, 2)} m/s`;
    $('#vHdg').textContent = `${fmt(hdg(p.yaw), 1)}° / ${fmt(hdg(e.yaw), 1)}°`;
    $('#vAtt').textContent = `${fmt(tl.roll * 57.3, 1)}° / ${fmt(tl.pitch * 57.3, 1)}°`;
    $('#vMargin').textContent = `${fmt(tl.margin, 1)}°`;
    const mb = $('#bMargin'); mb.style.width = Math.max(0, Math.min(100, tl.margin / 50 * 100)) + '%'; mb.style.background = tl.margin < 15 ? 'var(--err)' : tl.margin < 25 ? 'var(--warn)' : 'var(--ok)';
    $('#vSoc').textContent = `${fmt(R.power.soc * 100, 1)} %  (${fmt(R.power.wh / 1000, 1)} kWh, ${fmt(R.power.voltage, 0)} V)`;
    const sb = $('#bSoc'); sb.style.width = R.power.soc * 100 + '%'; sb.style.background = R.power.soc < A.reserveSoc ? 'var(--err)' : R.power.soc < 0.4 ? 'var(--warn)' : 'var(--ok)';
    $('#vPow').textContent = `${fmt((R.power.dockW || 0) / 1000, 1)} / ${fmt(R.power.loadW / 1000, 2)} kW`;
    { const D = sim.powerDock; $('#vDock').textContent = `bank ${fmt(D.soc * 100, 0)}% · sun ${fmt(D.solarW / 1000, 0)} kW · ${D.status}`; }
    $('#vDrive').textContent = `${fmt(R.power.driveW / 1000, 2)} kW  regen ${fmt(R.power.regenWh, 1)} Wh`;
    $('#vOdo').textContent = `${fmt(R.odo.dist / 1000, 3)} km / ${fmt(R.power.energyUsedWh / 1000, 2)} kWh`;
    $('#vLoad').textContent = R.loaded ? `${R.lift.cargo.id} ${(R.lift.cargo.T.mass / 1000).toFixed(0)} t (${(R.lift.cargo.T.mass * MOON.g / 1000).toFixed(1)} kN)` : 'none';
    $('#vLift').textContent = `${fmt(R.lift.pos, 2)} m  (target ${fmt(R.lift.target, 2)})`;
    $('#vClamp').textContent = `${R.lift.clamp} ${R.lift.clamp === 'closing' || R.lift.clamp === 'opening' ? (R.lift.clampT * 100).toFixed(0) + '%' : ''}`;
    $('#vLim').textContent = `${R.rpm.proto.toFixed(0)} rpm ${R.rpm.tier} · ${R.rpm.why} · cap ${R.speedCap().toFixed(1)} m/s`;
    const names = ['FL', 'FR', 'ML', 'MR', 'RL', 'RR']; const kS = Math.sqrt(K);
    $('#wheelRows').innerHTML = R.wheels.map((w, i) => `<tr><td class="${w.fault ? 'f' : ''}">${i + 1} ${names[i]}</td><td>${fmt(w.omega * 60 / (2 * Math.PI) * kS, 0)}</td><td>${fmt(w.torque, 0)}</td><td>${fmt(w.slip * 100, 0)}%</td><td>${fmt(w.sink * 1000, 0)}</td><td>${fmt(w.load / 1000, 2)}</td><td>${fmt(w.temp, 0)}</td></tr>`).join('');
    $('#bogieRow').textContent = R.bogies.map((b) => `${b.name} ${((ROVER.bogie.Lnom - b.L) * 1000).toFixed(0)}`).join('   ');
    const contact = R.wheels.filter((w) => w.contact);
    const avgSink = contact.reduce((a, w) => a + w.sink, 0) / Math.max(1, contact.length), avgSlip = contact.reduce((a, w) => a + Math.abs(w.slip), 0) / Math.max(1, contact.length);
    const rr = contact.reduce((a, w) => a + w.Rc, 0), load = contact.reduce((a, w) => a + w.load, 0);
    $('#soilRow').textContent = `Bekker sinkage ${(avgSink * 1000).toFixed(0)} mm · |slip| ${(avgSlip * 100).toFixed(1)} % · rolling resistance coeff ${(rr / Math.max(1, load)).toFixed(3)} · contacts ${contact.length}/8`;

    // SLAM stats
    const err = Math.hypot(e.x - p.x, e.z - p.z);
    $('#sErr').textContent = `${fmt(err, 2)} m · ${fmt((((e.yaw - p.yaw) * 180 / Math.PI + 540) % 360) - 180, 2)}°`;
    const lm = sim.slam.lastMatch, dg = sim.slam.degenerate;
    const lc = sim.slam.lastLoop;
    $('#sMatch').textContent = (lm.t ? `${lm.ok ? 'locked' : 'odometry'} · ${lm.used || 0} pts${dg && (dg.x || dg.z) ? ' · degenerate' : ''}` : 'waiting for map') + (lc ? ` · loop closed ${lc.shift.toFixed(1)} m (${((sim.time - lc.t) / 60).toFixed(0)} min ago)` : '');
    $('#sArea').textContent = `${fmt(sim.slam.cellsKnown * RES * RES / 1e4, 2)} ha · ${sim.slam.routes.length} routes`;
    $('#sPlan').textContent = A.lastPlan ? `${A.pathSource.split(' ')[0]} · ${A.lastPlan.ms.toFixed(0)}+${A.lastPlan.buildMs.toFixed(0)} ms` : '—';

    // autonomy
    $('#autoMode').textContent = A.mode; $('#autoMode').className = 'pill ' + (A.mode === 'AUTO' ? 'info' : '');
    $('#autoPhase').textContent = (A.missionName ? A.missionName + ' — ' : '') + A.phase;
    $('#autoDetail').textContent = A.detail || '';
    // docking
    const D = A.docking;
    $('#dockPanel').style.display = D ? 'block' : 'none';
    if (D) {
      $('#dockSteps').innerHTML = D.steps.map((s, i) => `<li class="${i < D.step ? 'done' : i === D.step ? 'cur' : ''}">${s}</li>`).join('');
      $('#dockP').textContent = `${fmt(D.pressure, 1)} kPa`; $('#dockL').textContent = D.leakRate ? `${D.leakRate.toFixed(4)} kPa/min` : '—';
    }
    // logistics plan
    const plan = A.logisticsPlan();
    $('#planBox').innerHTML = (plan.length ? plan.map((x, i) => `${i + 1}. ${x.id} <span style="color:var(--accent)">${x.category}</span> P${PRIORITY[x.category] ?? x.priority} → PORT-${x.port} · ETA ${x.eta.toFixed(0)} min · ${(x.energy / 1000).toFixed(1)} kWh ${x.feasible ? '' : '<span style="color:var(--err)">recharge</span>'}`).join('<br>') : 'all requests served') +
      '<br>' + A.manifest.map((m) => `${m.id}: ${m.status}`).join(' · ');
    const sm = A.summary();
    const acc = sm.accuracy;
    $('#metrics').innerHTML = `<span>mission time</span><b>${fmtT(sm.time)}</b><span>distance</span><b>${fmt(sm.dist, 0)} m</b><span>energy</span><b>${fmt(sm.energyWh / 1000, 2)} kWh</b><span>deliveries</span><b>${sm.deliveries}</b><span>dock accuracy</span><b>${acc ? `${acc.lateralMm.toFixed(0)}/${acc.verticalMm.toFixed(0)} mm ${acc.yawDeg.toFixed(2)}°` : '—'}</b><span>min tip margin</span><b>${fmt(R.stats.minMargin, 1)}°</b>`;

    $('#manifestPanel').style.display = this.ui.overlays.xray ? 'block' : 'none';
    const sc = sim.scout;
    $('#scoutBox').textContent = `${sc.enabled ? sc.mode.toUpperCase() : 'PARKED'} · ${sc.msg} · ${(Math.hypot(sc.x - tl.pose.x, sc.z - tl.pose.z)).toFixed(0)} m from AEGIS-V1`;
    this.drawMap();
    this.drawCamOverlay();
    this.drawTrends();
  }

  drawTrends() {
    const cv = $('#trend'); if (!cv || $('#vehicle').classList.contains('collapsed')) return;
    const g = cv.getContext('2d'), W = cv.width, H = cv.height, T = this.sim.telemetry;
    g.clearRect(0, 0, W, H);
    g.fillStyle = 'rgba(255,255,255,0.03)'; g.fillRect(0, 0, W, H);
    const lines = [[8, 'speed m/s', '#35d0ff', 0, 2.5], [11, 'tip margin°', '#3ddc84', 0, 50], [14, 'load kW', '#ffb020', 0, 20000], [12, 'SoC %', '#c58cff', 0, 100]];
    g.font = '18px sans-serif';
    lines.forEach(([i, label, col, lo, hi], k) => {
      const s = T.series(i, 300);
      g.strokeStyle = col; g.lineWidth = 2.5; g.beginPath();
      s.forEach(([t, v], j) => { const x = W - (this.sim.time - t) / 300 * W, y = H - 6 - (Math.max(lo, Math.min(hi, v)) - lo) / (hi - lo) * (H - 12); j ? g.lineTo(x, y) : g.moveTo(x, y); });
      g.stroke();
      const last = s.length ? s[s.length - 1][1] : 0;
      g.fillStyle = col; g.fillText(`${label} ${i === 14 ? (last / 1000).toFixed(1) : last.toFixed(i === 8 ? 2 : 0)}`, 8 + (k % 2) * 280, 22 + Math.floor(k / 2) * 22);
    });
  }

  tileImage(t) {
    let c = this.tileCache.get(t);
    if (c && c.ver === t.ver && !t.dirty) return c.canvas;
    if (t.dirty) this.sim.slam.computeHazard(t);
    if (!c) { const cv = document.createElement('canvas'); cv.width = cv.height = TILE; c = { canvas: cv, ctx: cv.getContext('2d'), img: null, ver: -1 }; c.img = c.ctx.createImageData(TILE, TILE); this.tileCache.set(t, c); }
    const d = c.img.data;
    for (let i = 0; i < TILE * TILE; i++) {
      const k = i * 4;
      if (!t.n[i]) { d[k + 3] = 0; continue; }
      // hillshade from neighbouring cells (NW light)
      const x = i % TILE, z = (i / TILE) | 0;
      const h = t.h[i], hl = x > 0 && t.n[i - 1] ? t.h[i - 1] : h, hu = z > 0 && t.n[i - TILE] ? t.h[i - TILE] : h;
      const shade = Math.max(0.35, Math.min(1.35, 1 + (h - hl) * 2.2 + (h - hu) * 2.2));
      const col = CLS_COLOR[t.cls[i]] || [80, 80, 80];
      d[k] = col[0] * shade; d[k + 1] = col[1] * shade; d[k + 2] = col[2] * shade; d[k + 3] = 255;
    }
    c.ctx.putImageData(c.img, 0, 0); c.ver = t.ver;
    return c.canvas;
  }

  drawMap() {
    const sim = this.sim, ctx = this.mapCtx, W = this.mapCanvas.width, H = this.mapCanvas.height;
    const e = sim.slam.est, z = this.mapZoom * 2; // canvas px per metre (canvas is 2x css)
    ctx.fillStyle = '#05070a'; ctx.fillRect(0, 0, W, H);
    const toPx = (x, zz) => [W / 2 + (x - e.x) * z, H / 2 + (zz - e.z) * z];
    ctx.imageSmoothingEnabled = false;
    const span = TILE * RES;
    for (const t of sim.slam.tiles.values()) {
      const [px, py] = toPx(t.tx * span, t.tz * span);
      if (px > W || py > H || px + span * z < 0 || py + span * z < 0) continue;
      ctx.drawImage(this.tileImage(t), px, py, span * z + 0.5, span * z + 0.5);
    }
    // geofences
    ctx.lineWidth = 2; ctx.setLineDash([8, 6]);
    for (const r of sim.terrain.restricted) { const [cx, cy] = toPx(r.x, r.z); ctx.strokeStyle = 'rgba(255,77,58,0.8)'; ctx.beginPath(); ctx.arc(cx, cy, r.r * z, 0, Math.PI * 2); ctx.stroke(); }
    ctx.setLineDash([]);
    // routes
    ctx.strokeStyle = 'rgba(255,210,61,0.8)'; ctx.lineWidth = 2;
    for (const r of sim.slam.routes) { ctx.beginPath(); r.pts.forEach((p, i) => { const [x, y] = toPx(p.x, p.z); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); }
    // trajectory
    ctx.strokeStyle = 'rgba(255,255,255,0.35)'; ctx.lineWidth = 1.5; ctx.beginPath();
    sim.slam.trajectory.slice(-1500).forEach((p, i) => { const [x, y] = toPx(p.x, p.z); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
    // plan
    const A = sim.autonomy;
    if (A.path && A.gen) { ctx.strokeStyle = '#35d0ff'; ctx.lineWidth = 2.5; ctx.beginPath(); A.path.forEach((p, i) => { const [x, y] = toPx(p.x, p.z); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); }
    // known infrastructure
    const mark = (x, zz, label, color) => { const [px, py] = toPx(x, zz); ctx.fillStyle = color; ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.fill(); ctx.font = '20px sans-serif'; ctx.fillText(label, px + 8, py - 6); };
    mark(WORLD.pad.x, WORLD.pad.z, 'LANDER', '#c8d2de');
    for (const p of sim.ports) mark(p.x, p.z, p.name, '#3ddc84');
    for (const c of sim.cargo) { const g = A.cargoEst.get(c.id); if (g) mark(g.x, g.z, c.id, '#ffd23d'); }
    if (sim.scout.enabled) { const p0 = sim.rover.pose(); mark(sim.scout.x - p0.x + e.x, sim.scout.z - p0.z + e.z, 'SCOUT-1', '#7dffb0'); }
    // LiDAR range
    ctx.strokeStyle = 'rgba(53,208,255,0.25)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(W / 2, H / 2, ROVER.lidar.range * z, 0, Math.PI * 2); ctx.stroke();
    // true pose ghost + estimated pose
    const p = sim.rover.pose();
    const drawRover = (x, zz, yaw, fill, stroke) => {
      const [px, py] = toPx(x, zz);
      ctx.save(); ctx.translate(px, py); ctx.rotate(-yaw);
      ctx.fillStyle = fill; ctx.strokeStyle = stroke; ctx.lineWidth = 2;
      ctx.fillRect(-7.5 * z, -3.1 * z, 15 * z, 6.2 * z); ctx.strokeRect(-7.5 * z, -3.1 * z, 15 * z, 6.2 * z);
      ctx.beginPath(); ctx.moveTo(7.5 * z + 8, 0); ctx.lineTo(7.5 * z, -6); ctx.lineTo(7.5 * z, 6); ctx.closePath(); ctx.fillStyle = stroke; ctx.fill();
      ctx.restore();
    };
    drawRover(p.x, p.z, p.yaw, 'rgba(255,255,255,0.08)', 'rgba(255,255,255,0.4)');
    drawRover(e.x, e.z, e.yaw, 'rgba(53,208,255,0.25)', '#35d0ff');
    ctx.fillStyle = '#8796a8'; ctx.font = '20px sans-serif';
    const sc = Math.pow(10, Math.floor(Math.log10(120 / z))); const bar = sc * z;
    ctx.fillRect(16, H - 24, bar, 4); ctx.fillText(`${sc} m`, 16, H - 32);
    ctx.fillText('N ↑ (−Z)', W - 110, 28);
  }

  drawCamOverlay() {
    const ctx = this.camCtx, W = this.camOverlay.width, H = this.camOverlay.height, S = this.sim.sensors;
    ctx.clearRect(0, 0, W, H);
    if (S.faults.camera || !S.cameraOn) {
      ctx.fillStyle = 'rgba(0,0,0,0.85)'; ctx.fillRect(0, 0, W, H); ctx.fillStyle = '#ff4d3a'; ctx.font = 'bold 30px monospace'; ctx.fillText(S.faults.camera ? 'CAMERA FAULT — NO SIGNAL' : 'CAMERA OFF', 80, H / 2);
      $('#camInfo').textContent = '—'; return;
    }
    ctx.lineWidth = 2; ctx.font = '18px monospace';
    for (const r of S.rockDetections) {
      const w = Math.max(10, r.w * W), h = Math.max(8, r.h * H);
      ctx.strokeStyle = r.d > 1.0 ? 'rgba(255,90,60,0.9)' : 'rgba(255,190,60,0.75)';
      ctx.strokeRect(r.u * W - w / 2, r.v * H - h, w, h);
      if (r.d > 0.8) { ctx.fillStyle = ctx.strokeStyle; ctx.fillText(`rock ${r.d.toFixed(1)}m ${(r.conf * 100).toFixed(0)}%`, r.u * W - w / 2, r.v * H - h - 4); }
    }
    for (const d of S.detections) {
      const s = Math.max(18, d.size / d.range * W * 1.2);
      ctx.strokeStyle = '#3dff8a'; ctx.lineWidth = 3; ctx.strokeRect(d.u * W - s / 2, d.vv * H - s / 2, s, s);
      ctx.fillStyle = '#3dff8a'; ctx.fillText(`${d.id} ${d.range.toFixed(1)}m`, d.u * W - s / 2, d.vv * H - s / 2 - 6);
    }
    ctx.strokeStyle = 'rgba(255,255,255,0.35)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(W / 2 - 20, H / 2); ctx.lineTo(W / 2 + 20, H / 2); ctx.moveTo(W / 2, H / 2 - 20); ctx.lineTo(W / 2, H / 2 + 20); ctx.stroke();
    ctx.fillStyle = 'rgba(255,255,255,0.7)'; ctx.font = '16px monospace'; ctx.fillText(`CAM-1 · 70° HFOV · ${new Date(this.sim.time * 1000).toISOString().substr(11, 8)}`, 10, 22);
    const c = S.cass;
    $('#camInfo').textContent = `fiducials ${S.detections.length} · rocks ${S.rockDetections.length}` + (c && c.valid ? ` · CASS lat ${(c.lateral * 100).toFixed(1)} cm yaw ${(c.yaw * 57.3).toFixed(2)}°` : '') + (c && c.ring ? ` · dock ring ${c.ring.along.toFixed(2)} m` : '');
  }
}
