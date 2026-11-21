// Operator HUD: telemetry, SLAM map, camera feed overlay, missions, faults, console, E-STOP.
import { ROVER, WORLD, MOON, K, FAILURES, ARM, CARGO_TYPES } from '../sim/config.js';
import { CLS, RES, TILE } from '../sim/slam.js';
import { runCommand, requestEstop } from '../sim/commands.js';
import { DelayLine } from '../sim/commsLink.js';
import { PRIORITY } from '../sim/autonomy.js';
import P1 from '../sim/lunar_logistics_manifest.json';
import { MODE, BAYS, LRUS, bayOf, unitLabel } from '../sim/health.js';
import { Copilot } from '../ai/copilot.js';
import { Instruments, instrumentsTemplate } from './instruments.js';
import { ThermalPanel, thermalTemplate } from './thermalPanel.js';

const SUBSYS = { MOB: 'mobility', PWR: 'power', AVI: 'avionics', NAV: 'navigation sensors', COM: 'comms', THM: 'thermal', CRG: 'cargo', SAF: 'safety / dock' };
const SUBSYS_OF = { mc: 'MOB', seize: 'MOB', cool: 'MOB', enc: 'MOB', strut: 'MOB', bat: 'PWR', runaway: 'PWR', pdu: 'PWR', imd: 'PWR', fc: 'AVI', seu: 'AVI', imu: 'NAV', sun: 'NAV', lidar: 'NAV', ldust: 'NAV', cam: 'NAV', cdust: 'NAV', com: 'COM', htr: 'THM', louver: 'THM', clamp: 'CRG', lcell: 'CRG', estop: 'SAF', coupler: 'SAF' };

// first words the console understands (a line starting with anything else, 3+ words long, goes to the AI)
const KNOWN = new Set(['help', '?', 'estop', 'e-stop', 'reset', 'start', 'status', 'mission', 'abort', 'resume', 'goto', 'speed',
  'limit', 'rpm', 'turbo', 'drive', 'stop', 'brake', 'lift', 'clamp', 'level', 'ride', 'lights', 'timescale', 'ts', 'pause',
  'ff', 'fastforward', 'warp', 'newmap', 'lidar', 'camera', 'fault', 'failures', 'random', 'health', 'fdir', 'procedure',
  'proc', 'repair', 'survival', 'thermal', 'brain', 'scenarios', 'scenario', 'array', 'clear', 'reroute', 'map', 'memory', 'routes', 'plan', 'priority', 'reserve', 'sun',
  'charge', 'dock', 'day', 'night', 'lunar', 'view', 'restricted', 'scout', 'steer', 'grapple', 'newterrain', 'teleport']);

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
    // the top bar wraps onto a second row on narrower screens: publish its height so the panels start below it (style.css --tb)
    { const tb = $('#topbar'), setTb = () => document.documentElement.style.setProperty('--tb', tb.offsetHeight + 'px');
      setTb(); if (typeof ResizeObserver !== 'undefined') new ResizeObserver(setTb).observe(tb); else window.addEventListener('resize', setTb); }
    this.inst = new Instruments(this.root);
    this.therm = new ThermalPanel(this.root, sim);
    this.mapZoom = 1.6; // px per metre
    this.mapCanvas = $('#map'); this.mapCtx = this.mapCanvas.getContext('2d');
    this.mapCanvas.width = 636; this.mapCanvas.height = 636;
    this.tileCache = new Map();
    this.camOverlay = $('#camOverlay'); this.camCtx = this.camOverlay.getContext('2d');
    this.camOverlay.width = 636; this.camOverlay.height = 476;
    this.history = []; this.hIdx = -1;
    this.bind();
    this.initLayers();
    this.fillManifest();
    sim.on('log', (e) => this.addLog(e));
    for (const e of sim.logs) this.addLog(e);
    this._t = 0;
    this.initAI();
  }

  // Local LLM copilot (src/ai/copilot.js). ?noai disables it; ?ai=<ollama url> points it elsewhere
  // (that Ollama must then allow this page's origin via OLLAMA_ORIGINS).
  initAI() {
    const q = new URLSearchParams(location.search);
    if (q.has('noai')) return;
    this.ai = new Copilot(this.sim, {
      ...(q.get('ai') ? { url: q.get('ai') } : {}), ...(q.get('aimodel') ? { model: q.get('aimodel') } : {}),
      exec: (c) => runCommand(this.sim, this.ui, c),
    });
    this.ai.probe().then((r) => this.sim.log(r.ok
      ? (this.ai.hasModel ? `AI copilot online (${this.ai.o.model}). Type "ai <question>" or plain English; FDIR advisor on.` : `AI copilot: Ollama reachable but ${this.ai.o.model} is not downloaded`)
      : `AI copilot offline (${r.why}): the sim runs normally without it`, r.ok && this.ai.hasModel ? 'ai' : 'warn'));
  }

  template() {
    const cargoOpts = this.sim.cargo.map((c, i) => `<option value="${c.id.slice(-1)}">${c.id} · ${c.T.label}</option>`).join('');
    return `
    <div id="topbar" class="panel">
      <div class="brand">AEGIS-V1<span class="brandx"> · LLASO-P2-XPORT-2026</span><small>Rocker-bogie skid-steer cargo carrier · lunar south-pole logistics</small></div>
      <span class="pill mono" id="clock">MET 00:00:00</span>
      <span class="pill" id="pMode">MANUAL</span>
      <span class="pill" id="pDrive">ARMED</span>
      <span class="pill eng" id="pComms">COMMS</span>
      <span class="pill eng" id="pLidar">LIDAR</span>
      <span class="pill eng" id="pCam">CAM</span>
      <span class="pill eng" id="pSteer">SKID STEER</span>
      <button class="pill eng" id="pHealth" title="Vehicle health / FDIR panel">HEALTH</button>
      <button class="pill eng" id="pTherm" title="Thermal brain panel (T)">THERMAL</button>
      <span class="pill eng" id="pAI" title="Local LLM copilot on the Mac mini (type: ai help)">AI</span>
      <div class="grow"></div>
      <div class="btnrow timebtns" id="timeBtns">
        ${[0, 1, 2, 3, 5, 10, 20].map((x) => `<button class="btn" data-ts="${x}" title="${x === 0 ? 'Pause (P)' : x >= 5 ? 'Time scale ' + x + '×: long cruise legs over mapped ground run on the transit-warp rails; pick-up, docking and unmapped ground run full physics as fast as the CPU allows' : 'Time scale ' + x + '× (full physics)'}">${x === 0 ? '❚❚' : x + '×'}</button>`).join('')}<button class="btn ff" data-ts="50" title="Fast forward: 50x sim time (X). Long autonomous drives use transit warp; precision steps run full physics as fast as the CPU allows">⏩ 50×</button>
      </div>
      <div class="btnrow" id="quickMission">
        <button class="btn on" data-m="6" title="Full logistics cycle: cargo CM-40A first, then the rest of the manifest">M6 Full cycle</button>
        <button class="btn" data-c="resume">resume</button><button class="btn danger" data-c="abort">abort</button>
      </div>
      <div class="btnrow" id="layerBtns">
        <button class="btn" id="hudBtn" title="Cycle the HUD layers: essential, engineering, hidden (H)">☰ HUD</button>
        <button class="btn" id="hideHudBtn" title="Hide the whole HUD (H or the ☰ button brings it back)">Hide All HUD</button>
      </div>
      <div class="btnrow eng" id="viewBtns">
        ${['chase', 'orbit', 'top', 'cam', 'free'].map((v) => `<button class="btn" data-view="${v}">${v === 'cam' ? 'onboard' : v}</button>`).join('')}
      </div>
      <select class="btn eng" id="qualitySel" title="Graphics quality"><option value="auto">GFX auto</option><option value="low">GFX low</option><option value="medium">GFX medium</option><option value="high">GFX high</option><option value="ultra">GFX ultra</option></select>
      <button class="btn eng" id="helpBtn">? help</button>
      <div id="safety"><div id="restart"><button class="btn danger" id="resetBtn" title="Release the E-STOP latch">RESET</button><button class="btn" id="startBtn" title="Power-on self-test and arm drive">START</button></div>
      <button id="estop" title="EMERGENCY STOP (Esc)">E-STOP</button></div>
    </div>
    <button id="hudMenu" class="btn" title="Show the HUD (H)">☰</button>
    <div id="warpIndicator" class="warp-badge">▶▶ 10× WARP</div>
    <div id="essential">
      <div id="eDrive" class="panel essw">
        <div class="eRow"><span class="eBig mono" id="eSpeed">0.00</span><span class="eUnit">m/s</span><span class="rpmBadge" id="eRpm">CRAWL</span></div>
        <div class="eSub mono" id="eRpmWhy">—</div>
        <div class="eRow eTime"><span class="k">Time</span><span class="mono" id="eWarp">1×</span></div>
      </div>
      <div id="ePower" class="panel essw">
        <div class="eRow"><span class="k">Battery</span><span class="mono" id="eSoc">0 %</span></div>
        <div class="bar"><i id="eSocBar"></i></div>
        <div class="eRow"><span class="k">Power draw</span><span class="mono" id="ePow">0 kW</span></div>
        <div class="eRow"><span class="k">Endurance</span><span class="mono" id="eEnd">—</span></div>
      </div>
      <div id="eMap" class="panel essw"><canvas id="miniMap" width="200" height="200"></canvas></div>
      <div id="eGuide" class="panel essw"><div class="eGuideMain" id="eGuideMain">IDLE</div><div class="eSub mono" id="eGuideDetail"></div></div>
      <div id="eAlarm" class="panel"></div>
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
        <span class="k" title="Dock charge power / vehicle electrical load">Charge / load</span><span class="v" id="vPow">0</span>
        <span class="k">Power dock</span><span class="v" id="vDock">-</span>
        <span class="k" title="Prototype (1:16.3) inductive charge into the 12.8 V 10 Ah pack; x3281 = full-scale">Dock → rover</span><span class="v" id="vDockChg">-</span>
        <span class="k">Drive power</span><span class="v" id="vDrive">0</span>
        <span class="k">Odometer / energy</span><span class="v" id="vOdo">0</span>
        <span class="k">Payload</span><span class="v" id="vLoad">none</span>
        <span class="k" title="Strut ride height: kneel -0.65 m / carry +0.82 m">Ride height</span><span class="v" id="vLift">0</span>
        <span class="k">Cradle</span><span class="v" id="vClamp">open</span>
        <span class="k" title="Smart rpm limit and the reason for it">Smart rpm</span><span class="v" id="vLim">0</span>
      </div>
      <table class="wheels"><thead><tr><th>whl</th><th>mtr rpm</th><th>T Nm</th><th>slip</th><th>sink mm</th><th>load kN</th><th>°C</th></tr></thead><tbody id="wheelRows"></tbody></table>
      <div class="sect" id="armSect"><div class="t">Arms · ACM handling</div>
        <div class="kv small">
          <span class="k">Mode</span><span class="v" id="aMode">—</span>
          <span class="k">Deck latch</span><span class="v" id="aDeck">—</span>
          <span class="k">Payload</span><span class="v" id="aLoad">none</span>
          <span class="k" title="ASME B30.5 style: lifts are refused above 0.75">Tip ratio</span><span class="v" id="aTip">—</span>
          <span class="k" title="available joint torque / required, at the current pose">Lift margin</span><span class="v" id="aMargin">—</span>
          <span class="k">Faults</span><span class="v" id="aFault">none</span>
        </div>
        <table class="wheels" id="armTable"><thead><tr><th style="text-align:left">joint</th><th>angle °</th><th title="joint torque as % of servo stall">τ % stall</th><th>I A</th><th>T °C</th></tr></thead><tbody id="armRows"></tbody></table>
        <div class="kv small" style="margin-top:4px"><span class="k">Hook · gate · load (port)</span><span class="v" id="aHookP">—</span><span class="k">Hook · gate · load (stbd)</span><span class="v" id="aHookS">—</span></div>
      </div>
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
      <div class="btnrow" style="margin-top:6px"><button class="btn" id="resumeBtn">resume</button><button class="btn danger" id="abortBtn">abort</button><button class="btn" data-c="goto hab">goto hab</button><button class="btn" data-c="goto pad">goto pad</button><button class="btn" data-c="charge">charge</button><button class="btn" data-c="lunar on">lunar cycle</button><button class="btn" data-c="night">night</button><button class="btn" data-c="day">day</button><button class="btn" data-c="newmap" title="Reload with a new random crater / boulder layout">new map</button></div>
      <div class="sect"><div class="t">M9 · SCOUT-1 (team tracked rover)</div>
        <div class="btnrow"><button class="btn" data-c="scout on">deploy</button><button class="btn" data-c="scout recall">recall</button><button class="btn" data-c="scout off">park</button></div>
        <div class="small mono" id="scoutBox" style="margin-top:4px">parked</div></div>
      <div class="sect"><div class="t">M10 · inject a fault</div>
        <div class="btnrow">
          <button class="btn" data-c="fault block">block road</button><button class="btn" data-c="fault lidar">LiDAR fail</button><button class="btn" data-c="fault camera">camera fail</button>
          <button class="btn" data-c="fault comms">comms loss</button><button class="btn" data-c="fault battery 18">low battery</button><button class="btn" data-c="fault wheel 3">wheel 3 fail</button>
          <button class="btn" data-c="fault shift">cargo shift</button><button class="btn" data-c="fault port 1">port 1 down</button><button class="btn" data-c="clear faults">clear all</button>
        </div>
        <div class="btnrow" style="margin-top:4px">
          <button class="btn" data-c="fault seize 5">wheel 5 seize</button><button class="btn" data-c="fault bat 2">battery string 2</button><button class="btn" data-c="fault fc A">flight computer A</button>
          <button class="btn" data-c="fault imu A">IMU A</button><button class="btn" data-c="fault ldust">LiDAR dust</button><button class="btn" data-c="fault runaway 3">cell overheat</button>
        </div>
        <div class="btnrow" style="margin-top:6px"><button class="btn rndsw" id="rndBtn" title="Random hardware failures (seeded), each unit at rate x / MTBF">RANDOM FAILURES: OFF</button>
          <select class="btn" id="rndRate" title="Failure rate multiplier"><option value="realistic">1x (realistic)</option><option value="100x">100x</option><option value="1000x">1000x</option><option value="demo" selected>demo (20000x)</option></select>
          <button class="btn" id="fdirBtn">FDIR panel</button></div></div>
      <div class="sect"><div class="t">M11 · logistics manager plan</div><div id="planBox" class="small mono"></div></div>
      <div class="sect"><div class="t">Mission metrics</div><div class="metric" id="metrics"></div></div>
      <div class="sect"><div class="t">Overlays</div><div class="btnrow">
        <button class="btn on" data-ov="lidar">LiDAR</button><button class="btn on" data-ov="path">path</button><button class="btn" data-ov="arcs">arcs</button><button class="btn on" data-ov="routes">routes</button><button class="btn on" data-ov="fences">geofences</button><button class="btn on" data-ov="dust">dust</button><button class="btn" data-ov="xray">X-ray cargo</button><button class="btn on" data-ov="eng" title="Engineering overlay (I): wheel forces, CoM, support polygon, predicted path, braking distance + dynamics instruments">physics</button>
      </div></div>
    </div></div>

    <div id="autoPanel" class="panel"><span class="pill" id="autoMode">MANUAL</span><span class="phase" id="autoPhase">Idle</span><span class="detail" id="autoDetail"></span></div>
    <div id="console" class="panel"><div id="log"></div><div id="cmdline"><span>&gt;</span><input id="cmd" autocomplete="off" spellcheck="false" placeholder='type a command ("mission cycle", "help") or ask the AI in plain English' /></div></div>
    ${instrumentsTemplate()}
    ${thermalTemplate()}
    <div id="dockPanel" class="panel"><h3>Berthing sequence</h3><div class="body"><ol id="dockSteps"></ol><div class="kv small" style="margin-top:6px"><span class="k">Vestibule pressure</span><span class="v" id="dockP">0</span><span class="k">Leak rate</span><span class="v" id="dockL">—</span></div></div></div>
    <div id="manifestPanel" class="panel"><h3>CM-40A interior · LLASO-P1 manifest <span class="tog">▾</span></h3><div class="body">
      <div class="small" id="manSum"></div>
      <table class="wheels" id="manTable"><thead><tr><th style="text-align:left">item</th><th>day</th><th>bay</th><th>kg</th><th style="text-align:left">actor</th></tr></thead><tbody></tbody></table>
    </div></div>
    <div id="healthPanel" class="panel"><h3>Vehicle health · FDIR <span class="tog" id="healthClose">✕</span></h3><div class="body">
      <div class="btnrow"><button class="btn rndsw" id="rndBtn2">RANDOM FAILURES: OFF</button><span class="small mono" id="hStats"></span></div>
      <div class="subsys" id="hSubs"></div>
      <div class="kv small" id="hRed"></div>
      <div class="sect"><div class="t">Thermal · survival · keep-alive array</div><div class="kv small" id="hTherm"></div>
        <div class="btnrow" style="margin-top:4px"><button class="btn" data-c="survival on">survival on</button><button class="btn" data-c="survival off">survival off</button><button class="btn" data-c="array deploy">deploy array</button><button class="btn" data-c="array stow">stow array</button></div></div>
      <div class="sect"><div class="t">Alarms (click a row: procedure card + part highlighted on the rover)</div><table class="wheels alarms"><thead><tr><th>sev</th><th style="text-align:left">unit · failure</th><th style="text-align:left">bay</th><th>since</th><th></th></tr></thead><tbody id="hAlarms"></tbody></table></div>
      <div class="sect" id="hJobBox"><div class="t">Repair in progress</div><div class="small mono" id="hJob"></div><div class="bar"><i id="hJobBar"></i></div><button class="btn danger" data-c="repair abort">abort repair</button></div>
      <div class="sect"><div class="t">Procedure card</div><div class="small proc" id="hProc">Select an alarm.</div></div>
      <div class="sect"><div class="t">Spares (on-board kit · dock stores)</div><div class="small mono" id="hSpares"></div></div>
    </div></div>
    <div id="overrideBanner" class="panel">OPERATOR OVERRIDE — autonomy paused · type "resume" or press the resume button</div>
    <div id="estopBanner" class="panel"><div class="big">E-STOP LATCHED</div><div style="margin-top:6px">Drive torque removed · spring brakes set · actuators locked</div><div class="small" style="margin-top:6px">Manual restart required: press <b>RESET</b>, then <b>START</b> (power-on self-test)</div></div>
    <div id="helpPanel" class="panel"><h3>Controls & commands <span class="tog" id="helpClose">✕</span></h3><div class="body">
      <div><b>Driving (manual)</b><br><kbd>W</kbd>/<kbd>S</kbd> forward / reverse (hold)<br><kbd>A</kbd>/<kbd>D</kbd> steer (skid steer: curves while moving, slow point-turn when stopped and unloaded)<br><kbd>Space</kbd> brake · <kbd>Shift</kbd> slow / precise<br>Smart rpm sets the limit itself, 120 to 280 (the reason is shown under Vehicle state); hold full throttle straight on flat ground for TURBO<br>Gamepad: left stick drive/steer</div>
      <div><b>Cargo handling</b><br><kbd>G</kbd> close / open ring jaws<br><kbd>R</kbd>/<kbd>F</kbd> ride height up / down (kneel under a module, rise to lift it)</div>
      <div><b>Safety</b><br><kbd>Esc</kbd> E-STOP (latching). Restart = RESET then START.<br>Touching the drive keys during a mission pauses autonomy (operator override).</div>
      <div><b>View</b><br><kbd>C</kbd> cycle camera · mouse drag orbit · wheel zoom<br><kbd>V</kbd> LiDAR overlay · <kbd>I</kbd> physics overlay + instruments · <kbd>T</kbd> thermal brain panel · <kbd>P</kbd> pause · <kbd>X</kbd> fast forward 50× · <kbd>H</kbd> cycle the HUD (essential → engineering → hidden) · <kbd>?</kbd> this help<br><kbd>Enter</kbd> focus console · <kbd>↑</kbd> history</div>
      <div><b>Useful commands</b><br>mission cycle · mission 3 B · goto hab<br>limit rpm 120 · rpm 240 · rpm auto · turbo on/off/auto<br>drive 0.5 0.05 · stop<br>lift kneel/carry · clamp close<br>fault block · fault wheel 2 · clear faults<br>map save · routes · plan · timescale 10</div>
    </div></div>`;
  }

  bind() {
    const sim = this.sim, ui = this.ui;
    // phones / short landscape screens: the three side panels start as a strip of headers and open one at a
    // time (same query as the compact layout in style.css)
    const phone = matchMedia('(max-width: 760px), (max-height: 520px)');
    const side = ['#vehicle', '#mapPanel', '#missions'].map((s) => $(s));
    const fitPhone = () => { if (phone.matches) side.forEach((p) => p.classList.add('collapsed')); };
    fitPhone(); phone.addEventListener?.('change', fitPhone);
    this.root.querySelectorAll('.panel h3').forEach((h) => h.addEventListener('click', (e) => {
      if (e.target.id === 'helpClose' || e.target.id === 'healthClose') return;
      const p = h.parentElement; p.classList.toggle('collapsed');
      if (phone.matches && !p.classList.contains('collapsed') && side.includes(p)) side.forEach((q) => { if (q !== p) q.classList.add('collapsed'); });
    }));
    $('#estop').addEventListener('click', () => requestEstop(sim, 'HUD button'));
    $('#resetBtn').addEventListener('click', () => sim.resetEstop());
    $('#startBtn').addEventListener('click', () => sim.startUp());
    $('#helpBtn').addEventListener('click', () => this.toggleHelp());
    $('#qualitySel').addEventListener('change', (e) => { ui.setQuality?.(e.target.value); e.target.blur(); });
    setTimeout(() => { if (ui.quality) $('#qualitySel').value = ui.quality; }, 500);
    $('#helpClose').addEventListener('click', () => this.toggleHelp(false));
    // health / FDIR panel and the random-failure switch
    const toggleHealth = (v) => { const p = $('#healthPanel'); const show = v ?? p.style.display !== 'block'; p.style.display = show ? 'block' : 'none'; this._hT = 0; };
    this.toggleHealth = toggleHealth;
    $('#pHealth').addEventListener('click', () => toggleHealth());
    // The thermal panel starts as a header strip (open, it and the instruments covered the rover on a 1600 px screen).
    // T / the THERMAL pill: closed or hidden -> open; open -> back to the header strip.
    this.toggleThermal = () => { const p = $('#thermPanel'), open = p.style.display !== 'none' && !p.classList.contains('collapsed'); p.style.display = ''; p.classList.toggle('collapsed', open); this.therm._t = 0; };
    $('#pTherm').addEventListener('click', () => this.toggleThermal());
    $('#fdirBtn').addEventListener('click', () => toggleHealth(true));
    $('#healthClose').addEventListener('click', () => toggleHealth(false));
    const rnd = () => this.exec(sim.health.random.on ? 'failures off' : `failures on ${$('#rndRate').value}`);
    $('#rndBtn').addEventListener('click', rnd); $('#rndBtn2').addEventListener('click', rnd);
    $('#rndRate').addEventListener('change', (e) => { if (sim.health.random.on) this.exec(`failures on ${e.target.value}`); e.target.blur(); });
    $('#hAlarms').addEventListener('click', (e) => {
      const b = e.target.closest('[data-rep]'); if (b) { this.exec(`repair ${b.dataset.rep}`); return; }
      const tr = e.target.closest('tr[data-k]'); if (tr) { sim.health.selected = tr.dataset.k; this._hT = 0; }
    });
    this.root.querySelectorAll('[data-ts]').forEach((b) => b.addEventListener('click', () => {
      const v = +b.dataset.ts; if (v === 0) sim.paused = !sim.paused; else { sim.paused = false; sim.timeScale = v; }
      this.syncTimeUi(); // the clicked button lights up at once, not at the next 10 Hz HUD tick
    }));
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
        manifest: A.manifest.map((m) => ({ id: m.id, category: m.category, port: m.port, status: m.status })), scout: { odometer_m: sim.scout.odo, mapped_cells: sim.scout.cellsMapped },
        health: { random_failures: sim.health.random, stats: sim.health.stats, events: sim.health.history, open_alarms: sim.health.summary(), redundancy: sim.health.redund, spares: sim.health.spares, vault_C: sim.health.th.Tv, battery_C: sim.health.th.Tb } };
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
      else if (e.key === 'Escape') { if (cmd.value) cmd.value = ''; else { cmd.blur(); requestEstop(this.sim, 'keyboard (Esc)'); } }
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
    const w = line.trim().split(/\s+/);
    if (this.ai && w[0].toLowerCase() === 'ai') { this.ai.command(w.slice(1)); return; }
    // plain English (3+ words, not a known command) goes to the copilot instead of "Unknown command"
    if (this.ai && this.ai.online && w.length >= 3 && !KNOWN.has(w[0].toLowerCase())) { this.ai.ask(line); return; }
    runCommand(this.sim, this.ui, line);
  }
  // ---- three-layer HUD (H / ☰ HUD): 0 clean (canvas + a faint ☰), 1 essential (default), 2 full engineering ----
  initLayers() {
    let l = 1; try { const v = +localStorage.getItem('aegis-hud-layer'); if ([0, 1, 2].includes(v) && localStorage.getItem('aegis-hud-layer') != null) l = v; } catch (e) { /* private mode */ }
    this.setLayer(l, true);
    $('#hudBtn').addEventListener('click', () => this.cycleLayer());
    $('#hideHudBtn').addEventListener('click', () => this.setLayer(0));
    $('#hudMenu').addEventListener('click', () => this.setLayer(1));
  }
  setLayer(l, quiet = false) {
    this.layer = l; this.root.dataset.layer = String(l); this._t = 0; // redraw the layer's widgets at the next frame
    const b = $('#hudBtn'); if (b) b.textContent = l === 1 ? '☰ HUD 1/2' : l === 2 ? '☰ HUD 2/2' : '☰ HUD';
    try { localStorage.setItem('aegis-hud-layer', String(l)); } catch (e) { /* private mode */ }
    if (!quiet) this.sim.log(`HUD layer ${l}: ${['clean view', 'essential', 'full engineering'][l]}`, 'info');
  }
  cycleLayer() { this.setLayer({ 1: 2, 2: 0, 0: 1 }[this.layer] ?? 1); } // 1 -> 2 -> 0 -> 1

  // time-scale buttons and the warp badge (top-right)
  syncTimeUi() {
    const sim = this.sim;
    this.root.querySelectorAll('[data-ts]').forEach((b) => b.classList.toggle('on', (+b.dataset.ts === 0 && sim.paused) || (!sim.paused && +b.dataset.ts === sim.timeScale)));
    const w = $('#warpIndicator'), ts = sim.timeScale, paused = sim.paused;
    w.style.display = paused || ts !== 1 ? 'block' : 'none';
    w.classList.toggle('paused', paused);
    w.textContent = paused ? '❚❚ PAUSED' : ts > 1 ? `▶▶ ${ts}× WARP` : `◀ ${ts}× SLOW`;
    w.classList.toggle('slow', !paused && ts < 1);
  }

  updateEssential(tl) {
    const sim = this.sim, R = sim.rover, A = sim.autonomy, H = sim.health;
    const e = sim.slam.est;
    $('#eSpeed').textContent = fmt(Math.abs(tl.fwdSpeed), 2);
    const tier = R.rpm.tier, rb = $('#eRpm'); rb.textContent = tier.toUpperCase(); rb.className = 'rpmBadge ' + tier;
    $('#eRpmWhy').textContent = `${R.rpm.proto.toFixed(0)} rpm · ${R.rpm.why}`;
    { const rails = sim.warp.active && sim.timeScale >= sim.warp.minScale; // say why the actual rate is what it is
      $('#eWarp').textContent = sim.paused ? 'PAUSED' : `${sim.timeScale}×${sim.timeScale > 1 ? ` (actual ${sim.actualRate >= 10 ? sim.actualRate.toFixed(0) : sim.actualRate.toFixed(1)}×${rails ? ', transit warp' : sim.lagging ? ', CPU-limited' : ''})` : ''}`; }
    const seen = this.seen || R.power;
    $('#eSoc').textContent = `${fmt(seen.soc * 100, 0)} %  ·  ${fmt(seen.wh / 1000, 0)} kWh`;
    const sb = $('#eSocBar'); sb.style.width = seen.soc * 100 + '%'; sb.style.background = seen.soc < A.reserveSoc ? 'var(--err)' : seen.soc < 0.4 ? 'var(--warn)' : 'var(--ok)';
    const loadW = R.power.loadW || 0;
    $('#ePow').textContent = `${fmt(loadW / 1000, 2)} kW`;
    // endurance: energy above the reserve at the present electrical load (drive + hotel)
    const usable = Math.max(0, R.power.wh - A.reserveSoc * ROVER.battery.capacityWh), hrs = loadW > 50 ? usable / loadW : Infinity;
    $('#eEnd').textContent = !Number.isFinite(hrs) ? '— (idle)' : hrs >= 99 ? '> 99 h' : `${Math.floor(hrs)} h ${String(Math.round((hrs % 1) * 60)).padStart(2, '0')} min`;
    // mission guidance:  PHASE -> DESTINATION | distance remaining
    const g = A.goal, auto = !!A.gen;
    const dist = auto && g ? Math.hypot(g.x - e.x, g.z - e.z) : null;
    const phase = auto ? (A.phase || 'Active') : 'IDLE';
    $('#eGuideMain').textContent = `${phase.toUpperCase()}${auto && g?.name ? ` → ${g.name}` : ''}${dist != null ? `  |  ${Math.round(dist).toLocaleString('en-US')} m` : ''}`;
    $('#eGuideDetail').textContent = auto ? (A.missionName ? A.missionName + ' · ' : '') + (A.detail || '') : 'no active mission: pick a mission (M6 Full cycle)';
    // alarms: banner only while something is wrong
    const msgs = [];
    if (R.estop) msgs.push('E-STOP LATCHED');
    for (const a of [...H.alarms.values()].sort((x, y) => ({ critical: 0, warning: 1, caution: 2 }[MODE[x.id].sev] - { critical: 0, warning: 1, caution: 2 }[MODE[y.id].sev])).slice(0, 3)) msgs.push(`${unitLabel(MODE[a.id], a.n)} · ${MODE[a.id].name}`);
    if (H.survival.on) msgs.push('SURVIVAL MODE');
    if (R.power.soc < A.reserveSoc) msgs.push(`battery ${fmt(R.power.soc * 100, 0)} % under the ${(A.reserveSoc * 100).toFixed(0)} % reserve`);
    if (A.faults.comms) msgs.push('COMMS LOS');
    if (sim.sensors.faults.lidar) msgs.push('LIDAR FAIL');
    const al = $('#eAlarm'); al.style.display = msgs.length ? 'block' : 'none';
    if (msgs.length) { al.textContent = '⚠ ' + msgs.join('  ·  '); al.classList.toggle('crit', R.estop || [...H.alarms.values()].some((a) => MODE[a.id].sev === 'critical')); }
    this.drawMini();
  }

  // 200 x 200 px overview: pad -> habitat route, the plan, remembered routes, the trail and the rover (SLAM estimate)
  drawMini() {
    const sim = this.sim, cv = $('#miniMap'), ctx = cv.getContext('2d'), W = cv.width, Hh = cv.height, A = sim.autonomy;
    const P = WORLD.pad, B = WORLD.habitat, cx = (P.x + B.x) / 2, cz = (P.z + B.z) / 2;
    const sc = (Math.min(W, Hh) - 36) / Math.max(Math.abs(B.x - P.x), Math.abs(B.z - P.z), 1);   // px per metre, route fills the frame
    const px = (x, z) => [W / 2 + (x - cx) * sc, Hh / 2 + (z - cz) * sc];
    ctx.fillStyle = '#05070a'; ctx.fillRect(0, 0, W, Hh);
    ctx.strokeStyle = 'rgba(140,170,210,0.18)'; ctx.lineWidth = 1; ctx.strokeRect(0.5, 0.5, W - 1, Hh - 1);
    ctx.setLineDash([4, 4]); ctx.strokeStyle = 'rgba(255,255,255,0.22)'; ctx.beginPath(); ctx.moveTo(...px(P.x, P.z)); ctx.lineTo(...px(B.x, B.z)); ctx.stroke(); ctx.setLineDash([]);
    ctx.strokeStyle = 'rgba(255,210,61,0.7)'; ctx.lineWidth = 1.5;
    for (const r of sim.slam.routes) { ctx.beginPath(); r.pts.forEach((q, i) => { const [x, y] = px(q.x, q.z); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); }
    const tr = sim.slam.trajectory; ctx.strokeStyle = 'rgba(255,255,255,0.4)'; ctx.lineWidth = 1; ctx.beginPath();
    for (let i = Math.max(0, tr.length - 1500), j = 0; i < tr.length; i += 4, j++) { const [x, y] = px(tr[i].x, tr[i].z); j ? ctx.lineTo(x, y) : ctx.moveTo(x, y); } ctx.stroke();
    if (A.path && A.gen) { ctx.strokeStyle = '#35d0ff'; ctx.lineWidth = 2; ctx.beginPath(); for (let i = 0; i < A.path.length; i += 3) { const [x, y] = px(A.path[i].x, A.path[i].z); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); } ctx.stroke(); }
    const dot = (x, z, col, label) => { const [a, b] = px(x, z); ctx.fillStyle = col; ctx.beginPath(); ctx.arc(a, b, 3.5, 0, 7); ctx.fill(); if (label) { ctx.font = '10px sans-serif'; ctx.textAlign = a > W - 40 ? 'right' : 'left'; ctx.fillText(label, a > W - 40 ? a - 6 : a + 6, b - 4); ctx.textAlign = 'left'; } };
    dot(P.x, P.z, '#c8d2de', 'PAD'); dot(B.x, B.z, '#3ddc84', 'HAB');
    for (const c of sim.cargo) { const g = A.cargoEst.get(c.id); if (g && c.state !== 'held') dot(g.x, g.z, '#ffd23d'); }
    const e = sim.slam.est, [rx, ry] = px(e.x, e.z);
    ctx.save(); ctx.translate(rx, ry); ctx.rotate(-e.yaw); ctx.fillStyle = '#35d0ff'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(8, 0); ctx.lineTo(-5, -5); ctx.lineTo(-5, 5); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
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

  // ---- vehicle health / FDIR panel (2 Hz) ----
  updateHealth() {
    const H = this.sim.health, R = this.sim.rover;
    const nCrit = [...H.alarms.values()].filter((a) => MODE[a.id].sev === 'critical').length;
    const hp = $('#pHealth');
    hp.textContent = H.survival.on ? 'SURVIVAL MODE' : H.job ? 'REPAIRING' : H.alarms.size ? `${H.alarms.size} ALARM${H.alarms.size > 1 ? 'S' : ''}` : 'HEALTH OK';
    hp.className = 'pill eng ' + (nCrit || H.survival.on ? 'err' : H.alarms.size || H.job ? 'warn' : 'ok');
    for (const id of ['#rndBtn', '#rndBtn2']) { const b = $(id); b.textContent = `RANDOM FAILURES: ${H.random.on ? 'ON · ' + H.random.preset : 'OFF'}`; b.classList.toggle('on', H.random.on); }
    if ($('#healthPanel').style.display !== 'block') return;
    this._hT = (this._hT || 0) - 0.1; if (this._hT > 0) return; this._hT = 0.5;
    const st = H.stats;
    $('#hStats').textContent = `failures ${st.injected} (${st.random} random) · detected ${st.detected}${st.detected ? ` (mean ${(st.latSum / st.detected).toFixed(1)} s)` : ''} · spurious ${st.spurious} · repaired ${st.repaired}`;
    const worst = (sub) => { let s = 0; for (const a of H.alarms.values()) if (SUBSYS_OF[a.id] === sub) s = Math.max(s, { caution: 1, warning: 2, critical: 3 }[MODE[a.id].sev]); return s; };
    $('#hSubs').innerHTML = Object.entries(SUBSYS).map(([k, n]) => { const s = worst(k); return `<span class="pill ${['ok', 'warn', 'warn', 'err'][s]}" title="${n}">${k}</span>`; }).join('');
    $('#hRed').innerHTML = [
      ['Flight computer', `FC-${H.redund.fc} in command${H.isFailed('fc', 'A') || H.isFailed('fc', 'B') ? ' (no backup)' : ' · backup ready'}`],
      ['IMU · radio · heaters', `IMU-${H.redund.imu} · radio-${H.redund.com} · zone ctrl ${H.redund.htr}`],
      ['Battery strings', `${4 - H.outStrings.size}/4 on line (${fmt(R.power.wh / 1000, 0)} kWh, cap ${fmt(ROVER.battery.capacityWh / 1000 * (R.power.capK ?? 1), 0)} kWh)`],
      ['Speed derate (FDIR)', `${fmt(H.fx.speedK * 100, 0)} %`],
    ].map(([k, v]) => `<span class="k">${k}</span><span class="v">${v}</span>`).join('');
    const th = H.th, a = H.array;
    $('#hTherm').innerHTML = [
      ['Avionics vault', `${fmt(th.Tv, 1)} C · louvers L ${fmt(th.louver.L * 100, 0)}% R ${fmt(th.louver.R * 100, 0)}%${Object.keys(th.stuck).length ? ' (STUCK ' + Object.keys(th.stuck).join(',') + ')' : ''}`],
      ['Battery pack', `${fmt(th.Tb, 1)} C · louver ${fmt(th.louverB * 100, 0)}%`],
      ['Heaters', `${fmt(H.heaterW, 0)} W (vault ${fmt(th.vaultW, 0)} · battery ${fmt(th.battW, 0)}) · radiators reject ${fmt(th.rejW, 0)} W`],
      ['Survival mode', H.survival.on ? `ON since ${fmtT(H.survival.t0)} (${H.survival.why})` : `off (auto ${H.survival.auto ? `below ${(FAILURES.survivalEnterSoc * 100).toFixed(0)} % away from the dock` : 'disabled'})`],
      ['Keep-alive array', `${a.state}${a.state === 'deploying' || a.state === 'stowing' ? ' ' + fmt(a.k * 100, 0) + '%' : ''} · ${fmt(a.W / 1000, 2)} kW`],
    ].map(([k, v]) => `<span class="k">${k}</span><span class="v">${v}</span>`).join('');
    const rows = [...H.alarms.values()].sort((x, y) => ({ critical: 0, warning: 1, caution: 2 }[MODE[x.id].sev] - { critical: 0, warning: 1, caution: 2 }[MODE[y.id].sev]));
    $('#hAlarms').innerHTML = rows.length ? rows.map((al) => {
      const m = MODE[al.id], b = BAYS[bayOf(m, al.n)], L = LRUS[m.lru];
      const rep = L && !m.transient ? `<button class="btn" data-rep="${al.key}" title="${L.where} repair, ${L.min} min">repair</button>` : '';
      return `<tr data-k="${al.key}" class="${H.selected === al.key ? 'sel' : ''}"><td class="sev ${m.sev}">${m.sev}</td><td style="text-align:left">${unitLabel(m, al.n)} · ${m.name}${al.spurious ? ' <i>(spurious)</i>' : ''}</td><td style="text-align:left">${b.name.replace(/ - .*/, '')}</td><td>${fmtT(al.tDet)}</td><td>${rep}</td></tr>`;
    }).join('') : '<tr><td colspan="5" style="text-align:left;color:var(--ok)">No alarms. All units passing BIT.</td></tr>';
    if (H.selected && !H.alarms.has(H.selected)) H.selected = rows[0]?.key || null;
    const sel = H.selected && H.alarms.get(H.selected);
    if (sel) {
      const p = H.procedure(sel.id), b = BAYS[bayOf(p.m, sel.n)];
      $('#hProc').innerHTML = `<b>${unitLabel(p.m, sel.n)} · ${p.m.name}</b><br><span class="dim">Location:</span> ${b.name} - ${b.where}. Access: ${b.door}.<br><span class="dim">Detected by:</span> ${sel.reading}<br>` +
        p.lines.slice(1).map((l) => l.replace(/^(\w[\w ]*?):/, '<span class="dim">$1:</span>')).join('<br>');
    } else $('#hProc').textContent = 'Select an alarm.';
    const j = H.job;
    $('#hJobBox').style.display = j ? 'block' : 'none';
    if (j) { const f = Math.min(1, (this.sim.time - j.t0) / j.dur); $('#hJob').textContent = `${j.k} by ${j.who} · step ${j.i + 1}/${j.steps.length}: ${j.steps[j.i]} · ${fmt((1 - f) * j.dur / 60, 1)} min left`; $('#hJobBar').style.width = f * 100 + '%'; }
    $('#hSpares').textContent = Object.keys(LRUS).filter((k) => LRUS[k].qty > 0).map((k) => `${k} ${H.spares.kit[k] || 0}·${H.spares.dock[k] || 0}`).join('  ');
  }

  camRect() { const r = $('#camFrame').getBoundingClientRect(); return $('#camPanel').classList.contains('collapsed') || getComputedStyle($('#camPanel')).display === 'none' ? null : r; }

  // Arms panel: everything from rover.telemetry().arm (arm.js telemetry()); lift margin is asked of the arm model at the current pose
  updateArm(R) {
    const A = R.arm; if (!A || !$('#armRows')) return;
    const T = A.telemetry(), cls = (el, c) => { el.className = 'v' + (c ? ' ' + c : ''); };
    $('#aMode').textContent = `${T.mode}${T.plan ? ' · ' + T.plan : ''}${T.status && T.status !== T.mode ? ' · ' + T.status : ''}`;
    $('#aDeck').textContent = T.deckLatch;
    const pl = T.payload;
    $('#aLoad').textContent = pl ? `${fmt(pl.mass, 0)} kg · ${pl.state} · hooks carry ${fmt(pl.loadFrac * 100, 0)} %` : 'none';
    const lim = ARM.tipRatioMax;
    $('#aTip').textContent = `${fmt(T.tipRatio, 2)} / ${fmt(lim, 2)} limit`; cls($('#aTip'), T.tipRatio > lim ? 'err' : T.tipRatio > 0.85 * lim ? 'warn' : '');
    let lm = null; try { lm = A.liftMargin(pl ? pl.mass : CARGO_TYPES.acm.mass, A.arms[0].q).margin; } catch (e) { lm = null; }
    $('#aMargin').textContent = lm == null ? '—' : fmt(lm, 2); cls($('#aMargin'), lm != null && lm < 1 ? 'err' : lm != null && lm < 1.3 ? 'warn' : '');
    $('#aFault').textContent = T.faults.length ? T.faults.join(', ') : 'none'; cls($('#aFault'), T.faults.length ? 'err' : '');
    const nm = ['port', 'stbd'], pc = (v) => (v > 0.9 ? 'err' : v > 0.6 ? 'warn' : '');
    const jn = ['S', 'E', 'W'];
    $('#armRows').innerHTML = [0, 1].flatMap((i) => [0, 1, 2].map((j) => `<tr><td style="text-align:left">${nm[i]} ${jn[j]}</td><td>${fmt(T.qDeg[i][j], 1)}</td><td class="${pc(T.pctStall[i][j])}">${fmt(T.pctStall[i][j] * 100, 0)}</td><td>${fmt(T.currentA[i][j], 1)}</td><td>${fmt(T.tempC[i][j], 0)}</td></tr>`)).join('');
    $('#aHookP').textContent = `${T.latch[0]} gate · ${fmt(T.hookLoad[0] / 1000, 1)} kN`;
    $('#aHookS').textContent = `${T.latch[1]} gate · ${fmt(T.hookLoad[1] / 1000, 1)} kN`;
  }

  update(dt) {
    this.inst.update(this.sim.rover, dt, !!this.ui.overlays.eng);
    this.therm.update(dt);
    { const [txt, cls] = this.therm.status(), e = $('#pTherm'); if (e.textContent !== txt) e.textContent = txt; e.className = 'pill eng ' + cls; }
    this._t -= dt; if (this._t > 0) return; this._t = 0.1;
    const sim = this.sim, R = sim.rover, A = sim.autonomy, S = sim.sensors;
    // downlink delay (lunar audit B4): over a delayed link the operator sees telemetry one light time old. A 10 Hz ring
    // buffer of snapshots is read sim.comms.delayS back; the drive state, speed, attitude and SoC shown come from it.
    const tlNow = R.telemetry();
    (this.tlm ||= new DelayLine(1024)).push(sim.time, { tl: { ...tlNow, pose: { ...tlNow.pose } }, soc: R.power.soc, wh: R.power.wh, estop: R.estop, powered: R.powered });
    const dl = sim.comms.delayS > 0, snap = dl ? this.tlm.at(sim.time - sim.comms.delayS) : null, tl = snap ? snap.tl : tlNow;
    const seen = snap || { soc: R.power.soc, wh: R.power.wh, estop: R.estop, powered: R.powered };
    this.seen = seen;
    $('#camPanel').style.top = ($('#mapPanel').getBoundingClientRect().bottom + 8) + 'px';
    // show the speed actually achieved: above ~5x the physics is CPU-bound, and 50x relies on transit warp
    const act = sim.actualRate, warpOn = sim.warp.active && sim.timeScale >= sim.warp.minScale;
    $('#clock').textContent = `MET ${fmtT(sim.time)}${sim.paused ? ' ❚❚' : ''} ${sim.timeScale !== 1 ? '· ' + sim.timeScale + '×' : ''}${sim.timeScale > 1 && !sim.paused ? ` (actual ${act >= 10 ? act.toFixed(0) : act.toFixed(1)}×${warpOn ? ', transit warp' : sim.lagging ? ', CPU-limited' : ''})` : ''}${S.decimated ? ' · sensors decimated' : ''}`;
    const pill = (id, txt, cls) => { const e = $(id); e.textContent = txt; e.className = (e.classList.contains('eng') ? 'pill eng ' : 'pill ') + cls; }; // keeps the layer marker
    pill('#pMode', A.mode === 'AUTO' ? 'AUTO' : A.paused && A.gen ? 'OVERRIDE' : 'MANUAL', A.mode === 'AUTO' ? 'info' : A.paused && A.gen ? 'warn' : '');
    pill('#pDrive', seen.estop ? 'E-STOP' : sim.startup ? 'SELF-TEST' : seen.powered ? 'DRIVE ARMED' : 'DISARMED', seen.estop ? 'err' : seen.powered ? 'ok' : 'warn');
    if (this.ai) pill('#pAI', this.ai.busy ? 'AI THINKING' : this.ai.online ? 'AI ONLINE' : this.ai.online === false ? 'AI OFFLINE' : 'AI', this.ai.busy ? 'info' : this.ai.online ? 'ok' : this.ai.online === false ? 'warn' : '');
    else $('#pAI').style.display = 'none';
    { const pend = sim.comms.queue.some((e) => e.estop);
      pill('#pComms', A.faults.comms ? 'COMMS LOS' : pend ? 'E-STOP IN FLIGHT' : dl ? `COMMS ${sim.comms.mode.toUpperCase()} · TLM −${sim.comms.delayS.toFixed(1)} s` : 'COMMS OK', A.faults.comms ? 'err' : pend ? 'warn' : dl ? 'info' : 'ok'); }
    pill('#pLidar', S.faults.lidar ? 'LIDAR FAIL' : S.lidarOn ? 'LIDAR' : 'LIDAR OFF', S.faults.lidar ? 'err' : S.lidarOn ? 'ok' : 'warn');
    pill('#pCam', S.faults.camera ? 'CAM FAIL' : S.cameraOn ? 'CAM' : 'CAM OFF', S.faults.camera ? 'err' : S.cameraOn ? 'ok' : 'warn');
    pill('#pSteer', `${R.rpm.proto.toFixed(0)} RPM ${R.rpm.tier.toUpperCase()}`, R.rpm.tier === 'turbo' ? 'warn' : R.rpm.tier === 'crawl' ? 'info' : '');
    $('#estop').classList.toggle('latched', seen.estop);
    $('#resetBtn').disabled = !R.estop; $('#startBtn').disabled = R.estop || R.powered || !!sim.startup;
    $('#estopBanner').style.display = seen.estop ? 'block' : 'none';
    $('#overrideBanner').style.display = A.paused && A.gen && !R.estop ? 'block' : 'none';
    this.syncTimeUi();
    this.root.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('on', b.dataset.view === this.ui.view));
    this.updateHealth(dt);

    if (this.layer === 0) return;                           // clean view: nothing on screen to feed
    if (this.layer === 1) { this.updateEssential(tl); return; } // essential layer: the engineering panels are hidden
    const e = sim.slam.est, p = tl.pose;
    const hdg = (y) => ((90 - y * 180 / Math.PI) % 360 + 360) % 360;
    $('#vSpeed').textContent = `${fmt(tl.fwdSpeed, 2)} m/s`;
    $('#vHdg').textContent = `${fmt(hdg(p.yaw), 1)}° / ${fmt(hdg(e.yaw), 1)}°`;
    $('#vAtt').textContent = `${fmt(tl.roll * 57.3, 1)}° / ${fmt(tl.pitch * 57.3, 1)}°`;
    $('#vMargin').textContent = `${fmt(tl.margin, 1)}°`;
    const mb = $('#bMargin'); mb.style.width = Math.max(0, Math.min(100, tl.margin / 50 * 100)) + '%'; mb.style.background = tl.margin < 15 ? 'var(--err)' : tl.margin < 25 ? 'var(--warn)' : 'var(--ok)';
    $('#vSoc').textContent = `${fmt(seen.soc * 100, 1)} %  (${fmt(seen.wh / 1000, 1)} kWh, ${fmt(R.power.voltage, 0)} V)`;
    const sb = $('#bSoc'); sb.style.width = seen.soc * 100 + '%'; sb.style.background = seen.soc < A.reserveSoc ? 'var(--err)' : seen.soc < 0.4 ? 'var(--warn)' : 'var(--ok)';
    $('#vPow').textContent = `${fmt((R.power.dockW || 0) / 1000, 1)} / ${fmt(R.power.loadW / 1000, 2)} kW`;
    { const D = sim.powerDock; $('#vDock').textContent = `bank ${fmt(D.bankV, 1)} V · ${fmt(D.bankAhNow, 0)}/${D.bankAhRated} Ah (${fmt(D.soc * 100, 0)}%) · ${fmt(D.bankA, 1)} A · sun ${fmt(D.solarW, 0)} W · ${fmt(D.Tbank, 0)} °C · ${D.status}`;
      $('#vDockChg').textContent = D.connected ? `${fmt(D.chargeA, 2)} A @ ${fmt(D.packV, 1)} V = ${fmt(D.chargeW, 1)} W · pack ${fmt(R.power.soc * 10, 1)} Ah · link ${fmt(D.linkEff * 100, 0)}% · ≙ ${fmt((R.power.dockW || 0) / 1000, 0)} kW` : 'not coupled'; }
    $('#vDrive').textContent = `${fmt(R.power.driveW / 1000, 2)} kW  regen ${fmt(R.power.regenWh, 1)} Wh`;
    $('#vOdo').textContent = `${fmt(R.odo.dist / 1000, 3)} km / ${fmt(R.power.energyUsedWh / 1000, 2)} kWh`;
    $('#vLoad').textContent = R.loaded ? `${R.lift.cargo.id} ${(R.lift.cargo.T.mass / 1000).toFixed(0)} t (${(R.lift.cargo.T.mass * MOON.g / 1000).toFixed(1)} kN)` : 'none';
    $('#vLift').textContent = `${fmt(R.lift.pos, 2)} m  (target ${fmt(R.lift.target, 2)})`;
    $('#vClamp').textContent = `${R.lift.clamp} ${R.lift.clamp === 'closing' || R.lift.clamp === 'opening' ? (R.lift.clampT * 100).toFixed(0) + '%' : ''}`;
    $('#vLim').textContent = `${R.rpm.proto.toFixed(0)} rpm ${R.rpm.tier} · ${R.rpm.why} · cap ${R.speedCap().toFixed(1)} m/s`;
    const names = ['FL', 'FR', 'ML', 'MR', 'RL', 'RR']; const kS = Math.sqrt(K);
    // ML / MR (driven: false) are passive idlers: no motor, so torque shows IDLE and the row is dimmed
    $('#wheelRows').innerHTML = R.wheels.map((w, i) => `<tr class="${w.driven === false ? 'idle' : ''}"><td class="${w.fault ? 'f' : ''}">${i + 1} ${names[i]}${w.driven === false ? ' <small>idler</small>' : ''}</td><td>${fmt(w.omega * 60 / (2 * Math.PI) * kS, 0)}</td><td>${w.driven === false ? 'IDLE' : fmt(w.torque, 0)}</td><td>${fmt(w.slip * 100, 0)}%</td><td>${fmt(w.sink * 1000, 0)}</td><td>${fmt(w.load / 1000, 2)}</td><td>${fmt(w.temp, 0)}</td></tr>`).join('');
    this.updateArm(R);
    $('#bogieRow').textContent = R.bogies.map((b) => `${b.name} ${((ROVER.bogie.Lnom - b.L) * 1000).toFixed(0)}`).join('   ');
    const contact = R.wheels.filter((w) => w.contact);
    const avgSink = contact.reduce((a, w) => a + w.sink, 0) / Math.max(1, contact.length), avgSlip = contact.reduce((a, w) => a + Math.abs(w.slip), 0) / Math.max(1, contact.length);
    const rr = contact.reduce((a, w) => a + w.Rc, 0), load = contact.reduce((a, w) => a + w.load, 0);
    $('#soilRow').textContent = `Bekker sinkage ${(avgSink * 1000).toFixed(0)} mm · |slip| ${(avgSlip * 100).toFixed(1)} % · rolling resistance coeff ${(rr / Math.max(1, load)).toFixed(3)} · contacts ${contact.length}/${R.wheels.length}`;

    // SLAM stats
    const err = Math.hypot(e.x - p.x, e.z - p.z);
    $('#sErr').textContent = `${fmt(err, 2)} m · ${fmt(((((e.yaw - p.yaw) * 180 / Math.PI + 180) % 360 + 360) % 360) - 180, 2)}°`;
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
    const lines = [[8, 'speed m/s', '#35d0ff', 0, 5.5], [11, 'tip margin°', '#3ddc84', 0, 50], [14, 'load kW', '#ffb020', 0, 20000], [12, 'SoC %', '#c58cff', 0, 100]];
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
    if (sim.scout.enabled) {
      const p0 = sim.rover.pose(), dy = e.yaw - p0.yaw, cy = Math.cos(dy), sy = Math.sin(dy);
      const rx = sim.scout.x - p0.x, rz = sim.scout.z - p0.z;
      mark(e.x + rx * cy + rz * sy, e.z - rx * sy + rz * cy, 'SCOUT-1', '#7dffb0');
    }
    // LiDAR range
    ctx.strokeStyle = 'rgba(53,208,255,0.25)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(W / 2, H / 2, ROVER.lidar.range * z, 0, Math.PI * 2); ctx.stroke();
    // true pose ghost + estimated pose
    const p = sim.rover.pose();
    const drawRover = (x, zz, yaw, fill, stroke) => {
      const [px, py] = toPx(x, zz);
      ctx.save(); ctx.translate(px, py); ctx.rotate(-yaw);
      ctx.fillStyle = fill; ctx.strokeStyle = stroke; ctx.lineWidth = 2;
      const hl = ROVER.frame.halfLen, hw = ROVER.frame.halfWid;
      ctx.fillRect(-hl * z, -hw * z, 2 * hl * z, 2 * hw * z); ctx.strokeRect(-hl * z, -hw * z, 2 * hl * z, 2 * hw * z);
      ctx.beginPath(); ctx.moveTo(hl * z + 8, 0); ctx.lineTo(hl * z, -6); ctx.lineTo(hl * z, 6); ctx.closePath(); ctx.fillStyle = stroke; ctx.fill();
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
    ctx.fillStyle = 'rgba(255,255,255,0.7)'; ctx.font = '16px monospace'; ctx.fillText(`CAM-1 · ${ROVER.camera.hFovDeg}° HFOV${ROVER.camera.stereo ? ' stereo' : ''} · ${new Date(this.sim.time * 1000).toISOString().slice(11, 19)}`, 10, 22);
    const c = S.cass;
    $('#camInfo').textContent = `fiducials ${S.detections.length} · rocks ${S.rockDetections.length}` + (S.stereo ? ` · depth ${S.stereo.n} pts (${S.stereo.fused} fused)` : '') + (c && c.valid ? ` · CASS lat ${(c.lateral * 100).toFixed(1)} cm yaw ${(c.yaw * 57.3).toFixed(2)}°` : '') + (c && c.ring ? ` · dock ring ${c.ring.along.toFixed(2)} m` : '');
  }
}
