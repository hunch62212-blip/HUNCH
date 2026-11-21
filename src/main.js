import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from './sim/sim.js';
import { ROVER, K, WORLD } from './sim/config.js';
import { heightAt } from './sim/terrain.js';
import { runCommand, requestEstop } from './sim/commands.js';
import { LosMonitor, DelayLine } from './sim/commsLink.js';
import { COMMS } from './sim/lunarParams.js';
import { createScene } from './render/scene.js';
import { TerrainMesh } from './render/terrainMesh.js';
import { RockField, buildLander, buildHabitat, buildCargo, buildGeofences, MAT, tagMesh, labelTexture as labelTex } from './render/props.js';
import { AegisModel, buildTeamCargo, buildTeamStands } from './render/aegis.js';
import { ArmModel, acmPose } from './render/arm.js';
import { CARGO_TYPES } from './sim/config.js';
import { loadTeamModels } from './render/teamModels.js';
import { Dust, Tracks, LidarCloud, PathOverlay, Pebbles } from './render/effects.js';
import { HUD } from './ui/hud.js';
import { DiskMemory } from './sim/memoryStore.js';
import { PowerDockModel } from './render/powerDock.js';
import { EngineeringOverlay } from './render/engOverlay.js';
import { trackedRoverCAD } from './render/cadModels.js';
import { labelTexture } from './render/props.js';

const loadFill = document.getElementById('load-fill'), loadMsg = document.getElementById('load-msg');
const step = async (pct, msg) => { loadFill.style.width = pct + '%'; loadMsg.textContent = msg; await new Promise((r) => setTimeout(r, 30)); };

// ---------------- persistence (IndexedDB) for the SLAM map + route memory ----------------
const DB = {
  open() { return new Promise((res, rej) => { const r = indexedDB.open('span40', 1); r.onupgradeneeded = () => r.result.createObjectStore('slam'); r.onsuccess = () => res(r.result); r.onerror = () => rej(r.error); }); },
  async put(k, v) { const db = await this.open(); return new Promise((res, rej) => { const tx = db.transaction('slam', 'readwrite'); tx.objectStore('slam').put(v, k); tx.oncomplete = () => { db.close(); res(); }; tx.onerror = () => { db.close(); rej(tx.error); }; }); },
  async wipe() { return new Promise((res) => { const r = indexedDB.deleteDatabase('span40'); r.onsuccess = r.onerror = r.onblocked = () => res(); }); },
  async get(k) { const db = await this.open(); return new Promise((res, rej) => { const tx = db.transaction('slam', 'readonly'); const q = tx.objectStore('slam').get(k); q.onsuccess = () => { db.close(); res(q.result); }; q.onerror = () => { db.close(); rej(q.error); }; }); },
};

async function main() {
  await step(5, 'Initialising Rapier physics (WASM)…');
  await RAPIER.init();
  // a new random crater / boulder layout on every load; ?seed=N reproduces a map
  const qs = new URLSearchParams(location.search);
  // ?site=last: reopen the most recently remembered site (rover memory store, scripts/memory-plugin.mjs)
  let lastSite = null;
  if (qs.get('site') === 'last') { const L = await DiskMemory.sites(); lastSite = L && L.sites && L.sites[0] ? L.sites[0].seed : null; }
  const seed = lastSite != null ? lastSite >>> 0 : qs.has('seed') && Number.isFinite(+qs.get('seed')) ? (+qs.get('seed') >>> 0) : (Math.random() * 2 ** 32) >>> 0;
  await step(15, `Generating ${(WORLD.size / 1000).toFixed(1)} × ${(WORLD.size / 1000).toFixed(1)} km lunar terrain (seed ${seed}): craters, boulders, highlands…`);
  // thermal scenarios happen at random in the browser (?noscen turns them off); headless tests opt in
  const sim = new Sim(RAPIER, { seed, thermalScenarios: !qs.has('noscen') });
  sim.idleSolar = !qs.has('noidlesolar');
  // operator link (lunar audit B4): Earth one-way delay >= 2.6 s (light time + ground segment); COMMS LOS raised from the
  // mast's line of sight to the habitat antenna / Earth (sim.losMonitor, once a second; "comms auto off" to disable)
  sim.comms.earthDelayS = Math.max(sim.comms.earthDelayS || 0, COMMS.earthDelayS);
  if (sim.comms.autoLos == null) sim.comms.autoLos = !qs.has('nolos');
  sim.log(`Terrain seed ${seed}: crater and boulder layout is random each load (add ?seed=${seed} to the URL to repeat this map, or type "newmap")`, 'info');
  await step(45, 'Building renderer, lunar sky and lighting…');
  const canvas = document.getElementById('view');
  const S = createScene(canvas);
  const { scene, camera, renderer } = S;  S.setSun(sim.sunDir);
  sim.on('sun', () => S.setSun(sim.sunDir));

  await step(55, 'Meshing terrain (chunked LOD, physics-exact heights)…');
  const terrainMesh = new TerrainMesh(scene, sim.terrain);
  const p0 = sim.rover.pose();
  terrainMesh.update(new THREE.Vector3(p0.x, 0, p0.z), 2000);
  await step(70, 'Placing boulders, lander, supply building and cargo…');
  const rocks = new RockField(scene, sim.terrain);
  sim.on('rocks', () => rocks.rebuild());
  scene.add(buildLander(sim.terrain));
  const hab = buildHabitat(sim.terrain, sim.ports);
  scene.add(hab.root);
  const powerDockVis = new PowerDockModel(scene, sim);
  await step(74, 'Loading the team AEGIS-V1, module and stand models…');
  const TM = await loadTeamModels();
  scene.add(await buildTeamStands(sim.cargo, heightAt, sim.terrain));
  const fences = buildGeofences(sim.terrain); scene.add(fences);
  const cargoVis = [];
  for (const c of sim.cargo) {
    if (c.T.handling === 'arm') continue;   // ACM meshes are made by syncAcmVis
    let g;
    if (c.T.shape === 'cylinder') {
      g = await buildTeamCargo(c);
      const id = c.id.charCodeAt(3), T = c.T, half = T.radius;
      const aft = tagMesh(id, 0.8); aft.position.set(-T.length / 2 - 0.02, -half * 0.62, 0); aft.rotation.y = -Math.PI / 2; g.add(aft);
      const fwd = tagMesh(id + 1, 0.6); fwd.position.set(T.length / 2 + 0.03, half * 0.62, 0); fwd.rotation.y = Math.PI / 2; g.add(fwd);
      for (const sz of [-1, 1]) { const t = tagMesh(id + 2, 0.8); t.position.set(-1.2, -0.3, sz * (half + 0.035)); t.rotation.y = sz > 0 ? 0 : Math.PI; g.add(t); }
    } else {
      g = buildCargo(c);
    }
    g.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    scene.add(g); cargoVis.push({ c, g });
  }
  // ACM (arm-handled cargo module) visuals. sim.cargo entries with T.handling === 'arm' get a mesh as soon as they exist (sim.spawnACM,
  // P3a wiring); until the sim owns one, a ghost ACM is shown while rover.arm.payload is set, hanging from the hook trunnions.
  const acmVis = new Map(); let acmGhost = null;
  const _ap = new THREE.Vector3(), _aq = new THREE.Quaternion();
  function syncAcmVis(R) {
    for (const c of sim.cargo) if (c.T.handling === 'arm' && !acmVis.has(c)) { const g = buildCargo(c); scene.add(g); acmVis.set(c, g); }
    const real = [...acmVis.keys()];
    const pl = R.arm.payload;
    if (!real.length && pl) {
      if (!acmGhost) { acmGhost = { c: { id: 'ACM', T: CARGO_TYPES.acm, mode: 'GROUND' }, g: buildCargo({ id: 'ACM', T: CARGO_TYPES.acm }) }; scene.add(acmGhost.g); }
      acmGhost.g.visible = true;
      if (acmPose(R, acmGhost.c, _ap, _aq)) { acmGhost.g.position.copy(_ap); acmGhost.g.quaternion.copy(_aq); }
    } else if (acmGhost) acmGhost.g.visible = false;
    for (const [c, g] of acmVis) { if (acmPose(R, c, _ap, _aq)) { g.position.copy(_ap); g.quaternion.copy(_aq); } }
  }
  // receiving stand (team pallet) rises out of the apron under the docked module during berthing step 6
  const jackVis = new Map();
  function updateJacks() {
    for (const port of sim.ports) {
      const c = port.occupied;
      if (!c || !(port.jackK > 0)) continue;
      let v = jackVis.get(port);
      if (!v) {
        const t = c.body.translation(), r = c.body.rotation();
        const q = new THREE.Quaternion(r.x, r.y, r.z, r.w), ax = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
        const yaw = Math.atan2(-ax.z, ax.x);
        const lugs = c.T.lugXs || [-c.T.lugX, c.T.lugX];
        const midX = (lugs[0] + lugs[1]) / 2, spanX = lugs[1] - lugs[0];
        const nomSpanX = 2 * (c.T.shape === 'cylinder' ? c.T.lugX : 0.21 * K);
        const wrap = new THREE.Group(); wrap.scale.setScalar(K);
        const st = TM.ModuleModels.buildStand(port.name);
        const half = c.T.adapterR || c.T.radius;
        const stX = t.x + Math.cos(yaw) * midX, stZ = t.z - Math.sin(yaw) * midX;
        TM.ModuleModels.placeStand(st, stX / K, stZ / K, (t.y - half) / K, () => port.ground / K);
        st.scale.x = spanX / nomSpanX;
        const pivot = new THREE.Group(); pivot.position.copy(st.position); st.position.set(0, 0, 0); pivot.rotation.y = yaw; pivot.add(st);
        wrap.add(pivot); scene.add(wrap);
        v = { wrap, depth: (t.y - half - port.ground) };
        jackVis.set(port, v);
      }
      v.wrap.position.y = -(1 - Math.min(1, port.jackK)) * (v.depth + 0.5);
    }
  }
  await step(80, 'Assembling AEGIS-V1 rover model…');
  const roverModel = await new AegisModel(scene).init();
  const armVis = new ArmModel(roverModel);   // two 3-DOF ACM arms + deck pins / latch / stow cradles, posed from rover.arm every frame
  // SCOUT-1 (team tracked-rover CAD, 3x scale)
  const scoutVis = trackedRoverCAD(3); scene.add(scoutVis);
  const scoutTag = new THREE.Sprite(new THREE.SpriteMaterial({ map: labelTexture('SCOUT-1', { color: '#35d0ff', size: 70, w: 512, h: 128 }), transparent: true, depthWrite: false }));
  scoutTag.scale.set(3.2, 0.8, 1); scoutTag.position.y = 2.2; scoutVis.add(scoutTag);
  const scoutCloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0x7dffb0, size: 2, sizeAttenuation: false, transparent: true, opacity: 0.8, depthWrite: false }));
  scoutCloud.frustumCulled = false; scene.add(scoutCloud);
  const dust = new Dust(scene, sim.terrain);
  const tracks = new Tracks(scene);
  const lidar = new LidarCloud(scene);
  const overlay = new PathOverlay(scene, sim.terrain);
  const engOverlay = new EngineeringOverlay(scene, sim.terrain);
  const pebbles = new Pebbles(scene, sim.terrain);

  // ---------------- UI state & camera ----------------
  const ui = {
    view: 'chase', manualHold: false,
    overlays: { lidar: true, path: true, arcs: false, routes: true, fences: true, dust: true, xray: false, eng: true },
    setView(v) { if (['chase', 'orbit', 'top', 'cam', 'free'].includes(v)) { ui.view = v; onViewChange(); sim.log(`View: ${v}`); } },
    noSave: false,
    async forgetMap() { ui.noSave = true; sim.slam.clear(); await DB.wipe(); await DiskMemory.forget(sim.terrain.seed); sim.log('Stored map and route memory erased. Reload the page for a completely fresh start (nothing is saved until then).', 'warn'); },
    async saveMap(quiet = false) {
      if (ui.noSave) return;
      const d = sim.slam.serialize();
      try { await DB.put('map', d); } catch (e) { /* browser copy is only the fallback */ }
      const r = await DiskMemory.save(d);
      if (!quiet || !r) sim.log(r ? `Rover memory saved: ${d.tiles.length} map tiles, ${d.routes.length} routes -> ${r.dir} (site ${d.seed}, ${(r.bytes / 1e6).toFixed(1)} MB)` : `SLAM map saved in this browser only (${d.tiles.length} tiles): the on-disk memory store is not reachable`, r ? 'ok' : 'warn');
    },
    async memoryStatus() {
      const L = await DiskMemory.sites();
      if (!L) { sim.log('Rover memory store not reachable (static build?): maps are kept in this browser only', 'warn'); return; }
      sim.log(`Rover memory store: ${L.dir} - ${L.sites.length} site(s)${L.sites.length ? ': ' + L.sites.slice(0, 6).map((s) => `${s.seed}${s.seed === sim.terrain.seed ? ' (this site)' : ''} ${(s.bytes / 1e6).toFixed(1)} MB, ${s.history} snapshot(s), saved ${new Date(s.savedAt).toLocaleString()}`).join(' | ') : ''}. Open ?site=last (or ?seed=N) to return to a remembered site.`, 'info');
    },
    async loadMap() { try { const d = (await DiskMemory.load(sim.terrain.seed)) || (await DB.get('map')); if (d && sim.slam.deserialize(d)) sim.log(`Remembered SLAM map loaded: ${d.tiles.length} tiles, ${d.routes.length} proven routes (saved ${new Date(d.savedAt).toLocaleString()})`, 'ok'); else sim.log('No saved map for this world', 'warn'); } catch (e) { sim.log('Map load failed: ' + e.message, 'err'); } },
  };
  // pixels the picture must move up so that the view centre lands in the middle of the panel-free band at the
  // horizontal centre of the window (between the top bar / thermal header and the instruments / status / console)
  let viewShift = 0, viewShiftWant = 0, vsW = 0, vsH = 0;
  function freeAreaShift() {
    const H = window.innerHeight, cx = window.innerWidth / 2; let top = 0, bot = H;
    for (const id of ['topbar', 'thermPanel', 'instPanel', 'autoPanel', 'console']) {
      const el = document.getElementById(id); if (!el) continue;
      const r = el.getBoundingClientRect(); if (!r.width || !r.height || r.left > cx || r.right < cx) continue;
      if (r.bottom < H * 0.35) top = Math.max(top, r.bottom); else if (r.top > H * 0.35) bot = Math.min(bot, r.top);
    }
    return Math.max(-0.3 * H, Math.min(0.3 * H, H / 2 - (top + bot) / 2));
  }
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true; controls.dampingFactor = 0.08; controls.maxPolarAngle = Math.PI * 0.495; controls.minDistance = 6; controls.maxDistance = 1800;
  camera.position.set(p0.x - p0.fwd.x * 32 + 14, p0.y + 13, p0.z - p0.fwd.z * 32 + 10);
  controls.target.set(p0.x, p0.y - 1, p0.z);
  const lastTarget = new THREE.Vector3().copy(controls.target);
  // low-rate onboard feed: small render target + full-screen quad (tone-mapped like the main view)
  const feedRT = new THREE.WebGLRenderTarget(480, 360, { type: THREE.HalfFloatType }); feedRT.ready = false;
  const feedScene = new THREE.Scene(), feedCam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
  feedScene.add(new THREE.Mesh(new THREE.PlaneGeometry(2, 2), new THREE.MeshBasicMaterial({ map: feedRT.texture, toneMapped: true, depthTest: false, depthWrite: false })));
  const roverCam = new THREE.PerspectiveCamera(2 * Math.atan(Math.tan(ROVER.camera.hFovDeg * Math.PI / 360) / (4 / 3)) * 180 / Math.PI, 4 / 3, 0.2, 6000);
  roverCam.updateProjectionMatrix();
  function onViewChange() {
    const p = sim.rover.pose();
    controls.enabled = ui.view !== 'cam';
    if (ui.view === 'top') { camera.position.set(p.x, p.y + 320, p.z + 1); controls.target.set(p.x, p.y, p.z); }
    if (ui.view === 'chase') { camera.position.set(p.x - p.fwd.x * 34, p.y + 12, p.z - p.fwd.z * 34); controls.target.set(p.x, p.y - 1, p.z); }
    lastTarget.copy(controls.target);
  }

  const hud = new HUD(sim, ui);
  sim.losMonitor = new LosMonitor(sim, { onRestore: () => runCommand(sim, ui, 'clear comms', true) });
  const camDelay = new DelayLine(256);   // onboard view: camera pose ring buffer, shown sim.comms.delayS late
  sim.on('routes', () => { overlay.setRoutes(sim.slam.routes); ui.saveMap(true); }); // quiet: this fires on every SLAM correction and filled the console with "memory saved" lines
  sim.on('path', () => overlay.setPath(sim.autonomy.path));
  sim.on('scan', (s) => { if (ui.overlays.lidar && lidar.points.visible) lidar.set(s, sim.rover.pose().y); });

  // double-click on the ground: autonomous drive to that point
  const rc = new THREE.Raycaster(), mouse = new THREE.Vector2();
  canvas.addEventListener('dblclick', (e) => {
    mouse.set(e.clientX / window.innerWidth * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
    rc.setFromCamera(mouse, camera);
    const hit = rc.intersectObjects(terrainMesh.group.children.filter((m) => m.visible), false)[0];
    if (!hit) return;
    // goals are given in the rover's SLAM frame: apply its current estimate offset
    const p = sim.rover.pose(), est = sim.slam.est;
    const gx = hit.point.x + (est.x - p.x), gz = hit.point.z + (est.z - p.z);
    hud.exec(`goto ${gx.toFixed(1)} ${gz.toFixed(1)}`);
  });

  // ---------------- manual driving: keyboard + gamepad ----------------
  const keys = new Set();
  const drive = { thr: 0, steer: 0 };
  // touch screens have no W/A/S/D: an on-screen stick gives the same analog throttle / steer as a gamepad
  const touchDrive = { on: false, thr: 0, steer: 0 };
  if (matchMedia('(pointer: coarse)').matches || navigator.maxTouchPoints > 0 || /[?&]touch=1/.test(location.search)) {
    const pad = document.createElement('div'); pad.id = 'stick'; pad.title = 'Drive: up = forward, sideways = turn';
    pad.innerHTML = '<div class="knob"></div><span>drive</span>';
    document.getElementById('hud').appendChild(pad);
    const knob = pad.firstChild; let pid = null;
    const dz = (v) => (Math.abs(v) < 0.15 ? 0 : v);
    const move = (e) => {
      const r = pad.getBoundingClientRect(), R0 = r.width / 2;
      let dx = e.clientX - r.left - R0, dy = e.clientY - r.top - R0; const m = Math.hypot(dx, dy);
      if (m > R0) { dx *= R0 / m; dy *= R0 / m; }
      knob.style.transform = `translate(${dx}px, ${dy}px)`;
      touchDrive.thr = dz(-dy / R0); touchDrive.steer = dz(-dx / R0);
    };
    const end = (e) => { if (e.pointerId !== pid) return; pid = null; touchDrive.on = false; touchDrive.thr = touchDrive.steer = 0; knob.style.transform = ''; };
    pad.addEventListener('pointerdown', (e) => { pid = e.pointerId; try { pad.setPointerCapture(pid); } catch (err) { /* keep driving without capture */ } touchDrive.on = true; move(e); e.preventDefault(); });
    pad.addEventListener('pointermove', (e) => { if (e.pointerId === pid) move(e); });
    pad.addEventListener('pointerup', end); pad.addEventListener('pointercancel', end);
  }
  window.addEventListener('keydown', (e) => {
    if (document.activeElement && document.activeElement.id === 'cmd') return;
    const k = e.key.toLowerCase();
    if (k === 'escape') { requestEstop(sim, 'keyboard (Esc)'); e.preventDefault(); return; }
    if (k === 'enter') { document.getElementById('cmd').focus(); e.preventDefault(); return; }
    keys.add(k);
    if (k === 'c') { const v = ['chase', 'orbit', 'top', 'cam', 'free']; ui.setView(v[(v.indexOf(ui.view) + 1) % v.length]); }
    if (k === 'v') { ui.overlays.lidar = !ui.overlays.lidar; }
    if (k === 'i') { ui.overlays.eng = !ui.overlays.eng; document.querySelector('[data-ov="eng"]')?.classList.toggle('on', ui.overlays.eng); }
    if (k === 't') hud.toggleThermal();
    if (k === 'p') sim.paused = !sim.paused;
    if (k === 'x') { sim.paused = false; sim.timeScale = sim.timeScale >= 50 ? 1 : 50; sim.log(sim.timeScale >= 50 ? 'FAST FORWARD 50x' : 'Time scale 1x', 'ok'); }
    if (k === 'h') hud.cycleLayer();
    if (k === '?') hud.toggleHelp();
    if (k === 'g') { const L = sim.rover.lift; const r = L.clamp === 'open' ? sim.rover.clampClose() : sim.rover.clampOpen(); sim.log(r.ok ? sim.rover.lift.msg : 'Refused: ' + r.why, r.ok ? 'ok' : 'warn'); }
    if ([' ', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) e.preventDefault();
  });
  window.addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));
  // HUD buttons must not keep keyboard focus (Space/Enter would re-trigger them while driving)
  document.addEventListener('click', (e) => { const b = e.target.closest && e.target.closest('button, select'); if (b && b.tagName === 'BUTTON') b.blur(); }, true);
  canvas.addEventListener('pointerdown', () => { if (document.activeElement && document.activeElement !== document.body) document.activeElement.blur(); });
  window.addEventListener('blur', () => keys.clear());

  function manualInput(dt) {
    const R = sim.rover, A = sim.autonomy;
    const has = (...k) => k.some((x) => keys.has(x));
    let thrIn = (has('w', 'arrowup') ? 1 : 0) - (has('s', 'arrowdown') ? 1 : 0);
    let steerIn = (has('a', 'arrowleft') ? 1 : 0) - (has('d', 'arrowright') ? 1 : 0);
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    for (const gp of pads) {
      if (!gp) continue;
      const dz = (v) => (Math.abs(v) < 0.12 ? 0 : v);
      if (dz(gp.axes[1])) thrIn = -gp.axes[1]; if (dz(gp.axes[0])) steerIn = -gp.axes[0];
      if (gp.buttons[1]?.pressed) requestEstop(sim, 'gamepad');
    }
    if (touchDrive.on && (touchDrive.thr || touchDrive.steer)) { thrIn = touchDrive.thr; steerIn = touchDrive.steer; }
    const active = thrIn || steerIn;
    if (active && A.mode === 'AUTO') A.override();
    if (A.mode === 'AUTO' || ui.manualHold) { drive.thr = drive.steer = 0; if (!active) return; ui.manualHold = false; }
    const precise = keys.has('shift') ? 0.3 : 1;
    const ramp = (cur, tgt, up, down) => { const r = Math.abs(tgt) > Math.abs(cur) && Math.sign(tgt || 1) === Math.sign(cur || tgt || 1) ? up : down; return cur + Math.max(-r * dt, Math.min(r * dt, tgt - cur)); };
    drive.thr = ramp(drive.thr, thrIn * precise, 0.5, 1.2);
    drive.steer = ramp(drive.steer, steerIn, 1.5, 2.5);
    if (!R.powered || R.estop) { R.cmd.vx = R.cmd.vz = R.cmd.wz = 0; return; }
    const vmax = R.speedCap();
    if (active && R.brake && !keys.has(' ')) { R.brake = false; }
    if (keys.has(' ')) { R.brake = true; drive.thr = 0; }
    // skid steer: steering is a yaw rate. Moving: curvature steering (R >= 12 m loaded). Stopped: slow point turn
    // (never loaded - the wheels would dig ruts with the CoM 6.8 m up).
    const vx = drive.thr * vmax;
    const kmax = R.loaded ? 1 / 20 : 1 / 8;
    let wz = Math.abs(vx) > 0.15 ? vx * drive.steer * kmax : (R.loaded ? 0 : drive.steer * 0.25 * precise);
    R.previewK = null;
    // turbo is only offered while driving straight at full throttle
    R.turboOk = drive.thr > 0.9 && Math.abs(drive.steer) < 0.05 && !R.loaded;
    if (sim.comms.delayS > 0) {
      // delayed link: the stick position reaches the rover one light time late, and speed is capped (move-and-wait driving)
      const cap = sim.comms.earthDriveCap, k = Math.abs(vx) > cap ? cap / Math.abs(vx) : 1, cvx = vx * k, cwz = wz * k;
      sim.comms.queue.push({ t: sim.time + sim.comms.delayS, fn: () => { if (A.mode !== 'AUTO' && R.powered && !R.estop) { R.cmd.vx = cvx; R.cmd.vz = 0; R.cmd.wz = cwz; } } });
    } else { R.cmd.vx = vx; R.cmd.vz = 0; R.cmd.wz = wz; }
    // ride height: R / F raise / lower (kneel -0.65 .. carry +0.82 m)
    const L = R.lift;
    if (keys.has('r')) L.target = Math.min(ROVER.ride.max, L.pos + 0.1);
    if (keys.has('f')) L.target = Math.max(ROVER.ride.kneel, L.pos - 0.1);
  }

  // ---------------- remembered map (teach & repeat survives reloads) ----------------
  if (qs.has('fresh')) { await DB.wipe(); await DiskMemory.forget(seed); qs.delete('fresh'); const rest = qs.toString(); history.replaceState(null, '', location.pathname + (rest ? '?' + rest : '')); sim.log('Fresh start requested (?fresh): stored map erased', 'warn'); }
  try { const disk = await DiskMemory.load(seed), d = disk || (await DB.get('map')); if (d && sim.slam.deserialize(d)) { sim.log(`Remembered SLAM map loaded from ${disk ? 'the rover memory store (disk)' : 'this browser'}: ${d.tiles.length} tiles, ${d.routes.length} proven route(s). The rover will reuse routes that worked.`, 'ok'); overlay.setRoutes(sim.slam.routes); } else sim.log('No stored map: the rover will explore and learn the route on its first trip (no pre-mapped path).', 'info'); } catch (e) { /* private mode */ }
  setInterval(() => ui.saveMap(true), 120000);
  window.addEventListener('beforeunload', () => { if (ui.noSave) return; try { const d = sim.slam.serialize(); DB.put('map', d); DiskMemory.saveBeacon(d); } catch (e) { /* ignore */ } });

  S.csmRegister(scene);
  // Adaptive quality: 'auto' walks down this ladder when the frame rate sags and back up when there is headroom
  const LADDER = [['high', 1], ['high', 0.85], ['medium', 0.85], ['medium', 0.7], ['low', 0.7], ['low', 0.55]];
  let rung = 0, auto = false, slowT = 0, fastT = 0, ema = 1 / 60, simBudget = 20, camEvery = 1;
  const warpT = { phys: 0, over: 8, every: 1, drew: true }; // time-warp frame accounting (ms): physics per frame, non-physics per drawn frame
  function applyRung(r) {
    rung = Math.max(0, Math.min(LADDER.length - 1, r)); const [q, sc] = LADDER[rung];
    S.setQuality(q, sc);
    simBudget = rung >= 4 ? 10 : rung >= 2 ? 14 : 20;   // ms of physics per frame
    camEvery = rung >= 4 ? 4 : rung >= 2 ? 3 : rung >= 1 ? 2 : 1; // onboard-camera refresh divisor
    // High-DPR monitors (e.g. 4K) leave less CPU headroom for physics
    if (window.devicePixelRatio > 1.5 && simBudget > 14) simBudget = 14;
  }
  ui.setQuality = (q) => {
    auto = q === 'auto'; ui.quality = q; slowT = fastT = 0;
    if (auto) { if (rung === 0 || rung === undefined) applyRung(startRung()); else applyRung(rung); } else { S.setQuality(q, 1); simBudget = 20; camEvery = 1; rung = 0; }
    try { localStorage.setItem('aegis-quality', q); } catch (e) { /* ignore */ }
    sim.log(`Graphics quality: ${q}`);
  };
  // first guess from the hardware; the governor corrects it within a few seconds
  function startRung() {
    let weak = 0;
    const cores = navigator.hardwareConcurrency || 4, mem = navigator.deviceMemory || 4;
    if (cores <= 4) weak++; if (mem <= 4) weak++;
    try { const gl = renderer.getContext(), ext = gl.getExtension('WEBGL_debug_renderer_info'); const r = ext ? String(gl.getParameter(ext.UNMASKED_RENDERER_WEBGL)) : '';
      if (/Intel|Mali|Adreno|PowerVR|SwiftShader|llvmpipe|Microsoft Basic/i.test(r) && !/Arc|Iris Xe/i.test(r)) weak += 2; } catch (e) { /* ignore */ }
    if (/Mobi|Android|iPhone|iPad/i.test(navigator.userAgent)) weak += 2;
    if (window.devicePixelRatio >= 2) weak++; // 4K display = more GPU work
    return weak >= 3 ? 3 : weak >= 2 ? 1 : 0;
  }
  // default = adaptive 'auto' (sizes itself to the device and steps down if the frame rate sags); a manual choice is remembered
  { let q = 'auto'; try { q = localStorage.getItem('aegis-quality') || 'auto'; } catch (e) { /* ignore */ } ui.setQuality(q); ui.quality = q; }
  sim.log('Type "help" for commands, H cycles the HUD layers, ? for controls. Try: mission cycle  |  W/A/S/D to drive  |  Esc = E-STOP', 'info');
  await step(100, 'Ready');
  document.getElementById('loading').style.display = 'none';

  // ---------------- frame loop ----------------
  const clock = new THREE.Clock();
  let frame = 0;
  const tmp = new THREE.Vector3();
  const _cameraWant = new THREE.Vector3();
  function loop() {
    requestAnimationFrame(loop);
    const rawDt = clock.getDelta();
    const dt = Math.min(rawDt, 0.1);
    // (the governor is frozen above 1x: a frame stretched by time-warp physics is not a slow GPU, and stepping the quality and the
    // physics budget down for it starved the very warp that caused it)
    if (auto && !document.hidden && rawDt < 0.25 && sim.timeScale <= 1) {   // ignore tab-switch stalls
      ema += (rawDt - ema) * 0.08;
      if (ema > 1 / 42) { slowT += rawDt; fastT = 0; } else if (ema < 1 / 56) { fastT += rawDt; slowT = 0; } else { slowT = fastT = 0; }
      if (slowT > 1.5 && rung < LADDER.length - 1) { applyRung(rung + 1); slowT = 0; ema = 1 / 50; }
      else if (fastT > 12 && rung > 0) { applyRung(rung - 1); fastT = 0; ema = 1 / 50; }
    }
    manualInput(dt);
    // What the previous frame spent outside the physics (drawing, presenting, waiting for the display): a running mean over drawn frames.
    if (warpT.drew && rawDt < 0.25) warpT.over += (Math.max(0, rawDt * 1000 - warpT.phys) - warpT.over) * 0.15;
    const tPhys = performance.now();
    const simDt = sim.advance(dt, simBudget); // sim seconds this frame (many at 50x)
    sim.losMonitor.update();
    warpT.phys = performance.now() - tPhys;
    const R = sim.rover, p = R.pose();
    roverModel.update(R, sim.time);
    armVis.update(R);
    syncAcmVis(R);
    powerDockVis.update(dt);
    { const sc = sim.scout; scoutVis.visible = true; scoutVis.position.set(sc.x, sc.y, sc.z);
      scoutVis.rotation.set(0, 0, 0); scoutVis.rotateY(sc.yaw); scoutVis.rotateZ(sc.pitch); scoutVis.rotateX(-sc.roll);
      scoutTag.visible = sc.enabled; if (scoutVis.userData.lidar) scoutVis.userData.lidar.rotation.z = sim.time * 40;
      if (sc.lastScan && ui.overlays.lidar && sc.enabled) { scoutCloud.geometry.setAttribute('position', new THREE.Float32BufferAttribute(sc.lastScan, 3)); scoutCloud.visible = true; sc.lastScan = null; }
      if (!sc.enabled || !ui.overlays.lidar) scoutCloud.visible = false; }
    for (const v of cargoVis) {
      if (v.c.T.handling === 'arm') continue;   // ACM: posed by syncAcmVis (ground body / hook trunnions / deck pins)
      const cad = v.g.userData.cad; if (cad && cad.userData.xray !== ui.overlays.xray) { cad.userData.setXray(ui.overlays.xray); v.g.traverse((o) => { if (o.userData.xrayable) { o.material = o.material.clone(); o.material.transparent = true; o.material.opacity = ui.overlays.xray ? 0.15 : 1; o.material.depthWrite = !ui.overlays.xray; } }); }
      const t = v.c.body.translation(), r = v.c.body.rotation();
      v.g.position.set(t.x, t.y, t.z); v.g.quaternion.set(r.x, r.y, r.z, r.w);
      if (v.c.hatchOpen && v.g.userData.hatch) v.g.userData.hatch.scale.setScalar(Math.max(0.02, v.g.userData.hatch.scale.x - dt * 0.4));
    }
    updateJacks();
    for (const pv of hab.portVis) {
      pv.bellows.scale.z = 1 + (pv.port.occupied ? 2.5 : 0);
      pv.status.material.emissive.setHex(pv.port.hatchOpen ? 0x33ff88 : pv.port.occupied ? 0xffa31a : sim.autonomy.faults.port[pv.port.id] ? 0xff2010 : 0x3d9bff);
      if (pv.port.hatchOpen) pv.door.scale.setScalar(Math.max(0.02, pv.door.scale.x - dt * 0.4));
    }
    // effects run on a capped step: at 50x one frame covers seconds of sim time and would spawn thousands of dust particles
    const fxDt = Math.min(simDt, 0.1);
    if (ui.overlays.dust && fxDt > 0) { dust.emit(R, fxDt); dust.update(fxDt); }
    dust.points.visible = ui.overlays.dust;
    if (simDt > 0) tracks.update(R);
    lidar.points.visible = ui.overlays.lidar;
    overlay.pathLine.visible = ui.overlays.path && !!sim.autonomy.gen;
    overlay.routeGroup.visible = ui.overlays.routes;
    fences.visible = ui.overlays.fences;
    if (frame % 6 === 0) overlay.setArcs(sim.autonomy.local.last, sim.slam.est, ui.overlays.arcs && sim.autonomy.mode === 'AUTO');
    overlay.setGoal(sim.autonomy.gen ? sim.autonomy.goal : null);
    engOverlay.update(R, ui.overlays.eng && ui.view !== 'cam' && !(sim.warp.active && sim.timeScale >= sim.warp.minScale));

    // camera follow
    const target = tmp.set(p.x, p.y - 1, p.z);
    if (ui.view === 'chase') {
      _cameraWant.set(p.x - p.fwd.x * 30, p.y + 11, p.z - p.fwd.z * 30);
      const d = target.clone().sub(lastTarget); camera.position.add(d);
      camera.position.lerp(_cameraWant, 1 - Math.exp(-dt * 0.8));
      controls.target.copy(target);
    } else if (ui.view === 'orbit' || ui.view === 'top') {
      const d = target.clone().sub(lastTarget); camera.position.add(d); controls.target.add(d);
    }
    lastTarget.copy(target);
    if (ui.view === 'cam') {
      // the onboard view is video downlinked over the operator link: it arrives one light time late
      const cpNow = sim.sensors.cameraPose();
      camDelay.push(sim.time, { pos: cpNow.pos, q: cpNow.q });
      const cp = sim.comms.delayS > 0 ? camDelay.at(sim.time - sim.comms.delayS) : cpNow;
      camera.position.copy(cp.pos); camera.quaternion.copy(cp.q).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2));
      camera.fov = 52;
    } else { camera.fov = 52; controls.update(); }
    if (!(camera.aspect > 0)) camera.aspect = 16 / 9;
    // Keep the rover in the part of the window the panels leave free. On a 900 px-high laptop screen the instruments
    // panel sits over the middle of the view, exactly where the followed rover is drawn. The picture is shifted up by
    // a lens-shift (view offset), which moves nothing in the world and keeps mouse picking exact.
    if (frame % 20 === 0) viewShiftWant = ui.view === 'cam' ? 0 : freeAreaShift();
    if (Math.abs(viewShiftWant - viewShift) > 0.5 || window.innerWidth !== vsW || window.innerHeight !== vsH) {
      viewShift += (viewShiftWant - viewShift) * Math.min(1, dt * 6);
      const W = vsW = window.innerWidth, H = vsH = window.innerHeight;   // the offset is in window pixels: redo it after a resize
      if (Math.abs(viewShift) < 1 && viewShiftWant === 0) { viewShift = 0; camera.clearViewOffset(); } else camera.setViewOffset(W, H, 0, Math.round(viewShift), W, H);
      camera.updateProjectionMatrix(); S.csm.updateFrustums();
    }
    camera.updateProjectionMatrix();

    // Draw throttle, only while the time warp is CPU-limited (sim.lagging above 1x). Each drawn frame costs warpT.over ms that the
    // physics does not get; when that is large (a 4K display, software rendering) the scene is drawn every 2nd-4th frame instead,
    // never less often than ~8 times a second, and the frames in between are physics only. When drawing is cheap, or the sim keeps
    // up (1x, or cruising on the transit-warp rails), every frame is drawn as always.
    warpT.every = sim.lagging && sim.timeScale > 1 && !sim.paused ? Math.max(1, Math.min(Math.round(warpT.over / 15), Math.floor(120 / Math.max(20, warpT.phys)), 4)) : 1;
    warpT.drew = warpT.every <= 1 || frame % warpT.every === 0;
    if (warpT.drew) {
      // shadows follow the area of interest
      const focus = ui.view === 'cam' ? new THREE.Vector3(p.x, p.y, p.z) : controls.target;
      S.sun.position.copy(focus).addScaledVector(sim.sunDir, 400); S.sun.target.position.copy(focus);
      S.sky.position.copy(camera.position);
      if (frame % 4 === 0) terrainMesh.update(camera.position, 6);
      if (frame % 10 === 0) pebbles.update(ui.view === 'cam' ? p : controls.target);

      if (frame % 90 === 0) S.csmRegister(scene);
      S.csm.update(); S.csmGuard(); S.shadowSchedule(frame);
      renderer.shadowMap.autoUpdate = true;
      S.composer.render();
      // onboard camera feed (scissored into the HUD frame)
      const rect = hud.camRect();
      if (rect && ui.view !== 'cam') {
        const H = window.innerHeight;
        const redraw = camEvery <= 1 || frame % camEvery === 0 || !feedRT.ready;
        if (camEvery <= 1) {
          // full rate: straight into the HUD frame (scissored)
          const cp = sim.sensors.cameraPose();
          roverCam.position.copy(cp.pos); roverCam.quaternion.copy(cp.q).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2));
          roverCam.updateMatrixWorld();
          renderer.shadowMap.autoUpdate = false;
          renderer.setScissorTest(true);
          renderer.setScissor(rect.left, H - rect.bottom, rect.width, rect.height); renderer.setViewport(rect.left, H - rect.bottom, rect.width, rect.height);
          const vis = lidar.points.visible; lidar.points.visible = false;
          renderer.autoClear = true; renderer.render(scene, roverCam);
          lidar.points.visible = vis;
          renderer.setScissorTest(false); renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);
        } else {
          // reduced rate: render the onboard feed into a small target every few frames, blit it every frame
          if (redraw) {
            const cp = sim.sensors.cameraPose();
            roverCam.position.copy(cp.pos); roverCam.quaternion.copy(cp.q).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2));
            roverCam.updateMatrixWorld();
            const vis = lidar.points.visible; lidar.points.visible = false;
            renderer.shadowMap.autoUpdate = false;
            renderer.setRenderTarget(feedRT); renderer.autoClear = true; renderer.render(scene, roverCam); renderer.setRenderTarget(null);
            lidar.points.visible = vis; feedRT.ready = true;
          }
          renderer.setScissorTest(true);
          renderer.setScissor(rect.left, H - rect.bottom, rect.width, rect.height); renderer.setViewport(rect.left, H - rect.bottom, rect.width, rect.height);
          renderer.autoClear = false; renderer.render(feedScene, feedCam); renderer.autoClear = true;
          renderer.setScissorTest(false); renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);
        }
      }
    }
    hud.update(dt);
    frame++;
  }
  loop();
  window.sim = sim; window.app = { armVis, camera, controls, ui, scene, renderer, roverModel, warpT, applyRung: (r) => { auto = false; applyRung(r); } }; // for debugging in the browser console
}

main().catch((e) => { console.error(e); loadMsg.textContent = 'Error: ' + e.message; loadMsg.style.color = '#ff4d3a'; });
