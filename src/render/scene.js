// Renderer, camera, lunar sky (black sky, sun disc, Earth, faint stars), sun/earthshine lighting
// and post-processing (bloom + ACES tone mapping).
import * as THREE from 'three';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/examples/jsm/postprocessing/OutputPass.js';
import { SMAAPass } from 'three/examples/jsm/postprocessing/SMAAPass.js';
import { ShaderPass } from 'three/examples/jsm/postprocessing/ShaderPass.js';
import { CSM } from 'three/examples/jsm/csm/CSM.js';
import { Lensflare, LensflareElement } from 'three/examples/jsm/objects/Lensflare.js';
import { mulberry32 } from '../sim/terrain.js';

// Lunar lighting budget (docs/changes/C.md). On the Moon the sky is black: there is no blue skylight from above.
//   SUN_I        direct sunlight (render units).
//   FILL_FRAC    regolith bounce, from below and opposite the sun, warm (regolith albedo ~0.12): <= 1 % of the sun, scaled by
//                how much sun falls on the ground (sin elevation, normalised to the 9 deg site default).
//   EARTHSHINE   full-Earth illuminance ~1/10,000 of sunlight, from the Earth's direction, times the Earth's lit fraction.
//   NIGHT_GAIN   a camera exposing for earthshine (~8.6 stops longer than for sunlit regolith). It is applied to the earthshine
//                light, not to toneMappingExposure, so lamps, screens and the sun-lit HUD do not blow out at night.
// The Earth does not move with the sun: it hangs at a fixed azimuth ~10 deg up (site near 80 deg S), nodding by the
// +-6.7 deg latitude libration (and +-7.9 deg longitude libration in azimuth) over the lunar day; its phase follows from
// the sun-Earth angle seen from the site.
const SUN_I = 5.2;
const FILL_FRAC = 0.01;
const EARTHSHINE = 1e-4;
const NIGHT_GAIN = 400;
const EARTH_AZ = 314 * Math.PI / 180, EARTH_EL = 10 * Math.PI / 180, EARTH_DIST = 26000;
const LIB_LAT = 6.7 * Math.PI / 180, LIB_LON = 7.9 * Math.PI / 180;
const SUN_AZ0 = 205 * Math.PI / 180;   // sim default sun azimuth (sim.js): libration phase 0 there, so the default view is unchanged

export function createScene(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: false, logarithmicDepthBuffer: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.75));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x000000);
  const camera = new THREE.PerspectiveCamera(52, window.innerWidth / window.innerHeight, 0.15, 40000);

  // --- sun: no atmosphere -> hard, near-white light, pitch-black sky
  const sun = new THREE.DirectionalLight(0xfff6ea, SUN_I);
  sun.castShadow = true;
  sun.shadow.mapSize.set(4096, 4096);
  const sc = sun.shadow.camera;
  sc.left = -70; sc.right = 70; sc.top = 70; sc.bottom = -70; sc.near = 1; sc.far = 900;
  sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.05;
  scene.add(sun, sun.target);
  // fill: black sky above, sunlit regolith below (warm). Together with `bounce` it is FILL_FRAC of the sun (see the top).
  const fill = new THREE.HemisphereLight(0x000000, 0x8a7e6c, 0);
  scene.add(fill);
  const bounce = new THREE.DirectionalLight(0xd8c8b0, 0); // from the ground opposite the sun, pointing up
  scene.add(bounce, bounce.target);
  // earthshine: sunlight reflected by the Earth, from the Earth's direction, slightly blue-white
  const earthshine = new THREE.DirectionalLight(0xcdd8ff, 0);
  scene.add(earthshine, earthshine.target);

  // image-based lighting for metals: black sky above, sunlit regolith below, the sun as a hot spot
  const pmrem = new THREE.PMREMGenerator(renderer);
  const envScene = new THREE.Scene();
  const envGeo = new THREE.SphereGeometry(10, 48, 24);
  const envMat = new THREE.ShaderMaterial({ side: THREE.BackSide, uniforms: { sun: { value: new THREE.Vector3(0.9, 0.16, -0.4) }, ground: { value: 0.42 } },
    vertexShader: 'varying vec3 vd; void main(){ vd = normalize(position); gl_Position = projectionMatrix*modelViewMatrix*vec4(position,1.0); }',
    fragmentShader: 'uniform vec3 sun; uniform float ground; varying vec3 vd; void main(){ float g = smoothstep(0.02, -0.12, vd.y); vec3 c = mix(vec3(0.0), vec3(1.0,0.97,0.92) * ground, g); c += vec3(40.0,38.0,35.0) * step(0.0, sun.y) * pow(max(dot(vd, normalize(sun)), 0.0), 900.0); gl_FragColor = vec4(c, 1.0); }' });
  envScene.add(new THREE.Mesh(envGeo, envMat));
  let _lastEnvUpdate = -999;
  function updateEnv(dir, ground) { envMat.uniforms.sun.value.copy(dir); envMat.uniforms.ground.value = ground; const now = performance.now(); if (now - _lastEnvUpdate < 2000) return; _lastEnvUpdate = now; const rt = pmrem.fromScene(envScene, 0.02); if (scene.environment) scene.environment.dispose(); scene.environment = rt.texture; }
  scene.environmentIntensity = 0.9;

  // --- sky dome objects
  const sky = new THREE.Group(); scene.add(sky);
  // stars: visible only faintly (a real camera exposing for sunlit regolith would not see them)
  {
    const rand = mulberry32(42);
    const N = 6000, pos = new Float32Array(N * 3), col = new Float32Array(N * 3);
    for (let i = 0; i < N; i++) {
      const u = rand() * 2 - 1, th = rand() * Math.PI * 2, r = 30000;
      const s = Math.sqrt(1 - u * u);
      pos[i * 3] = r * s * Math.cos(th); pos[i * 3 + 1] = r * Math.abs(u) * 0.98 + 200; pos[i * 3 + 2] = r * s * Math.sin(th);
      const b = Math.pow(rand(), 6) * 0.9 + 0.05, t = rand();
      col[i * 3] = b * (0.85 + 0.15 * t); col[i * 3 + 1] = b * 0.9; col[i * 3 + 2] = b * (1.05 - 0.15 * t);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    const m = new THREE.PointsMaterial({ size: 1.6, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: 0.55, depthWrite: false });
    const stars = new THREE.Points(g, m); stars.frustumCulled = false; stars.renderOrder = -2;
    sky.add(stars);
    sky.userData.stars = stars;
  }
  // sun disc + glare
  const sunDisc = new THREE.Mesh(new THREE.SphereGeometry(140, 24, 16), new THREE.MeshBasicMaterial({ color: new THREE.Color(8, 7.6, 7) }));
  const glare = new THREE.Sprite(new THREE.SpriteMaterial({ map: glareTexture(), color: 0xffffff, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, opacity: 0.85 }));
  glare.scale.set(2600, 2600, 1);
  sky.add(sunDisc, glare);
  // Earth: ~2 deg across, ~10 deg above the horizon. The sim's sun peaks at 9 deg, which puts the site near 80 deg S;
  // there the Earth hangs about (90 - latitude) up, not on the horizon as it would at the pole itself.
  // Lit by the real sun direction in its own shader (the scene's sun lights are off at night, when the Earth is near full):
  // the terminator, and so the phase, follow from the sun-Earth geometry. Log-depth chunks: the renderer uses a log depth buffer.
  const earthMat = new THREE.ShaderMaterial({
    uniforms: { map: { value: earthTexture() }, sunDir: { value: new THREE.Vector3(1, 0, 0) } },
    vertexShader: '#include <common>\n#include <logdepthbuf_pars_vertex>\nvarying vec2 vUv; varying vec3 vN; void main(){ vUv = uv; vN = normalize(mat3(modelMatrix) * normal); gl_Position = projectionMatrix*modelViewMatrix*vec4(position,1.0);\n#include <logdepthbuf_vertex>\n}',
    fragmentShader: '#include <common>\n#include <logdepthbuf_pars_fragment>\nuniform sampler2D map; uniform vec3 sunDir; varying vec2 vUv; varying vec3 vN; void main(){\n#include <logdepthbuf_fragment>\nvec3 t = texture2D(map, vUv).rgb; float d = max(dot(normalize(vN), sunDir), 0.0); gl_FragColor = vec4(t * (1.6 * d + 0.004), 1.0); }',
  });
  const earth = new THREE.Mesh(new THREE.SphereGeometry(520, 48, 32), earthMat);
  sky.add(earth);
  const earthDir = new THREE.Vector3();

  const composer = new EffectComposer(renderer, new THREE.WebGLRenderTarget(1, 1, { type: THREE.HalfFloatType }));
  composer.addPass(new RenderPass(scene, camera));
  // NaN/Inf guard: one bad pixel must stay one pixel - bloom's blur and SMAA would otherwise spread it into
  // a screen-sized black blob. Also caps HDR (well above the sun disc) so bloom can't overflow half-float.
  composer.addPass(new ShaderPass({
    uniforms: { tDiffuse: { value: null } },
    vertexShader: 'varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix*modelViewMatrix*vec4(position,1.0); }',
    fragmentShader: 'uniform sampler2D tDiffuse; varying vec2 vUv; void main(){ vec4 c = texture2D(tDiffuse, vUv); if (any(isnan(c)) || any(isinf(c))) c = vec4(0.0, 0.0, 0.0, 1.0); c.rgb = clamp(c.rgb, 0.0, 1000.0); gl_FragColor = c; }',
  }));
  const bloom = new UnrealBloomPass(new THREE.Vector2(window.innerWidth, window.innerHeight), 0.22, 0.45, 1.35);
  composer.addPass(bloom);
  const smaa = new SMAAPass(window.innerWidth, window.innerHeight);
  composer.addPass(smaa);
  composer.addPass(new OutputPass());
  composer.setSize(window.innerWidth, window.innerHeight);

  // Cascaded shadow maps: crisp contact shadows near the rover AND long low-sun shadows to the horizon
  sun.castShadow = false; sun.visible = false;
  const csm = new CSM({ camera, parent: scene, cascades: 4, maxFar: 1800, mode: 'practical', shadowMapSize: 2048, lightDirection: new THREE.Vector3(0, -1, 0), lightIntensity: 5.2, lightNear: 1, lightFar: 6000, lightMargin: 400, shadowBias: -0.00005 });
  // Depth bias is a fraction of the 6 km light range: -0.00025 was 1.5 m, so anything under ~1.5 m tall (SCOUT-1, small
  // boulders, footpads) cast a shadow that started a body length away from it. -0.00005 is 0.3 m; no acne seen on the
  // terrain or the rover at a 10 deg sun with the 2 cm normal offset.
  for (const l of csm.lights) { l.color.set(0xfff6ea); l.shadow.normalBias = 0.02; }
  const csmDone = new WeakSet();
  function csmRegister(root) {
    root.traverse((o) => {
      if (!o.isMesh && !o.isInstancedMesh) return;
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      for (const m of mats) {
        if (!m || csmDone.has(m) || !(m.isMeshStandardMaterial || m.isMeshPhongMaterial || m.isMeshLambertMaterial)) continue;
        csmDone.add(m);
        const old = m.onBeforeCompile && m.onBeforeCompile !== THREE.Material.prototype.onBeforeCompile ? m.onBeforeCompile : null;
        csm.setupMaterial(m);
        const csmFn = m.onBeforeCompile;
        m.onBeforeCompile = (sh, r) => { csmFn(sh, r); if (old) old(sh, r); };
        const key = 'csm' + (old ? '+' + (m.userData.shaderKey || 'custom') : '');
        m.customProgramCacheKey = () => key;
        m.needsUpdate = true;
      }
    });
  }

  // sun lens flare (camera optics; the sun itself has no atmosphere halo on the Moon)
  const flare = new Lensflare();
  const flareTex = (r, a) => { const c = document.createElement('canvas'); c.width = c.height = 128; const g = c.getContext('2d'); const gr = g.createRadialGradient(64, 64, 0, 64, 64, 64); gr.addColorStop(0, `rgba(255,255,255,${a})`); gr.addColorStop(r, `rgba(255,240,220,${a * 0.35})`); gr.addColorStop(1, 'rgba(0,0,0,0)'); g.fillStyle = gr; g.fillRect(0, 0, 128, 128); const t = new THREE.CanvasTexture(c); return t; };
  flare.addElement(new LensflareElement(flareTex(0.2, 1), 380, 0, new THREE.Color(1, 0.97, 0.9)));
  flare.addElement(new LensflareElement(flareTex(0.5, 0.35), 60, 0.4, new THREE.Color(0.6, 0.8, 1)));
  flare.addElement(new LensflareElement(flareTex(0.5, 0.3), 110, 0.65, new THREE.Color(0.9, 0.7, 1)));
  flare.addElement(new LensflareElement(flareTex(0.6, 0.25), 180, 0.95, new THREE.Color(0.7, 1, 0.8)));
  sunDisc.add(flare);

  // graphics quality presets
  let baseQ = 'high', renderScale = 1;
  function setQuality(q, scale = 1) {
    baseQ = q; renderScale = scale;
    const pr = ({ low: 0.75, medium: 1.0, high: Math.min(window.devicePixelRatio, 1.5), ultra: Math.min(window.devicePixelRatio, 2) }[q] ?? 1) * scale;
    renderer.setPixelRatio(Math.max(0.5, pr));
    // never ask for a shadow map bigger than the GPU supports (some browsers/GPUs silently render black)
    // ultra: 4096 on the two near cascades (rover / contact shadows, where texels show), 2048 on the far ones (their texels are
    // metres wide anyway): ~2x the shadow memory of high, not 4x
    const smMax = renderer.capabilities.maxTextureSize || 2048;
    const smFor = (i) => Math.min(q === 'ultra' ? (i < 2 ? 4096 : 2048) : ({ low: 1024, medium: 2048, high: 2048 }[q] ?? 2048), smMax);
    for (const [i, l] of csm.lights.entries()) { const sm = smFor(i); if (l.shadow.mapSize.x !== sm) { l.shadow.mapSize.set(sm, sm); if (l.shadow.map) { l.shadow.map.dispose(); l.shadow.map = null; } } }
    // the cascades only refresh every few frames; force them all to redraw now so a resized/disposed map is never shown empty (black boxes)
    for (const l of csm.lights) l.shadow.needsUpdate = true;
    bloom.enabled = q !== 'low'; smaa.enabled = q === 'ultra' || (q !== 'low' && scale >= 0.7); flare.visible = q !== 'low';
    setAnisotropy(q === 'ultra' ? renderer.capabilities.getMaxAnisotropy() : 0);
    resize();
  }
  // Texture filtering: ultra raises every texture to the GPU's maximum anisotropy (cheap, sharper ground at grazing angles);
  // the other tiers restore each texture's own setting. Textures added after a quality change pick it up at the next one.
  let anisoMax = 0;
  function setAnisotropy(max) {
    anisoMax = max;
    scene.traverse((o) => {
      const mats = o.material ? (Array.isArray(o.material) ? o.material : [o.material]) : [];
      for (const m of mats) for (const k of ['map', 'normalMap', 'roughnessMap', 'aoMap', 'bumpMap', 'emissiveMap', 'metalnessMap']) {
        const t = m[k]; if (!t || !t.isTexture) continue;
        if (t.userData.anisoBase === undefined) t.userData.anisoBase = t.anisotropy;
        const want = max > 0 ? Math.max(t.userData.anisoBase, max) : t.userData.anisoBase;
        if (t.anisotropy !== want) { t.anisotropy = want; t.needsUpdate = true; }
      }
    });
  }
  // Shadow cascades: the near cascade (rover contact shadows) updates every frame, the far ones less often
  // (their texels are metres wide, so a 2-6 frame lag is invisible). Cuts the shadow pass 2-3x.
  for (const l of csm.lights) l.shadow.autoUpdate = false;
  const CASCADE_EVERY = { ultra: [1, 1, 2, 2], high: [1, 2, 3, 6], medium: [1, 2, 4, 8], low: [2, 4, 8, 16] };
  function shadowSchedule(frame) {
    const ev = CASCADE_EVERY[baseQ] || CASCADE_EVERY.high;
    csm.lights.forEach((l, i) => { if (frame % (ev[i] ?? 8) === 0) l.shadow.needsUpdate = true; });
  }

  function setSun(dir) {
    sunDisc.position.copy(dir).multiplyScalar(22000);
    glare.position.copy(dir).multiplyScalar(21000);
    // Earth: fixed azimuth and elevation, plus libration. The sun azimuth advances one turn per lunar day, so it stands in
    // for the libration phase (the libration periods, 27.2 and 27.6 d, are close to the 29.5 d synodic month).
    const ph = Math.atan2(-dir.z, dir.x) - SUN_AZ0;
    const eAz = EARTH_AZ + LIB_LON * Math.sin(ph), eEl = EARTH_EL + LIB_LAT * Math.sin(ph + 1.1);
    earthDir.set(Math.cos(eEl) * Math.cos(eAz), Math.sin(eEl), -Math.cos(eEl) * Math.sin(eAz));
    earth.position.copy(earthDir).multiplyScalar(EARTH_DIST);
    earth.lookAt(0, 0, 0);
    earthMat.uniforms.sunDir.value.copy(dir).normalize();
    // Earth's lit fraction seen from the Moon: (1 - cos(sun-Earth elongation)) / 2 (full when the sun is opposite it)
    const earthLit = (1 - dir.dot(earthDir) / dir.length()) / 2;
    const day = THREE.MathUtils.smoothstep(dir.y, -0.03, 0.03);
    const groundSun = day * THREE.MathUtils.clamp(dir.y / Math.sin(9 * Math.PI / 180), 0, 1.5);   // sun on the ground vs the 9 deg default
    sun.intensity = dir.y > 0 ? SUN_I : 0;
    csm.lightDirection.copy(dir).negate();
    for (const l of csm.lights) l.intensity = dir.y > 0 ? SUN_I : 0;
    // regolith bounce: from below, opposite the sun (the lit ground on the sun side reflects back toward shadowed faces)
    bounce.position.set(-dir.x * 100, -40, -dir.z * 100); bounce.target.position.set(0, 0, 0);
    bounce.intensity = SUN_I * FILL_FRAC * 0.7 * groundSun;
    fill.intensity = SUN_I * FILL_FRAC * 0.3 * groundSun;
    // earthshine: ~1e-4 of the sun times the lit fraction, with the night camera gain once the sun is down
    const nightK = 1 - day;
    earthshine.position.copy(earthDir).multiplyScalar(100); earthshine.target.position.set(0, 0, 0);
    earthshine.intensity = SUN_I * EARTHSHINE * earthLit * (1 + (NIGHT_GAIN - 1) * nightK);
    // IBL ground: the regolith radiance under the sun (warm, ~0.1 at the default sun), or earthshine-lit at night
    updateEnv(dir, 0.1 * groundSun + 0.02 * earthLit * nightK);
    // modest exposure lift at night (the gain sits on the earthshine light), so the night scene stays dark, not washed out
    renderer.toneMappingExposure = 1.05 + 0.35 * nightK;
    if (sky.userData.stars) sky.userData.stars.material.opacity = 0.55 + 0.4 * nightK;
  }
  function resize() {
    if (!(window.innerWidth > 0 && window.innerHeight > 0)) return; // minimised / hidden: keep last valid size
    renderer.setSize(window.innerWidth, window.innerHeight);
    composer.setSize(window.innerWidth, window.innerHeight);
    if (typeof smaa !== 'undefined') smaa.setSize(window.innerWidth, window.innerHeight);
    if (typeof csm !== 'undefined') csm.updateFrustums();
    camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize);
  // GPU context loss (common on Safari / low-memory GPUs): keep the page alive and rebuild everything when it comes back
  renderer.domElement.addEventListener('webglcontextlost', (e) => { e.preventDefault(); });
  renderer.domElement.addEventListener('webglcontextrestored', () => {
    for (const l of csm.lights) { if (l.shadow.map) { l.shadow.map.dispose(); l.shadow.map = null; } l.shadow.needsUpdate = true; }
    setQuality(baseQ === 'ultra' ? 'high' : baseQ, renderScale);
  });
    // self-heal: recompute cascade frustums if they were ever built from an invalid camera
  function csmGuard() {
    const l = csm.lights[0];
    if (!Number.isFinite(l.position.x) || !Number.isFinite(camera.aspect) || camera.aspect <= 0) {
      if (window.innerWidth > 0 && window.innerHeight > 0) { camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix(); csm.updateFrustums(); }
    }
  }
  return { renderer, scene, camera, sun, fill, bounce, earthshine, sky, composer, bloom, setSun, earth, csm, csmRegister, setQuality, smaa, csmGuard, shadowSchedule, getQuality: () => ({ q: baseQ, scale: renderScale }) };
}

function glareTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 256;
  const g = c.getContext('2d');
  const grd = g.createRadialGradient(128, 128, 0, 128, 128, 128);
  grd.addColorStop(0, 'rgba(255,255,250,1)'); grd.addColorStop(0.06, 'rgba(255,250,235,0.85)');
  grd.addColorStop(0.2, 'rgba(255,240,220,0.15)'); grd.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = grd; g.fillRect(0, 0, 256, 256);
  // faint diffraction spikes
  g.globalCompositeOperation = 'lighter'; g.strokeStyle = 'rgba(255,250,240,0.18)'; g.lineWidth = 2;
  for (let i = 0; i < 4; i++) { const a = i * Math.PI / 4; g.beginPath(); g.moveTo(128 - Math.cos(a) * 128, 128 - Math.sin(a) * 128); g.lineTo(128 + Math.cos(a) * 128, 128 + Math.sin(a) * 128); g.stroke(); }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}

function earthTexture() {
  const W = 512, H = 256, c = document.createElement('canvas'); c.width = W; c.height = H;
  const g = c.getContext('2d'); const img = g.createImageData(W, H);
  const rand = mulberry32(9);
  const blobs = Array.from({ length: 26 }, () => ({ x: rand() * W, y: H * (0.15 + rand() * 0.7), r: 20 + rand() * 60 }));
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    let land = 0; for (const b of blobs) { const dx = Math.min(Math.abs(x - b.x), W - Math.abs(x - b.x)), dy = y - b.y; land = Math.max(land, 1 - Math.hypot(dx, dy * 1.4) / b.r); }
    const cloud = 0.5 + 0.5 * Math.sin(x * 0.07 + Math.sin(y * 0.05) * 3) * Math.cos(y * 0.11 + x * 0.013);
    const lat = Math.abs(y / H - 0.5) * 2;
    let r = 18, gg = 48, b = 110;
    if (land > 0.25) { r = 95 + 40 * land; gg = 105 + 25 * land; b = 70; }
    if (lat > 0.82) { r = gg = b = 235; }
    if (cloud > 0.78) { const k = (cloud - 0.78) * 4.5; r += (245 - r) * k; gg += (245 - gg) * k; b += (250 - b) * k; }
    const i = (y * W + x) * 4; img.data[i] = r; img.data[i + 1] = gg; img.data[i + 2] = b; img.data[i + 3] = 255;
  }
  g.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}
