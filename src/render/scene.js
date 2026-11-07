// Renderer, camera, lunar sky (black sky, sun disc, Earth, faint stars), sun/earthshine lighting
// and post-processing (bloom + ACES tone mapping).
import * as THREE from 'three';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/examples/jsm/postprocessing/OutputPass.js';
import { SMAAPass } from 'three/examples/jsm/postprocessing/SMAAPass.js';
import { CSM } from 'three/examples/jsm/csm/CSM.js';
import { Lensflare, LensflareElement } from 'three/examples/jsm/objects/Lensflare.js';
import { mulberry32 } from '../sim/terrain.js';

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
  const sun = new THREE.DirectionalLight(0xfff6ea, 5.2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(4096, 4096);
  const sc = sun.shadow.camera;
  sc.left = -70; sc.right = 70; sc.top = 70; sc.bottom = -70; sc.near = 1; sc.far = 900;
  sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.05;
  scene.add(sun, sun.target);
  // regolith multiple-scattering + earthshine: very weak fill so shadows stay deep but readable
  // fill light: earthshine from above plus sunlight bounced off the bright regolith (albedo ~0.12),
  // which is what lets cameras see detail inside lunar shadows
  const fill = new THREE.HemisphereLight(0x9fb3d6, 0x6b665e, 0.16);
  scene.add(fill);
  const bounce = new THREE.DirectionalLight(0xcfc6b8, 0.3); // opposite the sun, near horizontal
  scene.add(bounce, bounce.target);

  // image-based lighting for metals: black sky above, sunlit regolith below, the sun as a hot spot
  const pmrem = new THREE.PMREMGenerator(renderer);
  const envScene = new THREE.Scene();
  const envGeo = new THREE.SphereGeometry(10, 48, 24);
  const envMat = new THREE.ShaderMaterial({ side: THREE.BackSide, uniforms: { sun: { value: new THREE.Vector3(0.9, 0.16, -0.4) } },
    vertexShader: 'varying vec3 vd; void main(){ vd = normalize(position); gl_Position = projectionMatrix*modelViewMatrix*vec4(position,1.0); }',
    fragmentShader: 'uniform vec3 sun; varying vec3 vd; void main(){ float g = smoothstep(0.02, -0.12, vd.y); vec3 c = mix(vec3(0.0), vec3(0.42,0.41,0.39), g); c += vec3(0.06,0.07,0.09) * smoothstep(0.0, 0.5, vd.y) * 0.3; c += vec3(40.0,38.0,35.0) * pow(max(dot(vd, normalize(sun)), 0.0), 900.0); gl_FragColor = vec4(c, 1.0); }' });
  envScene.add(new THREE.Mesh(envGeo, envMat));
  function updateEnv(dir) { envMat.uniforms.sun.value.copy(dir); const rt = pmrem.fromScene(envScene, 0.02); if (scene.environment) scene.environment.dispose(); scene.environment = rt.texture; }
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
  // Earth (from the south pole it sits low over the horizon, ~2 deg apparent)
  const earth = new THREE.Mesh(new THREE.SphereGeometry(520, 48, 32), new THREE.MeshStandardMaterial({ map: earthTexture(), roughness: 0.9, metalness: 0, emissive: 0x0a1630, emissiveIntensity: 0.25 }));
  sky.add(earth);

  const composer = new EffectComposer(renderer, new THREE.WebGLRenderTarget(1, 1, { samples: 4, type: THREE.HalfFloatType }));
  composer.addPass(new RenderPass(scene, camera));
  const bloom = new UnrealBloomPass(new THREE.Vector2(window.innerWidth, window.innerHeight), 0.22, 0.45, 1.35);
  composer.addPass(bloom);
  composer.addPass(new OutputPass());
  const smaa = new SMAAPass(window.innerWidth * renderer.getPixelRatio(), window.innerHeight * renderer.getPixelRatio());
  composer.addPass(smaa);
  composer.setSize(window.innerWidth, window.innerHeight);

  // Cascaded shadow maps: crisp contact shadows near the rover AND long low-sun shadows to the horizon
  sun.castShadow = false; sun.visible = false;
  const csm = new CSM({ camera, parent: scene, cascades: 4, maxFar: 1800, mode: 'practical', shadowMapSize: 2048, lightDirection: new THREE.Vector3(0, -1, 0), lightIntensity: 5.2, lightNear: 1, lightFar: 6000, lightMargin: 400, shadowBias: -0.00025 });
  for (const l of csm.lights) { l.color.set(0xfff6ea); l.shadow.normalBias = 0.06; }
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
    const sm = { low: 1024, medium: 2048, high: 2048, ultra: 4096 }[q] ?? 2048;
    for (const l of csm.lights) { if (l.shadow.mapSize.x !== sm) { l.shadow.mapSize.set(sm, sm); if (l.shadow.map) { l.shadow.map.dispose(); l.shadow.map = null; } } }
    bloom.enabled = q !== 'low'; smaa.enabled = q !== 'low' && scale >= 0.7; flare.visible = q !== 'low';
    resize();
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
    // Earth: fixed az ~ 90 deg from sun, low elevation
    const az = Math.atan2(-dir.z, dir.x) + 1.9;
    earth.position.set(Math.cos(az) * 26000 * Math.cos(0.05), 26000 * Math.sin(0.05) + 300, -Math.sin(az) * 26000 * Math.cos(0.05));
    earth.lookAt(0, 0, 0);
    sun.intensity = dir.y > 0 ? 5.2 : 0;
    csm.lightDirection.copy(dir).negate();
    updateEnv(dir);
    for (const l of csm.lights) l.intensity = dir.y > 0 ? 5.2 : 0;
    bounce.position.set(-dir.x * 100, 30, -dir.z * 100); bounce.target.position.set(0, 0, 0);
    // lunar night: no sunlight or regolith bounce, only earthshine (Earth near full when the site is dark)
    const day = THREE.MathUtils.smoothstep(dir.y, -0.03, 0.03);
    fill.intensity = 0.16 * day + 0.42 * (1 - day); fill.color.setHex(day > 0.5 ? 0x9fb3d6 : 0x7f9cd8);
    bounce.intensity = 0.3 * day;
    renderer.toneMappingExposure = 1.05 + 0.9 * (1 - day);
    if (sky.userData.stars) sky.userData.stars.material.opacity = 0.55 + 0.4 * (1 - day);
  }
  function resize() {
    if (!(window.innerWidth > 0 && window.innerHeight > 0)) return; // minimised / hidden: keep last valid size
    renderer.setSize(window.innerWidth, window.innerHeight);
    composer.setSize(window.innerWidth, window.innerHeight);
    if (typeof smaa !== 'undefined') smaa.setSize(window.innerWidth * renderer.getPixelRatio(), window.innerHeight * renderer.getPixelRatio());
    if (typeof csm !== 'undefined') csm.updateFrustums();
    camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize);
    // self-heal: recompute cascade frustums if they were ever built from an invalid camera
  function csmGuard() {
    const l = csm.lights[0];
    if (!Number.isFinite(l.position.x) || !Number.isFinite(camera.aspect) || camera.aspect <= 0) {
      if (window.innerWidth > 0 && window.innerHeight > 0) { camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix(); csm.updateFrustums(); }
    }
  }
  return { renderer, scene, camera, sun, fill, bounce, sky, composer, bloom, setSun, earth, csm, csmRegister, setQuality, smaa, csmGuard, shadowSchedule, getQuality: () => ({ q: baseQ, scale: renderScale }) };
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
