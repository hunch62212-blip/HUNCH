// Dust (ballistic, no drag in vacuum, 1.62 m/s^2), wheel tracks, LiDAR cloud, path overlays.
import * as THREE from 'three';
import { MOON, ROVER } from '../sim/config.js';
import { heightAt } from '../sim/terrain.js';

function dotTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 64; const g = c.getContext('2d');
  const grd = g.createRadialGradient(32, 32, 0, 32, 32, 32); grd.addColorStop(0, 'rgba(255,255,255,1)'); grd.addColorStop(0.5, 'rgba(255,255,255,0.5)'); grd.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grd; g.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
}

// In vacuum a grain keeps its brightness until it lands: no drag, no suspension, no fading. Flight time is 2 v_up / g
// (<= ~4.5 s at the fastest wheel spray, ~7 s for a 5.7 m/s throw); the cap only frees grains that never meet the ground
// (thrown off a ledge past the streamed terrain).
const DUST_MAX_FLIGHT_S = 12;
const DUST_ALPHA = 0.9;

export class Dust {
  constructor(scene, terrain) {
    this.t = terrain; this.N = 4000;
    this.p = new Float32Array(this.N * 3); this.v = new Float32Array(this.N * 3); this.life = new Float32Array(this.N); this.i = 0;
    this._emitQ = new THREE.Quaternion();
    this._emitDir = new THREE.Vector3();
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(this.p, 3));
    this.alpha = new Float32Array(this.N); g.setAttribute('alpha', new THREE.BufferAttribute(this.alpha, 1));
    const m = new THREE.ShaderMaterial({
      transparent: true, depthWrite: false,
      uniforms: { map: { value: dotTexture() }, color: { value: new THREE.Color(0.62, 0.6, 0.57) } },
      vertexShader: `attribute float alpha; varying float va; void main(){ va = alpha; vec4 mv = modelViewMatrix*vec4(position,1.0); gl_PointSize = 70.0/(-mv.z); gl_Position = projectionMatrix*mv; }`,
      fragmentShader: `uniform sampler2D map; uniform vec3 color; varying float va; void main(){ vec4 t = texture2D(map, gl_PointCoord); if (va<=0.0) discard; gl_FragColor = vec4(color, t.a*va*0.55); }`,
    });
    m.extensions = {};
    this.points = new THREE.Points(g, m); this.points.frustumCulled = false;
    scene.add(this.points);
  }
  emit(rover, dt) {
    for (const w of rover.wheels) {
      if (!w.contact) continue;
      const vs = Math.abs(w.omega) * ROVER.wheel.radius;
      if (vs < 0.08) continue;
      const rate = vs * (2 + 25 * Math.abs(w.slip)) * dt * 6;
      let n = Math.floor(rate) + (Math.random() < rate % 1 ? 1 : 0);
      // spray leaves the rear of the contact patch, opposite to wheel surface motion
      const b = rover.bogies[w.id];
      const q = rover.body.rotation(); this._emitQ.set(q.x, q.y, q.z, q.w);
      this._emitDir.set(Math.cos(b.steer), 0, -Math.sin(b.steer)).applyQuaternion(this._emitQ).multiplyScalar(-Math.sign(w.omega));
      const dir = this._emitDir;
      while (n-- > 0) {
        const k = this.i; this.i = (this.i + 1) % this.N;
        this.p[k * 3] = w.cp.x + (Math.random() - 0.5) * 0.4; this.p[k * 3 + 1] = w.cp.y + 0.03; this.p[k * 3 + 2] = w.cp.z + (Math.random() - 0.5) * 0.4;
        const sp = vs * (0.4 + Math.random() * 0.9);
        const up = vs * (0.3 + Math.random() * 0.9);
        this.v[k * 3] = dir.x * sp + (Math.random() - 0.5) * 0.3; this.v[k * 3 + 1] = up; this.v[k * 3 + 2] = dir.z * sp + (Math.random() - 0.5) * 0.3;
        this.life[k] = DUST_MAX_FLIGHT_S;
      }
    }
  }
  update(dt) {
    const g = MOON.g;
    for (let k = 0; k < this.N; k++) {
      if (this.life[k] <= 0) { this.alpha[k] = 0; continue; }
      this.life[k] -= dt; // seconds of flight left before the safety cap
      this.v[k * 3 + 1] -= g * dt;
      this.p[k * 3] += this.v[k * 3] * dt; this.p[k * 3 + 1] += this.v[k * 3 + 1] * dt; this.p[k * 3 + 2] += this.v[k * 3 + 2] * dt;
      if (this.v[k * 3 + 1] < 0 && this.life[k] > 0 && this.p[k * 3 + 1] < heightAt(this.t, this.p[k * 3], this.p[k * 3 + 2])) this.life[k] = 0; // no air: lands where the arc ends
      this.alpha[k] = this.life[k] > 0 ? DUST_ALPHA : 0;   // sharp until it lands (see DUST_MAX_FLIGHT_S)
    }
    this.points.geometry.attributes.position.needsUpdate = true;
    this.points.geometry.attributes.alpha.needsUpdate = true;
  }
}

export class Tracks {
  constructor(scene) {
    this.MAX = 60000; // quads
    this.pos = new Float32Array(this.MAX * 4 * 3); this.uv = new Float32Array(this.MAX * 4 * 2);
    const idx = new Uint32Array(this.MAX * 6);
    for (let i = 0; i < this.MAX; i++) { const v = i * 4; idx.set([v, v + 1, v + 2, v + 2, v + 1, v + 3], i * 6); }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3));
    g.setAttribute('uv', new THREE.BufferAttribute(this.uv, 2));
    // lit material needs normals: without them the shader normalises (0,0,0) -> NaN pixels, which bloom
    // smears into a screen-sized black blob
    this.nrm = new Float32Array(this.MAX * 4 * 3);
    for (let i = 1; i < this.nrm.length; i += 3) this.nrm[i] = 1;
    g.setAttribute('normal', new THREE.BufferAttribute(this.nrm, 3));
    this._e1 = new THREE.Vector3(); this._e2 = new THREE.Vector3();
    g.setIndex(new THREE.BufferAttribute(idx, 1));
    g.setDrawRange(0, 0);
    const c = document.createElement('canvas'); c.width = 64; c.height = 64; const x = c.getContext('2d');
    for (let i = 0; i < 64; i++) { const chev = (i % 16) < 7; x.fillStyle = chev ? 'rgba(20,20,20,0.55)' : 'rgba(20,20,20,0.32)'; x.fillRect(0, i, 64, 1); }
    x.clearRect(0, 0, 4, 64); x.clearRect(60, 0, 4, 64);
    const tex = new THREE.CanvasTexture(c); tex.wrapT = THREE.RepeatWrapping;
    const m = new THREE.MeshStandardMaterial({ color: 0x9a968f, map: tex, transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -4, roughness: 1 });
    this.mesh = new THREE.Mesh(g, m); this.mesh.frustumCulled = false; this.mesh.receiveShadow = true;
    scene.add(this.mesh);
    this.count = 0; this.head = 0; this.last = new Map();
    this._Q = new THREE.Quaternion();
    this._axle = new THREE.Vector3();
  }
  update(rover) {
    const q = rover.body.rotation(); this._Q.set(q.x, q.y, q.z, q.w); const Q = this._Q;
    for (const w of rover.wheels) {
      if (!w.contact) { this.last.delete(w.id); continue; }
      const prev = this.last.get(w.id);
      const cp = w.cp.clone();
      if (!prev) { this.last.set(w.id, { p: cp, v: 0 }); continue; }
      const d = cp.distanceTo(prev.p);
      if (d < 0.35) continue;
      if (d > 3) { this.last.set(w.id, { p: cp, v: 0 }); continue; }
      const b = rover.bogies[w.id];
      this._axle.set(Math.sin(b.steer), 0, Math.cos(b.steer)).applyQuaternion(Q).multiplyScalar(ROVER.wheel.width / 2);
      const axle = this._axle;
      const k = this.head; this.head = (this.head + 1) % this.MAX; this.count = Math.min(this.MAX, this.count + 1);
      const put = (i, v) => { this.pos[(k * 4 + i) * 3] = v.x; this.pos[(k * 4 + i) * 3 + 1] = v.y + 0.015; this.pos[(k * 4 + i) * 3 + 2] = v.z; };
      put(0, prev.p.clone().sub(axle)); put(1, prev.p.clone().add(axle)); put(2, cp.clone().sub(axle)); put(3, cp.clone().add(axle));
      // face normal of the quad (follows slopes); always pointing up, (0,1,0) if degenerate
      const P = this.pos, o = k * 12;
      const n = this._e1.set(P[o + 3] - P[o], P[o + 4] - P[o + 1], P[o + 5] - P[o + 2])
        .cross(this._e2.set(P[o + 6] - P[o], P[o + 7] - P[o + 1], P[o + 8] - P[o + 2]));
      if (n.y < 0) n.negate();
      if (!(n.lengthSq() > 1e-12)) n.set(0, 1, 0); else n.normalize();
      for (let i = 0; i < 4; i++) { this.nrm[o + i * 3] = n.x; this.nrm[o + i * 3 + 1] = n.y; this.nrm[o + i * 3 + 2] = n.z; }
      const v0 = prev.v, v1 = prev.v + d / 0.9;
      this.uv.set([0, v0, 1, v0, 0, v1, 1, v1], k * 8);
      this.last.set(w.id, { p: cp, v: v1 % 100 });
      this.mesh.geometry.attributes.position.needsUpdate = true; this.mesh.geometry.attributes.uv.needsUpdate = true; this.mesh.geometry.attributes.normal.needsUpdate = true;
      this.mesh.geometry.setDrawRange(0, this.count * 6);
    }
  }
  clear() { this.count = 0; this.head = 0; this.mesh.geometry.setDrawRange(0, 0); }
}

export class LidarCloud {
  constructor(scene) {
    this.MAX = 64 * 720;
    this.pos = new Float32Array(this.MAX * 3); this.col = new Float32Array(this.MAX * 3);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3));
    g.setAttribute('color', new THREE.BufferAttribute(this.col, 3));
    g.setDrawRange(0, 0);
    this.points = new THREE.Points(g, new THREE.PointsMaterial({ size: 2.2, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: 0.9, depthWrite: false }));
    this.points.frustumCulled = false; scene.add(this.points);
    this.visible = true;
  }
  set(scan, roverY) {
    const n = Math.min(scan.n, this.MAX);
    this.pos.set(scan.world.subarray(0, n * 3));
    for (let i = 0; i < n; i++) {
      const k = scan.kind[i];
      const h = scan.world[i * 3 + 1] - (roverY - 3);
      let r, g, b;
      if (k === 1) { r = 1; g = 0.35; b = 0.2; }            // boulder returns
      else if (k === 2) { r = 0.75; g = 0.4; b = 1; }       // structures
      else if (k === 3) { r = 1; g = 0.85; b = 0.2; }       // cargo
      else { const t = Math.max(0, Math.min(1, (h + 3) / 6)); r = 0.1 + 0.2 * t; g = 0.55 + 0.45 * t; b = 1 - 0.5 * t; }
      this.col[i * 3] = r; this.col[i * 3 + 1] = g; this.col[i * 3 + 2] = b;
    }
    const geo = this.points.geometry;
    geo.attributes.position.needsUpdate = true; geo.attributes.color.needsUpdate = true; geo.setDrawRange(0, n);
  }
}

export class PathOverlay {
  constructor(scene, terrain) {
    this.t = terrain; this.scene = scene;
    this.pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x35d0ff, transparent: true, opacity: 0.9, depthTest: true }));
    this.pathLine.frustumCulled = false; scene.add(this.pathLine);
    this.routeGroup = new THREE.Group(); scene.add(this.routeGroup);
    this.arcGroup = new THREE.Group(); scene.add(this.arcGroup);
    this.goal = new THREE.Mesh(new THREE.TorusGeometry(2.2, 0.12, 8, 48), new THREE.MeshBasicMaterial({ color: 0x35d0ff }));
    this.goal.rotation.x = Math.PI / 2; this.goal.visible = false; scene.add(this.goal);
    this.arcMatOk = new THREE.LineBasicMaterial({ color: 0x3dff8a, transparent: true, opacity: 0.35 });
    this.arcMatBad = new THREE.LineBasicMaterial({ color: 0xff4040, transparent: true, opacity: 0.35 });
  }
  setPath(pts) {
    if (!pts || pts.length < 2) { this.pathLine.visible = false; return; }
    const v = pts.map((p) => new THREE.Vector3(p.x, heightAt(this.t, p.x, p.z) + 0.35, p.z));
    this.pathLine.geometry.dispose(); this.pathLine.geometry = new THREE.BufferGeometry().setFromPoints(v); this.pathLine.visible = true;
  }
  setRoutes(routes) {
    for (const c of [...this.routeGroup.children]) { c.geometry.dispose(); this.routeGroup.remove(c); }
    for (const r of routes) {
      const v = r.pts.map((p) => new THREE.Vector3(p.x, heightAt(this.t, p.x, p.z) + 0.25, p.z));
      const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(v), new THREE.LineDashedMaterial({ color: 0xffd23d, dashSize: 2, gapSize: 1.5, transparent: true, opacity: 0.85 }));
      l.computeLineDistances(); l.frustumCulled = false; this.routeGroup.add(l);
    }
  }
  setArcs(lp, est, show) {
    for (const c of [...this.arcGroup.children]) { c.geometry.dispose(); this.arcGroup.remove(c); }
    if (!show || !lp || !lp.arcs) return;
    for (const a of lp.arcs) {
      if (a.pts.length < 2) continue;
      const v = a.pts.map((p) => new THREE.Vector3(p.x, heightAt(this.t, p.x, p.z) + 0.3, p.z));
      this.arcGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(v), a.ok ? this.arcMatOk : this.arcMatBad));
    }
  }
  setGoal(g) {
    if (!g) { this.goal.visible = false; return; }
    this.goal.visible = true; this.goal.position.set(g.x, heightAt(this.t, g.x, g.z) + 0.3, g.z);
  }
}

// Decorative regolith clasts (3-12 cm): far below the scale that matters to 1.4 m wheels, they are
// visual only; seeded per 4 m cell so they never "pop" when they stream in around the camera.
export class Pebbles {
  constructor(scene, terrain) {
    this.t = terrain;
    const g = new THREE.IcosahedronGeometry(0.5, 1);
    const p = g.attributes.position;
    for (let i = 0; i < p.count; i++) { const k = 0.78 + 0.32 * Math.abs(Math.sin(i * 12.9898) * 43758.5453 % 1); p.setXYZ(i, p.getX(i) * k, p.getY(i) * k * 0.7, p.getZ(i) * k); }
    g.computeVertexNormals();
    this.mesh = new THREE.InstancedMesh(g, new THREE.MeshStandardMaterial({ color: 0x8a8781, roughness: 0.95, flatShading: true }), 12000);
    this.mesh.castShadow = true; this.mesh.receiveShadow = true; this.mesh.frustumCulled = false;
    scene.add(this.mesh);
    this.center = null;
  }
  update(focus) {
    if (this.center && Math.hypot(focus.x - this.center.x, focus.z - this.center.z) < 8) return;
    this.center = { x: focus.x, z: focus.z };
    const C = 4, R = 46, M = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(), v = new THREE.Vector3(), e = new THREE.Euler();
    let n = 0;
    const cx0 = Math.floor((focus.x - R) / C), cx1 = Math.floor((focus.x + R) / C), cz0 = Math.floor((focus.z - R) / C), cz1 = Math.floor((focus.z + R) / C);
    for (let gx = cx0; gx <= cx1; gx++) for (let gz = cz0; gz <= cz1; gz++) {
      let seed = (gx * 73856093) ^ (gz * 19349663);
      const rnd = () => { seed = (seed * 1664525 + 1013904223) | 0; return ((seed >>> 0) % 100000) / 100000; };
      const cnt = 5 + Math.floor(rnd() * 9);
      for (let k = 0; k < cnt && n < 12000; k++) {
        const x = (gx + rnd()) * C, z = (gz + rnd()) * C;
        if (Math.hypot(x - focus.x, z - focus.z) > R) continue;
        const dp = Math.hypot(x - this.t.pad.roverStart.x, z - this.t.pad.roverStart.z);
        const d = 0.03 + Math.pow(rnd(), 3) * 0.1;
        const y = heightAt(this.t, x, z) + d * 0.12;
        q.setFromEuler(e.set(rnd() * 0.6, rnd() * 6.28, rnd() * 0.6));
        M.compose(v.set(x, y, z), q, s.set(d, d, d));
        this.mesh.setMatrixAt(n++, M);
        void dp;
      }
    }
    this.mesh.count = n;
    this.mesh.instanceMatrix.needsUpdate = true;
  }
}
