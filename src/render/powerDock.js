// AEGIS power dock visuals: 4 sun-tracking vertical solar masts, the battery vault (POWER_DOCK.bankWh), the underbody
// charge-coupler pad and the charger pedestal. Geometry mirrors the colliders in sim/powerDock.js.
import * as THREE from 'three';
import { POWER_DOCK } from '../sim/config.js';
import { heightAt } from '../sim/terrain.js';
import { MAT, labelTexture } from './props.js';

const box = (w, h, d, m) => { const me = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m); me.castShadow = me.receiveShadow = true; return me; };
const cyl = (r1, r2, h, m, seg = 24) => { const me = new THREE.Mesh(new THREE.CylinderGeometry(r1, r2, h, seg), m); me.castShadow = me.receiveShadow = true; return me; };

function cellTexture() {
  // monocrystalline cell grid: 6 x 10 cells per string, busbars and fingers, slight per-cell tint variation
  const W = 512, H = 1024, c = document.createElement('canvas'); c.width = W; c.height = H; const g = c.getContext('2d');
  g.fillStyle = '#c9ccd2'; g.fillRect(0, 0, W, H);
  const cx = 8, cy = 16, cw = W / cx, ch = H / cy;
  for (let y = 0; y < cy; y++) for (let x = 0; x < cx; x++) {
    const v = ((x * 13 + y * 7) % 9) - 4;
    g.fillStyle = `rgb(${14 + v},${24 + v},${58 + v * 2})`;
    g.fillRect(x * cw + 3, y * ch + 3, cw - 6, ch - 6);
    // clipped corners (pseudo-square mono wafers)
    g.fillStyle = '#c9ccd2'; for (const [a, b] of [[0, 0], [1, 0], [0, 1], [1, 1]]) { g.beginPath(); const px = x * cw + 3 + a * (cw - 6), py = y * ch + 3 + b * (ch - 6); g.moveTo(px, py); g.lineTo(px + (a ? -7 : 7), py); g.lineTo(px, py + (b ? -7 : 7)); g.fill(); }
    g.strokeStyle = 'rgba(210,215,225,0.55)'; g.lineWidth = 1.5;
    for (let k = 1; k < 3; k++) { g.beginPath(); g.moveTo(x * cw + k * cw / 3, y * ch + 4); g.lineTo(x * cw + k * cw / 3, y * ch + ch - 4); g.stroke(); }
    g.strokeStyle = 'rgba(190,195,210,0.18)'; g.lineWidth = 0.6;
    for (let k = 1; k < 12; k++) { g.beginPath(); g.moveTo(x * cw + 4, y * ch + k * ch / 12); g.lineTo(x * cw + cw - 4, y * ch + k * ch / 12); g.stroke(); }
  }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; return t;
}

function padTexture() {
  // sintered-regolith pad: hazard border, lane chevrons, coupler target cross
  const N = 1024, c = document.createElement('canvas'); c.width = c.height = N; const g = c.getContext('2d');
  g.fillStyle = '#5f5b54'; g.fillRect(0, 0, N, N);
  const id = g.getImageData(0, 0, N, N); for (let i = 0; i < id.data.length; i += 4) { const n = (Math.random() - 0.5) * 22; id.data[i] += n; id.data[i + 1] += n; id.data[i + 2] += n; } g.putImageData(id, 0, 0);
  g.strokeStyle = 'rgba(30,30,30,0.35)'; g.lineWidth = 3; for (let k = 1; k < 4; k++) { g.beginPath(); g.moveTo(k * N / 4, 0); g.lineTo(k * N / 4, N); g.stroke(); g.beginPath(); g.moveTo(0, k * N / 4); g.lineTo(N, k * N / 4); g.stroke(); }
  const B = 46; g.save(); g.beginPath(); g.rect(0, 0, N, N); g.rect(B, B, N - 2 * B, N - 2 * B); g.clip('evenodd');
  g.fillStyle = '#e2ad1f'; g.fillRect(0, 0, N, N); g.fillStyle = '#141414';
  for (let i = -N; i < 2 * N; i += 64) { g.beginPath(); g.moveTo(i, 0); g.lineTo(i + 32, 0); g.lineTo(i + 32 + N, N); g.lineTo(i + N, N); g.fill(); }
  g.restore();
  g.fillStyle = 'rgba(235,235,225,0.85)';
  for (let k = 0; k < 4; k++) { const y = 140 + k * 70; g.beginPath(); g.moveTo(N / 2 - 90, y + 40); g.lineTo(N / 2, y); g.lineTo(N / 2 + 90, y + 40); g.lineTo(N / 2 + 90, y + 58); g.lineTo(N / 2, y + 18); g.lineTo(N / 2 - 90, y + 58); g.fill(); }
  g.strokeStyle = 'rgba(235,235,225,0.9)'; g.lineWidth = 10; g.beginPath(); g.arc(N / 2, N / 2, 120, 0, Math.PI * 2); g.stroke();
  g.beginPath(); g.moveTo(N / 2 - 170, N / 2); g.lineTo(N / 2 + 170, N / 2); g.moveTo(N / 2, N / 2 - 170); g.lineTo(N / 2, N / 2 + 170); g.stroke();
  g.font = 'bold 54px system-ui, sans-serif'; g.fillStyle = 'rgba(235,235,225,0.9)'; g.textAlign = 'center'; g.fillText('POWER DOCK  PD-1', N / 2, N - 90);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; return t;
}

export class PowerDockModel {
  constructor(scene, sim) {
    this.sim = sim; const D = POWER_DOCK, L = sim.powerDock.layout, T = sim.terrain;
    const root = this.root = new THREE.Group(); scene.add(root);
    const yaw = L.hab.yaw;
    const cellMat = new THREE.MeshStandardMaterial({ map: cellTexture(), roughness: 0.18, metalness: 0.35, envMapIntensity: 1.4 });
    const backMat = new THREE.MeshStandardMaterial({ color: 0xe9e9e4, roughness: 0.55, metalness: 0.1 });
    const lattice = new THREE.MeshStandardMaterial({ color: 0xa4a8ae, roughness: 0.4, metalness: 0.8 });

    // ---- sun-tracking solar masts
    this.heads = [];
    for (const a of L.arrays) {
      const g0 = heightAt(T, a.x, a.z), m = new THREE.Group(); m.position.set(a.x, g0, a.z); root.add(m);
      const foot = cyl(1.6, 2.0, 0.6, MAT.dark, 8); foot.position.y = 0.3; m.add(foot);
      // triangular lattice mast
      for (let k = 0; k < 3; k++) { const ang = k * 2 * Math.PI / 3; const leg = cyl(0.09, 0.09, D.mastH, lattice, 6); leg.position.set(Math.cos(ang) * 0.42, D.mastH / 2, Math.sin(ang) * 0.42); m.add(leg); }
      for (let y = 1; y < D.mastH; y += 1.6) { const ring = new THREE.Mesh(new THREE.TorusGeometry(0.42, 0.035, 4, 3), lattice); ring.rotation.x = Math.PI / 2; ring.position.y = y; ring.castShadow = true; m.add(ring); }
      const head = new THREE.Group(); head.position.y = D.mastH - D.panelH / 2 - 0.6; m.add(head);
      const gimbal = cyl(0.5, 0.5, 0.8, MAT.steel, 16); gimbal.position.y = D.panelH / 2 + 0.3; head.add(gimbal);
      // panel wing: 3 x 5 hinged sub-panels on a white truss backing
      const wing = new THREE.Group(); wing.position.x = 0.75; head.add(wing);
      const pw = D.panelW / 3, ph = D.panelH / 5;
      for (let i = 0; i < 3; i++) for (let j = 0; j < 5; j++) {
        const p = new THREE.Mesh(new THREE.BoxGeometry(0.06, ph - 0.12, pw - 0.12), [cellMat, backMat, backMat, backMat, backMat, backMat]);
        p.position.set(0, -D.panelH / 2 + ph * (j + 0.5), -D.panelW / 2 + pw * (i + 0.5)); p.castShadow = p.receiveShadow = true; wing.add(p);
      }
      const spine = box(0.25, D.panelH + 0.4, 0.25, lattice); spine.position.x = -0.2; wing.add(spine);
      for (const s of [-1, 1]) { const rib = box(0.2, 0.2, D.panelW, lattice); rib.position.set(-0.2, s * D.panelH / 4, 0); wing.add(rib); }
      const beacon = new THREE.Mesh(new THREE.SphereGeometry(0.16, 10, 8), MAT.amberGlow.clone()); beacon.position.y = D.mastH + 0.3; m.add(beacon);
      this.heads.push(head);
    }

    // ---- battery vault: 6 insulated cell containers under a radiator roof, half-bermed in regolith
    const b = L.bank, gb = heightAt(T, b.x, b.z);
    const vault = new THREE.Group(); vault.position.set(b.x, gb, b.z); vault.rotation.y = yaw; root.add(vault);
    const shell = box(b.d, b.h, b.w, MAT.offwhite); shell.position.y = b.h / 2; vault.add(shell);
    for (let k = 0; k < 6; k++) { const pod = box(b.d * 0.42, b.h * 0.7, b.w / 6 - 0.35, MAT.foil); pod.position.set(b.d / 2 + 0.05, b.h * 0.42, -b.w / 2 + (k + 0.5) * b.w / 6); pod.scale.x = 0.02; vault.add(pod); }
    const roofMat = new THREE.MeshStandardMaterial({ color: 0xf4f4f2, roughness: 0.15, metalness: 0.2 });
    for (let k = 0; k < 14; k++) { const fin = box(b.d + 0.6, 0.05, 0.7, roofMat); fin.position.set(0, b.h + 0.5, -b.w / 2 + 0.5 + k * (b.w - 1) / 13); fin.rotation.x = 0.35; vault.add(fin); }
    for (const s of [-1, 1]) { const post = box(0.2, 0.6, 0.2, MAT.steel); post.position.set(s * (b.d / 2), b.h + 0.25, 0); vault.add(post); }
    const bermMat = new THREE.MeshStandardMaterial({ color: 0x77726a, roughness: 1 });
    const berm = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, b.w + 3, 4, 1), bermMat); berm.rotation.x = Math.PI / 2; berm.rotation.y = Math.PI / 4; berm.scale.set(1.6, 1, 1.2); berm.position.set(-b.d / 2 - 0.6, 0.2, 0); berm.receiveShadow = true; vault.add(berm);
    const sign = new THREE.Mesh(new THREE.PlaneGeometry(6.5, 1.0), new THREE.MeshBasicMaterial({ map: labelTexture(`BATTERY BANK  ${(D.bankWh / 1e6).toFixed(1)} MWh`, { color: '#141414' }), transparent: true }));
    sign.position.set(b.d / 2 + 0.03, b.h - 0.55, 0); sign.rotation.y = Math.PI / 2; vault.add(sign);
    // state-of-charge bar: 20 segments
    this.socSegs = [];
    const segOn = new THREE.MeshStandardMaterial({ color: 0x002a10, emissive: 0x33ff88, emissiveIntensity: 2.2 });
    const segLow = new THREE.MeshStandardMaterial({ color: 0x2a1500, emissive: 0xffa31a, emissiveIntensity: 2.2 });
    const segOff = new THREE.MeshStandardMaterial({ color: 0x1a1c1f, roughness: 0.6 });
    for (let k = 0; k < 20; k++) { const s = box(0.05, 0.28, 0.42, segOff); s.position.set(b.d / 2 + 0.03, 1.0, -b.w / 2 + 2 + k * (b.w - 4) / 19); s.castShadow = false; vault.add(s); this.socSegs.push(s); }
    this.segMats = { segOn, segLow, segOff };

    // ---- charge pad + rails
    const gp = sim.powerDock.ground, P = L.pad;
    const padG = new THREE.Group(); padG.position.set(P.x, gp, P.z); padG.rotation.y = L.yaw; root.add(padG);
    const pad = new THREE.Mesh(new THREE.BoxGeometry(17, 0.18, 17), [MAT.dark, MAT.dark, new THREE.MeshStandardMaterial({ map: padTexture(), roughness: 0.92 }), MAT.dark, MAT.dark, MAT.dark]);
    pad.rotation.y = -Math.PI / 2; pad.position.y = 0.06; pad.receiveShadow = true; padG.add(pad);
    this.railMat = new THREE.MeshStandardMaterial({ color: 0xb87333, roughness: 0.3, metalness: 1, emissive: 0x33ccff, emissiveIntensity: 0 });
    for (const s of [-1, 1]) { const rail = box(5.5, 0.12, 0.35, this.railMat); rail.position.set(0, 0.2, s * 0.9); padG.add(rail); }
    const cover = box(6.2, 0.06, 2.6, MAT.black); cover.position.y = 0.16; padG.add(cover);
    // ---- charger pedestal + cable to the vault
    const pe = L.pedestal, gpe = heightAt(T, pe.x, pe.z);
    const ped = new THREE.Group(); ped.position.set(pe.x, gpe, pe.z); ped.rotation.y = yaw; root.add(ped);
    const cab = box(1.0, 1.8, 1.0, MAT.white); cab.position.y = 0.9; ped.add(cab);
    const capT = box(1.15, 0.12, 1.15, MAT.dark); capT.position.y = 1.86; ped.add(capT);
    this.screenMat = new THREE.MeshStandardMaterial({ color: 0x050a10, emissive: 0x1b6dff, emissiveIntensity: 0.6 });
    const screen = box(0.04, 0.45, 0.6, this.screenMat); screen.position.set(0.52, 1.35, 0); ped.add(screen);
    this.ledMat = MAT.amberGlow.clone(); const led = box(0.04, 0.9, 0.08, this.ledMat); led.position.set(0.52, 0.7, 0.36); ped.add(led);
    const cablePts = [pe, P, b].map((p) => new THREE.Vector3(p.x, heightAt(T, p.x, p.z) + 0.08, p.z));
    const cableMat = new THREE.MeshStandardMaterial({ color: 0x262626, roughness: 0.8 });
    const tube = (pts) => { const m = new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 40, 0.1, 6, false), cableMat); m.receiveShadow = true; root.add(m); };
    tube([cablePts[0], cablePts[2]]);
    for (const a of L.arrays) { const q = new THREE.Vector3(a.x, heightAt(T, a.x, a.z) + 0.08, a.z); tube([q, cablePts[2]]); }
    // ---- floodlights on two poles: light the pad through the lunar night
    this.flood = [];
    for (const s of [-1, 1]) {
      const fp = { x: P.x + Math.cos(L.yaw + Math.PI / 2) * 10 * s, z: P.z - Math.sin(L.yaw + Math.PI / 2) * 10 * s };
      const gf = heightAt(T, fp.x, fp.z), pole = cyl(0.1, 0.16, 8, MAT.steel, 8); pole.position.set(fp.x, gf + 4, fp.z); root.add(pole);
      const lens = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.12, 0.5), MAT.whiteGlow); lens.position.set(fp.x, gf + 8, fp.z); root.add(lens);
      const sp = new THREE.SpotLight(0xfff3e0, 0, 45, 0.75, 0.6, 1.2); sp.position.set(fp.x, gf + 8, fp.z); sp.target.position.set(P.x, gp, P.z);
      root.add(sp, sp.target); this.flood.push(sp);
    }
    this._t = 0;
  }
  update(dt) {
    const D = this.sim.powerDock, sun = this.sim.sunDir;
    const az = Math.atan2(-sun.z, sun.x);
    for (const h of this.heads) { let d = az - h.rotation.y; d = Math.atan2(Math.sin(d), Math.cos(d)); h.rotation.y += d * Math.min(1, dt * 0.8); }
    const n = Math.round(D.soc * 20);
    this.socSegs.forEach((s, i) => { s.material = i < n ? (D.soc < 0.25 ? this.segMats.segLow : this.segMats.segOn) : this.segMats.segOff; });
    this._t += dt;
    const charging = D.chargeW > 0;
    this.railMat.emissiveIntensity = charging ? 1.2 + 0.6 * Math.sin(this._t * 4) : D.connected ? 0.4 : 0;
    this.ledMat.emissive.setHex(charging ? 0x33ff88 : D.connected ? 0x35d0ff : 0xffa31a);
    this.screenMat.emissiveIntensity = charging ? 1.4 : 0.6;
    const night = sun.y < 0.02;
    for (const f of this.flood) f.intensity = night || charging ? 900 : 0;
  }
}
