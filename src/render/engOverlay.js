// Engineering overlay: the physics the vehicle model computes, drawn into the 3D scene the way engineering
// debug views and games show it (docs/GRAPHICS_PHYSICS_RESEARCH.md):
//  * centre of mass with a plumb line to the ground, and the support polygon coloured by the force-angle tip
//    margin; the edge the vehicle would tip about is drawn red
//  * predicted path (constant speed and yaw rate, 8 s) and a braking-distance bar across it (AR-HUD style)
// Everything is drawn without depth test so it reads through the vehicle body, like a CAD x-ray view.
import * as THREE from 'three';
import { heightAt } from '../sim/terrain.js';
import { vehicleDynamics, predictPath } from '../sim/dynamics.js';
import { ROVER } from '../sim/config.js';

const GREEN = new THREE.Color(0x3ddc84), AMBER = new THREE.Color(0xffb020), RED = new THREE.Color(0xff4d3a);
const ramp = (x, a, b) => Math.max(0, Math.min(1, (x - a) / (b - a)));
// green -> amber -> red as t goes 0 -> 1
const traffic = (t, out) => (t < 0.5 ? out.copy(GREEN).lerp(AMBER, t * 2) : out.copy(AMBER).lerp(RED, (t - 0.5) * 2));

function xray(mat) { mat.depthTest = false; mat.depthWrite = false; mat.transparent = true; return mat; }

export class EngineeringOverlay {
  constructor(scene, terrain) {
    this.terrain = terrain;
    this.group = new THREE.Group(); this.group.name = 'engineering-overlay'; this.group.renderOrder = 999;
    this.group.visible = false;
    scene.add(this.group);
    const add = (o) => { o.renderOrder = 999; o.frustumCulled = false; this.group.add(o); return o; };

    // (v1.3.0: the per-wheel load / traction arrows were removed at the operator's request; wheel loads and slip are
    // still in the vehicle-state table and the dynamics instruments)
    // centre of mass: checkered-style marker (sphere), plumb line, ground cross
    this.com = add(new THREE.Mesh(new THREE.SphereGeometry(0.32, 16, 12), xray(new THREE.MeshBasicMaterial({ color: 0xffd23d }))));
    this.plumbGeo = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(6), 3));
    this.plumb = add(new THREE.LineSegments(this.plumbGeo, xray(new THREE.LineDashedMaterial({ color: 0xffd23d, dashSize: 0.3, gapSize: 0.2 }))));
    this.comFoot = add(new THREE.Mesh(new THREE.RingGeometry(0.25, 0.42, 20).rotateX(-Math.PI / 2), xray(new THREE.MeshBasicMaterial({ color: 0xffd23d, side: THREE.DoubleSide }))));

    // support polygon (up to 12 vertices) + critical tip edge
    this.polyGeo = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(13 * 3), 3));
    this.polyMat = xray(new THREE.LineBasicMaterial({ color: 0x3ddc84, opacity: 0.95 }));
    this.poly = add(new THREE.Line(this.polyGeo, this.polyMat));
    this.critGeo = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(6), 3));
    this.crit = add(new THREE.Line(this.critGeo, xray(new THREE.LineBasicMaterial({ color: 0xff4d3a }))));
    // translucent fill so the polygon reads as an area
    this.fillGeo = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(12 * 9), 3));
    this.fillMat = new THREE.MeshBasicMaterial({ color: 0x3ddc84, transparent: true, opacity: 0.22, depthWrite: false, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: -2 }); // depth-tested: shades the ground, not the body
    this.fill = add(new THREE.Mesh(this.fillGeo, this.fillMat));

    // predicted path ribbon (two edge lines + centre) and braking bar
    this.N = 40;
    this.pathGeo = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array((this.N + 1) * 3), 3));
    this.path = add(new THREE.Line(this.pathGeo, xray(new THREE.LineDashedMaterial({ color: 0x35d0ff, dashSize: 0.8, gapSize: 0.5, opacity: 0.9 }))));
    this.edgeGeo = [0, 1].map(() => new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array((this.N + 1) * 3), 3)));
    this.edges = this.edgeGeo.map((g) => add(new THREE.Line(g, xray(new THREE.LineBasicMaterial({ color: 0x35d0ff, opacity: 0.35 })))));
    this.stopBar = add(new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.12, 1), xray(new THREE.MeshBasicMaterial({ color: 0xffb020, opacity: 0.85 }))));
    this.stopLabel = add(this._label());
    this._labelText = '';
    this._c = new THREE.Color();
    this.dyn = null;
  }

  _label() {
    const cv = document.createElement('canvas'); cv.width = 256; cv.height = 64;
    const tex = new THREE.CanvasTexture(cv); tex.colorSpace = THREE.SRGBColorSpace;
    const sp = new THREE.Sprite(xray(new THREE.SpriteMaterial({ map: tex })));
    sp.scale.set(4, 1, 1); sp.userData = { cv, tex };
    return sp;
  }
  _setLabel(text, color) {
    if (text === this._labelText) return;
    this._labelText = text;
    const { cv, tex } = this.stopLabel.userData, g = cv.getContext('2d');
    g.clearRect(0, 0, cv.width, cv.height);
    g.fillStyle = 'rgba(10,13,18,0.75)'; g.beginPath(); g.roundRect(2, 8, 252, 48, 10); g.fill();
    g.font = '600 26px "SF Mono", Menlo, monospace'; g.fillStyle = color; g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(text, 128, 33);
    tex.needsUpdate = true;
  }

  update(R, show) {
    this.group.visible = show;
    if (!show) return;
    const d = this.dyn = vehicleDynamics(R);
    const c = this._c;
    const p = R.pose();
    // ---- centre of mass + plumb line
    const cw = R.comWorld;
    if (cw) {
      this.com.visible = this.plumb.visible = this.comFoot.visible = true;
      this.com.position.copy(cw);
      const gy = heightAt(this.terrain, cw.x, cw.z);
      const a = this.plumbGeo.attributes.position.array; a[0] = cw.x; a[1] = cw.y; a[2] = cw.z; a[3] = cw.x; a[4] = gy + 0.05; a[5] = cw.z;
      this.plumbGeo.attributes.position.needsUpdate = true; this.plumb.computeLineDistances();
      this.comFoot.position.set(cw.x, gy + 0.06, cw.z);
    } else this.com.visible = this.plumb.visible = this.comFoot.visible = false;
    // ---- support polygon coloured by force-angle margin (green > 25 deg, red < 8 deg)
    const poly = R.tilt?.poly;
    if (poly && poly.length >= 3 && poly.length <= 12) {
      this.poly.visible = this.fill.visible = true;
      traffic(1 - ramp(d.marginDeg, 8, 25), c);
      this.polyMat.color.copy(c); this.fillMat.color.copy(c);
      const a = this.polyGeo.attributes.position.array;
      for (let i = 0; i <= poly.length; i++) { const q = poly[i % poly.length]; a[i * 3] = q.x; a[i * 3 + 1] = q.y + 0.08; a[i * 3 + 2] = q.z; }
      this.polyGeo.setDrawRange(0, poly.length + 1); this.polyGeo.attributes.position.needsUpdate = true;
      // fan triangulation (convex hull) for the fill
      const f = this.fillGeo.attributes.position.array; let k = 0;
      for (let i = 1; i < poly.length - 1; i++) for (const q of [poly[0], poly[i], poly[i + 1]]) { f[k++] = q.x; f[k++] = q.y + 0.06; f[k++] = q.z; }
      this.fillGeo.setDrawRange(0, (poly.length - 2) * 3); this.fillGeo.attributes.position.needsUpdate = true;
      const e0 = poly[R.tilt.crit % poly.length], e1 = poly[(R.tilt.crit + 1) % poly.length], ce = this.critGeo.attributes.position.array;
      ce[0] = e0.x; ce[1] = e0.y + 0.14; ce[2] = e0.z; ce[3] = e1.x; ce[4] = e1.y + 0.14; ce[5] = e1.z;
      this.critGeo.attributes.position.needsUpdate = true; this.crit.visible = true;
    } else this.poly.visible = this.fill.visible = this.crit.visible = false;
    // ---- predicted path + braking bar
    const moving = Math.abs(d.v) > 0.05;
    this.path.visible = this.edges[0].visible = this.edges[1].visible = this.stopBar.visible = this.stopLabel.visible = moving;
    if (!moving) return;
    const horizon = Math.max(4, Math.min(12, (d.stopDist + 15) / Math.max(0.3, Math.abs(d.v))));
    // the path and the braking distance start at the bumper on the side the rover is moving toward
    const bump = ROVER.frame.halfLen * Math.sign(d.v);
    const start = { x: p.x + Math.cos(p.yaw) * bump, z: p.z - Math.sin(p.yaw) * bump, yaw: p.yaw };
    const pts = predictPath(start, d.v, d.yawRate, horizon, this.N);
    const halfW = Number.isFinite(d.halfT) ? d.halfT : 4.4;
    const ca = this.pathGeo.attributes.position.array, la = this.edgeGeo[0].attributes.position.array, ra = this.edgeGeo[1].attributes.position.array;
    let stopPt = pts[pts.length - 1];
    for (let i = 0; i < pts.length; i++) {
      const q = pts[i], cy = Math.cos(q.yaw), sy = Math.sin(q.yaw), rx = sy, rz = cy; // body right = (sin, cos)
      ca[i * 3] = q.x; ca[i * 3 + 1] = heightAt(this.terrain, q.x, q.z) + 0.15; ca[i * 3 + 2] = q.z;
      for (const [arr, s] of [[la, -1], [ra, 1]]) {
        const x = q.x + rx * halfW * s, z = q.z + rz * halfW * s;
        arr[i * 3] = x; arr[i * 3 + 1] = heightAt(this.terrain, x, z) + 0.12; arr[i * 3 + 2] = z;
      }
      if (q.s <= d.stopDist) stopPt = q;
    }
    this.pathGeo.attributes.position.needsUpdate = true; this.path.computeLineDistances();
    for (const g of this.edgeGeo) g.attributes.position.needsUpdate = true;
    // braking bar at the stopping point, across the track
    const sy0 = heightAt(this.terrain, stopPt.x, stopPt.z) + 0.2;
    this.stopBar.position.set(stopPt.x, sy0, stopPt.z);
    this.stopBar.rotation.set(0, stopPt.yaw, 0);
    this.stopBar.scale.set(1, 1, halfW * 2);
    this.stopLabel.position.set(stopPt.x, sy0 + 1.4, stopPt.z);
    this._setLabel(`STOP ${d.stopDist.toFixed(1)} m`, d.stopDist > 25 ? '#ffb020' : '#dfe6ee');
  }
}
