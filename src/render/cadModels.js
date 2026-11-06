// Loads the team's CAD (public/models, copied from the Hunch folder) and converts the OBJ/MTL
// Phong materials into physically based ones that suit the lunar lighting.
import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';

const cache = new Map();

function pbrFrom(m, name) {
  const c = m.color ? m.color.clone() : new THREE.Color(0.8, 0.8, 0.8);
  const shin = m.shininess ?? 30;
  const n = (name || m.name || '').toLowerCase();
  const metal = /steel|motor|rack|ring|bulkhead|locker|sprocket|brass|gantryrail/.test(n) ? 0.75 : /solar|cells/.test(n) ? 0.5 : 0.05;
  const mat = new THREE.MeshStandardMaterial({
    color: c, roughness: THREE.MathUtils.clamp(1 - shin / 110, 0.18, 0.95), metalness: metal,
    side: THREE.DoubleSide,
  });
  mat.name = m.name;
  return mat;
}

export function loadOBJ(base, name) {
  const key = base + name;
  if (cache.has(key)) return cache.get(key);
  const p = new Promise((resolve, reject) => {
    new MTLLoader().setPath(base).load(name + '.mtl', (mtl) => {
      mtl.preload();
      const loader = new OBJLoader().setMaterials(mtl).setPath(base);
      loader.load(name + '.obj', (obj) => {
        obj.traverse((o) => {
          if (!o.isMesh) return;
          o.geometry.deleteAttribute('normal');
          o.geometry = mergeClose(o.geometry);
          o.geometry.computeVertexNormals();
          o.material = Array.isArray(o.material) ? o.material.map((m) => pbrFrom(m)) : pbrFrom(o.material, o.name);
          o.castShadow = true; o.receiveShadow = true;
        });
        resolve(obj);
      }, undefined, reject);
    }, undefined, reject);
  });
  cache.set(key, p);
  return p;
}

// OBJ exports are triangle soups; weld coincident vertices so smooth normals work
function mergeClose(g) {
  const pos = g.attributes.position.array;
  const map = new Map(), out = [], idx = [];
  for (let i = 0; i < pos.length; i += 3) {
    const k = `${Math.round(pos[i] * 2000)},${Math.round(pos[i + 1] * 2000)},${Math.round(pos[i + 2] * 2000)}`;
    let j = map.get(k);
    if (j === undefined) { j = out.length / 3; out.push(pos[i], pos[i + 1], pos[i + 2]); map.set(k, j); }
    idx.push(j);
  }
  const ng = new THREE.BufferGeometry();
  ng.setAttribute('position', new THREE.Float32BufferAttribute(out, 3));
  ng.setIndex(idx);
  for (const gr of g.groups) ng.addGroup(gr.start, gr.count, gr.materialIndex);
  return ng;
}

// LLASO-P1 40-ft module: CAD interior inside a closed flight hull. The CAD hull has a viewing
// slot cut in its top; for the sim it is replaced by a full shell that can turn into an X-ray.
export function cargoModuleCAD(T) {
  const root = new THREE.Group();
  const shellMat = new THREE.MeshStandardMaterial({ color: 0xd8d9d4, roughness: 0.55, metalness: 0.08, side: THREE.DoubleSide, transparent: true, opacity: 1 });
  const shell = new THREE.Mesh(new THREE.CylinderGeometry(T.radius, T.radius, 12.19, 64, 1, true), shellMat);
  shell.rotation.z = Math.PI / 2; shell.castShadow = shell.receiveShadow = true;
  root.add(shell);
  const goldMat = new THREE.MeshStandardMaterial({ color: 0xc8921e, roughness: 0.3, metalness: 0.85, transparent: true, opacity: 1 });
  for (const x of [-4.6, -1.5, 1.5, 4.6]) {
    const band = new THREE.Mesh(new THREE.CylinderGeometry(T.radius + 0.015, T.radius + 0.015, 0.45, 64, 1, true), goldMat);
    band.rotation.z = Math.PI / 2; band.position.x = x; root.add(band);
  }
  const interior = new THREE.Group(); interior.rotation.x = -Math.PI / 2; root.add(interior); // CAD is Z-up
  root.userData = { shellMat, goldMat, interior, xray: false };
  loadOBJ('/models/', 'llaso_lunar_cargo_container').then((obj) => {
    const g = obj.clone(true);
    g.traverse((o) => { if (o.isMesh && /HullOuter/i.test(o.name)) o.visible = false; });
    interior.add(g);
  }).catch((e) => console.warn('container CAD not loaded', e));
  root.userData.setXray = (on) => {
    root.userData.xray = on;
    shellMat.opacity = on ? 0.13 : 1; shellMat.depthWrite = !on;
    goldMat.opacity = on ? 0.2 : 1; goldMat.depthWrite = !on;
  };
  return root;
}

// Team's 3D-printed tracked rover prototype (Z-up CAD, metres)
export function trackedRoverCAD(scale = 1) {
  const root = new THREE.Group();
  const inner = new THREE.Group(); inner.rotation.x = -Math.PI / 2; inner.scale.setScalar(scale); root.add(inner);
  root.userData.ready = loadOBJ('/models/', 'tracked_lunar_rover').then((obj) => {
    const g = obj.clone(true);
    g.traverse((o) => { if (o.isMesh && /YDLIDAR/i.test(o.name)) root.userData.lidar = o; });
    inner.add(g);
    return g;
  }).catch((e) => console.warn('rover CAD not loaded', e));
  return root;
}
