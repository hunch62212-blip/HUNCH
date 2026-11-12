// Draw-call reduction for static props. A prop built from dozens of small meshes (a stand, the lander, the supply
// building) costs one draw call per mesh in the main pass and again in each of the four shadow cascades. mergeStatic
// bakes every mesh under `root` that shares a material into one mesh per material. The original meshes stay in the
// tree (their transforms, children and userData are untouched) and are only switched to a material that is not drawn,
// so code that holds a reference to one of them keeps working, but must opt it out with `skip` if it animates it.
import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

const HIDDEN = new THREE.MeshBasicMaterial({ visible: false });
HIDDEN.name = 'merged-away';

// skip(mesh) -> true keeps a mesh as it is (anything that moves, is shown / hidden, or has its material swapped later).
// Meshes under a node that `skip` rejects are kept too. Transparent materials are left alone (their draw order matters)
// unless opts.transparent is set, for parts that fade together as one rigid piece. Returns { before, after } mesh counts.
// ?nomerge in the URL leaves every part as its own mesh (for comparing draw calls or debugging a prop).
const OFF = typeof location !== 'undefined' && /[?&]nomerge\b/.test(location.search);

export function mergeStatic(root, skip = () => false, opts = {}) {
  if (OFF) return { before: 0, after: 0 };
  root.updateMatrixWorld(true);
  const inv = new THREE.Matrix4().copy(root.matrixWorld).invert();
  const groups = new Map();
  let before = 0;
  const walk = (o, hidden) => {
    if (o !== root && (skip(o) || o.userData.noMerge)) { o.traverse((c) => { if (c.isMesh) before++; }); return; }
    const vis = hidden && o.visible;
    if (o.isMesh) {
      before++;
      const m = o.material;
      const ok = vis && !o.isInstancedMesh && !o.isSkinnedMesh && !Array.isArray(m) && m && m.visible && (!m.transparent || opts.transparent) && !m.vertexColors
        && !o.userData.xrayable && o.geometry && o.geometry.attributes.position && !o.geometry.morphAttributes.position && !o.customDepthMaterial;
      if (ok) {
        const key = `${m.uuid}|${o.castShadow ? 1 : 0}${o.receiveShadow ? 1 : 0}`;
        if (!groups.has(key)) groups.set(key, { m, cast: o.castShadow, recv: o.receiveShadow, list: [] });
        groups.get(key).list.push(o);
      }
    }
    for (const c of o.children) walk(c, vis);
  };
  walk(root, true);
  let after = before;
  for (const { m, cast, recv, list } of groups.values()) {
    if (list.length < 2) continue;
    const needUv = !!(m.map || m.normalMap || m.roughnessMap || m.metalnessMap || m.emissiveMap || m.alphaMap || m.bumpMap || m.aoMap);
    if (needUv && !list.every((o) => o.geometry.attributes.uv)) continue;
    const geos = list.map((o) => {
      let g = o.geometry.index ? o.geometry.toNonIndexed() : o.geometry.clone();
      if (!g.attributes.normal) g.computeVertexNormals();
      for (const name of Object.keys(g.attributes)) if (name !== 'position' && name !== 'normal' && !(name === 'uv' && needUv)) g.deleteAttribute(name);
      g.applyMatrix4(new THREE.Matrix4().multiplyMatrices(inv, o.matrixWorld));
      if (o.matrixWorld.determinant() < 0) { // a mirrored part: flip the winding back so its faces stay front-facing
        const p = g.attributes.position.array, n = g.attributes.normal.array, u = g.attributes.uv?.array;
        for (let i = 0; i < p.length; i += 9) { for (let k = 0; k < 3; k++) { let t = p[i + 3 + k]; p[i + 3 + k] = p[i + 6 + k]; p[i + 6 + k] = t; t = n[i + 3 + k]; n[i + 3 + k] = n[i + 6 + k]; n[i + 6 + k] = t; } }
        if (u) for (let i = 0; i < u.length; i += 6) for (let k = 0; k < 2; k++) { const t = u[i + 2 + k]; u[i + 2 + k] = u[i + 4 + k]; u[i + 4 + k] = t; }
      }
      return g;
    });
    const merged = mergeGeometries(geos, false);
    for (const g of geos) g.dispose();
    if (!merged) continue;
    const mesh = new THREE.Mesh(merged, m); mesh.castShadow = cast; mesh.receiveShadow = recv; mesh.name = 'merged'; mesh.userData.noMerge = true;
    root.add(mesh);
    for (const o of list) o.material = HIDDEN;
    after -= list.length - 1;
  }
  return { before, after };
}
