// Loads the team's own procedural Three.js models (copied from Hunch/nasa-llaso-cad/js into
// public/team/js) unchanged. They are classic scripts that expect a global THREE (r128 era), so we
// expose this app's Three.js as window.THREE and load them in dependency order.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';

const ORDER = ['motion', 'rover_bom', 'rocker_bogie', 'module_carry', 'sensor_mast', 'lidar_mount', 'fusion_rover', 'module_models', 'dock_station', 'bay_design'];
let ready = null;

function loadScript(src) {
  return new Promise((res, rej) => { const s = document.createElement('script'); s.src = src; s.onload = res; s.onerror = () => rej(new Error('failed ' + src)); document.head.appendChild(s); });
}

export function loadTeamModels() {
  if (ready) return ready;
  window.THREE = Object.assign({}, THREE, { RoundedBoxGeometry });
  ready = ORDER.reduce((p, n) => p.then(() => loadScript(`/team/js/${n}.js`)), Promise.resolve()).then(() => ({
    FusionRover: window.NASA_FusionRover, ModuleModels: window.NASA_ModuleModels, Dock: window.NASA_Dock,
    ModuleCarry: window.NASA_ModuleCarry, RockerBogie: window.NASA_RockerBogie,
  }));
  return ready;
}

// AEGIS-V1 rover assembly (team design). Returns { group, rig } in the model's own metres (0.6 m class).
export async function buildAegis() {
  const T = await loadTeamModels();
  const group = new THREE.Group();
  const root = await T.FusionRover.init({ roverGroup: group, wheelGroups: [], wheels: [] });
  return { group, root, rig: root.userData.rig, GEOM: T.FusionRover.GEOM };
}
export async function buildTeamModule() { const T = await loadTeamModels(); return T.ModuleModels.buildModuleMesh(); }
export async function buildTeamStand(label) { const T = await loadTeamModels(); return T.ModuleModels.buildStand(label); }
