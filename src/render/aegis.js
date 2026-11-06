// AEGIS-V1 renderer: the team's own procedural model (Hunch/nasa-llaso-cad/js, loaded unchanged by teamModels.js)
// scaled 1:K to full size and driven by the physics. The six wheels follow the rocker-bogie linkage
// (rocker angle + two bogie angles solved from the wheel heights), the tyres spin with the motors,
// the air struts show the ride height (kneel / drive / carry) and the ring jaws close on the module.
import * as THREE from 'three';
import { ROVER, K } from '../sim/config.js';
import { buildAegis, loadTeamModels } from './teamModels.js';

// sim bogie index (FL FR ML MR RL RR) for the model's wheel order (FL ML BL FR MR BR)
const MODEL_TO_SIM = [0, 2, 4, 1, 3, 5];

export class AegisModel {
  constructor(scene) {
    this.scene = scene;
    this.root = new THREE.Group(); this.root.name = 'AEGIS-V1'; this.root.scale.setScalar(K);
    scene.add(this.root);
    this.ready = false; this.lights = [];
  }

  async init() {
    const { group, root, rig } = await buildAegis();
    this.rig = rig; this.model = root;
    this.root.add(group);
    // the team model's lamps are authored for a 0.6 m rover; swap them for lights sized for the real one
    root.traverse((o) => {
      if (o.isLight) o.visible = false;
      if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; }
    });
    for (const sz of [-1, 1]) {
      const s = new THREE.SpotLight(0xfff3dc, 0, 140, 0.55, 0.5, 1.2);
      s.position.set(0.27, 0.23, sz * 0.12); s.target.position.set(3, 0.05, sz * 0.12);
      this.root.add(s); this.root.add(s.target); this.lights.push(s);
    }
    this.wheelGroups = rig.wheelGroups; this.tires = rig.tires;
    this.bogie = window.roverBogie; this.cradle = window.moduleCradle; this.heads = window.lidarHeads || [window.lidarHead].filter(Boolean);
    this.lastRide = null;
    this.ready = true;
    return this;
  }

  update(rover, time) {
    const p = rover.body.translation(), q = rover.body.rotation();
    this.root.position.set(p.x, p.y, p.z);
    this.root.quaternion.set(q.x, q.y, q.z, q.w);
    if (!this.ready) return;
    const RB = window.NASA_RockerBogie.GEO;
    // wheel height change in model metres, relative to the nominal stance (ride height removed)
    const d = rover.bogies.map((b) => (ROVER.bogie.Lnom + rover.rideOffset - b.L) / K);
    const side = (i0) => { // sim indices of a side in F, M, R order
      const [f, m, r] = i0; const dyF = d[f], dyM = d[m], dyR = d[r];
      return { rho: (dyF - (dyM + dyR) / 2) / 0.3225, beta: (dyM - dyR) / (RB.xM - RB.xR) };
    };
    const L = side([0, 2, 4]), Rr = side([1, 3, 5]);
    const cl = (v, m) => Math.max(-m, Math.min(m, v));
    const rho = cl((L.rho - Rr.rho) / 2, RB.rhoMax);
    this.bogie.pose(rho, cl(L.beta, RB.betaMax), cl(Rr.beta, RB.betaMax));
    const ride = rover.rideOffset / K;
    for (let i = 0; i < 6; i++) {
      const wg = this.wheelGroups[i], w = rover.wheels[MODEL_TO_SIM[i]];
      wg.group.position.y -= ride;           // body rises on the struts, wheels stay down
      wg.strut.setLength(ride);
      this.tires[i].rotation.y = -w.angle;
    }
    // ring jaws
    const Lf = rover.lift;
    this.cradle.apply({ lid: Math.min(1, Lf.clampT * 1.25), squeeze: Math.max(0, (Lf.clampT - 0.8) * 5), locked: Lf.clamp === 'closed' });
    for (const h of this.heads) h.rotation.y = time * 2 * Math.PI * 10;
    const on = rover.lights && rover.powered ? 900 : 0;
    for (const s of this.lights) s.intensity = on;
  }
}

// ---- cargo module + stands (team module_models.js at full scale) ----
export async function buildTeamCargo(c) {
  const T = await loadTeamModels();
  const g = new THREE.Group();
  const m = T.ModuleModels.buildModuleMesh(); m.scale.setScalar(K); g.add(m);
  g.userData.cad = null;
  return g;
}
export async function buildTeamStands(cargo, heightAt, terrain) {
  const T = await loadTeamModels();
  const out = new THREE.Group();
  for (const c of cargo) {
    const wrap = new THREE.Group(); wrap.scale.setScalar(K); out.add(wrap);
    const st = T.ModuleModels.buildStand(`${c.id}`);
    const yB = (c.initial.y - (c.T.adapterR || c.T.radius)) / K;
    T.ModuleModels.placeStand(st, c.initial.x / K, c.initial.z / K, yB, (mx, mz) => heightAt(terrain, mx * K, mz * K) / K);
    // yaw about the stand centre
    const pivot = new THREE.Group(); pivot.position.copy(st.position); st.position.set(0, 0, 0); pivot.rotation.y = c.yaw; pivot.add(st);
    wrap.add(pivot);
    c.standVis = pivot;
  }
  return out;
}
