// Builds the Rapier physics world: terrain heightfield, boulders, lander, supply building,
// docking vestibules, cargo saddles and the cargo bodies themselves.
import * as THREE from 'three';
import { MOON, WORLD, CARGO_TYPES, DOCK, GROUPS, groups, REGOLITH as R, PHYS } from './config.js';
import { generateTerrain, toRapierHeights, heightAt, soilAt } from './terrain.js';
import { rockShapes, rockTransform } from './rockShapes.js';
import { acmGeom, makeFeet } from './cargoContact.js';
import { plateSinkage } from './terramechanics.js';

const ALL = 0xffff;
export const LANDER_SCALE = 2.6;
// Lander leg strut in unscaled model units (radius from the lander axis, height above the pad): the rendered
// leg (props.js) is a 4.4 m tube centred at r 5.4, y 1.75 aimed at (3.8, 3.6). Shared so physics == visuals.
export const LANDER_LEG = (() => {
  const cx = 5.4, cy = 1.75, half = 2.2, dx = 3.8 - cx, dy = 3.6 - cy, l = Math.hypot(dx, dy);
  return { r0: cx - dx / l * half, y0: cy - dy / l * half, r1: cx + dx / l * half, y1: cy + dy / l * half, radius: 0.17 };
})();
// Lander navigation tags (unscaled): on the descent-stage wall (r 4.6), 3.2 m up, 1.2 m square.
export const LANDER_TAG = { r: 4.62, y: 3.2, size: 1.2 };
// static geometry only needs to interact with moving things (avoids 6000+ static-static pairs)
const MOVERS = GROUPS.ROVER | GROUPS.CARGO | 32;
export const G_TERRAIN = groups(GROUPS.TERRAIN, MOVERS);
export const G_ROCK = groups(GROUPS.ROCK, MOVERS);
export const G_STRUCT = groups(GROUPS.STRUCT, MOVERS);
export const G_CARGO = groups(GROUPS.CARGO, ALL);
export const G_CARGO_HELD = groups(32, ALL & ~GROUPS.ROVER);   // held cargo does not touch its own carrier
// ACM (arm-handled cargo): its collider never meets TERRAIN (four custom regolith feet carry it, cargoContact.js);
// everything else (rocks, structures, rover, other cargo) still does. G_ACM_ARM: held by the arms, touches all but the rover.
export const G_ACM = groups(GROUPS.CARGO, ALL & ~GROUPS.TERRAIN);
export const G_ACM_ARM = groups(GROUPS.CARGO, ALL & ~GROUPS.ROVER);
export const G_ROVER = groups(GROUPS.ROVER, ALL & ~GROUPS.ROVER & ~32);
// query filters
export const Q_WHEEL = groups(GROUPS.ROVER, GROUPS.TERRAIN | GROUPS.ROCK | GROUPS.STRUCT | GROUPS.CARGO);
export const Q_SENSOR = groups(GROUPS.ROVER, GROUPS.TERRAIN | GROUPS.ROCK | GROUPS.STRUCT | GROUPS.CARGO);
export const Q_TERRAIN = groups(GROUPS.ROVER, GROUPS.TERRAIN | GROUPS.ROCK | GROUPS.STRUCT);

export function yawQuat(yaw) { return { x: 0, y: Math.sin(yaw / 2), z: 0, w: Math.cos(yaw / 2) }; }

export function buildWorld(RAPIER, seed = WORLD.seed) {
  const terrain = generateTerrain(seed);
  const world = new RAPIER.World({ x: 0, y: -MOON.g, z: 0 });
  // single source of the integrator step: PHYS.dt (config.js). Sim.stepOnce advances sim.time by PHYS.dt per world.step(),
  // so a mismatch would make the physics run at a different rate from the clock. Rapier stores it as float32.
  world.timestep = PHYS.dt;
  if (!(PHYS.dt > 0) || Math.abs(world.timestep - PHYS.dt) > 1e-6 * PHYS.dt) throw new Error(`world.timestep ${world.timestep} != PHYS.dt ${PHYS.dt}`);
  world.integrationParameters.numSolverIterations = 8;
  const eventQueue = new RAPIER.EventQueue(true);

  // All static geometry hangs off one fixed body: Rapier never pairs colliders of the same body.
  const fixed = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  // --- terrain heightfield
  const hf = RAPIER.ColliderDesc.heightfield(terrain.N, terrain.N, toRapierHeights(terrain), { x: WORLD.size, y: 1, z: WORLD.size })
    .setFriction(0.75).setCollisionGroups(G_TERRAIN);
  const terrainCollider = world.createCollider(hf, fixed);

  // --- boulders as convex hulls. Colliders are streamed: only rocks within ~140 m of the
  // rover (sensor range + margin) live in the physics world, because Rapier's broadphase cost
  // grows with every static collider. Geometry is precomputed so streaming is exact.
  const shapes = rockShapes();
  const q = new THREE.Quaternion(), qt = new THREE.Quaternion(), v = new THREE.Vector3();
  const rockGrid = new Map(); const CELL = 64;
  for (let i = 0; i < terrain.rocks.length; i++) {
    const r = terrain.rocks[i];
    // every rock, down to 25 cm, is a real collider (streamed in near the rover)
    const tr = rockTransform(r);
    const src = shapes[r.shape];
    const pts = new Float32Array(src.length);
    for (let k = 0; k < src.length; k += 3) { pts[k] = src[k] * tr.scale[0]; pts[k + 1] = src[k + 1] * tr.scale[1]; pts[k + 2] = src[k + 2] * tr.scale[2]; }
    q.setFromAxisAngle(v.set(0, 1, 0), tr.yaw); qt.setFromAxisAngle(v.set(1, 0, 0), tr.tilt); q.multiply(qt);
    r.pts = pts; r.pos = tr.pos; r.rot = { x: q.x, y: q.y, z: q.z, w: q.w }; r.top = tr.pos[1] + tr.scale[1] * 0.5;
    const key = Math.floor(r.x / CELL) + ',' + Math.floor(r.z / CELL);
    if (!rockGrid.has(key)) rockGrid.set(key, []);
    rockGrid.get(key).push(i);
  }
  const rockColliders = new Map(); // rock index -> collider, ENABLED (streamed-in) rocks only
  const colliderRock = new Map();  // collider handle -> rock index (every collider ever created)
  const rockCache = new Map();     // rock index -> collider, created once, then only enabled / disabled
  // colliders created / enabled since the last physics step: Sim.stepOnce re-marks them once after that step. A rock
  // added mid-run could otherwise miss its broadphase proxy (timing dependent): rays and wheel casts saw it but the
  // chassis never formed a contact pair and drove through it (tests/rock_climb.mjs on the 2.5 km map)
  const freshRocks = [];
  // Streaming enables / disables colliders; it never removes them. Removing far rocks and creating near ones in the
  // same call let Rapier hand a freed collider slot to a new rock before the next step processed the removal, and
  // the new rock never got a broadphase proxy: rays and wheel casts saw it, the chassis drove through it
  // (tests/rock_climb.mjs passed or failed depending on the step a boulder was injected; the M10 "block road"
  // boulders could be ghosts).
  function streamRocks(centers, radius = 140) {
    const want = new Set();
    for (const c of centers) {
      const cx0 = Math.floor((c.x - radius) / CELL), cx1 = Math.floor((c.x + radius) / CELL);
      const cz0 = Math.floor((c.z - radius) / CELL), cz1 = Math.floor((c.z + radius) / CELL);
      for (let gx = cx0; gx <= cx1; gx++) for (let gz = cz0; gz <= cz1; gz++) {
        const list = rockGrid.get(gx + ',' + gz); if (!list) continue;
        for (const i of list) { const r = terrain.rocks[i]; if (Math.hypot(r.x - c.x, r.z - c.z) < radius) want.add(i); }
      }
    }
    for (const [i, col] of rockColliders) if (!want.has(i)) { col.setEnabled(false); rockColliders.delete(i); }
    for (const i of want) {
      if (rockColliders.has(i)) continue;
      const cached = rockCache.get(i);
      if (cached) { cached.setEnabled(true); rockColliders.set(i, cached); freshRocks.push(cached); continue; }
      const r = terrain.rocks[i];
      if (r.skip) continue;
      const desc = RAPIER.ColliderDesc.convexHull(r.pts);
      if (!desc) { r.skip = true; continue; }
      desc.setTranslation(...r.pos).setRotation(r.rot).setFriction(0.8).setCollisionGroups(G_ROCK);
      try { const col = world.createCollider(desc, fixed); rockColliders.set(i, col); rockCache.set(i, col); colliderRock.set(col.handle, i); freshRocks.push(col); }
      catch (e) { r.skip = true; } // rare degenerate hull: dropped entirely so visuals == physics
    }
  }
  // validate every hull once up front so rendering never shows a rock physics can't hit
  { const probe = new RAPIER.World({ x: 0, y: 0, z: 0 });
    for (const r of terrain.rocks) { if (r.pebble) continue; try { const d = RAPIER.ColliderDesc.convexHull(r.pts); if (!d) { r.skip = true; continue; } probe.createCollider(d); } catch (e) { r.skip = true; } }
    probe.free(); }

  const addBox = (cx, cy, cz, hx, hy, hz, yaw, grp = G_STRUCT, body = fixed) => {
    const d = RAPIER.ColliderDesc.cuboid(hx, hy, hz).setTranslation(cx, cy, cz).setRotation(yawQuat(yaw)).setFriction(0.7).setCollisionGroups(grp);
    return world.createCollider(d, body);
  };
  const local = (ox, oz, ax, az, yaw) => { // rotate (ax,az) by yaw around origin ox,oz (THREE convention)
    const c = Math.cos(yaw), s = Math.sin(yaw);
    return { x: ox + ax * c + az * s, z: oz - ax * s + az * c };
  };

  // --- cargo lander at the pad (heavy cargo lander sized for a 7.96 m module, ~37 m tall)
  const P = WORLD.pad, pH = terrain.padH, LS = LANDER_SCALE;
  world.createCollider(RAPIER.ColliderDesc.cylinder(4.6 * LS, 3.4 * LS).setTranslation(P.x, pH + (2.6 + 4.6) * LS, P.z).setCollisionGroups(G_STRUCT), fixed);
  world.createCollider(RAPIER.ColliderDesc.cylinder(1.3 * LS, 4.6 * LS).setTranslation(P.x, pH + 2.6 * LS, P.z).setCollisionGroups(G_STRUCT), fixed);
  // Legs follow the rendered model (props.js buildLander, same LANDER_SCALE): a footpad plus one diagonal strut
  // from the pad up to the descent stage. The old horizontal strut boxes sat 3.8-4.6 m up at 10-18 m from the
  // axis where nothing was drawn: an invisible ramp the rover's wheels climbed until it rolled over.
  for (let k = 0; k < 4; k++) {
    const a = Math.PI / 4 + k * Math.PI / 2, ca = Math.cos(a), sa = Math.sin(a);
    addBox(P.x + ca * 6.6 * LS, pH + 0.12 * LS, P.z + sa * 6.6 * LS, 0.75 * LS, 0.12 * LS, 0.75 * LS, a);
    const r0 = LANDER_LEG.r0 * LS, y0 = LANDER_LEG.y0 * LS, r1 = LANDER_LEG.r1 * LS, y1 = LANDER_LEG.y1 * LS;
    const len = Math.hypot(r1 - r0, y1 - y0);
    const dir = new THREE.Vector3((r1 - r0) * ca, y1 - y0, (r1 - r0) * sa).normalize();
    const qa = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    world.createCollider(RAPIER.ColliderDesc.capsule(len / 2, LANDER_LEG.radius * LS)
      .setTranslation(P.x + (r0 + r1) / 2 * ca, pH + (y0 + y1) / 2, P.z + (r0 + r1) / 2 * sa)
      .setRotation({ x: qa.x, y: qa.y, z: qa.z, w: qa.w }).setFriction(0.3).setCollisionGroups(G_STRUCT), fixed);
  }
  // three descent-engine bells under the deck
  for (let k = 0; k < 3; k++) {
    world.createCollider(RAPIER.ColliderDesc.cylinder(0.7 * LS, 1.0 * LS).setTranslation(P.x + (k - 1) * 1.6 * LS, pH + 0.95 * LS, P.z).setCollisionGroups(G_STRUCT), fixed);
  }

  // --- supply building + docking vestibules + receiving saddles
  const H = terrain.hab, hH = terrain.habH;
  const b = H.building;
  addBox(b.x, hH + b.h / 2, b.z, b.w / 2, b.h / 2, b.d / 2, H.yaw);
  // radiation berm (regolith-filled bags) around three sides, modelled as low boxes
  const bermL = local(b.x, b.z, -b.w / 2 - 1.2, 0, H.yaw);
  addBox(bermL.x, hH + 1.6, bermL.z, 1.2, 1.6, b.d / 2 + 1.5, H.yaw);
  for (const sgn of [-1, 1]) {
    const bs = local(b.x, b.z, -1, sgn * (b.d / 2 + 1.2), H.yaw);
    addBox(bs.x, hH + 1.6, bs.z, b.w / 2 + 1.0, 1.6, 1.2, H.yaw);
  }
  // base structures beyond the supply building (dome, tunnel, modules, masts, arrays) are solid too
  const rot = (axis, ang) => { const q = new THREE.Quaternion().setFromAxisAngle(axis, ang); return { x: q.x, y: q.y, z: q.z, w: q.w }; };
  for (const it of terrain.base) {
    const y = heightAt(terrain, it.x, it.z);
    let d;
    if (it.kind === 'dome') d = RAPIER.ColliderDesc.ball(it.r).setTranslation(it.x, y, it.z);
    else if (it.kind === 'mast' || it.kind === 'light') d = RAPIER.ColliderDesc.cylinder(it.h / 2, it.r).setTranslation(it.x, y + it.h / 2, it.z);
    else if (it.kind === 'array') {
      world.createCollider(RAPIER.ColliderDesc.cylinder(7, 0.2).setTranslation(it.x, y + 7, it.z).setCollisionGroups(G_STRUCT), fixed);
      d = RAPIER.ColliderDesc.cuboid(it.w / 2, it.h / 2, 0.06).setTranslation(it.x, y + 9, it.z);
    } else {
      // horizontal cylinders: rotate Y axis to X, then yaw
      const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), it.yaw).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2));
      d = RAPIER.ColliderDesc.cylinder(it.len / 2, it.r).setTranslation(it.x, y + it.r + (it.kind === 'tunnel' ? 0.2 : 0.4), it.z).setRotation({ x: q.x, y: q.y, z: q.z, w: q.w });
    }
    world.createCollider(d.setCollisionGroups(G_STRUCT).setFriction(0.7), fixed);
  }
  void rot;
  const ports = [];
  for (const p of H.ports) {
    // berthing collar on the building face: only around the port (5.5-13.5 m), nothing low for the rover to hit
    const cx = (p.x + p.wallX) / 2, cz = (p.z + p.wallZ) / 2;
    const collarH = 8, collarY = hH + DOCK.portCenterH;
    const vest = addBox(cx, collarY, cz, DOCK.vestibuleLen / 2, collarH / 2, DOCK.vestibuleW / 2, H.yaw);
    ports.push({ ...p, y: hH + DOCK.portCenterH, ground: hH, collider: vest, occupied: null });
  }

  // --- cargo on the team's skid stands at the pad: four posts (outside the tyre track) hold the lug pins
  const cargo = [];
  for (const c of terrain.pad.cargo) {
    const T = CARGO_TYPES[c.type];
    const gy = heightAt(terrain, c.x, c.z);
    const ax = Math.cos(c.yaw), az = -Math.sin(c.yaw), px = -az, pz = ax; // axis + lateral (right) in world
    const half = T.adapterR || T.radius;
    const cy = gy + T.standY + half + 0.002;
    const stand = [];
    for (const lx of T.lugXs) for (const sz of [-1, 1]) {
      const x = c.x + ax * lx + px * sz * T.lugZ, z = c.z + az * lx + pz * sz * T.lugZ;
      const g0 = heightAt(terrain, x, z), top = cy - T.lugR;
      addBox(x, (g0 + top) / 2, z, 0.25, (top - g0) / 2, 0.25, c.yaw);
      stand.push({ x, z, g0, top });
    }
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(c.x, cy, c.z).setRotation(yawQuat(c.yaw))
      .setLinearDamping(0.02).setAngularDamping(0.05).setCcdEnabled(false));
    const ev = RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS;
    const mk = (d) => world.createCollider(d.setDensity(0).setFriction(0.6).setCollisionGroups(G_CARGO).setActiveEvents(ev).setContactForceEventThreshold(50), body);
    const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
    const qzR = { x: qz.x, y: qz.y, z: qz.z, w: qz.w };
    const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);
    const qxR = { x: qx.x, y: qx.y, z: qx.z, w: qx.w };
    let col;
    const extra = [];
    if (T.shape === 'cylinder') {
      col = mk(RAPIER.ColliderDesc.cylinder(T.length / 2, T.radius).setRotation(qzR));
    } else {
      col = mk(RAPIER.ColliderDesc.cuboid(T.length / 2, T.height / 2, T.width / 2));
      // ring-adapter sled: two ring bands the saddles and jaws grip, plus a continuous lower cylindrical keel shroud
      // connecting the aft sled (-4.35 m) through both rings up to +2.10 m (staying shy of the +3.029 m docking face)
      // so the under-module CASS laser rangefinders lock onto the exact adapterR cylinder contour well before contact.
      for (const rx of T.adapterRings) extra.push(mk(RAPIER.ColliderDesc.cylinder(0.41, T.adapterR).setTranslation(rx, 0, 0).setRotation(qzR)));
      const xAft = (T.adapterX || 0) - (T.adapterLen || T.length) / 2;
      const xFwd = Math.min(T.length / 2 - 0.9, (T.adapterRings?.[1] ?? 1.24) + 0.86);
      const keelPts = [];
      for (const kx of [xAft, xFwd]) {
        for (let a = -28; a <= 28; a += 4) {
          const th = a * Math.PI / 180;
          keelPts.push(kx, -T.adapterR * Math.cos(th), T.adapterR * Math.sin(th));
        }
        keelPts.push(kx, -T.height / 2, -1.1, kx, -T.height / 2, 1.1);
      }
      const kd = RAPIER.ColliderDesc.convexHull(new Float32Array(keelPts));
      if (kd) extra.push(mk(kd));
    }
    for (const lx of T.lugXs) for (const sz of [-1, 1]) {   // trunnion lug pins
      const inner = T.shape === 'cylinder' ? T.radius - 0.1 : T.adapterR - 0.1, len = T.lugZ + 0.25 - inner;
      extra.push(mk(RAPIER.ColliderDesc.cylinder(len / 2, T.lugR).setTranslation(lx, 0, sz * (inner + len / 2)).setRotation(qxR)));
    }
    const r2 = half * half;
    const inertia = { x: 0.8 * T.mass * r2, y: T.mass * (3 * r2 + T.length ** 2) / 12, z: T.mass * (3 * r2 + T.length ** 2) / 12 };
    const comX = T.adapterX || 0;
    body.setAdditionalMassProperties(T.mass, { x: comX * 0.3, y: 0, z: 0 }, inertia, { x: 0, y: 0, z: 0, w: 1 }, true);
    cargo.push({ ...c, T, body, collider: col, colliders: [col, ...extra], stand, groundY: gy, state: 'staged', dest: c.dest ?? (c.type === 'cyl40' ? 1 : 2), deliveredAt: null, initial: { x: c.x, y: cy, z: c.z, yaw: c.yaw } });
  }
  // receiving cradle jacks at each port are created when they extend (see Sim.extendJacks)
  for (const port of ports) {
    port.cargoYaw = Math.atan2(port.nz, -port.nx); // cargo +X points into the port (= -n)
    port.jackK = 0; port.jacks = null; port.hatchOpen = false;
  }

  function addRock(r) {
    const tr = rockTransform(r);
    const src = shapes[r.shape];
    const pts = new Float32Array(src.length);
    for (let k = 0; k < src.length; k += 3) { pts[k] = src[k] * tr.scale[0]; pts[k + 1] = src[k + 1] * tr.scale[1]; pts[k + 2] = src[k + 2] * tr.scale[2]; }
    q.setFromAxisAngle(v.set(0, 1, 0), tr.yaw); qt.setFromAxisAngle(v.set(1, 0, 0), tr.tilt); q.multiply(qt);
    r.pts = pts; r.pos = tr.pos; r.rot = { x: q.x, y: q.y, z: q.z, w: q.w }; r.top = tr.pos[1] + tr.scale[1] * 0.5;
    terrain.rocks.push(r);
    const i = terrain.rocks.length - 1;
    const key = Math.floor(r.x / CELL) + ',' + Math.floor(r.z / CELL);
    if (!rockGrid.has(key)) rockGrid.set(key, []);
    rockGrid.get(key).push(i);
    return i;
  }

  // --- ACM: arm-handled cargo module, dynamic body sitting on regolith (plan s5.5, s6.3). Not placed at build time:
  // call spawnACM(x, z, yaw, opts) {mass, id, tag0, settled=true, drop=0 (m above the surface)}. The returned cargo
  // entry is also pushed on `cargo` (type 'acm'); the caller registers colliderOwner and calls acmSoilStep each step.
  let acmCount = 0;
  function spawnACM(x, z, yaw = 0, opts = {}) {
    const T0 = CARGO_TYPES.acm, mass = opts.mass ?? T0.mass, T = mass === T0.mass ? T0 : { ...T0, mass };
    const G = acmGeom(T), feet = makeFeet(T);
    const cs = Math.cos(yaw), sn = Math.sin(yaw);
    // yaw about +Y (THREE): world = (x cos + z sin, -x sin + z cos)
    const wp = (lx, lz) => ({ x: x + lx * cs + lz * sn, z: z - lx * sn + lz * cs });
    const b = G.foot, A = b * b, Nf = mass * MOON.g / feet.length, p = Nf / A;
    let sum = 0, zEq = 0;
    for (const f of feet) { const q = wp(f.x, f.z), s = {}; soilAt(terrain, q.x, q.z, s);
      const z = opts.settled === false ? 0 : plateSinkage(p, b, { n: R.n, kc: R.kc * s.k, kphi: R.kphi * s.k });
      f.zp = z; zEq += z; sum += heightAt(terrain, q.x, q.z) - z; }
    const gy = sum / feet.length, y = gy + (opts.drop ?? 0) - G.groundY;
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z).setRotation(yawQuat(yaw))
      .setLinearDamping(0.02).setAngularDamping(0.05).setCanSleep(false).setCcdEnabled(false));
    const ev = RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS;
    const mk = (d) => world.createCollider(d.setDensity(0).setFriction(0.6).setCollisionGroups(G_ACM).setActiveEvents(ev).setContactForceEventThreshold(50), body);
    const col = mk(RAPIER.ColliderDesc.cuboid(G.L / 2, G.H / 2, G.W / 2).setTranslation(0, G.boxY, 0));
    const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);
    const extra = [];
    const tr = T.trunnion;
    for (const sz of [-1, 1]) extra.push(mk(RAPIER.ColliderDesc.cylinder(tr.len / 2, tr.r).setTranslation(0, G.trunY, sz * (G.W / 2 + tr.len / 2)).setRotation({ x: qx.x, y: qx.y, z: qx.z, w: qx.w })));
    const m = mass, inertia = { x: m * (G.H ** 2 + G.W ** 2) / 12, y: m * (G.L ** 2 + G.W ** 2) / 12, z: m * (G.L ** 2 + G.H ** 2) / 12 };
    body.setAdditionalMassProperties(m, { x: 0, y: 0, z: 0 }, inertia, { x: 0, y: 0, z: 0, w: 1 }, true);
    const id = opts.id ?? `ACM${++acmCount}`;
    const c = { id, type: 'acm', T, x, z, yaw, tag0: opts.tag0, body, collider: col, colliders: [col, ...extra], stand: [], geom: G, feet,
      groundY: gy, state: 'staged', mode: 'GROUND', dest: opts.dest ?? 2, deliveredAt: null, initial: { x, y, z, yaw }, soilN: 0 };
    cargo.push(c);
    return c;
  }

  world.step(eventQueue);
  return { RAPIER, world, eventQueue, terrain, terrainCollider, rockColliders, colliderRock, streamRocks, freshRocks, addRock, fixed, ports, cargo, spawnACM };
}
