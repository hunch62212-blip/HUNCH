// Operator link model, the half that commands.js did not have (lunar audit B4):
//  * DelayLine: a fixed-size ring buffer of time-stamped samples. The HUD pushes telemetry into it and reads it back
//    sim.comms.delayS late, so an Earth operator sees the rover where it was one light time ago (the downlink half of
//    the round trip; commands.js / main.js delay the uplink half).
//  * LosMonitor: once a second it casts a ray from the rover's antenna mast to the habitat relay antenna and one toward
//    Earth (fixed azimuth, ~10 deg up at the south pole) and raises COMMS LOS by itself when the link the current mode
//    needs is blocked by terrain or structures (local crew: the habitat; Earth: direct to Earth or via the habitat).
import { WORLD } from './config.js';
import { COMMS } from './lunarParams.js';
import { heightAt } from './terrain.js';
import { Q_SENSOR } from './world.js';

export class DelayLine {
  constructor(cap = 512) { this.cap = cap; this.t = new Float64Array(cap); this.v = new Array(cap); this.n = 0; this.head = 0; }
  push(t, v) { this.t[this.head] = t; this.v[this.head] = v; this.head = (this.head + 1) % this.cap; if (this.n < this.cap) this.n++; }
  clear() { this.n = 0; this.head = 0; }
  // newest sample with time <= t (the oldest one kept if all are newer; undefined when empty)
  at(t) {
    if (!this.n) return undefined;
    for (let k = 1; k <= this.n; k++) { const i = (this.head - k + this.cap) % this.cap; if (this.t[i] <= t) return this.v[i]; }
    return this.v[(this.head - this.n + this.cap) % this.cap];
  }
}

// direction of Earth in world coordinates (same azimuth convention as Sim.setSun)
export function earthDir(azDeg = COMMS.earthAzDeg, elDeg = COMMS.earthElDeg) {
  const a = azDeg * Math.PI / 180, e = elDeg * Math.PI / 180;
  return { x: Math.cos(e) * Math.cos(a), y: Math.sin(e), z: -Math.cos(e) * Math.sin(a) };
}

export class LosMonitor {
  // opts.onRestore(): called when a LOS this monitor raised clears (main.js sends the queued commands)
  constructor(sim, opts = {}) {
    this.sim = sim; this.onRestore = opts.onRestore || null;
    this.nextT = 0; this.blockedN = 0; this.raised = false;
    this.last = { hab: true, earth: true, ok: true, t: -1 };
    this.ray = sim.RAPIER ? new sim.RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 1, y: 0, z: 0 }) : null;
    this.earth = earthDir();
  }
  _clear(o, d, max) {
    if (!this.ray) return true;
    const r = this.ray; r.origin.x = o.x; r.origin.y = o.y; r.origin.z = o.z; r.dir.x = d.x; r.dir.y = d.y; r.dir.z = d.z;
    const hit = this.sim.world.castRay(r, max, true, undefined, Q_SENSOR, undefined, this.sim.rover.body);
    return !hit;
  }
  mast() {
    const p = this.sim.rover.pose(), q = p.q;
    // body +y axis in world
    const ux = 2 * (q.x * q.y - q.w * q.z), uy = 1 - 2 * (q.x * q.x + q.z * q.z), uz = 2 * (q.y * q.z + q.w * q.x);
    return { x: p.x + ux * COMMS.mastH, y: p.y + uy * COMMS.mastH, z: p.z + uz * COMMS.mastH };
  }
  check() {
    const sim = this.sim, m = this.mast(), H = WORLD.habitat;
    const hy = heightAt(sim.terrain, H.x, H.z) + COMMS.habitatAntennaH;
    const dx = H.x - m.x, dy = hy - m.y, dz = H.z - m.z, L = Math.hypot(dx, dy, dz);
    // the last 15 m are the habitat itself (antenna mast on its roof)
    const hab = L < 20 || this._clear(m, { x: dx / L, y: dy / L, z: dz / L }, L - 15);
    const earth = this._clear(m, this.earth, COMMS.earthRayM);
    const local = sim.comms.mode === 'local' || !(sim.comms.delayS > 0);
    const ok = local ? hab : (earth || hab);
    this.last = { hab, earth, ok, t: sim.time };
    return this.last;
  }
  // call every frame; works once per COMMS.checkS of sim time
  update() {
    const sim = this.sim;
    if (sim.comms.autoLos === false || sim.time < this.nextT) return;
    this.nextT = sim.time + COMMS.checkS;
    const r = this.check(), A = sim.autonomy;
    this.blockedN = r.ok ? 0 : this.blockedN + 1;
    if (!r.ok && this.blockedN >= COMMS.persist && !A.faults.comms) {
      A.faults.comms = true; this.raised = true;
      sim.log(`COMMS LOS: ${sim.comms.mode === 'local' ? 'habitat antenna' : 'Earth and habitat'} below the horizon from the mast (terrain / structure in the path). Rover continues autonomously; commands queued.`, 'err');
    } else if (r.ok && this.raised) {
      this.raised = false;
      if (A.faults.comms) {
        A.faults.comms = false;
        sim.log(`Comms link re-acquired (${r.hab ? 'habitat' : 'Earth'} in view)`, 'ok');
        this.onRestore?.();
      }
    }
  }
}
