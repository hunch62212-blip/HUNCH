// AEGIS power dock: sun-tracking vertical solar arrays + a night-rated battery bank + an underbody
// charge coupler pad beside the supply building. The arrays charge the bank through the lunar day; the bank
// carries its own survival heaters and recharges the rover through the ~354 h lunar night.
import { POWER_DOCK, ROVER } from './config.js';
import { heightAt } from './terrain.js';
import { G_STRUCT, Q_TERRAIN, yawQuat } from './world.js';

export function powerDockLayout(hab) {
  const D = POWER_DOCK, n = { x: hab.nx, z: hab.nz }, t = { x: hab.tx, z: hab.tz };
  const at = (u, v) => ({ x: hab.face.x + t.x * u + n.x * v, z: hab.face.z + t.z * u + n.z * v });
  const pad = at(D.padT, D.padV);
  return {
    pad, yaw: Math.atan2(n.z, -n.x), // rover heading when parked: nose toward the building line (-n)
    entry: at(D.padT, D.padV + 26),
    bank: { ...at(D.padT, D.padV - 13), w: D.bank.w, d: D.bank.d, h: D.bank.h },
    pedestal: at(D.padT + 7.5, D.padV - 2),
    arrays: D.arrays.map(([u, v]) => at(u, v)),
    hab: { yaw: hab.yaw },
  };
}

export class PowerDock {
  constructor(sim) {
    this.sim = sim; this.rate = 10;
    const L = this.layout = powerDockLayout(sim.terrain.hab);
    const D = POWER_DOCK, RAPIER = sim.RAPIER;
    this.ground = heightAt(sim.terrain, L.pad.x, L.pad.z);
    // physics: array masts, battery bank vault and the charger pedestal are real obstacles
    const add = (desc) => sim.world.createCollider(desc.setCollisionGroups(G_STRUCT).setFriction(0.7), sim.fixed);
    for (const a of L.arrays) {
      const g = heightAt(sim.terrain, a.x, a.z);
      add(RAPIER.ColliderDesc.cylinder(D.mastH / 2, 0.45).setTranslation(a.x, g + D.mastH / 2, a.z));
    }
    { const b = L.bank, g = heightAt(sim.terrain, b.x, b.z);
      add(RAPIER.ColliderDesc.cuboid(b.d / 2, b.h / 2, b.w / 2).setTranslation(b.x, g + b.h / 2, b.z).setRotation(yawQuat(L.hab.yaw))); }
    { const p = L.pedestal, g = heightAt(sim.terrain, p.x, p.z);
      add(RAPIER.ColliderDesc.cuboid(0.5, 0.9, 0.5).setTranslation(p.x, g + 0.9, p.z).setRotation(yawQuat(L.hab.yaw))); }
    this.bankWh = D.bankWh * 0.85;
    this.solarW = 0; this.loadW = 0; this.chargeW = 0; this.netW = 0;
    this.trackAz = 0; this.sunlit = 1; this._shadowT = 0;
    this.connected = false; this.genWh = 0; this.deliveredWh = 0;
    this.status = 'standby';
  }
  get soc() { return this.bankWh / POWER_DOCK.bankWh; }
  roverOnPad() {
    const R = this.sim.rover, p = R.pose(), L = this.layout;
    return Math.hypot(p.x - L.pad.x, p.z - L.pad.z) < POWER_DOCK.couplerTol && R.telemetry().speed < 0.08;
  }
  update(dt) {
    const sim = this.sim, D = POWER_DOCK, sun = sim.sunDir, R = sim.rover;
    // arrays: vertical panels on single-axis azimuth trackers (ideal for a low polar sun)
    this.trackAz = Math.atan2(-sun.z, sun.x);
    this._shadowT -= dt;
    if (this._shadowT <= 0) {
      this._shadowT = 2;
      const a = this.layout.arrays[0], g = heightAt(sim.terrain, a.x, a.z);
      const o = { x: a.x + sun.x * 4, y: g + D.mastH * 0.7 + sun.y * 4, z: a.z + sun.z * 4 };
      this.sunlit = sun.y > -0.005 && !sim.world.castRay(new sim.RAPIER.Ray(o, { x: sun.x, y: sun.y, z: sun.z }), 3000, true, undefined, Q_TERRAIN) ? 1 : 0;
    }
    const cosI = Math.sqrt(Math.max(0, 1 - sun.y * sun.y)); // panel normal horizontal, pointed at the sun azimuth
    this.solarW = D.arrays.length * D.arrayArea * D.efficiency * 1361 * cosI * this.sunlit;
    const night = this.solarW < 500;
    // survival load: avionics + vault thermal control (heaters work hardest at night, ~ -170 C)
    this.loadW = D.hotelW + (night ? D.nightHeaterW : D.dayHeaterW);
    // charging: underbody coupler engages when the rover is centred on the pad and knelt onto it
    const onPad = this.roverOnPad();
    const coupled = onPad && R.lift.pos < ROVER.ride.kneel + 0.12 && !R.estop && !(sim.health && sim.health.fx.coupler); // coupler fault: no pilot signal
    if (coupled && !this.connected) sim.log(`POWER DOCK: coupler engaged, charging at up to ${(D.chargeMaxW / 1000).toFixed(0)} kW`, 'ok');
    if (!coupled && this.connected) sim.log(`POWER DOCK: coupler released (rover ${(R.power.soc * 100).toFixed(0)}%, bank ${(this.soc * 100).toFixed(0)}%)`, 'info');
    this.connected = coupled;
    let want = 0;
    if (coupled) {
      const s = R.power.soc;
      // CC-CV: full power to 80 %, tapering to zero at 100 % (pack C-rate limited)
      want = s >= 0.999 ? 0 : D.chargeMaxW * (s < 0.8 ? 1 : Math.max(0.08, (1 - s) / 0.2));
    }
    const avail = Math.max(0, this.solarW - this.loadW) + (this.bankWh > D.bankWh * D.bankFloor ? D.bankMaxDischargeW : 0);
    this.chargeW = Math.min(want, avail);
    this.netW = this.solarW - this.loadW - this.chargeW / D.chargerEff;
    const cap = D.bankWh;
    const dWh = (this.netW > 0 ? this.netW * D.bankEff : this.netW) * dt / 3600;
    this.bankWh = Math.max(0, Math.min(cap, this.bankWh + dWh));
    this.genWh += this.solarW * dt / 3600; this.deliveredWh += this.chargeW * dt / 3600;
    R.power.dockW = this.chargeW;
    this.status = coupled ? (this.chargeW > 0 ? 'CHARGING' : 'ROVER FULL') : onPad ? 'rover on pad (kneel to couple)' : night ? 'NIGHT: bank supplying' : this.bankWh >= cap - 1 ? 'bank full' : 'charging bank';
  }
  // hours the bank can carry the dock's survival load through a night with no solar input
  nightHours() { return this.bankWh / (POWER_DOCK.hotelW + POWER_DOCK.nightHeaterW); }
}
