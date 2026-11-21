// AEGIS power dock (docs/CHARGING_DESIGN.md, docs/LUNAR_CHARGING.md). Energy source, PROTOTYPE-SCALE electrics: two sun-tracking
// vertical solar masts (VSAT-style) feed a 24 V 100 Ah LiFePO4 bank in a heated, bermed vault. Charging interface, FULL-SCALE
// geometry (src/sim/chargingParams.js): a SEGMENTED INDUCTIVE PAD (3 x 2 sealed transmitter modules flush in a sintered-regolith
// slab, no exposed contacts, after the Astrobotic / WiBotic lunar wireless charger) and a 0.9 m receiver on the rover belly. The
// rover centres on the pad, kneels to close the air gap and charges at up to the 20 kW charger cap; the link also carries the
// rover's bus load, so the pack gets the whole charge current, and parked on the pad at night ("night park") the dock keeps the
// rover alive from its bank. Bank / solar / pack electrics are in prototype units (V, A, W, Wh); the rover pack is treated as the
// 128 Wh prototype pack at the same state of charge and the only coupling to the 150 kWh sim pack is
// R.power.dockW = (chargeW + busW) x PROTO_TO_FULL_WH (config.js), capped at ROVER.battery.dockMaxW.
import { POWER_DOCK, PROTO_PACK, PROTO_TO_FULL_WH, LFP_OCV, ROVER, MOON } from './config.js';
import { CHARGING, padFrame } from './chargingParams.js';
import { heightAt } from './terrain.js';
import { G_STRUCT, Q_SENSOR, yawQuat } from './world.js';

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

const ocv = (soc) => { // per-cell LiFePO4 rest voltage, linear between table points
  const T = LFP_OCV, s = Math.max(0, Math.min(1, soc));
  for (let i = 1; i < T.length; i++) if (s <= T[i][0]) return T[i - 1][1] + (T[i][1] - T[i - 1][1]) * (s - T[i - 1][0]) / (T[i][0] - T[i - 1][0]);
  return T[T.length - 1][1];
};
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
// rotate body-frame vector v by the unit quaternion q (x, y, z, w)
const qRot = (q, v) => {
  const ix = q.w * v.x + q.y * v.z - q.z * v.y, iy = q.w * v.y + q.z * v.x - q.x * v.z, iz = q.w * v.z + q.x * v.y - q.y * v.x, iw = -q.x * v.x - q.y * v.y - q.z * v.z;
  return { x: ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y, y: iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z, z: iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x };
};
// full power inside `tol`, falling linearly to zero at `max` (the GA cannot drive rated current into a partly covered receiver)
const overlap = (x, tol, max) => clamp((max - Math.abs(x)) / (max - tol), 0, 1);

export class PowerDock {
  constructor(sim) {
    this.sim = sim; this.rate = 10;
    const L = this.layout = powerDockLayout(sim.terrain.hab);
    const D = POWER_DOCK, RAPIER = sim.RAPIER;
    this.ground = heightAt(sim.terrain, L.pad.x, L.pad.z);
    // physics: array masts, battery bank vault and the charger pedestal are real obstacles. The pad itself is flush (cover top
    // CHARGING.ga.topH = 57 mm, between the wheel tracks, under the kneeled belly): nothing can touch it, so it has no collider.
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
    this.Tbank = D.vault.initC; this.heaterW = 0; this.bankA = 0;
    this.solarW = 0; this.loadW = 0; this.chargeW = 0; this.chargeA = 0; this.busW = 0; this.inW = 0; this.netW = 0;
    this.packV = PROTO_PACK.cells * ocv(0.5); this.linkEff = CHARGING.eta.centre;
    this.night = false; this.bankCold = false; this.curtailed = false; this.coldHold = false; this.keepAlive = false;
    this.trackAz = 0; this.sunlit = 1; this._shadowT = 0;
    this.connected = false; this.genWh = 0; this.deliveredWh = 0; this.busWh = 0;
    // link state (full-scale W): coupling fraction, thermal derate, receiver / pad temperatures, loss heat into the rover
    const TH = CHARGING.thermal;
    this.Trx = TH.rx.initC; this.Tga = TH.ga.initC; this.linkFrac = 0; this.thermalFrac = 1; this.lossW = 0; this.rxHeatW = 0;
    this.align = { along: 0, across: 0, dyaw: 0, offset: 0, gap: 1, inCapture: false };
    this.scale = PROTO_TO_FULL_WH;
    this.status = 'standby';
  }
  get soc() { return this.bankWh / POWER_DOCK.bankWh; }
  get bankAhRated() { return POWER_DOCK.bankAh; }
  get bankAhNow() { return this.soc * POWER_DOCK.bankAh; }
  get bankV() { return POWER_DOCK.bankCells * ocv(this.soc) + this.bankA * POWER_DOCK.bankR; }
  get floorSoc() { return this.night ? POWER_DOCK.nightReserveSoc : POWER_DOCK.bankFloor; }
  get atFloor() { return this.soc <= this.floorSoc + 1e-6; }
  // Pad pilot-coil read-out: receiver-coil position against the pad centre in the pad frame (along the parked heading, across it),
  // heading error (mod 180 deg, the rover may park either way round), horizontal offset and the air gap between the receiver face
  // and the pad cover. Used for the final square-up before kneeling and for the capture / coupling checks.
  padAlignment() {
    const p = this.sim.rover.pose(), L = this.layout, C = CHARGING;
    const r = qRot(p.q, { x: C.rx.x, y: C.rx.yFace, z: C.rx.z });
    const f = padFrame(p.x + r.x - L.pad.x, p.z + r.z - L.pad.z, L.yaw);
    let dyaw = Math.atan2(Math.sin(p.yaw - L.yaw), Math.cos(p.yaw - L.yaw));
    if (Math.abs(dyaw) > Math.PI / 2) dyaw -= Math.sign(dyaw) * Math.PI;
    const gap = p.y + r.y - this.ground - C.ga.topH;
    const inCapture = Math.abs(dyaw) < C.tol.yawDeg * Math.PI / 180 && Math.abs(f.along) < C.tol.alongMax && Math.abs(f.across) < C.tol.acrossMax;
    return this.align = { along: f.along, across: f.across, dyaw, offset: Math.hypot(f.along, f.across), gap, inCapture };
  }
  roverOnPad() {
    const al = this.padAlignment();
    return al.inCapture && this.sim.rover.telemetry().speed < CHARGING.tol.speed;
  }
  update(dt) {
    const sim = this.sim, D = POWER_DOCK, V = D.vault, sun = sim.sunDir, R = sim.rover, cap = D.bankWh, C = CHARGING, TH = C.thermal;
    // arrays: vertical panels on single-axis azimuth trackers (ideal for a low polar sun)
    this.trackAz = Math.atan2(-sun.z, sun.x);
    this._shadowT -= dt;
    if (this._shadowT <= 0) {
      this._shadowT = 2;
      // lit fraction of every mast: terrain / structure shadow (ray from the panel centre) and the shadow of the masts
      // sunward of it. A panel wing is panelW wide and panelH tall, facing the sun; a wing d metres up-sun and l metres
      // off the sun line covers (panelW - |l|) of the width and (panelH - d tan(el)) of the height of the wing behind.
      const A = this.layout.arrays, hs = Math.hypot(sun.x, sun.z) || 1, sx = sun.x / hs, sz = sun.z / hs, tanEl = Math.max(0, sun.y) / hs;
      let lit = 0;
      this.litFrac = A.map((a, i) => {
        if (sun.y <= -0.005) return 0;
        const g = heightAt(sim.terrain, a.x, a.z);
        const o = { x: a.x + sun.x * 0.6, y: g + D.mastH - D.panelH / 2 + sun.y * 0.6, z: a.z + sun.z * 0.6 };
        if (sim.world.castRay(new sim.RAPIER.Ray(o, { x: sun.x, y: sun.y, z: sun.z }), 3000, true, undefined, Q_SENSOR)) return 0;
        let f = 1;
        for (let j = 0; j < A.length; j++) {
          if (j === i) continue;
          const dx = A[j].x - a.x, dz = A[j].z - a.z, d = dx * sx + dz * sz, l = Math.abs(-dx * sz + dz * sx);
          if (d <= 0) continue;
          const dy = heightAt(sim.terrain, A[j].x, A[j].z) - g - d * tanEl; // shadow of mast j, vertical offset at mast i
          const cover = Math.max(0, 1 - l / D.panelW) * Math.max(0, 1 - Math.abs(dy) / D.panelH);
          f *= 1 - cover;
        }
        lit += f; return f;
      });
      this.sunlit = lit / A.length;
    }
    // 1. solar: AM0 flux x cell efficiency x derate x MPPT, on the prototype panel area (vertical panel facing the sun azimuth)
    const cosI = Math.sqrt(Math.max(0, 1 - sun.y * sun.y));
    this.solarW = D.arrays.length * D.panelAreaProto * D.cellEff * D.arrayDerate * D.mpptEff * MOON.solarConstant * cosI * this.sunlit;
    const night = this.night = this.solarW < D.nightSolarW;
    // 2. bank vault thermal node: heater (thermostat on the vault air), RHU, bank I^2 R, loss to the bermed sink
    const atHard = this.bankWh <= D.bankFloor * cap;
    this.heaterW = atHard ? 0 : clamp(V.heaterGain * (V.setC - this.Tbank), 0, V.heaterMaxW);
    const sink = night ? V.sinkNightC : V.sinkDayC;
    this.Tbank += dt * (this.heaterW + V.rhuW + this.bankA * this.bankA * D.bankR - V.G * (this.Tbank - sink)) / V.C;
    // 3. survival load: avionics + vault heater (+ pad pilot / foreign-object detection while a rover is in the capture envelope)
    const onPad = this.roverOnPad(), al = this.align;
    this.loadW = D.hotelW + (onPad ? D.padStandbyW * (night && this.connected ? C.night.pilotDuty : 1) : 0) + this.heaterW;
    // 4. coupling: rover in the capture envelope, still, kneeled so the air gap is inside the coil's range, pilot handshake present
    //    (coupler fault = no pilot signal), receiver and pad below their cut-out temperature
    const gapOk = al.gap >= C.gap.min && al.gap <= C.gap.max;
    const hot = this.Trx >= TH.rx.maxC || this.Tga >= TH.ga.maxC;
    const coupled = onPad && gapOk && !hot && !R.estop && !(sim.health && sim.health.fx.coupler);
    if (coupled && !this.connected) sim.log(`POWER DOCK: inductive link engaged (receiver ${al.along.toFixed(2)} m along / ${al.across.toFixed(2)} m across, gap ${(al.gap * 1000).toFixed(0)} mm), up to ${(C.maxW / 1000).toFixed(0)} kW`, 'ok');
    if (!coupled && this.connected) sim.log(`POWER DOCK: inductive link released (rover ${(R.power.soc * 100).toFixed(0)}%, bank ${(this.soc * 100).toFixed(0)}%)`, 'info');
    this.connected = coupled;
    // link capability: coil overlap (pad frame) x thermal derate of the hotter coil; efficiency falls with offset and gap
    this.linkFrac = coupled ? Math.min(overlap(al.along, C.tol.along, C.tol.alongMax), overlap(al.across, C.tol.across, C.tol.acrossMax)) : 0;
    this.thermalFrac = Math.min(clamp((TH.rx.maxC - this.Trx) / (TH.rx.maxC - TH.rx.derateC), 0, 1), clamp((TH.ga.maxC - this.Tga) / (TH.ga.maxC - TH.ga.derateC), 0, 1));
    { const u = Math.min(1, Math.max(Math.abs(al.along) / C.tol.alongMax, Math.abs(al.across) / C.tol.acrossMax));
      this.linkEff = C.eta.centre * (1 - C.eta.edgeLoss * u * u) * (1 - C.eta.gapLossPerM * Math.max(0, al.gap - C.gap.nom)); }
    const eta = this.linkEff, S = this.scale;
    const linkOutW = C.maxW * this.linkFrac * this.thermalFrac / S; // prototype W the link can put on the rover bus
    // 5. rover demand: its bus load first (pass-through), then the CC-CV charge current into the pack
    let chargeWant = 0, busWant = 0;
    if (coupled) {
      const s = R.power.soc;
      busWant = Math.max(0, R.power.loadW - (R.power.solarW || 0) - (R.power.auxSolarW || 0)) / S;
      // CC-CV into the prototype pack: constant current to 80 %, tapering to zero at 100 %
      let I = s >= 0.999 ? 0 : D.chargeCC_A * (s < 0.8 ? 1 : Math.max(0.08, (1 - s) / 0.2));
      // cell-temperature limits: no charge below chargeMinCellC (lithium plating; the pack heaters warm it first),
      // taper to zero between 40 C and the 50 C pack limit
      const Tb = sim.health ? sim.health.th.Tb : 20;
      this.coldHold = Tb < (D.chargeMinCellC ?? -99);
      if (this.coldHold) I = 0; else I *= Math.max(0, Math.min(1, (48 - Tb) / 8));
      this.packV = Math.min(PROTO_PACK.cells * PROTO_PACK.cvCell, PROTO_PACK.cells * ocv(s) + I * PROTO_PACK.R);
      chargeWant = I * this.packV;
    } else this.coldHold = false;
    // 6. what the dock can supply (prototype W at the dock bus): solar surplus + bank discharge. Charging may draw the bank down to
    //    its floor (the 50 % night reserve at night); the rover keep-alive (bus load at night, capped) may go down to keepAliveFloorSoc.
    const surplus = Math.max(0, this.solarW - this.loadW), warmBank = this.Tbank > D.dischargeMinBankC;
    const canDis = warmBank && this.bankWh > this.floorSoc * cap;
    const canKeep = warmBank && this.bankWh > Math.min(this.floorSoc, C.night.keepAliveFloorSoc) * cap;
    const busCap = night ? C.night.keepAliveMaxW / S : Infinity;
    const busW = Math.max(0, Math.min(busWant, busCap, linkOutW, (surplus + (canKeep ? D.bankMaxDischargeW : 0)) * eta));
    const chargeW = Math.max(0, Math.min(chargeWant, linkOutW - busW, (surplus + (canDis ? D.bankMaxDischargeW : 0)) * eta - busW));
    this.keepAlive = coupled && night && busW > 0;
    this.busW = busW; this.chargeW = chargeW;                      // prototype W onto the rover bus / into the pack
    this.inW = (busW + chargeW) / eta;                             // prototype W drawn from the dock bus
    this.chargeA = this.packV > 0 ? this.chargeW / this.packV : 0;
    // 7. link thermal (full-scale W): the loss heats the receiver (into the rover pack loop) and the pad (into the slab)
    this.lossW = (this.inW - busW - chargeW) * S;
    this.rxHeatW = this.lossW * TH.rx.share;
    const Tpack = sim.health ? sim.health.th.Tb : 20;
    this.Trx += dt * (this.rxHeatW - TH.rx.G * (this.Trx - Tpack)) / TH.rx.C;
    this.Tga += dt * (this.lossW * TH.ga.share - TH.ga.G * (this.Tga - (night ? TH.ga.sinkNightC : TH.ga.sinkDayC))) / TH.ga.C;
    // 8. bank energy balance (LiFePO4 will not take charge below 0 C: the MPPT backs off until the vault heater warms it)
    let netW = this.solarW - this.loadW - this.inW;
    this.bankCold = this.Tbank < D.chargeMinBankC;
    this.curtailed = false;
    if (netW > 0 && this.bankCold) { this.curtailed = true; netW = 0; }
    netW = Math.min(netW, D.bankMaxChargeA * D.bankV);
    this.netW = netW;
    const dWh = (netW > 0 ? netW * D.bankEff : netW / D.bankEff) * dt / 3600; // loss on the way in and on the way out
    this.bankWh = clamp(this.bankWh + dWh, 0, cap);
    this.bankA = netW / Math.max(1, this.bankV);
    this.genWh += this.solarW * dt / 3600; this.deliveredWh += this.chargeW * dt / 3600; this.busWh += this.busW * dt / 3600;
    // 9. the only coupling to the full-scale sim: bus load + charge into the 150 kWh pack, limited to the full-scale charger rating
    R.power.dockW = Math.min((this.chargeW + this.busW) * S, ROVER.battery.dockMaxW);
    // (a coupled rover that gets no charge is not always full: the bank may be at its floor, the pack too hot, the coils too hot)
    const Tb1 = sim.health ? sim.health.th.Tb : 20;
    const floorTxt = night ? `HOLD: bank at night reserve ${(D.nightReserveSoc * 100).toFixed(0)} %, waiting for sunrise` : `HOLD: bank at floor ${(D.bankFloor * 100).toFixed(0)} %, waiting for sun`;
    this.status = coupled
      ? (this.chargeW > 0 ? (this.thermalFrac < 1 ? 'CHARGING (coil hot, derated)' : this.linkFrac < 1 ? 'CHARGING (off-centre, derated)' : 'CHARGING')
        : this.keepAlive ? 'NIGHT PARK: dock carrying rover load' : this.coldHold ? 'HOLD: pack below 0 C, warming' : R.power.soc >= 0.999 ? 'ROVER FULL (bus supplied)' : Tb1 >= 50 ? 'HOLD: pack at 50 C, cooling' : floorTxt)
      : hot ? 'HOLD: charge coil over temperature' : onPad ? (gapOk ? 'rover on pad (no pilot signal)' : `rover on pad (kneel to couple: gap ${(al.gap * 1000).toFixed(0)} mm)`)
      : night ? 'NIGHT: bank supplying' : this.curtailed ? 'bank cold: warming before charge' : this.bankWh >= cap - 1 ? 'bank full' : 'charging bank';
  }
  // dock survival load on a lunar night: avionics + the part of the vault heat loss the 1 W RHU does not cover
  nightLoadW() { const D = POWER_DOCK; return D.hotelW + Math.max(0, D.vault.G * (D.vault.setC - D.vault.sinkNightC) - D.vault.rhuW); }
  // hours the bank can carry that load with no solar input (from its current charge down to the BMS floor)
  nightHours() { const D = POWER_DOCK; return (this.bankWh - D.bankFloor * D.bankWh) * D.bankEff / this.nightLoadW(); }
  // night park: hours the bank can carry its own night load plus a parked rover's bus load (full-scale W, default the ~154 W
  // survival load, docs/POWER_BUDGET.md) through the link, down to the keep-alive floor
  nightParkHours(roverW = 154) {
    const D = POWER_DOCK, C = CHARGING, w = Math.min(roverW, C.night.keepAliveMaxW) / this.scale / C.eta.centre + D.padStandbyW * C.night.pilotDuty;
    return (this.bankWh - C.night.keepAliveFloorSoc * D.bankWh) * D.bankEff / (this.nightLoadW() + w);
  }
  // hours of sun to refill the bank from the floor at the present solar input
  refillHours() { const D = POWER_DOCK; return D.bankWh * (1 - D.bankFloor) / D.bankEff / Math.max(0.01, this.solarW - this.loadW); }
  // minutes to charge the rover pack to `target` SoC at the present charge power (prototype Wh over prototype W)
  etaMin(target) { return Math.max(0, target - this.sim.rover.power.soc) * PROTO_PACK.Wh / Math.max(0.1, this.chargeW) * 60; }
}
