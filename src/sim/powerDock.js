// AEGIS power dock, FULL-SCALE electrics (docs/LUNAR_CHARGING.md, docs/SCALING.md): two sun-tracking vertical solar masts
// (VSAT-style, 8.4 kW of cells) feed a 200 kWh LiFePO4 bank in a heated, bermed vault; an INDUCTIVE pad (no exposed contacts,
// dust tolerant, after the Astrobotic / WiBotic lunar wireless charger concept) charges the rover's 150 kWh, 400 V pack.
// Everything here is in real units (V, A, W, Wh) and R.power.dockW = chargeW directly. Until the review pass the dock was
// modelled at prototype scale and multiplied by 1172 (150 kWh / 128 Wh), which implied a 148 kW array and a 2.8 MWh bank.
import { POWER_DOCK, PACK, LFP_OCV, ROVER, MOON } from './config.js';
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
    this.Tbank = D.vault.initC; this.heaterW = 0; this.bankA = 0;
    this.solarW = 0; this.loadW = 0; this.chargeW = 0; this.chargeA = 0; this.inW = 0; this.netW = 0;
    this.packV = PACK.cells * ocv(0.5); this.linkEff = D.linkEff;
    this.night = false; this.bankCold = false; this.curtailed = false; this.coldHold = false;
    this.trackAz = 0; this.sunlit = 1; this._shadowT = 0;
    this.connected = false; this.genWh = 0; this.deliveredWh = 0;
    this.status = 'standby';
  }
  get soc() { return this.bankWh / POWER_DOCK.bankWh; }
  get bankAhRated() { return POWER_DOCK.bankAh; }
  get bankAhNow() { return this.soc * POWER_DOCK.bankAh; }
  get bankV() { return POWER_DOCK.bankCells * ocv(this.soc) + this.bankA * POWER_DOCK.bankR; }
  get floorSoc() { return this.night ? POWER_DOCK.nightReserveSoc : POWER_DOCK.bankFloor; }
  get atFloor() { return this.soc <= this.floorSoc + 1e-6; }
  // Pad pilot-coil alignment read-out: the inductive pilot reports the coil offset and the heading error (mod 180 deg, the
  // rover can park either way round) of the receiver coil against the pad, used for the final square-up before kneeling.
  padAlignment() {
    const p = this.sim.rover.pose(), L = this.layout;
    let dyaw = Math.atan2(Math.sin(p.yaw - L.yaw), Math.cos(p.yaw - L.yaw));
    if (Math.abs(dyaw) > Math.PI / 2) dyaw -= Math.sign(dyaw) * Math.PI;
    return { dyaw, offset: Math.hypot(p.x - L.pad.x, p.z - L.pad.z) };
  }
  roverOnPad() {
    const R = this.sim.rover, p = R.pose(), L = this.layout;
    const dyaw = Math.abs(Math.atan2(Math.sin(p.yaw - L.yaw), Math.cos(p.yaw - L.yaw)));
    const aligned = Math.min(dyaw, Math.abs(Math.PI - dyaw)) < 8 * Math.PI / 180;
    return aligned && Math.hypot(p.x - L.pad.x, p.z - L.pad.z) < POWER_DOCK.couplerTol && R.telemetry().speed < 0.08;
  }
  update(dt) {
    const sim = this.sim, D = POWER_DOCK, V = D.vault, sun = sim.sunDir, R = sim.rover, cap = D.bankWh;
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
    // 1. solar: AM0 flux x cell efficiency x derate x MPPT, on the full-scale panel area (vertical panel facing the sun azimuth)
    const cosI = Math.sqrt(Math.max(0, 1 - sun.y * sun.y));
    this.solarW = D.arrays.length * D.panelArea * D.cellEff * D.arrayDerate * D.mpptEff * MOON.solarConstant * cosI * this.sunlit;
    const night = this.night = this.solarW < D.nightSolarW;
    // 2. bank vault thermal node: heater (thermostat on the vault air), RHU, bank I^2 R, loss to the bermed sink
    const atHard = this.bankWh <= D.bankFloor * cap;
    this.heaterW = atHard ? 0 : clamp(V.heaterGain * (V.setC - this.Tbank), 0, V.heaterMaxW);
    const sink = night ? V.sinkNightC : V.sinkDayC;
    this.Tbank += dt * (this.heaterW + V.rhuW + this.bankA * this.bankA * D.bankR - V.G * (this.Tbank - sink)) / V.C;
    // 3. survival load: avionics + vault heater (+ inductive pilot / foreign-object detection while a rover is on the pad)
    const onPad = this.roverOnPad();
    this.loadW = D.hotelW + (onPad ? D.padStandbyW : 0) + this.heaterW;
    // 4. charging: the inductive link engages when the rover is centred on the pad and knelt onto it (closes the coil gap)
    const coupled = onPad && R.lift.pos < ROVER.ride.kneel + 0.12 && !R.estop && !(sim.health && sim.health.fx.coupler); // coupler fault: no pilot signal
    if (coupled && !this.connected) sim.log(`POWER DOCK: inductive link engaged, up to ${D.chargeCC_A.toFixed(1)} A into the pack`, 'ok');
    if (!coupled && this.connected) sim.log(`POWER DOCK: inductive link released (rover ${(R.power.soc * 100).toFixed(0)}%, bank ${(this.soc * 100).toFixed(0)}%)`, 'info');
    this.connected = coupled;
    let wantW = 0;
    if (coupled) {
      const s = R.power.soc;
      // CC-CV into the rover pack: constant current to 80 %, tapering to zero at 100 %
      let I = s >= 0.999 ? 0 : D.chargeCC_A * (s < 0.8 ? 1 : Math.max(0.08, (1 - s) / 0.2));
      // cell-temperature limits: no charge below chargeMinCellC (lithium plating; the pack heaters warm it first),
      // taper to zero between 40 C and the 50 C pack limit
      const Tb = sim.health ? sim.health.th.Tb : 20;
      this.coldHold = Tb < (D.chargeMinCellC ?? -99);
      if (this.coldHold) I = 0; else I *= Math.max(0, Math.min(1, (48 - Tb) / 8));
      this.packV = Math.min(PACK.cells * PACK.cvCell, PACK.cells * ocv(s) + I * PACK.R);
      wantW = I * this.packV;
      const p = R.pose(), d = Math.hypot(p.x - this.layout.pad.x, p.z - this.layout.pad.z);
      this.linkEff = D.linkEff * (1 - D.linkOffsetLoss * (d / D.couplerTol) ** 2); // coil misalignment loss
    } else this.coldHold = false;
    // 6. what the dock can supply: solar surplus + bank discharge (down to the floor; the night reserve at night)
    const canDis = this.bankWh > this.floorSoc * cap && this.Tbank > D.dischargeMinBankC;
    const avail = Math.max(0, this.solarW - this.loadW) + (canDis ? D.bankMaxDischargeW : 0);
    this.inW = Math.min(wantW / this.linkEff, avail, D.linkMaxW);
    // the charger rating (20 kW, ROVER.battery.dockMaxW) limits the charge, so the bank, the delivered-Wh accounting and the rover
    // pack all see the same power
    const maxW = ROVER.battery.dockMaxW;
    if (this.inW * this.linkEff > maxW) this.inW = maxW / this.linkEff;
    this.chargeW = this.inW * this.linkEff;                       // W into the rover pack
    this.chargeA = this.packV > 0 ? this.chargeW / this.packV : 0;
    // 7. bank energy balance (LiFePO4 will not take charge below 0 C: the MPPT backs off until the vault heater warms it)
    let netW = this.solarW - this.loadW - this.inW;
    this.bankCold = this.Tbank < D.chargeMinBankC;
    this.curtailed = false;
    if (netW > 0 && this.bankCold) { this.curtailed = true; netW = 0; }
    netW = Math.min(netW, D.bankMaxChargeA * D.bankV);
    this.netW = netW;
    const dWh = (netW > 0 ? netW * D.bankEff : netW / D.bankEff) * dt / 3600; // loss on the way in and on the way out
    this.bankWh = clamp(this.bankWh + dWh, 0, cap);
    this.bankA = netW / Math.max(1, this.bankV);
    this.genWh += this.solarW * dt / 3600; this.deliveredWh += this.chargeW * dt / 3600;
    // 9. the coupling to the rover: the pack takes chargeW (already limited to the charger rating above)
    R.power.dockW = Math.min(this.chargeW, ROVER.battery.dockMaxW);
    // (a coupled rover that gets no power is not always full: the bank may be at its floor, or the pack too hot)
    const Tb1 = sim.health ? sim.health.th.Tb : 20;
    const floorTxt = night ? `HOLD: bank at night reserve ${(D.nightReserveSoc * 100).toFixed(0)} %, waiting for sunrise` : `HOLD: bank at floor ${(D.bankFloor * 100).toFixed(0)} %, waiting for sun`;
    this.status = coupled ? (this.chargeW > 0 ? 'CHARGING' : this.coldHold ? 'HOLD: pack below 0 C, warming' : R.power.soc >= 0.999 ? 'ROVER FULL' : Tb1 >= 50 ? 'HOLD: pack at 50 C, cooling' : floorTxt)
      : onPad ? 'rover on pad (kneel to couple)' : night ? 'NIGHT: bank supplying' : this.curtailed ? 'bank cold: warming before charge' : this.bankWh >= cap - 1 ? 'bank full' : 'charging bank';
  }
  // dock survival load on a lunar night: avionics + the part of the vault heat loss the RHU units (rhuW) do not cover
  nightLoadW() { const D = POWER_DOCK; return D.hotelW + Math.max(0, D.vault.G * (D.vault.setC - D.vault.sinkNightC) - D.vault.rhuW); }
  // hours the bank can carry that load with no solar input (from its current charge down to the BMS floor)
  nightHours() { const D = POWER_DOCK; return (this.bankWh - D.bankFloor * D.bankWh) * D.bankEff / this.nightLoadW(); }
  // hours of sun to refill the bank from the floor at the present solar input
  refillHours() { const D = POWER_DOCK; return D.bankWh * (1 - D.bankFloor) / D.bankEff / Math.max(0.01, this.solarW - this.loadW); }
  // minutes to charge the rover pack to `target` SoC at the present charge power
  etaMin(target) { return Math.max(0, target - this.sim.rover.power.soc) * ROVER.battery.capacityWh / Math.max(1, this.chargeW) * 60; }
}
