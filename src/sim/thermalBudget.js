// Steady-state thermal and energy budget of the rover's vault, battery and hubs, in closed form (review item 7). The same
// equations as health.js _thermal, solved for equilibrium instead of stepped, so the night-energy and hot-day numbers can be
// stated without running the sim. Night survival used to be "measured" by tests/fdir.mjs E over 11 simulated minutes, which is
// the transient of the heaters catching up (about 154 W), not the steady state (several times that with the old 12 m^2 radiator).
import { THERMAL, ROVER, POWER_DOCK } from './config.js';
import { SOLAR, RAD_ALPHA } from './lunarParams.js';

const K0 = 273.15, sig = THERMAL.sigma;
const sinkTemp = (regK, T = THERMAL) => (T.skyViewFraction * regK ** 4) ** 0.25;     // effective radiative sink, K
const sinkT4 = (regK, T = THERMAL) => T.skyViewFraction * regK ** 4;

// heat a box loses at temperature Tc (C): radiator at louver fraction l over both sides, plus the conduction leak
export function boxLossW(Tc, b, leak, l, regK, T = THERMAL) {
  return l * b.emis * sig * b.radArea * ((Tc + K0) ** 4 - sinkT4(regK, T)) + leak * (Tc + K0 - sinkTemp(regK, T));
}

// Survival at the set points on a lunar night. Returns the electrical load of the parked rover (W): hotel + vault, battery and hub heaters.
//   cfg overrides: THERMAL-like { vault, battery, leakWperK, hub, ... }, hotelW, armW, rhu { vault, battery } (W of isotope heat)
export function survivalNight(cfg = {}) {
  const T = { ...THERMAL, ...cfg }, regK = cfg.regK ?? T.sinkNightK;
  const hotelW = cfg.hotelW ?? 60, armW = cfg.armW ?? 3, rhu = cfg.rhu || { vault: 0, battery: 0 };
  const vaultLoss = boxLossW(T.vault.survivalSetC, T.vault, T.leakWperK.vault, T.vault.louverMin, regK, T);
  const battLoss = boxLossW(T.battery.survivalSetC, T.battery, T.leakWperK.battery, T.battery.louverMin, regK, T);
  const vaultHeater = Math.max(0, vaultLoss - hotelW - rhu.vault);        // the 60 W hotel is dissipated in the vault
  const battHeater = Math.max(0, battLoss - rhu.battery);
  const H = T.hub, hubLossW = H.emis * sig * H.radArea * ((T.hubFreezeC + K0) ** 4 - sinkT4(regK, T)); // per hub, held at the freeze point
  const hubs = 6 * Math.min(hubLossW, T.hubHeaterW);
  const loadW = hotelW + armW + vaultHeater + battHeater + hubs;
  const nightKWh = loadW * 354 / 1000;
  return { loadW, vaultLoss, battLoss, vaultHeater, battHeater, hubs, hotelW, armW, nightKWh, packPct: nightKWh * 1000 / ROVER.battery.capacityWh * 100 };
}

// Dock overnight: the dock bank carries the rover's survival load through the pad link (PowerDock above nightReserveSoc)
export function dockCarriesNight(loadW, D = POWER_DOCK) {
  const needKWh = loadW * 354 / 1000;                                            // into the rover pack
  const availKWh = (1 - D.nightReserveSoc) * D.bankWh / 1000 * D.bankEff * D.linkEff;
  return { needKWh, availKWh, margin: availKWh / needKWh };
}

// Day case: vault equilibrium temperature (C) with avionics and instruments on, regolith sink at regK, sun on one side at cosine cs.
export function hotDayVault({ cfg = {}, alpha = RAD_ALPHA.clean, regK = THERMAL.sinkDayK, cs = 0.99, loadW = ROVER.avionicsW + ROVER.lidarW + ROVER.cameraW + ROVER.commsW } = {}) {
  const T = { ...THERMAL, ...cfg }, V = T.vault;
  const balance = (Tc) => {
    const lv = Math.min(1, Math.max(V.louverMin, (Tc - V.louverOpenC) / (V.louverFullC - V.louverOpenC)));
    const A = V.radArea / 2, e4 = (Tc + K0) ** 4 - sinkT4(regK, T);
    let rej = T.leakWperK.vault * (Tc + K0 - sinkTemp(regK, T)), sol = 0;
    for (const side of [cs, 0]) {
      const qSun = alpha * SOLAR.S0 * A * side, open = V.emis * sig * A * e4;
      const l = open - qSun < 0 ? V.louverMin : lv;
      rej += l * open; sol += l * qSun;
    }
    return loadW + sol - rej;
  };
  let lo = -60, hi = 200;
  for (let i = 0; i < 80; i++) { const m = (lo + hi) / 2; if (balance(m) > 0) lo = m; else hi = m; }
  return (lo + hi) / 2;
}

// Day case: battery equilibrium (C) while charging at the dock (I^2 R heat), sun on one side
export function hotDayBattery({ cfg = {}, alpha = RAD_ALPHA.clean, regK = THERMAL.sinkDayK, cs = 0.99, chargeW = ROVER.battery.dockMaxW } = {}) {
  const T = { ...THERMAL, ...cfg }, B = T.battery;
  const qB = (chargeW / ROVER.battery.nominalV) ** 2 * ROVER.battery.internalR;
  const balance = (Tc) => {
    const lb = Math.min(1, Math.max(B.louverMin, (Tc - B.louverOpenC) / (B.louverFullC - B.louverOpenC)));
    const A = B.radArea / 2, e4 = (Tc + K0) ** 4 - sinkT4(regK, T);
    let rej = T.leakWperK.battery * (Tc + K0 - sinkTemp(regK, T)), sol = 0;
    for (const side of [cs, 0]) { const qSun = alpha * SOLAR.S0 * A * side, open = B.emis * sig * A * e4; const l = open - qSun < 0 ? B.louverMin : lb; rej += l * open; sol += l * qSun; }
    return qB + sol - rej;
  };
  let lo = -60, hi = 200;
  for (let i = 0; i < 80; i++) { const m = (lo + hi) / 2; if (balance(m) > 0) lo = m; else hi = m; }
  return (lo + hi) / 2;
}
