// Scale audit: K = 8.16 prints, 3.0 m module, 150 kWh, 4WD, ~5 t rover (user decisions, Nov 2026). run: node tests/unit_scale.mjs
// Pure config checks plus the derived quantities (ride frequency, braking limit, charger cap); no physics stepping.
import { within, equal, check } from './lib/check.mjs';
import { ROVER, K, K_LEGACY, MOON, SCALE, CARGO_TYPES, DRIVE, PACK, POWER_DOCK, PROTO_PACK } from '../src/sim/config.js';
import { froude, G_EARTH } from '../src/sim/scaling.js';
import { brakeDecel } from '../src/sim/dynamics.js';

const KS = K / K_LEGACY, g = MOON.g;
equal('K (prototype print scale)', K, 8.16);
within('K_LEGACY (old module print 12.192 / 0.747)', K_LEGACY, 16.32, 16.33, '', 3);
equal('battery capacity (Wh)', ROVER.battery.capacityWh, 150000);
// Froude scaling includes gravity (review item 6, src/sim/scaling.js, docs/SCALING.md)
{ const gR = MOON.g / G_EARTH, f = froude(K, MOON.g);
  within('gR = g_moon / g_earth', gR, 0.1650, 0.1653, '', 4);
  within('wheel rpm ratio full / prototype = sqrt(gR / K), not 1 / sqrt(K)', f.rpm, Math.sqrt(gR / K) - 1e-12, Math.sqrt(gR / K) + 1e-12, '', 4);
  within('speed ratio = sqrt(K gR)', f.speed, 1.160, 1.162, '', 4);
  const vProto = 300 * 2 * Math.PI / 60 * (ROVER.wheel.radius / K);   // 300 rpm no-load on the 0.065 m prototype wheel
  within('prototype no-load speed (m/s)', vProto, 2.03, 2.05, ' m/s', 3);
  within('full-scale no-load speed = prototype x sqrt(K gR) (m/s)', ROVER.motor.noLoadSpeed * ROVER.wheel.radius, 2.36, 2.38, ' m/s', 3);
  equal('SCALE in config.js is the froude() sheet', SCALE.rpm, f.rpm);
  check('power ratio = force x speed ratio', Math.abs(f.power - f.force * f.speed) < 1e-6 * f.power, `${f.power.toExponential(3)}`);
}
// the dock is sized at full scale (no x1172 from the 128 Wh bench pack)
{ const D = POWER_DOCK, arrayKW = D.arrays.length * D.panelArea * D.cellEff * D.arrayDerate * D.mpptEff * MOON.solarConstant / 1000;
  within('dock array (kW, from the drawn 1:K masts: 2 x 13.4 m^2)', arrayKW, 7, 10, ' kW', 2);
  equal('rover pack in the dock model is the 150 kWh pack', PACK.Wh, ROVER.battery.capacityWh);
  within('rover pack Ah x V = capacity', PACK.Ah * PACK.Vnom, 149999, 150001, ' Wh', 0);
  within('dock bank Ah x V = Wh', D.bankAh * D.bankV, D.bankWh - 1, D.bankWh + 1, ' Wh', 0);
  check('dock bank is not the bench-scaled 2.8 MWh', D.bankWh < 300000, `${D.bankWh / 1000} kWh`);
  const nightKWh = (D.hotelW + D.vault.G * (D.vault.setC - D.vault.sinkNightC) - D.vault.rhuW) * 354 / 1000 / D.bankEff;
  check('dock night reserve covers its own 354 h load (+ floor)', D.nightReserveSoc >= D.bankFloor + nightKWh / (D.bankWh / 1000), `reserve ${D.nightReserveSoc}, need ${(D.bankFloor + nightKWh / (D.bankWh / 1000)).toFixed(3)}`);
  check('prototype bench pack stays 128 Wh', PROTO_PACK.Wh === 128); }
const C = CARGO_TYPES.cyl40;
equal('cyl40 length (m)', C.length, 12.192);
equal('cyl40 diameter (m)', 2 * C.radius, 3.0);
check('cyl40 keeps shape / handling', C.shape === 'cylinder' && C.handling === 'straddle', `${C.shape} / ${C.handling}`);
within('cyl40 module prints at 1:K (length, m)', C.length / K, 1.49, 1.50, ' m', 3);

// mass
const M = ROVER.sprungMass + ROVER.corners.length * ROVER.unsprungMassPerBogie;
within('rover mass (sprung + 6 unsprung, kg)', M, 4500, 5500, ' kg', 0);
check('wheel mass <= unsprung corner mass', ROVER.wheel.mass <= ROVER.unsprungMassPerBogie, `${ROVER.wheel.mass} / ${ROVER.unsprungMassPerBogie} kg`);
within('CoM height (comLocal.y, m) ~ legacy x KS', ROVER.comLocal.y, 1.0, 1.3, ' m', 3);
within('roll inertia / (m r^2) plausibility (Ixx / (ms * halfTrack^2))', ROVER.inertia.x / (ROVER.sprungMass * Math.max(...ROVER.corners.map((c) => Math.abs(c[1]))) ** 2), 0.3, 1.5, '', 3);
within('wheel radius ~ legacy 1.05 m x KS', ROVER.wheel.radius / (1.05 * KS), 0.95, 1.05, '', 3);

// ride: four... six corner springs carry the sprung mass at ~1 Hz
const fRide = Math.sqrt(ROVER.susp.k * ROVER.corners.length / ROVER.sprungMass) / (2 * Math.PI);
within('heave ride frequency (Hz)', fRide, 0.85, 1.15, ' Hz', 3);
const zeta = ROVER.susp.c / (2 * Math.sqrt(ROVER.susp.k * ROVER.sprungMass / ROVER.corners.length));
within('heave damping ratio (legacy 0.72 kept)', zeta, 0.3, 0.8, '', 3);
const fHop = Math.sqrt(ROVER.wheel.tireK / ROVER.unsprungMassPerBogie) / (2 * Math.PI);
within('wheel-hop frequency (Hz) resolved at 120 Hz (omega dt <= 0.4)', 2 * Math.PI * fHop / 120, 0, 0.4, '', 3);
within('bump-stop damper c dt / m (explicit limit 2; implicit now)', ROVER.susp.bumpC / 120 / ROVER.unsprungMassPerBogie, 0, 3, '', 2);

// drive / braking: 4 of 6 wheels driven and braked
const nDriven = DRIVE.driven.length;
equal('driven wheels (4WD, outer)', nDriven, 4);
const aBrakeMax = 0.79 * g * nDriven / ROVER.corners.length;
within('accel.brakeMax = mu g x driven / all', ROVER.accel.brakeMax, aBrakeMax - 0.002, aBrakeMax + 0.002, ' m/s^2', 3);
within('accel.lon <= 0.75', ROVER.accel.lon, 0, 0.75, ' m/s^2', 3);
within('accel.lon x estopFactor <= brakeMax (0.853)', ROVER.accel.lon * ROVER.accel.estopFactor, 0, aBrakeMax + 0.002, ' m/s^2', 3);
within('planning brake decel (dynamics.brakeDecel, unloaded) <= brakeMax', brakeDecel(false), 0.6, 0.6, ' m/s^2', 2);
const Tbrake = ROVER.motor.brakeTorque * nDriven / ROVER.wheel.radius / M;
within('brake torque can reach brakeMax (4 wheels, m/s^2)', Tbrake, aBrakeMax, null, ' m/s^2', 3);
const Tdrive = ROVER.motor.peakTorque * nDriven / ROVER.wheel.radius / M;
within('peak drive torque: accel on flat (m/s^2) >= accel.lon', Tdrive, ROVER.accel.lon, null, ' m/s^2', 3);

// power
within('surface charger cap dockMaxW (W)', ROVER.battery.dockMaxW, 10000, 20000, ' W', 0);
within('charger C-rate at cap', ROVER.battery.dockMaxW / ROVER.battery.capacityWh, 0, 0.2, ' C', 3);
