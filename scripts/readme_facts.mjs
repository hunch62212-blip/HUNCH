// Prints every README spec number that is derived from config or data files, so the README can be checked:
//   node scripts/readme_facts.mjs
// No physics is run. Values are full scale (team 1:16.3 prototype x K).
import { readFileSync } from 'node:fs';
import { ROVER, CARGO_TYPES, DOCK, MOON, REGOLITH, WORLD, K } from '../src/sim/config.js';

const r = ROVER.wheel.radius, M = ROVER.motor, c = ROVER.corners;
const ms = (rpm) => (rpm * 2 * Math.PI / 60 * r).toFixed(2);
const rows = [
  ['vehicle name', ROVER.name],
  ['scale factor K', K.toFixed(3)],
  ['wheels (corners)', c.length],
  ['wheel diameter x width (m)', `${(2 * r).toFixed(2)} x ${ROVER.wheel.width.toFixed(2)}`],
  ['track, wheel centres (m)', (2 * Math.abs(c[0][1])).toFixed(2)],
  ['wheelbase, front-rear (m)', (2 * c[0][0]).toFixed(2)],
  ['frame L x W (m)', `${(2 * ROVER.frame.halfLen).toFixed(2)} x ${(2 * ROVER.frame.halfWid).toFixed(2)}`],
  ['mass, empty (kg)', ROVER.sprungMass + c.length * ROVER.unsprungMassPerBogie],
  ['  sprung + unsprung per corner (kg)', `${ROVER.sprungMass} + ${c.length} x ${ROVER.unsprungMassPerBogie}`],
  ['steering', ROVER.steer.skid ? `skid steer (maxAngle ${ROVER.steer.maxAngle})` : `+-${ROVER.steer.maxAngle} deg`],
  ['wheel speed cap / turbo / crawl (rpm)', `${M.capRpmWheel} / ${M.turboRpmWheel} / ${M.crawlRpmWheel}`],
  ['ground speed at cap / turbo / crawl (m/s)', `${ms(M.capRpmWheel)} / ${ms(M.turboRpmWheel)} / ${ms(M.crawlRpmWheel)}`],
  ['speed limit, empty / loaded (m/s)', `${ROVER.speedLimitEmpty} / ${ROVER.speedLimitLoaded}`],
  ['peak wheel torque (N m)', M.peakTorque],
  ['battery (kWh, V)', `${ROVER.battery.capacityWh / 1000}, ${ROVER.battery.nominalV}`],
  ['on-board solar panels', `${ROVER.solar.panels} x ${ROVER.solar.panelArea} m2`],
  ['hoist travel / docking stage (m)', `${ROVER.lift.travel} / ${ROVER.lift.stageMax}`],
  ['ride kneel / carry / max (m)', `${ROVER.ride.kneel} / ${ROVER.ride.carry} / ${ROVER.ride.max}`],
  ['climb limit, edge height (m)', ROVER.climbMax.toFixed(3)],
  ['LiDAR channels, tilt (deg), range (m), rate (Hz)', `${ROVER.lidar.channels}, ${ROVER.lidar.tiltDeg}, ${ROVER.lidar.range}, ${ROVER.lidar.rateHz}`],
  ['camera HFOV (deg)', ROVER.camera.hFovDeg],
  ['cyl40 module L x D (m), mass (kg)', `${CARGO_TYPES.cyl40.length.toFixed(2)} x ${(2 * CARGO_TYPES.cyl40.radius).toFixed(2)}, ${CARGO_TYPES.cyl40.mass}`],
  ['box20 L x W x H (m), mass (kg)', `${CARGO_TYPES.box20.length} x ${CARGO_TYPES.box20.width} x ${CARGO_TYPES.box20.height}, ${CARGO_TYPES.box20.mass}`],
  ['dock port centre height (m)', DOCK.portCenterH],
  ['pad-habitat distance (km)', (Math.hypot(WORLD.habitat.x - WORLD.pad.x, WORLD.habitat.z - WORLD.pad.z) / 1000).toFixed(3)],
  ['lunar g (m/s2)', MOON.g],
  ['regolith n, kc, kphi, c, phi, K', `${REGOLITH.n}, ${REGOLITH.kc / 1000} kN/m2, ${REGOLITH.kphi / 1000} kN/m3, ${REGOLITH.c / 1000} kPa, ${(REGOLITH.phi * 180 / Math.PI).toFixed(0)} deg, ${(REGOLITH.K * 100).toFixed(1)} cm`],
];
const man = JSON.parse(readFileSync(new URL('../src/sim/lunar_logistics_manifest.json', import.meta.url)));
const items = man.manifest.reduce((a, m) => a + (m.Quantity ?? 1), 0);
const mass = man.manifest.reduce((a, m) => a + m.Mass_kg * (m.Quantity ?? 1), 0);
rows.push(['P1 manifest lines / items (recomputed)', `${man.manifest.length} / ${items} (file header says ${man.total_cargo_items})`]);
rows.push(['P1 manifest mass kg (recomputed)', `${mass.toFixed(1)} (file header says ${man.total_payload_mass_kg})`]);
for (const [k, v] of rows) console.log(`${k.padEnd(48)} ${v}`);
