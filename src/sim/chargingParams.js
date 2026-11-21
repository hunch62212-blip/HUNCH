// AEGIS-V1 lunar charging interface (docs/CHARGING_DESIGN.md). Chosen design: a SEGMENTED INDUCTIVE PAD at the power dock
// (sealed transmitter modules flush in a sintered-regolith slab, no exposed contacts) and a belly-mounted receiver ("vehicle
// assembly", VA) on the rover. The rover centres on the pad on its fiducial + pilot-coil read-out, kneels to close the air gap and
// charges at up to the 20 kW charger cap. Parked on the pad through the lunar night it stays coupled and the dock carries its
// survival load from the dock bank ("night park"), so the 150 kWh pack starts the next day full.
//
// Units: geometry is FULL-SCALE metres (K = 8.16, body frame +X forward, +Y up, +Z right, origin on the ground plane at ride 0,
// x = 0 at the middle wheel axle); powers marked "full-scale" are rover-side W. The dock bank / solar model in powerDock.js
// stays in PROTOTYPE units (24 V 100 Ah bank, POWER_DOCK in config.js) and is coupled through PROTO_TO_FULL_WH as before.
// Provenance: [SRC] value traced to docs/CHARGING_DESIGN.md sources; [ASSUMED] engineering estimate, open.
import { ROVER, K } from './config.js';

const kp = (mm) => +(mm / 1000 * K).toFixed(3); // prototype mm -> full-scale m (the CAD team works on the 1:8.16 print)

export const CHARGING = {
  type: 'inductive, segmented ground pad (Astrobotic/WiBotic lunar UFPC heritage, SAE J2954-class coils)',
  maxW: ROVER.battery.dockMaxW,            // 20 kW full-scale charger cap (USER DECISION, config.js); the link never exceeds it

  // ---- vehicle assembly (receiver) on the rover belly. Body frame, full scale. [ASSUMED] layout, sized for 20 kW at ~2 kW/m^2
  // of coil face (J2954 WPT3 11 kW pads are ~0.5 x 0.5 m; 0.90 m square carries 20 kW with margin and a 0.3 m flux overlap).
  rx: {
    x: 0, z: 0,                            // coil centre: belly centreline, over the middle-wheel axle (CG, clear of the nose avionics vault)
    yFace: kp(61),                         // 0.498 m: receiver face above the ground plane at ride 0, 32 mm below the belly tub bottom
                                           // (k(0.065) = 0.530 m); the housing is recessed 50 mm into the tub. Kneeled (ride -0.33 m,
                                           // measured body sag -0.364 m) the face sits ~0.134 m above the ground: gap ~77 mm to the pad
    lx: kp(110), lz: kp(110), t: kp(10),   // 0.898 x 0.898 x 0.082 m housing (prototype 110 x 110 x 10 mm)
    massKg: 55,                            // [ASSUMED] litz coil + ferrite tiles + Al shield + rectifier (ORNL 50 kW 3-phase VA class)
  },

  // ---- ground assembly: 3 x 2 transmitter modules in a sintered-regolith slab; only the modules under the receiver are driven
  ga: {
    along: 3, across: 2, module: kp(98),   // 0.80 m square modules -> 2.40 m (along the parked heading) x 1.60 m (across) active area
    moduleMaxW: 7000,                      // [ASSUMED] per module; 3 modules under the 0.9 m receiver carry 20 kW
    topH: kp(7),                           // 0.057 m: pad cover top above the slab datum (= terrain height at the pad centre)
    coverT: 0.015,                         // [ASSUMED] glass-ceramic / alumina cover over the coils (RF transparent, abrasion resistant)
    slab: { along: 7.0, across: 5.0, t: 0.15 }, // [ASSUMED] sintered-regolith pad slab (no loose dust under the rover)
  },

  // ---- capture tolerances (receiver centre vs pad centre, in the pad frame; heading mod 180 deg, the rover may park either way)
  //   full power inside (along, across); the coupling falls off to zero at (alongMax, acrossMax): the GA cannot drive rated
  //   current into a receiver that only half covers a module. [ASSUMED] from the 0.9 m receiver on the 2.4 x 1.6 m active area:
  //   full overlap while |along| <= (2.40 - 0.90) / 2 = 0.75 and |across| <= (1.60 - 0.90) / 2 = 0.35.
  tol: { along: 0.75, alongMax: 1.00, across: 0.35, acrossMax: 0.60, yawDeg: 8, speed: 0.08 },
  // air gap = receiver face height above the pad cover. At ride 0 it is ~0.41 m (no link: too weak); kneeling closes it to ~77 mm.
  // Measured autonomous parking (3 starts, scratch probe): |along| <= 0.21 m, |across| <= 0.28 m, |yaw| <= 2.5 deg: inside the
  // full-power box with margin.
  gap: { min: 0.0, nom: 0.075, max: 0.22 }, // [ASSUMED] J2954 Z1-Z3 classes cover 100-250 mm clearance

  // ---- link efficiency, DC (dock bus) to DC (rover bus). [SRC] ORNL 50 kW 3-phase WPT 95 %; J2954 WPT3 > 90 % at 250 mm.
  // centre 0.92 after a lunar derate for the extra DC-DC stage and hot-case operation. [ASSUMED] offset and gap losses.
  eta: { centre: 0.92, edgeLoss: 0.05, gapLossPerM: 0.25 },

  // ---- thermal path (full-scale W). The link loss is split between the receiver (rover side) and the pad + inverter (dock side).
  //   rx: the receiver housing is bolted to the belly tub with a heat-pipe spreader into the battery loop: G W/K to the pack
  //       (health.th.Tb). Its loss heat is useful at night (it warms the pack it charges). Charge derates above derateC, stops at maxC.
  //   ga: the pad modules sit under the rover (no sky view while charging): they reject into the slab / regolith (G W/K to sink).
  thermal: {
    rx: { C: 50000, G: 40, share: 0.45, derateC: 90, maxC: 110, initC: 10 },   // [ASSUMED] 55 kg of Al / ferrite / Cu
    ga: { C: 180000, G: 30, share: 0.55, derateC: 90, maxC: 110, initC: -10, sinkDayC: 0, sinkNightC: -60 }, // [ASSUMED] no GA heater: coils and
    //     ferrite run cold; the inverter lives in the heated battery vault
  },

  // ---- night park: parked and coupled with no sun, the dock carries the rover's bus load (keep-alive) from its bank down to
  // keepAliveFloorSoc (below the 50 % night reserve that limits CHARGING at night), capped at keepAliveMaxW full-scale.
  // Budget (docs/CHARGING_DESIGN.md s5): rover survival ~154 W full-scale = 0.13 W prototype / 0.92 link = 0.14 W, + the dock's own
  // 2.0 W night load + the pad pilot / foreign-object check duty-cycled to pilotDuty of POWER_DOCK.padStandbyW (0.1 W: the rover is
  // kneeled and braked, a 1 Hz handshake is enough), x 354 h = 794 Wh at the bus, 819 Wh from the cells = 34 % of the 2.4 kWh
  // bank: a 50 % reserve ends the night at ~16 %, above the 12 % keep-alive floor and the 10 % BMS floor.
  night: { keepAliveMaxW: 600, keepAliveFloorSoc: 0.12, pilotDuty: 0.2 }, // [ASSUMED]
};

// Capture envelope helper shared by the sim and the render: pad-frame offsets of the receiver centre from the pad centre.
export function padFrame(dx, dz, padYaw) {
  // heading unit (cos yaw, -sin yaw), right unit (sin yaw, cos yaw) in world x, z (rover yaw convention, rover.js pose)
  const c = Math.cos(padYaw), s = Math.sin(padYaw);
  return { along: dx * c - dz * s, across: dx * s + dz * c };
}
