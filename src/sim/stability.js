// Loaded stability of the rover, closed form (review item 1). The vehicle used to carry a 14.9 t module with its centre of gravity
// 4.5 m up on a 3.9 x 3.8 m wheel footprint: static tip angle about 28 deg, which the autonomy answered with a 14 deg tilt stop
// (the cause of the last M6 failure). This module states the tip angle from the geometry, on the design slope (not on flat ground),
// and derives the loaded slope limits and the tilt reflex from it, so that changing the track, the wheelbase or the cradle height
// moves the limits with it. rover.js _stability measures the live margin from the wheel contacts (same definition: outer tyre edges).
import { ROVER, CARGO_TYPES, MOON, DESIGN } from './config.js';

const DEG = 180 / Math.PI;

// Geometry of the loaded vehicle. All lengths in m, body frame y = 0 on the ground at nominal ride.
//   halfTrack  wheel-centre half track (z of the corner wheels)   halfBase  half wheelbase (x of the front / rear wheels)
//   cradleY    height of the module's lowest point above the ground when carried (drive ride)
//   extraKg / extraY  any other mass (hook-lift arm, feet) and its height
export function geometry(over = {}) {
  const zs = ROVER.corners.map((c) => Math.abs(c[1])), xs = ROVER.corners.map((c) => Math.abs(c[0]));
  const cargo = over.cargo ?? CARGO_TYPES.cyl40;
  return {
    halfTrack: Math.max(...zs), halfBase: Math.max(...xs), tyreHalfW: ROVER.wheel.width / 2,
    wheelR: ROVER.wheel.radius, sprungKg: ROVER.sprungMass, sprungY: ROVER.comLocal.y,
    unsprungKg: ROVER.corners.length * ROVER.unsprungMassPerBogie,
    cargoKg: cargo.mass, cargoRadius: cargo.shape === 'cylinder' ? cargo.radius : cargo.height / 2 + 0.1,
    cradleY: ROVER.seatY + ROVER.ride.carry,
    extraKg: 0, extraY: 0, cargoX: 0,
    ...over,
  };
}

// Combined centre of gravity of rover + carried module (+ extra), the sprung body at its own CoM, the six axles at wheel height.
export function combinedCG(g) {
  const cargoY = g.cradleY + g.cargoRadius;
  const m = g.sprungKg + g.unsprungKg + g.cargoKg + g.extraKg;
  const y = (g.sprungKg * g.sprungY + g.unsprungKg * g.wheelR + g.cargoKg * cargoY + g.extraKg * g.extraY) / m;
  const x = g.cargoKg * g.cargoX / m;
  return { m, x, y, cargoY };
}

// Static tip angles on level ground (deg): the slope at which the vehicle starts to roll over about the stated edge.
//   rollCL  about the wheel centre line (conservative: the tyre can sink and bulldoze at its outer edge)
//   rollEdge about the outer tyre edge (what rover.js reports as tilt.margin)
//   pitch   nose-down / nose-up about the front / rear contact line (uses the wheel centre fore and aft)
export function tipStatic(g) {
  const c = combinedCG(g);
  return {
    cg: c,
    rollCL: Math.atan2(g.halfTrack, c.y) * DEG,
    rollEdge: Math.atan2(g.halfTrack + g.tyreHalfW, c.y) * DEG,
    pitchFwd: Math.atan2(g.halfBase - c.x, c.y) * DEG,   // nose-down slope: tips about the front axle
    pitchAft: Math.atan2(g.halfBase + c.x, c.y) * DEG,
    pitch: Math.atan2(g.halfBase - Math.abs(c.x), c.y) * DEG,
  };
}

// Lean the manoeuvre adds: lateral acceleration a on a body at lunar gravity leans the resultant by atan(a / g)
export const leanDeg = (aLat, gMoon = MOON.g) => Math.atan(aLat / gMoon) * DEG;

// What is left on the steepest loaded design slope, in the worst direction (cross-slope for roll, up/down for pitch).
export function marginOnSlope(g, slopeDeg = DESIGN.slopeLoadedMaxDeg, aLat = ROVER.accel.latLoaded) {
  const t = tipStatic(g), worst = Math.min(t.rollCL, t.pitch);
  return { ...t, worst, slopeDeg, onSlope: worst - slopeDeg, afterLean: worst - slopeDeg - leanDeg(aLat) };
}

// Loaded limits derived from the static tip angle (replaces the hand-set 10.5 / 6 deg slopes and the 14 deg tilt reflex, which
// were tuned for ~29 deg: slopeLethal = tip - 18.5, reflex = slope + 3.5).
export function loadedLimits(tipDeg) {
  const slopeLethal = Math.min(DESIGN.slopeLoadedMaxDeg, Math.max(6, +(tipDeg - DESIGN.slopeMarginDeg).toFixed(1)));
  return {
    slopeLethal, slopeCaution: +(slopeLethal * DESIGN.cautionFrac).toFixed(1),
    tiltReflex: Math.min(DESIGN.emptyReflexDeg, +(slopeLethal + DESIGN.reflexOverDeg).toFixed(1)),
  };
}

// Approach / departure angle of the module's ends (the module overhangs the wheelbase by (L - 2 halfBase) / 2 at each end)
export function endAngleDeg(g, lengthM = CARGO_TYPES.cyl40.length) {
  const overhang = Math.max(0.1, lengthM / 2 - g.halfBase);
  return Math.atan2(g.cradleY, overhang) * DEG;
}

// The sim's geometry as configured now (config.js): what the autonomy limits derive from, so they follow the hardware
export const actualGeometry = () => geometry();
export const actualTip = () => marginOnSlope(actualGeometry());
export const actualLimits = () => { const t = actualTip(); return loadedLimits(Math.min(t.rollCL, t.pitch)); };
// The trade study's target (DESIGN.target: the module lowered to the cradle height of docs/STABILITY_TRADE.md)
export const targetGeometry = () => geometry(DESIGN.target || {});
export const targetTip = () => marginOnSlope(targetGeometry());
