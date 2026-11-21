// Lunar south-pole environment constants used by the thermal, mechanism, camera-lighting and comms models
// (docs/changes/B.md). Kept out of config.js on purpose: config.js holds the vehicle; this file holds the Moon.
// Values without a source are [ASSUMED].

export const SOLAR = {
  S0: 1361,              // W/m^2 solar constant at 1 AU (no atmosphere)
};

// Radiator / hub-face solar absorptivity (alpha). White radiator paint or OSR starts near 0.10; a thin regolith film
// raises alpha (dust is dark, alpha ~0.9 in bulk); emissivity barely changes (it stays ~0.85-0.9). [ASSUMED]
//   clean 0.10, dusty 0.30 (light film after a drive), fins 0.50 (the "dust-fins" thermal scenario, heavy coat)
export const RAD_ALPHA = { clean: 0.10, dusty: 0.30, fins: 0.50 };

// Radiative sinks for exposed mechanisms (arm joints, latches, roller-screw struts).
//   skyK   deep space;  regNightK south-pole regolith at night / in shadow (Diviner ~50-100 K);  viewSky fraction of the
//   hemisphere that is sky for a vertical / side-facing surface.
export const SINK = { skyK: 3, regNightK: 90, viewSky: 0.5 };

// [ASSUMED] Lumped thermal node of an exposed mechanism. The grease in the gearboxes / roller screws (PFPE, the same as
// the wheel hubs, THERMAL.hubFreezeC) sets the motion lockout at -65 C. A thermostatic film heater holds each node above
// heaterOnC (off heaterHystK higher).
//   C J/K;  epsA m^2 effective emittance x area (MLI-wrapped);  alphaA m^2 solar absorptance x projected area
export const MECH = {
  lockC: -65,
  joint: { C: 18000, epsA: 0.04, alphaA: 0.010, heaterW: 15, heaterOnC: -45, heaterHystK: 5, T0: 20 },   // shoulder / elbow / wrist gearbox
  latch: { C: 2500, epsA: 0.010, alphaA: 0.003, heaterW: 10, heaterOnC: -45, heaterHystK: 5, T0: 20 },   // hook gate / deck latch servo
  strut: { C: 25000, epsA: 0.05, alphaA: 0.012, heaterW: 25, heaterOnC: -45, heaterHystK: 5, T0: 10 },   // roller-screw strut cartridge
};

// Camera lighting (sensors.js): the low polar sun (~9 deg) throws long shadows; a surface not lit by the sun is seen
// only in the rover's own lamps. glareDeg: nothing within this cone of the sun can be imaged. shadowRayM: length of the
// shadow ray cast from each hit toward the sun (local terrain / rocks; far ridges are covered by the rover's own
// sunlit test).
export const LIGHT = { lampRangeM: 35, glareDeg: 12, shadowRayM: 120 };

// Operator link [ASSUMED]. Earth: 1.28 s light time each way + DSN / ground-segment latency -> 2.6 s one way, so a
// command and its telemetry reply take >= 5 s. Earth from the south pole sits near the horizon at a fixed azimuth
// (librations move it a few degrees; ignored). The habitat relay antenna is on a mast at the habitat.
export const COMMS = {
  earthDelayS: 2.6,
  earthAzDeg: 0, earthElDeg: 10,       // sim azimuth convention (sim.setSun): from +X toward -Z
  mastH: 5.0,                          // m, rover antenna above the body origin (sensor bar / avionics pod)
  habitatAntennaH: 10,                 // m above the terrain at the habitat
  earthRayM: 3000,                     // m, terrain ray length toward Earth
  checkS: 1,                           // s between LOS checks
  persist: 2,                          // consecutive blocked checks before COMMS LOS is raised
};
