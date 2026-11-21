// One scaling sheet: prototype (Earth bench, geometric 1:K) -> full scale (Moon). Review item 6: wheel rpm was divided by
// sqrt(K) alone, which keeps the Froude number only if gravity is the same at both scales. The prototype runs in Earth gravity,
// the rover in lunar gravity, so every dynamic quantity carries gR = g_moon / g_earth.
//
// Froude similarity (v^2 / (g L) equal at both scales), lambda = K (length ratio), gR = g_full / g_proto:
//   length       x K                  mass (same density)  x K^3            acceleration  x gR
//   speed        x sqrt(K gR)         time                 x sqrt(K / gR)
//   angular rate x sqrt(gR / K)       (wheel rpm of the full-scale wheel = prototype rpm x sqrt(gR / K))
//   force        x K^3 gR             torque x K^4 gR      energy x K^4 gR
//   power        x K^3 gR sqrt(K gR)  pressure / stress x K gR
// Electrical and thermal quantities do NOT scale this way: a battery, a radiator or a solar array is sized for the full-scale
// job (docs/SCALING.md, POWER_DOCK in config.js), not multiplied up from the bench part.
export const G_EARTH = 9.80665;

export function froude(K, gFull, gProto = G_EARTH) {
  const gR = gFull / gProto;
  const speed = Math.sqrt(K * gR);
  return {
    K, gR, gFull, gProto,
    length: K, mass: K ** 3, accel: gR,
    speed, time: Math.sqrt(K / gR), omega: Math.sqrt(gR / K),
    rpm: Math.sqrt(gR / K),                // wheel rpm ratio full / prototype
    force: K ** 3 * gR, torque: K ** 4 * gR, energy: K ** 4 * gR,
    power: K ** 3 * gR * speed, stress: K * gR,
  };
}
