// Local planner on a perfect plane: at which uphill slope does it stop finding any safe arc?
// (A plane has no steps; the empty rover is rated to 20 deg, loaded to 8.5 deg.)
import { LocalPlanner } from '../../src/sim/planner.js';
const mk = (deg, loaded) => {
  const g = Math.tan(deg * Math.PI / 180);
  const limits = loaded ? { slopeLethal: 8.5, slopeCaution: 4.5, stepLethal: 0.6, stepCaution: 0.3 } : { slopeLethal: 20, slopeCaution: 9, stepLethal: 0.85, stepCaution: 0.4 };
  const heightAt = (x) => x * g; // uphill toward +x (rover heading yaw 0)
  return { limits, heightAt, wheelHazard(x, z) { const h = heightAt(x); const slope = g; return { h, lethal: slope > Math.tan(limits.slopeLethal * Math.PI / 180) * 1.05, cost: 1 }; } };
};
for (const loaded of [false, true]) for (const deg of [0, 4, 6, 8, 9, 10, 11, 12, 14, 16, 18]) {
  const lp = new LocalPlanner(mk(deg, loaded));
  const r = lp.evaluate({ x: 0, z: 0, yaw: 0 }, { x: 40, z: 0 }, 2, loaded);
  console.log(`${loaded ? 'loaded' : 'empty '} uphill ${String(deg).padStart(2)} deg: ${r.ok ? 'drives' : 'BLOCKED'}  (${r.arcs.filter((a) => a.ok).length}/${r.arcs.length} arcs ok)`);
}
// a real belly obstacle must still block: a 1.0 m high, 1.5 m wide mound between the wheel tracks, 8 m ahead,
// on flat ground and on a 10 deg uphill
for (const deg of [0, 10]) {
  const S = mk(deg, false), base = S.heightAt;
  S.heightAt = (x, z) => base(x) + (Math.abs(x - 8) < 0.75 && Math.abs(z) < 0.75 ? 1.0 : 0);
  const r = new LocalPlanner(S).evaluate({ x: 0, z: 0, yaw: 0 }, { x: 40, z: 0 }, 2, false);
  const straight = r.arcs[Math.floor(r.arcs.length / 2)];
  console.log(`1.0 m mound under the belly, ${deg} deg uphill: straight arc ${straight.ok ? 'ok' : 'blocked'} (free ${straight.freeDist} m)`);
}
