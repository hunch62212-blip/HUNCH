// Navigation recover-and-retry (v1.7.0, autonomy.js navTo / _retryPrep).
// Pass rules: a leg that gives up with a navigation failure is re-tried after an autonomous recovery, at most NAV.legRetries
// times; the obstacle marks the failed attempt learned are kept for the retry; a failed remembered route is retried as a fresh
// exploration; a non-navigation failure (no path at all) is not retried; the drive to the charger gets one retry only.
// The legs themselves are stubbed (_navLeg), so this checks the executive logic, not the terrain.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { NAV } from '../src/sim/config.js';
import { check, equal } from './lib/check.mjs';
await RAPIER.init();

const sim = new Sim(RAPIER), A = sim.autonomy;
const logs = []; sim.on('log', (e) => logs.push(e.msg));
for (let i = 0; i < 240; i++) sim.stepOnce(); // settle on the pad

// run a navTo to completion inside a mission (retries only happen while a mission is running)
function runNav(script, goal = { x: 0, z: 0, name: 'test goal' }) {
  const calls = [];
  A._navLeg = function* (g, opts) {
    const k = calls.length;
    calls.push({ opts: { ...opts }, blockedAtStart: A.planner.blocked.map((b) => b.tag).filter(Boolean) });
    yield;
    return script(k);
  };
  let res;
  A.start('nav_retry', (function* () { res = yield* A.navTo(goal); return res; })());
  for (let i = 0; i < 120 * 600 && A.gen; i++) sim.stepOnce();
  return { res, calls };
}
const navFail = (why, extra = {}) => ({ ok: false, why, retry: true, blocks: [], usedRoute: false, ...extra });

{ // fail once (boxed in on a remembered route), then arrive
  const t = sim.time;
  const { res, calls } = runNav((k) => (k === 0 ? navFail('boxed in by obstacles: operator assistance required', { usedRoute: true, blocks: [{ x: 50, z: 50, r: 3, t, ttl: 1200, tag: 'learned' }] }) : { ok: true }));
  check('retry: leg that boxed in is retried and arrives', res && res.ok === true, `ok=${res && res.ok}`);
  equal('retry: attempts used', calls.length, 2);
  check('retry: obstacle learned by the failed attempt is kept for the retry', calls[1] && calls[1].blockedAtStart.includes('learned'));
  check('retry: a failed remembered route is retried as fresh exploration', calls[1] && calls[1].opts.noMemory === true);
  check('retry: reported in the log', logs.some((m) => /Autonomous recovery and retry 1\/\d/.test(m)));
  check('retry: counted in the mission metrics', (A.history.at(-1)?.ok === true) && A.metrics.retries === 1, `retries ${A.metrics.retries}`);
}
{ // never gets there: gives up after NAV.legRetries retries
  const { res, calls } = runNav(() => navFail('no progress toward test goal for 4 min: operator assistance required'));
  equal('retry: attempts before asking the operator', calls.length, 1 + NAV.legRetries);
  check('retry: then the leg fails with the reason', res && res.ok === false && /no progress/.test(res.why));
  check('retry: an exploration that failed gets the remembered route back', calls[1] && calls[1].opts.noMemory === false);
}
{ // not a navigation failure: no retry
  const { calls } = runNav(() => ({ ok: false, why: 'no path to goal (check geofences / goal location)' }));
  equal('retry: no-path failure is not retried', calls.length, 1);
}
{ // drive to the charger: one retry only
  const { calls } = runNav(() => navFail('moving away from power dock entry'), { x: 0, z: 0, name: 'power dock entry', toCharger: true });
  equal('retry: drive to the charger is retried once', calls.length, 2);
}

{ // unwind trace after an earlier unwind (v1.7.0): never leads back into the first pocket
  // drove east from x=0 to the pocket at x=60 (t 0..60), unwound west to x=35 (t 60..85), drove east again to x=58 (t 85..108)
  const T = [];
  for (let x = 0, t = 0; x <= 60; x += 2, t += 2) T.push({ x, z: 0, t });
  for (let x = 58, t = 62; x >= 35; x -= 2, t += 2) T.push({ x, z: 0.5, t });
  for (let x = 37, t = 88; x <= 58; x += 2, t += 2) T.push({ x, z: 1, t });
  const S = sim.slam, keep = S.trajectory;
  S.trajectory = T;
  A._unwindSpans = [{ t0: 60, t1: 85, end: { x: 35, z: 0.5 } }];
  const { trace } = A._unwindTrace({ x: 58, z: 1 }, 50, 110);
  check('unwind trace: never turns back east into the first pocket', trace.every((p, i) => i === 0 || p.x <= trace[i - 1].x + 0.01), trace.map((p) => p.x.toFixed(0)).join(' '));
  check('unwind trace: continues west past the first unwind end along the old entry path', trace.at(-1).x < 30, `ends at x ${trace.at(-1).x.toFixed(0)}`);
  A._unwindSpans = []; S.trajectory = keep;
}
