# Implementer C: dock capture, timestep, lunar lighting, quality tiers

## Changed (C-owned files)
- `src/sim/autonomy.js:18-22` `DOCK_CAPTURE_MAX_ANGLE` (0.25 deg), `DOCK_LEAK_CHECK_S` (1800 s), `dockAngles()`.
  - The soft-capture gate (~2015) now needs |yaw| and |pitch| < 0.25 deg. Refusals are counted in `dockAngleRefusals`.
  - Yaw squaring now also runs near the limit.
  - The leak check uses `this.leakCheckS` (configurable, also through `new Sim(R, {leakCheckS})`) and runs on the quiescent-hold warp (2054).
- `src/sim/sim.js` changes:
  - `dockCapture` (205) refuses a weld outside the angle envelope and returns `{ok, why, yaw, pitch}`.
  - Bellows contact is split over `BELLOWS_POINTS = 4` rim points, each a k/N, c/N spring-damper with its own point velocity.
  - `_holdActive` and `_holdStep` (294-303) add the quiescent hold: 0.5 s steps with no solver, while the idle power, modules and comms still run. A 30 min leak check costs about 3600 cheap steps.
  - `railsStep` (449) gives a fixed rails step of round(scale/60 / dt) physics steps, clamped to warp.H.
  - `advance` uses `railsStep` instead of the wall-clock backlog.
- `src/sim/world.js:44` sets `world.timestep = PHYS.dt` and throws if the two differ.
- `src/render/scene.js` changes:
  - The sky and hemisphere are black.
  - Fill is at most 1% of sunlight, as warm bounce from below.
  - Earthshine is 1e-4 of sunlight, from Earth's direction, scaled by Earth's lit fraction.
  - Earth sits at az 314 deg, el 10 deg, with ±6.7/7.9 deg libration. Its phase comes from the sun and Earth geometry in a shader.
  - Night exposure is 1.05 to 1.40.
  - Ultra: 4096 shadow maps on the near cascades (capped at maxTextureSize), SMAA always on, anisotropy at GPU max. No new post-processing passes.
- `src/render/effects.js:16` dust keeps alpha 0.9 until it lands (flight capped at 12 s).
- Tests:
  - `tests/unit_timestep.mjs` (new).
  - `tests/unit_dock_capture.mjs` (new): 1 deg yaw injected on final approach.
  - Both are registered in `tests/run-all.mjs`.

## Requests to A (config.js / rover.js)
- `config.js:463` `PHYS.dt` is the single source. `rover.js:21` `ROVER_DT` should import `PHYS.dt` and not keep its own 1/120.
- **Blocker:** the in-progress config.js edit dropped `label`, `shape: 'cylinder'` and `handling` from `CARGO_TYPES.cyl40`. As a result, `world.js:218` throws `T.adapterRings is not iterable` and every Sim-based test fails. C tested against a scratch copy with those 3 lines restored.
- Optional: move `DOCK_CAPTURE_MAX_ANGLE` and `DOCK_LEAK_CHECK_S` into `DOCK` in config.js and tag them [ASSUMED]. Until then they live in autonomy.js.

## Request to B (sensors.js)
- Add `ring.pitch` (the module axis elevation relative to the port, with noise like `ring.yaw`) at sensors.js:532. Until then autonomy falls back to the body truth for pitch.

## CHANGES.md entries (owner of CHANGES.md)
- Capture refuses at 0.25 deg yaw/pitch. Bellows use 4 rim points. Leak check is 30 min on the hold warp.
- Rapier step comes from PHYS.dt. The rails step is fixed per scale.
- Lunar lighting: black sky, 1% bounce, earthshine 1e-4, Earth fixed with phase from geometry.
- Ultra quality: 4096 near-cascade shadows, max anisotropy, SMAA always on.

## Notes
- `tests/v17_suite.mjs` section 5 calls `stepAuto(owed)` directly. That is still supported, since only `advance()` switched to `railsStep()`.
- The mission takes about 1560 s longer in sim time, because the leak check went from 240 s to 1800 s. Tests that cap sim time may need more headroom.

## Docs only (not implemented)
- Floating origin (`_precisionPost`): rebase the world when the rover is more than 2 km from the origin. Float32 gives about 61 um ulp at 800 m.
- Larger terrain and albedo textures.
- SSAO: needs a post pass, which the brief forbids.
- Rover and dock redesign.

## Test results (2026-11-21)
- `unit_timestep`: 18/18 PASS.
- `unit_dock_capture`:
  - PASS: latches refuse a module at 1 deg and weld one at 0.1 deg.
  - PASS: with 1 deg injected on final approach and held for 90 s, there were 0 welds.
  - Not verified: the module did not latch afterwards, failing after 3 attempts at 862 s, so the leak-check timing assert never ran.
- `test:physics` 15/17: "kneel lowers body" (0.32 m) and "carry ride raises body" (0.40 m) fail. They fail the same way with the HEAD sim files, so these are not from C's changes (likely A's rover/config work).
- `dock.mjs 0` (cyl40, scratch copy with the cyl40 lines restored):
  - Capture on attempt 3 at 792 s: lat 3.6 mm, vert 0.3 mm, yaw 0.02 deg, peak 827 N.
  - Baseline was attempt 1 at 229 s.
  - Isolation runs with the 4-point bellows, the squaring change, the angle gate and the hold warp all reverted still fail attempt 1 (capture on attempt 2 at 537 s).
  - The cause is outside those changes. The gap holds at about 0.043 m at about 500 N against captureGap 0.036, so capture depends on transient dips.
- `dock.mjs 0` also fails the leak/hatch checks, because the test caps at `sim.time < 2400` while the leak check is now 1800 s. **Request:** raise the cap in tests/dock.mjs:45 to 4800 (not a C file).
- `dock.mjs 1` (pod, PORT-2): **BLOCKER.**
  - The module sits 0.71-0.81 deg nose-pitched relative to the port with yaw at about 0.
  - The 0.25 deg pitch gate refuses every latch, so the delivery fails.
  - The rover has no cradle pitch trim. Needs A (a cradle pitch levelling actuator in rover.js/acmCarry.js) or a coordinator decision.
