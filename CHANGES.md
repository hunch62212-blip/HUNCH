# CHANGES — docs/config/tests verification pass (2026-11-06)

Baseline: git commit `f4e7c18` (the tree as found). All changes below are uncommitted.
**No physics constant or model behaviour was changed by this pass.** One input-validation fix (fault wheel range)
and log-text fixes are listed in §2.

## 1. Vehicle identity: README vs config (at `f4e7c18`)

Source of truth chosen: **config / code (AEGIS-V1)**. Reason: the simulator, HUD, page title, rover model,
autonomy and team CAD are all AEGIS-V1. Aligning the code to SPAN-40 would mean replacing the vehicle model.
README rewritten to match; the numbers below come from `node scripts/readme_facts.mjs`.

| Item | README (SPAN-40) | file:line | Config / code (AEGIS-V1) | file:line | Resolution |
|---|---|---|---|---|---|
| Name | SPAN-40 | README.md:1, :122 | AEGIS-V1 | src/sim/config.js:39 | README → AEGIS-V1 |
| Page title | (SPAN-40 implied) | — | AEGIS-V1 … Simulator | index.html:6 | already AEGIS |
| HUD brand | — | — | AEGIS-V1 | src/ui/hud.js:40 | already AEGIS |
| Wheels | 8 | README.md:137 | 6 corners | config.js:49 | README → 6 |
| Suspension | 4 walking-beam bogies | README.md:138 | rocker-bogie, 6 struts | config.js:48-56, rover.js:3 | README → rocker-bogie |
| Wheel Ø | 1.40 m × 0.45 m | README.md:137, :229 | 2.12 m × 1.22 m (k(0.065), k(0.075)) | config.js:58 | README → 2.12 m |
| Steering | ±66°, 18°/s, Ackermann/crab/pivot | README.md:139, :87 | skid steer, maxAngle 0 | config.js:76 | README → skid steer |
| Mass | 6,980 kg (5,900 + 4×270) | README.md:135 | 8,000 kg (6,200 + 6×300) | config.js:43-44 | README → 8,000 kg |
| Battery | 140 kWh | README.md:142 | 420 kWh | config.js:80 | README → 420 kWh |
| Solar | 2 × 11.5 m² tracking panels, solar mast | README.md:142, :118, :133 | 0 panels | config.js:81 | README → none on board |
| Speed loaded | 1.4 m/s | README.md:141 | 3.8 m/s limit | config.js:77 | README → 3.8 m/s |
| Speed empty | 2.2 m/s | README.md:141 | 4.95 m/s at 180-rpm cap (4.81 measured) | config.js:69-74 | README → 4.95 m/s |
| Peak torque | 1,450 N·m, 42 rpm | README.md:140 | 5,600 N·m, 44.6 rpm cap | config.js:72 | README updated |
| Size L × W × H | 15.0 × 6.2 × 5.6 m | README.md:133 | frame 9.79 × 9.86 m | config.js:50 | README updated |
| Hoist | 0.9 m hoist, 30 kN | README.md:136, :143 | travel 0 (ride height lifts) | config.js:83 | README updated |
| Docking stage | 0.6 m / ±0.18 m stage | README.md:112, :143 | stageMax 0 | config.js:83 | README: rover drives to capture |
| Module | 12 t, R 1.54 m | README.md:40, :111, :136 | 14,940 kg, R 3.98 m | config.js:94 | README updated, old marked superseded |
| LiDAR tilt | 15° nose-down | README.md:45 | −10° | config.js:85 | README → 10° |
| Climb limit | 0.6 m | README.md:76 | 0.902 m (0.85 × r) | config.js:87 | README → 0.90 m |
| Track / CG / tip angle | 5.5 m, 3.1 m, 42° | README.md:146 | track 7.67 m; loaded CoM ≈ 6.7 m | config.js:49, :77 | README: measured margin only |
| Wheel fault | "7-wheel drive" | README.md:116 | 6 wheels → 5 driven | sim.js:192 | README + code text fixed (§2) |
| Keys 1/2/3, Q/E | Ackermann / crab / pivot | README.md:87 | still bound, no effect on skid steer | src/main.js:165, :184 | documented; open question |
| Export filenames | — | — | `span40_*.csv/json` | hud.js:175, :182 | → `aegis-v1_*` (§2) |
| IndexedDB name | — | — | `span40` | main.js:22, :24 | **kept** (renaming drops saved maps) |
| Unused SPAN-40 model | — | — | `SPAN-40` labels in unused file | src/render/roverModel.js:1, :115, :250 | left; open question |
| Launch config name | — | — | `span40-sim` | .claude/launch.json:4 | left |

## 2. Code / config changes in this pass (before → after)

| File | Item | Before | After | Behaviour impact |
|---|---|---|---|---|
| src/sim/sim.js `injectFault` | `fault wheel n` range | clamp 1..8 | clamp 1..`R.wheels.length` (6) | `fault wheel 7/8` no longer throws on `R.wheels[6]` undefined; valid inputs unchanged |
| src/sim/sim.js `injectFault` | log text | "redistributed to 7 wheels, speed derated 30%" | "to 5 wheels, speed derated 25% (autonomy.js)" | text only; code derate is `v *= 0.75` (autonomy.js) |
| src/sim/commands.js | help text | `wheel <1-8>` | `wheel <1-6>` | text only |
| src/ui/hud.js | export filenames | `span40_telemetry_*.csv`, `span40_mission_report_*.json` | `aegis-v1_…` | filename only |
| src/sim/config.js | comments | external path `Hunch/nasa-llaso-cad/js/…` | provenance tags [CAD]/[WONG]/[PHYS]/[ASSUMED] per group; repo-relative paths | none (comments only; values unchanged) |
| src/render/teamModels.js, aegis.js | comments | `Hunch/nasa-llaso-cad/js` | `public/team/js` | none |
| package.json | scripts | `test:physics/rocks/dock/faults/cycle` direct node calls; no `test` | `test`, `test:quick`, `test:long`; old names route through the runner | — |
| .gitignore | new | — | node_modules, dist, tests/out | — |

## 3. Test changes

| Test | Change | Setup change? |
|---|---|---|
| all `tests/*.mjs` | explicit thresholds, one `PASS`/`FAIL` line per check, `process.exitCode = 1` on failure (`tests/lib/check.mjs`) | — |
| headless.mjs | adds a ride-0 settle before kneel so kneel/carry deltas are measured from ride 0 | minor |
| odo.mjs | start 15 m off the pad edge (as headless.mjs), 15 s legs instead of 25 s | **yes**: original drove ~280 m straight from spawn and was stopped at 3.5 / 4.9 m/s (true 6.2 m and 0 m vs odometry 50.7 / 57.2 m), so it measured a collision, not odometry |
| dock.mjs | setup picks the module up autonomously (acquire) instead of `clampClose` + `lift.target = 0.6` | **yes**: the old setup assumed the SPAN-40 hoist; it failed for the 20-ft box ("no cargo held") |
| pickup_time.mjs | `process.exit(code)` → checks | — |
| nav_bench.mjs | counts loop closures and route-memory reuse from the log | — |
| dbg_*.mjs (6) | moved to `tests/scratch/`, imports fixed, excluded from `npm test` | — |
| new | `tests/run-all.mjs` runner (parallel, `TEST_JOBS`, `--quick`, logs to tests/out/) | — |
| new | `scripts/readme_facts.mjs` prints config-derived README numbers | — |

## 4. Changes NOT made by this pass (concurrent session, 2026-11-06 19:04–19:06)

Another Claude session ("NASA cargo transport robot simulation") edited the same working tree during this pass.
Listed here because they change physics constants without a before/after record:

| File | Item | Before | After |
|---|---|---|---|
| src/sim/config.js | `ROVER.lift.rate` | 0.25 | 0.5 |
| src/sim/config.js | `ROVER.lift.jawS` | 6 | 2.5 |
| src/sim/config.js | `POWER_DOCK` (new) | — | solar power dock, 4 × 120 m² arrays, 4 MWh bank, 350 kW charger (no provenance tag) |
| src/sim/autonomy.js | low-battery behaviour | wait for on-board solar (0 W → never recovers) | drive to power dock and charge; `missionCharge` |
| src/sim/rover.js | power bus | — | adds `dockW` charging term |
| src/sim/sim.js | lunar day/night cycle | — | `sim.lunar`, `day`/`night`/`lunar` commands |
| src/sim/powerDock.js, src/render/powerDock.js, tests/power_dock.mjs | new | — | power dock model, renderer, test (not yet using `tests/lib/check.mjs` or in `run-all.mjs`) |
| src/main.js, src/render/scene.js, src/render/terrainMesh.js | render | — | power dock visuals |

Test runs in this pass started before or during these edits. See §5 for which code each result reflects.

## 5. Test results

<!-- RESULTS -->

## 6. Runtime Three.js check (browser, `npm run dev`, 2026-11-06)

`window.THREE.REVISION` = "170"; `RoundedBoxGeometry` present; all 10 `public/team/js` scripts loaded
(`NASA_*` globals present); no `<script>` loading another Three.js; console clean (no "multiple instances of
Three.js" warning). The team scripts use only APIs present in r170 (BoxGeometry, CylinderGeometry,
ExtrudeGeometry, MeshPhysicalMaterial, …; no legacy `Geometry`/`Face3`).
