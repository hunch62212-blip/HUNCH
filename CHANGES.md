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

## 7. Wheel-odometry slip estimate: remove wheel spin-up torque (2026-11-06)

**Why.** `drift`, `faults`, `nav_bench` fail at `f4e7c18` and at `4f74946` alike (SLAM error 5.6–6.0 % of
distance, no loop closures). Raw encoder distance reads +2.8 % long (normal slip); the slip-corrected odometry
reads 5.5 % **short**. `Sensors.slipFromTorque` treats the whole motor torque as soil thrust, but the wheel
solver (`rover.js`, `I·dω/dt = T − H·r`) spends part of it spinning the 120 kg·m² wheel up and down. The
estimate therefore over-reads slip whenever the wheel speed changes (accelerating, PI corrections over rocks).
LiDAR scan matching on/off gives the same drift, so it is not the source (and does not correct it).

| File | Item | Before | After |
|---|---|---|---|
| src/sim/sensors.js `update` | torque given to `slipFromTorque` | motor torque `T` | `T − I_wheel·Δω/Δt` (encoder speed change; wheel inertia from `ROVER.wheel.inertia`) |

Behaviour impact: SLAM dead reckoning only. No physics constant changed; vehicle motion is unchanged.

Measured on the pad → habitat drive, scan matching off (`tests/scratch/dbg_slip_terms.mjs`, `dbg_slip_est.mjs`):

| Measure | Before | After |
|---|---|---|
| per-wheel estimated ground speed / physics true-slip speed (180 s) | 0.980 | 1.000 |
| per-wheel samples read as 100 % slip (wheel reported stalled) | 3.29 % | 1.40 % |
| body odometry distance / true distance, all driving (240 s) | 0.945 | 0.966 |
| … while accelerating | 0.926 | 0.962 |
| … steady | 0.985 | 0.990 |
| `odo.mjs` ratio at 1 / 2 / 3.5 / 4.9 m/s | 0.995 / 0.998 / 0.994 / 0.997 | 1.001 / 1.001 / 0.999 / 1.002 |
| `drift` worst error / distance; final error | 6.04 %; 104.03 m | 4.47 % (pass); 74.74 m (fail) |

Long tests after the change (`TEST_JOBS=4`, tree also held another session's uncommitted `config.js`, `rover.js`,
`scout.js` edits): `drift` drift ratio now passes (6.04 → 4.47 %) but final error is 74.7 m and it does not
arrive in 900 s; `faults`, `nav_bench`, `cycle` still fail. Remaining ~3.4 % under-read is wheel contact speed vs
body speed (per-wheel estimate already matches the physics), and nothing bounds the accumulated error: scan
matching does not correct it and loop closure fired once (10.5 m) on the whole nav_bench leg.

## 8. Fiducial position fixes after the map is built (2026-11-07)

**Why.** Nothing bounds dead-reckoning drift on a one-way drive: on the pad → habitat drive scan matching
applied 0.00 m of correction in 180 s (87 % of matches rejected by the 1 % score gate, the rest flagged
degenerate in x and z; `tests/scratch/dbg_match.mjs`), loop closure needs a revisit, and surveyed fiducials
were ignored once the map passed 30,000 cells (about the first minute) because snapping the pose would leave
the map, routes and plan in the old frame.

| File | Item | Before | After |
|---|---|---|---|
| src/sim/autonomy.js `beaconFix` | map > 30,000 cells | sighting ignored (advisory only) | sightings ≤ 60 m averaged (weight 1/σ², σ from the camera fiducial model; window resets after a 3 s gap); after ≥ 3 sightings, when the mean is good to max(0.4 m, 5 % of the offset) and the offset is ≥ 0.5 m and ≤ 5 m + 8 % of distance since the last fix, apply it (a large drift is fixed coarsely, then refined) |
| src/sim/slam.js `shiftFrame` (new) | applying a fix to a built map | — | estimate moves by the full offset; map cells, trajectory and route points first recorded since the last fix move by offset × (odometer distance at that time / distance since last fix); older cells and points stay |
| src/sim/autonomy.js | plan state after a fix | — | current path, learned obstacle marks (not fault marks) and port estimates move by the full offset; forces a replan |
| src/sim/slam.js | trajectory / route points | `{x, z, t}` / `{x, z}` | add odometer distance `d` / time `t` (needed for the above) |
| (unchanged) | map ≤ 30,000 cells | 0.12 × confidence nudge per sighting, yaw too | unchanged |

Yaw is not corrected by large-map fixes (the sun sensor already bounds heading; rotating the map is not done).
Behaviour impact: SLAM estimate, map, routes and plan only. No physics constant changed.

| Measure | Before (§7 code) | After |
|---|---|---|
| `drift` worst error / distance; final error; arrived | 4.47 %; 74.74 m; no | unchanged (4.47 %; 74.74 m; no): the 900 s cap ends before the ports are in view. Run to 1500 s (`tests/scratch/dbg_fiducials.mjs`): fix of 81.3 m at 1013 s, error 0.4 m, arrived 1065 s |
| `faults` | 1/3 | 1/3: boxed in by obstacles 1,100 m from the goal (no fiducial in view there) |
| `nav_bench` | 0/2 | 3/5: hab leg passes (fix 81.3 m → error < 1 m, 0 warnings); pad leg reuses the route but is boxed in (24 warnings) |
| `cycle` | 1/3 | 1/3: boxed in near the pad at 5.6 min (SLAM error 2–6 m), unaffected by this change |

## 9. "Boxed in": belly-clearance check and loaded roll levelling (2026-11-07)

**Why.** At each "boxed in" stop in `cycle` and `faults`, the blocking points were checked against the true
geometry (ray casts through terrain, rocks and structures; `tests/scratch/dbg_boxed.mjs`): 6 of 7 stops were
29/29 phantom points.

* **Belly check.** `LocalPlanner.evaluate` compares the ground under the tub with the *mean* wheel height,
  ignoring that the tub tilts with the rover. On a perfect uphill plane (`tests/scratch/dbg_belly_plane.mjs`) the
  empty rover is blocked from 11° (rated 20°). This is the `faults` stop (map 8–10°, step 0.06 m).
* **Roll levelling.** `Rover._leveling` moves each strut seat at a rate proportional to body roll (integral
  action only). Loaded (CoM ≈ 6.8 m), roll swings with growing amplitude (period ≈ 10 s, strut seats ±2 m,
  1–3 of 6 wheels on the ground) until the rover rolls over at 77–87° on 1–4° ground (`dbg_tip_cycle.mjs`,
  `dbg_tip_wheels.mjs`). Same at the committed `config.js`/`rover.js`. With levelling frozen while driving loaded
  (`dbg_level_ab.mjs LEVEL=freeze`): no roll-over, peak roll < 5° over 1,300 m.

| File | Item | Before | After |
|---|---|---|---|
| src/sim/planner.js `LocalPlanner.evaluate` | belly clearance reference | mean of wheel ground heights | least-squares plane through the wheel ground heights (body frame), evaluated under each belly point |
| src/sim/rover.js `_leveling` | strut seat rate | `0.6 × err` (err = −0.85 × corner roll height) | `0.6 × (err − Td × g × 0.85 × d(corner roll height)/dt)`, `ROVER.susp.levelTd` = 6 s (new), g = speed gate 0 below 0.1 m/s → 1 above 0.4 m/s. Trials (`cycle`, Td = 1 / 3 / 6 s ungated): peak loaded roll 4.4° / 1.1° / 0.7°, no roll-over; but ungated (and with a 0.3 s smoothed rate) `dock_cyl40` failed the coupling-cone lateral check, so the gate keeps the original response near standstill |

Not changed here: map slope over-read (5–6° true reads 9°; blocks the loaded rover at its 8.5° limit).

| Measure | Before | After |
|---|---|---|
| empty rover, perfect uphill plane: steepest drivable | 10° | 18° (rated 20°); a 1.0 m mound under the belly still blocks on flat and 10° ground |
| loaded rover, perfect uphill plane: steepest drivable | 8° (limit 8.5°) | 8° (limit 8.5°; the concurrent session has since raised it to 10.5°) |
| `cycle` peak roll while loaded; outcome | 77° roll-over at 321 s; boxed in | scratch copy with the gate: peak 0.9°, no roll-over, then "no progress for 4 min" toward the PORT-1 lead-in at 583 s. Full suite: boxed in at 8.9 min, no tilt reflex |
| `drift` / `faults` / `nav_bench` / `cycle` | 1/3 · 1/3 · 3/5 · 1/3 | 1/3 · 1/3 · 3/5 · 1/3 (full suite 11/17; `dock_cyl40` 1/5 and `dock_box20` 1/5). The tree also held the concurrent session's docking, slope-limit, step and beacon edits; in a copy taken before those, `dock_cyl40` passed 7/7 with this change and `dock_box20` failed with and without it |

## 10. Delivery accuracy, faster pickup / drop-off, loaded driving (2026-11-07, other session)

Not re-tested after the last edits in this section (user asked for no test runs); every file passes `node --check` and imports.

| File | Item | Before | After |
|---|---|---|---|
| src/sim/sensors.js | docking-ring camera | lateral / vertical only, 7 m range | adds relative yaw (ring ellipse, 0.03° noise, ASSUMED); 12 m range |
| src/sim/autonomy.js `_deliver` | near-field steering (ring camera, now from 10 m) | `0.9·(yaw − atan(1.2·lat))`: 6.7 m slow mode, arrived ~1.5° skewed; squaring at contact swung the nose ~0.1 m | nose-point law `kap = (2p − L·p²)·yaw − p²·lat`, p = 0.5 /m, L = nose offset: lateral and yaw settle together |
| src/sim/autonomy.js `_deliver` | soft-contact creep | lateral-only steering | same nose-point law on 0.8 s-filtered ring readings; outer creep 0.04 → 0.06 m/s |
| src/sim/autonomy.js | speeds | drive-under 0.9 / 0.4 / 0.2·along; line-up ≤ 1.0; final approach 1.0 / 0.15·gap; fiducial wait 3 s | 1.5 / 0.7 / 0.35·along; ≤ 1.8; 1.2 / 0.25·gap; 1.5 s |
| src/sim/slam.js `setPayload` | loaded slope lethal / caution | 8.5° / 4.5° | 10.5° / 6° (static loaded tip angle ~29°; works around the §9 slope over-read) |
| src/sim/autonomy.js | loaded tilt reflex | 12° | 14° |
| src/sim/slam.js `wheelHazard` | step estimate | `max − min − 0.5·slope` (half the rise left as a phantom step) | `max − min − max(|gx|, |gz|)` |
| src/sim/sensors.js | power-dock beacon | none | surveyed 1.0 m tag on the battery vault face toward the pad |
| src/sim/autonomy.js `chargeAtDock` | goal offset | true pose − estimate (ground-truth leak) | none: surveyed layout in the map frame |
| src/sim/scout.js | `slopeMax` | inside a comment (undefined, slope slow-down never fired) | 25° |

Last measured before the speed / range edits: `dock_cyl40` captured at −9.4 mm lateral, −0.9 mm vertical, −0.20° yaw (3rd attempt).
Still open: the pickup lift check reads the true cargo height; `dock_box20` not re-run.

## 11. Teleport drift anchor, screen layouts, touch driving (2026-11-07, "Heavy tasks" session)

Checked in the browser (`vite`, Chrome) at 375×812, 812×375, 768×1024, 1280×720 and 1920×1080; `vite build` passes. No physics constant changed.

| File | Item | Before | After |
|---|---|---|---|
| src/sim/slam.js `relocalize` | drift anchor after `teleport` | unchanged: the next fiducial fix spread its correction over map / route data recorded before the jump | reset to the teleport time and odometer, so only data after the jump is re-placed |
| src/ui/hud.js, src/style.css | E-STOP in the top bar | last item in a clipped bar: off screen below ~1,200 px wide (also RESET / START) | pinned to the right edge; the rest of the bar scrolls sideways under it |
| src/style.css | 761–1,100 px wide (tablets) | log squeezed to ~90 px between the side columns at 768 px | log and status bar span the full width; side panels sized to stop above them |
| src/style.css | 1,101–1,300 px wide (laptops) | log and status bar widened under the map panel (overlap at 1280×720) | log keeps its centre column |
| src/style.css | camera panel | shown down to 761 px tall, ran off the bottom of a 768 px screen | hidden under 900 px tall (needs ~880 px below the top bar) |
| src/ui/hud.js, src/style.css | phones (≤ 760 px wide or ≤ 520 px tall) | panels overlapped each other and the log; 3D view mostly hidden | Vehicle / Map / Missions start as a strip of headers; tapping one opens it full width (one at a time); log + command line along the bottom |
| src/main.js, src/style.css | manual driving on touch screens | keyboard / gamepad only (no way to drive on a phone or tablet) | on-screen stick (touch screens only, or `?touch=1`): analog throttle / steer through the same path as the gamepad; touching it overrides autonomy like a key press |
| src/style.css | touch screens | 11 px buttons; iOS Safari zoomed the page when the command line got focus (12 px text) | larger tap targets; 16 px command-line text; no pull-to-refresh / page scroll on the 3D view |

## 12. Audit fixes: docking, fiducial fixes, slope read, stalls, comms, logistics, legacy code (2026-11-07)

| File | Item | Before | After |
|---|---|---|---|
| src/sim/autonomy.js `mapFix` | when a frame fix may apply | always (moved the port target 0.5-1.7 m mid-approach) | never while `lcSuspended`, in a `Final approach` / `Dock ` phase, or within 25 m of a port |
| | accumulator | one, all tags pooled | one per fiducial id |
| | acceptance | ≥ 3 sightings, std err ≤ max(0.4 m, 5 %) | ≥ 15 sightings, std err ≤ max(0.3 m, 3 %) (0.15 m floor tried: the refinement after a large fix never came, drift final error 5.3 m) |
| src/sim/slam.js `wheelHazard` | slope | 2-point difference over 1 m | least-squares plane, 9-point cross, 2 m baseline, per-axis centred; worst case for ±6 cm noise ~4° |
| | step | `max − min − 0.5·slope` | spread of the inner 5 points about that plane |
| src/sim/autonomy.js `_deliver` | ring-camera lateral | used as the nose offset | it is a bearing in the module frame; axis offset = `lat·cos ψ + range·sin ψ` (driving in 1.8° off-axis the two cancelled and every attempt failed) |
| | steering, ring leg (10 m) and creep | nose-point law | turning-centre line law `kap = 2p·ψ − p²·(y + L·ψ)`, p = 0.6 /m: stable for any assumed control point at/ahead of the turning centre |
| | steering, fiducial leg | bearing pursuit | same law on the rover's offset from the port axis, p = 0.3 /m |
| | yaw-rate | open loop (loaded scrub: 10 % of the commanded curvature achieved) | PI on the gyro (`cmdCurv`), authority ∝ speed |
| | ride trim on approach | +8 % of the raw vertical per tick (ride swung 0.57-0.98 m) | ~1 s filtered, 0.25 /s |
| | cone abort | single sample > 6 cm | ~1 s filtered lateral > 6 cm |
| | soft capture | latch on contact | latch on contact with the ring reading inside ±6 mm (loaded pitch rock bobs the nose ±3 cm); last 4 cm inch until aligned |
| | camera lost in the creep | waited 180 s × 3 | 5 s, then backs out: `docking visual sensor lost` |
| | loaded fiducial reposition | cross-axis > 2 m or > 6° | > 3.5 m or > 10° (the axis law absorbs it; repositioning circled inside the turning circle) |
| src/sim/config.js | `DOCK.vestibuleLen` | 1.2 m | 2.4 m: the 20-ft container rides 1.75 m behind the rover front, which hit the wall 0.5 m short of the port |
| src/sim/autonomy.js `checkPower` | low SoC while loaded | `chargeAtDock` refused, then an endless wait | continue if the battery covers the delivery (`deliveryEnergyWh`, ×1.25) with 5 % spare; else `pauseForOperator` (explicit status, `resume` overrides). Dock unreachable: same pause |
| src/sim/autonomy.js `navTo` | cargo shift fault | ignored | speed ×0.5 with a warning until re-centred |
| src/sim/autonomy.js `missionCycle` | `item.feasible` | ignored | infeasible leg recharges first; delivers to `cargo.dest` (reroute-aware) |
| src/sim/autonomy.js `_deliver` | faulted port hold | waits for the fault to clear | also leaves the hold when the operator reroutes |
| src/sim/commands.js | comms loss | command dropped (and the message never logged) | queued; `clear faults` / `clear comms` sends them in order |
| | new | — | `reroute <A|B> <port>` |
| | help | listed `steer ackermann|crab|pivot`, Q/E crab, 1/2/3 keys | removed |
| src/main.js | `ui.steerMode`, keys 1/2/3, `crabIn` / `drive.crab`, Q/E | present (unused on skid steer) | removed |
| src/render/roverModel.js | legacy SPAN-40 model | present, not imported | deleted |
| tests/drift.mjs | default `tmax` | 900 s (leg needs ~1,100 s) | 1500 s |

Results (sequential where noted): quick suite 12/12; `dock_cyl40` 4.4 / 3.4 mm, 0.14°; `dock_box20` 5.8 / 5.0 mm, 0.14°;
`drift` pass (worst 4.29 %, final 3.11 m, arrived 923 s); `m3` pass. Still failing, as at the §8-9 baseline: `faults` 1/3
(boxed in after the boulder block), `nav_bench` 3/5 (pad leg: tilt reflex 18-22° then boxed in), `cycle` 1/3 (loaded rover
loops near the pad for 7 min after a tilt reflex, no progress watchdog). The long tests ran before the 0.3 m `mapFix` floor.

## 13. Vehicle health (FDIR), random failures, thermal / survival, serviceability (2026-11-07)

New module `src/sim/health.js`: 20 line-replaceable units in 10 service bays, 24 failure modes, each with a sensor-based
detector (it reads sensor channels, never the injected truth), an automatic response, an operator procedure and a timed
repair. Random failures are a switch (`failures on [realistic|100x|1000x|demo]`, HUD M10 / Health panel), seeded and off by
default. Procedures: `docs/FAULT_PROCEDURES.md` (`npm run docs:faults`); NASA comparison: `docs/SURVIVABILITY.md`.

| Where | What | Before | After |
|---|---|---|---|
| src/sim/rover.js hotel load | heaters | fixed `ROVER.heaterW` 600 W while powered | thermostatic (`health.heaterW`, 200 W/K, vault ≤ 800 W, battery ≤ 1,500 W); 0 W on a sunny day |
| | powered-off hotel | 60 W | 60 W + heaters (survival keeps the vault ≥ −20 C, battery ≥ −10 C) |
| | generation | solar + dock | solar + keep-alive blanket (`auxSolarW`, 12 m², 30 %, parked only) + dock |
| | state of charge | Wh / nameplate capacity | Wh / on-line capacity (`capK`; an isolated string drops it to 0.75) |
| | hub thermal loss | motor losses | + `extraHeat` (cooling-path fault) |
| | clamp jaw time | nominal | × `clampSlow` (2 on the backup winding) |
| src/sim/sensors.js | yaw rate | gyro model | + `imuFaultBias` (IMU fault until voted out) |
| | sun sensor | always on | silent while failed |
| | rock-detect range | nominal | × `camRangeK` (dusty / failed camera) |
| src/sim/autonomy.js | FDIR hold | — | `holdUntil` zeroes the command (e.g. 8 s on a flight-computer switch-over) |
| | speed | wheel-fault derate | × `health.fx.speedK` (product of active alarm derates, e.g. 0.7 hub over-temperature) |
| src/sim/powerDock.js | coupling | geometric | also needs a working charge coupler |
| src/sim/sim.js | RESET | always | refused while the E-STOP loop is open |
| | START | always | refused while `health.canArm()` reports a reason |
| src/sim/commands.js | new | — | `fault list`, `fault <id> [unit]`, `failures on|off [preset]`, `health`, `procedure <id>`, `repair <id|all|abort>`, `survival on|off|auto`, `array deploy|stow` |
| | comms-loss whitelist | — | also `health`, `procedure`, `repair`, `failures`, `fault` |
| src/sim/telemetry.js | columns | — | + vault_temp, battery_temp, heaters, keepalive_solar, alarms, strings_online, fdir_speed_factor |
| src/render/aegis.js | model | — | louvered side radiators, service doors with status lamps, 4 omni whips + beacon, hazcams, keep-alive mast and blanket, red locator on the failed part |
| src/sim/health.js `_detect` | built-in-test faults (heater, louver, clamp, load cell) | needed the persistence time, but a BIT reading lasts one tick: never detected | a BIT reading alarms at once |
| | after a repair / clear | detector re-armed at once (a still-hot hub re-alarmed: spurious) | waits until its signature has gone once; a new injection re-arms it at once |

Results (`node tests/fdir.mjs`, 2026-11-07): 60/60 checks pass. Nominal 3 min drive 0 alarms, vault 19.7 C. Mode sweep 24/24
detected and isolated, 0 spurious (electrical faults 0.1–2 s; BIT faults at the next BIT; slow ones: hub cooling 295 s,
strut leak 151 s, LiDAR dust 78 s, camera dust 93 s). Repairs: dock and kit spares used, dock-only repair refused in the
field. Random failures at 3e5 x for 300 s: 5 failures, 0 spurious, none missed. Survival: 147 W (heaters 58 W), a 354 h night
= 12.4 % of the pack, vault −20.3 C. Keep-alive array 4.8 kW in sun, stows before a drive. Before the two `_detect` fixes:
20/24 detected, 1 spurious. Other test files were not rerun.

## 14. Loaded traverse: nose-over braking, slope learning, odometry in turns (2026-11-07)

Investigated the failing `cycle` test (loaded rover looping near the pad, then "no progress toward lead-in PORT-1 for
4 min"). Traced with `tests/scratch/dbg_cycle.mjs`; three real faults were found and fixed. The test still fails (see below).

| Where | What | Before | After |
|---|---|---|---|
| src/sim/rover.js `_smartRpm` | rpm ceiling drop | instant: a loaded "crawl" cut clamped the wheel targets past the `_allocate` acceleration limiter and braked the vehicle hard | loaded: falls at the `accel.lonLoaded` rate |
| src/sim/config.js `accel.lonLoaded` | loaded braking / acceleration | 0.5 m/s² | 0.2 m/s². Each rocker-bogie side shares load equally across its 3 wheels, so only `rb.diff` (0.45) of the springs resists pitch: with CoM 6.8 m the rear wheels lift at ≈ 0.45 · g · 3.9 / 6.8 ≈ 0.42 m/s² on flat ground, less downhill. Measured: 0.42 m/s² on a 2.6° slope pitched the body to −16.7° on 2 wheels and fired the 14° tilt reflex |
| src/sim/autonomy.js `navTo` | loaded tilt speed law | `3 − 0.22 · body tilt`: braking pitch → slower → more braking (positive feedback) | uses the mapped ground slope under the footprint (body tilt only if unmapped) |
| | loaded stopping-distance governor | 0.3 m/s² | 0.2 m/s² (matches the new limit) |
| | loaded local-planner block | 30 hit disks pushed every tick, forgotten after 180 s: A* side-stepped a few metres into the same 11–12° face (its 2.5 m window reads it under 10.5°), and came back once the marks expired | hits learned once per block for 900 s, plus a sweep of the wheel-scale hazard over 30 × 21 m ahead (whole face) |
| | periodic replan | 15 % hysteresis | + a reversal guard: a route that turns the rover back (> 100°) must be 40 % cheaper |
| src/sim/sensors.js wheel odometry | slip estimate | per wheel: in a skid-steer turn the outer (driving) and inner (braking) scrub torques were each inverted through the nonlinear thrust curve and did not cancel. Loaded odometry read 1.4 % short straight, 6–7 % short in turns (≈ 30 m SLAM error after 1.5 km) | one body-level slip from the net drive torque and mean load. Loaded: 0.2 % (straight and turning). Empty `drift` final error 3.11 m → 1.57 m |

Results:
- Quick suite 12/12, dock_cyl40 2.7 / 2.6 mm / 0.26°, drift pass (final 1.57 m, worst 2.64 %/distance).
- Long suites: m3 3/3, fdir 59/59, `faults` now passes 3/3 (it was 1/3, boxed in after the boulder block). nav_bench is unchanged at 3/5: the pad leg did not arrive.
- `cycle` still fails. The rover no longer noses over, loops by the pad or forgets learned slopes, and its SLAM error now stays a few metres for the first ~1 km.
- The remaining failure is exploration. The first loaded traverse of the 2 km pad → PORT-1 leg crosses an unmapped crater field: crater complexes near x −680..−400 and x −120..130. A* "explore" (unknown ground 1.5 /m) aims straight into each complex. The loaded rover cannot point-turn, so it gets boxed into pockets between rims and the 4-minute progress watchdog fires.
- Fixing this needs ground knowledge before the loaded leg: a scout-led survey, an empty survey drive with teach & repeat, or an orbital hazard prior. Tuning won't do it.

## 15. Survey drive before the first loaded leg (teach & repeat) (2026-11-07)

Follows §14. Before it carries a module, `missionCycle` now calls the new `surveyRoute(cargo)`, unless a proven route already joins the cargo and the port approach.
- **Outbound:** the empty rover drives to a point 45 m out on the destination port's axis. It uses the loaded traversability limits (`this.surveyLimits`: loaded slope/step/belly limits, local planner and costmap), but keeps empty point turns and speeds. The drive is recorded as a route.
- **Return:** an ordinary empty drive back to pad staging; the pad apron itself reads lethal under the loaded limits.
- **Loaded leg:** repeats the route, or, if the repeat is rejected, A* plans over the now-mapped corridor, where proven-route cells get a cost bonus.
- **Planning fix:** `navTo` now sets the leg's limits (`slam.setPayload`) before its first plan. It used to set them only inside the drive loop, so a leg's first plan used the previous leg's limits.

Result: `cycle` still fails, now at 82 min instead of 13 min.
- The survey works. The empty rover maps the corridor and reaches the PORT-1 approach (route R1, 3793 m, 64 min).
- SLAM error builds to ~36 m along the long winding route, then 65 m on the way back, so the recorded route and the map no longer line up and the return drive stalls.
- Straight-line odometry is exact. In turns it is within ~1 %. The error comes from pivot turns: the skid-steer body slides (~27 m in total over 25 min) and odometry cannot see it.
- A single fiducial fix at the habitat would only shift the whole map rigidly.
- Fixing this needs the drift spread back along the trajectory (pose-graph loop closure, or fiducial fixes that bend the recorded route), or a scout-led survey. That is a larger SLAM change.

Quick suite 12/12, drift and m3 pass.

## 16. Black-screen fix: post-processing chain and shadow memory (2026-11-08)

Safari WebGL2 and external high-DPR displays could show a black 3D viewport. Changes in `src/render/scene.js`:
- **Composer target:** dropped `samples: 4` from the `EffectComposer` render target (a multisampled half-float target fails its framebuffer resolve on Safari WebGL2). It stays `HalfFloatType`. Anti-aliasing comes from SMAA.
- **Pass order:** `RenderPass` → `UnrealBloomPass` → `SMAAPass` → `OutputPass`. SMAA now runs in linear space; `OutputPass` is last and does the ACES tone mapping and sRGB conversion.
- **SMAA size:** `smaa.setSize(innerWidth, innerHeight)`, matching `composer.setSize`. It no longer multiplies by the pixel ratio, because the composer applies the ratio itself, so SMAA was 2× too large on retina.
- **Shadow cap:** the `ultra` CSM shadow map is 2048, down from 4096. Four 4096² cascades exhausted GPU memory on external monitors and Apple Silicon unified memory.

Checked: `npx vite build` passes; the viewport renders at default quality and at `ultra`, with no console errors (Chrome-based browser pane, DPR 1 only; Safari and high-DPR displays not tested here).

## 17. Flight-readiness pass: crater-trap costing, pose-graph fiducial fixes, impedance docking, e-mech struts, hub heaters, fenders (2026-11-08)

Status: **in progress**. This table is written before the code changes (project rule). The "After" column is the intended value; the
results subsection at the end records what was measured. Section numbers are cited from code comments (`CHANGES.md §17`).

Baseline (before any edit, `TEST_JOBS=4 node tests/run-all.mjs --quick`): see "Results" below.

| Where | What | Before | After |
|---|---|---|---|
| src/sim/planner.js `buildCostmap` | unmapped ground near hazards | every unmapped cell 1.5 /m, so A* "explore" routed straight into unmapped pockets between crater rims (§14, §15) | an unmapped cell within 15 m of a `CLS.SLOPE` or `CLS.CRATER` cell costs 12.0 /m (`Planner.unknownNearHazardCost`, `unknownHazardRadius`) |
| src/sim/planner.js `smooth` | string pulling | any chord that is not costlier than the grid path; corners unbounded | loaded: a chord is accepted only if the corner it makes with the previous segment can be filleted at R >= 18 m (`Planner.loadedMinRadius`); the output is then filleted so the planned centre line keeps R >= 18 m where the segments are long enough (see results for the residual) |
| src/sim/autonomy.js `navTo` | loaded, local planner `ok === false` | learn hits, replan, then `recover` (reverse on a clear arc, <= 5 m) up to 8 episodes, then fail | loaded: new `*unwindPath(25)` after the second blocked episode: reverses 25 m along the recorded entry breadcrumbs at 0.15 m/s with the rear arcs checked by the local planner, marks the corridor ahead lethal for 1200 s, forces a global replan. A third unwind fails the leg |
| src/sim/slam.js `shiftFrame` | fiducial fix on a built map | map cells / breadcrumbs / routes since the last fix translated by (offset x odometer fraction), no rotation | keyframe graph (node every 10 m or 15 deg); the fix pins the sighting node, an SE(2) pose-graph relaxation (Gauss-Seidel with SOR over relative-pose edges) spreads translation and heading error back along the nodes; each cell / breadcrumb moves with the blend of its two neighbouring nodes' rigid corrections |
| src/sim/sensors.js `update` | gyro yaw rate | true body yaw rate + bias + noise | minus `ROVER.imu.kScrub * sgn(wz) * abs(vx * wz)` while turning loaded (value and measurement in the results) |
| src/sim/autonomy.js `_deliver` M5 Dock | contact phase | creep v = 0.002 / 0.012 / 0.06 m/s by gap and alignment, -0.01 above `maxPushN`; capture on the first aligned (+-6 mm) sample with contact > 900 N | inside 0.1 m: v = clamp(Kf (1200 N - F), -0.02, 0.015) with Kf = 1e-5 m/(s N); at F > 200 N the wheel targets are zeroed (ramp <= 0.1 s); latch only after +-5 mm vertical and lateral for 1.5 s |
| src/sim/config.js `ROVER.susp`, health.js `LRUS.STRUT`, mode `strut` | strut technology | air strut + levelling valve, "slow leak", vent / re-pressurise repair | roller-screw electromechanical actuator (brushless DC servomotor, metal bellows seal). Same k, c, bumpK, bumpC. Fault `strut` (id unchanged) is now "Roller-screw actuator jam / motor winding open circuit" |
| src/sim/config.js `THERMAL` | wheel hub heating | none | `hubHeaterW: 25` per corner, `hubFreezeC: -65`; health.js adds a hub thermal state and powers the hub heaters when a hub is below -65 C at night or in survival (counted in `heaterW`) |
| src/render/aegis.js, src/sim/config.js | wheel fenders | none | `FENDER` parameters in config.js; a fender over each wheel and an outboard ejecta deflector in front of each deck radiator |

## 18. 2.5 km random site, 50x fast forward (transit warp), map-edge slope bug, creep fix (2026-11-09)

Requested: bug-fix pass, a 2.5 x 2.5 km site with random crater / boulder placement, the pad and the habitat 2 km
apart, a shorter trip, and a fast-forward button that runs sim time 50x faster.

### Measured before the changes
* Per physics step: 2.1 ms (sensors 46 %, rover 24 %, autonomy 20 %, Rapier 10 %), so the full physics tops out at
  ~4x real time headless. In the browser the existing **20x button delivered 1.3-1.6x** ("CPU-limited").
* Full cycle (tests/out/cycle.log, 2026-11-08): FAILED, 0 deliveries, 3.8 km survey route for a 2 km leg, SLAM error
  up to 47 m. Re-run on 2026-11-09: 430 m of progress in the first 24 sim-min (pivots, creeping, unwinds).

### Changes
| Where | What | Why |
|---|---|---|
| config.js `WORLD` | size 2048 -> 2500 m; pad / habitat placed symmetrically about the centre, exactly `routeLen` = 2000 m apart (was 1997 m) | requested |
| terrain.js | crater and boulder counts scale with map area (same density: 3373 craters, ~8.9k boulders); the three "route" craters and the boulder field are placed at random spots along the route from the seed (were fixed offsets) | requested: random placement |
| main.js | random seed per page load, `?seed=N` repeats a map, `newmap` command / "new map" button rerolls; tests keep the fixed default seed | requested |
| terrainMesh.js | chunk count `ceil(size / 128)`, overhanging chunk vertices clamped to the heightfield edge | 2500 is not a multiple of 128; the old raw index would have read past the end of a heightfield row |
| sim.js `warpStep / stepAuto / advance`, rover.js `railsBegin / railsPlace / railsEnd`, autonomy.js `_railsCheck / _railsAdvance`, sensors.js `warpScan` | **transit warp** at >= 50x: long cruise legs (and empty navigation pivots) run kinematically along the planned path in 0.5 s steps; the body sits on the terrain plane under the six wheel stations, held cargo moves rigidly with it, drive power comes from the planning energy model, the odometry the SLAM filter integrates is the rails motion, a coarse LiDAR sweep every 6 m keeps the map ahead filled. Rails only while the next 30 m of path is mapped and clear under all six wheel tracks, > 45 m from the goal, tip margin > 20 deg; everything else (pickup, docking, charging, recoveries, loaded turns near obstacles) stays on full physics | full physics cannot run 50x |
| hud.js, main.js, style.css | ⏩ 50x button, `X` hotkey, `ff` command; the clock shows the speed actually achieved ("actual 48x, transit warp" / "CPU-limited"); `advance` returns sim seconds and the dust effect uses a capped step | requested; the old clock showed the requested scale, not the achieved one |
| slam.js `computeHazard` | slope plane solved with the full 2x2 normal equations; nearly collinear samples (one LiDAR ring) use the minimum-norm fit (slope along the ring only) and read ROUGH | **bug**: the x and z slopes were solved independently, ignoring their correlation, and a single ring's cross-ring slope was noise. The whole edge of the mapped area read 10-20 deg (lethal under the loaded / survey limits), walling the planner in and causing the pivot / reverse thrash |
| slam.js `pinhole`, planner.js `evaluate` | an unmapped 0.5 m cell with >= 5 of its 8 axis neighbours mapped no longer ends `knownDist` | **bug**: pinholes between LiDAR rings made the rover "creep: unmapped ground ahead" at 0.3 m/s on mapped ground |
| sensors.js `sunTick` | sun sensor also ticks during warp | FDIR raised a false "sun sensor failure" during warp |
| autonomy.js `unwindPath` | empty rover unwinds its own fresh track at 0.4 m/s (loaded stays 0.15) | a 25 m unwind took ~3 min |
| autonomy.js `stabilityCheck` | the reported cargo shift is measured (module position in the rover frame, max over the test) | **bug**: it was the constant `R.lift.cargo ? 0 : 1` |
| commands.js `ride` | sets `lift.target` | **bug**: it wrote `rideOffset`, which the lift controller pulled straight back to `lift.target` the next step |
| sim.js `_precisionPost` | the float64 position guard only corrects below 0.1 m/s (its purpose: creeping under a module / into a dock); `propagateModifiedBodyPositionsToColliders` after it | **bug**: re-setting the rover position after every step at driving speed left its broadphase proxy stale, so it never formed a contact pair with rocks streamed in nearby. In tests/rock_climb.mjs the chassis drove **through** the 1.4 m boulder, and the 0.57 m boulder "climb" showed 0.1 deg of roll because it never touched it either (now 0.7 deg). Ray and wheel casts still saw the rocks, which hid it |
| world.js `streamRocks` | rock colliders are created once and then enabled / disabled, never removed; newly streamed ones are re-marked after the next step | removing and creating colliders in the same call let Rapier reuse a freed slot before the removal was processed |
| tests/autonomy.mjs | `WARP=1` (run as the browser does at 50x), `SEED=n`, `PRINT=s`; yaw error printed wrapped | the log printed 360 / 720 deg yaw errors |
| tests/scratch/profile_step.mjs, dbg_costmap.mjs, dbg_cls.mjs | per-module step profiler; costmap and SLAM class renders (PPM) | debugging aids |

## 19. Black-blob fix: wheel-track mesh had no normals (2026-11-09)

At load, a black blob covered most of the 3D viewport and came and went (Chrome and Safari, 1920×1080, any quality with bloom on).
- **Cause:** the wheel-track decal (`Tracks`, `src/render/effects.js`) uses a lit `MeshStandardMaterial`, but its geometry had only `position` and `uv`. WebGL supplies a normal of (0,0,0) and `normalize(0)` = NaN, so every track pixel rendered as NaN. The rover lays tracks under the chase camera at load. `UnrealBloomPass` (mip blur) and SMAA spread those few pixels into a screen-sized black blob. A full-resolution float render showed 117–393 NaN pixels; hiding only this mesh cleared them all. It was the only lit mesh without normals.
- **Fix:** `Tracks` now has a `normal` attribute (default up); each new quad gets its face normal (pointing up) so the ruts are lit correctly on slopes.
- **Guard:** `src/render/scene.js` adds a `ShaderPass` between `RenderPass` and bloom that replaces NaN/Inf pixels with black and clamps HDR to 1000. Any future bad pixel stays one pixel instead of spreading.

Checked: `npx vite build` passes. In Chrome at 1920×1080, GFX ultra, DPR 1: three reloads showed no blob; a full-resolution float render of the scene had 0 NaN pixels before and after driving (1,392 track quads laid). Safari not tested here.

## 20. Survival hardware mounted on the body (radiators, skirts, whips, beacon, keep-alive outrigger) (2026-11-09)

The §17/§18 hardware was positioned from `ROVER.frame.halfWid` (4.93 m), but the team body is much narrower. So the radiators, ejecta deflectors, the four whip antennas, the beacon and the keep-alive canister hung in mid-air 1.5–2.6 m outboard of any structure. Measured on the rendered team model (ray casts, body frame): tub walls z ±2.23–2.33 (y 1.6–3.4), deck top y 3.67 out to z ±3.09 and x ±4.73, rocker beams z ±2.95, wheel stations z ±3.836 (fender inner edge ±3.09).

| Part | Before | After |
|---|---|---|
| Vault radiators | 8.0 × 0.86 m plates at z ±4.86, y 3.18, nothing behind them | `RADIATOR` in config.js: 7.2 × 0.85 m (12.2 m², matches `THERMAL.vault.radArea`) on two rails on each tub wall, z ±2.36, y 2.95, under the deck overhang and inboard of the rocker beams and fenders; louvers unchanged |
| Ejecta deflectors | 7.6 × 1.05 m plates at z ±5.08, free-floating | ejecta skirts: 7.2 × 0.45 m plates hinged to the tub wall along each radiator's lower edge, sloping 40° outboard-down (outer edge z ±2.72) |
| Omni whips | 2.6 m, at (±4.7, ±4.7), base 1.6 m off the deck | 2.0 m on base pucks seated on the deck edge (z ±2.95; front pair at x 3.0, aft of the LiDAR heads). They stay 0.6 m under the carried module's underside; Earth is near the horizon at the pole, so they need a horizontal view, not a zenith one |
| Beacon | sphere at (-4.7, 4.0, 4.4) | on a 0.45 m post on the starboard-aft deck corner |
| Keep-alive canister + mast | box at (-4.6, 4.2, -4.75), nothing under it | same plan position (outside the module footprint), on an outrigger: post on the port-aft deck corner, 0.2 m arm at `KEEPALIVE.armY` 4.35 m (above the rear fender's worst case, ~4.15 m at full kneel + bogie lift), tie rod above the arm; mast base moves up to 5.45 m |
| AV door, hazcams | 0.2–0.35 m in front of the front face | flush on the front face (x 4.28) and on the front/rear faces (x 4.17 / -4.43) |

`health.js` BAYS.RAD and `partPos` now read `RADIATOR`, so the failed-part locator points at the new panels. Deck items are seated with a ray cast onto the team model at build time.

Checked: `npx vite build` passes; no console errors; looked at the rover from port-aft, port side (radiator behind the rocker), front and top in the browser; physics, planner and scout tests pass. Earlier the same session, `npm run test:quick` was 12/13: `dock_cyl40` failed in navigation (the rover drove away from PORT-1, "m to go" 1173 → 1364). No test imports render code, so §19 can't have caused it; it is still open.
