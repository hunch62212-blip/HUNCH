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

## 21. Lander collision bug, burned-out headlights, panel labels; physics overlay + dynamics instruments (2026-11-09)

Bugs from the 2026-11-09 9:55 / 9:59 PM screenshots:

* **Rover climbed the lander and rolled over** (pitch 70°, tip margin −84.8°). The lander's colliders were 2.6×
  (`LANDER_SCALE`) the drawn model: invisible horizontal leg struts 3.8–4.6 m up at 10–18 m from the axis made a
  ramp. Wheel shape-casts took any face they touched as ground and the strut pushed straight up, so once the body
  tilted the rover walked up the strut. Fixes: the drawn lander uses the same `LANDER_SCALE`; the leg colliders are
  diagonal capsules matching the drawn legs (`LANDER_LEG`), plus engine-bell colliders; a wheel on a structure
  bears only on faces within 40° of both the body axis and true vertical (a contact pushes only along its
  normal; friction 0.7 climbs ≤ 35°). Regression test `tests/lander.mjs` (`npm test` entry `lander`): driving into
  a leg now peaks at 3.7° pitch (was 64°).
* **Lander tags never seen ("fiducials 0").** The four surveyed tags sat 7 m inside the descent-stage collider, so
  the camera's line-of-sight ray hit the hull first. Now on the scaled descent-stage wall (`LANDER_TAG`).
* **Camera feed and ground ahead burned out white.** Headlights were 900 cd and always on: 36 at 5 m against the
  sun's 5.2. Now 150 cd in shadow or at night, 15 cd in sunlight, eased (`src/render/aegis.js`).
* **Vehicle panel labels wrapped onto 3 lines.** Labels no longer wrap, long values wrap in their own column,
  shorter labels with tooltips; side panels 320 px.

New: **engineering overlay + dynamics instruments** (toggle **I** / Overlays → physics). Research and formulas:
[docs/GRAPHICS_PHYSICS_RESEARCH.md](docs/GRAPHICS_PHYSICS_RESEARCH.md). `src/sim/dynamics.js` (SSF, rollover tilt,
utilisation, stopping distance, drawbar pull, traction / resistance coefficients, slip, power and efficiency,
Wh/km, cost of transport, Froude number), `src/render/engOverlay.js` (wheel load / traction arrows, CoM + plumb
line, support polygon with critical edge, predicted path, braking bar), `src/ui/instruments.js` (attitude
indicator with the rover's own rollover limits, LMI-style stability plan view, readouts with formula tooltips).

## 22. Runaway after false "wheel seized" isolations; wheel force arrows removed (2026-11-10)

* **Runaway from PORT-1 (the `dock_cyl40` failure, and one way M6 "heads somewhere random").** Pushing the loaded rover
  against the building stalls every wheel at once: full phase current, resolver ~0. The `seize` signature matched that on
  four wheels within 6 s. FDIR declutched all four, the rover was left with two driven wheels that could not make the
  commanded turns, and it drove straight away (goal distance 9 m → 1,364 m). The signature now also requires at least
  three of the other wheels to be turning (`health.js` MODE `seize`). A common-mode stall is a blocked vehicle, not six
  failed hubs. `tests/fdir.mjs` still detects an injected seizure (wheel 5, 1.7 s) and declutches it: 65/65.
* **Wheel load / traction arrows removed** from the engineering overlay (`src/render/engOverlay.js`) at the operator's
  request. CoM, support polygon, predicted path and braking bar stay.

Still open (being worked on for the next version): the navigation watchdog counts newly mapped ground as progress, so a
rover heading away from its goal is not stopped. A* prices unmapped ground (1.5 /m) below typical mapped ground, so the
route flips as the map fills in. Point turns at > 50° heading error; smart rpm is 3 fixed tiers capped at 180.

## 23. Lunar-realism review fixes: traction, speeds, thermal, power dock, sensing, comms (2026-11-10)

An engineering review of what would not work on the Moon. Fixed in this pass:

| Where | What | Why |
|---|---|---|
| config.js `ROVER.accel` | lon 1.6 → 0.9, lat 1.2 → 0.8 m/s², E-STOP factor 2.0 → 1.2 | regolith gives at most ~0.70 W of shear (tan 35°), so nothing above ~1.13 m/s² is reachable in 1.62 m/s² gravity. The old E-STOP asked for 3.2 m/s² (2 g_moon) |
| config.js `speedLimitEmpty` 3.0, `speedLimitLoaded` 3.8 → 1.5 m/s; rover.js `speedCap` | operating limits | the Froude-scaled motor caps (4.95 / 8.2 m/s) show similarity with the prototype, they are not safe speeds for an 8 t (23 t loaded) vehicle on cratered ground |
| rover.js motor thermal, health.js hub node, config.js `THERMAL.hub` | winding heat now conducts into the hub mass, which radiates through a temperature-switched radiator face | the winding was cooled by a fixed 20 W/K path to a constant −20 °C: convection, which does not exist in vacuum. The hub model separately assumed 8 W of motor heat |
| health.js `_thermal` battery | pack heat uses the net bus current (charge or discharge) and R(T) | charging heat was not modelled: 350 kW (875 A) put ~15 kW into a pack that can radiate ~1 kW |
| config.js `POWER_DOCK`, powerDock.js | charger 350 → 60 kW; no charge below 0 °C cells, taper 40–50 °C; bank 4.0 → 1.5 MWh with a written budget; bank discharge loss; array derate 0.85; every mast checked for terrain shadow and for the shadow of the masts up-sun of it | heat-limited charging, lithium plating, ~20 t of cells nobody needed, and only mast 0 was ever shadow-tested |
| config.js `THERMAL` | closed-louver fraction 0.06 → 0.12; parasitic leak 0.25 / 0.4 W/K; battery resistance rises 3 %/K below 20 °C | optimistic louvers and no heat leak. Night survival is now 226 W, 19.1 % of the pack over 354 h (was 147 W, 12.4 %) |
| terramechanics.js, terrain.js `soilAt`, config.js `REGOLITH.vary / zMax` | soil stiffness, cohesion (±30 %) and friction angle (±3°) vary over ~45 m patches; slip-sinkage digs in above 40 % slip; sinkage limit 0.25 → 0.5 m | the rover could not bog down and the soil was uniform. The on-board slip estimator still assumes nominal soil, so odometry no longer inverts the exact truth model |
| rover.js rocker-bogie | warp (diagonal) component of the corner spring forces removed while all four corner wheels are grounded | **bug**: parked on flat ground the wheel loads read 3.1 / 1.5 / 2.0 / 1.7 / 1.5 / 3.2 kN. Now 2.0–2.3 kN each. (With a corner wheel in the air the stiffness stays: without it the lander ram test rolled over) |
| sensors.js `_doCamera` | camera cannot decode within 12° of the sun, loses half its range with the sun in frame, and at night or in shadow sees cargo tags and rocks only inside 35 m of headlight | the camera worked identically in darkness and looking into the sun |
| sensors.js `switchLidar`, health.js `powerCycle` | starboard LiDAR head is a cold spare: FDIR switches to it when head A does not survive its power cycle | the model has two heads; the sim used one and a hard failure ended autonomy |
| sim.js `comms`, commands.js `comms earth / local / delay`, main.js | one-way operator delay (0 for habitat crew, 1.3 s for Earth): commands, E-STOP and stick inputs arrive late; delayed manual drive capped at 0.3 m/s | no signal delay existed |
| health.js `cool` signature | a hot hub that is cooling down is not "cooling path degraded" | with a real (slow) heat path the repaired hub re-alarmed: 1 spurious alarm in `fdir` |
| health.js `seize` signature | also requires the vehicle to be moving (> 0.12 m/s) | in a tight loaded skid turn some wheels stall at the current limit while others spin and the rover stands still; FDIR isolated 3–4 healthy hubs as "seized" in both dock tests (the v1.3.0 "three others turning" rule did not cover it) |
| autonomy.js docking steps | bellows "inflating" and "CO2 purge" → spring-loaded metal bellows, EDS pulse + brush-seal sweep | gas consumables with no supply modelled |
| props.js rocks | spherical UVs on the rock hulls | **bug**: `ConvexGeometry` has no uv attribute, so the rock texture sampled one texel (flat grey rocks) |
| scene.js | Earth ~10° above the horizon | the sun peaks at 9° (site ≈ 80° S); Earth was drawn on the horizon as at the pole |
| render/powerDock.js | vault sign reads the configured bank size | sign said 4.0 MWh |
| telemetry.js | `warp` column | rows produced by the kinematic transit warp carry no wheel-soil physics and must not be used as evidence |
| tests | `physics` top speed 2.7–3.1 m/s; `odo` speeds 0.5–2.9 m/s; `fdir` raises the sun for the sun-sensor mode if the rover stands in terrain shadow; `power_dock` allows 4.5 h for a 60 kW charge | follow the new limits |
| CLAUDE.md | simulator section | the file described only the prototype firmware (Python / ROS2), none of which is in this repo |

Not fixed, still open:
* Rocks are fixed bodies (a 25 cm rock does not move when a 23 t vehicle hits it) and soil traction is applied on rock.
* Total landed mass (module 15 t, container 8.5 t, rover 8 t, bank ~7.5 t, arrays) is not checked against a lander.
* The 420 kWh rover pack is not traced to an energy budget (a loaded 2 km leg costs a few kWh; a survival night 80 kWh).
* No line-of-sight or link budget for comms; the rock detector still reads true rock positions (range- and light-limited).
* Battery capacity does not fall with temperature (only its resistance rises).
* The demo day/night cycle (900 s per lunation) runs faster than the thermal time constants, so its temperatures mean nothing.
* One camera. `cycle`, `faults`, `nav_bench` were not re-run in this pass (25+ min each); their last saved results failed.
* `dock_cyl40` fails (1/5), on this code and on v1.3.0 alike: the loaded rover circles the PORT-1 lead-in point on
  9 m-radius turns, is declared boxed in and replans a 470 m detour until the test's time cap. The false "seized"
  isolations are gone; the navigation problem (see §22 "still open") remains.

Tests after this pass: `npm run test:quick` 14/15 (all but `dock_cyl40`), `fdir` 65/65, `vite build` OK.

## 24. Thermal environment, random thermal scenarios and the thermal brain (2026-11-10)

Design and numbers: [docs/THERMAL_BRAIN.md](docs/THERMAL_BRAIN.md). Code: `src/sim/thermal.js`, `src/ui/thermalPanel.js`.

* **Better temperatures.** The regolith surface temperature now follows the sun (radiative equilibrium, lagged):
  ~242 K at the 9° polar sun, ~344 K (71 °C) on a 30° sun-facing slope, ~95 K at night, ~60 K in a permanently
  shadowed crater. The vault, battery and wheel-hub radiators radiate to a sink built from it (half sky, half
  regolith) instead of the fixed 230 K / 100 K. New thermal nodes: LiDAR head, camera (film heater, thermo-electric
  cooler, sunshade) and the motor-controller bays (losses scale with drive power and the current allowed).
* **Random preplanned scenarios**, on by default in the browser (`?noscen` turns them off), one every ~3.3 min of sim
  time on average: sun-facing slope heat soak, crater-shadow cold snap, Earth eclipse, dust on the hub radiators,
  long heavy climb, sun glint into the sensors, vault heater stuck on, controller switching losses, cold-soaked
  wheel ends. `scenarios on|off|often|rare`, `scenario <id>|random|end`.
* **Thermal brain** (utility AI): predicts every temperature 2 min ahead from its rate (spike detection), scores the
  options of each channel and applies the best, escalating fast and relaxing slowly. Drive: **REV DOWN** 1/2 (rpm cap
  and torque cut, so I²R losses drop), **COOL-DOWN STOP**, **REV UP** (stator heating 120 / 350 W per motor: heat, no
  torque). Radiators: pre-cool, shed load, conserve (louvers shut; heater set points +3 K only while driving, never in survival). LiDAR and camera: heat, TEC cool, TEC cool + sunshade.
  Instruments that leave their range by 8 K are protected OFF; with the brain on that never happens in any scenario.
  `thermal`, `brain on|off`; panel **T** / THERMAL pill.
* Hooks: rover rpm cap / torque factor / stator heat into the winding loss and the power budget; health radiator
  sink, louver override, heater set-point bias, vault load; sensors skip a protected instrument.
* Night headlights 150 → 70 cd (the near ground still burned out in the camera at night).

Test `tests/thermal.mjs` (`npm test` entry `thermal`), 24 checks: with the brain the LiDAR peaks at 43 °C on a hot
slope with sun glint (op limit 50 °C); without it, 58 °C and protected off.

Tests after this change: quick set 15/16 (`lander`, `thermal` 24/24 and everything else pass; `dock_cyl40` 1/5 as on
v1.3.0, see §23), `fdir` 65/65 (survival night 17.5 % of the pack: the brain's heater set-point bias is off in survival).
`tests/power_dock.mjs` is truncated (syntax error at line 19, from §23) and is not in `npm test`.

## 25. Navigation brain, gentle turning, smart rpm v2, orbital-map SLAM fixes, rover memory store (2026-11-10)

Operator report: M6 "started heading toward a random place"; the route line keeps changing its mind; the rover stops and
spins instead of turning while it drives; turns are too aggressive; smart rpm should allow more (cap 280); SLAM and
mapping need work; the map needs a durable home; the per-wheel arrows should go (done in §22).

Measured first, on M6 at 50x (`WARP=1 node tests/autonomy.mjs cycle 1500`, default seed): in 25 sim-min the rover
drove 2,229 m to get 674 m closer to PORT-1, with pivots of 143°, 155° and 123° and a stretch driving away from the
goal; the SLAM error reached 5-9 m.

| Area | Before | After |
|---|---|---|
| Unmapped-ground price (planner.js) | fixed 1.5 /m, while mapped ground costs a median 6-8 /m (most ground is in some rock's soft inflation skirt), so every scan made the route ahead dearer than unmapped alternatives and the plan swung sideways or back | median cost of the mapped cells (smoothed across builds, clamped 1.5..6); A* heuristic weight scaled with it (1.07 x) so search time is unchanged |
| Plan commitment (autonomy.js replan) | periodic replan adopts any plan > 15 % cheaper | held 20 s after a switch unless blocked; then > 15 % AND > 12 m of typical ground cheaper; > 60° swing needs 30 % (> 100°: 40 %, as before) |
| Progress watchdog (navTo) | newly mapped ground counted as progress for up to 1 h, unbounded | only while within max(60 m, 10 %) of the best distance; > max(150 m, 25 %) further than the best = stop, ask for help |
| Steering authority (navTo) | none | moving, commanded to turn, gyro < 25 % of the command for 15 s = stop, ask for help (the §22 runaway shape) |
| Turning (navTo) | stop and pivot at > 50° heading error (> 100° within 6 m of the last pivot) | moving arcs up to 115° (> 150° within 6 m); speed eased by the heading change (x 0.35..1) and by curvature (lateral 0.3 m/s² empty, 0.15 loaded) |
| Arc set (planner.js LocalPlanner) | kmax 0.16 slow / 0.11 (inner side down to 39 % of the outer) | \|k\| x 3.84 m <= 0.5 below 1 m/s, 0.35 above 3 m/s (inner >= 50-65 %); the full set (0.2) only as a fallback at <= 0.5 m/s before a block is declared. Horizon up to 30 m (was 18: capped speed at ~4.2 m/s) |
| Wheel allocation (rover.js _allocate) | each wheel clipped at the rpm limit separately: in a fast turn the outer side saturated and the curvature changed | v and w scaled together, so the commanded curvature is kept |
| Smart rpm (rover.js) | 3 tiers 120 / 180 / 300 by thresholds, rpm up 150 rpm/s, down instantly | continuous: ceilings from slope, slip, roughness, tip margin, turn (outer side under the cap), motor temperature, battery and the thermal brain; lowest wins and is named on the HUD; filtered inputs; up 40 rpm/s, down 160 rpm/s; cap 280 (turbo 300 on clear straights) |
| Speed limits (config.js) | 3.0 / 1.5 m/s (§23) | 5.0 m/s empty, 2.5 m/s loaded (operator: "middle ground"); accel limits from §23 kept. 5.0 m/s is ~182 prototype rpm |
| Transit-warp cruise cap | 3.4 m/s empty | 5.0 m/s (still under speedCap and the governors) |
| Scan-matcher degeneracy (slam.js) | sharpness tested on world x / z | tested along / across the rover's track; only sharp components applied |
| Map-relative localization | none: fiducials only at pad / habitat / dock | `src/sim/orbitalMap.js`: every 40 m the local SLAM heights (plane removed) are correlated with an orbital DEM [ASSUMED: 4 m posting, 0.3 m noise, ±0.3 m warp]; the 3 newest agreeing matches are averaged and applied through the pose graph (`Autonomy.applyFrameShift`) |
| Map storage | IndexedDB in one browser; maps keyed to a seed that is random each load | rover memory store on disk (`scripts/memory-plugin.mjs` on the Vite server, `src/sim/memoryStore.js`): `~/.hunch22-data/memory/site-<seed>.bin` (gzip), 10 history snapshots per site, shared by every launched version; `?site=last` reopens the newest site; `memory` command; IndexedDB kept as fallback |
| FDIR seize false alarm | (§22) | kept |

Results, same M6 run (25 sim-min, 50x): 400 m to go (was 1,335 m), SLAM error <= 3.3 m with 4 orbital-map fixes
(was up to 9.3 m). Quick suite 15/16: physics 14/14 (top speed 4.95 m/s), odo 10/10 (exact at 4.5 m/s), planner,
rocks, lander, thermal, ai_copilot, creep, both pickup_gt, both pickup_time, dock_box20 (8/8), scout, m1 all pass.

Still failing, not caused by this change: `dock_cyl40`. With v1.4.0 it circled the PORT-1 lead-in until the time cap;
now it reaches the port, closes the ring-camera approach to 1.4 cm at 3.4 m, and then the loaded rover's heading
drifts 0 -> 1.8 deg over the last 2 m although the curvature law keeps commanding the correction, so it arrives
12 cm off the axis (cone 6 cm). The full M6 cycle therefore picks up CM-40A and then fails at that port (the
progress watchdog now stops it after 4 min instead of letting it wander). An uncommitted rework of rover.js
(geometric wheel contact point instead of the cast witness point, combined CoM, energy booking; with
tests/scratch/dbg_drive_under.mjs and dbg_park_*.mjs) fixes that final alignment in testing, but it also breaks the
20-ft container pickup (3 failed drive-unders) and leaves a 5.9 kN port contact force. It is from another session
and unfinished, so it is NOT in v1.5.0 and stays uncommitted in the working folder for its owner.

## Fix (2026-11-11): team CAD viewer crash

`public/team/js/fusion_rover.js` used `mWhite` (Z-93 radiator panel, line ~430) without defining it, causing `Can't find variable: mWhite`. Added the material next to `mCopper`. Visual only; no physics or sim behaviour change.

## 26. Model and test review (2026-11-11): container sled, supply building, low-battery crash, Mission 10 / 11 / 12 tests

Review of every 3D model in the running sim (rover empty and loaded, both cargo types, stands, scout, lander, supply
building, power dock) and of the test evidence. Changes:

| Item | Before | After |
|---|---|---|
| 20-ft container sled (`src/render/props.js`, `src/main.js`) | the keel was a capped partial cylinder turned 90 deg off the bottom (drew as a large wedge on the side), the two clamp rings were single-sided hoops with nothing joining them to the box, and the trunnion pins floated in space | `buildRingAdapterSled`: two clamp rings with thickness, seven spokes each to the box, the keel shroud with the same section and ends as the physics hull (`world.js`), U-hoops and tie beams at both lug stations, longerons and aft braces. Visual only: colliders unchanged |
| Shared material | the old keel set `MAT.steel.side = DoubleSide` for every steel part in the scene | the sled has its own materials |
| Supply building (`props.js buildHabitat`) | a blank 20 x 46 x 15.5 m box | frame ribs (left off the docking face at the port vestibules), panel seams, plinth, parapet, roof radiators, hatch and high-gain dish, larger sign. All within 0.12 m of the wall collider |
| Low battery (`autonomy.js navTo / chargeAtDock`) | unloaded and below the 20 % reserve at the start of any leg (or `charge` below 20 %): `checkPower -> chargeAtDock -> navTo -> checkPower -> ...`, mission aborted with "Maximum call stack size exceeded". Present in v1.5.0 | the drive to the charger is marked `toCharger` and skips the reserve check |
| Mission 10 tests | 2 of 8 fault scenarios tested | `tests/faults_m10.mjs` (28 checks): LiDAR, camera, comms, low battery, cargo shift, port unavailable |
| Mission 11 / 12 tests | none for the logistics manager or active levelling | `tests/logistics.mjs` (12 checks) |

Measured: quick suite 18/18 files (the 16 earlier ones plus `logistics` and `faults_m10`), `dock_cyl40` 9/9 on this
working tree. Scene: 1,856 meshes, ~1,430 draw calls and 5.2 M triangles per frame (four shadow cascades included),
60 frames/s on the M4 Mac mini. Loaded static tip margin 28.0 deg (40-ft module), 31.0 deg (20-ft container); wheel
sinkage 2.8 cm loaded.

Not changed, noted for later: the scout's shadow starts ~2 m from its tracks (shadow-map bias on a 1 m object); the
lander is a plain cylinder above its descent stage; the draw-call count is high for laptops without a discrete GPU
(the quality ladder in `main.js` handles it by dropping shadows and effects).

## 27. Follow-up to §26 (2026-11-11): survival recovery, watchdog clocks, shadows, lander, draw calls

| Item | Before | After |
|---|---|---|
| Keep-alive array (`health.js _array`) | survival mode asked for the array once, at entry. Entered while still rolling (battery ran down mid-drive) the deploy was refused ("refused while moving") and never asked for again: the rover sat with the array stowed until the pack died | asked again as soon as the rover has stopped in sunlight (deployed 91 s after the fault, 4.8 kW) |
| Mission after survival (`health.js setSurvival`) | stayed paused until an operator typed `resume`, which cannot arrive during a comms loss | a mission paused by a low-battery survival entry resumes 8 s after the self-test, once SoC is above the 30 % exit level; the array stows first |
| Watchdog clocks (`autonomy.js update / navTo`) | time spent paused, powered down or in an FDIR hold counted against the 20 s stuck check and the 4 min progress watchdog: the resumed mission failed at once with "no progress for 4 min", and any hold over 20 s was followed by a false "No progress for 20 s" recovery and a blocked-area mark | `frozenS` counts the time the executive did not run; navTo takes it off `stuckT`, `bestT`, `t00`, `lastReplan` |
| Shadow bias (`scene.js`) | -0.00025 of the 6 km light range = 1.5 m: SCOUT-1, small boulders and footpads cast shadows that started a body length away | -0.00005 (0.3 m), normal offset 2 cm; no acne seen at a 10 deg sun |
| Lander (`props.js buildLander`) | bare cylinder and cone on the descent stage | ring frames, stringers, interstage band, cargo-bay door, RCS quads, side struts on each leg, antennas; all within 0.2 model units of the colliders |
| Draw calls (`src/render/mergeStatic.js`) | every small part its own mesh: 3,478 calls per frame at the pad, 3,543 at the habitat (top quality rung, four shadow cascades) | static parts merged per material on the lander, supply building and base structures, power dock, stands, 40-ft module, container sled and the six tyres: 1,588 at the pad (-54 %), 2,295 at the habitat (-35 %). `?nomerge` turns it off. Parts that move (sun-tracking heads, charge segments, port doors, struts, jaws) are left alone |
| Tests (`tests/faults_m10.mjs`) | 28 checks | 44 checks: adds LiDAR loss on a proven route, battery run-down mid-drive with recovery, camera dead at the port |

Camera dead at the port needed no change: the rover already refuses the berthing, backs off with the module clamped and
reports "docking visual sensor lost" (0 N on the port).

Still open, M6 full cycle. `cycle` (12,000 s limit) on this working tree: no collision and no abort, but not finished.
0-55 min survey drive pad -> PORT-1 -> pad; 105 min CM-40A delivered; 129-150 min CC-20B pickup (two "not centred under
the module" retries, one box-in); 162-184 min loaded on the remembered route near (-336, -296): "close obstacle along
remembered route", three box-in unwinds while the SLAM error grew from 2.4 m to 13.5 m, then free again; 661 m from
PORT-2 at the limit. The time is lost in navigation (navTo / planner), which another session is working on.

Measured afterwards with the limit raised to 16,000 s (`node tests/autonomy.mjs cycle 16000`, same working tree): the
cycle completes unaided. CM-40A delivered at 105.0 min, CC-20B at 231.3 min (PORT-2 soft capture 1.4 mm lateral,
3.1 mm vertical, 0.16 deg), mission complete at 250.0 min = 14,999 s, 17.3 km driven, 26.2 kWh. So M6 is 25 % over its
12,000 s limit, with seven box-in unwinds on the loaded CC-20B leg (162-175 min and 203-214 min) as the main loss.

## 28. Review pass (2026-11-11): view framing, console noise, survival details

| Item | Before | After |
|---|---|---|
| Rover hidden by the instruments panel (`main.js`) | at 1440 x 900 the followed rover was drawn in the middle of the window, under the dynamics-instruments panel | the picture is shifted (camera view offset, eased) so the view centre sits in the middle of the band the panels leave free at the window's centre; recomputed when panels are toggled or the window is resized; off in the onboard view |
| Console noise (`main.js`) | "Rover memory saved ..." logged on every SLAM correction (twice in the first 16 s of a drive) | the automatic save is silent; `map save` still reports |
| Help card (`hud.js`) | "Smart rpm picks 120 / 180 / 300" (the v1.4 tiers) | 120 to 280, continuous |
| Keep-alive array request (`health.js`) | §27 retried the deploy whenever survival was on, so an operator `array stow` was undone at once, and a survival entry at night never deployed at sunrise | a `pending` request lasts as long as survival does and is cleared by a successful deploy or an operator stow |
| Auto-resume after survival (`health.js`) | §27 resumed any paused mission | only one that survival mode itself paused |

Checked with no finding: no console errors or exceptions while switching the five views, toggling every overlay and
X-ray, and running Mission 3 at 20x; no non-finite transforms in the scene afterwards.

## 29. HUNCH-brief review (2026-11-11): third cargo system, port coupling, seal interlock, idle solar

| Item | Before | After |
|---|---|---|
| Cargo systems (`config.js CARGO_TYPES.pod4`, `terrain.js`, `props.js buildPodRack`) | two (40-ft module, 20-ft container); the brief lists three | adds CP-36C: 36 Astrobotic-style pods (4 ft x 2 ft) on a 20-ft rack, same ring interface, destination PORT-3 (Missions 3 and 8, category `power`). Ports now at z = -11.5, 1.5, 14.5 m. Console accepts cargo A/B/C and port1-3 |
| Port coupling (`config.js DOCK.bellows`, `sim.js _bellowsStep`) | rigid wall: capture contact force 2-6 kN, `dock_cyl40` failing | spring-damper metal bellows; peak 950 N for all three cargo types (gate 3,000 N, now checked for every cargo in `tests/dock.mjs`) |
| Berthing order (`autonomy.js`) | seal compressed first, dust shield after (traps regolith in the seal) | dust-shield pulse + sweep with faces 30 mm apart, hard capture, bellows shroud, leak check, equalise, jacks, hatch |
| Leak check (`autonomy.js`, `faults.seal`) | always passed | `fault seal` fouls the seal: hatch stays closed, clean and re-seat once, second failure holds for the operator (`clear seal`, `resume`). `A.lastBerth` records leak-pass time and vestibule pressure |
| Soft-capture dwell (`autonomy.js`, `config.js`) | the dwell needed the ring under 30 mm, but the bellows rests ~32 mm proud at 900 N, and one ring-camera sample past 5 mm restarted the timer; with some noise streams capture failed three times (1 of 4 streams) | gate on `DOCK.captureGap` 36 mm / `softCaptureN` 750 N, timer holds within 1.6 x the 5 mm limit; 4 of 4 noise streams capture on attempt 1, peak 0.8 kN |
| Idle solar (`health.js`, `main.js`) | no generation except the emergency array | parked idle in sunlight for 20 s the 12 m2 array deploys (4.8 kW at noon) and stows before any drive; browser default on (`?noidlesolar` off), headless default off |
| Self-park (`autonomy.js missionCycle`) | returned to pad staging after the last delivery | when nothing is waiting the rover parks in the power dock and charges |
| Saved maps (`slam.js MAP_LAYOUT`) | maps from the old two-port layout were reloaded | maps carry a layout number; older ones are ignored |
| Copilot prompt, fault procedures | two cargo; gas-strut text | three cargo; regenerated `docs/FAULT_PROCEDURES.md` |
| M6 cycle limit (`tests/run-all.mjs`) | 12,000 s for two deliveries (measured 14,999 s) | 24,000 s for three deliveries |

Judge/space-realism notes (not changed in code): SCOUT-1 renders as the printed prototype; a flight vehicle would use
metal-link tracks (rubber is brittle in lunar cold). Top speed 5.0 m/s empty is at the edge of crewed rover practice
(Apollo LRV 18 km/h peak); the rocker-bogie layout is normally a low-speed design and the per-wheel struts are the
mitigation. Prototype BOM parts (Raspberry Pi, Arduino) are Earth-demo stand-ins per the HUNCH "Suggestions" section.

Open, M6 full cycle with three cargo (`node tests/autonomy.mjs cycle 24000`, 2026-11-12). Not passing. Run 1: aborted at 57 min on the
empty survey drive back to the pad ("no progress toward survey"). Run 2 (different ring-camera noise stream): survey fine,
CM-40A delivered at 128 min, CP-36C picked up, then at 206 min 75 m from PORT-3 on the loaded leg the steering-authority
check stopped the mission ("yaw rate < 25 % of command for 15 s"), one of three delivered. With the pod rack removed the
survey drive completed. The failures are navigation / traction events on the long traverse that depend on the noise stream
(the same area as the 14,999 s two-cargo run's box-in unwinds), not on docking: all three cargo types dock in their own tests.
The 24,000 s limit in `tests/run-all.mjs` is therefore a guess that has not yet been met by a passing run.

## 30. v1.7.0 work order (2026-11-13/14): straight-to-cargo mission, time warp, suspension and wheel drawing, sensors, HUD, navigation

Source: the AEGIS-V1 "master system specification" (nine tasks), followed by a review pass (2026-11-14) in which every change was
measured against the original code on a clean `git archive HEAD` copy. The review found that several first-pass changes followed the
spec's stated root cause without checking it, and some were wrong; this section describes the tree as it is after the review, and says
for each task what the real cause turned out to be. Constants are in `src/sim/config.js` (`MISSION`, `NAV`, `PHYS`, `ROVER.lift.sCurve`,
`ROVER.lidar`, `ROVER.camera`), tagged `[ASSUMED]` unless noted. Measurement scripts: `tests/scratch/ab_departure.mjs`, `ab_nav.mjs`,
`dbg_ride_body.mjs`, `dbg_wheel_visual.mjs`, `dbg_rise_impact.mjs`, `dbg_loaded_lift.mjs`, `dbg_stereo_quality.mjs`, `dbg_layout.mjs`.

**Task 1: full-mission loop and rpm.** `missionCycle` no longer starts with `surveyRoute` (an empty 2 x 2 km survey drive and a 60 s fiducial
hold before touching the cargo). It is opt-in: `MISSION.preSurveyRoute` (default false). The loop already re-planned for the next manifest
item, drove back through `acquire`'s pre-grasp leg and recharged through `navTo`'s `checkPower`; an explicit SoC <= reserve check was added
at the top of every leg. Turbo (300 rpm) stays empty-rover only: the loaded rover's speed cap (2.5 m/s) and tip governor, not the rpm ceiling,
set its speed. New `Rover.crawlZone` (smart-rpm entry "precision zone crawl", 120 rpm): from the pre-grasp point through the lift and the stand
exit, and for the last `MISSION.crawlDockM` = 20 m of the port approach through soft capture; cleared on exit, abort and mission end. (120 rpm
is 3.3 m/s at full scale, above every speed used in those phases, so the cap changes the displayed tier, not the motion.)
Trade-off kept on purpose: without the survey the first loaded leg explores unmapped ground (no point turns while loaded).

**Task 2: time warp.** Measured cause: full physics tops out at ~3x real time on this machine (headless, one thread: 2.8x before, 3.1x now), so
no frame budget can deliver 5x, 10x or 20x. What was done:
- `PHYS.railsMinScale` = 5 (was 50): the kinematic transit warp the 50x mode always used now carries long cruise legs from 5x up;
  `Sim.stepAuto(owed)` sizes each rails step to what the frame owes, so the rover moves every frame instead of in 0.5 s jumps.
- `Sim.advance`: per-frame physics budget `min(budget x min(scale, 2.5), 50 ms)`, step cap `min(40 x scale, 240)`.
- `main.js`: the auto-quality governor is frozen above 1x (it read warp load as a slow GPU and cut both the graphics and the physics budget);
  a draw throttle acts only while the warp is CPU-limited, drawing every 2nd-4th frame if a drawn frame's non-physics time is large, never
  less than ~8 draws a second.
- First pass, since removed: a 160 ms budget (6 frames a second) and a fixed "draw 1 frame in floor(scale / 2)" (2-6 draws a second even
  when the CPU was idle on rails).
- HUD: 3x button, `.on` set at the click, `#warpIndicator` badge (`▶▶ 10× WARP`, `❚❚ PAUSED`, `◀ 0.5× SLOW`), "actual N×, transit warp /
  CPU-limited" in both HUD layers. Pause stays the `paused` flag (keeps the scale for resume).
- Measured in a software-rendered headless Chromium on an idle machine, empty leg to the habitat over unmapped ground (set -> actual):
  2x -> 1.7x, 3x -> 2.1x, 5x -> 4.6x, 10x -> 6.6x, 20x -> 10x, ~20 draws a second throughout; rails carried 75-82 % of the sim time
  from 5x up. The set speed is reached on rails legs only: pick-up, docking and unmapped or tight ground run full physics (~2-3x).

**Task 3: sensors.**
- LiDAR mount: NOT the spec's centre mast at y = k(0.35) = 5.7 m. That point is inside the carried module (underside 5.2 m at ride 0), and
  the team's own trade study (`public/team/js/lidar_mount.js`, options A-D) rejected a mast for that reason and chose fixed deck-corner
  scanners under the module, which is what the model draws and where the sim had it. The first pass moved it to the mast; the review moved
  it back. One line in `ROVER.lidar.mountLocal` changes it if the team decides otherwise.
- Scanner: 32 beams over -30..+15 deg (was 16 over -30..+10), same 10 deg nose-down pitch. Rated 10 Hz x 360 steps is 115 000 rays/s, 6x
  the old cost; the first pass ran 35 000-115 000 rays/s and made `pickup_gt` time out and docking outcomes change. Now `tiers` is the old
  scanner's own schedule (5 Hz x 240, 8 Hz x 150, 12 Hz x 100) and the 32 beams fire as two interleaved sets of 16 on alternate sweeps:
  19 200 rays/s as before, all 32 rings in the map over two sweeps. At time scale s the scan rate is divided by s (min 1 Hz).
- `_selfMask` drops returns inside the chassis box and (loaded) the carried module's enclosing cylinder. The ray queries already skipped the
  rover's and the held cargo's colliders (collision groups); the mask is the explicit second line. Known simplification, unchanged: rays pass
  through the vehicle instead of being blocked by it.
- Camera: 90 deg HFOV stereo depth (`_doStereo`, 32 x 18 grid at 4 Hz, sigma_z = z^2 sigma_d / (f B)). `Slam.integrate`: stereo cells are
  provisional (`tile.st`): fused only at sigma <= `fuseSigma` 0.10 m (~14 m) into cells with no LiDAR measurement, averaged over up to 8
  stereo samples, replaced outright by the first LiDAR return, never used for scan matching or loop closure. Measured: stereo alone (LiDAR
  off) maps 5 800 cells in 60 m with 0 false rock / slope / lethal cells and 4.7 cm height RMS (the first pass's 0.25 m gate gave 30 %
  rough cells and false slopes). With the LiDAR on it adds almost nothing (15 provisional cells at most): its value is the map it keeps
  building if the LiDAR is down. The LiDAR-fault speed policy was not changed to use it.
- LiDAR and stereo range noise have their own random stream (`gaussRange`): they shared the IMU / camera / alignment-sensor stream, so any
  change to the scan changed every precision sensor's noise and the millimetre docking outcome with it.
- Planner hard inflation: 6.5 m loaded (`NAV.inflateLoaded`), 5.8 m empty; `navTo` sets it from the payload state.

**Task 4: wheels and suspension.** Two real faults, neither the one the spec named:
- Drawing (`aegis.js`). `const ride = cl(rover.rideOffset / K, -0.045, 0.065)`: `cl(v, m)` clamps to +-m and ignores a third argument, so
  this was `max(0.045, min(-0.045, v))` = +0.045 for every input. The wheels were always drawn 0.73 m below their nominal position (0.73 m
  into the ground at ride 0, 1.39 m at full kneel) and the strut graphics never moved. Now `drawnRide()`: a real range clamp, and fed
  with the ACTUAL mean strut extension, not the commanded ride. Measured in the browser frame by frame (130-148 frames per move, terrain
  clamp off): drawn hub within 0.1 cm of the physical hub through kneel, lift and squat; the only thing under the surface is the tyres' real
  2 cm of sinkage. The terrain clamp from the first pass stays as a guard (it alone hid the sinking and left the cause in place).
- Motion (`rover.js`). The body followed the ride command through the levelling servo only (0.30 m/s rate limit, 0.6 /s gain): a
  kneel-to-carry lift took 8.6 s of body motion, started with a step in speed, and the body trailed the command by up to 0.9 m. The first
  pass put an S-curve on the command and tested the command (3.73 s); the body then took 9.3 s. Now: the S-curve (minimum-jerk quintic; any
  stroke >= 0.5 m takes at least 3.7 s) is fed forward to the actuators with a lead term for the dampers' velocity lag (c / k). Measured on
  the body: squat-down settled within 2 cm in 3.1 s, lift in 3.4 s, peak 0.31 / 0.63 m/s^2, overshoot < 1 cm, body within 1.4 cm of the set point.
- Soft seat. One fast stroke meets the module on its stand at ~0.65 m/s: a 19 m/s^2 (12 lunar g) jolt that bounces it off the saddles. The
  lift controller now stops 3 cm under a module overhead (cradle sensors, `grappleCheck().seatErr`) and lifts from rest: jolt 2.6-3.7 m/s^2
  (the old servo: 7.2), struts at carry 5.4 s after the command (old: 9.1 s), 0.8 cm strut overshoot. So a pick-up rise is two strokes,
  ~2.4 s to the seat and 3.7 s of lift, not one 3-5 s stroke as the spec's acceptance line reads.
- `acquire`'s `waitRide` and the stand-exit check now wait for the struts (`Rover.rideActual`), not the set point.

**Task 5.** `docs/LUNAR_CARGO_RESEARCH.md` was already present and was not changed.

**Task 6: stand exit.** The spec asked for a straight reverse. Measured: every stand on the pad points its module exactly at the habitat
(`dbg_layout.mjs`: 0 deg), so forward is the direction of travel and the far end of the stand is open. The original forward exit: 11.9 m in
26 s, 0.11 deg of heading change, 2 cm of drift, 0 N of stand contact on the module, 200 m from the stand 3.4 min after the clamp. The
first-pass reverse exit: the loaded rover, which cannot point-turn, was left behind its own stand facing it and did not get 30 m away in
25 min (4 boxed-in recoveries). Now: forward again (`MISSION.standExitDir` = +1; -1 is still there), heading held with curvature limited to
0.05 /m, starting only when the struts are at carry and the jaws read closed. Measured: 0.03 deg, 1 cm, 0 N, 200 m in 2.9 min, 0 warnings.

**Task 7: HUD.** Three layers on `#hud[data-layer]`, cycled 1 -> 2 -> 0 -> 1 by H, the `☰ HUD` button, or `Hide All HUD`; kept in localStorage.
Layer 0: canvas, a faint ☰ and the E-STOP-latched banner (Esc still stops the rover; the E-STOP button is hidden with the rest). Layer 1: a slim
control bar (clock, mode, time scale, M6 / resume / abort, HUD buttons, RESET / START / E-STOP), speed + rpm tier badge, battery bar / draw /
endurance, phase -> destination | distance, warp status, a 200 x 200 route minimap, and an alarm banner only while something is wrong. Layer 2:
every existing panel. The help panel moved to `?`.

**Task 8: navigation.** Lookahead >= 10 m (`max(10, 3.5 v)`); +-3 deg heading deadband (hysteresis to 4 deg) on gentle paths; 0.1 rad/s^2
slew on the navigation's yaw command; on ground tilted < 5 deg a 0.5 m/s floor (was 0.3) through up to 12 m of unmapped cells; a LiDAR dropout
derates the leg only after 2.5 s; routine replans switch the path only for a lethal obstacle within 25 path points or a 30 % cheaper plan, and
hold a switched path for 15 m (not a path whose cost is already infinite: fixed in the review). Yaw damping: the first pass halved the yaw command
whenever the heading error was < 5 deg and the yaw rate > 0.03 rad/s, which also fired on every gentle curve; it now acts only on a straight
stretch while still rotating toward the target. There was no "+-1 deg deadband" or "SLAM confidence watchdog" in the code as the spec assumed.
Measured, empty rover, first 10 sim-min toward the habitat (original -> now): 857 -> 993 m, stops 1 -> 0, time under 0.4 m/s 121 -> 16 s, path
switches 54 -> 27, yaw-command reversals 3.1 -> 2.0 /min, warnings 3 -> 0. The original showed ~2 yaw reversals a minute on this leg, so the
"wiggle" itself was not reproduced; what improved is creeping and path thrash.

**Found in the review: the solver's position grid (`sim.js _precisionPost`, `PHYS.posGuardMaxSpeed`).** A pre-existing fault, and the real
cause of low-speed sideways drift. The physics engine moves a body in steps of 2^-11 m = 0.49 mm per axis (8 float32 ulps at these
coordinates; `tests/scratch/dbg_quant.mjs`), so every step's displacement is v dt rounded to that grid: up to 0.03 m/s of error per axis at any
speed, while the body's velocity (and so the wheel odometry) is unaffected. Commanded straight on the flat pad the rover travelled 6.2 deg left
of its heading at 0.131 m/s for any command from 0.12 to 0.16 m/s, -4.2 deg at 0.3 m/s, -2.8 deg at 0.5 m/s (`dbg_crab_speed.mjs`). A float64
guard already corrected this, but only below 0.1 m/s, and the pick-up's final squaring speed is 0.12 m/s: in the last metre under a module the
rover slid ~10 cm sideways whatever the controller asked. The original code got away with it because its camera estimate left the rover ~6 cm
to the right at the hand-off to the saddle rangefinders and the slide carried it through the centre; with the wider camera the rover arrives
centred and the slide carried it out (pod rack: 7 of 12 first-attempt pick-ups, one outright failure). The guard now runs up to 0.8 m/s and
integrates the velocity of the body origin (v + w x r; it used the centre-of-mass velocity). Its old limit came from a stale-broadphase fault
(the chassis drove through a boulder) that the later fresh-rock refresh removed: with the guard always on, the 1.4 m boulder still blocks at
0.3 and 0.6 m/s (`dbg_rock_shadow.mjs`), and `rocks`, `lander`, `physics`, `creep` pass. Measured after: 0.00 deg of crab from 0.06 to 0.7 m/s
and the commanded speed; pod-rack pick-up first-attempt in 13 of 14 noise realisations (`ab_pickup_noise.mjs`; original code on the same 14: 12,
and its other 2 failed outright), CM-40A and CC-20B 12 of 12.
Not changed, noted: `acquire` retries with `RD.cruise`, which does not exist in `ROVER.ride`, so a retry re-approaches kneeled.

**Tests.** `tests/v17_suite.mjs` (51 checks: body motion, drawn-vs-physical wheel height, LiDAR mount / interlace, mask, stereo cell semantics and
false hazards, time-warp budget and rails step, crawl cap, no-survey start) and `tests/stand_exit.mjs` (soft-seat jolt, struts at carry before
moving, forward distance, heading, drift, stand contact, 100 m clear, no boxed-in recovery); both in `npm run test:quick`. The first-pass versions
tested the set point and a reverse exit, and passed while the behaviour was wrong.

## 31. v1.7.0 release pass (2026-11-14): the full three-delivery cycle completes; navigation recovery, route straightening, landmark and pick-up fixes

Starting point: the §30 tree passed `npm run test:quick` (23/23 files) but the M6 cycle (`node tests/autonomy.mjs cycle 24000`) failed: CM-40A
delivered, CP-36C delivered after two box-ins on the loaded PORT-3 leg (89-119 min), then "no progress toward lead-in PORT-2 for 4 min" at
187 min, 2 of 3 delivered. Each change below was found in that run or in a run of the change before it.

| Item | Before | After |
|---|---|---|
| Leg failure (`autonomy.js navTo / _retryPrep`, `NAV.legRetries` = 2) | any navigation give-up (no progress 4 min, boxed in after 4 unwinds, moving away, steering not answering, pivot stall) ended the whole mission | the leg is retried twice after an autonomous recovery: back out of the pocket (unwind; reverse on a clear arc for steering / stall), keep the obstacle marks the failed attempt learned, force an orbital-map position check, and switch route source (a remembered route that failed is retried as fresh exploration and the other way round). Then the operator is asked. Not retried: "no path to goal". The drive to the charger gets one retry |
| Proven route near an obstacle (`navTo` squeeze test) | a close obstacle (< 11 m free) beside a remembered route made the loaded rover leave the route and explore next to the obstacle: it boxed in at the same spot in every M6 run (§27, §29) | on a route already driven with this load the local planner steers past on the route; only a route that is actually blocked splices or replans |
| Route memory (`planner.js straighten`, `NAV.straightenRoutes`) | routes were stored as driven: R1 kept every probe of the first loaded exploration (2.6-2.9 km for a 2.0 km straight line) and each repeat drove them again | when a route is stored, every stretch whose chord runs over ground already mapped, outside the hard margin, no costlier than the stretch, and passes the full wheel + belly footprint check with the route's load at every metre, is replaced by the chord; loaded corners rounded at R 18 m. Never over unmapped ground. The driven line is kept (`rawPts`, moved by frame corrections like the route itself, saved with it) and offered as the fallback: a chord crosses ground first seen from a distance, and closer scans later found rocks on two of R1's chords, so the loaded leg threw the whole route away until the driven line was kept; a candidate that needs no long bridging detour is preferred. Measured: R1 2,630 -> 2,376 m, R2 2,462 -> 2,324 m, R3 2,515 -> 2,343 m; the empty return to the pad 30.0 -> 20.8 min |
| Landmark estimates after a position fix (`slam.js shiftFrame warpAt`, `autonomy.js applyFrameShift`) | port and cargo estimates got one rigid correction about the current pose: a 0.7 deg heading fix at the habitat swung a cargo estimate at the pad, 2 km away, by ~25 m ("CC-20B: cargo estimate disagreed with 10 sightings by 34.6 m"), and the next pick-up approached a wrong spot | estimates move exactly as the map cells seen at the same time do (warp at their time stamp); seen before the previous fix, not at all |
| Sightings of the carried module (`updateEstimates`) | the cameras see the held module's tags the whole loaded leg; fusing them dragged its pad estimate 2 km with the rover ("disagreed ... by 2004.9 m") | tags of held, docked and delivered cargo are ignored |
| Steering-authority check (`navTo`) | instantaneous: yaw rate < 25 % of the command for 15 s. The normal lag of a loaded skid-steer turn (the command reverses on an S-bend before the yaw rate has built) counted, and stopped a loaded leg 75 m from PORT-3 (§29) | compared on 6 s averages in the commanded direction, against `NAV.noSteerRatio` = 10 %: measured, a healthy rover achieves 40-55 % of the commanded yaw rate (skid-steer scrub) and one with a free-rolling wheel 15-25 % (`tests/scratch/dbg_steer.mjs`), so 25 % on averages stopped the five-wheel traverse of `tests/faults.mjs`. A body that really does not yaw still trips it in ~15 s, and now it leads to a retry, not an abort |
| Back-out after a failed drive-under (`acquire backOut`, `backUp`) | reversed with the normal rear-hazard check, which stopped it part-way under the pod rack ("Reverse stopped: obstacle behind the vehicle"); the next attempt raised the struts and drove off wedged under the rack: no yaw response, wheels spinning, SLAM error 2.4 -> 16.6 m in a minute | reverses on the line the drive-under came in on with the heading held and the relaxed limits (as `unwindPath`), retries once; if the deck front is still past the module's aft end (geometry on the module axis, not the distance reversed: an attempt abandoned 4.7 m short of the module never had the deck under it) the pick-up stops for the operator instead of raising into it |
| Pick-up retry ride height (`acquire`) | compared with `RD.cruise`, which does not exist: never raised, every retry re-approached the module kneeled (0.65 m lower) | raises to the driving ride (0) once out from under |
| Pinned / spinning in place (`navTo`, `_scanSample`, `_scanSame`) | only the unwind could tell; while driving, wheel odometry carried the estimate away from a vehicle that was not moving | every 8 s: wheels turned > 2.5 m, heading unchanged, and >= 75 % of the LiDAR scene within 0.2 m of where it was -> the estimate is put back, the spot marked, recovery; 4 times in a leg -> retry / operator |
| Thin posts (`planner.js buildCostmap`) | light masts and array posts (r 0.2-0.45 m) do not show in the LiDAR height map, so only the base-map stamp keeps the rover off them, and the hard inflation (5.8 / 6.5 m) is a disc sized for the half width: the 9.8 x 9.9 m frame's corners reach 6.95 m. On a second site (seed 20261114) a frame corner caught an apron light mast 6.8 m from the centre and pinned the loaded rover 20 m from PORT-1 for 10 min (found by the pinned detector below) | the local planner (which drives the arcs, and cuts inside the planned line on a bend) now tests every surveyed post against the rover's rectangle, grown to the carried module's when loaded: an arc is refused where a post comes within 0.6 m of it and closer than it is now (so a rover already touching one can still move off). A wider disc in the global costmap was tried first and dropped: it made the pad-apron masts close the start of the proven loaded route R1, and the loaded leg explored instead |
| Tip safety in recovery (`unwindPath`, `navTo`, `NAV.upsetDeg` 25) | the unwind follows its entry trace even where the rear arcs read blocked, stopping when the FILTERED (0.8 s) tip margin fell under 10 deg; reversing a loaded rover over a crater rim it read 10+ deg until the vehicle had gone over (a v1.7.0 test run: tilt 36, then 64 deg), and the leg retry then kept trying to unwind | the unwind stops on the raw margin, at 15 deg loaded (10 empty); past 25 deg of pitch or roll navigation stops with the brakes set and no retry; no retry starts on a steep tilt (> 12 deg or tip margin < 18 deg loaded); after "boxed in again after 4 reverse recoveries" the retry does not unwind again |
| Unwind after an earlier unwind (`_unwindTrace`) | the way back is the breadcrumb trail read backwards, and it held the earlier unwind's own reversing: read backwards that leads back INTO the first pocket. A second box-in near the first one reversed a loaded rover over the first pocket's rim until it tipped (36 deg, then 64 deg) | earlier unwinds are jumped over together with the stretch of entry path they retraced; the trace resumes at the pre-unwind breadcrumb nearest that unwind's end, or stops |
| Route memory rejection (`routeFromMemory`) | "R1 is now blocked - exploring a new route", no reason; a stretch that could not be detoured locally threw the whole 2 km route away | the reason is logged (e.g. "no local detour around (-440, -342)"); a stretch the local repair cannot get round is bridged by a longer A* detour (larger search budget) to a point up to 300 m further on where the route is clear again; a route found obstructed mid-leg is re-joined through the same search (driven line, bridging) before the leg drops to exploration (up to 3 times a leg): exploring from there sent a loaded rover round the back of the habitat into the power-dock yard, where it boxed in 25 m from PORT-2's pre-dock point |
| Orbital-fix log | "149941.7142857143 map cells re-placed" (8 values per moved cell, divided by 7) | divided by 8 |

**Result.** M6 full cycle (`node tests/autonomy.mjs cycle 24000`), three deliveries, on two sites:

| Site | CM-40A | CP-36C | CC-20B | Mission complete (incl. self-park, charge to 95 %) | Driven | Energy | Autonomous retries |
|---|---|---|---|---|---|---|---|
| default seed | 59.5 min | 125.5 min | 197.7 min | 286.3 min (17,177 s) | 13.8 km | 24.0 kWh | 0 |
| seed 20261114 | 49.8 min | 107.4 min | 210.4 min | 302.4 min (18,147 s) | 15.0 km | 26.0 kWh | 1 (PORT-2 pre-dock) |

Before this pass the same test failed on the default site with 2 of 3 (§30 tree) and, on v1.6.0, aborted on navigation events.
Docking at the last port: 2.6 / 4.8 mm lateral, 0.67 / 0.78 deg, 754 / 751 N contact. Later loaded legs reuse and re-join their routes instead of
re-exploring. Box-in unwinds over the whole cycle: 4 (default) and 6 (second site), mostly on the first loaded leg, which explores unmapped
ground; all recovered without operator help (the §30 tree failed after 2). Still seen, not fixed: the loaded rover sometimes reaches a port's lead-in point
from the side (the grid planner has no heading) and has to back off and retry the pre-dock line-up; the empty rover's pivot on a remembered
route can stall (recovered by the retry). `npm run test:quick` 25/25 files; long set 6/6 (m3, drift, faults, fdir 65/65, power_dock, nav_bench), all on the final code.

Also merged from the §30 review (another session, same day): the float64 position guard up to `PHYS.posGuardMaxSpeed` 0.8 m/s (`sim.js`).

New tests (both in `npm run test:quick`): `tests/nav_retry.mjs` (13 checks: retry after a failed leg, learned obstacles kept, route source
switched, retry limit, no retry for "no path", charger leg, unwind trace never leads back into an earlier pocket) and `tests/frame_shift.mjs` (4 checks: an estimate seen before the previous fix does
not move under a 0.72 deg fix 2 km away; one seen now moves with the map). `tests/planner.mjs` +11 checks for route straightening (shorter,
clears a boulder by the hard margin, mapped ground only, ends kept, loaded radius >= 18 m, no chord across unmapped ground).
