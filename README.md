# AEGIS-V1 — Lunar External Cargo Transport Simulator

**NASA HUNCH SFT 2026-27 · LLASO Project 2 · LLASO-P2-XPORT-2026**
Simulation of a robot that moves a landed 40-ft cargo module from the landing pad to the supply building
**2 km away** and docks it to a pressurised port, with no EVA.

The vehicle is **AEGIS-V1**, the team's rocker-bogie skid-steer cargo rover (CAD in `public/team/js/`), simulated
at 1:1 from the 1:16.3 prototype. Earlier versions of this README described **SPAN-40**, a straddle-carrier concept
that is no longer in the code; see [CHANGES.md](CHANGES.md).

Runs in a web browser. Physics: Rapier rigid-body engine plus our own vehicle and terramechanics model.
Graphics: Three.js.

Tags used below: **[test: name]** = checked by `npm test` entry *name*; **[script]** = printed by
`node scripts/readme_facts.mjs`; **UNVERIFIED** = no test or script reproduces it.

---

## Run it

```bash
npm install
npm run dev
```

Open http://127.0.0.1:5173 in Chrome, Edge or Safari. The rover's learned map and routes are saved in the
browser; open http://127.0.0.1:5173/?fresh (or type `map forget`) to start with no map.

## Tests

Headless, no browser. Each check prints one `PASS`/`FAIL` line; any failure gives exit code 1.
Full logs: `tests/out/<name>.log`.

```bash
npm test               # everything (the long autonomy runs take 25+ min wall each)
npm run test:quick     # skips m3, drift, faults, nav_bench, cycle
npm run test:long      # only those five
node tests/run-all.mjs dock_cyl40 rocks   # named entries
TEST_JOBS=2 npm test   # limit parallel sims (each needs ~0.5-1 GB RAM)
```

| Entry | File | What it checks |
|---|---|---|
| physics | headless.mjs | load balance, speed tracking, skid turn, pivot, stop, reverse, park drift, top speed, kneel/carry ride |
| rocks | rock_climb.mjs | 0.57 m boulder climbed, 1.40 m boulder blocks, stays upright |
| creep | creep.mjs | 0.01–0.3 m/s speed tracking within 10 % |
| odo | odo.mjs | odometry within 5 % of truth at 1–4.9 m/s |
| pickup_gt_cyl40 / _box20 | pickup_gt.mjs | scripted pickup: grapple check, clamps lock, loaded tip margin, wheel contact |
| pickup_time_cyl40 / _box20 | pickup_time.mjs | autonomous pickup completes |
| dock_cyl40 / _box20 | dock.mjs | Mission 5 delivery, soft-capture accuracy |
| scout | scout.mjs | Mission 9 SCOUT-1 separation, mapping, lead |
| m1 / m3 / cycle | autonomy.mjs | Missions 1, 2+3, 6 complete |
| drift | drift.mjs | SLAM error vs distance on a 2 km drive |
| faults | faults.mjs | 2 km drive with a blocked road and a failed wheel |
| nav_bench | nav_bench.mjs | 2 km there and back, zero warnings, route reuse on return |

Requirement traceability: [REQUIREMENTS.md](REQUIREMENTS.md). Ad-hoc debug scripts: `tests/scratch/` (not in `npm test`).

## What's new (v2)

Numbers here were written for SPAN-40. Each is now tagged.

* **Team CAD in the sim:**
  * `llaso_lunar_cargo_container.obj` is the 40-ft module (visual). The physics module is the team's
    `module_models.js` design: 12.19 m × 7.96 m Ø, 14,940 kg [script]. The old figure "R 1.54 m, ±6.39 m" is
    **superseded** (not what the physics uses).
  * Interior (14 rack bays, lockers, bags, tanks, pods, deck, gantry robot), shown with **X-ray cargo**:
    bay count **UNVERIFIED**.
  * The LLASO-P1 manifest (22 items, 748.5 kg [script]) sets the module's centre of mass and fills the manifest panel.
  * `tracked_lunar_rover.obj` is **SCOUT-1** for Mission 9.
* **Navigation:**
  * Speed limit uses stopping distance on *mapped* ground.
  * LiDAR tilted nose-down: config is **10°** [script]; the earlier "15°" is superseded.
  * Obstacles the local planner hits are learned into the global map and expire.
  * Planner fallbacks: drop temporary blocks, then reduce the obstacle margin.
  * Collision-checked reverse recovery; a progress watchdog asks the operator for help.
  * Route memory reuses any part of a proven route, in either direction [test: nav_bench, return leg].
  * Plan hysteresis: keep the path unless blocked or a new plan is > 15 % cheaper (code constant,
    `src/sim/autonomy.js:316`; behaviour **UNVERIFIED** by test).
  * SLAM loop closure on revisits, wide search "±12 m, ±3°": **UNVERIFIED** (no test isolates it; nav_bench counts closures).
  * Surveyed beacons only initialise the map frame.
  * Tip-over margin uses the full wheel contact polygon.
  * **Measured** (`node tests/nav_bench.mjs hab,pad`): see [Measured results](#measured-results). The SPAN-40
    figures (2.19 km / 25.4 min / 1.44 m/s; 2.22 km / 5 loop closures / 24.2 min / 1.52 m/s) are **UNVERIFIED**
    for AEGIS-V1 and are replaced by the table below.
* **Graphics:** cascaded shadow maps, lunar IBL, fill light, SMAA, lens flare, two-scale terrain detail shader,
  pebbles, base expansion (habitat, tunnel, modules, light masts, cable run, pad beacons, all solid colliders),
  quality selector. Not tested headless.
* **Features:** SCOUT-1 cooperative mapping (`scout on`) [test: scout], live trend charts, telemetry CSV and
  mission report JSON export, double-click to drive, `?fresh` / `map forget`.
* **Bug fixes** (from v2; regression-tested where tagged):
  * Wheel spin limit cycle in loaded pivots (fully implicit solve) [test: physics, pivot].
  * Parked rover crept under docking loads (static-friction brake model) [test: physics, park drift].
  * Wheels could step onto large boulders: climb limit is now **0.90 m** edge height (0.85 × wheel radius
    [script]); the earlier "0.6 m" was SPAN-40. [test: rocks]
  * Others (HUD key repeat, standstill wheel steer visual, start heading, cargo-supported ray origin, NaN shadow
    cascades, base colliders, short cargo reach): **UNVERIFIED** (browser-only or no test).

## Measured results

Filled from `npm test` output; see [CHANGES.md](CHANGES.md) for the run date and code state.

<!-- MEASURED-TABLE -->

## Quick start in the simulator

| Do this | How |
|---|---|
| Run the full mission | Click **M6 Full cycle** (or `mission cycle`). Time buttons fast-forward; sensors are decimated above 3×. |
| Drive it yourself | `W`/`S` throttle, `A`/`D` turn (skid steer), `Space` brake, `Shift` precise. `1`/`2`/`3` and `Q`/`E` (Ackermann / crab / pivot) are left over from SPAN-40 and have no effect on a skid-steer rover. |
| Handle cargo by hand | `G` clamp / release; `R`/`F` ride height up / down (kneel ↔ carry) |
| Emergency stop | Red **E-STOP** (top right) or `Esc`. Latching: **RESET**, then **START** (power-on self-test). |
| Commands | Press `Enter` to focus the console. `help` lists everything. |
| Views | `C` cycles chase / orbit / top / onboard / free. |
| Send the rover somewhere | Double-click the terrain or SLAM map, or `goto <x> <z>` / `goto hab` |
| See inside the module | **X-ray cargo** overlay |
| Mission 9 scout | **deploy** under M9 (or `scout on`), then start any drive |
| Export data | **export telemetry CSV** / **mission report** in the Vehicle panel |

Useful commands: `limit rpm 20`, `limit speed 0.8`, `speed 1.5`, `rpm 180`, `turbo on|off|auto`,
`drive 0.5 0 0.2`, `stop`, `lift kneel|carry`, `clamp close`, `level off`, `ride 0.1`, `lidar channels 32`,
`fault block`, `fault wheel 3` (wheels 1–6), `fault battery 15`, `clear faults`, `plan`,
`priority B life-support`, `routes`, `map save`, `map clear`, `sun 205 12`, `timescale 10`.

---

## Mission coverage

| NASA mission | How it is demonstrated |
|---|---|
| **M1** Leave the landing zone | Rover starts beside the lander, plans around lander, cargo and stands, drives to staging. [test: m1] |
| **M2** Find & approach cargo | Offload report → camera fiducial fix → line up on the module axis behind it → kneel → drive under on the cradle alignment sensors. [test: m3, pickup_time] |
| **M3** Pick up & secure | Rise to carry ride until the module is seated, then ring clamps close (refused outside tolerance); load check logs the minimum tip margin. [test: pickup_gt, pickup_time] |
| **M4** Transport | 2 km over craters, boulder fields, slopes and loose regolith carrying the module. [test: cycle] |
| **M5** Deliver & place | Port fiducial and docking-ring camera; the rover drives the module to soft capture under contact-force limits (AEGIS-V1 has no separate docking stage), then hard capture → seal → purge → leak check → equalise → hatch open; rover releases and backs out. [test: dock] |
| **M6** Full logistics cycle | Logistics manager orders requests → acquire → deliver → return → next load. [test: cycle] |
| **M7** 2 km autonomous route | 1.997 km pad–habitat separation [script]; SLAM mapping, hazard avoidance, geofences, route memory. [test: nav_bench, drift, faults] |
| **M8** Different cargo systems | 40-ft cylinder and 20-ft ISO box in a ring-adapter sled, same ring clamps. [test: pickup_*_box20, dock_box20] |
| **M10** Something goes wrong | Blocked road, LiDAR failure, camera failure, comms loss, low battery, wheel motor failure (5 of 6 wheels driven, speed −25 %), cargo shift, port unavailable. Only blocked road + wheel failure are tested [test: faults]; see REQUIREMENTS.md. |
| **M11** AI logistics manager | Scores requests by category priority, ETA, energy. `plan`, `priority B life-support`. UNVERIFIED by test beyond the cycle order. |
| **M12** Our own capability | Teach-and-repeat route memory [test: nav_bench], model-based slip estimation [test: odo], active load leveling. (The SPAN-40 sun-tracking solar mast is not on AEGIS-V1: 0 panels [script].) |

---

## Rover design: AEGIS-V1

**Concept.** The team's rocker-bogie rover at full scale. It kneels, drives under the module along its axis,
rises on its active struts until the module is seated in the cradle, and locks two ring clamps. Skid steer:
the wheels do not steer. All values below are [script] unless marked.

| Item | Value |
|---|---|
| Frame L × W | 9.79 × 9.86 m |
| Mass | 8,000 kg empty (6,200 sprung + 6 × 300 kg corners) — [ASSUMED] in config |
| Payload | 40-ft module 14,940 kg (12.19 × 7.96 m Ø); 20-ft ISO box 8,500 kg |
| Wheels | 6 × 2.12 m Ø × 1.22 m woven steel hoop tyres with chevron cleats (team `rover_bom.js`, scaled) |
| Suspension | Rocker-bogie (track 7.67 m, wheelbase 7.02 m) with active struts: kneel −0.65 m / carry +0.82 m / max +0.98 m ride |
| Steering | Skid steer (fixed wheels): yaw from left/right speed difference; pivot in place |
| Drive | 6 hub gearmotors, 5,600 N·m peak per wheel, PI speed loop, spring brakes, thermal model |
| Speed | 4.95 m/s at the 180-rpm-equivalent cap (unloaded); 3.8 m/s loaded limit, further trimmed by the tip governor |
| Power | 420 kWh, 400 V. No on-board solar; charged at the dock station |
| Cargo handling | 2 ring clamps; lift by strut ride height (no hoist travel, no docking stage) |
| Sensors | 16-channel LiDAR (90 m, 5 Hz, 10° nose-down), 70° camera, IMU, sun sensor, wheel encoders, strut load cells, cradle alignment suite, docking-ring camera |
| Climb limit | 0.90 m edge height [script; test: rocks] |

**Static stability.** The force-angle tip margin is computed live from the wheel contact polygon and the
combined CG. Autonomy slows when it drops below 25°. Loaded margin: see Measured results.

## Physics models

* **Gravity** 1.62 m/s² [script], vacuum, hard shadows, low polar sun (default 9°, adjustable).
* **Vehicle.** Sprung chassis is a Rapier rigid body with collision shapes. Six wheel stations, each a 1-DOF
  unsprung mass on a long-stroke active strut that stands in for the rocker-bogie links (a differential term
  shares pitch between rockers). Each wheel finds the ground with a cylinder shape-cast.
* **Terramechanics** (`src/sim/terramechanics.js`), Wong / Apollo lunar regolith: n = 1, k_c = 1.4 kN/m²,
  k_φ = 820 kN/m³, c = 0.17 kPa, φ = 35°, K = 1.8 cm [script].
  * Bekker sinkage, compaction resistance, Janosi–Hanamoto thrust, lateral force from the same shear law
  * Wheel spin solved implicitly each step (bisection on the torque balance with the PI controller and brake inside)
  * Braked wheels use a static-friction (bristle) model
* **Power.** Motor electrical power (efficiency, copper loss, regen), hotel loads, actuator power. Solar input
  term exists but AEGIS-V1 has 0 panels.
* Rocks larger than 25 cm are convex-hull colliders, rendered with the same hull. Colliders are streamed within
  140 m of the rover (`src/sim/world.js:60`).

## Autonomy stack (nothing uses ground truth)

1. **Perception.** Spinning LiDAR, motion de-skewed; camera fiducial pose (noise ∝ range²), line of sight, rock detector.
2. **Localisation (SLAM).** Slip-compensated wheel odometry, gyro with bias walk, sun-sensor heading, surveyed
   fiducial beacons, correlative scan-to-map matching on a 2.5D elevation map, loop closure.
3. **Mapping & traversability.** 0.5 m elevation tiles; slope, step, crater classes.
4. **Global planning.** Weighted A* on a 2 m costmap; geofences lethal; remembered-route cells cheaper.
5. **Teach & repeat** (after Furgale & Barfoot). Proven legs stored and re-validated; persisted in IndexedDB.
6. **Local planning.** 25 constant-curvature arcs, wheel-footprint checks; speed from free distance, tip margin, faults.
7. **Terminal guidance.** Fiducials → cradle alignment sensors → docking-ring camera.

## Safety

* **E-STOP** removes drive torque, sets spring brakes, locks actuators. Latching: RESET then START (4 s self-test).
* **Operator override.** Any drive input pauses autonomy; `resume` continues.
* **Interlocks.** Clamp refused outside tolerance; release inhibited unless cargo is supported; hatch opens only
  after seal and leak check.

## Three.js versions (r128 vs 0.170.0)

* The app imports **three 0.170.0** (`package.json`, pinned) as an ES module.
* The team CAD scripts in `public/team/js/*.js` are classic scripts written against **r128** that read a global
  `THREE`. `src/render/teamModels.js` sets `window.THREE` to a copy of the app's 0.170.0 namespace (plus
  `RoundedBoxGeometry`) before loading them in order. **No r128 copy is loaded**, so there is one Three.js at runtime.
* Risk: any r128-only API in the team scripts would fail at load. Runtime check: see CHANGES.md (browser check).

## Simplifications

* Procedural (seeded) site, not LOLA data. 2.2 × 2.2 km terrain plus a decorative horizon.
* Rocker-bogie modelled as six struts with a pitch-sharing term, not separate link bodies. Wheels are
  shape-casts, not rigid bodies.
* Camera detections are geometric, not image-based.
* Uniform regolith; no electrostatic dust model.
* Mission 9 is cooperative mapping only. SCOUT-1 uses a kinematic track model.
* Pebbles under 12 cm are visual only.

## Code map

```
src/sim/config.js        engineering parameters, with provenance tags per group
src/sim/terrain.js       terrain, craters, boulders, site layout
src/sim/world.js         Rapier world, colliders, rock streaming
src/sim/rover.js         vehicle dynamics, motors, suspension, lift, power
src/sim/terramechanics.js Bekker / Janosi wheel-soil model
src/sim/sensors.js       LiDAR, camera, IMU, odometry, sun sensor, cradle sensors
src/sim/slam.js          elevation-map SLAM, hazards, route memory, persistence
src/sim/planner.js       costmap, A*, local arc planner
src/sim/autonomy.js      mission executive, docking, faults, logistics manager
src/sim/commands.js      operator console commands
src/render/*             terrain LOD, AEGIS-V1 model (team CAD), props, effects, sky
src/ui/hud.js            operator HUD
public/team/js/*         team CAD scripts (r128-era, global THREE)
scripts/readme_facts.mjs prints config-derived README numbers
tests/*.mjs              headless pass/fail tests; tests/run-all.mjs runner
```
