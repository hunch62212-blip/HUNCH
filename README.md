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
npm run test:quick     # skips m3, drift, faults, fdir, power_dock, nav_bench, cycle
npm run test:long      # only those seven
node tests/run-all.mjs dock_cyl40 rocks   # named entries
TEST_JOBS=2 npm test   # limit parallel sims (each needs ~0.5-1 GB RAM)
```

| Entry | File | What it checks |
|---|---|---|
| physics | headless.mjs | load balance, speed tracking, skid turn, pivot, stop, reverse, park drift, top speed, kneel/carry ride |
| rocks | rock_climb.mjs | 0.57 m boulder climbed, 1.40 m boulder blocks, stays upright |
| lander | lander.mjs | driving into the cargo lander: no climb / roll-over, ends upright; lander tags seen; dynamics readouts sane |
| thermal | thermal.mjs | regolith temperature vs sun; hot slope + glint: instruments stay in range with the thermal brain (overheat without); cold start: rev up; drive heat: rev down; all 9 scenarios; brain idle on a normal day |
| planner | planner.mjs | unmapped ground beside a mapped slope / crater costs more; loaded paths keep a turning radius ≥ 18 m |
| ai_copilot | ai_copilot.mjs | copilot allowlist, one motion command per answer, reply parsing, advisor waits for `ai ok` (scripted model, no network) |
| creep | creep.mjs | 0.01–0.3 m/s speed tracking within 10 % |
| odo | odo.mjs | odometry within 5 % of truth at 0.5–2.9 m/s |
| pickup_gt_cyl40 / _box20 | pickup_gt.mjs | scripted pickup: grapple check, clamps lock, loaded tip margin, wheel contact |
| pickup_time_cyl40 / _box20 | pickup_time.mjs | autonomous pickup completes |
| dock_cyl40 / _box20 | dock.mjs | Mission 5 delivery, soft-capture accuracy |
| scout | scout.mjs | Mission 9 SCOUT-1 separation, mapping, lead |
| m1 / m3 / cycle | autonomy.mjs | Missions 1, 2+3, 6 complete |
| drift | drift.mjs | SLAM error vs distance on a 2 km drive |
| faults | faults.mjs | 2 km drive with a blocked road and a failed wheel |
| power_dock | power_dock.mjs | autonomous charge 30 → 90 % at the 60 kW charger, pack temperature, coupler release, bank lasts a lunar night |
| fdir | fdir.mjs | all 24 hardware failure modes detected and isolated from sensor data, automatic responses, field + dock repairs, random failures, 0 false alarms, lunar-night survival energy, keep-alive array |
| nav_bench | nav_bench.mjs | 2 km there and back, zero warnings, route reuse on return |

Requirement traceability: [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md). Ad-hoc debug scripts: `tests/scratch/` (not in `npm test`).

## AI copilot (local LLM, optional)

A local language model (**Qwen3.5-9B** on Ollama, on the team Mac mini) sits beside the operator console. It is
**not in the control loop**: physics, SLAM, planning, FDIR detection and the automatic fault responses stay
deterministic. The copilot reads a ~1 kB state snapshot plus the last log lines and can only act by typing the
same console commands an operator would, through an allowlist (`src/ai/copilot.js`). If the model is
unreachable the sim runs exactly as before. Design report: [docs/AI_COPILOT.md](docs/AI_COPILOT.md).

* **Ask in plain English:** `ai <request>` or just type a sentence (3+ words that are not a command), e.g.
  "slow down to 2 m/s and turn the lights on" → `speed 2`, `lights on`.
* **FDIR advisor:** each new alarm gets an assessment with its procedure card; suggested commands wait for
  `ai ok`. `ai on|off` toggles it.
* `ai brief` situation summary · `ai status` model, latency, tok/s · `ai model <name>` · `ai forget`.
* **Never allowed to the AI:** `reset` / `start` (re-arming), `drive` (raw velocity), `turbo on`, `fault`,
  `failures`, `clear`, `newmap`, `map`, `restricted`, time / sun / world settings. One mission / goto / charge
  per answer (a second would abort the first).
* **Where the model runs:** the page calls `/ollama`, which Vite proxies to `OLLAMA_URL`
  (default `http://100.92.134.100:11434`, the Mac mini over Tailscale, published with
  `tailscale serve --bg --tcp 11434 tcp://127.0.0.1:11434`; undo with `tailscale serve --tcp=11434 off`).
  `OLLAMA_URL=http://127.0.0.1:11434 npm run dev` uses a local Ollama. `?noai` turns the copilot off,
  `?aimodel=qwen3.5:4b` picks another model.
* Live check against the real model: `node scripts/ai_probe.mjs`. Offline unit checks: [test: ai_copilot].

## Vehicle health, failures, survival (since v1.1.0)

Design write-up and NASA rover comparison: [docs/SURVIVABILITY.md](docs/SURVIVABILITY.md). Every failure mode,
its sensors, detection rule, automatic response and repair steps: [docs/FAULT_PROCEDURES.md](docs/FAULT_PROCEDURES.md)
(generated from the code by `npm run docs:faults`). [test: fdir]

* **Random failures switch:** `failures on [realistic|100x|1000x|demo]` / `failures off`, or the
  **RANDOM FAILURES** button in the M10 panel. Off by default, seeded, at most 3 at once.
* **FDIR:** 24 failure modes on 20 replaceable units (motor controllers, wheel ends, resolvers, struts, battery
  strings, PDU, flight computers, IMUs, sensors, windows, radios, heaters, louvers, clamp drives, load cells,
  E-STOP loop, dock coupler, radiation upsets). Each is found from its sensor readings, isolated to the unit and
  bay, and answered automatically (switch to the backup, declutch, isolate, derate, EDS clean, power-cycle, safe).
* **Repairs:** `repair <id>|all`: timed procedure with steps, spares from the on-board kit or the dock stores;
  heavy units only on the dock service pad.
* **Serviceable layout:** all electronics in bays reachable from the ground with the rover knelt (1.0-1.6 m).
* **Survival:** vault and battery thermal model with louvered side radiators and redundant heaters; survival
  mode for the lunar night or a flat battery; 12 m² keep-alive solar blanket.
* **HUD:** HEALTH pill → FDIR panel (alarms, procedure card, repair progress, redundancy, thermal, spares).
  3D: radiators, bay doors with status lamps, antennas, beacon, hazcams, keep-alive mast, failed-part locator.

## Features carried over from the SPAN-40 simulator

These were written for SPAN-40 (the earlier straddle-carrier concept). Each number is now tagged for AEGIS-V1.

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
  * Plan hysteresis: keep the path unless blocked or a new plan is > 15 % cheaper (code constant in
    `Autonomy.replan`, `src/sim/autonomy.js`; behaviour **UNVERIFIED** by test).
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
* **Bug fixes** (from the SPAN-40 version; regression-tested where tagged):
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

Useful commands: `limit rpm 20`, `limit speed 0.8`, `speed 1.5`, `rpm 240`, `memory`, `turbo on|off|auto`,
`drive 0.5 0.05`, `stop`, `lift kneel|carry`, `clamp close`, `level off`, `ride 0.1`, `lidar channels 32`,
`fault block`, `fault wheel 3` (wheels 1–6), `fault battery 15`, `clear faults`, `plan`,
`priority B life-support`, `routes`, `map save`, `map clear`, `sun 205 12`, `timescale 10`, `ff` (50× fast forward), `newmap`,
`failures on demo`, `fault list`, `fault seize 5`, `health`, `procedure bat`, `repair all`, `survival on`, `array deploy`.

---

## Mission coverage

| NASA mission | How it is demonstrated |
|---|---|
| **M1** Leave the landing zone | Rover starts beside the lander, plans around lander, cargo and stands, drives to staging. [test: m1] |
| **M2** Find & approach cargo | Offload report → camera fiducial fix → line up on the module axis behind it → kneel → drive under on the cradle alignment sensors. [test: m3, pickup_time] |
| **M3** Pick up & secure | Rise to carry ride until the module is seated, then ring clamps close (refused outside tolerance); load check logs the minimum tip margin. [test: pickup_gt, pickup_time] |
| **M4** Transport | 2 km over craters, boulder fields, slopes and loose regolith carrying the module. [test: cycle] |
| **M5** Deliver & place | Port fiducial and docking-ring camera; the rover drives the module to soft capture under contact-force limits (AEGIS-V1 has no separate docking stage), then (dust-shield pulse, hard capture, metal-bellows shroud) → leak check → equalise → hatch open; a failed leak check keeps the hatch closed (`fault seal`); rover releases and backs out. [test: dock] |
| **M6** Full logistics cycle | Logistics manager orders requests → acquire → deliver → return → next load. [test: cycle] |
| **M7** 2 km autonomous route | 2.000 km pad–habitat separation [script]; SLAM mapping, hazard avoidance, geofences, route memory. [test: nav_bench, drift, faults] |
| **M8** Different cargo systems | Three cargo systems on the same ring clamps: 40-ft cylinder (PORT-1), 20-ft ISO box (PORT-2), and a 20-ft rack of 36 Astrobotic-style 4 ft x 2 ft pods (PORT-3). [test: pickup_*_box20, pickup_*_pod4, dock_box20, dock_pod4] |
| **M10** Something goes wrong | Blocked road, LiDAR failure, camera failure, comms loss, low battery, wheel motor failure (5 of 6 wheels driven, speed −25 %), cargo shift, port unavailable [test: faults: blocked road + wheel]. Plus 24 hardware failure modes with sensor-based detection, automatic response and repair, and a random-failure switch [test: fdir]; see docs/REQUIREMENTS.md and docs/FAULT_PROCEDURES.md. |
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
| Suspension | Rocker-bogie (track 7.67 m, wheelbase 7.83 m) with active struts: kneel −0.65 m / carry +0.82 m / max +0.98 m ride |
| Steering | Skid steer (fixed wheels): yaw from left/right speed difference; pivot in place |
| Drive | 6 hub gearmotors, 5,600 N·m peak per wheel, PI speed loop, spring brakes, thermal model |
| Speed | Operating limits 5.0 m/s empty, 2.5 m/s loaded (operator choice, v1.5.0), further trimmed by smart rpm, the curve limit (0.3 m/s² lateral empty, 0.15 loaded), the stopping-distance governor and the tip governor [ASSUMED]. Motor rpm: continuous smart rpm between crawl 120 and the 280 cap (turbo 300 = no-load on clear straights); 5.0 m/s is ~182 prototype rpm, so the cap mainly gives the outer wheels headroom in turns. Acceleration and braking commands stay under the soil traction limit (~1.1 m/s² on the Moon) |
| Power | 420 kWh, 400 V in 4 strings with their own contactors; PDU converters A/B. Charged at the dock; 12 m² solar array (4.8 kW at noon): deploys by itself when the rover is parked idle in sunlight, and in survival mode (parked only) [ASSUMED] |
| Thermal | avionics vault + battery: louvered vertical radiators under both deck edges (the module covers the deck), thermostatic heaters on controllers A/B [ASSUMED] |
| Avionics | flight computers A/B (hot standby), IMU A/B + voter gyro, primary + UHF radios, 4 corner omni antennas, locator beacon, E-STOP hard-wired loop [ASSUMED] |
| Serviceability | every electronics unit in a bay at 1.0-1.6 m working height with the rover knelt; field kit + dock spares (docs/SURVIVABILITY.md) |
| Cargo handling | 2 ring clamps (dual-winding jaw motors); lift by strut ride height (no hoist travel, no docking stage) |
| Sensors | 32-beam 360° LiDAR (90 m, 10° nose-down, front deck-corner mount per the team CAD; port head prime, starboard head cold spare; two interleaved sets of 16 beams on alternate sweeps at 5-12 Hz; returns on the chassis and the carried module are masked), 90° stereo depth camera (0.18 m baseline, depth to 35 m; depth points with sigma ≤ 0.10 m fill map cells the LiDAR has not measured and are replaced by its first return; blinded within 12° of the sun, headlight range 35 m at night), IMU, sun sensor, wheel encoders, strut load cells, cradle alignment suite, docking-ring camera; health sensors on every unit (heartbeats, phase currents, resolvers, thermistors, string voltages, insulation monitor, louver encoders, vent gas) |
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
  140 m of the rover (`streamRocks`, `src/sim/world.js`).

## Autonomy stack (nothing uses ground truth)

1. **Perception.** Spinning LiDAR, motion de-skewed; camera fiducial pose (noise ∝ range²), line of sight, rock detector.
2. **Localisation (SLAM).** Slip-compensated wheel odometry, gyro with bias walk, sun-sensor heading, surveyed
   fiducial beacons, correlative scan-to-map matching on a 2.5D elevation map (degeneracy tested along/across track),
   loop closure, and map-relative fixes against an orbital elevation model every ~40 m (`src/sim/orbitalMap.js`).
3. **Mapping & traversability.** 0.5 m elevation tiles; slope, step, crater classes.
4. **Global planning.** Weighted A* on a 2 m costmap; geofences lethal; remembered-route cells cheaper; unmapped ground priced
   at the median cost of mapped ground; a new plan must be clearly cheaper (and is held 20 s) before the rover switches to it.
5. **Teach & repeat** (after Furgale & Barfoot). Proven legs stored and re-validated; each stored route is straightened over
   ground it has mapped (chords that pass the full loaded footprint check; 6-10 % shorter); a route proven with this load is
   kept through close obstacles. Persisted on disk in the rover memory store (`~/.hunch22-data/memory`, one file per site +
   10 snapshots; `memory` command, `?site=last`), IndexedDB as fallback.
6. **Local planning.** 25 constant-curvature arcs (gentle set: inner side >= 50-65 % of the outer; full set only in tight spots),
   wheel-footprint checks; speed from free distance, tip margin, curve and heading change, faults. Bends up to 115° are
   driven as moving arcs; pivots only for a target behind the rover.
7. **Terminal guidance.** Fiducials → cradle alignment sensors → docking-ring camera.
8. **Recovery.** A leg the navigation gives up on (boxed in, no progress, steering not answering) is re-tried twice after
   an autonomous recovery (back out, keep what it learned, re-localise, other route source) before the operator is asked;
   a vehicle whose wheels turn while the LiDAR scene stands still is detected as pinned and its estimate held.

## Safety

* **E-STOP** removes drive torque, sets spring brakes, locks actuators. Latching: RESET then START (4 s self-test).
* **Operator override.** Any drive input pauses autonomy; `resume` continues.
* **Interlocks.** Clamp refused outside tolerance; release inhibited unless cargo is supported; hatch opens only
  after seal and leak check. Bay doors open their solid-state breaker; RESET is refused while the E-STOP loop is
  open; START is refused with both flight computers or both PDU converters failed.
* **Safe / survival modes.** Loss of both units of a redundant pair safes the rover (drive off, beacon).
  Survival mode (`survival on`, automatic below 6 % away from the dock) holds the vault and battery at their
  survival set points on ~0.15 kW (a 354 h lunar night ≈ 12 % of the pack).

## Three.js versions (r128 vs 0.170.0)

* The app imports **three 0.170.0** (`package.json`, pinned) as an ES module.
* The team CAD scripts in `public/team/js/*.js` are classic scripts written against **r128** that read a global
  `THREE`. `src/render/teamModels.js` sets `window.THREE` to a copy of the app's 0.170.0 namespace (plus
  `RoundedBoxGeometry`) before loading them in order. **No r128 copy is loaded**, so there is one Three.js at runtime.
* Risk: any r128-only API in the team scripts would fail at load. Runtime check: see CHANGES.md (browser check).

## Simplifications

* Procedural (seeded) site, not LOLA data. 2.5 × 2.5 km terrain plus a decorative horizon; landing pad and
  habitat exactly 2,000 m apart. The browser draws a new random seed on every load (crater / boulder layout
  changes; `?seed=N` repeats a map, `newmap` rerolls); the headless tests use the fixed default seed.
* Fast forward (⏩ 50×, key `X`): full physics cannot run faster than ~5× real time, so at 50× the long
  autonomous cruise legs use **transit warp**: the rover is moved kinematically along its planned path
  (body on the terrain plane under the wheels, drive power from the planning energy model, a coarse LiDAR
  sweep every 6 m keeps the map ahead filled). The autonomy still plans on its own SLAM map and the warp
  only runs while the next 30 m of path is mapped and clear; pickup, docking, charging, pivots and
  recoveries always run on full physics. The clock shows the speed actually achieved. `warp off` disables it.
* Rocker-bogie modelled as six struts with a pitch-sharing term, not separate link bodies. Wheels are
  shape-casts, not rigid bodies.
* Camera detections are geometric, not image-based.
* Regolith stiffness, cohesion (±30 %) and friction angle (±3°) vary over ~45 m patches from the seed; the on-board slip
  estimator assumes the nominal soil. Dust is modelled only as window / radiator fouling failure modes, not electrostatically.
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
src/sim/health.js        FDIR catalogue (units, bays, failure modes, sensors, procedures), random failures,
                         thermal model, survival mode, keep-alive array, repairs
src/sim/commands.js      operator console commands
src/render/*             terrain LOD, AEGIS-V1 model (team CAD), props, effects, sky
src/ui/hud.js            operator HUD
public/team/js/*         team CAD scripts (r128-era, global THREE)
scripts/readme_facts.mjs prints config-derived README numbers
scripts/fault_procedures.mjs writes docs/FAULT_PROCEDURES.md from the FDIR catalogue
scripts/launch.sh        versioned launcher (see VERSIONS.md)
scripts/memory-plugin.mjs Vite plugin for the rover memory store
scripts/ai_probe.mjs     AI provider probe
tests/*.mjs              headless pass/fail tests; tests/run-all.mjs runner
tests/lib/check.mjs      shared test helpers
tests/scratch/           debug scripts (not run by npm test)
docs/                    design docs, requirements, agent prompts
_archive/                retired files (see _archive/README.md)
```
