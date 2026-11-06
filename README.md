# SPAN-40 — Lunar External Cargo Transport Simulator

**NASA HUNCH SFT 2026-27 · LLASO Project 2 · LLASO-P2-XPORT-2026**
High-fidelity simulation of a robot that moves a landed 40-ft cargo module from the landing pad to the
supply building **2 km away**, and docks it to a pressurised port through a dust-exclusion coupling,
with no EVA.

The simulator runs in a web browser. Physics comes from the Rapier rigid-body engine, with our own
vehicle and terramechanics model on top. Graphics use Three.js.

---

## Run it

```bash
npm install
npm run dev
```

Open http://127.0.0.1:5173 in Chrome, Edge or Safari. Loading takes about 5–10 s while the terrain is generated.
The rover's learned map and routes are saved in the browser. Open http://127.0.0.1:5173/?fresh (or type `map forget`) to start over with no map.

Headless checks (no browser needed):

```bash
npm run test:physics          # drive / turn / crab / pivot / park / grapple / lift / carry
node tests/autonomy.mjs m1    # Mission 1 autonomously
node tests/autonomy.mjs m3    # Mission 2 + 3 autonomously (approach, straddle, grapple, lift, load check)
npm run test:rocks           # wheel track meets a 0.57 m and a 1.4 m boulder (climb vs. blocked)
npm run test:dock            # Mission 5 docking in isolation, both cargo types
npm run test:faults          # 2 km traverse with a blocked road + failed wheel injected
npm run test:cycle           # Mission 6: full logistics cycle (~25 min wall time)
node tests/nav_bench.mjs hab,pad   # 2 km there and back: exploration, then route-memory reuse
node tests/scout.mjs         # Mission 9: SCOUT-1 leads, maps ahead, keeps separation
```

## What's new (v2)

* **Team CAD in the sim** (from the Hunch folder):
  * `llaso_lunar_cargo_container.obj` is the 40-ft module. The physics now uses its exact dimensions (R 1.54 m, ±6.39 m with collars). Its 14 rack bays, lockers, cargo bags, cryo tanks, science pods, floor deck and gantry robot are inside the hull, shown with **X-ray cargo**.
  * The LLASO-P1 manifest (22 items, 748.5 kg) sets the module's centre of mass and fills the interior manifest panel.
  * `tracked_lunar_rover.obj` is **SCOUT-1** for Mission 9.
* **Navigation overhaul:**
  * The speed limit uses stopping distance on *mapped* ground, so the rover creeps to a crater rim instead of driving into a crater it can't see yet.
  * The LiDAR is tilted 15° nose-down to shrink the near-field blind zone.
  * Obstacles the local planner hits are learned into the global map, and those marks expire.
  * The planner has fallbacks: drop temporary blocks, then reduce the obstacle margin.
  * Recovery uses collision-checked reverse arcs, and a progress watchdog asks the operator for help instead of looping forever.
  * Route memory reuses any part of a proven route, in either direction.
  * Plan hysteresis: the rover keeps its path unless it's blocked or a new plan is at least 15% cheaper. This stopped the rover thrashing between near-equal detours around craters.
  * **SLAM loop closure:** on revisiting an area mapped long ago, a wide search (±12 m, ±3°) against the old map, with structures included, snaps the pose back into agreement. This removes the ghost obstacles that drift created.
  * Surveyed beacons now only initialise the map frame. Snapping to the survey later would break consistency with the existing map.
  * Pivot turns have hysteresis.
  * The tip-over margin now uses all four corners (it used to falsely slow the rover on rough ground).
  * Measured (`node tests/nav_bench.mjs hab,pad`), zero warnings on both legs:
    * pad→habitat: 2.19 km exploring, 25.4 min, 1.44 m/s average
    * habitat→pad: 2.22 km on the remembered route with 5 loop closures, 24.2 min, 1.52 m/s average
* **Graphics:**
  * Cascaded shadow maps (shadows reach the horizon), lunar image-based lighting for metals, a regolith bounce/fill light and SMAA.
  * Sun lens flare and a two-scale terrain detail shader.
  * Crinkled insulation blankets, decorative regolith pebbles, and a detailed rover: harnesses, handrails, ladder, trolley rails, nav lights, sun sensor, star tracker, motor fins, hazard striping.
  * Base expansion: habitat dome, tunnel, two modules, light masts, cable run and pad beacons. All of these are solid colliders.
  * Graphics quality selector (low / medium / high / ultra).
* **Features:**
  * SCOUT-1 cooperative mapping (`scout on`).
  * Live trend charts.
  * **Telemetry CSV export** and **mission report JSON**.
  * Double-click the terrain to drive there.
  * `?fresh` / `map forget` to start with no map.
* **Bug fixes:**
  * Space/Enter could re-trigger HUD buttons while driving.
  * Wheels didn't visibly steer at standstill.
  * The rover started facing the lander.
  * Wheel spin had a numerical limit cycle in loaded pivots (now a fully implicit solve).
  * Parked rover crept under docking loads (now a static-friction brake model).
  * Wheels could "step up" onto 1.4 m boulders (now a 0.6 m climb limit).
  * The cargo-supported check cast its rays from the world origin.
  * Shadows went black after the window was hidden (NaN cascade frustums).
  * Base structures had no colliders.
  * Short cargo couldn't reach the port (it now rides forward in the bay).

## Quick start in the simulator

| Do this | How |
|---|---|
| Run the full mission | Click **M6 Full cycle** (or type `mission cycle`). Use the time buttons to fast-forward. A typical laptop sustains about 4–8× (sensors are decimated above 3×; the clock shows "CPU-limited" when it can't keep up). The full cycle is about 2 h of mission time. |
| Drive it yourself | `W`/`S` throttle, `A`/`D` steer, `Q`/`E` crab, `1`/`2`/`3` = Ackermann / crab / pivot, `Space` brake, `Shift` precise |
| Handle cargo by hand | `G` grapple / release, `R`/`F` hoist up / down, `T`/`Y` docking stage |
| Emergency stop | Small red **E-STOP** button (top right) or `Esc`. It **latches**: press **RESET**, then **START** (power-on self-test) to drive again. |
| Commands | Type in the console (press `Enter` to focus it). `help` lists everything. |
| Views | `C` cycles chase / orbit / top / onboard / free. Mouse: drag to orbit, scroll to zoom. |
| Send the rover somewhere | Double-click on the terrain or the SLAM map, or `goto <x> <z>` / `goto hab` |
| See inside the module | **X-ray cargo** overlay button (shows the LLASO-P1 interior + manifest) |
| Mission 9 scout | **deploy** under M9 (or `scout on`), then start any drive |
| Export data | **export telemetry CSV** / **mission report** in the Vehicle panel |

Useful commands: `limit rpm 20`, `limit speed 0.8`, `limit torque 60`, `speed 1.5`, `steer crab`,
`drive 0.5 0 0.2`, `stop`, `lift up`, `clamp close`, `stage 0.2`, `level off`, `ride 0.1`,
`lidar channels 32`, `fault block`, `fault wheel 2`, `fault battery 15`, `clear faults`,
`plan`, `priority B life-support`, `routes`, `map save`, `map clear`, `sun 205 12`, `timescale 10`.

---

## Mission coverage

| NASA mission | How it is demonstrated |
|---|---|
| **M1** Leave the landing zone | Rover starts beside the lander; it plans around the lander, cargo and saddles, then drives to the staging area. Start / stop / turn / reverse are available manually. |
| **M2** Find & approach cargo | Lander offload report (±1.5 m) → camera fiducial fix → pre-grasp pose behind the module → pivot to axis → straddle approach under visual servo, then cradle sensors take over. No manual repositioning. |
| **M3** Pick up & secure | Cradle arms close (misalignment > 0.35 m / 0.22 m / 3.5° is refused), hoist lifts 0.6 m, then a start / stop / turn / reverse load check logs the minimum tip-over margin. |
| **M4** Transport | 2 km route over craters, boulder fields, slopes and loose regolith, with the 12 t module held by the cradle. |
| **M5** Deliver & place | Final approach on the port fiducial and docking-ring camera. The docking stage pushes to soft capture under force control, then: hard capture → bellows seal → dust purge → leak check → equalise → cradle jacks → hatch open. The rover then releases and backs out. Lateral / vertical / yaw accuracy is logged. |
| **M6** Full logistics cycle | Logistics manager orders the requests → acquire → deliver → return → next load → return to staging. Logs time, distance, energy, accuracy and deliveries. |
| **M7** 2 km autonomous route | Real 2.0 km separation. No pre-mapped path: the rover maps as it drives (SLAM), avoids craters, rocks, steep slopes and **geofenced zones** (an active landing site and a science preserve), and **remembers routes that worked**. |
| **M8** Different cargo systems | The same cradle handles a 40-ft cylinder (Blue Origin-class) and a 20-ft ISO-class box: trolley positions, arm length and saddle heights adapt per cargo type. |
| **M10** Something goes wrong | Fault buttons: blocked road (boulder field dropped on the path → replan), LiDAR failure (slow down on a proven route, otherwise hold), camera failure, comms loss (keeps working; commands rejected except E-STOP), low battery (park and solar-charge), wheel motor failure (derate, 7-wheel drive), cargo shift (detected, re-centred), receiving port unavailable (hold and ask the operator). |
| **M11** AI logistics manager | Scores requests by category priority (life-support > power > supplies > science), ETA and energy feasibility. `plan` shows the order, and `priority B life-support` re-orders it. |
| **M12** Our own capability | Teach-and-repeat route memory (persistent across sessions), sun-tracking solar mast with shadow detection, model-based slip estimation, active load leveling. |

---

## Rover design: SPAN-40 (Straddle Portal Autonomous Navigator)

**Concept.** Instead of lifting a 12 t module *onto* a deck (high CG, tipping risk in 1/6 g), the
rover is a **straddle carrier**: a portal frame on four legs drives *over* the module and slings it
between the wheel tracks. The CG stays low and inside a wide support polygon. Rocks pass harmlessly
between the tracks, and the module can be pushed straight into a port. This is how Earth port
straddle carriers and gantry cranes work. Venturi Astrolab's FLEX rover and JPL's ATHLETE use the
same "come to the payload" idea.

| Item | Value |
|---|---|
| Overall L × W × H | 15.0 × 6.2 × 5.6 m (7.5 m to the top of the solar mast) |
| Clear bay (W × H) | 4.4 × 4.7 m: fits a 3.2 m Ø cylinder or a 2.44 × 2.59 m ISO box |
| Mass | 6,980 kg (5,900 sprung + 4 × 270 kg bogies) |
| Payload | 12,000 kg 40-ft module (19.4 kN lunar weight), 30 kN hoist rating |
| Wheels | 8 × 1.40 m Ø × 0.45 m woven spring-steel mesh tyres with titanium chevrons (Apollo LRV / NASA Glenn spring-tyre heritage) |
| Suspension | 4 steerable walking-beam bogies (load equalising). Each has a coil-over strut (62 kN/m, 11 kN·s/m, ±0.3 m travel) with **active leveling** (±0.45 m seat), plus bump and top-out stops |
| Steering | ±66° per bogie, 18°/s: double-Ackermann, crab and pivot-in-place |
| Drive | 8 hub BLDC + 120:1 harmonic drives, 1,450 N·m peak per wheel, 42 rpm, PI speed loop, spring-applied brakes, first-order thermal model with derating |
| Speed | 2.2 m/s empty, 1.4 m/s loaded (tip-margin governed) |
| Power | 2 × 11.5 m² vertical tracking solar panels (30 %): up to ~9 kW at the polar low sun. 140 kWh Li-ion, 400 V. Regenerative braking. |
| Cargo handling | 4 cradle arms on trolleys (lug spacing and carry position set per cargo type: a short 20-ft box rides 1.9 m forward so its hatch reaches the port), 0.9 m hoist, **docking stage** with 0.6 m longitudinal and ±0.18 m lateral travel and force-controlled push |
| Sensors | 1 × 16-channel 360° LiDAR (90 m, 5 Hz, 2 cm noise) · 1 × 70° camera (fiducials + rock detector) · tactical-grade IMU · sun sensor · wheel encoders · strut load cells · cradle alignment suite (8 lateral laser rangefinders at 4 stations, 2 × 57-beam light curtain) · docking-ring close-range camera · contact-force sensing at the dock |

**Static stability.** Track 5.5 m, loaded CG ≈ 3.1 m: about 42° static lateral tip angle. The
force-angle tip-over margin is computed live from the actual wheel contact polygon and the combined
rover + cargo CG. Autonomy slows down when the margin drops below 25°.

## Physics models

* **Gravity** 1.62 m/s², vacuum (dust is ballistic with no drag), black sky, hard shadows, low polar sun
  (default 9° elevation, adjustable).
* **Vehicle.** The sprung chassis is a Rapier rigid body with real collision shapes: legs, girders,
  cross-beams, mast and battery pods all collide with rocks, structures and cargo. Each bogie is
  a 1-DOF unsprung mass on its strut. Each wheel finds the ground with a **cylinder shape-cast**,
  so it climbs rock edges with the correct contact normal. Load is shared through the walking beam,
  and the radial tyre spring/damper gives the normal force.
* **Terramechanics** (`src/sim/terramechanics.js`) uses lunar regolith parameters from Wong / Apollo
  soil mechanics: n = 1, k_c = 1.4 kN/m², k_φ = 820 kN/m³, c = 0.17 kPa, φ = 35°, K = 1.8 cm.
  * Bekker sinkage z = [3W / ((3−n)(k_c + b·k_φ)√D)]^(2/(2n+1)), with slip-sinkage
  * Compaction (rolling) resistance R_c = b(k_c/b + k_φ) z^(n+1)/(n+1)
  * Janosi–Hanamoto thrust H = (A·c + W·tanφ)·[1 − K/(s·l)·(1 − e^(−s·l/K))]
  * Lateral force from the same shear law, driven by slip angle; friction-ellipse coupling
  * Wheel spin is solved **fully implicitly** each step: bisection on the wheel torque balance, with
    the PI motor controller and brake inside the solve. This is stable even when thrust saturates in
    scrubbing pivot turns or stalls.
  * Braked wheels use a **static-friction (bristle) model** anchored to the regolith, so a parked
    rover holds on slopes and against docking-stage reaction loads without creeping.
  * Low gravity naturally means lower wheel loads, so less traction.
* **Power.** Motor electrical power (efficiency + copper loss + regen), hotel loads, hoist and steering
  power. Solar input uses the incidence angle and a terrain/structure shadow ray-cast every second.
* Every rock larger than 25 cm is a real convex-hull collider. Rendering uses the **same hull**, so
  what you see is exactly what the rover can hit. Collider streaming keeps only rocks within 140 m
  of the rover in the physics world, because Rapier's broadphase cost grows with static colliders.

## Autonomy stack (nothing uses ground truth)

1. **Perception.** The spinning LiDAR scans one sector per tick and is motion de-skewed. The camera
   does fiducial pose estimation (noise grows with range²), line-of-sight checks and a simulated rock
   detector.
2. **Localisation (SLAM)** (see also loop closure under "What's new"). Slip-compensated wheel odometry (motor torque + strut load → invert the
   soil thrust curve → slip), gyro with bias random walk, a sun-sensor heading fix, and **surveyed
   fiducial beacons** on the lander and habitat ports that give absolute fixes at both ends of the
   route. This is corrected by **correlative scan-to-map matching** on a 2.5D elevation map, with:
   * per-candidate vertical offset solving
   * per-point score normalisation (removes the "known-cells" bias)
   * variance-gated cells (vertical faces are excluded)
   * a degeneracy check: on a flat, featureless pad it correctly falls back to odometry
3. **Mapping & traversability.** Tiled 0.5 m elevation map with running mean/variance. A
   plane-fit gives slope; residuals give step/rock height; depression detection gives craters.
   Each cell is classed as free / rough / rock / slope / crater / lethal.
4. **Global planning.** Weighted A* on a 2 m costmap with hard inflation (half-width + margin) and a
   soft cost skirt. Unknown cells get optimistic-but-higher cost. Geofences are lethal.
   **Remembered-route cells are cheaper.** A 2 km plan takes ~15 ms.
5. **Teach & repeat** (after Furgale & Barfoot's visual teach-and-repeat). Every successful leg
   between sites is stored. The next trip re-validates the stored route against the current map and
   repeats it if it is still clear; otherwise it replans. The map and routes persist in the browser
   (IndexedDB), so **map it once, then it goes the old way if it worked**.
6. **Local planning.** 25 constant-curvature arcs are rolled out over the map, checking each wheel
   footprint and a **straddle-aware belly check**: rocks between the tracks are fine if they are
   lower than the clearance. Speed is governed by free distance, tip margin and faults. A pivot is
   used for large heading changes only if the sweep circle is clear. Stall, stuck and blocked
   recovery uses a collision-checked reverse.
7. **Terminal guidance.** Fiducials, then the cradle light curtain + rangefinders (straddle accuracy
   ~1 cm lateral), then the docking-ring camera (mm-level) for the port.

## Safety

* **E-STOP** removes drive torque, sets the spring brakes and locks the actuators (self-locking ball
  screws). It is latching: it requires RESET and then START (a 4 s power-on self-test). Afterwards
  the rover comes up in MANUAL, and autonomy has to be re-commanded.
* **Operator override.** Any drive input pauses autonomy, and `resume` continues it.
* **Interlocks.** Grapple is refused outside tolerance. Cargo release is inhibited unless the cargo is
  supported (on saddles or hard-docked). The module hatch opens only after the seal and leak check pass.

## Honest simplifications

* The site is procedural (seeded), not real LOLA data. The terrain is 2.2 × 2.2 km, with a decorative
  horizon ring beyond it.
* Bogie unsprung dynamics are 1-DOF (vertical). The walking-beam angle is quasi-static. Wheels are
  shape-casts against the world, not separate rigid bodies.
* The camera's detections are geometric (projection, line of sight, range-dependent noise). They are
  not image-based. The rendered camera feed is for the operator.
* Regolith properties are uniform over the site. There is no electrostatic dust adhesion model.
  Dust exclusion is represented by the docking procedure.
* Mission 9 is cooperative mapping (SCOUT-1 leads and maps; SPAN-40 plans on the shared map). The two robots do not jointly lift a payload.
* SCOUT-1 drives with a simplified kinematic track model that follows the terrain and rocks (it is ray-cast against real geometry). It is not a full Rapier multibody like SPAN-40.
* Decorative pebbles (under 12 cm) are visual only. They are below the size that matters to 1.4 m wheels.

## Code map

```
src/sim/config.js        all engineering parameters (rover, regolith, cargo, dock)
src/sim/terrain.js       terrain, craters, boulders, site layout
src/sim/world.js         Rapier world, colliders, rock streaming
src/sim/rover.js         vehicle dynamics, motors, steering, suspension, hoist, power
src/sim/terramechanics.js Bekker / Janosi wheel-soil model
src/sim/sensors.js       LiDAR, camera, IMU, odometry, sun sensor, cradle sensors
src/sim/slam.js          elevation-map SLAM, hazards, route memory, persistence
src/sim/planner.js       costmap, A*, local arc planner
src/sim/autonomy.js      mission executive, docking, faults, logistics manager
src/sim/commands.js      operator console commands
src/render/*             terrain LOD, rover model, props, effects, sky
src/ui/hud.js            operator HUD
tests/*.mjs              headless physics + autonomy runs
```
