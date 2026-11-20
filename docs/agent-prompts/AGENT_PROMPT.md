# AEGIS-V1 HUNCH SIMULATOR — MASTER SYSTEM SPECIFICATION & IMPLEMENTATION PROMPT

> **FOR THE AUTONOMOUS AGENT EXECUTING THIS TASK:**
> You are implementing and fixing the core simulation, autonomy, physics, sensors, graphics, and HUD systems for the **AEGIS-V1 Lunar Cargo Rover Simulator** (`NASA HUNCH SFT 2026-27 · LLASO Project 2 · LLASO-P2-XPORT-2026`).
> This prompt is completely self-contained. It contains exact file paths, line numbers, physics constants, identified root causes, architectural patterns, algorithmic solutions, and acceptance tests. Follow this guide systematically from start to finish.

---

## 1. MISSION CONTEXT & SYSTEM ARCHITECTURE

The vehicle is **AEGIS-V1**, a full-scale (1:1 from 1:16.3 scale prototype, scaling factor $K = 16.32$) rocker-bogie skid-steer cargo rover. Its mission is to autonomously leave the landing zone, navigate to a landed 40-foot pressurized cargo module ($14,940\text{ kg}$), drive under it in a squat/kneel stance, lift it with six active roller-screw electromechanical actuator (EMA) suspension struts, lock it into dual circumferential ring clamps, transport it $2.0\text{ km}$ across unmapped lunar south polar terrain to the supply building, and soft-capture dock it to a pressurized habitat port ($\le 5\text{ mm}$ lateral/vertical error, $\le 1^\circ$ yaw error)—with zero astronaut EVA.

### Tech Stack
* **Rendering & Visuals:** Three.js, WebGL, custom shader/post-processing (`src/render/`, `src/style.css`, `index.html`)
* **Physics Engine:** Rapier 3D rigid-body physics (WASM) stepped at $120\text{ Hz}$ (`PHYS.dt = 1/120`) + Bekker-Wong / Janosi-Hanamoto lunar regolith terramechanics (`src/sim/terramechanics.js`, `src/sim/rover.js`)
* **Autonomy & SLAM:** 2.5D elevation/hazard grid SLAM ($0.5\text{ m}$ cell resolution), global A* planner ($2\text{ m}$ costmap), reactive LocalPlanner arc evaluator, and ES6 generator coroutines (`src/sim/autonomy.js`, `src/sim/slam.js`, `src/sim/planner.js`)
* **Workspace Root:** `/Users/harshan/Hunch 22/` (iCloud: `/Users/harshan/Library/Mobile Documents/com~apple~CloudDocs/Hunch 22`)
* **Dev Server:** `npm run dev` $\to$ `http://127.0.0.1:5173`
* **Test Suites:** `npm test`, `npm run test:quick`, `npm run test:long`

### File Map
```
src/
├── sim/
│   ├── autonomy.js    ← Mission executive (1874 lines). Generator coroutines (*missionCycle, *acquire, *navTo, etc.)
│   ├── commands.js    ← Console & UI command dispatcher ('mission', 'speed', 'timescale', 'warp', etc.)
│   ├── config.js      ← Engineering constants (ROVER, WORLD, MOON, CARGO_TYPES, DOCK, PHYS, THERMAL)
│   ├── dynamics.js    ← Real-time vehicle dynamics (SSF, stopping distance, drawbar pull, power)
│   ├── planner.js     ← Global A* planner (2m costmap) & LocalPlanner (reactive arc rollout on 0.5m grid)
│   ├── rover.js       ← Vehicle physics (929 lines). Rocker-bogie, hub motors, smart-RPM, EMA struts, ring clamps
│   ├── sensors.js     ← LiDAR, monocular camera, IMU, CASS laser rangefinders, odometry
│   ├── sim.js         ← Main loop (394 lines). stepOnce(), warpStep(), timeScale, advance(), E-STOP
│   ├── slam.js        ← LiDAR SLAM (776 lines). 2.5D elevation & hazard map, scan matching, route memory
│   ├── terrain.js     ← Procedural lunar heightfield (2.5 x 2.5 km)
│   └── world.js       ← Rapier world entities, cargo stands, habitat docking ports, landing pad
├── render/
│   ├── aegis.js       ← Three.js rover visual model, wheel group positioning, fender meshes, strut visuals
│   ├── cadModels.js   ← CAD model loaders and materials
│   ├── props.js       ← 3D props (landing pad, habitat, cargo stands, ports)
│   └── scene.js       ← Three.js scene, camera controls, lighting, render loop
└── ui/
    ├── hud.js         ← HUD controller (574 lines). DOM panels, mission buttons, telemetry tables, time warp UI
    ├── instruments.js ← Dynamics instruments overlay
    ├── thermalPanel.js← Thermal brain display
    └── style.css      ← UI styling, HUD layout, panels
```

### Critical Architectural Rules
1. **Never use Ground Truth in Autonomy:** All navigation and manipulation decisions must use estimated state (`this.slam.est`) and sensor measurements (`this.sensors.*`). Never inject `sim.rover.pose()` into decision loops.
2. **Generator Coroutines:** Missions in `autonomy.js` are `function*` generators. The sim ticks `this.gen.next(dt)` once per autonomy tick. Use `yield` at frame boundaries. Never write infinite synchronous loops.
3. **Physical Scale Constants:** Always read vehicle and environmental parameters from `src/sim/config.js`.

---

## 2. DETAILED REQUIREMENTS & IMPLEMENTATION SPECIFICATIONS

---

### TASK 1: Full-Mission Autonomy Loop & RPM Speed Optimization
* **Primary Files:** `src/sim/autonomy.js` (lines 1819–1850), `src/sim/commands.js` (lines 99–117), `src/sim/rover.js` (lines 122–165)

#### Root Cause Analysis:
In `src/sim/autonomy.js`, the generator `*missionCycle()` (line 1819) begins with:
```javascript
let r = yield* this.surveyRoute(item.cargo); if (r.ok === false) return r;
```
`surveyRoute()` drives the empty rover across the entire 2.0 km map to the habitat port, holds for 60 seconds waiting for an optical fiducial fix, and drives 2.0 km back to the pad before touching the cargo. To the user, clicking "Do Full Mission" makes the rover drive somewhere completely random instead of picking up the module.

#### Implementation Steps:
1. **Remove / Bypass Mandatory Pre-Survey Drive:**
   * In `*missionCycle()` in `src/sim/autonomy.js`, bypass `yield* this.surveyRoute(item.cargo)`.
   * The mission loop must directly sequence:
     ```javascript
     // 1. Acquire cargo (approach -> kneel -> rise -> clamp)
     let r = yield* this.acquire(item.cargo);
     if (r.ok === false) return r;

     // 2. Transport to destination port and dock
     r = yield* this.deliver(item.cargo.dest);
     if (r.ok === false) return r;
     ```
2. **Continuous Multi-Cargo Autonomous Loop:**
   * Once cargo 1 (CM-40A) is docked at Port 1, check the remaining manifest (`CC-20B`, `CP-36C`).
   * If battery SOC > `reserveSoc` (0.20), the rover must automatically drive back to pad staging (`this.sim.terrain.pad.staging`), locate the next cargo, and continue the cycle with no pauses or manual clicks.
   * If SOC $\le$ `reserveSoc`, autonomously route to the power dock (`this.chargeAtDock(0.90)`), recharge, and resume.
3. **RPM & Speed Strategy:**
   * Motor specs in `config.js`: `protoNoLoadRpm: 300`, `protoCapRpm: 280`, `protoCrawlRpm: 120`.
   * **Open Traverse Cruise:** Prioritize speed. During open-field transit, set `this.turboOk = true` and allow smart-RPM to utilize the full 280–300 RPM ceiling on flat, firm ground.
   * **Precision Approach Crawl:** Enforce a strict 120 RPM cap (`protoCrawlRpm = 120`) during:
     * The final 15 meters of cargo approach and under-module docking.
     * Strut kneeling and lifting operations.
     * The final 20 meters of habitat vestibule entry and soft-capture push.
   * Immediately restore full smart-RPM once clear of approach zones.

---

### TASK 2: Time-Warp Multipliers (2×, 3×, 5×, 10×, 20×, 50×)
* **Primary Files:** `src/sim/sim.js` (lines 318–394), `src/ui/hud.js` (lines 80–95), `src/render/scene.js`

#### Root Cause Analysis:
`sim.js` implements kinematic rails warp only when `timeScale >= 50` via `warpStep()`. At 2× through 20×, `stepAuto()` falls back to `stepOnce()` (full 120 Hz Rapier physics). In `advance(realDt, budgetMs=20)`, the frame loop attempts to step physics up to `realDt * timeScale`. At 5×, 10×, or 20×, this exceeds `budgetMs = 20` ms and `PHYS.maxSubSteps = 40`, dropping physics steps. As a result, the sim runs at near 1× real-time regardless of button selection.

#### Implementation Steps:
1. **Dynamic Physics Time Budget & Substep Scaling:**
   * In `src/sim/sim.js` `advance(realDt, budgetMs = 20)`:
     ```javascript
     // Dynamically scale physics budget and max substeps based on timeScale
     const scale = Math.max(1, this.timeScale);
     const effectiveBudget = Math.min(budgetMs * Math.min(scale, 10), 160); // Allow up to 160ms CPU budget
     const effectiveMaxSteps = Math.min(PHYS.maxSubSteps * scale, 240);     // Allow up to 240 substeps
     ```
2. **Selective Render Skipping (Frame Throttling):**
   * In `src/render/scene.js` (or main animation loop):
     * When `timeScale >= 5`, render 1 visual frame every `Math.floor(timeScale / 2)` animation frames.
     * The physics integrator continues stepping every tick, but bypassing Three.js draw calls frees 80%+ of frame time for physics catch-up.
3. **HUD Multiplier UI & Active State Indicator:**
   * In `src/ui/hud.js`:
     * Add `3×` to the timescale button array if missing (`[0, 1, 2, 3, 5, 10, 20, 50]`).
     * On button click, remove `.on` from all timescale buttons and add `.on` to the clicked button.
     * Render a persistent warp badge in the top-right corner:
       ```html
       <div id="warpIndicator" class="warp-badge">▶▶ 10× WARP</div>
       ```
       Show when `timeScale >= 2`, hide when `timeScale === 1`.
     * Ensure the `❚❚` (pause) button cleanly freezes physics (`timeScale = 0`) and changes badge to `PAUSED`.

---

### TASK 3: Sensor Suite Upgrade — Stereo Depth Camera + 360° 3D LiDAR
* **Primary Files:** `src/sim/config.js` (lines 142–146), `src/sim/sensors.js`, `src/sim/slam.js`, `src/sim/planner.js`

#### Implementation Steps:
1. **Stereo Depth Camera System:**
   * In `src/sim/config.js`, update `ROVER.camera`:
     ```javascript
     camera: {
       mountLocal: { x: k(0.275), y: k(0.29), z: 0 },
       hFovDeg: 90,           // Widened from 70° to 90° for peripheral awareness
       pitchDeg: -12,
       range: 70,
       stereo: true,          // Enable stereo disparity mode
       baselineM: 0.18,       // 18 cm optical baseline
       depthRange: 35,        // Dense stereo depth mapping up to 35m
       disparityNoise: 0.03
     }
     ```
   * In `src/sim/sensors.js`, add stereo depth disparity projection: project depth-detected obstacle points into `slam.integrate()` alongside LiDAR hits.
2. **360° 3D LiDAR with Dynamic Cargo Occlusion Masking:**
   * In `src/sim/config.js`, update `ROVER.lidar`:
     ```javascript
     lidar: {
       mountLocal: { x: 0, y: k(0.35), z: 0 }, // Center-top avionics mast (~5.71m Y)
       channels: 32,                          // 32-beam vertical array
       vFovDeg: [-30, 15],                    // -30° down to +15° up
       tiltDeg: 0,
       hRes: 360,                             // Full 360° continuous azimuth sweep
       range: 90,
       rateHz: 10,                            // 10 Hz rotation rate
       noise: 0.02
     }
     ```
   * **Dynamic Self-Occlusion Filter:** In `src/sim/sensors.js`, when `rover.loaded === true`, the carried cylindrical cargo (radius ~3.98 m, length ~12.2 m) occupies the space directly behind the mast. Raycasts hitting the rover chassis AABB or the carried cargo cylinder must be masked out so the vehicle does not map its own payload as a collision hazard.
3. **Continuous 360° Costmap Integration:**
   * Ensure `src/sim/slam.js` and `src/sim/planner.js` incorporate obstacles from all 360° directions.
   * Update obstacle safety inflation in `src/sim/planner.js` from 5.8 m to 6.5 m for loaded paths to guarantee full clearance around rocks, lander legs, and structures.

---

### TASK 4: Fix Wheel & Fender Sinking Visual Bug During Kneel
* **Primary Files:** `src/render/aegis.js` (lines 194–215), `src/sim/rover.js`, `src/sim/config.js`

#### Root Cause Analysis:
In `src/render/aegis.js` (lines 206–210):
```javascript
const ride = cl(rover.rideOffset / K, -0.045, 0.065);
for (let i = 0; i < 6; i++) {
  const wg = this.wheelGroups[i], w = rover.wheels[MODEL_TO_SIM[i]];
  wg.group.position.y -= ride;
  wg.strut.setLength(ride);
  this.tires[i].rotation.y = -w.angle;
}
```
When the rover kneels to $-0.653\text{ m}$ (`ride.kneel`), the physical chassis lowers. The Three.js visual model shifts `wg.group.position.y` without clamping to the ground surface heightfield. Because the rocker-bogie visual kinematics mismatch the ground elevation, the wheels and fenders clip into the lunar terrain.

#### Implementation Steps:
1. **Terrain Height Clamping for Visual Wheels:**
   * For each wheel station visual mesh, compute world position $(x_w, z_w)$. Query true terrain elevation: `const groundY = sim.terrain.heightAt(xw, zw);`.
   * Enforce that visual wheel bottom never penetrates the terrain:
     ```javascript
     const minCenterY = (groundY + ROVER.wheel.radius) / K;
     wg.group.position.y = Math.max(wg.group.position.y, minCenterY);
     ```
2. **Fender Alignment:**
   * Ensure fender visual meshes remain strictly attached to the wheel hub group so they follow the clamped wheel elevation and never intersect regolith.
3. **Smooth Suspension Easing (NASA Flight Emulation):**
   * Real planetary roller screws operate smoothly over several seconds.
   * In `src/sim/rover.js`, replace abrupt linear ride-height steps with S-curve / critically damped interpolation:
     * Squat-down duration: 3.5 to 4.5 seconds with ease-in/ease-out.
     * Lift-up duration: 3.5 to 4.5 seconds with smooth damping.

---

### TASK 5: Lunar Cargo Research Report
* **Deliverable:** `docs/LUNAR_CARGO_RESEARCH.md`
* **Status:** **PRE-COMPLETED.** A comprehensive research report has been generated at `docs/LUNAR_CARGO_RESEARCH.md` covering NASA ATHLETE, LSC, VIPER, RASSOR, Chang'e 5/6, Lunokhod 1/2, ESA EL3, and Artemis HLS, comparing them with AEGIS-V1. Use this file as your reference for flight heritage and design decisions.

---

### TASK 6: Fix Loading Bay Stand Exit (Post-Pickup Departure)
* **Primary Files:** `src/sim/autonomy.js` (lines 1350–1375, 1420–1438), `src/sim/world.js`

#### Root Cause Analysis:
After the ring jaws close on the module, `_needStandExit` (line 1420) commands forward motion or prematurely turns while the rover tires and 12-meter module are still nestled inside the 4-post stand structure (`T.lugXs`, `T.lugZ = 5.63 m`). In 1/6 g with a 23-ton gross mass, lateral skid-steer scrubbing pins the module or tires against the stand posts.

#### Implementation Steps:
1. **Full Lift Verification:**
   * Do not initiate movement until suspension struts have reached 100% of `ROVER.ride.carry` (+0.816 m) and the lugs are confirmed clear of stand saddles.
2. **Mandatory Straight Reverse Departure:**
   * Enforce a pure straight reverse departure for at least 10–12 meters:
     ```javascript
     // Command straight reverse until clear of the stand perimeter
     this.cmd(-0.35, 0, 0);
     ```
3. **Lock Yaw Steering Inside Stand Footprint:**
   * Inhibit all yaw commands (`wz = 0`) until the rover's center is $> 12\text{ m}$ away from the stand origin.
   * Only then execute a wide arc turn (radius $\ge 18\text{ m}$) onto the planned transit corridor.

---

### TASK 7: Three-Layer HUD Overhaul
* **Primary Files:** `src/ui/hud.js`, `src/style.css`

#### Implementation Steps:
Implement a 3-layer HUD architecture cycled via keyboard key **`H`** or a top-right `[☰ HUD]` button:

1. **Layer 0 — Clean / Cinematic View:**
   * Hide all telemetry boxes, panels, tables, and top bar.
   * Display only the full 3D simulation canvas with a minimal, semi-transparent `[☰]` menu button in the corner.
2. **Layer 1 — Essential HUD (Default Operational):**
   * Show only high-priority operational telemetry in sleek corner widgets:
     * **Speed & Drive:** Current Speed (`m/s`), Motor RPM tier badge (`CRUISE`, `FAST`, `TURBO`, `CRAWL`).
     * **Power & Energy:** Battery SOC (%) bar, power draw (kW), estimated endurance.
     * **Mission Guidance:** Active phase name, destination, distance remaining (`e.g., TRANSIT → PORT-1 | 1,840 m`).
     * **Warp Status:** Active time-warp multiplier (`1×, 10×, 50×`).
     * **Navigation Mini-Map:** Compact 200×200 px top-down route preview.
     * **Alarms:** Dynamic banner (only visible when active alarms exist).
3. **Layer 2 — Full Engineering HUD:**
   * Reveal all existing panels: SLAM elevation grid canvas, thermal brain panel, terramechanics slip/sinkage table, motor temperature readouts, FDIR diagnostic cards, and developer console.
4. **Controls & Discoverability:**
   * Add a prominent "Hide All HUD" button in Layer 1 and Layer 2 headers.
   * Cycle smoothly: `Layer 1 -> Layer 2 -> Layer 0 -> Layer 1`.

---

### TASK 8: Navigation Smoothness — Eliminate Random Stops & Wiggling
* **Primary Files:** `src/sim/autonomy.js` (`*navTo()`, lines 648–900), `src/sim/planner.js`, `src/sim/rover.js`

#### Root Cause Analysis:
1. **Phantom Stops:** The stopping-distance governor decelerates to zero when unmapped `UNKNOWN` cells appear along the path ahead, even on flat terrain before the LiDAR scan cycle completes.
2. **Heading Oscillation (Wiggling):** Skid-steer control uses a tight heading deadband ($\pm 1^\circ$) and short pure-pursuit lookahead ($< 6\text{ m}$). Delayed regolith shear reaction causes the vehicle to continuously overshoot and hunt left-right.
3. **Replan Thrashing:** Global A* replans every 5 s or upon minor obstacle flickers, causing path jumps.

#### Implementation Steps:
1. **Eliminate Phantom Stops on Unknown Cells:**
   * In `*navTo()`, distinguish between `LETHAL` / `ROCK` obstacles and benign `UNKNOWN` cells.
   * On flat terrain (incline $< 5^\circ$), allow a continuous coast speed ($\ge 0.5\text{ m/s}$) for up to 12 meters through unmapped cells while LiDAR integrations finish.
   * Add a 2.5-second hysteresis to the SLAM confidence watchdog to prevent transient dropouts from aborting the leg.
2. **Eliminate Heading Oscillation (Wiggling / Jiggling):**
   * Increase heading deadband to $\pm 3.0^\circ$ (within $\pm 3^\circ$, command `wz = 0`).
   * Increase minimum pure-pursuit lookahead distance to $\ge 10.0\text{ m}$ (`lookAhead = Math.max(10.0, speed * 3.5)`).
   * Implement **yaw-rate damping**: when $|\text{headingError}| < 5^\circ$ and $|\dot{\psi}_{\text{imu}}| > 0.03\text{ rad/s}$, scale back the commanded yaw rate by 50% to prevent overshoot.
   * Apply an acceleration limiter on yaw commands (`maxYawJerk = 0.1 rad/s²`).
3. **Replan Hysteresis:**
   * Only trigger a path replan if a confirmed lethal obstacle blocks the trajectory within 25 m, or if cumulative path cost increases by $> 30\%$. Lock committed trajectory segments for at least 15 m of travel.

---

### TASK 9: Chrome Verification & Headless Regression Testing

1. **Headless Regression Test Suite:**
   * Execute:
     ```bash
     npm run test:quick
     ```
   * Ensure that `physics`, `rocks`, `lander`, `thermal`, `planner`, `odo`, `fdir`, and `creep` all report `PASS`.
2. **Chrome End-to-End Mission Verification:**
   * Run `npm run dev` and open `http://127.0.0.1:5173`.
   * Click **"M6 Full cycle"** (or type `mission 6`).
   * Click the **10×** time-warp button.
   * Verify against the following acceptance criteria:
     * [ ] Rover drives straight to Cargo CM-40A without an empty 4 km survey detour.
     * [ ] Visual wheels and fenders never penetrate the terrain surface during kneeling.
     * [ ] Rover rises smoothly (3–5 s), clamps cargo, and reverses straight 10–12 m clear of the stand.
     * [ ] Traverses 2.0 km at 280–300 RPM on open ground without phantom stops or heading wiggling.
     * [ ] Slows to 120 RPM crawl at Port 1 and soft-capture docks within spec ($\le 5\text{ mm}$, $\le 1^\circ$).
     * [ ] Returns autonomously to the pad for Cargo 2 (CC-20B).
     * [ ] Pressing `H` cycles cleanly through Clean, Essential, and Engineering HUD layers.
     * [ ] Time warp at 10× visually accelerates the mission ~10× faster.

---

## 3. STRICT AGENT OPERATING DIRECTIVES
* **Preserve Integrity:** Maintain existing comments and docstrings.
* **Consolidate Constants:** Place all physical and geometric constants in `src/sim/config.js`.
* **Zero Regressions:** Do not break existing passing tests in `tests/`.
* When all tasks are implemented and verified, append `<!-- GOAL_COMPLETE -->` to your summary.
