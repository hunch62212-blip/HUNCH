# Requirements traceability

Maps the LLASO-P2 missions (M1–M12) and the Mission 10 fault list to the tests that verify them.
Test names are `npm test` entries (`tests/run-all.mjs`). Thresholds are in each test file's header comment.
Result column: written by `node scripts/record-results.mjs` from the last `tests/run-all.mjs` record (`tests/out/results.json`)
and the seed sweeps (`tests/seed_sweep.mjs` → `tests/out/<name>.seeds.json`). Each cell names the commit and date it was
measured at. A requirement whose tests are not in the current record reads "not run at <commit>": an old pass is not
carried forward. Earlier hand-written results are kept under "Last recorded by hand" below for history only.

Thresholds are the requirement's own. The tests may not relax them (v1.7.0 passed M6 only with the cap raised from
12,000 s to 24,000 s; `tests/run-all.mjs` now runs `cycle` at 12,000 s). M6 and M7 also need a seed sweep (≥ 10 map seeds,
`node tests/seed_sweep.mjs cycle 10`, `... nav_bench 10`): one fixed map seed proves little.

## Missions

| ID | Requirement (short) | Verified by | What is checked | Result |
|---|---|---|---|---|
| M1 | Leave the landing zone | `m1` (autonomy.mjs m1) | mission finishes and reports OK | <!--m1-->not run at 7729e75<!--/m1--> |
| M2 | Find & approach cargo, no manual repositioning | `m3`, `pickup_time_cyl40`, `pickup_time_box20` | approach phase completes as part of acquire | <!--m2-->not run at 7729e75<!--/m2--> |
| M3 | Pick up & secure | `pickup_gt_cyl40`, `pickup_gt_box20`, `pickup_time_*`, `m3` | grapple check, clamps lock, cargo held, loaded tip margin ≥ 25°, all wheels in contact | <!--m3-->not run at 7729e75<!--/m3--> |
| M4 | Transport 2 km with cargo | `cycle` | only end-to-end (deliveries = cargo count); no dedicated loaded-traverse test | <!--m4-->not run at 7729e75<!--/m4--> |
| M5 | Deliver & place (dock) | `dock_cyl40`, `dock_box20` | mission OK, cargo delivered, port occupied, soft capture ≤ 5 mm lat/vert, ≤ 1° yaw, on the first attempt, peak contact force ≤ 3 kN (cyl40) | <!--m5-->not run at 7729e75<!--/m5--> |
| M6 | Full logistics cycle | `cycle` | finishes in 12,000 s sim, OK, delivers every cargo | <!--m6-->not run at 7729e75; seed sweep not run<!--/m6--> |
| M7 | 2 km autonomous route, no pre-mapped path | `nav_bench`, `drift`, `faults` | legs arrive, zero warnings, SLAM error ≤ 5 % of distance, ≤ 5 m at end | <!--m7-->not run at 7729e75; seed sweep not run<!--/m7--> |
| M8 | Different cargo systems | `pickup_gt_box20`, `pickup_time_box20`, `dock_box20` | same checks as M3/M5 on the 20-ft box | <!--m8-->not run at 7729e75<!--/m8--> |
| M9 | Cooperative robot (SCOUT-1) | `scout` | finite state, separation ≥ 7 m, maps cells, gets ≥ 10 m ahead | <!--m9-->not run at 7729e75<!--/m9--> |
| M10 | Something goes wrong | `faults` (partial), `fdir` | see the two fault tables below | <!--m10-->not run at 7729e75; seed sweep not run<!--/m10--> |
| M11 | AI logistics manager (priority ordering) | `logistics` | plan lists every cargo with ETA and energy, life-support first, `priority` re-orders it, no leg feasible below the battery reserve | <!--m11-->not run at 7729e75<!--/m11--> |
| M12 | Own capability: teach & repeat | `nav_bench` | return leg logs a route-memory reuse | <!--m12a-->not run at 7729e75<!--/m12a--> |
| M12 | Own capability: slip-compensated odometry | `odo` | odometry/truth within 5 % at 0.5–2.9 m/s | <!--m12b-->not run at 7729e75<!--/m12b--> |
| M12 | Own capability: active load leveling | `logistics` | parked across a 5.5 deg side slope: body roll 5.0 deg with levelling off, 0.0 deg on; tip margin cost <= 2 deg | <!--m12c-->not run at 7729e75<!--/m12c--> |
| — | Vehicle dynamics baseline | `physics`, `creep`, `rocks` | speed tracking, turn, pivot, stop, park, top speed, ride range, creep, climb/block | <!--base-->not run at 7729e75<!--/base--> |

## Mission 10 fault list

Faults from `sim.injectFault` (`src/sim/sim.js`).

| Fault (`fault …`) | Expected response (README) | Verified by | Result |
|---|---|---|---|
| `block` — boulder field on path | replan around it | `faults` (arrives at habitat) | <!--fblock-->not run at 7729e75<!--/fblock--> |
| `wheel <n>` — motor failure | derate 25 %, 3 of the 4 driven wheels remain (outer wheels driven, middle ML/MR passive idlers) | `faults` (a driven wheel; arrives) | <!--fwheel-->not run at 7729e75<!--/fwheel--> |
| `lidar` — LiDAR loss | slow on proven route, else hold | `faults_m10` (off-route: stopped in 2.5 s, 0 m/s while blind, spare head B in 23 s, mission continues); on a proven route: 0.30 m/s crawl until head B is on, 23 s) | <!--flidar-->not run at 7729e75<!--/flidar--> |
| `camera` — camera / fiducial loss | FDIR alarm; traverse continues (docking needs the camera) | `faults_m10` (alarm raised, 188 m covered in the next 60 s; camera dead at the port: berthing refused, 0 N on the port, module held, reason reported) | <!--fcamera-->not run at 7729e75<!--/fcamera--> |
| `comms` — comms loss | continue; queue commands except E-STOP | `faults_m10` (command queued and not applied, mission continues, queue delivered on `clear comms`, E-STOP accepted) | <!--fcomms-->not run at 7729e75<!--/fcomms--> |
| `battery <%>` — low battery | park and charge | `faults_m10` (12 % beside the dock at the start of a leg: diverts, couples in 150 s, charges at 1.4 A, 0.14 C, ≈61 kW full-scale). Mid-leg at 12 % it finishes the leg at the 160 rpm low-battery cap. Run down to 5 % mid-drive, 1.8 km from the dock: survival mode stops it, the keep-alive array deploys (4.8 kW) and the pack gains charge; above 30 % the mission resumes by itself. The ~22 h recharge itself is skipped in the test | <!--fbattery-->not run at 7729e75<!--/fbattery--> |
| `shift` — cargo shift | detect and re-centre | `faults_m10` (derate announced, re-centred in 10.5 s, derate lifted, module held) | <!--fshift-->not run at 7729e75<!--/fshift--> |
| `port <n>` — port unavailable | hold and ask operator | `faults_m10` (holds 48 m out with the module clamped; delivers after `clear port 1`) | <!--fport-->not run at 7729e75<!--/fport--> |

## Hardware failure modes (vehicle health / FDIR)

Catalogue in `src/sim/health.js` (`fault list` in the console); procedures in `docs/FAULT_PROCEDURES.md`.
Verified by `fdir` (`tests/fdir.mjs`, `npm run test:fdir`):

| Check | What is checked | Result |
|---|---|---|
| A. Nominal drive | 3 min goto: 0 alarms, 0 spurious, vault 5–35 C | <!--fdirA-->not run at 7729e75<!--/fdirA--> |
| B. Mode sweep | each of the 24 modes injected in turn (driving, parked BIT or on the pad as the mode needs): detected from sensor readings, right unit isolated, automatic response checked where it has one, 0 spurious alarms | <!--fdirB-->not run at 7729e75<!--/fdirB--> |
| C. Repairs | dock repair with a dock spare; strut repair refused away from the dock; field motor-controller swap uses the on-board kit | <!--fdirC-->not run at 7729e75<!--/fdirC--> |
| D. Random failures | switch on at a very high rate for 300 s: ≥ 2 failures happen, 0 spurious alarms, every immediately observable one is detected within 20 s (latent ones wait for motion or the BIT) | <!--fdirD-->not run at 7729e75<!--/fdirD--> |
| E. Lunar night | survival mode holds the set points; steady-state 354 h night (closed form, `docs/THERMAL_BUDGET.md`): docked, the dock bank carries it with ≥ 1.25× margin; stranded off the dock ≤ 60 % of the pack (the old ≤ 25 % needs ~100 W of isotope heat) | <!--fdirE-->not run at 7729e75<!--/fdirE--> |
| F. Keep-alive array | deploys ≥ 3 kW parked, stows before driving | <!--fdirF-->not run at 7729e75<!--/fdirF--> |

## Last recorded by hand (history, not current results)

| Row | Value | When | Note |
|---|---|---|---|
| M11 | pass 7/7 | 2026-11-11 | before the K = 8.16 rescale |
| M12 levelling | pass 5/5 | 2026-11-11 | before the rescale; the hook-lift redesign (review item 3) removes active levelling |
| Faults lidar/camera/comms/battery/shift/port | pass | v1.7.x | battery row quoted a ≈61 kW full-scale charge; the charger is 20 kW |
| FDIR A–F | pass (E: 147 W, 12.4 %) | before the 150 kWh pack | the 147 W / 12.4 % was a transient on the old pack; steady state is 232 W (55 %) at 150 kWh, see E |
| M6 | v1.7.0 "pass" at 17,000–18,000 s | 2026-11 | only with the cap at 24,000 s; the requirement is 12,000 s. Last full run at 7729e75: 0/3 delivered, boxed in |

## Assumptions the verification depends on

* **Cargo placement.** The concept depends on how the lander leaves the cargo. Until v1.8.0 the straddle pickup assumed the
  lander sets each module on stands about 2.5 m high (`CARGO_TYPES.*.standY`) so the rover can drive under it. With the
  hook-lift arm (review item 3, `HOOKLIFT` in `config.js`) the assumption is: **the lander sets the module down on the
  regolith on its two skid shoes, hook bail at one end, with ≥ 15 m of open, ≤ 5° ground in front of the bail.** This is a
  lander interface requirement; it must be agreed with the lander side, not assumed by the rover team.
* **Map seeds.** Results on the fixed test seed (`WORLD.seed`) are one sample. Success rates come from the seed sweep.
* **Loaded routes.** Until the loaded tip margin is ≥ 35° on the design slope, loaded trips only use routes driven empty first
  (`MISSION.preSurveyRoute`).

## Gaps

* M6 is not met: the last full run at 7729e75 delivered 0 of 3 cargos (stopped by the 14° loaded tilt reflex, then boxed in after 4 reverse recoveries). v1.7.0's pass needed 17,000–18,000 s against the 12,000 s requirement.
* M7 is not met: `nav_bench` was boxed in on both legs in the last full run, and its "zero warnings" check already failed on HEAD (cargo-estimate warnings).
* E (lunar night): the 25 % limit is not met stranded off the dock at 150 kWh (55 % steady state, 232 W); the night solution is to park on the dock (`docs/THERMAL_BUDGET.md`, review item 7). The earlier 36–45 % was an 11-minute transient, not the steady state.
* M10's fault detection was true by construction for several modes (detectors read the injected value); see review item 8.
* Not tested: the full ~22 h keep-alive recharge, docking the 20-ft container with a failed camera.
* M4 is only covered through the full cycle.
* Browser-only features (HUD, graphics, export, persistence in IndexedDB) have no automated test.
