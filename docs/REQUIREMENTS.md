# Requirements traceability

Maps the LLASO-P2 missions (M1–M12) and the Mission 10 fault list to the tests that verify them.
Test names are `npm test` entries (`tests/run-all.mjs`). Thresholds are in each test file's header comment.
Result column: last `npm test` run, see CHANGES.md for date and code state.

## Missions

| ID | Requirement (short) | Verified by | What is checked | Result |
|---|---|---|---|---|
| M1 | Leave the landing zone | `m1` (autonomy.mjs m1) | mission finishes and reports OK | <!--m1--> |
| M2 | Find & approach cargo, no manual repositioning | `m3`, `pickup_time_cyl40`, `pickup_time_box20` | approach phase completes as part of acquire | <!--m2--> |
| M3 | Pick up & secure | `pickup_gt_cyl40`, `pickup_gt_box20`, `pickup_time_*`, `m3` | grapple check, clamps lock, cargo held, loaded tip margin ≥ 25°, all wheels in contact | <!--m3--> |
| M4 | Transport 2 km with cargo | `cycle` | only end-to-end (deliveries = cargo count); no dedicated loaded-traverse test | <!--m4--> |
| M5 | Deliver & place (dock) | `dock_cyl40`, `dock_box20` | mission OK, cargo delivered, port occupied, soft capture ≤ 5 mm lat/vert, ≤ 1° yaw, on the first attempt, peak contact force ≤ 3 kN (cyl40) | <!--m5--> |
| M6 | Full logistics cycle | `cycle` | finishes in 12,000 s sim, OK, delivers every cargo | <!--m6--> |
| M7 | 2 km autonomous route, no pre-mapped path | `nav_bench`, `drift`, `faults` | legs arrive, zero warnings, SLAM error ≤ 5 % of distance, ≤ 5 m at end | <!--m7--> |
| M8 | Different cargo systems | `pickup_gt_box20`, `pickup_time_box20`, `dock_box20` | same checks as M3/M5 on the 20-ft box | <!--m8--> |
| M9 | Cooperative robot (SCOUT-1) | `scout` | finite state, separation ≥ 7 m, maps cells, gets ≥ 10 m ahead | <!--m9--> |
| M10 | Something goes wrong | `faults` (partial), `fdir` | see the two fault tables below | <!--m10--> |
| M11 | AI logistics manager (priority ordering) | `logistics` | plan lists every cargo with ETA and energy, life-support first, `priority` re-orders it, no leg feasible below the battery reserve | pass (7/7, 2026-11-11) |
| M12 | Own capability: teach & repeat | `nav_bench` | return leg logs a route-memory reuse | <!--m12a--> |
| M12 | Own capability: slip-compensated odometry | `odo` | odometry/truth within 5 % at 0.5–2.9 m/s | <!--m12b--> |
| M12 | Own capability: active load leveling | `logistics` | parked across a 5.5 deg side slope: body roll 5.0 deg with levelling off, 0.0 deg on; tip margin cost <= 2 deg | pass (5/5, 2026-11-11) |
| — | Vehicle dynamics baseline | `physics`, `creep`, `rocks` | speed tracking, turn, pivot, stop, park, top speed, ride range, creep, climb/block | <!--base--> |

## Mission 10 fault list

Faults from `sim.injectFault` (`src/sim/sim.js`).

| Fault (`fault …`) | Expected response (README) | Verified by | Result |
|---|---|---|---|
| `block` — boulder field on path | replan around it | `faults` (arrives at habitat) | <!--fblock--> |
| `wheel <n>` — motor failure | derate 25 %, 5 of 6 wheels driven | `faults` (wheel 3; arrives) | <!--fwheel--> |
| `lidar` — LiDAR loss | slow on proven route, else hold | `faults_m10` (off-route: stopped in 2.5 s, 0 m/s while blind, spare head B in 23 s, mission continues); on a proven route: 0.30 m/s crawl until head B is on, 23 s) | pass |
| `camera` — camera / fiducial loss | FDIR alarm; traverse continues (docking needs the camera) | `faults_m10` (alarm raised, 188 m covered in the next 60 s; camera dead at the port: berthing refused, 0 N on the port, module held, reason reported) | pass |
| `comms` — comms loss | continue; queue commands except E-STOP | `faults_m10` (command queued and not applied, mission continues, queue delivered on `clear comms`, E-STOP accepted) | pass |
| `battery <%>` — low battery | park and charge | `faults_m10` (12 % beside the dock at the start of a leg: diverts, couples in 150 s, charges at 1.4 A, 0.14 C, ≈61 kW full-scale). Mid-leg at 12 % it finishes the leg at the 160 rpm low-battery cap. Run down to 5 % mid-drive, 1.8 km from the dock: survival mode stops it, the keep-alive array deploys (4.8 kW) and the pack gains charge; above 30 % the mission resumes by itself. The ~22 h recharge itself is skipped in the test | pass |
| `shift` — cargo shift | detect and re-centre | `faults_m10` (derate announced, re-centred in 10.5 s, derate lifted, module held) | pass |
| `port <n>` — port unavailable | hold and ask operator | `faults_m10` (holds 48 m out with the module clamped; delivers after `clear port 1`) | pass |

## Hardware failure modes (vehicle health / FDIR)

Catalogue in `src/sim/health.js` (`fault list` in the console); procedures in `docs/FAULT_PROCEDURES.md`.
Verified by `fdir` (`tests/fdir.mjs`, `npm run test:fdir`):

| Check | What is checked | Result |
|---|---|---|
| A. Nominal drive | 3 min goto: 0 alarms, 0 spurious, vault 5–35 C | pass (0 alarms, 19.7 C) |
| B. Mode sweep | each of the 24 modes injected in turn (driving, parked BIT or on the pad as the mode needs): detected from sensor readings, right unit isolated, automatic response checked where it has one, 0 spurious alarms | pass (24/24, 0 spurious) |
| C. Repairs | dock repair with a dock spare; strut repair refused away from the dock; field motor-controller swap uses the on-board kit | pass |
| D. Random failures | switch on at a very high rate for 300 s: ≥ 2 failures happen, 0 spurious alarms, every immediately observable one is detected within 20 s (latent ones wait for motion or the BIT) | pass (5 failures, none missed) |
| E. Lunar night | survival mode holds the set points; a 354 h night costs ≤ 25 % of the pack | pass (147 W, 12.4 %) |
| F. Keep-alive array | deploys ≥ 3 kW parked, stows before driving | pass (4.8 kW) |

## Gaps

* M6: the two-cargo cycle does not finish inside its 12,000 s limit on the current working tree (2026-11-11: CM-40A delivered at 105 min, CC-20B picked up at 150 min and 661 m from PORT-2 at the limit; no collision, no abort). With the limit raised to 16,000 s it completes unaided: both delivered, mission complete at 14,999 s. See CHANGES.md §27.
* Not tested: the full ~22 h keep-alive recharge, docking the 20-ft container with a failed camera.
* M4 is only covered through the full cycle.
* Browser-only features (HUD, graphics, export, persistence in IndexedDB) have no automated test.
