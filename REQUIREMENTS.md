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
| M5 | Deliver & place (dock) | `dock_cyl40`, `dock_box20` | mission OK, cargo delivered, port occupied, soft capture ≤ 10 mm lat/vert, ≤ 1° yaw | <!--m5--> |
| M6 | Full logistics cycle | `cycle` | finishes in 12,000 s sim, OK, delivers every cargo | <!--m6--> |
| M7 | 2 km autonomous route, no pre-mapped path | `nav_bench`, `drift`, `faults` | legs arrive, zero warnings, SLAM error ≤ 5 % of distance, ≤ 5 m at end | <!--m7--> |
| M8 | Different cargo systems | `pickup_gt_box20`, `pickup_time_box20`, `dock_box20` | same checks as M3/M5 on the 20-ft box | <!--m8--> |
| M9 | Cooperative robot (SCOUT-1) | `scout` | finite state, separation ≥ 7 m, maps cells, gets ≥ 10 m ahead | <!--m9--> |
| M10 | Something goes wrong | `faults` (partial) | see fault table | <!--m10--> |
| M11 | AI logistics manager (priority ordering) | **not verified** | `cycle` runs the manager but no check on order or `priority` re-ordering | not verified |
| M12 | Own capability: teach & repeat | `nav_bench` | return leg logs a route-memory reuse | <!--m12a--> |
| M12 | Own capability: slip-compensated odometry | `odo` | odometry/truth within 5 % at 1–4.9 m/s | <!--m12b--> |
| M12 | Own capability: active load leveling | **not verified** | — | not verified |
| — | Vehicle dynamics baseline | `physics`, `creep`, `rocks` | speed tracking, turn, pivot, stop, park, top speed, ride range, creep, climb/block | <!--base--> |

## Mission 10 fault list

Faults from `sim.injectFault` (`src/sim/sim.js`).

| Fault (`fault …`) | Expected response (README) | Verified by | Result |
|---|---|---|---|
| `block` — boulder field on path | replan around it | `faults` (arrives at habitat) | <!--fblock--> |
| `wheel <n>` — motor failure | derate 25 %, 5 of 6 wheels driven | `faults` (wheel 3; arrives) | <!--fwheel--> |
| `lidar` — LiDAR loss | slow on proven route, else hold | **not verified** | not verified |
| `camera` — camera / fiducial loss | — | **not verified** | not verified |
| `comms` — comms loss | continue; reject commands except E-STOP | **not verified** | not verified |
| `battery <%>` — low battery | park and charge | **not verified**; with 0 on-board solar panels the wait loop cannot recover (open question) | not verified |
| `shift` — cargo shift | detect and re-centre | **not verified** | not verified |
| `port <n>` — port unavailable | hold and ask operator | **not verified** | not verified |

## Gaps

* No test for M11, leveling, or 6 of the 8 faults.
* M4 is only covered through the full cycle.
* Browser-only features (HUD, graphics, export, persistence in IndexedDB) have no automated test.
