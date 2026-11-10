# AEGIS-V1 survivability and serviceability

What lunar rovers that actually flew (or are being built for NASA) carry to survive, what AEGIS-V1 was missing,
and what this version adds. The failure catalogue with every detection rule and repair procedure is generated
from the code: [FAULT_PROCEDURES.md](FAULT_PROCEDURES.md). Implementation: `src/sim/health.js`; constants:
`THERMAL`, `KEEPALIVE`, `FAILURES` in `src/sim/config.js`. All new numbers are **[ASSUMED]** engineering estimates.

## 1. What other lunar rovers carry

| Survival feature | Apollo LRV (1971-72) | Lunokhod 1/2 (1970-73) | Yutu-2 (2019-) | VIPER (built 2024) | Artemis LTV (in development) | AEGIS-V1 before | AEGIS-V1 now |
|---|---|---|---|---|---|---|---|
| Thermal control | wax heat sinks, radiators with dust covers | sealed body, fans, louvered lid | radiators, heaters | warm-box vault, top radiators, loop heat pipes, thermal valve closes radiators at 10 C | must survive the lunar night | fixed 600 W heater line in the power budget, no temperatures | vault + battery thermal model, louvered side radiators, thermostatic heaters on redundant controllers |
| Lunar night | not needed (crewed day sortie) | Po-210 radioisotope heater, hibernate | RHU, hibernate, wake on sun | not designed for the night | full 14-day night (older RFI: 85-125 h) | charged at the dock only | survival mode: ~0.15 kW, a 354 h night costs ~12 % of the pack (tested) |
| On-board generation | none (primary batteries) | solar lid | solar wings | solar panels | solar | none (0 panels) | 12 m2 keep-alive blanket on a 6.5 m mast, parked only |
| Redundant power | two independent batteries, either powers all | | | | | one 420 kWh pack | 4 strings with their own contactors; PDU converters A/B diode-OR'd |
| Redundant computers | | | | integrated avionics unit | | one | flight computers A/B (hot standby, checkpoint), watchdog, EDAC memory, SEU reset |
| Mobility failure | each wheel's drive could be disengaged to free-wheel | each wheel could be disconnected (break bolt) | | active suspension can "walk" out of soft soil | | wheel motor fault = free-rolling only | per-wheel declutch on seize, 5-wheel derated driving, hub cooling and resolver monitoring |
| Dust | fenders (Apollo 17 repaired one with tape and maps) | | | dust-tolerant mechanisms | electrodynamic dust shield (EDS) | none | EDS on LiDAR / camera windows with automatic cleaning cycles; louver blade cleaning in the procedures |
| Navigation in the dark | | | | headlights, 4 hazcams, star tracker | | headlights, 1 LiDAR, 1 camera | + front/rear stereo hazard cameras on the bumpers (visual), IMU A/B with a third-gyro voter |
| Communications | direct-to-Earth (LCRU) | direct-to-Earth | via Queqiao relay | direct-to-Earth (DSN) | | comms modelled as a flag | primary surface-network radio + UHF backup, four corner omni antennas outboard of the module, locator beacon |
| Fault protection | crew | ground | ground + safe mode | flight software safing | autonomy + teleoperation | 8 injectable faults, 2 tested | 24 failure modes on 20 replaceable units, each with sensors, an isolation rule, an automatic response and a repair; all 24 tested |
| Maintenance | crew field repairs | none | none | none | crew-serviceable units over a 10-year life | not considered | every unit has a bay, an access path, a working height, spares and a timed repair procedure |

Sources: NASA VIPER thermal and navigation hardware (NTRS 20240013903; IAC-24 VIPER paper, NTRS 20240010794);
NASA LTV reporting 2025-26 (National Geographic; Aviation Week). LRV and Lunokhod details are from the
standard mission histories.

## 2. Gaps found and what was added

1. **No thermal control or night survival.** Added a two-node thermal model (avionics vault, battery). Radiators
   are vertical, under the deck edges, because the carried 7.96 m module covers the whole deck and would block a
   VIPER-style top radiator. Each side alone can reject the full day load, so one stuck louver is not a mission
   loss. Heater power now comes from the thermostat instead of a fixed 600 W (see CHANGES.md).
2. **No way to recover from a flat battery away from the dock** (REQUIREMENTS.md listed this as an open question
   for `fault battery`). Added automatic survival mode below 6 % state of charge and the keep-alive solar blanket.
   In sun the blanket gives ~4.9 kW at normal incidence, enough to hold survival loads and slowly recharge.
3. **Single points of failure in avionics, power and comms.** Added A/B flight computers, A/B PDU converters,
   A/B IMUs with a third-gyro voter, primary/backup radios, A/B heater controllers, 4 battery strings.
4. **No failure detection.** Faults were flags that the rest of the code read directly. Now every failure only
   changes what its sensors read; FDIR detectors find it from those readings, isolate the unit, and run the
   automatic response. Detectors also run with no failure present, so false alarms are counted (the test requires 0).
5. **Electronics not serviceable.** The avionics sat in the pod on the deck, 3.7 m up and under the module.
   The service layout below moves every replaceable electronics unit to a bay reachable from the ground.
6. **Dust.** Added window dust as a failure mode with electrodynamic dust shield cleaning, and louver / hub fin
   cleaning in the repair procedures.
7. **No random failures.** Added a seeded random-failure generator with an on/off switch.

## 3. Service layout (electronics reachable and repairable)

The rover kneels (ride -0.65 m) before any repair, which brings the bays to working height.

| Bay | Contents | Working height | Access |
|---|---|---|---|
| AV, belly-tub nose | flight computers A/B, PDU A/B, IMU A/B, radios, heater controllers, E-STOP loop relay, spares locker | 1.55 m | front door hinges down into a work shelf |
| MC-L / MC-R, tub walls between front and middle wheels | three motor-controller drawers per side | 1.25 m | side door, reach in from outboard |
| BT-L / BT-R, tub walls between middle and rear wheels | two 105 kWh battery string drawers per side | 1.05 m | drawers roll out on rails onto the dock exchange cart |
| Sensor bar, front of the pod | LiDAR, camera, sun sensor heads | 1.6 m in service position | bar pivots down on a service hinge; heads on kinematic quick-release mounts |
| Radiators, deck edges | louver actuators | 2.0 m | boxes at the lower front corner of each panel |
| Wheel ends | hub motor, gearbox, brake, resolver, declutch | ground | strut lifts the corner (3-legged stance) |
| Cradle | jaw gearmotors (dual winding), load cells | 1.5 m from the deck walkway | module off; jaw drives are dock-gantry work |

Rules for every bay: front access only, no unit behind another unit, captive fasteners, one tool (7/16 in EVA hex
driver), blind-mate connectors on slide rails, field-replaceable units under 25 kg, a built-in-test lamp on each door
(green / amber / red, shown on the 3D model), and a door interlock that opens the bay's solid-state breaker so no
400 V is live with a door open. Field repairs are by crew on a scheduled EVA or the base service robot; heavy units
(battery strings, wheel ends, struts, jaw drives, charge coupler) are changed on the dock service pad by its gantry
and exchange cart. Spares: on-board kit (MC, resolver, harness, load cell, E-STOP harness) plus dock stores.

## 4. Simulator features added

* `failures on|off [realistic|100x|1000x|demo]`: the random-failure switch (also a button in the M10 panel). Off by default.
* `fault list`, `fault <id> [unit]`: inject any catalogued failure (legacy `fault wheel/lidar/...` still work).
* `health`: alarms, redundancy, temperatures, detection statistics. **HEALTH** pill opens the FDIR panel:
  subsystem status, alarms, procedure card, repair progress, spares, thermal, survival and array controls.
* `procedure <id>`: the procedure card. `repair <id>|all|abort`: timed repair, consumes a spare, refuses dock-only
  repairs away from the dock and repairs without a spare.
* `survival on|off|auto`, `array deploy|stow`.
* 3D: louvered radiators (slats follow the thermostat), service doors with status lamps, corner omni antennas,
  beacon (flashes in survival mode or on a critical alarm), hazard cameras, the keep-alive mast and blanket
  (tracks the sun), and a red locator on the failed part when an alarm is selected.
* Telemetry CSV: vault and battery temperature, heater power, keep-alive power, alarm count, strings on line,
  FDIR speed factor. Mission report JSON: FDIR statistics, event history, open alarms, spares.

## 5. Still missing (not modelled or not designed)

* Radiation shielding mass and total-dose life; micrometeoroid protection.
* Direct-to-Earth link: at the site (about 80° S, sun peaking at 9°) Earth sits about 10° above the horizon, so
  terrain can mask it and the design relies on the base surface network and a relay. The sim models the operator
  delay (`comms earth`: 1.3 s one way, manual drive capped at 0.3 m/s) but not line of sight or link budget.
* Star tracker for absolute attitude in permanent shadow (VIPER has one).
* Recovery of a rover that cannot move at all (towing by a second vehicle): tow points are not designed.
* Multi-node thermal (per-wheel gearbox lubricant, sensor heads) and a real dust deposition model.
* The new masses (radiators, blanket, redundant units) are inside the assumed 6,200 kg sprung mass; no BOM yet.
