# AEGIS-V1 charging interface: design decision and CAD interface spec

Status: decided (charging agent, Nov 2026). Code: `src/sim/chargingParams.js` (all numbers below), `src/sim/powerDock.js`
(physics, power flow, thermal, night park), `src/render/powerDock.js` (visuals), `tests/power_dock.mjs` (checks).
Fixed decisions this design does not revisit: K = 8.16, 150 kWh pack, 20 kW dock cap (`ROVER.battery.dockMaxW`), ~5 t rover,
4WD outer wheels, 3 m module. Values marked [ASSUMED] are engineering estimates without a primary source.

## 1. Decision

**Primary: a segmented inductive pad (ground assembly, GA) in a sintered-regolith slab and a 0.9 m receiver (vehicle assembly,
VA) recessed in the rover belly. The rover parks on the pad, kneels to close the air gap to ~75 mm and charges at up to 20 kW.
The link also carries the rover's bus load, and the rover can stay parked through the lunar night with the dock bank carrying its
survival load ("night park").**

Fallbacks:
1. Night: the rover's own keep-alive array plus survival mode (already in the sim) when it cannot reach the dock.
2. Night, later: a fission surface power (FSP) or larger bank tie-in on the dock bus. The pad interface does not change; only
   the source behind it does.

Why inductive:
* No exposed contacts. Regolith is abrasive, electrostatically charged and gets into everything; contacts in vacuum can cold
  weld. Both the GA and the VA are sealed behind RF-transparent covers, so dust on the pad does not stop the link.
* Flight precedent at small scale: the Astrobotic / WiBotic lunar wireless charger passed flight qualification in 2024, including
  operation in dirty thermal vacuum under about 4 cm of regolith simulant and from about -180 C to +105 C.
* Large position tolerance (here +-0.75 m along and +-0.35 m across at full power) with no moving parts. The autonomy already
  parks within 0.21 m / 0.28 m / 2.5 deg (measured, section 4), so no new mechanism or new docking phase is needed.
* The power level is already proven on Earth: SAE J2954 covers 3.7 to 11 kW at 100 to 250 mm gaps with over 90 % efficiency, and
  ORNL demonstrated 50 kW at 95 %.
* Night support comes for free: a coupled rover can draw its 154 W survival load from the dock all night. That removes the
  36 %-of-pack night drain (docs/POWER_BUDGET.md, open decision) whenever the rover overnights at the dock.

Rejected:
* **Conductive dust-tolerant blind-mate connector**: best efficiency (~99 %) and the cheapest night tie-in, but the contacts are
  exposed to dust and cold welding. It needs +-5 to 10 mm alignment (a funnel and an actuated probe on a 5 t rover), and is
  TRL 4 to 5 for lunar dust cycling (NASA KSC, Honeybee DTC). Kept as a future option for MW-class FSP tie-ins, not for daily
  rover charging.
* **Laser / microwave power beaming**: 10 to 20 % end to end (about 50 % at a GaAs receiver after laser and optics losses). That
  is roughly 100 to 200 kW at the source for a 20 kW charge, and it needs line of sight. Good for remote keep-alive far from the
  dock, not for charging.
* **Battery swap**: a 150 kWh pack is roughly 750 to 1000 kg. It needs a heavy manipulator, exposed high-current connectors
  anyway, and thermal conditioning of the spare packs. Too complex for the gain.
* **RTG / radioisotope at the dock**: an MMRTG gives about 110 We, and Pu-238 is scarce. It is only useful as night heat (RHUs
  are already in the dock vault), not as a charger.
* **Solar + battery bank at the dock**: not an interface but the energy source. Kept as the dock's source (existing VSAT-style
  masts and bank).
* **Fission surface power**: also a source, not an interface. 40 kWe, under 6 t, around 2030 or later. This is fallback 2 above.

## 2. Trade table

Scores 1 (poor) to 5 (best). Scored for this rover: 20 kW cap, 150 kWh pack, south pole, autonomy docking speed < 0.08 m/s.

| Option | Efficiency | Dust tolerance | Alignment tolerance | Thermal | Complexity (5 = simple) | TRL (lunar, at 20 kW) | Night support | Total |
|---|---|---|---|---|---|---|---|---|
| Inductive segmented pad (chosen) | 4 (0.92) | 5 | 4 (+-0.75 / +-0.35 m) | 3 (1.7 kW loss at 20 kW) | 4 | 4 (6 at 125 W) | 5 | **29** |
| Conductive blind-mate connector | 5 (0.99) | 2 | 2 (+-5-10 mm) | 4 | 2 (funnel + actuator) | 4 | 5 | 24 |
| Battery swap | 5 | 2 | 1 | 2 | 1 | 3 | 3 | 17 |
| Laser beaming | 1 (0.1-0.2) | 3 | 3 | 1 | 2 | 4 (1.8 kW demo planned) | 3 | 17 |
| Microwave beaming | 1 | 4 | 3 | 2 | 2 | 3 | 3 | 18 |
| RTG at dock (heat / 110 We) | n/a | 5 | 5 | 4 | 3 | 7 (MMRTG) | 3 (heat only) | n/a |

## 3. Environment and constraints

| Constraint | How the design meets it |
|---|---|
| Vacuum, regolith dust | Sealed GA and VA, no contacts, no shutters. The GA cover is flush with the slab, so dust is swept, not trapped. Dust on the cover only adds gap (the link works through 4 cm of simulant, Astrobotic). |
| -173 to +127 C | Coils and ferrite run unheated (they warm from their own loss when charging). The GA inverter sits in the heated, bermed battery vault, and the VA rectifier sits on the battery cold plate. |
| South-pole sun ~9 deg elevation, 360 deg azimuth | The pad is shadowed by the parked rover (no sky view), so pad heat goes into the slab. Masts track azimuth (existing). |
| 354 h night | Night park: the dock carries the rover's bus load (cap 600 W full-scale), down to bank SoC 12 %. Charging still stops at the 50 % night reserve. |
| 150 kWh pack, 20 kW cap | Link limit = 20 kW (`CHARGING.maxW = ROVER.battery.dockMaxW`), shared by bus load first, then CC-CV charge. 0.14 C, same charge time as before. |
| Docking: speed < 0.08 m/s, couplerTol | Capture box replaces the 3.0 m radius `couplerTol`. Receiver within +-1.00 m along / +-0.60 m across, yaw < 8 deg, speed < 0.08 m/s, air gap 0 to 0.22 m (i.e. kneeled). |
| Mass, survivability | VA ~55 kg [ASSUMED], about 1 % of the rover. No moving parts on the rover or the pad. A single failed GA module only removes the coils under it (the receiver covers 3 of 6). |

## 4. Measured parking accuracy (existing autonomy, unchanged)

Charge mission from three starts around the dock entry (scratch probe, receiver centre in the pad frame at the couple phase):

| Start offset from entry | Along | Across | Yaw error |
|---|---|---|---|
| (+8, +8) m | -0.214 m | +0.244 m | -2.47 deg |
| (-6, +10) m | -0.139 m | +0.018 m | +1.24 deg |
| (+12, -4) m | -0.125 m | +0.281 m | +0.14 deg |

All three are inside the full-power box (0.75 / 0.35 m). The kneeled body sits 0.364 m below its ride-0 position (ride -0.332 m
plus tyre and suspension sag). The receiver face height was set from this, so the kneeled gap is about 77 mm against the
75 mm design value.

## 5. Power flow, efficiency, thermal, night budget (as implemented)

* **Coupling** needs all of: receiver in the capture box, still (< 0.08 m/s), air gap in [0, 0.22] m, pilot handshake present
  (the health coupler fault removes it), coils below 110 C, and no e-stop. At ride 0 the gap is ~0.41 m, so there is no link.
  Kneeling makes it.
* **Coupling fraction**: 1 inside (0.75, 0.35) m, falling linearly to 0 at (1.00, 0.60) m.
* **Efficiency** (DC dock bus to DC rover bus):
  eta = 0.92 x (1 - 0.05 u^2) x (1 - 0.25 x max(0, gap - 0.075)),
  where u is the larger of the along and across offsets as a fraction of the capture limit.
  0.92 is a lunar derate of the ORNL 95 % / J2954 > 90 % figures. The edge and gap losses are [ASSUMED].
* **Priority**: the rover bus load is supplied first (pass-through), then the CC-CV charge (1.4 A to 80 %, then taper). The total
  is capped at 20 kW full-scale. The pack gets the whole charge current, so the rover's idle load no longer eats into the charge.
* **Thermal**: loss at 20 kW is about 1.74 kW.
  * The receiver takes 45 %. It is bolted to the belly with a heat-pipe spreader into the battery coolant loop (G = 40 W/K), so
    it runs about 20 K above the pack (time constant ~21 min).
  * The GA takes 55 % into the slab (G = 30 W/K to a 0 C day / -60 C night sink).
  * Both derate linearly from 90 C to 0 at 110 C.
  * The receiver loss heat (`PowerDock.rxHeatW`) is exposed but not yet credited to the pack thermal node in health.js (open item).
* **Night park budget** (prototype bank units, 2.4 kWh bank):

  | Load | Power |
  |---|---|
  | Dock night load | 2.0 W |
  | Pad pilot, duty-cycled | 0.1 W (20 % of 0.5 W [ASSUMED]) |
  | Rover survival load, 154 W full-scale / 1172 / 0.92 | 0.14 W |
  | **Total** | **2.24 W** |

  2.24 W x 354 h = 794 Wh at the bus, or about 819 Wh from the cells = 34 % of the bank. Starting from the 50 % night reserve,
  the bank ends at about 16 %, above the 12 % keep-alive floor and the 10 % BMS floor. `PowerDock.nightParkHours(154)` is about
  394 h. The rover pack does not discharge at all over the night.

## 6. Physical interface spec (for CAD; Fusion agent)

Frames:
* **Rover body frame**: +X forward, +Y up, +Z right. Origin on the ground plane at ride 0, X = 0 at the middle wheel axle.
* **Pad frame**: +along = the parked rover heading (nose toward the habitat building line), +across = rover right. Origin = pad
  centre on the slab datum (the terrain height at the pad centre).

Full scale is in metres. Prototype mm = full-scale m / 8.16 x 1000.

### 6.1 Receiver (VA) on the rover, `CHARGING.rx`

| Item | Full scale | Prototype (1:8.16) |
|---|---|---|
| Coil centre (body frame) | X = 0.000, Z = 0.000 (belly centreline over the middle axle) | X 0, Z 0 |
| Receiver face height at ride 0 | Y = 0.498 m | 61 mm |
| Housing (X x Z x Y) | 0.898 x 0.898 x 0.082 m | 110 x 110 x 10 mm |
| Housing top / recess | top at Y = 0.580; recessed 0.050 m into the belly tub (tub bottom Y = 0.530), face 0.032 m proud of the tub | top 71 mm, recess 6 mm, proud 4 mm |
| Lateral clearance to the wheels | housing edge \|Z\| = 0.449 m; wheel inner faces at \|Z\| = 1.612 m (1.16 m clear) | 55 / 198 mm |
| Face cover (dust) | 8 mm alumina-filled PEEK radome, flush, no seams on the face; optional EDS (electrodynamic dust shield) traces [ASSUMED] | 1 mm cover plate |
| Behind the coil | ferrite tile layer + 3 mm Al shield plate, then the rectifier on the battery cold plate [ASSUMED] | n/a |
| Thermal path | Al backplate with 2 heat pipes into the battery coolant loop, G = 40 W/K; peak loss into the VA ~0.78 kW at 20 kW | n/a |
| Mass | ~55 kg [ASSUMED] | n/a |
| Kneeled position | ride -0.326 m command (measured body -0.364 m), so the face is ~0.134 m above the ground and ~77 mm above the pad cover | 16 / 9 mm |

### 6.2 Ground assembly (GA) and slab at the dock, `CHARGING.ga`

| Item | Full scale | Prototype (1:8.16) |
|---|---|---|
| Modules | 3 along x 2 across, each 0.80 x 0.80 m, max 7 kW each [ASSUMED] | 98 x 98 mm |
| Active area | 2.40 m along x 1.60 m across, centred on the pad origin | 294 x 196 mm |
| Module frame (outline) | 2.52 x 1.72 m | 309 x 211 mm |
| Cover top above the slab datum | 0.057 m (flush with the frame) | 7 mm |
| Cover | 15 mm glass-ceramic (alumina) cover, RF transparent, abrasion resistant [ASSUMED] | 2 mm |
| Slab | sintered regolith, 7.0 m along x 5.0 m across x 0.15 m, top flush with the slab datum [ASSUMED] | 858 x 613 x 18 mm |
| Inverter / electronics | in the heated battery vault (13 m from the pad centre toward the building); cable trench to the GA | n/a |

### 6.3 Alignment guides and capture envelope, `CHARGING.tol`

* **No mechanical funnel or curbs.** The wheels are shape-cast against structure colliders and would climb a curb rather than be
  guided by it. A funnel strong enough to steer a 5 t rover is also heavy and a dust trap. Alignment comes from the
  autonomy (fiducial and beacon localisation), then the pilot-coil read-out (`PowerDock.padAlignment()`: along, across, yaw,
  gap), then the existing pilot square-up.
* **Capture box**, for the receiver centre against the pad centre:
  * full power at |along| <= 0.75 m and |across| <= 0.35 m
  * falling to zero power at 1.00 m and 0.60 m
  * yaw within +-8 deg (mod 180 deg: the rover may park either way round)
  * speed < 0.08 m/s
  * air gap 0 to 0.22 m (75 mm nominal)
  * painted as a yellow 2.0 x 1.2 m outline on the slab
* **Wheel lanes**: painted lines at |across| = 1.612 m and 2.224 m (inner and outer faces of the wheels: track ±1.918 m,
  wheel width 0.612 m), the full slab length.
* **Fiducial board**: an 0.8 x 0.8 m 8 x 8 square binary marker on the charger pedestal face toward the pad. The pedestal is at
  pad frame (along +2.0 m, across -7.5 m), i.e. 7.5 m to the parked rover's left and 2 m ahead of the pad centre. Board centre 1.25 m above the ground.
  * Pedestal: 1.0 x 1.8 x 1.0 m, with a collider in the sim.

### 6.4 Things the CAD must keep clear

* Belly region X in [-0.50, +0.50] m and Z in [-0.50, +0.50] m, from Y = 0.498 up to 0.580: receiver only (no skid plate,
  harness or jack points).
* The bottom of the belly must stay at Y >= 0.530 elsewhere. Kneeled, the belly is ~0.166 m above the ground and the GA top is
  0.057 m, so 0.109 m clears the GA.
* No metal within 0.15 m of the receiver edge in the belly plane (eddy-current heating) [ASSUMED].

## 7. Open items

* health.js does not yet add `PowerDock.rxHeatW` (receiver loss heat, up to ~0.78 kW at 20 kW) to the pack thermal node. The
  real heating would be a few K more on long charges, and useful heat at night.
* The square-up step in autonomy (`autonomy.js` chargeAtDock) corrects heading and the along-track offset only. A cross-track
  error over 0.60 m would make the couple fail (it reports "coupler did not engage"). The measured worst case is 0.28 m.
* At 20 kW lunar, the TRL is about 4: the flight-qualified unit is 125 W and the 20 kW coils are terrestrial (J2954 / ORNL).
  Coil mass and heat rejection at 20 kW in vacuum need a test article.
* Night park assumes the user accepts "overnight at the dock" as the night-survival answer (docs/POWER_BUDGET.md open
  decision). The FSP tie-in is the long-term source.

## Sources

Inductive:
* Astrobotic lunar wireless charger qualified for flight (2024): https://www.astrobotic.com/astrobotic-lunar-wireless-charger-system-qualified-for-flight/
* https://www.allaboutcircuits.com/news/the-first-wireless-charging-system-for-the-moon-cleared-for-flight/
* Astrobotic / WiBotic tests: https://www.therobotreport.com/astrobotic-wibotic-test-lunar-wireless-charging-system/
* NASA lunar wireless power (NTRS): https://ntrs.nasa.gov/api/citations/20240011234/downloads/20240011234.pdf
* SAE J2954 classes and tolerances: https://chargedevs.com/newswire/sae-publishes-wireless-charging-standard/
* https://saemobilus.sae.org/content/2019-01-0868
* ORNL 50 kW misalignment-tolerant 3-phase WPT: https://www.ornl.gov/publication/misalignment-tolerant-three-phase-wireless-fast-charging-system-electric-vehicles
* DOE high-power inductive charging: https://www.energy.gov/eere/vehicles/articles/high-power-inductive-charging-system-development-and-integration-mobility-1

Conductive / dust-tolerant connectors:
* NASA KSC dust-tolerant connectors: https://ntrs.nasa.gov/citations/20100005258
* KSC Dust Tolerant Automated Umbilical: https://ntrs.nasa.gov/api/citations/20150000177/downloads/20150000177.pdf
* ESA lunar plug/socket study: https://nebula.esa.int/sites/default/files/neb_study/2686/C4000134106FP.pdf
* Honeybee DTC (LSIC 2024): https://lsic.jhuapl.edu/uploadedDocs/focus-files/2532-2024-06-26%20LSIC%20Surface%20Power%20Honeybee%20Robotics%20DTC.pdf
* https://www.techbriefs.com/component/content/article/13674-dust-mitigation-connectors-for-electrical-cables-and-fluid-lines

Beaming:
* NASA lunar laser beaming (AIAA): https://ntrs.nasa.gov/api/citations/20205005063/downloads/Lunar-laser_beaming-AIAA_Paper%20Complete.pdf
* Volta Space LightGrid: https://pv-magazine-usa.com/2026/04/13/volta-space-technologies-leverages-government-partnerships-and-funding-to-develop-laser-enabled-lunar-pv-power-network

Sources (power at the dock):
* NASA Fission Surface Power: https://www.nasa.gov/exploration-systems-development-mission-directorate/fission-surface-power
* FSP IAC 2024: https://ntrs.nasa.gov/api/citations/20240011694/downloads/IAC%202024%20Manuscript_FSP%20FINAL.pdf
* VSAT and LunaGrid: see docs/LUNAR_CHARGING.md "Sources"
