# Thermal environment, scenarios and the thermal brain

Code: `src/sim/thermal.js` (model, scenarios, brain), `src/ui/thermalPanel.js` (HUD panel, key **T**),
hooks in `rover.js` (rpm cap, torque, stator heating), `health.js` (radiator sink, louvers, heater set points,
vault load), `sensors.js` (instrument thermal protection). Test: `tests/thermal.mjs` (`npm test` entry `thermal`).

## Why

On the Moon there is no air: everything the rover makes as heat has to be radiated away, and what it can radiate
to depends on what its radiators see. Sunlit regolith is hot, shadowed regolith and the sky are cold. The
same rover, doing the same work, can overheat on a sun-facing slope and freeze in a crater shadow ten minutes
later. AEGIS-V1 already had thermostats (louvers, heaters, a hub thermal switch). The brain adds the part
thermostats cannot do: see a temperature spike coming and choose the best action across the whole vehicle,
including changing how the rover drives.

## Environment model

| Quantity | Model | Values |
|---|---|---|
| Regolith surface temperature | radiative equilibrium `T = ((1-A) S sin i / (eps sigma))^1/4`, floor at night / shadow, first-order lag (tau 240 s) | 9 deg polar sun ~242 K; 30 deg sun-facing slope ~344 K (71 C); night ~95 K; permanently shadowed crater ~60 K; eclipse ~120 K |
| Radiator sink | half sky (3 K), half regolith: `T_sink^4 = 0.5 T_reg^4` | used by the vault, battery and hub radiators |
| Instrument / controller node | `C dT/dt = P + alpha A S + eps A F sigma T_reg^4 - eps A sigma T^4 + G (T_mount - T) + heater - TEC` | LiDAR, camera, motor controllers |
| Motor winding and hub | winding -> hub conduction -> hub radiator with a paraffin thermal switch (rover.js / health.js) | stator heating and scenario heat add to the winding loss |

## Scenarios (random, seeded, "pretty often")

On by default in the browser: one every ~3.3 min of sim time on average (minimum 1 min apart), each lasting 3-12 min.
`scenarios on|off|often|rare`, `scenario <id>|random|end`, or the panel's **trigger** button.

| id | What happens |
|---|---|
| slope-heat | sun-facing 30 deg slope: regolith heads for ~70 C, every radiator loses most of its sink |
| psr-cold | permanently shadowed crater floor: no sun, regolith ~60 K |
| eclipse | the Earth covers the Sun: regolith cools toward ~120 K |
| dust-fins | dust on the hub and controller radiators: emissivity -60 % / -40 % |
| climb | motors near stall torque: +600 W copper loss per motor (scales with torque squared) |
| glint | sunlight focused on the sensor windows: LiDAR +35 W, camera +8 W |
| heater-stuck | vault heater relay stuck on: +600 W into the avionics vault |
| mc-overload | gate-drive fault: +400 W in the controller bays (scales with the torque allowed) |
| cold-start | wheel hubs and windings cold-soaked to -68 C (windings -72 C) |

## The brain (utility AI)

Every second it predicts each temperature 2 minutes ahead from its filtered rate of change (that is the spike
detector: a part rising 10 K/min at 40 C is treated like a part already at 60 C). It turns the predictions into hot
and cold urgencies (0..1, smoothstep between a warning and a limit), scores every option of each channel with a
response curve, and applies the winner. A small bonus for the current option and an 8 s minimum dwell stop it
from chattering. Every change is logged with its reason.

| Channel | Options | What they do |
|---|---|---|
| Drive | nominal, **REV DOWN 1** (150 rpm, 85 % torque), **REV DOWN 2** (120 rpm, 70 %), **COOL-DOWN STOP**, **REV UP** (120 W stator heating per motor), **REV UP MAX** (350 W) | rev down cuts the current, so copper and controller losses (I^2 R) drop; rev up drives d-axis current through the stator (heat, no torque) to warm the windings and the gear grease; the stop parks the rover to radiate |
| Radiators | thermostat, **PRE-COOL**, **SHED LOAD**, **CONSERVE** | louvers forced open ahead of a heat event; avionics low-power (-25 % vault heat); louvers closed when cold, and heater set points +3 K while the drive is powered (never in survival: the night energy budget) |
| LiDAR, camera | nominal, **HEAT**, **TEC COOL**, **TEC COOL + SUNSHADE** | film heater; thermo-electric cooler pumping heat into the deck; a sunshade cuts direct sun on the housing by 80 % |

Penalties keep it sensible: no rev-down on a loaded slope unless the heat is serious (torque is needed there),
no stator heating when the battery is low.

Instruments are protected OFF if they leave their range by 8 K (back on 3 K inside it). With the brain on this
never happens in any scenario (`tests/thermal.mjs`); with the brain off the LiDAR overheats on a hot slope.

## Instrument ranges [ASSUMED]

| Part | Operating | Brain's band |
|---|---|---|
| LiDAR head | -20..50 C | 0..35 C |
| Camera | -30..60 C | -5..40 C |
| Motor controllers | -40..70 C | -20..55 C |
| Motor windings | -60..125 C | warn above 90 C or below -30 C |
| Wheel hubs (gear grease) | above -60 C | warn below -30 C |
| Avionics vault | below 45 C | pre-cool above 30 C predicted |
