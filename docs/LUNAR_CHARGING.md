# Lunar power dock: precedent, mechanism and scale coupling

The sim dock (`src/sim/powerDock.js`, `POWER_DOCK` / `PACK` in `src/sim/config.js`) is modelled at **full scale in real units**
(400 V, 150 kWh rover pack, 250 kWh bank, 8.4 kW array; review item 6). It used to be modelled in prototype units and multiplied by 1172, which implied
a 148 kW array and a 2.8 MWh bank; the bench dock now lives in `POWER_DOCK_PROTO` (not used by the sim). Figures marked [verify] come from
memory or secondary sources and were not re-checked against a primary document.

## 1. Lunar precedent

| System | Approach | What the dock takes from it |
|---|---|---|
| Apollo LRV | two 36 V silver-zinc primary batteries, not rechargeable [verify capacity] | why a sustained fleet needs a recharge point |
| Lunokhod 1/2 | solar panel in a hinged lid, open by day and closed at night; Po-210 heat source [verify] | an isotope heat source cuts the electric night heating (`vault.rhuW`) |
| Yutu / Yutu-2 | solar plus radioisotope heater units, hibernates through the night, wakes at sunrise [verify] | hibernation floor, night reserve, bank cold-hold at sunrise |
| VIPER | solar at the south pole, low sun, battery for shadow, illumination planning [verify] | vertical sun-tracking panels, refill sized well inside the lit period |
| NASA VSAT (Astrobotic LAMPS / VSAT-XL, Honeybee, Lockheed) | about 10 kW masts, arrays 10 m or more up, mast up to 20 m, autonomous deploy; VSAT-XL over 30 m and 50 kW | vertical azimuth-tracking mast panels |
| Astrobotic LunaGrid-Lite | 1 kW over about 500 m of cable (one release says 1 km) from lander arrays to a CubeRover | a central dock feeding rovers |
| Astrobotic + WiBotic UFPC (NASA Tipping Point, about $5.7M) | wireless inductive charging, no exposed contacts, no degradation under 4 cm of regolith, dirty-TVAC tested, flight model qualified May 2024, 125 W commercial model | the charging mechanism: inductive pad, rover kneels to close the gap, 125 W link limit |
| NASA dust-tolerant connector work | abrasion, electrostatic adhesion and cold welding defeat exposed contacts | why the shuttered contact pucks were removed |

### Mechanism
A flush, sealed transmitter coil sits in the sintered-regolith pad under a glass-ceramic cover. The receiver coil is in the rover belly. The
rover centres on the pad (existing 3 m / 8 degree tolerance) and kneels to close the coil gap (existing kneel check). A pilot / handshake
signal is required, so the existing `coupler` fault (no pilot signal) still stops charging. Link efficiency is 0.85 at the centre and falls
quadratically with lateral offset (25 % loss at the 3 m tolerance). Fallback if the dock is unusable: the rover keep-alive array and survival
mode (`KEEPALIVE`, `health.js`).

### Why not contacts, a plug or a pack swap
`public/team/js/dock_station.js` models a pack swap and the previous sim dock had shuttered blind-mate contacts. Both need a moving, exposed
mechanical interface. Lunar regolith is abrasive and electrostatically sticky, and bare metal contacts can cold-weld in vacuum, so a sealed
inductive link is the lowest-risk interface for a sustained dock. The cost is efficiency (about 85 %) and a power limit set by the coil
(125 W commercial class, which this dock respects).

## 2. Scale
The dock is sized for the full-scale job (`docs/SCALING.md`); electrical quantities do not scale by the Froude ratios. The rover pack is the 125S
LiFePO4, 400 V, 375 Ah (150 kWh) pack (`ROVER.battery`, `PACK`), charged CC-CV at 0.14 C = 52.5 A (about 21.7 kW) and capped at the 20 kW charger:

    R.power.dockW = min(chargeW, ROVER.battery.dockMaxW)

The bench dock (12.8 V 10 Ah pack, 24 V 100 Ah bank, 126 W array, 125 W link) is the prototype's own spec (`POWER_DOCK_PROTO`, `PROTO_PACK`). Geometry is
drawn at 1:1 of the prototype dock via `k()`, like the rover, so the masts are 11 m tall with 3.4 x 3.9 m wings (13.4 m^2 each).

## 3. Sizing

* Array: 2 masts x 13.4 m^2 = 26.8 m^2 of 30 % triple-junction cells. 1361 W/m^2 x 26.8 x 0.30 = 10.9 kW AM0, x 0.80 derate (dust 0.93,
  cell temperature 0.90, wiring 0.97, ageing 0.98) x 0.96 MPPT = 8.4 kW at a sun-facing panel.
* Refill: 250 kWh x 0.90 usable / 0.97 charge efficiency = 232 kWh, divided by about 8.3 kW net = about 28 h of sun (about 31 h with the rover charging). That is
  8 % of a 354 h lunar day. A 20 kW rover charge is 2.4x the array: the bank carries the difference.
* Night: 120 W avionics + vault heater (0.9 W/K x 55 K = 50 W, electric; `rhuW` 0) = 170 W x 354 h / 0.97 = 62 kWh = 25 % of the bank.
* Night reserve SoC 0.40 = 0.10 BMS floor + 0.25 night load + 0.05 margin. At night the rover may draw the bank down to this level only
  (about 124 kWh through the 85 % link, 1.5x the rover's steady-state 354 h survival load of 82 kWh, docs/THERMAL_BUDGET.md).
* Bank: 400 V 625 Ah = 250 kWh. [ASSUMED] (no lunar-qualified bank of this size exists to cite; about 1.9 t of LFP cells).

## 4. Cold charging
LiFePO4 must not be charged below 0 C. The bank: below `chargeMinBankC` the MPPT backs off (`curtailed`) while the vault heater (thermostat
5 C, up to 400 W) warms it. Discharge is allowed down to -20 C. The rover pack: `chargeMinCellC` 0 C cold-hold and the 40 to 50 C taper are
unchanged from the previous dock.

## 5. Known limitation
The compressed `lunar` command (15-minute day) scales time, not energy: the arrays and bank are in real units and do not follow it, so a compressed
night drains the bank at the real hourly rate over a much shorter span. Use `night` / `day` or the real-time soak test for dock energy.

## Sources
* astrobotic.com/astrobotic-wins-5-7-million-nasa-tipping-point-contract-for-wireless-charging-on-the-moon/
* astrobotic.com/astrobotic-lunar-wireless-charger-system-qualified-for-flight/
* science.nasa.gov/wp-content/uploads/2023/11/astrobotic-wireless-charger.pdf
* nasa.gov/news-release/three-companies-to-help-nasa-advance-solar-array-technology-for-moon
* ntrs.nasa.gov/api/citations/20230012849/downloads/GCD%20Annual%20Review%20VSAT%20September%202023%20Final.pdf
* astrobotic.com/astrobotic-awarded-lunar-power-study-with-vsat-xl/
* astrobotic.com/astrobotic-wins-34-6m-for-power-demo-mission-on-the-moon/
* astrobotic.com/lunagrid-lite-completes-critical-design-review-flight-model-underway/
