# Lunar power dock: precedent, mechanism and scale coupling

The sim dock (`src/sim/powerDock.js`, `POWER_DOCK` / `PROTO_PACK` in `src/sim/config.js`) is modelled in **prototype electrical units**
(V, A, Ah, W, Wh) for the 1:16.3 prototype, and coupled to the 420 kWh full-scale pack by state of charge. Figures marked [verify] come from
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

## 2. Scale coupling

The rover pack is treated as the 12.8 V 10 Ah (128 Wh) LiFePO4 4S prototype pack (`rover_bom.js` BATTERY; `ARM_CARGO_BUILD_PLAN.md` BOM #7),
with prototype state of charge equal to `R.power.soc`.

    PROTO_TO_FULL_WH = ROVER.battery.capacityWh / PROTO_PACK.Wh = 420000 / 128 = 3281.25
    R.power.dockW    = chargeW_proto x PROTO_TO_FULL_WH           (the only coupling line)

Charge current is the same C-rate as the old heat-limited full-scale charge: 60 kW / 420 kWh = 0.14 C, so 1.4 A into 10 Ah (about 18.6 W
at 13.3 V, equal to about 61 kW full-scale). Charge times are identical at both scales, and the full-scale pack thermal model in `health.js`
(I^2 R from `P.netW / 400 V`) stays valid. Geometry is drawn at 1:1 of the prototype dock via `k()`, like the rover.

## 3. Sizing

* Array: 2 masts x 0.42 x 0.48 m = 0.4032 m^2 of 30 % triple-junction cells. 1361 W/m^2 x 0.4032 x 0.30 = 165 W AM0, x 0.80 derate (dust 0.93,
  cell temperature 0.90, wiring 0.97, ageing 0.98) x 0.96 MPPT = 126 W at a sun-facing panel.
* Refill: 2.4 kWh x 0.90 usable / 0.97 charge efficiency = 2.23 kWh, divided by about 124 W net = about 18 h of sun (about 22 h with the rover
  charging). That is under 7 % of a 354 h lunar day.
* Night: 0.8 W avionics + vault heater (0.04 W/K x 55 K = 2.2 W loss, minus the 1 W RHU = about 1.2 W) = 2.0 W x 354 h = 708 Wh = 30 % of the bank.
* Night reserve SoC 0.50 = 0.10 BMS floor + 0.30 night load + 0.10 margin. At night the rover may draw the bank down to this level only
  (about 1.2 kWh, about 12 prototype 30 to 90 % charges).
* Bank: 24 V 100 Ah nameplate = 2.4 kWh (user decision; 8S LiFePO4 is 25.6 V nominal, so this is about 6 % conservative).

## 4. Cold charging
LiFePO4 must not be charged below 0 C. The bank: below `chargeMinBankC` the MPPT backs off (`curtailed`) while the vault heater (thermostat
5 C, up to 10 W) warms it. Discharge is allowed down to -20 C. The rover pack: `chargeMinCellC` 0 C cold-hold and the 40 to 50 C taper are
unchanged from the previous dock.

## 5. Known limitation
The compressed `lunar` command (15-minute day) scales time, not energy: the prototype-scale arrays and bank do not follow it, so a compressed
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
