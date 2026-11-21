# AEGIS-V1 power budget and recharge estimates

Numbers come from `src/sim/config.js` (battery, charger, keep-alive array) and from the test runs (`power_dock`, `cycle`).
Hand estimates, not new measurements.

| Item | Value | Source |
|---|---|---|
| Battery | 150 kWh, 400 V, 4 strings (user decision, Nov 2026; was 420 kWh) | `ROVER.battery` |
| Dock charger (inductive, full scale) | 52.5 A CC (0.14 C of 375 Ah) at about 412 V = 21.7 kW, link 85 % at centre, capped at the 20 kW charger rating (`ROVER.battery.dockMaxW`, applied in `powerDock.js` and `rover.js`); 23.5 kW into the link. [ASSUMED]: no 20 kW lunar inductive link exists (the lunar-qualified reference is the 125 W Astrobotic / WiBotic unit, kept as the bench spec in `POWER_DOCK_PROTO`) | `POWER_DOCK.chargeCC_A`, `linkEff`, `linkMaxW`; `docs/SCALING.md` |
| Dock solar (full scale) | 2 masts x 13.4 m² (the drawn 1:K masts) x 30 % x 0.80 x 0.96 MPPT x 1361 W/m² = about 8.4 kW at a sun-facing panel. (The earlier prototype-units model multiplied 126 W by 1172 and implied 148 kW; the drawn masts never gave that.) | `POWER_DOCK.panelArea`, `cellEff`, `arrayDerate`, `mpptEff` |
| Dock bank | 400 V 625 Ah = 250 kWh (about 1.9 t of LFP cells), floor 10 %, night reserve 40 %. Dock's own night load 170 W (avionics 120 W + vault heater 50 W) x 354 h / 0.97 = 62 kWh = 25 % of the bank; the rover may draw 124 kWh through the link above the reserve against its steady-state 354 h survival load of 82 kWh (`docs/THERMAL_BUDGET.md`; the 154 W / 55 kWh in `docs/changes/A.md` was a transient). [ASSUMED] | `POWER_DOCK.bankWh`, `bankFloor`, `nightReserveSoc`, `vault` |
| Rover array | 12 m², 30 %, 4.8 kW at noon; opens by itself when parked idle in sunlight | `KEEPALIVE`; `tests/logistics.mjs` (4,839 W) |
| Arm pair, cradled standby (arms stowed and settled, servos unpowered) | 3 W | `ARM_ASSUMED.standbyW` (`src/sim/arm.js`) |
| Arm pair, active (hotel + servos) | 120 W hotel plus servos; about 465 W while moving, measured in `tests/arm.mjs` | `ARM.hotelW` (`src/sim/config.js`), `rover.js` `P.armW` |
| Arm pair, rover in survival | 0 W | `arm.step` with `powered === false` |
| Drive motors | 4 driven wheels (FL, FR, RL, RR); the two middle wheels are unpowered idlers, so no motor power or copper heat from them | `DRIVE`, `rover.js` `w.driven` |
| One full cycle (3 deliveries) | about 26 kWh per two deliveries in the 2-cargo cycle (17.3 km) | `CHANGES.md` §27 |

Recharge-time estimates
* Dock, 20 % to 90 % (105 kWh at the 20 kW cap): about 5.3 h with the CC-CV taper. The `power_dock` test shows the same slope (about 0.25 % SoC per simulated minute).
* Rover array alone, same 105 kWh at 4.8 kW: about 22 h of full sun, i.e. under one lunar day (about 354 h of sunlight).
* One two-delivery cycle (26 kWh): about 5.4 h on the array, or about 80 min at the 20 kW dock.
* The array is a keep-alive and top-up source, not the primary charger. The dock is.

Dock sizing (full scale; the prototype-units coupling and `PROTO_TO_FULL_WH` were removed in the review pass, item 6)
* The dock is modelled in real units (125S LFP, 400 V, 150 kWh rover pack; 250 kWh bank; 8.4 kW array) and `R.power.dockW = chargeW` (capped at 20 kW). The bench dock (24 V 100 Ah bank, 126 W array, 128 Wh pack) is its own spec (`POWER_DOCK_PROTO`, `docs/PROTOTYPE_POWER_BUDGET.md`) and is not scaled up. See `docs/SCALING.md`.
* A 20 kW charge is more than twice the array (8.4 kW): the bank covers the difference. After one 30 to 90 % charge (90 kWh into the pack, about 109 kWh from the bank and array over about 4.7 h) the bank is down about 70 kWh and needs about 28 h of full sun from its floor, 8 % of a 354 h lunar day. The old "solar covers the charge" result came from the x1172 multiplication.
* Dock bank refill from the 10 % floor: 250 kWh x 0.9 / 0.97 / about 8.3 kW = about 28 h of sun.
* Night: the rover side is in `docs/THERMAL_BUDGET.md`; the dock side is 170 W x 354 h = 62 kWh (25 % of the bank).

Night survival at 150 kWh (open decision)
* Measured with the `tests/fdir.mjs` section E procedure: survival load about 154 W, a 354 h night costs about 55 kWh = 36 % of the 150 kWh pack (45 % with hub heaters). The old 420 kWh pack made this under 25 %; `tests/fdir.mjs` E still asserts 25 % and fails. The mechanism heaters (`docs/changes/B.md`) add about 15 to 40 W of night average on top.
* Options: a larger pack, RHUs, parking at the dock for night power, or raising the limit. Pending user decision; the test was deliberately not changed.
