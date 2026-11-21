# AEGIS-V1 power budget and recharge estimates

Numbers come from `src/sim/config.js` (battery, charger, keep-alive array) and from the test runs (`power_dock`, `cycle`).
Hand estimates, not new measurements.

| Item | Value | Source |
|---|---|---|
| Battery | 150 kWh, 400 V, 4 strings (user decision, Nov 2026; was 420 kWh) | `ROVER.battery` |
| Dock charger (inductive, prototype units) | 1.4 A CC (0.14 C) into the 12.8 V 10 Ah pack = about 18.6 W, link 85 % at centre, 125 W link limit; full-scale equivalent about 21.8 kW (150 kWh / 128 Wh = 1172 x 18.6 W), capped at the 20 kW charger rating (`ROVER.battery.dockMaxW`, applied in `powerDock.js` and `rover.js`) | `POWER_DOCK.chargeCC_A`, `linkEff`, `linkMaxW`; `power_dock` test peak 1.4 A |
| Dock solar (prototype units) | 2 masts x 0.2016 m² x 30 % x 0.80 x 0.96 MPPT = 126 W; full-scale equivalent about 148 kW | `POWER_DOCK.panelAreaProto`, `cellEff`, `arrayDerate`, `mpptEff` |
| Dock bank | 24 V 100 Ah = 2.4 kWh (full-scale equivalent about 2.8 MWh), floor 10 %, night reserve 50 % | `POWER_DOCK.bankWh`, `bankFloor`, `nightReserveSoc` |
| Rover array | 12 m², 30 %, 4.8 kW at noon; opens by itself when parked idle in sunlight | `KEEPALIVE`; `tests/logistics.mjs` (4,839 W) |
| Arm pair, cradled standby (arms stowed and settled, servos unpowered) | 3 W | `ARM_ASSUMED.standbyW` (`src/sim/arm.js`) |
| Arm pair, active (hotel + servos) | 120 W hotel plus servos; about 465 W while moving, measured in `tests/arm.mjs` | `ARM.hotelW` (`src/sim/config.js`), `rover.js` `P.armW` |
| Arm pair, rover in survival | 0 W | `arm.step` with `powered === false` |
| Drive motors | 4 driven wheels (FL, FR, RL, RR); the two middle wheels are unpowered idlers, so no motor power or copper heat from them | `DRIVE`, `rover.js` `w.driven` |
| One full cycle (3 deliveries) | about 26 kWh per two deliveries in the 2-cargo cycle (17.3 km) | `CHANGES.md` §27 |

Recharge-time estimates
* Dock, 20 % to 90 % (105 kWh full-scale at 20 kW, i.e. 89.6 Wh into the 128 Wh prototype pack at 18.6 W): about 5.3 h with the CC-CV taper. The `power_dock` test shows the same slope (about 0.25 % SoC per simulated minute).
* Rover array alone, same 105 kWh at 4.8 kW: about 22 h of full sun, i.e. under one lunar day (about 354 h of sunlight).
* One two-delivery cycle (26 kWh): about 5.4 h on the array, or about 80 min at the 20 kW dock.
* The array is a keep-alive and top-up source, not the primary charger. The dock is.

Scale coupling (dock)
* The dock is modelled in prototype units (12.8 V 10 Ah LiFePO4 pack, 128 Wh) and coupled to the 150 kWh sim pack by state of charge:
  `PROTO_TO_FULL_WH = 150000 / 128 = 1171.875`, `R.power.dockW = min(chargeW_proto x 1171.875, 20 kW)`. The C-rate is the same (0.14 C), so charge times match
  at both scales. See `docs/LUNAR_CHARGING.md`.
* Dock bank refill from the 10 % floor: 2.4 kWh x 0.9 / 0.97 / about 124 W = about 18 h of sun (a lunar day is about 354 h). Night load 2.0 W x 354 h = 708 Wh = 30 % of the bank.

Night survival at 150 kWh (open decision)
* Measured with the `tests/fdir.mjs` section E procedure: survival load about 154 W, a 354 h night costs about 55 kWh = 36 % of the 150 kWh pack (45 % with hub heaters). The old 420 kWh pack made this under 25 %; `tests/fdir.mjs` E still asserts 25 % and fails. The mechanism heaters (`docs/changes/B.md`) add about 15 to 40 W of night average on top.
* Options: a larger pack, RHUs, parking at the dock for night power, or raising the limit. Pending user decision; the test was deliberately not changed.
