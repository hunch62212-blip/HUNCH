# AEGIS-V1 power budget and recharge estimates

Numbers come from `src/sim/config.js` (battery, charger, keep-alive array) and from the test runs (`power_dock`, `cycle`).
Hand estimates, not new measurements.

| Item | Value | Source |
|---|---|---|
| Battery | 420 kWh, 400 V, 4 strings | `ROVER.battery` |
| Dock charger (inductive, prototype units) | 1.4 A CC (0.14 C) into the 12.8 V 10 Ah pack = about 18.6 W, link 85 % at centre, 125 W link limit; full-scale equivalent about 61 kW | `POWER_DOCK.chargeCC_A`, `linkEff`, `linkMaxW`; `power_dock` test peak 1.4 A |
| Dock solar (prototype units) | 2 masts x 0.2016 m² x 30 % x 0.80 x 0.96 MPPT = 126 W; full-scale equivalent about 415 kW | `POWER_DOCK.panelAreaProto`, `cellEff`, `arrayDerate`, `mpptEff` |
| Dock bank | 24 V 100 Ah = 2.4 kWh (full-scale equivalent about 7.9 MWh), floor 10 %, night reserve 50 % | `POWER_DOCK.bankWh`, `bankFloor`, `nightReserveSoc` |
| Rover array | 12 m², 30 %, 4.8 kW at noon; opens by itself when parked idle in sunlight | `KEEPALIVE`; `tests/logistics.mjs` (4,839 W) |
| One full cycle (3 deliveries) | about 26 kWh per two deliveries in the 2-cargo cycle (17.3 km) | `CHANGES.md` §27 |

Recharge-time estimates
* Dock, 20 % to 90 % (294 kWh full-scale at 61 kW, i.e. 89.6 Wh into the 128 Wh prototype pack at 18.6 W): about 5.3 h with the CC-CV taper. The `power_dock` test shows the same slope (about 0.25 % SoC per simulated minute).
* Rover array alone, same 294 kWh at 4.8 kW: about 61 h of full sun, i.e. under one lunar day (about 354 h of sunlight).
* One two-delivery cycle (26 kWh): about 5.4 h on the array, or 27 min at the dock.
* The array is a keep-alive and top-up source, not the primary charger. The dock is.

Scale coupling (dock)
* The dock is modelled in prototype units (12.8 V 10 Ah LiFePO4 pack, 128 Wh) and coupled to the 420 kWh sim pack by state of charge:
  `PROTO_TO_FULL_WH = 420000 / 128 = 3281.25`, `R.power.dockW = chargeW_proto x 3281.25`. The C-rate is the same (0.14 C), so charge times match
  at both scales. See `docs/LUNAR_CHARGING.md`.
* Dock bank refill from the 10 % floor: 2.4 kWh x 0.9 / 0.97 / about 124 W = about 18 h of sun (a lunar day is about 354 h). Night load 2.0 W x 354 h = 708 Wh = 30 % of the bank.
