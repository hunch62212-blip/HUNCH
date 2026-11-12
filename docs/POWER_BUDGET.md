# AEGIS-V1 power budget and recharge estimates

Numbers come from `src/sim/config.js` (battery, charger, keep-alive array) and from the test runs (`power_dock`, `cycle`).
Hand estimates, not new measurements.

| Item | Value | Source |
|---|---|---|
| Battery | 420 kWh, 400 V, 4 strings | `ROVER.battery` |
| Dock charger | 60 kW limit, 95 % efficient | `POWER_DOCK.chargeMaxW`, `chargerEff`; `power_dock` test peak 60 kW |
| Dock solar field | 4 arrays of 120 m², 30 % efficient (165 kW seen in the test) | `POWER_DOCK`; `power_dock` log |
| Rover array | 12 m², 30 %, 4.8 kW at noon; opens by itself when parked idle in sunlight | `KEEPALIVE`; `tests/logistics.mjs` (4,839 W) |
| One full cycle (3 deliveries) | about 26 kWh per two deliveries in the 2-cargo cycle (17.3 km) | `CHANGES.md` §27 |

Recharge-time estimates
* Dock, 20 % to 90 % (294 kWh at 60 kW x 0.95): about 5.2 h. The `power_dock` test shows the same slope (about 0.25 % SoC per simulated minute).
* Rover array alone, same 294 kWh at 4.8 kW: about 61 h of full sun, i.e. under one lunar day (about 354 h of sunlight).
* One two-delivery cycle (26 kWh): about 5.4 h on the array, or 27 min at the dock.
* The array is a keep-alive and top-up source, not the primary charger. The dock is.
