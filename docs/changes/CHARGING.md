# Charging agent: segmented inductive pad, pass-through, night park

Design and sources: `docs/CHARGING_DESIGN.md`.

## New
- `src/sim/chargingParams.js`: the charging interface constants (`CHARGING`): receiver geometry in the body frame, ground
  assembly modules and slab, capture tolerances, air gap, efficiency model, link thermal nodes, night park. `padFrame()`
  helper. config.js is not changed.

## Behaviour changes (`src/sim/powerDock.js`)
- **Coupling** used to test "rover origin within 3.0 m of the pad (couplerTol), yaw < 8 deg, still, lift below kneel + 0.12".
  It now tests:
  - the receiver centre (body (0, 0.498, 0)) inside the capture box: +-1.00 m along, +-0.60 m across, yaw < 8 deg
  - speed < 0.08 m/s
  - air gap 0 to 0.22 m (kneeled, about 77 mm; about 0.41 m at ride 0)
  - pilot present
  - coils below 110 C

  `roverOnPad()` now means "in the capture box and still". health.js `atDock` and the coupler fault signature use it, and
  `atDock` keeps its 12 m radius fallback.
- **Coupling fraction**: full power inside +-0.75 / +-0.35 m, linear to zero at the capture limits.
- **Link efficiency**: was 0.85 x (1 - 0.25 (d / 3)^2). It is now 0.92 x (1 - 0.05 u^2) x (1 - 0.25 x (gap - 0.075)+), and it
  only affects the energy drawn from the dock bank. The old 125 W prototype link limit is replaced by the 20 kW full-scale cap
  (17.1 W prototype).
- **Pass-through**: while coupled, the dock now supplies the rover bus load (net of the rover's own solar) first, then the
  CC-CV charge, sharing the 20 kW cap. `R.power.dockW = (chargeW + busW) x 1171.875`. Before, the bus load came out of the
  20 kW, so the pack got 20 kW minus the load. Now it gets the full CC-CV current whenever the cap allows. New fields: `busW`,
  `busWh`.
- **Night park**: coupled at night, CC-CV still stops at the 50 % bank reserve. The dock still carries the rover bus load
  (capped at 600 W full-scale) down to bank SoC 12 %, and the pad pilot is duty-cycled to 20 %. New fields: `keepAlive`,
  `nightParkHours(roverW)`, and the status "NIGHT PARK: dock carrying rover load".
- **Link thermal**: receiver node `Trx` (into the pack, 40 W/K) and pad node `Tga` (into the slab), both derating from
  90 C to 0 at 110 C. New fields: `lossW`, `rxHeatW`, `linkFrac`, `thermalFrac`, `align`. The rover pack does not yet receive
  `rxHeatW` (open item).
- The engage log now reports the receiver along / across offset and the gap.
- **No change** to the bank, solar, vault thermal or CC-CV current model. HUD and command fields are unchanged.

## Visuals (`src/render/powerDock.js`)
- Removed: the 17 m raised pad box (top was 0.15 m), the shroud, the curbs at +-1.45 m (inside the wheel track, with no
  colliders) and the raised round coil and cover (top 0.31 m, which would have cut through the kneeled belly).
- Added:
  - a flush apron with painted wheel lanes
  - a 7 x 5 m sintered slab
  - a 2.4 x 1.6 m GA with 6 modules under a glass-ceramic cover (top 0.057 m); only the modules under the receiver glow
  - a yellow capture-box outline
  - an 8 x 8 fiducial board on the pedestal
  - a purple pedestal LED for night park

  The pad still has no collider.

## Tests (`tests/power_dock.mjs`)
- New checks:
  - receiver offsets, gap and efficiency at engagement during the autonomous charge
  - pass-through energy
  - coil temperatures
  - no link at ride height or with the receiver 0.8 m across
  - link when kneeled on the centre, with the gap 40 to 110 mm
  - night park: no charging below the reserve, link carries the rover load (+-10 %, cap 600 W)
  - a 354 h night park with a 154 W rover: 54.5 kWh carried, bank above its floor
