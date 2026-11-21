# Implementer A: mass path, K = 8.16 scale, braking, derate, bump stop

## Changes (owned files)
- `src/sim/config.js`: K = 8.16 (K_LEGACY = 12.192/0.747 exported, KS = K/K_LEGACY). Rover rescaled: sprung 4100 kg + 6 x 150 kg
  unsprung = 5000 kg; CoM, inertia, hardpoints, strut strokes, wheel (r 0.53 m), susp k/c (~1.0 Hz heave, legacy damping 0.72),
  bump stop, motor torques/gains derived from K. Tyre tireK 210000 / tireC 3500 (scaled with unsprung mass: keeps the legacy
  6 Hz wheel hop; a length scaling gave 9.4 Hz and the wheel loads chattered). DRIVE.driven = 4 outer wheels; accel.brakeMax =
  0.79 g x 4/6 = 0.853 m/s^2, lon 0.75, estopFactor so lon x factor = 0.853. battery 150000 Wh, dockMaxW 20 kW.
  cyl40 = 12.192 x 3.0 m. Motor RPM formulas unchanged (sim RPM undecided).
- `src/sim/rover.js`: single mass path `massProps()` / `syncMass()` (sprung + unsprung at nominal axles + extraLoads) ->
  `setAdditionalMassProperties`, on build and whenever loads change mass > 1 kg or move the CoM > 2 cm (linvel corrected).
  Strut block takes the unsprung along-strut weight and inertia back out of the body; gPerp impulse removed; weight-only
  extraLoads impulse removed. `totalMass` getter used by the tyre solver and rails. `_stability` gets the sprung CoM.
  Torque-speed derate only when motoring (T x omega > 0). Bump-stop damper implicit (backward Euler, one-sided).
  Legacy-scale collider / capture metres x KS. Dock charge capped at battery.dockMaxW.
- `src/sim/dynamics.js`: m = R.totalMass (includes held loads). brakeDecel 0.6 kept.
- `src/sim/acmCarry.js`: no change needed (rover re-syncs mass each step from extraLoads).
- New tests: `tests/unit_mass.mjs`, `tests/unit_scale.mjs`.

## Night energy (config.js comment corrected)
Measured with the tests/fdir.mjs E procedure: survival load ~154 W -> 354 h night ~55 kWh = ~36 % of 150 kWh. The night passes,
but tests/fdir.mjs E (`<= 25 %`) will FAIL with the 150 kWh pack. Owner should change the limit or the survival budget.

## Needed in files I do not own
- `tests/headless.mjs`: kneel / carry ride thresholds (>= 0.5 m) -> ride strokes are now 0.323 / 0.402 m (scale x KS ~0.5).
- `tests/fdir.mjs` E: 25 % night assertion (see above). `tests/power_dock.mjs`: check against the 20 kW charger cap.
- `src/sim/powerDock.js:157`: dockW = chargeW x PROTO_TO_FULL_WH can exceed 20 kW; capped in rover.js, better cap at source.
- Render (`aegis.js`, `props.js`, `world.js` visuals): hard-coded geometry for the new K and the 12.192 x 3.0 m module.
- `CARGO_TYPES.box20` sled metres (lugXs, adapterLen 8.4, carryX, lugX 3.7), ARM link masses / inertias, ACT torques,
  ACM 2400 kg, THERMAL radiator area, DOCK vestibule: not rescaled (config entries I left, need an owner decision).
- Wheel casts on a flat cuboid collider chatter (also with the old rover.js); the test ramp uses a heightfield.
- `docs/POWER_BUDGET.md`, `CLAUDE.md` ("1:16.3"), `CHANGES.md` entry.

## Docs-only notes (not implemented)
- float32 floating origin: world coordinates ~900 m give ~0.1 mm float32 resolution in Rapier; re-centre for long traverses.
- Tyre shear: Janosi displacement uses slip x patch length per step; a true shear-displacement state per contact would fix
  low-speed creep and parked-anchor behaviour.
- Rocker-bogie: corners are independent struts with a warp-removal fix, not a real rocker / differential linkage.
- Froude similarity: prototype speed matching the full-scale rover needs v_p = v_f / sqrt(K) (and g scaling Earth vs Moon:
  v_p = v_f sqrt(g_E / (g_M K)) ~ 0.86 v_f) - affects how prototype 100 RPM maps to the sim RPM (still undecided).
