# Implementer B: lunar thermal, mechanisms, camera lighting, comms

Constants for the Moon (not the vehicle) are in `src/sim/lunarParams.js`. Values without a source are [ASSUMED].

## 1. Sun on the radiators (health.js `_thermal`, ~l.695)
- The solar load on each side panel is `Q = alpha * 1361 * A * max(0, n·s) * sunlit`, with n = ±body z (L = −z, R = +z). This applies to the vault (12 m², split per side), the battery (3 m², split per side) and the hubs.
- alpha is 0.10 when clean, 0.30 when dusty, and 0.50 in the `dust-fins` scenario.
- Louvers are set per side. A side whose net heat rejection is negative (it absorbs more sun than it radiates) closes to louverMin. The other side keeps regulating.
- Hubs: the flat `THERMAL.hub.solarW` is no longer used. Hub solar load is now alpha·S·A·cos scaled by the louver emissivity.
- **The overhang-shade claim is false.** config.js:380-381 says the deck overhang shades the side radiators. At a 9° polar sun the overhang casts only about 0.12 m of shade, so the side panels get almost full sun whenever they face it.
- **Fix to the `dust-fins` scenario (thermal.js:193):** dust raises solar absorptivity from about 0.1 to about 0.5. It does not reduce emissivity. The scenario now sets `fx.dustAlpha` (extra solar on the motor-controller node, l.122). The old emissivity cut has been removed.

## 2. Mechanism thermal nodes (actuators.js `MechNode`, arm.js, health.js)
- Each arm joint gets a radiating node with a 15 W heater, each latch and the deck latch a 10 W heater, and each of the 6 struts a 25 W heater.
- Each node has a thermostat that switches on at −45 C with 5 K hysteresis.
- Motion is locked out below −65 C (PFPE grease limit). Motor copper heat flows into the node.
- Arm under lockout: `moveTo`, `latch` and `deckLatch` are refused, and event `cold_lockout` fires.
- Struts under lockout: an error is logged and autonomy holds.
- Heater power is added to arm `powerW` and to health `heaterW`.

## 3. Camera and stereo lighting (sensors.js)
- For each hit, `_sunlitAt` casts one ray toward the sun, up to 120 m. It casts no ray at night.
- A hit counts as visible only if it is sunlit, floodlit, or within 35 m with the lamps on.
- The 12° sun glare cone now applies to stereo as well as the camera, and only when the camera itself is in sunlight.
- To keep the ray count low, the sun ray is cast only after the cheap FOV and size checks pass. The count is kept in `sensors.shadowRays`.

## 4. Comms (commsLink.js, commands.js, main.js, ui/hud.js)
- `requestEstop` (commands.js:11): an E-STOP is immediate on the local link. Otherwise it is queued for delayS, and repeated presses while one is in flight are dropped. It is wired to the HUD button (hud.js:264), Esc in the console (hud.js:330), Esc on the keyboard (main.js:248) and the gamepad (main.js:278).
- `comms earth` sets a 2.6 s one-way delay. New console commands: `comms auto on|off` and `comms los`.
- `DelayLine` ring buffer:
  - The HUD shows telemetry, SoC and the E-STOP state delayS late (hud.js:524).
  - The comms pill shows `TLM −x s` and `E-STOP IN FLIGHT`.
  - The onboard `cam` view is delayed too (main.js:427).
- `LosMonitor` (main.js:203/375) runs once a second. It casts two rays: mast (5 m) → habitat antenna (+10 m), and mast → Earth (az 0, el 10°).
  - Required line of sight: the local link needs the habitat; Earth mode needs Earth or the habitat.
  - It raises COMMS LOS after 2 blocked checks in a row.
  - It clears only a LOS it raised itself, and then runs `clear comms`, which sends the queued commands.
  - URL `?nolos` turns it off.

## Requests to other owners
- **sim.js:42**: set `earthDelayS: 2.6`. main.js already raises it at startup.
- **rover.js**: refuse strut and ride-height motion while `health.th.strutLock` is set. Honour `arm.coldLock`.
- **config.js:380-381**: remove the overhang-shade comment. `THERMAL.hub.solarW` is now unused.
- **CHANGES.md**: add an entry pointing here.
- **Arm geometry**: tests/arm.mjs now shows 4 FAILs (deck place x −0.347 m, asymmetry flag, hook keep-out, strike count). They come from A's config.js ARM/K edits (stow pose, keepOutMargin, K = 8.16), not from the thermal code.

## Docs only (not implemented)
- **Night-load rebudget at 150 kWh.** The survival load measures 154 W (fdir: 36.3 % of the pack over 354 h; 45.1 % with hub heaters). The allowance is 106 W (25 %). The new mechanism heaters add about 15–40 W of night average on top. Options:
  - a bigger pack,
  - RHUs,
  - parking at the dock for night power.
- **RHU / dock parking** for night heat.
- **True 29.5-day lunar cycle preset** and a **1.8 h eclipse** case.
- **Hook friction** μ ≈ 0.6 in vacuum (no adsorbed films; galling risk).
- **Berthing leak check**: 30 min before the hatch is opened.

## Tests
- `tests/unit_lunar_thermal.mjs`: 15/15 PASS.
  - Joint −145 C, latch −164 C and strut −141 C after 14 days without heaters.
  - With heaters, all nodes are held at −41 to −43 C, with mean heater power of 5.8 W, 1.6 W and 7.2 W.
  - The arm lockout refuses moves and clears 30.7 h after the heaters come on.
- `tests/unit_sensors_light.mjs`: 5/5 PASS.
- `tests/unit_comms.mjs`: 13/13 PASS.
- `tests/actuators.mjs`: PASS.
- `npm run test:fdir`: 66/68 checks pass. The 2 FAILs are the night-budget items above.
- `npm run test:faults`: 1/3 checks pass. The navigation run stopped with "no path to habitat apron" (terrain/route), which is unrelated to this work.
