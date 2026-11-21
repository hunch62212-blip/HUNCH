# Earth prototype thermal load report (2026-11-19)

Scope: the physical 1:16.3 prototype in `docs/ARM_CARGO_BUILD_PLAN.md` §2 (4 × Pololu 37D 30:1, 2 × Cytron MDD10A,
Raspberry Pi 5, ESP32-S3, 5 V buck, 2 UBECs, 9 servos, 12.8 V 10 Ah LiFePO4). The lunar sim already has its own
thermal brain (`docs/THERMAL_BRAIN.md`). That model is out of scope here.

> **Update 2026-11-20:** the prototype now uses a **Raspberry Pi 4 Model B 4 GB** and a **12.8 V 15 Ah LiFePO4** pack (see `PROTOTYPE_POWER_BUDGET.md`). The Pi 4 dissipates less than the Pi 5 (about 4.5 W typical, 7 W peak, against 5.5 W / 8 W used below), so the Pi 5 results here are conservative for the Pi 4. Two parts change: the Pi 5 Active Cooler does not fit the Pi 4, so use a Pi 4 heatsink plus a 30–40 mm 5 V fan (BOM row 33); and `usb_max_current_enable` is a Pi 5 setting. The Pi 4's USB ports share 1.2 A, so power both LiDARs from the 5.1 V buck and use the Pi's USB for data only.

Calculator: `node scripts/thermal_load_proto.mjs`. It is a hand model, not a measurement. Electrical values come from
the plan (§1.5, §5.1). Every thermal resistance and capacitance is `[ASSUMED]` until the bench test in §6 replaces it.

## 1. Verdict

| Part | Problem? | Why |
|---|---|---|
| **Drive motors (37D 30:1)** | **Yes** | Stalls and hard pivot turns. The MDD10A has no current limit, so a stall at full charge gives 6.6 A and 95 W of heat in the motor. The winding passes 130 °C in about 24 s. In a 20 min demo loop with pivot turns on a grippy floor, the winding peaks at 108 °C and the can at 71 °C (25 °C room), or 128 / 88 °C at 40 °C outdoors. |
| **Printed PETG motor mounts** | **Yes** | A can at 71–88 °C is at or above PETG's heat-deflection temperature (about 70 °C). The clamp creeps, the motor loosens and the wheel goes out of line. |
| **Raspberry Pi 5** | **Yes, if bare or sealed in the tub** | A bare Pi 5 throttles under full load even on an open bench (86 °C). In the closed tub at 40 °C it throttles all the time (94–113 °C). With the official Active Cooler and a vented tub it stays at 64–73 °C. |
| **Electronics tub (sealed)** | **Borderline** | 13–16 W of steady heat raises the air by 13–16 K. Sun on a dark deck adds about 15 K more. |
| **Arm servos / UBEC-A** | **Conditional** | Holding the ACM at full reach draws about 11 W per shoulder (plan §4.3), and hobby servos have no brake. Safe for a 25 s pick. Not safe for a hold of several minutes. |
| Cytron MDD10A | No | Rated 10 A continuous with no heatsink. Our worst running current is 3.8 A (6.6 A at stall): under 1 W per channel. |
| 5 V buck | No (verify the part) | About 2 A load, about 1.4 W loss. Cheap "5 A" bucks are often good for only about 3 A without a heatsink. |
| LiFePO4 pack | No for normal driving | About 1 W at 5 A. Peak current is the real issue, not heat (§4.8). |
| ESP32-S3, PCA9685, BNO085, cameras | No | Each is under 0.5 W. The BNO085 gyro bias does drift with temperature, so keep it away from the Pi and UBECs. |
| YDLIDAR X2L | Outdoors only | Rated operating range tops out at about 40 °C `[VERIFY datasheet]`. A sunlit dark nose will exceed it. |

## 2. Drive motors in detail

Motor constants (plan §5.1): R = 2.18 Ω, Kt(output) = 0.258 N·m/A, k = 0.334 V·s/rad, gearbox continuous limit 0.98 N·m.
Loaded rover is 9.3 kg with 4 driven wheels. Driven axles carry 2/3 of the weight at x_cg = 0.

| Case (per motor) | Torque | % gear limit | Current | Heat | Steady winding, 25 °C room | Time to 100 °C |
|---|---|---|---|---|---|---|
| Cruise, flat floor | 0.07 N·m | 8 % | 0.5 A | 1.9 W | 38 °C | never |
| Cruise, sand / gravel | 0.22 | 23 % | 1.1 A | 4.4 W | 58 °C | never |
| 10° ramp | 0.40 | 41 % | 1.8 A | 9.5 W | 101 °C | 16 min |
| 20° ramp | 0.65 | 66 % | 2.7 A | 20 W | 191 °C | 3.3 min |
| Pivot turn, low grip (μ 0.4) | 0.52 | 53 % | 2.2 A | 13 W | 176 °C | 7.5 min |
| **Pivot turn, rubber on concrete (μ 0.8)** | **0.93** | **95 %** | **3.8 A** | **37 W** | 448 °C | **44 s** |

Stall at full PWM (the MDD10A does not limit current):

| Pack voltage | Current | Heat | To 100 °C | To 130 °C | To 130 °C at 40 °C room |
|---|---|---|---|---|---|
| 14.4 V (fresh off the charger) | 6.6 A | 95 W | 14 s | 24 s | 20 s |
| 13.2 V (nominal) | 6.0 A | 80 W | 17 s | 32 s | 25 s |
| 12.4 V (low) | 5.7 A | 70 W | 21 s | 41 s | 31 s |

How to read this:
- Normal driving is fine. Cruising never gets hot, and short ramps are fine.
- **Skid-steer pivot turns are the real thermal load.** On a grippy floor each driven motor runs at 95 % of the gearbox's continuous torque. So pivots also wear out the gears, not only heat the motor.
- **A stall is a 15–30 s fuse.** A wheel jammed on a rock with no detection reaches 131 °C in 30 s. Detection that cuts power after 5 s caps it at 61 °C.
- A full LiFePO4 at 14.4 V drives a 12 V motor 20 % over its rating. Stall heat rises by V², so that is 44 % more heat.

## 3. Pi 5 and electronics tub

Heat inside the tub (420 × 292 × 156 mm, closed by the deck; the motors are outboard in the pods):

| State | Pi | Other 5 V | Buck | MDD10A ×2 | UBECs | Total | Air rise, sealed | Air rise, one 40 mm fan |
|---|---|---|---|---|---|---|---|---|
| Idle | 2.7 W | 4.8 | 1.0 | 0 | 0 | 8.5 W | +8 K | +3 K |
| Driving on sand | 5.5 | 4.8 | 1.4 | 0.2 | 0.2 | 13 W | +13 K | +4 K |
| Arm holding at reach | 5.5 | 4.8 | 1.4 | 0 | 3.8 | 16 W | +16 K | +5 K |
| Worst-case peak (seconds only) | 8 | 4.8 | 1.7 | 2.3 | 9.9 | ~27 W (+ pack) | — | — |

Pi 5 SoC temperature with ROS, 2 LiDARs and SLAM (5.5 W) and at full load (8 W). It throttles at 80 °C:

| Cooling | Open bench, 25 °C | Sealed tub, 25 °C room | Sealed tub, 40 °C | Vented tub, 40 °C | Sealed + sun on dark deck |
|---|---|---|---|---|---|
| Bare board | 67 / **86** | 79 / **98** | **94 / 113** | **86 / 105** | **109 / 128** |
| Passive aluminium case | 54 / 67 | 66 / 79 | **81 / 94** | 73 / **86** | **96 / 109** |
| **Official Active Cooler** | 45 / 54 | 57 / 66 | 72 / **81** | **64 / 73** | **87 / 96** |

(Each cell is ROS load / full load, in °C. **Bold** means at or over 80 °C.)

A throttled Pi does not break, but it slows SLAM and navigation, and that shows up as laggy autonomy in a demo.

## 4. Fixes, in priority order

### 4.1 Firmware stall and thermal protection on the ESP32 (most important, costs $0)
The MDD10A cannot protect the motors, so the ESP32 has to. Put this beside the existing 250 ms heartbeat watchdog.
- **Stall rule:** if |duty| > 0.3 and |ω_wheel| < 0.5 rad/s (from the PCNT encoder) for 1.5 s, set that wheel's PWM to 0. Latch a `WHEEL_STALL` fault. Allow a retry after 10 s, at most 3 times, then hold.
- **Estimated-temperature model (I²t):** estimate the current per wheel every 10 ms:
  `I = (duty·V_bat − k·ω) / R`, with R = 2.18 Ω and k = 0.334 V·s/rad.
  Run the two-node model from `scripts/thermal_load_proto.mjs` (8 lines of C). Derate the duty linearly from an estimated 90 °C winding, and cut power at 110 °C.
- **Voltage compensation:** cap `duty ≤ 12.0 / V_bat`, so a fresh 14.4 V pack never puts more than 12 V on a motor. Read V_bat from the existing INA226 or a divider.
- Report the estimated winding temperatures, the stall flags and Pi `vcgencmd get_throttled` in telemetry.

Effect: a stall cut at 5 s peaks at 61 °C instead of 131 °C. This closes the damage case completely.

### 4.2 Turn gently, don't pivot ($0)
In the demo loop, arc turns instead of pivots drop the peak from 108 / 71 °C to 82 / 57 °C. They also take the gearbox off its 95 % limit.
- Planner: prefer turn radii of at least 1 track width (0.47 m). Allow a pivot only when it is required, for at most 3 s, then pause for 10 s.
- The sim's v1.5.0 "gentle turning" navigation already behaves this way at full scale. Carry the same rule into the prototype's nav stack.

### 4.3 Switch to 37D 50:1 gearmotors (about $0 extra; same body, +2 mm length)
The same motor gives 1.67× torque per amp, so copper loss on every load case falls to about 36 %. The demo loop drops to 61 / 48 °C (25 °C room) and 78 / 64 °C (40 °C).
- Trade-off: no-load speed falls from 330 rpm to 200 rpm. That is still about 1.3 m/s at the wheel, well above any demo speed.
- If chosen, change `params.csv` `mot_L` from 68 to 70 mm. In `src/sim/config.js`, update `ROVER.motor.protoNoLoadRpm` / `protoCapRpm` (300 / 280) and `ACT.proto.gm37d30` (the sim scales full-scale speeds from these).
- **This is a design decision for you.** The plan currently picks 30:1.

### 4.4 Motor mounts: PETG → ASA (or aluminium)
- Print `PRT_MOTOR_MOUNT` in **ASA** (HDT about 95 °C). ASA is already on the materials list and needs the same printer setup. That alone removes the creep failure.
- An aluminium 37D bracket (Pololu sells one) also works as a small heatsink (−3 to −4 °C). It is mainly a stiffness and creep fix, not a cooling one.

### 4.5 Pi 5 cooling and tub ventilation (about $12)
- **Official Raspberry Pi Active Cooler** ($5): required, not optional. It is the only option that keeps the Pi out of throttling in a warm tub.
- **One 40 mm 5 V fan** exhausting high at the rear of the tub, plus a filtered intake low at the front (dust/sand mesh). Put the Pi's cooler in the intake airflow path. This drops the tub air rise from +13 K to +4 K.
- Keep the MDD10As and UBECs downstream of the Pi, not under it.

### 4.6 Sun and outdoor demos
- Use a light-coloured or white deck. Print the nose and battery lid in white ASA. A black deck in sun adds about 15 K to everything inside.
- Check the X2L operating range against the datasheet before any outdoor summer run.

### 4.7 Arm servos
- Never hold the ACM at reach. Go straight to the deck place point or the stow cradle.
- Once the arm is seated in the stow cradle, send PCA9685 full-off to that channel. Most servos then go limp, and hold current drops to about 0. The cradle carries the load.
- Add a hold timer: if the arm is loaded and away from stow for more than 20 s, set it down. The plan's `arm_servo` over-temp fault (§6) is the sim mirror of this.
- Mount UBEC-A on the tub wall in the airflow. It sheds about 4 W while the arm holds and about 10 W at peak.

### 4.8 Wiring and battery (current, not heat)
- Four stalled motors at 14.4 V draw 26 A, plus up to 12 A of servo peak: about 38 A. That is over the 30 A fuse, which is correct behaviour. **Check the pack's BMS continuous rating** `[VERIFY]`. Many 12.8 V 10 Ah packs have a 10–20 A BMS, which would trip before the fuse. The firmware stall cut (4.1) keeps you well under it.
- Use at least 20 AWG for the motor leads. At a 6.6 A stall, 22 AWG warms noticeably over a long run.
- The 5 V buck should be a real 5 A part with headroom. The Pi 5 also needs a 5 A supply, or `usb_max_current_enable=1` in `config.txt`, to give two USB LiDARs more than 600 mA.

## 5. Recommended package

| Fix | Cost | Effect |
|---|---|---|
| 4.1 Firmware stall + I²t + voltage cap | $0 | Removes the stall damage case |
| 4.2 Arc turns, timed pivots | $0 | Demo peak 108 → 82 °C |
| 4.4 ASA motor mounts | about $0 (filament on hand) | No mount creep |
| 4.5 Active Cooler + 40 mm fan | about $12 | Pi never throttles below 40 °C ambient |
| 4.7 Servo stow + PWM off | $0 | No servo cook-off during holds |
| 4.3 50:1 gearmotors | about $0 | Large margin (61 °C peak); a speed trade-off, your call |

Combined (arc turns + 50:1 + aluminium or ASA mount): 51 / 42 °C winding / can at 25 °C, and 67 / 58 °C at 40 °C.

## 6. Bench test to replace the assumptions (about 1 h)
1. Epoxy a 10 kΩ NTC (or put a K-type thermocouple) on one 37D can, and another in the tub air. Log them on the ESP32 ADC.
2. Hold one wheel and pivot on carpet for 60 s, then let it cool for 10 min. Log the INA226 current and the can temperature.
3. Fit R_ha and C_h to the cooling curve: τ = R·C, ΔT_ss = P·R. Replace the `[ASSUMED]` values in `TH` in the script and rerun it.
4. Run the Pi with `stress-ng --cpu 4` in the closed tub for 15 min and log `vcgencmd measure_temp`.

## 7. Other things found
- `public/team/js/rover_bom.js` (the team page's power/thermal model) still describes 6 sealed brushless hub motors, an Arduino Mega, 3-phase FET drivers and a vacuum OSR radiator. It does not describe the Earth build in the plan (4 × 37D, MDD10A, ESP32-S3). Its thermal numbers do not apply to the real prototype. Updating it is plan phase P4.
- `CLAUDE.md` says "6WD tracked skid-steer; track separation = 0.42 m". The plan has 4 driven wheels and 2 idlers, with a 0.47 m track (`tr_z` = 235). This report uses the plan.
