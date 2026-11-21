# AEGIS-V1 prototype power budget (12.8 V 15 Ah LiFePO4)

Hand calculations, not measurements. Bench checks are in §9. The full-scale sim budget is in `POWER_BUDGET.md`.

Rev A, 2026-11-19. NASA HUNCH prototype rover. Load, peak current and runtime for the bench rover on a 15 Ah pack: 4 × 12 V drive gearmotors on 2 dual drivers, a 9-servo twin arm, and a Raspberry Pi 4 (4 GB) with the sensor set from the arm/cargo build plan.

## Verdict

**Yes. One 12.8 V 15 Ah LiFePO4 pack runs the whole rover.**

- **Energy is not the problem.** A typical demo hour averages about 47 W (3.6 A). 157 Wh usable gives about 3.4 h, 2.4 h on a hard field test, and 1.1 h of nonstop 20° climbing.
- **Peak current is the problem.** Normal driving peaks near 18 A. All four wheels stalled at full charge is 28 A. Everything stalled at once is 45 A. Many cheap 15 Ah packs have a 15–16 A BMS and will cut out, which reboots the Pi.
- **Condition for "yes":** the pack's BMS must be rated at least **20 A continuous / 40 A pulse** (30 A / 45 A preferred), and the ESP32 firmware must ramp motor PWM and block arm motion while the wheels are driving.
- **Two batteries are not needed.** Use the second 15 Ah pack as a hot-swap spare. Split power (one pack for motors, one for logic) only if you are stuck with a weak 15 A BMS.

| Figure | Value | Meaning |
|---|---|---|
| Pack energy | 192 Wh | 12.8 V × 15 Ah |
| Usable energy | 157 Wh | Keeping the 18 % reserve the plan already uses |
| Average draw | 3.6 A | Demo hour (0.24 C) |
| Realistic peak | 18 A | Drive with soft-start |
| Drive stall peak | 28 A | 4 motors, 14.4 V |
| Runtime | 3.4 h | Demo hour profile |

## 1. What was assumed

"15 Ah" has no voltage or chemistry attached, so this report uses a **12.8 V (4S) LiFePO4** pack. That matches the 12 V motors and the 12.8 V 10 Ah pack already in the build plan. Voltage swings from 14.4 V right off the charger to 10.0 V at BMS cutoff, and sits near 13.0–13.2 V for most of the discharge.

**Other chemistries**
- **3S Li-ion 11.1 V 15 Ah** (166 Wh): also works. Runtime drops about 13 %; motors run about 8 % slower at nominal.
- **12 V sealed lead-acid 15 Ah**: avoid. Around 4.5 kg (the mass budget gives the battery 1.65 kg), only about 50 % usable, and voltage sags hard under motor peaks.

**Rover numbers used**
- Loaded mass 9.6 kg: the plan's 9.30 kg plus about 0.3 kg for the heavier 15 Ah pack.
- Wheel radius 65 mm (130 mm wheels), 4 driven, 2 idlers.
- Rolling resistance: 0.05 hard floor, 0.15 sand, gravel or regolith simulant.
- Converter efficiency 90 % (bucks, UBECs); motor driver 95 %.

## 2. Every part on the bus

Average is a realistic working value. Peak is the highest value that part can pull for a second or two. Watts are measured at the part; the battery-side totals in section 3 add converter losses.

| Part | Qty | Rail | Avg each | Peak each | Basis |
|---|---|---|---|---|---|
| **Drive: battery direct through 2 × Cytron MDD10A** | | | | | |
| Pololu 37D 30:1 12 V gearmotor + 64 CPR encoder (#4752) | 4 | Batt | 0.5–2.5 A | 5.5 A (6.6 A @ 14.4 V) | No-load 0.2 A, stall 5.5 A, stall torque 14 kg·cm. Avg depends on terrain, see section 3. |
| Cytron MDD10A dual driver | 2 | Batt | 0.3 W | 30 A / ch (<10 s) | 10 A continuous per channel, 5–25 V. Has **no current limit**: it will pass full stall current. |
| **Arm rail A: 7.4 V via UBEC-A (20 A)** | | | | | |
| 60 kg·cm HV servo (DS5160 class), shoulders + elbows | 4 | 7.4 V | 0.1–1.5 A | 5 A [ASSUMED] | Shoulder holds 20 kg·cm at reach (46 % of stall), plan measures about 11 W hold per shoulder. Stall current not published; measure it. |
| **Arm rail B: 6.0 V via UBEC-B (10 A)** | | | | | |
| MG996R, 2 wrists + deck latch | 3 | 6 V | 0.1–0.5 A | 2.5 A | Datasheet stall 2.5 A. Wrist load under 5 % of stall. |
| MG90S, hook latch gates | 2 | 6 V | 0.05 A | 0.7 A | Runs a few times per pick, then idles. |
| **Logic rail: 5.1 V via a 5 A buck** | | | | | |
| Raspberry Pi 4 Model B 4 GB | 1 | 5.1 V | 4.5 W | 7.0 W | Measured 2.5–3 W idle, 6.4–6.8 W at full 4-core load. Avg assumes ROS + vision at moderate load. |
| YDLIDAR X2L | 2 | 5 V | 1.75 W | 2.5 W | X2 datasheet: 350 mA working, 500 mA at spin-up. |
| OV5647 camera (CSI) | 1 | Pi | 0.3 W | 0.3 W | The Pi 4 has one CSI port. See section 8. |
| ESP32-S3 DevKitC-1 (motion MCU) | 1 | 5 V | 0.5 W | 1.0 W | Wi-Fi off; PCNT, MCPWM, I²C only. |
| BNO085 IMU, PCA9685, 2 × INA226, 2 × VL53L1X | — | 3.3/5 V | 0.2 W | 0.2 W | All small signal parts together. |
| Pi cooling fan (30–40 mm) | 1 | 5 V | 1.0 W | 1.0 W | Needed: the Pi 4 throttles near 80 °C in a closed bay. |
| **Logic rail total** | | 5.1 V | **10.0 W** | **14.5 W** | 11.1 W avg / 16.1 W peak at the battery. A 5 A buck runs at 58 % at peak. |

## 3. Load by operating mode

Battery-side power for each thing the rover does. Drive current comes from the motor's linear torque–current line: `I = 0.2 A + (τ / 1.37 N·m) × 5.3 A`, with τ = total rolling and grade force × 0.065 m ÷ 4 motors.

| Mode | Torque / motor | Current / motor | Drive W | Total W | Battery A | Note |
|---|---|---|---|---|---|---|
| Idle, parked, sensors on, arm stowed | — | — | 0.6 | 15 | 1.2 | Cut servo rail power when stowed to save 3–4 W. |
| Cruise, flat hard floor | 0.8 kg·cm | 0.50 A | 25 | 40 | 3.2 | Classroom, gym, tile. |
| Arm pick-and-place (parked) | — | — | 0.6 | 68 | 5.3 | About 25 s per pick. Peak 132 W / 10.3 A while lifting. |
| Cruise, sand / gravel / simulant | 2.3 kg·cm | 1.09 A | 55 | 70 | 5.5 | Rolling resistance 0.15. |
| 10° slope, loose surface | 5.0 kg·cm | 2.10 A | 106 | 122 | 9.5 | |
| 20° slope, hard surface, loaded | 6.1 kg·cm | 2.50 A | 126 | 142 | 11.1 | Under the 10 kg·cm gearbox limit. Rear motors work harder than this average. |

## 4. Peak current against the BMS

This decides whether one battery works. Three common BMS classes found on 12.8 V 15 Ah LiFePO4 packs are compared below.

| Event | Battery A | Basic BMS (16 A cont / 25 A pk) | Mid BMS (20 A cont / 40 A pk) | Strong BMS (30 A cont / 45 A pk, 2 s) |
|---|---|---|---|---|
| Idle | 1.2 | OK | OK | OK |
| Flat cruise | 3.2 | OK | OK | OK |
| Arm lift peak, wheels stopped | 10.3 | OK | OK | OK |
| Sustained 20° climb | 11.1 | OK | OK | OK |
| Drive start or obstacle, PWM ramped over ~300 ms | 17.5 | Over continuous, short burst | OK | OK |
| All 4 wheels stalled, full charge (14.4 V) | 27.9 | Trips | OK for seconds | OK |
| Wheels stalled + both arms stalled + all servos stalled | 45.1 | Trips | Trips | At pulse limit |

**Why a trip matters more than a dead battery.** A BMS trip cuts every rail at once, so the Pi loses power without shutting down, which can corrupt its SD card, and the arm drops whatever it is holding. A weak pack fails because of these trips, long before it runs out of energy.

The 45 A row should never happen if the firmware rules in section 7 are followed. It is listed to size the main fuse and to show what the interlock protects against.

## 5. Runtime

Three duty cycles, each on 157 Wh usable (15 Ah) against 105 Wh usable for the 10 Ah pack in the current plan.

| Profile | Time split | Avg W | Avg A | 15 Ah | 10 Ah |
|---|---|---|---|---|---|
| Bench / software dev | 70 % idle, 20 % flat drive, 10 % arm | 26 | 2.0 | **6.1 h** | 4.1 h |
| Demo hour | 30 % idle, 35 % flat, 10 % sand, 5 % 20° slope, 20 % arm | 47 | 3.6 | **3.4 h** | 2.3 h |
| Hard field test | 10 % idle, 30 % flat, 30 % sand, 15 % 20° slope, 15 % arm | 66 | 5.2 | **2.4 h** | 1.6 h |
| Nonstop 20° climb (worst case) | 100 % slope | 142 | 11.1 | **1.1 h** | 0.7 h |
| Parked, sensors on | 100 % idle | 15 | 1.2 | **10.3 h** | 6.9 h |

Charge time with a 14.6 V LiFePO4 charger: about 3.2 h at 5 A, 1.6 h at 10 A. With two packs, one charges while the other runs.

## 6. One battery or two

| Setup | Good | Bad | Use it when |
|---|---|---|---|
| **One pack, split regulated rails** (recommended) | Lightest, simplest wiring, one charger, one fuel gauge. | Needs a BMS of 20 A / 40 A or better and the firmware rules. | You can buy or already have a pack with a 20–30 A BMS. |
| Two identical packs in parallel | 30 Ah, each BMS sees half the peak (14 A for a 28 A event). Runtime doubles. | About +2 kg. Packs must be the same model and charged to the same voltage before you connect them. | Both packs have weak 15 A BMS boards and you need long runtime. |
| Two packs, one for motors, one for logic | Motor stalls can never reboot the Pi. | Two chargers or a swap routine, uneven drain, ground must still be shared. | Last resort if only weak-BMS packs are available. |

The plan's idea of a spare pack charging at the dock still fits: run one 15 Ah pack on the rover and keep the second charged as a swap.

Mass: a 12.8 V 15 Ah LiFePO4 pack is usually about 1.8–2.2 kg against 1.65 kg in the mass table. Mounted low at x = −80 mm, the extra 0.35 kg moves the CG about 3 mm rearward, which does not change the CG verdict.

## 7. Wiring, fuses and regulators

```
12.8 V 15 Ah LiFePO4  (BMS ≥ 20 A cont / 40 A pk)
 └─ XT60 ── main fuse 30 A ── main switch ── INA226 #3 (main shunt)      12 AWG
     ├─ E-stop (cuts motors + servos, NOT the Pi)
     │   ├─ fuse 15 A ── MDD10A #1 ── FL, RL motors        14 AWG, 1000 µF low-ESR cap at driver
     │   ├─ fuse 15 A ── MDD10A #2 ── FR, RR motors        14 AWG, 1000 µF low-ESR cap at driver
     │   ├─ fuse 15 A ── UBEC-A 20 A @ 7.4 V ── 4 × 60 kg servos   16 AWG, INA226 #1
     │   └─ fuse 7.5 A ── UBEC-B 10 A @ 6.0 V ── 3 × MG996R, 2 × MG90S   16 AWG, INA226 #2
     └─ fuse 3 A ── 5.1 V 5 A buck ── Pi 4, ESP32, LiDARs, fan, sensors   18 AWG
```

**Changes to the current BOM**
- **Main bus wire** from 16 AWG to **12 AWG** between battery, fuse, switch and the split. 16 AWG is fine for the servo branches.
- **Add a 5.1 V 5 A buck** for the logic rail. The BOM has no Pi power supply. Do not run the Pi from a servo UBEC.
- **Add a third INA226** (or a current-sense module) on the main line to count amp-hours. LiFePO4 voltage is flat, so voltage alone is a poor fuel gauge.
- **Branch fuses** as shown, plus a 1000–2200 µF low-ESR capacitor at each MDD10A input.
- Check the E-stop's contact rating. If it is under 30 A DC, have it switch a relay or contactor instead.

**Firmware rules (ESP32)**
1. Ramp drive PWM from 0 to target over at least 300 ms. Never jump straight to full reverse.
2. Stall guard: if an encoder reads near zero with PWM above 50 % for 500 ms, cut that motor.
3. Arm interlock: no shoulder or elbow motion while wheels are commanded. Holding is fine.
4. Limit total current: if the main INA226 reads above 25 A for 200 ms, halve drive PWM.
5. Low battery: warn at 12.4 V under light load (about 20 %), stop arm work at 12.0 V, return to dock at 11.6 V.

## 8. Pi 4 instead of Pi 5

The build plan and the CAD were written for a Pi 5. The Pi 4 4 GB uses less power (about 7 W peak against 10–12 W for a Pi 5), but it has some limits.

| Item | Impact | Fix |
|---|---|---|
| **One CSI camera port** | The plan lists OV5647 cameras (plural). A Pi 4 can only run one on CSI. | Use one CSI camera plus one USB camera, or an Arducam camera multiplexer. |
| **USB ports share 1.2 A** | Two LiDARs at spin-up take about 1.0 A. Adding a USB camera can go over. | Power the LiDARs from the 5.1 V buck (through the X2L adapter's power input or a powered hub) and keep only data on the Pi's USB. |
| **Supply voltage** | The Pi 4 shows an under-voltage warning and throttles below about 4.63 V. | Set the buck to 5.1 V, keep the feed wire short (20 AWG or thicker), and feed the USB-C port or 5 V GPIO pins. |
| **Heat** | Throttles near 80–85 °C in a closed bay. | Heatsink + fan (included in the budget above). |
| Board outline | Pi 4 and Pi 5 share the 58 × 49 mm hole pattern. | Electronics tray needs no change. Update the BOM line and the open item in §11 of the build plan. |

## 9. Measure these on the bench

The numbers marked as assumed should be checked before the final pack is bought. All three INA226 monitors are already in, or added to, the BOM for this.

1. **60 kg servo stall current** at 7.4 V. Use a current-limited bench supply. If it is above 6 A, UBEC-A (20 A) is too small for four stalled servos and should be upgraded to 25–30 A.
2. **Drive current on your real test surface** at cruise and on the steepest ramp you will show. Compare with the 0.5 A and 2.5 A per motor in section 3.
3. **Your pack's BMS rating.** Read the label or datasheet for continuous and peak discharge current. This is the one number that decides section 4.
4. **A full demo-hour run** logged with the main INA226. It should use about 47 Wh. Above 60 Wh means something is dragging (wheel rub, servos fighting, idler friction).

## Sources

- Project: `docs/ARM_CARGO_BUILD_PLAN.md` (BOM, servo torques, mass table, 7.4 V × 12 A arm peak, 11 W shoulder hold) and `public/team/js/rover_bom.js` (12.8 V 10 Ah pack, 18 % reserve).
- Pololu 37D 30:1 12 V with encoder: [pololu.com/product/4752](https://www.pololu.com/product/4752), [#4742 specs](https://www.pololu.com/product/4742/specs) (0.2 A no-load, 5.5 A stall, 14 kg·cm).
- Raspberry Pi 4 power: [Raspberry Pi thermal testing](https://www.raspberrypi.com/blog/thermal-testing-raspberry-pi-4/) (2.47 W idle, 6.79 W worst case), [pidramble.com](https://pidramble.com/node/27) (2.7 W / 6.4 W), [element14 review](https://community.element14.com/products/roadtest/b/blog/posts/raspberry-pi-4-model-b-review).
- YDLIDAR X2 datasheet (no X2L electrical table was found): [waveshare.com](https://files.waveshare.com/upload/b/be/YDLIDAR_X2_Datasheet.pdf).
- DS5160 servo specs (65 kg·cm at 7.4 V; no stall current published): [sigmanortec.ro](https://sigmanortec.ro/en/servomotor-ds5160-metal-ip67-60kg-74v), [motorobit.com](https://www.motorobit.com/ds5160-hv-60kg-digital-servo-motor-180).
- MG996R: [TowerPro datasheet (Cornell)](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2017/kmv48_rp625/kmv48_rp625/kmv48_rp625/MG996RTowerPro.pdf).
- 15 Ah LiFePO4 BMS examples: [Bioenno BLF-1215WS](https://cdn.shopify.com/s/files/1/0219/0154/files/BLF-1215WS_SEALED_SPECIFICATION_b.pdf) (30 A cont / 45 A 2 s), [LiTech 4S1P 15 Ah](https://litechpower.com/?p=3703) (16 A cont / 25 A peak).
