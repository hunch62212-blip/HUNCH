# AEGIS-V1 Arm-Handled Cargo, 4-Wheel Outer Drive and Prototype Build Plan (v1.8.0)

> - **Frames.** Body frame is +X forward, +Y up, +Z right. y = 0 is the ground at nominal ride (wheel centre at y = r).
> - **Scale.** K = 16.32. Prototype mm × K gives full-scale metres.
> - **Gravity.** Prototype numbers use Earth g (9.81 m/s², bench testing). Full-scale numbers use lunar g (1.62 m/s²).
> - **Tags.** `[ASSUMED]` means the value has no source; it goes in `config.js` with that tag. `[VERIFY]` means a placeholder the team will replace with measured or datasheet sizes.

---

## 0. Decisions

| Topic | Decision |
|---|---|
| Arm role | Arms handle a new small cargo class, the **ACM (Arm-handled Cargo Module)**, which sits directly on regolith. The 14.9 t cyl40, box20 and pod4 stay on the existing straddle/saddle system. Arms cannot lift those: tip ratio is 263 % for cyl40 and 108 % for box20 (§3.4). |
| Number of arms | **2 mirrored arms** (port and starboard), commanded in sync. The ACM passes *between* them in an overshot swing from the front ground to the rear deck. |
| Arm DOF | **3 per arm**: shoulder pitch, elbow pitch, wrist pitch. A latch gate on each hook adds a 4th actuated function. Total servos: 6 joints + 2 latches + 1 deck latch = **9**. |
| End effector | **V-seat trunnion hook plus servo latch gate.** The ACM carries two Ø12 mm trunnions on its ±Z faces, 20 mm above its CG, so it hangs as a self-levelling pendulum. Hook side cheeks have a 30° lead-in to centre it laterally. |
| Deck interface | Two tapered locating pins on the deck (one round, one diamond; 3-2-1 kinematic locate). The ACM underside has a round bushing and a slot bushing. A servo rotary latch on the deck grips a keeper bar on the ACM. |
| Prototype lengths | L1 = L2 = 200 mm. Shoulder at (x 150, y 275, z ±175) mm. Pick point at x = 420 mm, place point at x = −120 mm. |
| Capacity | Prototype rated **1.0 kg** on Earth (proof test 1.25 kg). Full scale rated **3,000 kg** lunar; default ACM is 2,400 kg `[ASSUMED]`. |
| Prototype servos | Shoulder and elbow: **60 kg·cm-class HV servo** at 7.4 V (one part number for all four). Wrist: MG996R. Latches: MG90S. Deck latch: MG996R. |
| Drive | **FL, FR, RL, RR driven; ML and MR are passive idlers.** Sim: peakTorque 5600 → 8400 N·m and brakeTorque 9000 → 13500 N·m (keeps the 6-motor budget). Prototype: Pololu 37D 30:1 with encoder ×4. |
| Weight layout | Use **Option R** (§3): battery low at x = −80 mm, electronics at +50 mm, ACM deck place point at −120 mm, loaded CG at x ≈ 0. Reject the "everything at the back" layout (Option U). |
| Tip rule | Arm lifts are refused or aborted when the predicted tip ratio is above **0.75** (ASME B30.5-style "rated ≤ 75 % of tipping"), or when `tilt.marginF` < 8°. |
| CAD | Fusion 360 scripts in `cad/fusion/`. **`cad/fusion/params.csv` is the single source of truth.** Script `00` creates a new Hybrid design. Each later script deletes and rebuilds its own component, so reruns are safe. |
| Material | **PETG by default.** ASA for parts exposed to sun or outdoors. PA-CF only for hooks and towers, and only if the team has a hardened nozzle, an enclosure and a dryer. PLA is for fit coupons only. |

The only open items are component sizes the user will supply. They are listed in §11.

---

## 1. Research summary and print-vs-buy rules

### 1.1 Materials
- **PETG** is the default for brackets, links and press-fit features. It is tougher than PLA, creeps less, and needs no enclosure. Sources: 3dprinting.com engineering-filament guide; Wevolver filament guide.
- **ASA** is for parts exposed to UV or heat, such as the sensor nose and battery lid. It is the outdoor replacement for ABS: similar printing but UV-stable (amfg.ai ASA/PETG/PC comparison). Do not use ABS (warping, fumes).
- **CF-nylon (PA-CF, PAHT-CF)** is the strongest option but highly anisotropic. PAHT CF15 is about 103 MPa in the XY plane and about 18 MPa in Z. It needs a hardened nozzle, an enclosure and dry storage (qidi3d CF article). Use it only for hooks and towers, and always orient the load path in the layer plane.
- **PLA** creeps under sustained load and softens at about 55–60 °C. Use it for test coupons only.

### 1.2 Fits
Measured values vary by printer, so print the coupon from script `05_fit_coupon.py` first and tune the `clr_*` parameters.
- **Press fits:** 0–0.05 mm on Bambu-class printers ([CCI Arts wiki](https://wiki.cci.arts.ac.uk/books/digital-fabrication-lab/page/design-the-clearance-and-tolerance)).
- **Slip/clearance fits:** 0.15–0.2 mm ([Sovol](https://www.sovol3d.com/blogs/news/fdm-3d-printing-tolerances-clearances-how-to-design-parts-that-fit), [Snapmaker](https://www.snapmaker.com/blog/3d-printing-tolerances)).
- **PLA** cracks when press-fitted tightly.
- **Plan defaults (per side):** `clr_slip` 0.20 mm, `clr_snug` 0.10 mm, `clr_press` 0.00 mm plus 0.3 mm crush ribs on bearing seats.

### 1.3 Heat-set inserts
Use heat-set inserts on every clamped or re-assembled joint. Never thread screws into bare plastic.
- **Blind holes:** make them insert length + 1 mm deep, with no chamfer ([CNC Kitchen tips](https://www.cnckitchen.com/blog/tipps-amp-tricks-fr-gewindeeinstze-im-3d-druck-3awey)).
- **Wall:** at least 1.6 mm around the hole ([DigiKey FreeCAD tutorial](https://www.digikey.com/es/maker/tutorials/2025/intro-to-freecad-part-4-how-to-use-heat-set-inserts)). This plan uses ≥ 2.0 mm.
- **Interference:** about 0.2–0.4 mm on the knurl OD ([UW ME design lab](https://medesignlab.me.wisc.edu/?p=162)).
- **M3 hole diameter:** sources conflict (3.9 mm vs 4.5 mm). Default is `ins_m3_d` = 4.0 mm. Confirm it with the coupon.

### 1.4 Bed size
`bed_max` = 220 mm by default; set it to 256 for Bambu X1/P1. Any printed body larger than this is split with a tongue-and-groove joint plus 2× M3 bolts into inserts, or with a lap joint and Ø3 pegs at `clr_snug`. The ACM shell (240 mm) is always split into two halves.

### 1.5 Servos and gearmotors
- **DS3218:** 21.5 kg·cm at 6.8 V (18 at 5 V), 0.14 s/60°, 40 × 20 × 40.5 mm, 60 g (manuals.plus DIYmall DS3218 manual).
- **MG996R:** 11 kg·cm at 6 V, 0.14 s/60°, stall 2.5 A ([Cornell MG996R datasheet](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2017/kmv48_rp625/kmv48_rp625/kmv48_rp625/MG996RTowerPro.pdf), [components101](https://components101.com/node/467)). Dynamic torque is about 6 kg·cm. Rule: **continuous load ≤ 50 % of stall.**
- **Pololu 37D, 12 V** ([category page](https://www.pololu.com/category/271/12v-37d-metal-gearmotors), [#4743](https://www.pololu.com/product/4743/specs), [#4753](https://www.pololu.com/product/4753/specs)):
  - 30:1 = 330 rpm, 14 kg·cm stall, 5.5 A stall. 50:1 = 200 rpm, 21 kg·cm.
  - Gearbox continuous limit is about **10 kg·cm**. Output shaft is 6 mm D, 16 mm long.
  - Body length with encoder: 68 mm (30:1) or 70 mm (50:1).
- **Crane rule:** rated load ≤ 75 % of tipping load on wheels/crawlers without outriggers, and 85 % on outriggers ([OSHA 1910.180](https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.180), [Cal. T8 §4923](https://dir.ca.gov/title8/4923.html), [Manitowoc B30.5 chart](https://www.manitowoc.com/media/1705/download)). The rover has no outriggers, so use **75 %**.
- **Soil:** Bekker plate sinkage p = (k_c/b + k_φ)·zⁿ, with Mohr–Coulomb shear τ = c + σ·tanφ (Wong, *Theory of Ground Vehicles*). Use the REGOLITH constants already in `config.js`.

### 1.6 Print vs buy

| Buy | Print (PETG unless noted) |
|---|---|
| Gearmotors, encoders, bearings (608-2RS), shafts and pins, 2020 extrusion, deck sheet, servos, metal 25T servo horns, electronics, ToF sensors, switches, fasteners and inserts, trunnion rod (Ø12 mm 6061), battery | Motor mounts, idler hub carriers, rocker/bogie joint blocks, frame corner brackets, battery tray, electronics tray, sensor nose (ASA), LiDAR mast (ASA), arm towers, link plates, servo cradles, hooks, latch gates, stow cradles (TPU pads), deck pins and latch, ACM shell, ACM feet and bushings, fit coupon |

**Print settings** (0.4 mm nozzle, 0.2 mm layers; 0.16 mm for fit-critical small parts):

| Class | Walls | Top/bottom | Infill | Orientation |
|---|---|---|---|---|
| Structural (towers, mounts, joint blocks) | 5–6 | 5 | 50–60 % gyroid | Load plane parallel to bed |
| Link plates | 5 | 5 | 50 % | Flat |
| Hooks and latch gates | 6 | 6 | 100 % | Flat, so the hook profile is in the layer plane |
| Trays and brackets | 4 | 4 | 30–40 % | Flat |
| Covers, ACM shell | 3 | 4 | 15 % | Open side up |

Extra rule: any insert boss gets at least 4 walls, so the region around the insert is solid.

---

## 2. Bill of materials (prototype rover + ACM)

Prices are approximate 2026 USD. Items marked "existing" are already in `public/team/js/rover_bom.js`.

| # | Item | Qty | Role | Est. $ |
|---|---|---|---|---|
| 1 | Pololu 37D 30:1 12 V gearmotor + 64 CPR encoder (#4752 `[VERIFY SKU]`) | 4 | FL/FR/RL/RR drive | 4×52 |
| 2 | Pololu 6 mm universal aluminium hub | 4 | Wheel to D-shaft | 2×8 |
| 3 | 608-2RS bearings (pack of 20) | 1 | Idlers (2×2), rocker/bogie pivots (8), spares | 10 |
| 4 | 8 mm steel shaft 300 mm (or M8 bolts) | 2 | Idler axles, pivot pins | 12 |
| 5 | Wheels 130 × 75 mm (existing) | 6 | 4 driven, 2 idler | — |
| 6 | Cytron MDD10A dual 10 A driver (3.3 V logic OK) | 2 | One channel per driven motor | 2×25 |
| 7 | 12.8 V 10 Ah LiFePO4 + BMS (existing), 30 A fuse, switch, XT60 | 1 | Main power | — |
| 8 | Latching E-stop mushroom button | 1 | Cuts the motor and servo rails | 10 |
| 9 | Raspberry Pi 5, BNO085, YDLIDAR X2L ×2, OV5647 cams (existing) | — | High-level control and sensing | — |
| 10 | **ESP32-S3 DevKitC-1** (replaces the Mega for motion: PCNT ×4 encoders, MCPWM, I²C) | 1 | Motion MCU with 250 ms watchdog | 15 |
| 11 | PCA9685 16-ch servo driver (OE pin wired to the watchdog) | 1 | 9 servos | 8 |
| 12 | UBEC-A 20 A adjustable, set to 7.4 V | 1 | Shoulder and elbow servos | 25 |
| 13 | UBEC-B 10 A, set to 6.0 V | 1 | Wrist, latch and deck-latch servos | 15 |
| 14 | INA226 current/voltage monitor | 2 | Measures both servo rails (validates the sim) | 2×5 |
| 15 | **60 kg·cm-class HV servo** (e.g. DS5160-type, about 65 × 30 × 48 mm, 160 g `[VERIFY]`) | 4 + 1 spare | Shoulders, elbows | 5×30 |
| 16 | MG996R (or DS3218) | 3 + 1 spare | Wrists, deck latch | 4×10 |
| 17 | MG90S | 2 + 1 spare | Hook latch gates | 3×4 |
| 18 | 25T aluminium disc servo horns | 6 | Link interfaces | 6×4 |
| 19 | VL53L1X ToF breakouts (XSHUT address reassign) | 2 | Range and yaw to the ACM face | 2×15 |
| 20 | Subminiature roller-lever microswitches | 7 | Hook seated ×2, latch closed ×2, deck latch ×1, ACM seated ×2 | 7×1 |
| 21 | Heat-set inserts: M3×5.7 (100), M2.5 (50), M4 (50) | — | All clamped joints | 30 |
| 22 | Stainless SHCS M2.5/M3/M4 kit, nyloc nuts, washers | 1 | Fasteners | 20 |
| 23 | 2020 V-slot extrusion, 3 m total, plus corner brackets and T-nuts | 1 set | Chassis frame | 40 |
| 24 | 3 mm aluminium-composite or 4 mm plywood deck, 600 × 604 mm | 1 | Deck | 20 |
| 25 | Wire: 16 AWG silicone, 22 AWG, servo leads 300 mm ×9, XT30/JST | 1 set | Harness | 25 |
| 26 | PETG 2 kg; ASA 1 kg; TPU 0.25 kg | — | Prints | 70 |
| 27 | Ø12 mm 6061 rod, 300 mm | 1 | ACM trunnions (2 × 40 mm) | 8 |
| 28 | Steel washers or ballast shot, about 0.6 kg | 1 | ACM ballast to 1.0 kg, CG set by `acm_cg_y` | 5 |
| 29 | Printed AprilTag (36h11, 60 mm) | 1 | ACM fiducial | — |

Estimated new spend is about $800. The prototype mass budget is in §3.1: 8.30 kg curb with arms, 9.30 kg loaded.

---

## 3. Weight distribution and CG analysis

### 3.1 Why "everything at the back" is only partly right
- AEGIS uses a **rocker-bogie** (`public/team/js/rocker_bogie.js` GEO).
  - The rocker pivot is at x = 0.
  - The bogie pivot is at x = **−120 mm**, midway between the middle and rear wheels. The front wheel is at +240 mm.
- In pitch, the rocker therefore rests on two points: the front wheel and the bogie pivot. The bogie then splits its load 50/50 between the middle and rear wheels:
  ```
  front share  f = (x_cg − x_B)/(x_F − x_B) = (x_cg + 0.120)/0.360
  middle share = rear share = (1 − f)/2
  driven share (outer wheels only) = f + (1 − f)/2
  ```
- The **effective rear support is the bogie pivot at −120 mm, not the rear axle at −240 mm.** If x_cg < −120 mm, the front wheels lift (a wheelie) until the rocker stops engage at 0.21 rad.
- With only the outer wheels driven, unloading the front also removes half of the driven wheels.
- Slopes move the CG: Δx = −h·tanθ uphill. Loaded h ≈ 155 mm (prototype) and 2.88 m (full scale).

**Prototype mass table** (kg, x in mm). U = user's all-rear layout. R = recommended layout.

| Item | m | x (U) | x (R) |
|---|---|---|---|
| Chassis, deck, tub | 2.00 | 0 | 0 |
| Rocker-bogie + 6 wheels | 1.50 | 0 | 0 |
| 4 drive gearmotors (symmetric ±240) | 0.84 | 0 | 0 |
| 2 idler hubs | 0.12 | 0 | 0 |
| Battery | 1.65 | −200 | **−80** (low in tub) |
| Electronics | 0.45 | −180 | **+50** |
| Sensors (nose LiDAR, cams, rear mast) | 0.40 | +220 | +220 |
| Arm fixed parts (2 × servo + tower) | 0.44 | +150 | +150 |
| Arm moving parts (2 × 0.40), stowed / lifting | 0.80 | +60 / +359 | +60 / +359 |
| Deck guides and latch | 0.10 | −180 | −120 |
| **Curb total** | **8.30** | x = −27.4 | x = +9.7 |
| ACM on deck (place point) | 1.00 | −180 | **−120** |

### 3.2 Prototype results

| Case | U: x_cg / front / driven | R: x_cg / front / driven |
|---|---|---|
| Empty, stowed, flat | −27 mm / 25.7 % / 62.9 % | +10 mm / 36.0 % / 68.0 % |
| Loaded, flat | −44 / 21.2 % / 60.6 % | −4 / 32.2 % / 66.1 % |
| **Loaded, 20° uphill** | **−100 / 5.5 % (near wheelie)** / 52.7 % | −61 / 16.5 % / 58.2 % |
| Front lift, 1 kg ACM at x = 420 | tip ratio **9.1 %** (tipping at 11.0 kg) | tip ratio **10.8 %** (tipping at 9.3 kg) |
| Arm reach to place point | Δx = 330 mm (over the 270 design reach: +22 % shoulder torque) | Δx = 270 mm (within design) |

Tip ratio = overturning moment ÷ restoring moment about the front contact line (x = 240):
- Restoring = Σm(0.24 − x).
- Overturning = m_ACM·(0.42 − 0.24) = 0.18 kg·m.
- Example (R): 0.18 / (8.30 × (0.24 − 0.0385)) = 0.108.

**Verdict:**
- Rear mass does increase the forward-lift margin (9.1 % vs 10.8 % of tipping), but the lift does not need that margin.
- With cargo also at the back, a 20° climb leaves 5.5 % of the weight on the front driven wheels. That is effectively 2-wheel drive with dragging idlers, one bump from a wheelie.
- Option R keeps the loaded CG at x ≈ 0 (one-third on each axle) and the arm reach inside its design value.
- **Design rule:** choose `dk_place_x` so the loaded CG sits at x = 0 ± 0.02·wheelbase. Then check 20° uphill front share ≥ 15 % and lift tip ratio ≤ 0.75.

### 3.3 Full scale (sim)

Inputs:
- Rover 8,000 kg at comLocal x = 0.
- Arm fixed 2 × 210 kg at x = 2.45 m.
- Arm moving 2 × 430 kg at 0.98 m stowed, 5.85 m lifting.
- Deck guides 150 kg.
- ACM 2,400 kg.
- Balance rule gives `DECK.placeX` = −(1029 + 843)/(150 + 2400) = **−0.734 m**. The sim computes this at init. It differs from the prototype's −120 mm because the mass ratios differ.

| Case | Result |
|---|---|
| Empty, stowed | x = +0.187 m, front 36.5 % |
| Loaded, flat | x ≈ 0, front 33.3 % |
| Loaded, 15° / 20° uphill | front 20.2 % / 15.5 % |
| Front lift (ACM at x = 6.855 m) | restoring 30,987 kg·m; overturning 7,051 → **tip ratio 22.8 %**; tipping payload 10,547 kg; 75 %-rated 7,910 kg |
| Lift on a 10° nose-down slope | tip ratio 28.3 % |
| Same layout with place at −2.94 m (U-style) | loaded flat front 25.3 %; 20° uphill **7.4 %** |

### 3.4 Arms cannot take the existing cargo

| Cargo | Mass | CG ahead of front axle | Overturning | Tip ratio |
|---|---|---|---|---|
| cyl40 | 14,940 kg | 5.45 m | 81,423 kg·m | **263 %** (tips) |
| box20 (crosswise) | 8,500 kg | 2.69 m | 22,865 kg·m | **74 %**, and 8,500 > 7,910 rated |

pod4 (5,900 kg) is above the 3 t structural rating. All three keep the straddle pickup.

---

## 4. Arm design and torque calculations

### 4.1 Geometry (prototype mm; ×K for full scale)
- Shoulder axis S = (150, 275, ±175). L1 = L2 = 200, so the envelope is 0–400 mm.
- **Pick:** trunnion at (420, 90), Δ = (270, −185), d = 327 mm (82 % of reach).
- **Place:** trunnion at (−120, 225 + 90 = 315), Δ = (−270, +40), d = 273 mm.
- **Apex via-point:** (150, 515), straight above the shoulder.
- **Stow:** hook in a stow cradle at (50, 305). Joint angles about q = [2.35, −2.55, 1.2] rad.
- **Trunnion position:**
  ```
  p = S + L1·[cos q1, sin q1] + L2·[cos(q1+q2), sin(q1+q2)]
  hook angle ψ = q1 + q2 + q3
  ```
- **IK, elbow-up:**
  ```
  D  = (d² − L1² − L2²)/(2·L1·L2)
  q2 = −acos(D)
  q1 = atan2(dy, dx) + atan2(L2·sin|q2|, L1 + L2·cos q2)
  q3 = ψ_des − q1 − q2
  ```
- **Lateral layout:** the arm plane at z = ±175 has its inner face at ±155. The ACM spans ±120, trunnion tips reach ±140, and hook fingers reach inward. The front wheel inner face is at 197.5 mm: `99_verify` must report no interference.
- **Swept-volume keep-out:** z ∈ [−150, 150], x ∈ [−210, 300], from deck + 10 mm up. Nothing may be mounted there: no crossbar, no mast. The front LiDAR goes low in the nose (y 150). The rear LiDAR goes on a mast at x −270, top at y 385.

### 4.2 Mass assumptions (prototype, per arm)

| Part | Mass |
|---|---|
| L1 twin plates | 0.06 kg, CG at mid-length |
| Elbow servo (60 kg-class) + cradle | 0.18 kg |
| L2 plates | 0.05 kg |
| Wrist MG996R + bracket | 0.07 kg |
| Hook + MG90S latch | 0.04 kg |
| Payload share | 0.50 kg (half of a 1.0 kg ACM), at the trunnion, which coincides with the wrist axis |

### 4.3 Torques

Formula:
```
τ_static = g·Σ m_k·x_k   (x_k = horizontal lever arm)
dynamic factor 1.25
continuous rule: τ_dyn ≤ 0.5 × stall
```

Design pose: maximum horizontal reach r = 270 mm, elbow at 135 mm.

| Joint | Prototype (g = 9.81) | Full scale (g = 1.62, ×K, 1200 kg/arm) |
|---|---|---|
| Shoulder | Σm·x = 0.00405 + 0.0243 + 0.0101 + 0.61×0.27 = 0.2032 kg·m → **1.99 N·m static (20.3 kg·cm)**, 2.49 N·m dynamic. 60 kg-class at 7.4 V (≈55 kg·cm = 5.39 N·m): **46 %** ✓. DS3218 would be 118 % ✗. | Σm·x = 132 + 242 + 396 + 1280×4.41 = 6415 kg·m → **10.39 kN·m static**, 12.99 dynamic. Stall 26 kN·m (50 %). |
| Elbow | 0.05×0.10 + 0.61×0.20 = 0.127 kg·m → 1.25 N·m (12.7 kg·cm), 15.9 kg·cm dynamic. 60 kg-class: **29 %** ✓. DS3235 at 6 V would be 51 % ✗. | 196 + 1280×3.26 = 4369 kg·m → 7.08 kN·m static, 8.85 dynamic. Stall 18 kN·m (49 %). |
| Wrist | Payload sits on the wrist axis; only the hook offset and trunnion friction (μ·N·r = 0.3×4.9×0.006 = 0.009 N·m) remain: **< 0.5 kg·cm**. MG996R: < 5 %. | Stall 1.2 kN·m |
| Latch gate | 2 g bump: 4.9 N × 15 mm = 0.074 N·m. MG90S 2.2 kg·cm: **34 %**. Make the gate over-centre so it cannot be back-driven. | Stall 300 N·m |

- **Link stress (prototype):** two plates 5 × 28 mm with the shoulder moment of 2.0 N·m gives σ = M·c/I ≈ 2.1 MPa. PETG is far stronger, so stiffness and backlash govern the design, not strength.
- **Speeds:**
  - Prototype operating rate 0.6 rad/s.
  - Full scale `rateMax` 0.12 rad/s and `accMax` 0.05 rad/s². Hardware ω0 = 0.35 rad/s.
  - Full pick cycle ≈ 25 s of swing plus approach.
- **Pendulum and input shaping:** the CG sits d = 20 mm below the trunnion (×K = 0.33 m).
  - Full-scale period T = 2π√(d/g) = 2.84 s. Prototype on Earth: 0.28 s.
  - Without shaping, 0.2 m/s² of trunnion acceleration swings the load about 7°.
  - Apply a **ZV input shaper** to the trajectory: impulses 0.5 at t = 0 and 0.5 at T/2.
- **Power:**
  - **Full scale, per pick:** about 2400×1.62×5.8 m + 860×1.62×3 m ≈ 27 kJ of mechanical work. At η = 0.6 that is 45 kJ ≈ 12.5 Wh, plus 120 W hotel × 120 s = 4 Wh, so **≈ 17 Wh**. This is negligible against 420 kWh.
  - **Full-scale peak:** τ·ω/η ≈ 2.1 kW per shoulder. Cap with `ARM.peakW` = 6 kW; the planner scales rates down to fit.
  - **Prototype:** about 0.06 Wh per pick. Peak ≈ 7.4 V × 12 A. Holding draws about 11 W per shoulder, because hobby servos have no brake.

---

## 5. Physics models (equations)

### 5.1 DC motor (`src/sim/actuators.js` → `DCMotor`)
Specified in datasheet form: V_nom, τ_s (stall, output), ω_0 (no-load, output), I_s, I_0, I_lim, η_back, R_th, C_th, T_max. Optional tables `TsV` / `w0V` for non-linear voltage scaling.

```
R   = V_nom / I_s
k   = (V_nom − I_0·R)/ω_0                        (output-referred back-EMF, V·s/rad)
η   = τ_s / (k·(I_s − I_0))                       (calibrated so stall matches the datasheet)
I   = clamp((u·V_bus − k·ω)/R, −I_lim, I_lim)     (quasi-static; L/R ≪ dt)
τ   = η·k·I − η·k·I_0·sign(ω)                     (Coulomb no-load loss)
        back-driven (τ·ω < 0): τ /= η_back
P_el = u·V_bus·I    (prototype: regen clipped to 0; full scale: 50 % recovery)
T_w' = (I²R − (T_w − T_amb)/R_th)/C_th            derate linearly above T_max
V_bus = V_src − R_src·ΣI                          (rail sag shared by all actuators on the rail)
```

Check: 37D 30:1 at 12 V gives R = 2.18 Ω and k = 0.334 V·s/rad output. Stall returns 1.37 N·m at 5.5 A, and no-load speed is 34.6 rad/s.

### 5.2 Servo (`Servo` extends `DCMotor` and adds the internal loop, potentiometer and backlash)

```
θ_ref slews at ≤ rateMax (arm planner), then goes through the ZV shaper
e  = θ_ref − θ_pot;   e = 0 if |e| < deadband (0.1° ≈ 1 µs)
u  = clamp(Kp·e + Ki·∫e − Kd·ω_g, −1, 1)
     Kp = 1/θ_sat, θ_sat = 5°  [ASSUMED: typical digital servo saturates within ±5°]
     Full scale: u = (PID + gravity feed-forward G(q)) / τ_s
Gear stage:  J_g·ω̇_g = τ_motor(u, ω_g) − τ_t
Backlash:    τ_t = k_s·dz(θ_g − q, b/2) + c_s·(ω_g − q̇)·[engaged]
             dz(x, h) = x − h (x > h) | x + h (x < −h) | 0
             k_s = J_eff·(2π·f_s)²; f_s = 12 Hz prototype, 4 Hz full scale
             b = 0.5° prototype, 0.1° full scale
Holding:     a back-driven servo resists up to holdFactor·τ_s (1.3) [ASSUMED]
Full-scale joints have power-off brakes: engaged when |θ̇| < 1e-3 for 0.5 s → zero holding current.
```

### 5.3 Arm dynamics (`src/sim/arm.js`)
- Each arm is a planar 2-link model (q1, q2). The wrist q3 is a rate-limited first-order servo, because its load is negligible.
- Integrated with `ARM.substeps` = 8 (dt = 1/960 s), semi-implicit Euler.
- Equation of motion: `M(q)·q̈ + C(q,q̇) + G(q) = τ_t + J_hᵀ·F_hook`.
- Masses: m1 at lc1, m_e at L1, m2 at lc2, m_t (wrist + hook) at L2.

```
M11 = I1 + I2 + m1·lc1² + m_e·L1² + m2·(L1² + lc2² + 2·L1·lc2·c2) + m_t·(L1² + L2² + 2·L1·L2·c2)
M12 = I2 + m2·(lc2² + L1·lc2·c2) + m_t·(L2² + L1·L2·c2)
M22 = I2 + m2·lc2² + m_t·L2²
h   = −(m2·L1·lc2 + m_t·L1·L2)·s2
C1  = h·(2·q̇1·q̇2 + q̇2²)      C2 = −h·q̇1²
G   = −Σ_k J_kᵀ·(m_k·g_b)
g_b = R_bodyᵀ·(0, −g, 0) − a_body      (non-inertial body frame)
```

**Reaction on the rover.** The Rapier rover body carries only the arms' fixed mass (`sprungMass` += 2 × `fixedMassPerArm`). Moving links and the payload are applied as a wrench at each shoulder via `applyImpulseAtPoint`, plus `applyTorqueImpulse` for −τ_shoulder about the body z-axis:
```
F_rover = Σ m_k·(g_b − a_k,rel) + F_ACM→arm
```

### 5.4 Hook, grip and ACM coupling (three modes)
The trunnion axis sits at the same in-plane point for both arms. The hook is a spring-damper from the wrist point to the trunnion:
```
F_i = k_h·(p_w,i − p_t) + c_h·(v_w,i − v_t)
```
- k_h = m_share·(2π·f_h)²
  - Full scale: f_h = 3 Hz → k_h 4.3e5 N/m, c_h 2.27e4 N·s/m.
  - Prototype: f_h = 10 Hz → k_h ≈ 1974 N/m.
- Load sharing between the arms emerges from this. If one arm lags, it carries more load.
- Twist torque (F_L − F_R) × z-offset is reported; flag it if it exceeds `ACM.twistMax`.

**Seat check:**
- V-seat half-angle α = 45°.
- If the hook load direction leaves the seat cone (angle > α + atan μ) and the latch is open → **drop event**.
- If the latch is closed, the latch carries the outward component up to `latchMaxN`; beyond that → drop.
- Friction torque on the trunnion: τ_f = μ·|F|·r_t. This damps the pendulum.

**Modes:**
1. **GROUND.**
   - The ACM is a dynamic Rapier body; feet forces come from §5.5.
   - Hook forces are applied as impulses at the trunnion world points, and their reactions feed into the arm.
   - Go to CARRY when Σ foot N < 2 % W for 0.3 s and clearance > `pick.clearance`.
2. **CARRY.**
   - The ACM switches to `kinematicPositionBased`, driven by an analytic planar state: trunnion point p_t plus pendulum angle φ.
   - Equations:
     ```
     m·p̈_t = ΣF_i + m·g_b
     I_t·φ̈ = −m·d·[(g + ÿ_t)·sin φ + ẍ_t·cos φ] − τ_f
     I_t   = I_c + m·d²
     ```
   - Rocks and terrain still see the kinematic body (group `G_ACM_ARM` = all except ROVER).
3. **DECK.**
   - The ACM is descending, its bushings are within pin capture (radial ≤ `DECK.capture` = k(0.006)), and its bottom is ≤ deck + ε.
   - It then blends to the exact seated pose over 0.3 s with critical damping (the tapered pins centre it physically).
   - Then `_attach(c)`, the existing fixed impulse joint.
   - If outside capture → `PIN_TOP` fault: lift 20 mm and retry once.

### 5.5 ACM–regolith contact (`src/sim/cargoContact.js` + `terramechanics.js` helpers)
- The ACM box collider does **not** collide with TERRAIN. Four feet carry it, mirroring how the wheels are custom force elements. Use `heightAt(sim.terrain, x, z)` from `terrain.js` and `soilAt` for the soil parameters.
- Full-scale foot: k(0.03) = 0.49 m square, A = 0.24 m².

```
d   = h_terrain − y_foot                                       (penetration)
Loading (d ≥ z_p):    N = A·(k_c/b + k_φ)·dⁿ;  z_p = d          (plastic sinkage memory)
Unloading (d < z_p):  N = max(0, A·p(z_p) − K_u·(z_p − d)), K_u = 10 × secant stiffness  [ASSUMED]
Damping:              N += c_s·ḋ, ζ = 0.7 (only while N > 0)
Creep / settling:     ż_p = (1.15·z_eq − z_p)/τ_set, τ_set = 120 s  [ASSUMED]
Adhesion / breakout:  N ≥ −c_adh·A, c_adh = 0.3·c (≈ 50 Pa)  [ASSUMED]
Shear:                bristle anchor, k_b = 3e5 N/m (same as the parked brake)
                      |F_t| ≤ c·A + N·tanφ   (Mohr–Coulomb), else the anchor slides
```

Check: W/4 = 972 N → p = 4050 Pa → z = 4050 / (1400/0.49 + 820000) = **4.9 mm**, settling to 5.7 mm. New helpers in `terramechanics.js`: `plateSinkage(p, b, soil)` and `plateLoad(z, b, A, soil)`.

### 5.6 Stability during the lift
- `_stability(Q, com, dt)` adds `this.arm.massProps()`: moving links plus a suspended ACM at their world CoM. It still produces `tilt.margin` / `marginF` (force-angle).
- New method `arm.tipRatio()`:
  ```
  n̂ = horizontal unit normal of the FL–FR contact line, pointing outward
  ℓ_i = n̂·(r_i − p_line)
  ratio = Σ_{ℓ>0} m_i·ℓ_i / Σ_{ℓ<0} m_i·|ℓ_i|
  ```
  Use the **measured** hook load after breakout and the manifest mass × 1.25 before.
- Governor:
  - Pre-lift refusal if predicted ratio > 0.75 or `marginF` < 8°.
  - During motion: stop and reverse to set-down.
- Also report `axleShare()` = front-wheel `w.load` ÷ total.

### 5.7 Drive: 4 driven wheels, 2 idlers (`rover.js` wheel loop, about lines 247–520)
- `w.driven = DRIVE.driven.includes(name)` is set in the constructor.
- **Passive wheel:**
  - Tm = 0 and the integrator is reset (treated like the `failed` path at lines 351–366).
  - It is **not** braked: change the brake conditions at 366, 416, 428–431 and 463 to `(parkBrake && w.driven) || seized`.
  - Add bearing drag to the spin solver's `need` torque: `−sign(ω)·min(|·|, DRIVE.idlerDragNm) − DRIVE.idlerViscous·ω`.
- Power: `_motorPower(0, ω)` is already 0, so driveW comes only from the driven wheels. No double counting at lines 369 and 485.
- Heat: idlers produce no copper loss, because Tm = 0.
- Rails mode (lines 923–935): divide `Fdrive` and power by the number of driven, healthy wheels, not `this.wheels.length`.
- Traction check:
  - Driven wheels carry 2/3 of the weight at x_cg = 0.
  - Parking hold: tan θ ≤ tanφ × 0.667 → about 25° holds on 4 brakes.
  - Total drive torque stays 4 × 8400 = 6 × 5600.
- `NAV.noSteerRatio` and yaw efficiency (healthy 0.4–0.55): re-measure in `tests/drive4.mjs`. The middle wheels sit at the yaw centre, so expect a small change.

### 5.8 Power (`_powerStep`)
- `P.armW` = Σ actuator P_el, plus `ARM.hotelW` (120 W) whenever the arm is not asleep and stowed.
- `P.loadW += P.armW`. Add `armWh` to telemetry.
- Prototype budget: update the `rover_bom.js` 'arm_pick' power mode to 25 W average and 90 W peak.

---

## 6. Sim changes, file by file

### 6.1 `src/sim/config.js` (new blocks; full-scale SI)
```js
export const DRIVE = { driven: ['FL','FR','RL','RR'], idlerDragNm: 15, idlerViscous: 4 }; // drag [ASSUMED]
// ROVER.motor.peakTorque 5600 → 8400; ROVER.motor.brakeTorque 9000 → 13500 (4 motors keep the 6-motor budget)
export const ACT = {
  shoulder: { V:400, Ts:26000, w0:0.35, Is:40, I0:0.8, Ilim:32, backlash:0.0017, fs:4, Rth:0.6, Cth:9000, Tmax:120, brake:true }, // [ASSUMED]
  elbow:    { V:400, Ts:18000, w0:0.35, Is:28, I0:0.6, Ilim:22, backlash:0.0017, fs:4, Rth:0.8, Cth:6000, Tmax:120, brake:true },
  wrist:    { V:400, Ts:1200,  w0:0.6,  Is:4,  I0:0.2, Ilim:3.2, backlash:0.003, fs:6, brake:true },
  latch:    { V:28,  Ts:300,   w0:1.5,  Is:6,  I0:0.3, Ilim:5 },
  proto: {  // Earth bench values used by tests and the calculator
    big60:  { V:7.4,  Ts:5.39, w0:6.0,  Is:5.0, I0:0.25, backlash:0.0087 },  // [VERIFY datasheet]
    ds3218: { V:6.8,  Ts:2.11, w0:7.48, Is:2.5, I0:0.15, TsV:[[5,1.77],[6.8,2.11]] },
    mg996r: { V:6.0,  Ts:1.08, w0:7.48, Is:2.5, I0:0.17 },
    mg90s:  { V:6.0,  Ts:0.216, w0:13.1, Is:0.7, I0:0.1 },
    gm37d30:{ V:12,   Ts:1.37, w0:34.6, Is:5.5, I0:0.2, gearLimitNm:0.98 },
  },
};
export const ARM = {
  enabled:true, arms:2,
  shoulder:{ x:k(0.15), y:k(0.275), z:k(0.175) },          // ± z per side
  L1:k(0.20), L2:k(0.20),
  links:{ m1:120, lc1:k(0.10), I1:110, mElbow:110, m2:120, lc2:k(0.10), I2:110, mWrist:40, mHook:40 },  // [ASSUMED]
  fixedMassPerArm:210,
  limits:{ q1:[-1.05,2.8], q2:[-2.8,0.05], q3:[-2.6,2.6] },
  rateMax:0.12, accMax:0.05, substeps:8,
  stow:{ q:[2.35,-2.55,1.2] }, apex:{ dy:k(0.24) },
  pick:{ xBody:k(0.42), clearance:k(0.02), preBelow:k(0.02), preFwd:k(0.03), seatV:0.02 },
  hook:{ mouth:k(0.030), seatHalf:0.785, mu:0.3, fh:3, zeta:0.5, latchMaxN:2.5e4 },
  ctrl:{ kp:1/0.087, ki:0.2, kd:0.05, deadband:0.0017, gravityFF:true },
  tipRatioMax:0.75, minMarginDeg:8, ratedKg:3000, peakW:6000, hotelW:120,
};
export const ACM_PICK = { standoff:k(0.25), creepV:0.05, latTol:k(0.007), yawTolDeg:3, alongTol:k(0.04),
  retries:2, overloadK:1.6, seatTimeout:20 };
export const DECK = { placeX:null /* computed at init: loaded CG → x = 0 */, capture:k(0.006),
  pinSpan:k(0.14), latchTime:0.6, seatedEps:k(0.002) };
export const ACM_SOIL = { foot:k(0.03), nFeet:4, Ku:10, zeta:0.7, creep:0.15, tauSettle:120, cAdhK:0.3, bristleK:3e5 };
// CARGO_TYPES.acm = { name:'ACM arm-handled cargo module', handling:'arm',
//   dims:[k(0.18), k(0.14), k(0.24)], mass:2400, trunnion:{ r:k(0.006), len:k(0.02), y:k(0.09) },
//   cgY:k(0.07), twistMax:2e4, stand:false }
```
The existing straddle types get `handling: 'straddle'`.

### 6.2 New modules
- **`src/sim/actuators.js`**
  - Classes `DCMotor` and `Servo`; helpers `dz`, `fromSpec(spec)`.
  - Every class is pure, with methods `step(u|θref, ω_load, V_bus, dt)` returning `{ τ, I, P_el, T_w }`.
- **`src/sim/arm.js`**
  - `class ArmSystem`:
    - `constructor(rover, sim)`, `fk(side, q)`, `ik(side, p, ψ)`, `massMatrix()`, `gravity()`, `step(dt)`
    - `plan(waypoints)` (quintic joint segments + ZV shaper + rate/accel/peakW scaling)
    - `deploy()`, `stow()`, `engage(c)`, `latch(on)`, `release()`
    - `massProps()`, `tipRatio(extraMass, pose)`, `powerW`, `stowed`, `payload`, `mode`, `faults`, `telemetry()`
  - Pure function `protoCalc(layout)` for §3 tables, used by tests.
- **`src/sim/cargoContact.js`**
  - `class FootSoil`; `acmSoilStep(c, dt, sim)` applies impulses to the ACM body and stores `c.feet[i].{N, z_p}`.

### 6.3 Edits to existing files
- **`rover.js`**
  - Constructor: `w.driven`, `this.arm = new ArmSystem(this, sim)`, add the arm fixed mass.
  - `step()`: call `this.arm.step(dt)` before the wheel loop; apply the passive-wheel logic (§5.7).
  - `_stability`: add `arm.massProps`.
  - `_powerStep`: add armW.
  - `speedCap()`: 0 while a payload is suspended; 0.1 m/s while not stowed.
  - `loaded`: true when an ACM is latched.
  - `telemetry`: add arm and axleShare.
  - Interlock: an arm pick is refused if `this.lift.cargo` is set, and a straddle pick is refused if an ACM is on the deck.
- **`dynamics.js`**
  - Mass includes the ACM, from `R.lift.cargo` or `R.arm.payload`.
  - Drawbar = Σ_driven H − Σ_all Rc.
- **`world.js`**
  - `spawnACM(x, z, yaw, opts)` creates a dynamic body with a cuboid collider in group `G_ACM` (all except TERRAIN), two trunnion cylinder colliders, mass properties at `cgY`, and `c.feet` state. It is registered in the cargo list with `type: 'acm'`.
- **`sim.js`**
  - Staging: two ACMs near the lander (2,400 kg and 1,200 kg).
  - Call `acmSoilStep` for every ACM in GROUND mode.
- **`autonomy.js`**
  - Add `acquireArm(c, opts)` / `_acquireArm`, mirroring `_acquire`'s step pattern, abort plumbing and logging (§6.4).
  - Add `deliverArm(target)`, the reverse sequence that puts the ACM on a ground pad.
  - `missionPickup` and `missionCycle` branch on `c.type.handling`.
  - `stabilityCheck` uses `arm.tipRatio`.
- **`health.js`**
  - The `mc` fault applies only to driven corners (n ∈ {1, 2, 5, 6}).
  - New `idler_seize`: the idler skids with drag μN; response is to derate 50 % and schedule a dock bearing swap.
  - New `arm_servo` (stall or over-temperature → controlled set-down) and `arm_latch` (switch disagreement → hold and retry).
  - Text change: "5 driven wheels" → "3 driven wheels derated 35 %".
  - Update `docs/FAULT_PROCEDURES.md`.
- **`commands.js`**
  - `arm status|deploy|stow|pick [id]|place <x> <z>|abort`
  - `acm spawn [massKg]`
- **`public/team/js/rover_bom.js`**
  - 4 drive motors and 2 idlers, arm parts, ESP32-S3, UBECs, PCA9685, ToF, ACM mass table, 'arm_pick' power mode.
- **`docs/POWER_BUDGET.md`**: add an arm row.

### 6.4 `acquireArm` state machine (full-scale thresholds)

| # | State | Action | Exit / fail |
|---|---|---|---|
| 1 | NAV_STANDOFF | `navTo` the point `standoff` in front of the ACM face, heading toward it (fiducial / `cargoGuess`) | Arrived |
| 2 | ALIGN | `alignYaw` to ACM yaw. If \|lat\| > latTol, `backUp` and re-approach (skid steer cannot strafe) | \|lat\| ≤ 0.11 m, yaw ≤ 3° (yaw tolerance comes from deck pin capture: pinSpan/2·θ ≤ capture − 2 mm) |
| 3 | CREEP | Creep at 0.05 m/s; ToF pair gives range and yaw = atan((r_L − r_R)/baseline) | ACM centre at pick.xBody ± alongTol → park brake |
| 4 | PRECHECK | Predicted tipRatio (manifest mass × 1.25) ≤ 0.75; marginF ≥ 8°; pitch/roll within limits; arm healthy; deck clear | Fail → abort with reason |
| 5 | DEPLOY | Stow → pre-grasp (trunnion − preBelow·ŷ − preFwd·x̂), hook gate facing up | Reached |
| 6 | ENGAGE | Hooks rise at `seatV` | Seat switch, or hook load > 5 % W/2; timeout → retract, re-align, retry (≤ 2) |
| 7 | LATCH | Close gates (0.4 s) | Latch switches agree, else `arm_latch` |
| 8 | BREAKOUT | Lift at `seatV` to the clearance height; monitor feet N → 0, hook load, tipRatio | Load > overloadK·W → set down and abort; GROUND→CARRY handover |
| 9 | SWING | Shaped quintic through the apex to above the place point | tipRatio/margin governor; swing angle < 5° |
| 10 | PLACE | Descend the last k(0.06) at 0.02 m/s | Seat (load < 20 %) within capture → DECK; else PIN_TOP retry |
| 11 | DECK_LATCH | Deck latch 0.6 s, then `_attach(c)` | Switch confirms |
| 12 | RELEASE | Open gates; hooks move down and out by k(0.03) | — |
| 13 | STOW | Move to stow and engage brakes | `stowed = true` → drive enabled with loaded caps |

`deliverArm` runs steps 13 → 5 in reverse: unlatch the deck, engage, lift, swing forward, lower to the ground pad, release, stow.

### 6.5 Render
- **`src/render/arm.js`** (new):
  - `buildArms(root)` builds both arms in **prototype metres** directly under `AegisModel.root`, which is scaled by K. This makes the CAD numbers and the render geometry identical.
  - Parts: tower, dark servo housings, twin plates, hook (`THREE.Shape` V-seat extrusion with cheeks), latch gate, stow cradles, deck pins and latch.
  - `updateArms(group, rover)` poses q1/q2/q3, latch angles and the deck latch from sim state (sim-driven animation, no keyframes).
- **`aegis.js`**: call `buildArms` and `updateArms`; give idler hubs grey caps and driven hubs orange.
- **`props.js`**: `buildACM()` with trunnions (gold), feet, bushings, keeper and an AprilTag face.
- **`main.js` cargoVis**: an ACM case that poses the mesh from the Rapier body, including sinkage.
- **UI telemetry panel** (grep `margin` in `src/ui`), arm block:
  - mode, q in degrees, per-joint % of stall bars, hook loads, tipRatio bar (red > 0.75), axleShare, latch states, armW.
  - Optional translucent swept-volume ghost in the engineering overlay.

### 6.6 Tests (register in `tests/run-all.mjs`)

| Test | Quick or long | Assertions |
|---|---|---|
| `actuators.mjs` | quick | 37D: stall 1.37 N·m ± 2 % at 5.5 A; no-load 34.6 rad/s ± 2 %. DS3218 60° step ≤ 0.20 s at 6.8 V and slower at 5 V (TsV). Backlash hysteresis = b ± 5 %. Saturation: I ≤ Ilim. Rail sag ΔV = ΣI·R_src. |
| `arm.mjs` | quick | FK∘IK error < 1e-9. Prototype shoulder G at the r = 0.27 pose = 1.99 N·m ± 1 % (Earth); full scale 10.39 kN·m ± 1 % (Moon). Every planned-trajectory pose has τ/τ_s ≤ 0.5 at rated load. `protoCalc` reproduces §3.2: tip ratio R 0.108 ± 0.005, U 0.091; U uphill front share 0.055 ± 0.01. Short flat-ground `acquireArm` succeeds. |
| `acm_soil.mjs` | quick | Foot sinkage 4.9 mm ± 10 %, 5.7 mm ± 10 % after 600 s. Peak breakout hook force ≤ 1.05 W. Sliding at c·A + N·tanφ. |
| `drive4.mjs` | quick | ML/MR Tm ≡ 0 and never braked. Reaches cruise on flat. Climbs 15° loaded. Parks on 20° with < 0.05 m creep in 60 s. A driven-wheel `mc` fault leaves 3 driven. Idler seize works. Measures yaw efficiency (update `NAV.noSteerRatio` if outside 0.4–0.55). |
| `cad_params.mjs` | quick | Parses `cad/fusion/params.csv`; asserts arm_L1, arm_L2, arm_sh_*, acm_* × K equal the ARM/CARGO_TYPES.acm values. |
| `arm_pickup.mjs` | long | Full mission: pickup in < 240 s sim, latched pose error < capture, min marginF > 8°, max tipRatio < 0.30, energy < 30 Wh, then deliverArm. Misalignment of 0.4 m → retry succeeds. A 9 t ACM is refused pre-lift. cyl40 via arm is refused. |
| `faults.mjs` / `fdir.mjs` (edit) | — | Move `R.wheels[2]` → `R.wheels[0]` and 'wheel 3' / 'mc:3' → 'wheel 1' / 'mc:1'. The seize test stays on `R.wheels[4]` (RL, driven); fix the "5 driven" strings. |

After the drive change, re-baseline long tests whose speeds or power shift (v17_suite, rock_climb, nav_bench, power_dock, cycle). Record the old → new numbers in CHANGES.

### 6.7 CHANGES.md §33 (draft) and version
```
## §33  v1.8.0: arm-handled cargo (ACM), outer-wheel drive, actuator physics
- Drive: FL/FR/RL/RR driven; ML/MR passive idlers (no torque, no brake, 15 N·m + 4 N·m·s/rad drag [ASSUMED]).
  peakTorque 5600→8400, brakeTorque 9000→13500 N·m (4 motors keep the 6-motor traction/park budget).
  mc faults only on driven corners; idler_seize added; faults/fdir tests moved wheel 3 → wheel 1.
- actuators.js: DC motor (torque-speed, back-EMF, current limit, thermal, rail sag) and servo
  (P/PID loop, deadband, backlash two-mass, saturation, holding); datasheet-form specs in ACT.
- arm.js: two 3-DOF arms, planar Lagrangian dynamics at 960 Hz, hook spring coupling, pendulum payload,
  ZV-shaped quintic trajectories, rover reaction wrench, tip-ratio governor (≤ 0.75, ASME B30.5-style).
- ACM cargo type (2,400 kg, 2.94 × 2.28 × 3.92 m, trunnions), Bekker/Mohr–Coulomb foot contact with
  plastic sinkage, creep settling and breakout adhesion.
- Autonomy acquireArm / deliverArm (13-state machine); straddle pickup unchanged for cyl40/box20/pod4.
- Deck pins + latch; DECK.placeX from the loaded-CG balance rule (−0.734 m).
- Stability includes arm links + suspended ACM; axleShare telemetry; power armW + 120 W hotel.
- Render: animated arms, hooks, latches, deck latch, ACM; idler hubs grey.
- Measured: <fill from tests: yaw efficiency, climb, pickup time, energy, min margin>.
```
Bump `package.json` to **1.8.0**, and grep `1.7.0` across `src/` and `index.html` for any displayed version strings.

---

## 7. Fusion 360 CAD plan (`cad/fusion/`)

### 7.1 Layout
```
cad/fusion/
  params.csv              single source of truth (Name,Unit,Expression,Value,Comments,Favorite)
  lib/aegis_cad.py        shared helpers (imported via sys.path + importlib.reload)
  tools/fusion_mcp.py     copy of the MCP client (the scratchpad copy is temporary)
  00_new_design.py   01_params.py   05_fit_coupon.py
  10_chassis.py      20_drive_pods.py  25_wheels.py  30_rocker_bogie.py
  40_power_electronics.py  45_sensors.py
  50_arm.py          60_hook_latch.py  70_deck_guides.py  80_acm.py
  90_assembly_joints.py  95_pose.py  98_export.py  99_verify.py
  out/shots/  out/stl/   (generated)
  README.md
```

**Run a step:**
```
python3 cad/fusion/tools/fusion_mcp.py script cad/fusion/NN_x.py
```
Then take screenshots:
```
python3 cad/fusion/tools/fusion_mcp.py shot cad/fusion/out/shots/NN_x_iso.png iso-top-right
```
Also take `front` and `top`. The agent opens each PNG with Read and checks the step's expectations.

Fusion executes scripts one at a time. Agents may *write* scripts in parallel, but **only one agent runs Fusion at a time**.

### 7.2 Script conventions
- Each script starts with `CAD_DIR = r'<repo>/cad/fusion'` because `__file__` is unavailable when the MCP sends code text. It then calls `sys.path.insert(0, CAD_DIR + '/lib')`, `import aegis_cad as A` and `importlib.reload(A)`.
- Entry point is `def run(_context: str):`. **No try/except.** Use `print()` for results.
- Check `app.activeDocument.name.startswith('AEGIS_PROTO')`, otherwise print an error and return.
- **Idempotent:** `A.fresh_component(parent, 'ASM_ARM_L')` deletes any existing occurrence of that name and rebuilds it.
- **Naming:**
  - `PRT_` printed body, `BUY_` bought placeholder (translucent blue), `REF_` reference-only (swept volume).
  - Components `ASM_<MODULE>`. Joints `J_<SHOULDER_L>`.
- **Parameter prefixes:** `fit_/clr_`, `ins_`, `bolt_`, `ch_`, `wh_`, `mot_`, `brg_`, `rb_`, `bat_`, `ele_`, `sen_`, `srv_big_`, `srv_std_`, `srv_mic_`, `arm_`, `hook_`, `dk_`, `acm_`, `bed_`.
- **Appearances:** copy into the design first (`design.appearances.addByCopy(lib.appearances.itemByName(...), name)`), then assign.
  - Driven pods orange; idlers grey; arms safety-yellow; `BUY_` translucent blue; ACM white with gold trunnions; deck pins red.
- **Mirroring:** occurrence transforms cannot be mirrored (negative determinant), so `build_arm(side=±1)` draws each side with z × side.

### 7.3 Fusion API gotchas
(C) = confirmed by probe or docs this session; (V) = verify with `fusion_mcp_read` before relying on it.
1. (C) Internal length unit is **cm**. `ValueInput.createByReal(1)` = 1 cm or 1 rad. Always use `ValueInput.createByString('arm_L1')` or `'200 mm'` so parameters stay linked.
2. (C) `userParameters.add(name, ValueInput, units, comment)`. `01_params.py` **upserts**: `itemByName(n)` → set `.expression`, else `add`. Unitless values use `''`. Masses and densities are stored as unitless numbers with "(kg)" in the comment.
3. (C) `importUserParameters` / `exportUserParameters` exist (CSV header as in 7.1). Use export only, for a backup snapshot, because upsert is deterministic.
4. (C) `app.documents.add(DocumentTypes.FusionDesignDocumentType)` cannot be called inside command events. A plain script is fine.
5. (C) Set `design.designType = ParametricDesignType`. Set `design.designIntent` to `HybridDesignIntentType`, because a Part intent cannot hold child components.
6. (C) `rootComponent.occurrences.addNewComponent(Matrix3D.create())` returns an Occurrence; use `.component`.
7. (C) `asBuiltJoints.createInput(occ1, occ2, JointGeometry.createByPoint(pt))`, then `setAsRevoluteJointMotion(JointDirections.ZAxisJointDirection)`. Pass geometry = None for rigid joints.
8. (C) `setAllExtent` is retired. Use `setOneSideExtent(ThroughAllExtentDefinition.create(), ExtentDirections.PositiveExtentDirection)`.
9. (C) Pockets: `combineFeatures.createInput(target, ObjectCollection[tool])`, then `operation = CutFeatureOperation` and `isKeepToolBodies = False`.
10. (C) A script fails while a command dialog is open. Do not save the document unless the user asks.
11. (V) Sketches stay parametric only if dimensions are added and bound, e.g. `sketchDimensions.addDistanceDimension(...)` then `.parameter.expression = 'arm_L1'`. Coordinates alone are frozen numbers. Helpers `rect_param`, `circle_param` and `slot_param` do this. Set `sketch.isComputeDeferred = True` while building.
12. (V) Feature creation inside an occurrence uses `occ.component.features` and sketches. References to another occurrence's geometry need `createForAssemblyContext(occ)` proxies.
13. (V) `revoluteJointMotion.rotationValue` is in radians; used by `95_pose.py`.
14. (V) `design.createInterferenceInput(ObjectCollection)` with `design.analyzeInterference(input)`. Fallback: an oriented bounding-box overlap test.
15. (V) `exportManager.createSTLExportOptions(body, path)` and `exportManager.execute(opts)`.
16. (V) Up-axis is a user preference. Scripts use the sim frame (X fwd, Y up, Z right). If Fusion is Z-up, screenshots look rotated but the geometry is correct.

### 7.4 Script content and checks

| Script | Builds | Screenshot / print check |
|---|---|---|
| 00 | New document `AEGIS_PROTO_v1_8`; Parametric + Hybrid; empty ASM_ skeleton | prints doc name, 9 components |
| 01 | Upsert all rows of `params.csv` | prints "N params, 0 errors" |
| 05 | `PRT_FIT_COUPON`: pegs and holes at clr 0 / 0.05 / 0.10 / 0.15 / 0.20 / 0.25 / 0.30; insert holes 3.8–4.4 mm; 608 seat with crush ribs | Print it first; write the chosen values back to the CSV |
| 10 | `BUY_EXT_2020` rails (2 × 600, 3 × 560), `BUY_DECK` sheet with insert/bolt pattern, `PRT_CORNER_BRKT` ×8, tub | Deck top at `ch_deckY` |
| 20 | `PRT_MOTOR_MOUNT` ×4 (Ø`mot_d`+2·clr_slip clamp bore, 6 × M3 on `mot_bc_d`, 2 × M4 inserts to the rocker); `BUY_MOTOR_37D` ×4; `PRT_IDLER_HUB` ×2 + `BUY_608` ×4 | 4 orange pods at the outer corners, 2 grey idlers |
| 25 | `WHEEL_DRIVEN` (Pololu hub pattern), `WHEEL_IDLER` (608 seats); 6 occurrences | Wheel centres match `ROVER.corners`/K |
| 30 | Rocker and bogie links from GEO params (rb_P, rb_B), `PRT_JOINT_BLOCK` with bearing seats, differential bar placeholder | Rocker pivot above the middle wheel; bogie pivot at x −120 |
| 40 | `PRT_BATTERY_TRAY` (pocket bat_L/W/H + 2·clr_slip, strap slots) at `bat_x`; `PRT_ELEC_TRAY` with M2.5 standoffs for Pi 5, ESP32-S3 cradle, MDD10A ×2, PCA9685, UBEC ×2, INA226 ×2, IMU; matching `BUY_` placeholders | Battery low at −80 |
| 45 | `PRT_SENSOR_NOSE` (ASA; LiDAR low at y 150, cam), `PRT_LIDAR_MAST_REAR` (x −270) | Nothing inside `REF_SWEEP` |
| 50 | `build_arm(±1)`: `PRT_TOWER` with srv_big pocket and M3 inserts, `PRT_UPPER_PLATE` ×2 (25T horn bolt circle), `PRT_ELBOW_CRADLE`, `PRT_FORE_PLATE` ×2, `PRT_WRIST_BRKT` (srv_std pocket), ToF mount; `BUY_SERVO_*` placeholders | Arms at z ±175; stow pose |
| 60 | `PRT_HOOK` (V-seat `hook_seat_ang`, mouth `hook_mouth`, 30° cheeks, seat-switch pocket), `PRT_LATCH_GATE` + MG90S pocket, over-centre stop | Hook mouth faces up in pre-grasp |
| 70 | `PRT_PIN_ROUND`, `PRT_PIN_DIAMOND` (at `dk_place_x` ± `dk_pin_span`/2), `PRT_DECK_LATCH` + MG996R pocket, `PRT_STOW_CRADLE` ×2 (TPU pad), ACM-seated switch pockets, `REF_SWEEP` | Pins red; sweep ghost visible |
| 80 | `PRT_ACM_SHELL_A/B` (split, tongue-and-groove + M3 inserts), `BUY_TRUNNION` ×2 (Ø12), `PRT_FOOT` ×4, `PRT_BUSH_ROUND`, `PRT_BUSH_SLOT`, `BUY_KEEPER`, `PRT_BALLAST_TRAY` at `acm_cg_y`, fiducial plate | ACM at the ground pick pose |
| 90 | Rigid as-built joints between modules; revolutes for wheels ×6, rockers, bogies, shoulder/elbow/wrist ×2, latches, deck latch, all with limits from params | Joint count printed (expect 21 revolute) |
| 95 | `POSE = 'stow'|'pregrasp'|'pick'|'apex'|'place'`: IK in Python (same formulas as §4.1) → `rotationValue` | One iso + one side shot per pose, to confirm reach and clearance by eye |
| 98 | STL per `PRT_` body to `out/stl/`; `params_export.csv` | prints file list |
| 99 | Checks: every `PRT_` body fits the bed (sorted dims a ≥ b ≥ c: (b, c ≤ bed and a ≤ bed_z) or (a, b ≤ bed)); interference at all 5 poses (arm vs wheel, chassis, sensors); mass = volume × `rho_petg` × `infill_eff` per component; assembly CG vs §3.1 (±10 mm); naming; insert count | prints PASS/FAIL lines |

### 7.5 Parameter table (`params.csv`; mm unless stated; `[VERIFY]` = user supplies)

| Name | Default | Unit | Meaning |
|---|---|---|---|
| bed_max | 220 | mm | Usable bed X/Y (256 for Bambu) |
| bed_z | 250 | mm | Usable height |
| clr_slip | 0.20 | mm | Per-side slip-fit clearance |
| clr_snug | 0.10 | mm | Per-side locating fit |
| clr_press | 0.00 | mm | Per-side press fit (plus crush ribs) |
| crush_rib | 0.30 | mm | Rib height in press bores |
| wall_min | 2.0 | mm | Minimum wall around holes and inserts |
| ins_m3_d / ins_m3_depth / ins_m3_boss | 4.0 / 6.7 / 8.0 | mm | M3×5.7 insert hole, depth, boss OD |
| ins_m25_d / ins_m25_depth | 3.6 / 5.0 | mm | M2.5 insert |
| ins_m4_d / ins_m4_depth | 5.6 / 9.1 | mm | M4 insert `[VERIFY]` |
| bolt_m3_clr / bolt_m4_clr / bolt_m25_clr | 3.4 / 4.5 / 2.9 | mm | Clearance holes |
| ch_halfLen / ch_halfWid / ch_deckY | 300 / 302 / 225 | mm | Frame (= ROVER.frame / K) |
| ch_tubZ / ch_tubY0 / ch_deck_t / ext_size | 146 / 65 / 4 / 20 | mm | Tub half-width, tub floor, deck sheet, extrusion |
| wb_x / tr_z | 240 / 235 | mm | Front/rear axle x, track half-width |
| wh_r / wh_w | 65 / 75 | mm | Wheel radius, width `[VERIFY]` |
| mot_d / mot_L / mot_shaft_d / mot_shaft_L | 37 / 68 / 6 / 16 | mm | 37D gearmotor with encoder |
| mot_bc_d / mot_holes / mot_collar_d | 31 / 6 / 12 | mm, – , mm | Face bolt circle, count, collar `[VERIFY]` |
| brg_od / brg_id / brg_w | 22 / 8 / 7 | mm | 608 bearing |
| rb_P_x / rb_P_y / rb_B_x / rb_B_y | 0 / 194 / −120 / 125 | mm | Rocker and bogie pivots (GEO) |
| rb_tube | 12 | mm | Link section |
| bat_L / bat_W / bat_H / bat_x / bat_y | 155 / 100 / 100 / −80 / 70 | mm | Battery envelope and position `[VERIFY]` |
| ele_x | 50 | mm | Electronics tray centre |
| pi_L / pi_W / pi_hdx / pi_hdy | 85 / 56 / 58 / 49 | mm | Pi 5 + holes |
| esp_L / esp_W | 69 / 26 | mm | ESP32-S3 DevKitC-1 `[VERIFY]` |
| mdd_L / mdd_W / mdd_H | 84 / 62 / 16 | mm | MDD10A `[VERIFY]` |
| pca_L / pca_W | 62.5 / 25.4 | mm | PCA9685 |
| ubec_L / ubec_W / ubec_H | 50 / 27 / 13 | mm | UBEC `[VERIFY]` |
| imu_L / imu_W | 25.4 / 20.3 | mm | BNO085 breakout `[VERIFY]` |
| lidar_d / lidar_h / sen_nose_y / sen_mast_x | 70 / 50 / 150 / −270 | mm | LiDAR placeholder and mounts `[VERIFY]` |
| tof_L / tof_W | 13 / 18 | mm | VL53L1X breakout |
| srv_big_L / W / H | 65.5 / 30 / 48 | mm | 60 kg-class body `[VERIFY]` |
| srv_big_flangeL / srv_big_flange_z | 82 / 36 | mm | Ear span, ear height from bottom `[VERIFY]` |
| srv_big_holeX / srv_big_holeY / srv_big_shaft_off | 74 / 20 / 15 | mm | Ear holes, spline offset from end `[VERIFY]` |
| srv_std_L / W / H | 40.7 / 19.7 / 42.9 | mm | MG996R / DS3218 body |
| srv_std_flangeL / srv_std_flange_z / srv_std_holeX / srv_std_holeY / srv_std_shaft_off | 54 / 28 / 49.5 / 10 / 10 | mm | Standard servo ears and spline `[VERIFY]` |
| srv_mic_L / W / H / holeX | 22.8 / 12.2 / 28.5 / 27.8 | mm | MG90S |
| horn_bc / horn_holes | 14 / 4 | mm, – | 25T disc horn bolt circle `[VERIFY]` |
| arm_sh_x / arm_sh_y / arm_z | 150 / 275 / 175 | mm | Shoulder axis (= ARM.shoulder / K) |
| arm_L1 / arm_L2 | 200 / 200 | mm | Link lengths |
| arm_plate_t / arm_plate_h | 5 / 28 | mm | Link plate section |
| arm_plate_gap | srv_big_W + 2*clr_slip | mm | Twin-plate gap |
| hook_mouth / hook_seat_ang / hook_cheek_ang | 30 / 90 / 30 | mm, deg, deg | Hook capture |
| latch_gap | 1.0 | mm | Gate to trunnion |
| trun_d / trun_len | 12 / 20 | mm | ACM trunnion |
| dk_place_x | −120 | mm | ACM centre on deck (balance rule) |
| dk_pin_tip / dk_pin_base / dk_pin_h / dk_pin_span | 8 / 14 / 15 / 140 | mm | Locating pins |
| dk_bush_d | dk_pin_base + 2*clr_slip | mm | ACM bushings |
| acm_L / acm_W / acm_H | 180 / 240 / 140 | mm | ACM x / z / y size `[VERIFY with user]` |
| acm_wall / acm_trun_y / acm_cg_y | 2.4 / 90 / 70 | mm | Shell wall, trunnion height, CG height (ballast) |
| acm_foot / acm_foot_h | 30 / 5 | mm | Foot pad |
| acm_mass | 1.0 | (kg) | Ballasted mass |
| fid_size | 60 | mm | AprilTag |
| sweep_halfW / sweep_x0 / sweep_x1 | 150 / −210 / 300 | mm | Keep-out zone |
| rho_petg / infill_eff | 1.27 / 0.55 | (g/cc), – | Mass estimate |

---

## 8. Phased implementation for Sonnet agents

Each phase lists its files. **A file has exactly one owner per phase.** Run `npm run test:quick` at the end of every sim phase. Each sim phase adds its own CHANGES.md bullet under a WIP §33.

| Phase | Agents (parallel) | Owns | Depends on | Done when |
|---|---|---|---|---|
| **P0** constants | 1 | `src/sim/config.js` (DRIVE, ACT, ARM, ACM_PICK, DECK, ACM_SOIL, CARGO_TYPES.acm, motor torque/brake) | — | Quick tests unchanged except intended drive diffs; constants tagged |
| **P1a** actuators | 1 | `src/sim/actuators.js`, `tests/actuators.mjs` | P0 | actuators test passes |
| **P1b** drive change | 1 | `src/sim/rover.js` (wheel loop, rails mode, brake conditions), `src/sim/dynamics.js`, `src/sim/health.js`, `tests/drive4.mjs`, `tests/faults.mjs`, `tests/fdir.mjs`, `docs/FAULT_PROCEDURES.md` | P0 | drive4 + quick pass; long suite re-baselined |
| **P1c** soil + ACM body | 1 | `src/sim/cargoContact.js`, `src/sim/terramechanics.js` (helpers only), `src/sim/world.js` (spawnACM), `src/render/props.js` (buildACM), `tests/acm_soil.mjs` | P0 | acm_soil passes |
| **P1d** CAD core | 1 (sole Fusion operator) | `cad/fusion/**` scripts 00, 01, 05, 10, 20, 25, 30, 40, 45; `params.csv`; `lib/`; `tools/` | — | Screenshots in `out/shots`; 99 partial PASS |
| **P2a** arm physics | 1 | `src/sim/arm.js`, `tests/arm.mjs`; `rover.js` integration (constructor, step call, `_stability`, `_powerStep`, `speedCap`, `loaded`, `telemetry`) after P1b merges | P1a, P1b, P1c | arm test passes |
| **P2b** CAD arm/deck/ACM | same Fusion operator | `cad/fusion` 50, 60, 70, 80 | P1d | Pose shots at stow/pick/place |
| **P3a** autonomy | 1 | `src/sim/autonomy.js`, `src/sim/commands.js`, `src/sim/sim.js` (staging + acmSoilStep), `tests/arm_pickup.mjs` | P2a | arm_pickup (long) passes |
| **P3b** render/UI | 1 | `src/render/arm.js`, `src/render/aegis.js`, `src/main.js`, `src/ui/*` telemetry block | P2a (state shape only) | Visual check via the run skill: arms animate through a full pick |
| **P3c** CAD finish | Fusion operator | `cad/fusion` 90, 95, 98, 99; `tests/cad_params.mjs`; `cad/fusion/README.md` | P2b | 99 all PASS; STLs exported |
| **P4** integration | 1 | `CHANGES.md` §33, `package.json` 1.8.0, `docs/POWER_BUDGET.md`, `public/team/js/rover_bom.js`, this doc's "measured" values | All | `npm run test:quick` and `npm run test:long` green |

**Conflict rules:**
- `rover.js` is edited by P1b first, then by P2a. No two agents touch it at the same time.
- `config.js` is frozen after P0. Later constant tweaks go through the P4 agent, or a single owner per request.

---

## 9. Risks and mitigations
- **Hobby-servo backlash and pot noise** cause hook placement error (±3 mm at 400 mm). Mitigations: hook mouth 30 mm, cheeks, ToF/fiducial alignment, and pin capture ±6 mm. In the sim this is modelled through the backlash `b`.
- **Two-arm fight under position mismatch.** Hook spring compliance plus a 1 mm latch gap keeps it bounded. The sim reports the twist torque.
- **Rapier stiffness.** Arm sub-stepping is internal to `arm.js`. The ACM is kinematic while carried, and deck seating is analytic to avoid thin-pin contact jitter.
- **Long-test drift from the drive change.** Re-baseline once in P1b and document it in CHANGES.

## 10. Firmware notes (prototype, outside this repo)
- The ESP32-S3 runs the motion layer: 4 wheel PI loops on PCNT encoders, servo commands via PCA9685, switches, ToF.
- **250 ms heartbeat watchdog:** drives PCA9685 OE high and the MDD10A PWM to 0. Frames are `0xAA 0x55` + CRC16, per CLAUDE.md.
- Arm commands from the Pi are joint-space waypoints. Shaping and rate limiting run on the ESP32, using the same equations as `arm.js`.

## 11. Open items (sizes the user will supply)
1. Exact 60 kg-class servo model and its body, ear and spline dimensions (`srv_big_*`).
2. Final wheel, motor and battery choices and sizes (`wh_*`, `mot_*`, `bat_*`).
3. Board outlines and holes (Pi 5 stays; MDD10A, UBEC, IMU, LiDAR exact sizes).
4. Printer bed size (`bed_max`, `bed_z`).
5. ACM size and mass for the demo (`acm_L/W/H`, `acm_mass`; defaults 180 × 240 × 140 mm, 1.0 kg).

