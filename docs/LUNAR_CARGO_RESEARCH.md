# Lunar & Planetary Cargo Logistics — Engineering Deep-Dive & AEGIS-V1 Comparative Study

**NASA HUNCH SFT 2026-27 · LLASO Project 2 · LLASO-P2-XPORT-2026**
**Document:** `docs/LUNAR_CARGO_RESEARCH.md`
**Author:** Antigravity Autonomous Systems Engineering
**Date:** November 2026

---

## 1. Executive Summary

Transporting a 40-foot landed pressurized cargo module (14.9 metric tons) over 2.0 km of unprepared lunar south polar terrain without crew EVA is an unprecedented engineering challenge. This report conducts a comprehensive cross-agency analysis of real, historical, and conceptual lunar and planetary cargo-handling systems across NASA, ESA, CNSA, ISRO, and Roscosmos. We evaluate their pickup architectures, terramechanics, dust mitigation, and thermal-survival strategies against our **AEGIS-V1** rocker-bogie skid-steer transporter.

Finally, we address the core question: **Is our current design better than what existing agencies use, and what specific improvements should we adopt?**

---

## 2. Survey of Global Lunar & Planetary Cargo Systems

### 2.1 NASA Systems

#### A. NASA ATHLETE (All-Terrain Hex-Limbed Extra-Terrestrial Explorer)
* **Agency / Center:** NASA JPL
* **Architecture:** 6-limbed hexapod with driven wheels at the end of each multi-DOF limb.
* **Payload Handling:** Walks or rolls straddling cargo pallets. The limbs can lower the central chassis down to ground level, engage mechanical hardpoint grapples, and raise the payload into a high-slung belly cradle.
* **Mobility:** Rolling mode (up to 10 km/h on flat terrain) and walking mode (stepping over extreme boulders, ledges, or loose crater rims where wheels slip).
* **Strengths:** Unrivaled terrain clearance and obstacle handling (can climb 35°+ inclines); inherent 6-DOF load leveling and multi-point jacking.
* **Weaknesses:** Extreme mechanical complexity (~36 active actuators for 6 limbs); high mass overhead; seal vulnerability across dozens of rotary joints.

#### B. NASA LSC (Lunar Surface Cargo) / LSMS (Lunar Surface Manipulation System)
* **Agency / Center:** NASA Langley Research Center (LaRC)
* **Architecture:** Lightweight cable-actuated or articulated truss crane mounted either on the lander deck or on a mobile carrier.
* **Payload Handling:** Top-down vertical offload from high lander decks (e.g., Starship HLS 20+ m deck height) using spreader beams and tag-lines.
* **Strengths:** Solves vertical lowering from landers without requiring the ground rover to drive up ramps.
* **Weaknesses:** Cable swing in lunar vacuum/low-g; high tipping moments; wind-free but pendulum-prone oscillations.

#### C. NASA VIPER (Volatiles Investigating Polar Exploration Rover)
* **Architecture:** 4-wheel active suspension (wheel-walking) rover.
* **Mobility / Regolith:** Uses independent steer/drive and active lift struts to perform peristaltic "wheel walking" to escape deep, low-bearing-capacity regolith sinkage.
* **Thermal Architecture:** "Warm box" vault insulated by Multi-Layer Insulation (MLI), loop heat pipes (LHPs), and top-mounted louvers.

#### D. NASA RASSOR / LANCE
* **Architecture:** Counter-rotating bucket drum excavators for lunar in-situ resource utilization (ISRU).
* **Significance:** Demonstrates zero-net-reaction excavation and low-gravity bulk material transport.

---

### 2.2 International Space Agency Systems

#### E. CNSA Chang'e 5 / 6 & Yutu-2
* **Sample / Cargo Transfer:** 4-DOF robotic arm with dual end-effectors (scoop + core drill). Transferred sealed sample canisters into the ascent module docking cone using optical alignment targets and multi-point mechanical latching.
* **Thermal Survival:** Yutu-2 survived >60 lunar nights (14-day darkness at ~90 K) using Radioisotope Heater Units (RHUs) + closing solar panels over the warm bay to retain thermal mass.

#### F. ESA EL3 (European Large Logistics Lander)
* **Payload Handling:** 1.5-ton cargo delivery to the lunar surface. Studied automated forklift-style tines and roll-off ramps for small cargo rovers.
* **Concepts:** Standardized ISO-like intermodal cargo containers with corner-casting locking pins.

#### G. USSR / Roscosmos Lunokhod 1 & 2
* **Mobility:** 8-wheel independent drive with wire mesh tyres and titanium chevron cleats.
* **Survival:** Sealed pressurized "bathtub" vessel heated by a Polonium-210 radioisotope thermal source during the 354-hour lunar night, with a top lid that closed at sunset.
* **Key Lesson:** Traversed 39 km and 10.5 km on the Moon; proved that skid/differential multi-wheel drive has superior redundancy when single wheels jam.

#### H. ISRO Chandrayaan-3 (Pragyan Rover)
* **Deployment:** Two-segment ramp with wire-pulley deployment from the Vikram lander.
* **Mobility:** 6-wheel rocker-bogie passive suspension with wheel-cleat soil shear teeth.

---

## 3. Technology Comparison Matrix

| Feature | NASA ATHLETE | NASA VIPER | CNSA Chang'e / Yutu | ESA EL3 Concepts | **AEGIS-V1 (Our System)** |
|---|---|---|---|---|---|
| **Chassis / Suspension** | 6 multi-joint limbs | 4-wheel active strut | 6-wheel rocker-bogie | 4-6 wheel conventional | **6-wheel rocker-bogie + roller-screw EMA struts** |
| **Cargo Interface** | Straddle belly latch | Internal instrument bay | Direct robotic latch | Roll-off / Forklift tines | **Dual ring clamps + V-saddles on lugs** |
| **Loaded Mass Capacity** | ~10–15 tons | ~450 kg (total rover) | ~100 kg (lander mounted) | ~1.5 tons | **14.94 t module (23 t gross vehicle mass)** |
| **Traverse Range** | ~10 km | ~20 km | ~2 km | ~5–10 km | **2 km continuous transit (10+ round trips)** |
| **Lunar Night Survival** | Thermal blankets (demo) | Warm box + louvers | RHU (Radioisotope) | Fuel cells / Li-ion | **420 kWh pack + 1.5 MWh dock bank + keep-alive array** |
| **Dust Mitigation** | Booted joints | Rotary labyrinth seals | Sealed bay + brushes | Labyrinth + bellows | **Metal-bellows strut seals + wheel fenders + deflector skirts** |

---

## 4. In-Depth Comparative Analysis: Is AEGIS-V1 Better?

### 4.1 Cargo Pickup & Interface: Ring-Clamp vs. Straddle vs. Crane
* **How others do it:** ATHLETE straddles cargo; cranes lift from overhead; forklifts insert tines.
* **AEGIS-V1 approach:** Drive directly under the module along its cylindrical axis while in a **kneel stance** (-0.65 m), position under the cradle bands, activate **roller-screw EMAs to rise** (+0.82 m), lifting the trunnion lugs cleanly out of the stand saddles, then close dual circumferential ring-jaw clamps.
* **Is AEGIS-V1 better?**
  * **YES, for 15-ton cylindrical modules.** An overhead crane requires massive outriggers and risks pendulum oscillations in vacuum. Forklifts create intolerable cantilever moments that would tip a 1/6 g rover. Driving *under* the center-of-mass and lifting vertically keeps the vertical thrust aligned through the rocker-bogie footprint, preserving tip margins (>25°).
  * **Where we can improve:** Current stand clearance is tight (~8 cm margin). Adopting a slight chamfer guide on the stand notches and widening our CASS ultrasonic/laser guide tolerance will make insertion foolproof.

### 4.2 Suspension: Rocker-Bogie + Roller-Screw EMA vs. Walking Legs
* **How others do it:** ATHLETE's walking limbs are versatile but require 36 motors and heavy planetary gearheads. VIPER uses 4 active steer/lift actuators.
* **AEGIS-V1 approach:** Classic 6-wheel **rocker-bogie** differential mechanism for passive obstacle climbing (equalizes ground reaction forces over boulders up to 0.57 m) combined with **active roller-screw electromechanical actuators (EMAs)** on each bogie station for ride-height adjustment and active roll-leveling.
* **Is AEGIS-V1 better?**
  * **YES.** Passive rocker-bogie kinematics consume zero power during open-field cruising, whereas walking limbs require continuous motor torque. Roller screws have very high lead stiffness, zero hydraulic fluid leak risk in hard vacuum ($10^{-12}$ Torr), and use hermetic welded metal bellows seals.
  * **Where we can improve:** Transition rates. Real planetary roller screws operate smoothly at 0.05–0.10 m/s with S-curve acceleration. Our sim's instant or overly rapid visual kneeling causes wheel sinking bugs and abrupt force spikes.

### 4.3 Drive & Steering: Skid-Steer vs. Coordinated Ackermann
* **How others do it:** Curiosity, Perseverance, and LRV use 4-wheel independent steering (Ackermann). Lunokhod used 8-wheel skid-steer.
* **AEGIS-V1 approach:** 6-wheel skid steer (differential wheel speed, zero steering knuckles).
* **Is AEGIS-V1 better?**
  * **Skid-steer eliminates 6 steering actuators and knuckles** (the primary failure mode for Apollo LRV and Mars rovers in abrasive regolith).
  * **Trade-off:** Skid-steer induces severe regolith scrubbing, high lateral shear resistance, and heading delay when carrying 23 tons. That explains the current heading wiggle/jiggle in navigation!
  * **Verdict:** Mechanically superior and more reliable, but requires smarter software: heading dead-bands, pure-pursuit look-ahead $\ge 10\text{ m}$, and yaw-rate damping.

### 4.4 Thermal & Power Architecture: 354-Hour Lunar Night Survival
* **How others do it:** Chang'e/Yutu and Lunokhod rely on nuclear RHUs (decay heat). NASA VIPER avoided the lunar night entirely by staying in polar sunlight zones.
* **AEGIS-V1 approach:** Pure non-nuclear architecture. A central 1.5 MWh stationary power dock battery bank beside the habitat + 4x 120 m² tracking arrays + rover on-board 420 kWh pack. For emergencies away from the dock, an outrigger **keep-alive solar blanket (12 m²)** deploys to trickle-charge the pack above survival thresholds.
* **Is AEGIS-V1 better?**
  * **YES for commercial/NASA HUNCH realism.** Nuclear RHUs face immense regulatory and launch-safety barriers. Non-nuclear high-efficiency battery + high-voltage DC power dock is the modern commercial paradigm (Artemis Base Camp logistics).

---

## 5. Key Recommendations for the AEGIS-V1 Simulator

Based on real lunar mission heritage, we recommend five concrete software and physics updates:

1. **Straight-Line Stand Departure Guidance (from Apollo / Shuttle payload ops):**
   * Do not allow yaw steering commands within 10 m of the cargo stand. Enforce a mandatory straight reverse crawl ($0.2\text{ m/s}$) to ensure the $12\text{ m}$ module completely clears the stand posts before initiating route turns.

2. **Damped Suspension Motion Profiles (from JPL Mars/Lunar Actuator Telemetry):**
   * Replace linear `levelRate` displacement with smooth S-curve interpolation ($3\text{–}5\text{ s}$ total squat duration) and enforce strict ground clamping on wheel visual meshes to eliminate the terrain-sinking artifact.

3. **Pure-Pursuit Lookahead & Scrub-Compensated Steering (from Lunokhod Terramechanics):**
   * Skid-steering a 23-ton loaded vehicle on regolith requires leading the turn. Set lookahead distance to $\max(10\text{ m}, 3.0 \times v)$ and implement a $\pm 3^\circ$ deadband with yaw-rate damping to eliminate heading hunting.

4. **Continuous 360° LiDAR Mapping with Dynamic Cargo Occlusion Masking:**
   * Place the primary LiDAR on the central avionics mast. Dynamically mask rays directed toward the carried cylinder (angles $\pm 60^\circ$ to $\pm 120^\circ$ at module height) to avoid self-mapping phantom obstacles.

5. **Eliminate Pre-Mission Empty Survey Trips:**
   * Remove the mandatory 4 km unladen `surveyRoute()` run from the "Do Full Mission" loop. The rover should immediately transition: **Acquire Cargo $\to$ Loaded Transit $\to$ Port Dock $\to$ Return Staging $\to$ Next Load**.
