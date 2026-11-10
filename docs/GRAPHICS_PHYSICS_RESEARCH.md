# Engineering graphics, physics readouts and game UI: research notes

Written 2026-11-09 for the dynamics instruments and engineering overlay (`src/sim/dynamics.js`,
`src/render/engOverlay.js`, `src/ui/instruments.js`). Toggle both with **I** or the **physics** button in
Overlays.

## 1. How engineering projects draw physics

| Practice | Where it comes from | What we built |
|---|---|---|
| Draw contact points, contact normals and contact forces as scaled arrows, refreshed every frame | physics-engine debug views (Havok/Babylon, Cannon/Rapier `debugRender`), collision-debugging guides | normal-load arrow per wheel (length = load / static share, colour = slip), tractive-force arrow along the ground (cyan = drive, amber = brake) |
| Support polygon = convex hull of ground contacts; keep the CoM projection inside it, with a margin | legged / mobile-robot stability (PlaCo CoM-polygon constraint) | support polygon on the ground, coloured by tip margin; the edge the rover would tip about drawn red; CoM sphere with a plumb line |
| Force-angle stability measure: smallest angle the net force through the CoM must turn to cross a tip axis; sensitive to CoM height | Papadopoulos & Rey 1996/2000 (forestry vehicles, mobile manipulators) | already computed in `Rover._stability`; now shown in the ADI, the plan view and the readouts |
| Crane load-moment indicator (LMI): actual vs allowed load as a utilisation bar, warnings before the limit, few displays | Hirschmann / crane RCI-LMI products | stability plan view with a "tip use" bar: 100 % = wheels lift |
| Render x-ray debug geometry over the model | CAD / physics debug views | overlay lines draw without depth test (seen through the body); the polygon fill is depth-tested so it shades the ground only |

## 2. Physics calculations (all in `src/sim/dynamics.js`, each with its formula in the HUD tooltip)

| Quantity | Formula | Notes |
|---|---|---|
| Static stability factor | SSF = T_eff / 2h | NHTSA rollover metric; T_eff = 2 x CoM-to-nearest-side-edge, h = CoM height above the contact plane. AEGIS-V1 empty ~2.0 |
| Rollover / pitch-over tilt | atan(SSF), atan(L_eff / 2h) | static tilt at which the CoM passes the polygon edge; drawn red on the ADI roll scale |
| Force-angle margin | Papadopoulos & Rey | the autonomy governor slows below 25 deg (amber band on the ADI) |
| Lateral acceleration and limit | a_y = v r; a_lim = g SSF | lunar g = 1.62 m/s^2, so a turn tips at 1/6 the Earth lateral acceleration |
| Rollover utilisation | (tan abs(roll) + abs(a_y)/g) / SSF | slope and turn combined; the LMI bar |
| Stopping distance | d = v t_r + v^2 / 2a | t_r 0.5 s, a 0.6 m/s^2 empty / 0.2 loaded (same limits as the autonomy speed governor) |
| Drawbar pull | DP = sum(H) - sum(R_c) | Bekker-Wong soil thrust minus compaction resistance (terramechanics.js) |
| Traction / resistance coefficients | sum(H)/sum(N), sum(R_c)/sum(N) | Wong's lunar wheel work puts flexing resistance near 4 %; we read ~0.045 on flat regolith |
| Slip | s = (r w - v) / max(r w, v), load-weighted | |
| Drive power and efficiency | P_mech = sum(T w); eta = P_mech / P_elec | |
| Specific energy, cost of transport | E/d = P_elec / v; COT = P_elec / (m g v) | COT is our derived figure (the sources give energy per distance, not COT directly) |
| Froude number | Fr = v^2 / (g L) | equal Fr = dynamically similar to the 1:16.3 bench prototype |
| Kinetic energy, grade force | m v^2 / 2; m g sin(pitch) | |

## 3. How games put this in the UI

* **Diegetic / spatial / meta / non-diegetic** (the standard game-UI taxonomy). Dead Space's spine health bar is
  the reference diegetic design; the lesson quoted with it is that functionality wins over immersion. The
  braking bar and path ribbon are *spatial* UI (in the world, not part of the fiction); the instruments are
  non-diegetic, which studies found works best for status that needs precise reading.
* **Flight-sim attitude indicator / KSP navball:** a horizon that rolls and pitches with the vehicle plus a roll
  scale. Community critiques of the navball (markers hidden at the edge, low contrast, hard to read specific
  angles) led us to keep a fixed roll scale with bold red / amber limit bands and a numeric readout under it.
* **Inclinometer HUDs (off-road):** real-time pitch and roll; a Train Sim World complaint shows gradient readouts
  must follow the direction of travel. Our path and braking bar start at the bumper the rover is moving toward.
* **Racing lines and braking cues:** players ask for braking points without a cluttered full line; AR-HUD research
  (TUM) anchors a braking-distance bar on the road. We draw one bar at the stopping point with its distance.

## 4. Graphics findings applied to the bugs

* three.js tone mapping only applies when drawing to the screen, and exposure is a separate knob from the tone
  mapper. The burned-out ground in front of the rover was not tone mapping: the headlights were 900 cd and
  always on, ~7x full sun at 5 m. They now follow E = I / d^2 against the sun's 5.2: 150 cd in shadow, 15 cd
  in sunlight.

## Sources

* [Force-angle measure of tipover stability margin](https://acnpsearch.tweb-dev.unibo.it/singlejournalindex/69730595), [tip-over stability review (MDPI)](https://mdpi.com/2075-1702/11/1/44)
* [PlaCo CoM polygon constraint](https://placo.readthedocs.io/en/0.9.5/_sources/kinematics/com_polygon_constraint.rst.txt), [Havok physics debug visualisation (Babylon forum)](https://forum.babylonjs.com/t/seeking-feedback-on-havok-physics-visualizations-for-debug/53749/8), [debugging collisions with visualisation](https://app.studyraid.com/en/read/15034/519945/debugging-collision-issues-with-visualization)
* [Hirschmann LMI](https://forconstructionpros.com/equipment/crane/product/12126521/hirschmann-automation-and-control-inc-hirschmann-qscale-i2i3-load-moment-indicator), [Crane Briefing on LMI/RCI](https://www.cranebriefing.com/news/lmisrcis-serve-as-digital-hub/2133846.article)
* [RoverDevKit Bekker-Wong toolkit](https://arxiv.org/pdf/2606.21755), [Wong, xTerra lunar wheels](https://kiss.caltech.edu/workshops/xTerra/presentations2/wong.pdf), [rover chassis evaluation tools (DP = H - R)](https://www.academia.edu/69011559/Rover_Chassis_Evaluation_Tools)
* [Game UI lecture (Stony Brook)](https://www3.cs.stonybrook.edu/~tony/ui/lectures/game_design/gameUI.pdf), [diegetic vs HUD study (York U)](https://www.yorku.ca/mack/ec2018.html), [Dead Space UI deep dive](https://www.giantbomb.com/profile/gamer_152/blog/markers-i-a-deep-dive-into-dead-spaces-ui/249377/)
* [KSP navball wiki](https://wiki.kerbalspaceprogram.com/wiki/Navball), [KSP2 UI critique](https://forum.kerbalspaceprogram.com/topic/210240-the-new-ui-for-ksp2-improvements-and-regressions-from-previous-concepts)
* [TUM AR braking-distance HUD](https://www.campar.in.tum.de/pub/toennis2007ismarDrivePath/toennis2007ismarDrivePath.pdf), [Train Sim World gradient HUD issue](https://forums.dovetailgames.com/threads/hud-gradient-units-not-updating-when-changing-direction.26489/)
* [three.js tone mapping overview](https://discourse.threejs.org/t/tone-mapping-overview/75204)
