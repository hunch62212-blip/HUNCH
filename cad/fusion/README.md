# AEGIS prototype CAD (Autodesk Fusion 360, Python scripts)

Parametric model of the 1:8.16 prototype (K = 8.16, `src/sim/config.js`) from `docs/ARM_CARGO_BUILD_PLAN.md`: chassis, drive pods, wheels, rocker-bogie,
power/electronics, sensors, two 3-DOF arms with V-seat hooks and latch gates, deck guides, and the ACM cargo shell.
Everything is built by scripts from `params.csv`; nothing is hand-modelled.

## Frames and units
Sim frame (what the plan and the simulator use): +X forward, +Y up, +Z starboard, millimetres. Fusion's frame is mapped by
`lib/aegis_cad.py` (`V()`: sim (x, y, z) -> Fusion cm (x, -z, y)). Starboard = R, port = L (each side is drawn, not mirrored).

## Change a size and rebuild
1. Edit the row in `params.csv` (Name, Unit, Expression, Value, Comments, Favorite). Keep Expression and Value consistent
   (`tests/cad_params.mjs` checks this and the derived invariants: `node tests/cad_params.mjs`).
2. Start Fusion, open the design `AEGIS_PROTO_simmatch` (a Save As copy of `AEGIS_PROTO_v1_8`, which stays as the untouched
   original; see `docs/changes/FUSION1.md`; first run: `00_new_design.py`), then push the parameters:
   `python3 tools/fusion_mcp.py script 01_params.py`
3. Re-run the scripts that use the parameter, in this order (each script deletes and recreates its own components):

| Script | Builds | Re-run when you change |
|---|---|---|
| 05_fit_coupon | print-fit coupon (clearances, insert holes) | `clr_*`, `ins_*` |
| 10_chassis | deck, 2020 frame, tub, corner brackets, deck M3 holes | `ch_*`, `rb_P_x` / `rb_P_y`, anything feeding `deck_mount_pts()` (tower bases, `stow_x` / `stow_y` cradles, `dk_place_x` pins and latch) |
| 20_drive_pods, 25_wheels | motors/mounts/idlers, wheels | `mot_*`, `wh_*` |
| 30_rocker_bogie | rocker, bogie, joint blocks | `rb_*`, `jb_*` |
| 40_power_electronics, 45_sensors | battery, boards, LiDAR/cams, `REF_SWEEP` | `bat_*`, `ch_tubY0`, `sen_*`, `sweep_*` |
| 50_arm | towers, upper arm, forearm, servos, horns (both sides) | `arm_*`, `srv_*`, `stow_*` |
| 60_hook_latch | hooks, latch gates, MG90S | `hook_*`, `gate_t`, `latch_gap` |
| 70_deck_guides | round + diamond pins, deck latch, stow cradles | `dk_*`, `stow_x` / `stow_y` |
| 80_acm | ACM shell A/B, trunnions, bushings, feet, ballast | `acm_*`, `trun_*`, `keeper_d` |
| 90_assembly_joints | as-built joints at stow: 21 revolute about the sim axes (6 wheels, 3 arm joints x2, 2 gates, deck latch, rocker x2, differential x2, bogie x2) + rigid ties (tub-mounted parts, rocker / bogie hardware, wheel pods, arm servos); `BUY_TUB` grounded; motion link `ML_DIFFERENTIAL` (left rocker = -right rocker). Prints each revolute's axis, origin offset and sim limits, and any moving-assembly part no joint holds | after any rebuild, and after 95 / 99 / `pose` |
| 95_pose | joint check (drives rocker / differential / bogie / wheel joints, compares wheel centres with the rocker-bogie kinematics: `JOINT CHECK PASS/FAIL`), then cycles the 5 poses, ends at stow | after 90 |
| 96_clean_names | tries to strip ' (1)' suffixes (see Gotchas: Fusion refuses while the name is held by deleted history) | after 90 |
| 98_export | STL per `PRT_` body -> `/tmp/aegis_stl` | - |
| 99_verify | all checks (below); removes the joints (re-run 90 afterwards) | always, last |

Run a script with `python3 tools/fusion_mcp.py script NN_name.py` (the client embeds `lib/*.py` and the CSV because Fusion's
Python cannot read iCloud Drive). A run can exceed the shell timeout (about 2 min); it still completes in Fusion.
Note: 50 then 60 (arms must be rebuilt together since 50 clears `ASM_ARM_R/L`); 80 after changing `trun_*`/`acm_*`.

## Client commands (`tools/fusion_mcp.py`)
`pose stow|pregrasp|pick|apex|place` (moves both arms + ACM; deletes the as-built joints, re-run 90 afterwards if wanted),
`cam ex ey ez tx ty tz` (sim mm, fits to visible geometry), `shot out.png`, `vis hide|show|only|showall <names>`,
`stlcopy` (after 98: `/tmp/aegis_stl` -> `out/stl`, because Fusion cannot write into iCloud).
Shots: `out/shots/` (`95_<pose>_iso|side.png`).

## 99_verify checks
Naming prefixes (PRT_/BUY_/REF_/ASM_/WHEEL_), printer-bed fit for every `PRT_` body (sorted dims a>=b>=c: b,c <= 220 and a <= 250,
or a,b <= 220), mass = volume x `rho_petg` x `infill_eff`, heat-set insert count, keep-out intrusion by non-arm bodies,
arm extent vs `REF_SWEEP` and arm-vs-rest interference at stow / pregrasp / pick / apex / place (Fusion `analyzeInterference`,
threshold 0.02 cm3; trivial self overlaps of stylised mounts are listed as info), and centre of mass vs plan s3 (curb x = +9.7 mm,
loaded about -4 mm, +-10 mm; masses from the plan table, positions from the model).

## Sim sync (arm kinematics v1.8.x, source: `src/sim/config.js` ARM, `src/sim/arm.js`)
Prototype numbers = full-scale / K, K = 8.16 (`config.js` writes them as `k(prototype metres)`; `node` over `fk`, `computeDeckPlaceX`).
Full sim-vs-CAD table and the remaining sim-side items: `docs/changes/FUSION1.md`.
* Limits (rad): q1 [-1.05, 3.0], q2 [-3.0, 2.4], q3 [-2.8, 2.8]. `lib ARM_LIM`; `J_SHOULDER/ELBOW/WRIST_x` limits are these as deltas from stow
  (shoulder -11.5..+220.6 deg, elbow -297.9..+11.5 deg, wrist -91.7..+229.2 deg).
* Stow q = [2.8, -2.8, 1.2] (160.4, -160.4, 68.8 deg), used verbatim (`lib STOW_Q`): elbow (-38.4, 342.0), wrist axis / sim hook point
  (161.56, 342.0) = `stow_x`, `stow_y`, psi 1.2 rad. The CAD hook seat sits `hook_off` = 34 mm further along psi: (173.9, 373.7)
  (`lib stow_seat_xy()`); the stow cradle pin is there. (Matching the seat to (161.56, 342) instead needs q1 = 176.3 deg, outside the sim limit.)
* Pregrasp: seat at the sim preGrasp (`pick_x` - preFwd, `acm_trun_y` - preBelow) = (420 - 30, 90 - 20) = (390, 70) (was (420, 40));
  pick (420, 90); apex (`arm_sh_x`, 515). The sim's prePlace pose is not modelled.
* Place: `dk_place_x` = -42.57 mm (sim DECK.placeX -0.3474 m / 8.16), q2 > 0 branch (`ik2(branch='pos')`); sim q = [1.880, 2.113, 2.290]. The CAD IK
  targets the seat (trunnion) at (-42.57, 315) with the wrist `hook_off` behind it, so CAD place q differs slightly from the sim's tip-based q.
  Pick / apex use the q2 < 0 branch; q3 is wrapped into (-180, 180] deg (was 60..420) so it stays inside the sim limit.
* Deck pins at -42.57 +- 70 mm, deck latch `dk_latch_x` = -148.57 mm. Stow cradle posts stand inboard (d 108..121 mm, hook plate d 126..132) with a
  pin out through the hook seat (the tower and forearm plates fill d >= 132 at that x).
* Suspension joint limits (no sim limits exist; `public/team/js/rocker_bogie.js` GEO): rocker +-12.03 deg (rhoMax 0.21 rad),
  bogie +-16.62 deg (betaMax 0.29 rad).
* `95_pose`/`pose stow|place` reproduce these. `99_verify` at v1.8 (before the sim-match pass): fails = 0, no arm-vs-rest overlap in any of the
  5 poses; CG curb x = +3.9 mm, loaded x = -1.1 mm. Sim-match results: `docs/changes/FUSION1.md`.

## Servo convention and design deviations
* The shaft axis of every servo is its **H** dimension (lateral); L runs along the link, W is in-plane. So the arm plate gap is
  `arm_plate_gap = srv_big_H + 2*clr_slip` (plan said W: corrected). The layer stack (`arm_layers()`) puts the arm at
  |z| 135.7..217 mm, wider than the plan's 155..197.5 budget; it clears wheels and chassis in all 5 poses (99_verify).
* `hook_off = 34`: the hook seat sits 34 mm ahead of the wrist axis (plan: coincident) so the load torque on the wrist stays small.
* `trun_len = 15` (was 20) so the trunnion tip stays clear of the forearm inner plate.
* Wrist angle q3 is wrapped into (-180, 180] deg so every pose stays inside the sim limit [-2.8, 2.8] rad (it used to be unwrapped into 60..420 deg).
* Kept on purpose, listed in `docs/changes/FUSION1.md`: `rb_bogie_z` 159 (sim 157), `trun_len` 15 (sim 20), `hook_off` 34 (the sim's hook point is on the wrist axis), sensor layout, no fender.

## Gotchas (Fusion API)
* iCloud paths are unreadable/unwritable from Fusion's Python; use the client (`script`, `stlcopy`).
* Poses use transform overrides on root-context proxies (`root.allOccurrences`); nested occurrences reject `transform2`, and
  `addExistingComponent` composes with the old occurrence's transform, so do not delete+re-add to move parts.
* Deleted components stay in undo history, so a rebuilt component can be named `PRT_X (1)`; `aegis_cad.bn()` strips it.
  Renaming back (96_clean_names) and reopening the document did NOT clear it (Fusion keeps the base name reserved); 22 arm/wheel components still carry ' (1)'. Cosmetic only; every script matches names through `bn()`.
* `constructionPoints.add` is unsupported in this design type: joints use sketch points.
* `JointGeometry.createByPoint(sketchPoint)` takes its frame from the sketch (Z = sketch normal). The joint sketches lie on the xZ plane
  (normal = Fusion +Y = sim lateral), so lateral revolutes use `ZAxisJointDirection`; the old `YAxisJointDirection` turned all 20 lateral
  joints about the vertical. `lib joint_sim_sign()` gives the sign from a Fusion joint value to the sim angle (+ about sim +z).
* A chained joint (bogie, wheels, elbow, wrist, gate) needs its point in the parent link's component (`lib comp_joint_pt`, sketches
  `J_PTS`); a root-sketch point stays fixed in the root, so the axis would not follow the parent.
* `lib clear_joints()` deletes the motion links before the joints (a motion link holds its joints).
* 30_rocker_bogie builds the shared hardware per side (`PRT_BOGIE_SPACER_R/L`, `BUY_608_P`, `BUY_608_B_R/L`, `BUY_BOGIE_PIN_R/L`) so each
  part can be tied to its own link; it deletes the old shared `PRT_BOGIE_SPACER`, `BUY_608`, `BUY_BOGIE_PIN` in `ASM_SUSPENSION` only
  (the idler `BUY_608` of 20_drive_pods in `ASM_DRIVE` is untouched).
* `importlib.reload` is unsupported; every run starts from fresh modules.
