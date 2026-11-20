# AEGIS prototype CAD (Autodesk Fusion 360, Python scripts)

Parametric model of the 1:16.3 prototype from `docs/ARM_CARGO_BUILD_PLAN.md`: chassis, drive pods, wheels, rocker-bogie,
power/electronics, sensors, two 3-DOF arms with V-seat hooks and latch gates, deck guides, and the ACM cargo shell.
Everything is built by scripts from `params.csv`; nothing is hand-modelled.

## Frames and units
Sim frame (what the plan and the simulator use): +X forward, +Y up, +Z starboard, millimetres. Fusion's frame is mapped by
`lib/aegis_cad.py` (`V()`: sim (x, y, z) -> Fusion cm (x, -z, y)). Starboard = R, port = L (each side is drawn, not mirrored).

## Change a size and rebuild
1. Edit the row in `params.csv` (Name, Unit, Expression, Value, Comments, Favorite). Keep Expression and Value consistent
   (`tests/cad_params.mjs` checks this and the derived invariants: `node tests/cad_params.mjs`).
2. Start Fusion, open the design `AEGIS_PROTO_v1_8` (first run: `00_new_design.py`), then push the parameters:
   `python3 tools/fusion_mcp.py script 01_params.py`
3. Re-run the scripts that use the parameter, in this order (each script deletes and recreates its own components):

| Script | Builds | Re-run when you change |
|---|---|---|
| 05_fit_coupon | print-fit coupon (clearances, insert holes) | `clr_*`, `ins_*` |
| 10_chassis | deck, 2020 frame, tub, corner brackets, deck M3 holes | `ch_*`, anything feeding `deck_mount_pts()` |
| 20_drive_pods, 25_wheels | motors/mounts/idlers, wheels | `mot_*`, `wh_*` |
| 30_rocker_bogie | rocker, bogie, joint blocks | `rb_*`, `jb_*` |
| 40_power_electronics, 45_sensors | battery, boards, LiDAR/cams, `REF_SWEEP` | `sen_*`, `sweep_*` |
| 50_arm | towers, upper arm, forearm, servos, horns (both sides) | `arm_*`, `srv_*`, `stow_*` |
| 60_hook_latch | hooks, latch gates, MG90S | `hook_*`, `gate_t`, `latch_gap` |
| 70_deck_guides | round + diamond pins, deck latch, stow cradles | `dk_*` |
| 80_acm | ACM shell A/B, trunnions, bushings, feet, ballast | `acm_*`, `trun_*`, `keeper_d` |
| 90_assembly_joints | as-built joints (wheels, 3 arm joints x2, gates, deck latch) | after any rebuild |
| 95_pose | cycles the 5 poses, ends at stow | - |
| 98_export | STL per `PRT_` body -> `/tmp/aegis_stl` | - |
| 99_verify | all checks (below) | always, last |

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

## Servo convention and design deviations
* The shaft axis of every servo is its **H** dimension (lateral); L runs along the link, W is in-plane. So the arm plate gap is
  `arm_plate_gap = srv_big_H + 2*clr_slip` (plan said W: corrected). The layer stack (`arm_layers()`) puts the arm at
  |z| 135.7..217 mm, wider than the plan's 155..197.5 budget; it clears wheels and chassis in all 5 poses (99_verify).
* `hook_off = 34`: the hook seat sits 34 mm ahead of the wrist axis (plan: coincident) so the load torque on the wrist stays small.
* `trun_len = 15` (was 20) so the trunnion tip stays clear of the forearm inner plate.
* Wrist angle q3 is unwrapped into 60..420 deg (one servo sweep covers all poses).

## Gotchas (Fusion API)
* iCloud paths are unreadable/unwritable from Fusion's Python; use the client (`script`, `stlcopy`).
* Poses use transform overrides on root-context proxies (`root.allOccurrences`); nested occurrences reject `transform2`, and
  `addExistingComponent` composes with the old occurrence's transform, so do not delete+re-add to move parts.
* Deleted components stay in undo history, so a rebuilt component can be named `PRT_X (1)`; `aegis_cad.bn()` strips it.
  Reopen the saved document to get clean names.
* `constructionPoints.add` is unsupported in this design type: joints use sketch points.
* `importlib.reload` is unsupported; every run starts from fresh modules.
