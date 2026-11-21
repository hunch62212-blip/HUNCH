# Fusion agent 1: AEGIS prototype CAD matched to the sim (K = 8.16)

Design and sources: sim = `src/sim/config.js` (ROVER, ARM, DECK), `src/sim/arm.js`, `public/team/js/rocker_bogie.js` (GEO),
`public/team/js/fusion_rover.js` (GEOM), `src/render/cadModels.js`, `public/models/tracked_lunar_rover.obj`. CAD = `cad/fusion/`
(`params.csv`, `lib/aegis_cad.py`, numbered scripts). The sim is the source of truth; prototype mm = full-scale / K, K = 8.16.

## Status
- Fusion design found open: `AEGIS_PROTO_v1_8`, version 5 (the only file in folder Assets).
- Fusion is reachable through `tools/fusion_mcp.py` (HTTP 127.0.0.1:27182; read-only scripts run). The session's `fusion` MCP
  connection reports ECONNREFUSED.
- Safety copy `AEGIS_PROTO_simmatch`: **not made**. The Fusion add-in refuses every script that can change the design while a command
  dialog is open ("Cannot perform 'script' while a command dialog is open"), and Fusion cannot save during a command. Save As attempt
  1 of 3 was refused at 09:20; no retry while a dialog stays open. Dialogs seen (none opened by this agent, none cancelled):
  - Fillet (`FusionFilletEdgesCommand`, 191 faces / 476 edges, radius 0.00 mm): open from the first check at 09:20 to 10:30.
  - Appearance: opened at 10:30. (This agent then stopped on a rate limit until 14:11.)
  - Shell (`FusionShellBodyCommand`): open from 14:11 to at least 14:32 (polled every 15 s to 14:25, checked again at 14:30 and 14:32).
- The open document has unsaved edits since this morning (`isModified` false at 09:58, true at 14:12; timeline still 375 features).
  A Save As puts them in the copy; `AEGIS_PROTO_v1_8` v5 stays as saved.
- No script that changes the design has run; this agent has not modified `AEGIS_PROTO_v1_8`.
- Pending in Fusion (steps 3-4): Save As, script re-run, 95 joint check, 99_verify, 98 export + `stlcopy`, shots. Exact sequence below.

## Discrepancy table
Positions are sim frame (+X forward, +Y up, +Z starboard), prototype mm.

| Item | Sim (source) | CAD before | CAD now | Result |
|---|---|---|---|---|
| Scale in comments | K = 8.16 (`config.js`; values written as `k(prototype m)`) | "1:16.3", "/ 16.32" (README, `params.csv`, lib, 70) | K = 8.16 | fixed (numbers were already prototype scale) |
| Wheels | r 65, w 75 (GEO wheelR 0.065, wheelW 0.075 m); x +240 / 0 / -240, z +-235; FL FR RL RR driven, ML MR idlers | same, `wh_r` / `wh_w` tagged [VERIFY] | same, tags removed | match |
| Rocker pivot P | (0, 194) | (0, 194) | - | match |
| Bogie pivot B | (-120, 125) | (-120, 125) | - | match |
| Rocker plane z | 175 | 175 | - | match |
| Bogie plane z `rb_bogie_z` | 157 | 159 | 159 | not changed (2 mm), sim-side list |
| Arm limits (rad) | q1 [-1.05, 3.0], q2 [-3.0, 2.4], q3 [-2.8, 2.8] | same | - | match |
| Stow q (rad) | [2.8, -2.8, 1.2] | same | - | match |
| Stow wrist axis `stow_x`, `stow_y` | fk(stow q) / K = (161.556, 342.0) | (161.5, 342) | (161.56, 342) | fixed |
| Stow hook seat | (sim hook point = wrist axis) | (173.8, 373.7) | (173.9, 373.7) | follows `stow_x` (+ `hook_off` along psi 1.2) |
| Pregrasp seat | preGrasp = (xBody - preFwd, TRUN_Y - preBelow) = (420 - 30, 90 - 20) = (390, 70) (`config.js:275` ARM.pick) | (420, 40) | (390, 70) | fixed (lib `pose_target`) |
| Pick seat | (xBody, TRUN_Y) = (420, 90) | (420, 90) | - | match |
| Apex | y 515 | (150, 515) | - | match |
| Place x `dk_place_x` | computeDeckPlaceX() -0.3474 m / K = -42.569 | -42.6 | -42.57 | fixed |
| Deck latch x `dk_latch_x` | (`dk_place_x - acm_L/2 - 16`) | -148.6 | -148.57 | follows `dk_place_x` |
| Deck pins | placeX +- 70 | -42.6 +- 70 | -42.57 +- 70 | follows `dk_place_x` |
| Trunnion | d 12, y 90, length 20 | d 12, y 90, `trun_len` 15 | - | length kept (tip clears the forearm inner plate) |
| Hook offset | hook point on the wrist axis | `hook_off` 34 | - | kept (keeps the load torque on the wrist small) |
| prePlace pose | (placeX, deckY + TRUN_Y + ARM.prePlaceH) | none | none | open |
| Revolute axes | wheels, rockers, bogies, arm joints, gates turn about sim +z (lateral); deck latch about +y | the 20 lateral revolutes turned about the vertical (see Joint-axis bug) | lateral, sign mapped to the sim | fixed |
| Chained joint axes | bogie axis rides on the rocker, wheel axes on their pod, elbow / wrist / gate on the parent link | points on a root sketch (axis stays put when the parent moves) | point in the parent's component (`J_PTS`) | fixed |
| Differential | left rocker = -right rocker | `J_DIFF_R/L` free (+-30 deg), no motion link: the rockers moved independently | motion link `ML_DIFFERENTIAL` on `J_DIFF_R/L` | fixed |
| Rocker / bogie limits | GEO rhoMax 0.21 rad, betaMax 0.29 rad (`src/sim` has none) | rocker and diff +-30 deg, bogie +-35 deg | +-12.03 deg / +-16.62 deg | fixed |
| Body frame | tub = rover body | nothing grounded; only the rocker shafts and bogie joint blocks tied (to their links) | `BUY_TUB` grounded; joint blocks, pivot 608s, diff housing, towers, deck latch base tied to it | fixed |
| Pods on links | F pods ride the rocker, M / R pods the bogie | not tied ("rocker/bogie motion moves only the link bodies"); wheels jointed to the pods | every `ASM_DRIVE` part at a wheel centre tied to its link | fixed |
| Per-side suspension hardware | each side's spacer, bogie 608s and bogie pin move with that side | one shared component each (`PRT_BOGIE_SPACER`, `BUY_608`, `BUY_BOGIE_PIN`) | split per side (30) | fixed |
| Shoulder servo | fixed to the tower | not jointed | tied to `PRT_TOWER_R/L` | fixed |
| Sensors | 2 deck LiDARs at (250, 262..266, +-150); stereo cameras at (270..275, 275..290, +-34) | nose LiDAR, rear mast LiDAR, cameras at y 172, z +-48 | unchanged | open (moving them needs a `REF_SWEEP` / keep-out check in Fusion) |
| Fender | yes | none | none | open (fusion agent 2) |
| Team viewer tub (`fusion_rover.js:173`, not in GEOM) | half-width 138 (TUB_W 0.276 m), bottom y 66 (TUB_Y0), length 500 (TUB_L) | inner half-width 146 (`ch_tubZ`), floor sheet y 61 to 65 (`ch_tubY0` 65 = top), length 420 (`ch_tubL`) | unchanged | sim-side list (the viewer is a visual model and disagrees with itself, see below) |
| Legacy model (`tracked_lunar_rover.obj`, `cadModels.js`) | a tracked rover, not AEGIS | - | - | sim-side list |

## Joint-axis bug
- `JointGeometry.createByPoint(sketchPoint)` takes its frame from the sketch: Z = sketch normal. The joint sketches lie on the xZ
  plane, whose normal is Fusion +Y = sim lateral. The old `make_joint` used `YAxisJointDirection`, i.e. a vertical axis, so all 20
  lateral revolutes (6 wheels, 2 rockers, 2 differential shafts, 2 bogies, 3 arm joints x 2, 2 gates) turned about the vertical.
  Now `ZAxisJointDirection`; `joint_sim_sign()` reads the axis and maps the Fusion joint value and limits to the sim sign.
- Chained joints had their point on a root sketch, so the axis did not follow the parent link. Now `comp_joint_pt()` puts the point
  in the parent's own component (sketch `J_PTS`) and returns its assembly-context proxy.

## Changes (`cad/fusion/`)
- `params.csv`: `stow_x` 161.5 -> 161.56, `dk_place_x` -42.6 -> -42.57 (`dk_latch_x` value -148.6 -> -148.57), comments to K = 8.16
  and the sim source, `wh_r` / `wh_w` [VERIFY] removed.
- `lib/aegis_cad.py`: `make_joint` (Z axis, limits in the sim sign), `joint_sim_sign`, `comp_joint_pt`, `clear_comp_joint_pts`,
  `clear_joints` (motion links, then joints; used by `apply_pose` and 99), `pose_target` pregrasp (390, 70) from preFwd 30 / preBelow 20,
  docstrings, K comment.
- `30_rocker_bogie.py`: `PRT_BOGIE_SPACER_R/L`, `BUY_608_P` (pivot bearings), `BUY_608_B_R/L`, `BUY_BOGIE_PIN_R/L`; deletes the old shared
  components in `ASM_SUSPENSION` only (the idler `BUY_608` of 20 in `ASM_DRIVE` is untouched). Body names unchanged.
- `90_assembly_joints.py`: rewritten. Grounding, rigid ties, motion link, chained joint points, GEO limits, unique joint names, a report
  per revolute (sim axis, origin, off-axis mm, sim limits) and a list of moving-assembly parts that no joint holds.
- `95_pose.py`: joint check first (6 suspension tests against the rocker-bogie kinematics, 4 wheel-spin tests, 0.05 mm / 0.1 deg), then the 5 poses.
- `99_verify.py`: `clear_joints()` before the pose checks. `70_deck_guides.py`: comments. `README.md`: scale, safety copy, script table,
  sim-sync numbers, deviations, gotchas.

## Checks run
- `node tests/cad_params.mjs`: 15 / 15 PASS on 215 rows. All edited scripts compile.
- Offline maths (lib + `params.csv`, Fusion mocked): every pose is inside the sim limits. Seats: stow wrist (161.56, 342.00) and seat
  (173.88, 373.69), pregrasp (390.00, 70.00), pick (420.00, 90.00), apex (150.00, 515.00), place (-42.57, 315.00). The 95 wheel-centre
  prediction keeps the rocker and bogie lengths (rigid-body check passes).
- In Fusion: nothing yet (blocked, see Status).

## Verify output
- Not run: Fusion blocked (see Status). The 99_verify numbers in `cad/fusion/README.md` are from v1.8, before this pass.

## Export
- Not run. `out/stl` is unchanged from v1.8 (78 files: 77 STL + `params_export.csv`, 2026-11-20 00:04); `out/shots` has 22 files
  (2026-11-20 00:03), none from this pass. `/tmp/aegis_stl` is empty.

## Run sequence (when no Fusion command dialog is open)
```sh
cd cad/fusion
python3 tools/fusion_mcp.py read '{"queryType":"activeCommand"}'   # must not be a dialog command
# 0. Fusion: File > Save As "AEGIS_PROTO_simmatch" (AEGIS_PROTO_v1_8 v5 stays as the original); confirm it is the open document
python3 tools/fusion_mcp.py script 01_params.py
python3 tools/fusion_mcp.py script 10_chassis.py           # deck M3 holes follow dk_place_x and stow_x (deck_mount_pts)
python3 tools/fusion_mcp.py script 30_rocker_bogie.py      # per-side spacer, bogie 608s and pin
# 50_arm / 60_hook_latch: no re-run (they use the stow joint angles, unchanged, not stow_x)
python3 tools/fusion_mcp.py script 70_deck_guides.py       # pins, deck latch, stow cradle
python3 tools/fusion_mcp.py script 90_assembly_joints.py   # 21 revolute, off-axis ~0 mm, 0 parts held by no joint
python3 tools/fusion_mcp.py script 95_pose.py              # JOINT CHECK PASS (10 tests, 0 failures), then the 5 poses
python3 tools/fusion_mcp.py script 90_assembly_joints.py   # 95 / 99 remove the joints
python3 tools/fusion_mcp.py script 99_verify.py            # fails = 0
python3 tools/fusion_mcp.py script 90_assembly_joints.py
python3 tools/fusion_mcp.py script 98_export.py            # -> /tmp/aegis_stl
python3 tools/fusion_mcp.py stlcopy                        # -> out/stl
python3 tools/fusion_mcp.py shot out/shots/simmatch_iso.png
```

## Sim-side geometry to change later (`src/`, `public/`; not edited here)
- `src/sim/config.js` L5, 83, 124, 131, 237, 323: comments still say 1:16.3 / 16.32 (the values are right at K = 8.16).
- `src/sim/arm.js:11`: comment placeX -0.734 m; `computeDeckPlaceX()` now gives -0.3474 m.
- `public/team/js/module_carry.js:31`: the 40 ft module is labelled "(1:16.3)" and modelled 0.747 m long (12.192 / 0.747 = 16.32);
  at K = 8.16 the print is 1.494 m.
- `src/sim/config.js:93`: wheelbase comment 7.83 m; the geometry gives 2 x 240 mm x 8.16 = 3.92 m.
- `src/sim/config.js:199`: "2.1 m wheel"; the wheel diameter is 2 x 65 mm x 8.16 = 1.06 m.
- `src/sim/config.js:100`: bogie halfSpan / maxBeamAngle 0 gives a divide by zero at `src/sim/rover.js:406`.
- `src/sim/config.js:147`: diff 0.45 is a stiffness fraction, not geometry (no CAD counterpart).
- `src/sim` has no rocker / bogie joint limits; only the team page GEO has rhoMax 0.21 / betaMax 0.29 rad (the CAD now uses those).
- `src/sim/config.js:128` ("Team drive: 300 rpm gearmotors on 130 mm wheels") and `src/sim/health.js:57` ("12 V 300 rpm
  gearmotor"): the team decision for the prototype motor is 100 RPM and the sim value is still open; update both when it is settled.
- Pick one value and update the other side: bogie plane z (sim 157, CAD 159), trunnion length (sim 20, CAD 15), hook offset
  (sim: on the wrist axis, CAD 34).
- Sensors: sim 2 deck LiDARs + stereo cameras (positions in the table) vs the CAD layout; decide which one is the design.
- The prePlace pose exists only in the sim.
- `public/team/js/fusion_rover.js:173` tub: TUB_W 0.276, TUB_L 0.50, TUB_Y0 0.066 m against the CAD's inner half-width 146, length
  420 and floor sheet y 61 to 65 mm; its own comment at L121 (y 0.065 to 0.185) also disagrees with TUB_Y1 0.205. Update the viewer
  to the CAD.
- `public/models/tracked_lunar_rover.obj` and `src/render/cadModels.js` still describe a legacy tracked rover; replace them with an
  AEGIS export (`cad/fusion/out/stl`) or drop them.

## Charging receiver (noted, not modelled yet)
- Spec (`docs/CHARGING_DESIGN.md` 6.1, written by the charging agent): receiver (VA) housing 110 x 110 x 10 mm (0.898 x 0.898 x
  0.082 m / 8.16), coil centre at body x 0, z 0, face at y 61 (0.498 m), top at y 71 (0.580 m), recessed 6 mm (0.050 m) into the belly
  with the face 4 mm proud, 2 heat pipes. Keep-out x +-61, z +-61, y 61 to 71: receiver only (no skid plate, harness or jack points).
- CAD today: no pocket or receiver. Nothing sits under the tub floor at the belly centre; the middle cross rail (y 201 to 221) and
  `BUY_DIFF_HOUSING` (y 176 to 212) are above it.
- Before modelling, two clashes to settle:
  - The spec takes the belly underside as y 65 (0.530 m), but the CAD floor sheet is y 61 to 65 (`ch_tubY0` 65 is its top,
    `ch_tub_t` 4, `10_chassis.py:72`). With the face at 61 the receiver is flush with the CAD underside, not 4 mm proud, and its top
    stands 6 mm above the floor inside the tub.
  - Inside the tub it overlaps the battery tray (5 mm floor on y 65) and the pack (bottom y 70, x -157.5 to -2.5, z +-50) for x -55
    to -2.5. The 420 mm tub leaves about 155 mm each side of the receiver, the pack's own length, so the pack has to go over it (raising
    it about 6 mm puts its top at y 176, where the diff housing starts) or the receiver gets thinner.
- When modelled: a pocket in `TUB_FLOOR` and a receiver body in `10_chassis.py`, new `params.csv` rows, and `40_power_electronics.py`
  if the pack moves.

## Open problems
- Fusion is blocked by an open command dialog (Shell from 14:11; Fillet and Appearance before): safety copy, re-run, joint check,
  99_verify, export and shots are not done. Close or finish the dialog, then run the sequence above. The new joint code (Z-axis
  frame, chained points, motion link, grounding) is only checked offline; the first 90 + 95 run is its real test.
- The open `AEGIS_PROTO_v1_8` has unsaved edits made after 09:58 (not by this agent). Save them to v1_8 first if they belong there;
  otherwise the Save As carries them into `AEGIS_PROTO_simmatch`.
- Charging receiver pocket: not modelled; two clashes to settle first (see Charging receiver).
- To confirm in 95: `J_ROCKER_x` and `J_DIFF_x` are both revolutes of the rocker group about P (coaxial, redundant); the motion link
  must carry a `J_ROCKER_R` move to the left rocker (tests 1, 2 and 6 cover this).
- Grounding `BUY_TUB` fixes the rover body in Fusion: dragging moves the suspension and arms, not the rover.
- Re-running 20_drive_pods duplicates `REF_ORPHAN_BUY_608` (pre-existing).
- `docs/ARM_CARGO_BUILD_PLAN.md:670` still names the old shared `BUY_608` (outside this agent's files, not edited).
- 22 arm / wheel components still carry Fusion's ' (1)' suffix (cosmetic, pre-existing; `bn()` handles it).
- Sensor layout, fender and prePlace are not changed (see the table).
