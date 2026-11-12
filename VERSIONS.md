# AEGIS-V1 simulator — version history

Every release is a git tag `vX.Y.Z` on `main` and a row in this table; `package.json` carries the same number.
The highest tag is the latest version.

```
npm run launch                 # launch the latest version (highest tag)
bash scripts/launch.sh v1.0.0  # launch a specific older version
bash scripts/launch.sh list    # list all versions, newest first
```

Each version runs from its own clean checkout of its tag (a git worktree in `~/.hunch22-versions/<tag>`), never from
this working folder, so unreleased edits here never leak into a launch. The checkouts are outside iCloud, so their
`node_modules` stay intact. Every version is served at http://127.0.0.1:5173, and logs go to
`~/.hunch22-versions/logs/`. Use `npm run dev` to run the working folder itself.

To add a release: commit, bump `version` in `package.json`, add a row at the top of the table, then
`git tag -a vX.Y.Z -m "<summary>"`. Bump the minor number (1.1 → 1.2) for features and fixes, and the major number for a
redesign.

| Version | Date | Commit | What changed |
|---|---|---|---|
| v1.6.0 | 2026-11-12 | see tag | HUNCH-brief review: third cargo system (CP-36C, 36 Astrobotic-style pods on a ring-adapter rack, PORT-3), spring-damper metal-bellows port coupling (capture force 0.8-1 kN, was 2-6 kN), dust-shield-first berthing order, leak-check hatch interlock with `fault seal`, soft-capture dwell made robust to camera noise, idle solar array, self-park at the power dock, saved-map layout version, per-material mesh merging (draw calls -35 %), shadow and framing fixes, low-battery recursion and frozen-time watchdog fixes, survival-mode retry/resume fixes, new tests (faults_m10 53 checks, logistics, dock/pickup for pods). Known: the full three-delivery cycle does not complete yet: it aborts on navigation events during the long drives (survey return or loaded leg near PORT-3), depending on the noise stream; docking itself passes for all three cargo types. |
| v1.5.0 | 2026-11-10 | see tag | Navigation brain: unmapped ground priced at what mapped ground costs (was ~5x too cheap, so routes kept flipping), plans held 20 s and switched only when clearly cheaper, watchdog no longer lets the rover wander away from its goal, steering-authority check. Gentle turning: moving arcs instead of stop-and-pivot up to 115 deg, inner side >= 50-65 % of the outer, curve speed limit, curvature kept at the rpm limit. Smart rpm v2: continuous 120..280 (cap was 180), names its limiting factor. Speeds 5.0 / 2.5 m/s (operator: middle ground). SLAM: along/across-track degeneracy test, orbital-DEM map-relative fixes (error <= 3.3 m on the 2 km leg, was 9 m). Rover memory store on disk (~/.hunch22-data/memory, `memory`, `?site=last`). Also another session's fixes: fiducial-fix significance test, clamp-retry set-down, planner bounds, command fixes. Quick suite 15/16; known issue: dock_cyl40 final 1 m alignment (CHANGES.md §25), so a full M6 cycle still stops at PORT-1. |
| v1.4.0 | 2026-11-10 | see tag | Thermal brain: regolith temperature follows the sun (hot sun-facing slopes, cold crater shadow, eclipse), LiDAR / camera / motor-controller thermal nodes, nine random thermal scenarios (on in the browser), and a utility-AI controller that revs the motors down when hot, up (stator heating) when cold, and runs instrument heaters / TEC coolers / sunshade and radiator louvers; panel **T** (CHANGES.md §24, `docs/THERMAL_BRAIN.md`, test `thermal`). Also the lunar-realism review pass (§23: traction-limited acceleration, operating speeds, power dock sizing, soil variation, camera lighting, LiDAR A/B, comms delay). Known issue: `dock_cyl40` still fails. |
| v1.3.0 | 2026-11-10 | see tag | Fix: false "wheel seized" isolations when the whole vehicle stalls against a wall (all wheels at current limit) declutched 4 wheels and the rover ran away from the port; seizure now needs the other wheels turning (CHANGES.md §22). Wheel load / traction arrows removed from the physics overlay. Not included: another session's unfinished realism pass (speed limits, soil variation, power dock sizing in config.js / terrain.js / terramechanics.js). |
| v1.2.0 | 2026-11-09 | see tag | Engineering overlay + dynamics instruments (toggle **I**): wheel load / traction arrows, CoM and support polygon, predicted path, braking bar, attitude indicator, stability plan view, SSF / rollover / drawbar / slip / cost-of-transport readouts (CHANGES.md §21; `docs/GRAPHICS_PHYSICS_RESEARCH.md`). Lander regression test `tests/lander.mjs` added to `npm test`. AI copilot: repair-command rule in the prompt, docs `docs/AI_COPILOT.md` and `AI_COPILOT_PLAN.md`, README section. Old lander debug scratch scripts removed. (The §21 lander-collision, lander-tag and headlight fixes themselves shipped in v1.1.0.) Launcher opens Google Chrome. Known issue: `dock_cyl40` still fails. |
| v1.1.0 | 2026-11-09 | see tag | Black-blob fix: NaN wheel tracks smeared by bloom; NaN/Inf guard before bloom (CHANGES.md §19). Survival hardware (radiators, ejecta skirts, whips, beacon, keep-alive outrigger) mounted on the body instead of floating (§20). Also includes all work since v1.0.0 (CHANGES.md §7–§18: audit fixes, vehicle health and FDIR, thermal and survival, loaded traverse, teach & repeat, fenders, 2.5 km random site, 50x transit warp, AI copilot). Known issue: `dock_cyl40` test fails (navigation drives away from PORT-1). |
| v1.0.0 | 2026-11-06 | 4f74946 | AEGIS-V1 bug fixes, power dock, test harness. |
| v0.1.0 | 2026-11-06 | f4e7c18 | Initial import of the simulator as found. |
