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
| v1.2.0 | 2026-11-09 | see tag | Engineering overlay + dynamics instruments (toggle **I**): wheel load / traction arrows, CoM and support polygon, predicted path, braking bar, attitude indicator, stability plan view, SSF / rollover / drawbar / slip / cost-of-transport readouts (CHANGES.md §21; `docs/GRAPHICS_PHYSICS_RESEARCH.md`). Lander regression test `tests/lander.mjs` added to `npm test`. AI copilot: repair-command rule in the prompt, docs `docs/AI_COPILOT.md` and `AI_COPILOT_PLAN.md`, README section. Old lander debug scratch scripts removed. (The §21 lander-collision, lander-tag and headlight fixes themselves shipped in v1.1.0.) Launcher opens Google Chrome. Known issue: `dock_cyl40` still fails. |
| v1.1.0 | 2026-11-09 | see tag | Black-blob fix: NaN wheel tracks smeared by bloom; NaN/Inf guard before bloom (CHANGES.md §19). Survival hardware (radiators, ejecta skirts, whips, beacon, keep-alive outrigger) mounted on the body instead of floating (§20). Also includes all work since v1.0.0 (CHANGES.md §7–§18: audit fixes, vehicle health and FDIR, thermal and survival, loaded traverse, teach & repeat, fenders, 2.5 km random site, 50x transit warp, AI copilot). Known issue: `dock_cyl40` test fails (navigation drives away from PORT-1). |
| v1.0.0 | 2026-11-06 | 4f74946 | AEGIS-V1 bug fixes, power dock, test harness. |
| v0.1.0 | 2026-11-06 | f4e7c18 | Initial import of the simulator as found. |
