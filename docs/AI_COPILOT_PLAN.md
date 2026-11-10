# AI copilot: plan for the next session

Written 2026-11-09. **Do not start until the user asks** ("implement the copilot AI" or similar). Background and
measurements: [AI_COPILOT.md](AI_COPILOT.md). This file is the to-do list and everything needed to pick it up cold.

## 0. Where things stand (as of writing)

The first version is **built and working**. Next time, finish and harden it; don't rebuild it.

| Piece | State |
|---|---|
| `src/ai/copilot.js` | Copilot class, state snapshot, system prompt, allowlist, one-motion rule, FDIR advisor, `ai ...` commands. Early version committed in v1.1.0 (c462458); the "repair, not clear faults" prompt rule is **uncommitted** |
| `src/ui/hud.js` | console routing (`ai ...` and plain English → AI), **AI ONLINE** pill. Committed |
| `src/style.css` | `#log .ai` colour. Committed |
| `vite.config.js` | `/ollama` proxy → `OLLAMA_URL` (default `http://100.92.134.100:11434`), Origin header removed. Committed |
| `tests/ai_copilot.mjs` | 17 offline checks, registered in `tests/run-all.mjs` as `ai_copilot`. Committed, passing |
| `scripts/ai_probe.mjs` | live scenarios against the real model. Committed |
| `README.md` "AI copilot" section, `docs/AI_COPILOT.md` | **uncommitted** |
| Mac mini | Ollama 0.40.16, models `qwen3.5:9b` (default) and `qwen3.5:4b` (low-memory); `tailscale serve --bg --tcp 11434 tcp://127.0.0.1:11434` active |

Measured: 9B is 12.5-20 tok/s and 2-18 s per answer on an idle Mac (4-7 tok/s and up to 87 s while 3 test sims
ran); 4B is 23-25 tok/s, 1.5-12 s, 5 GB loaded. macOS GPU limit ~12.7 GB. Quick suite 13/14; `dock_cyl40` fails
from a **known navigation bug unrelated to the AI** (see VERSIONS.md v1.1.0).

## 1. Things to watch out for

* **Another Claude session works in this repo at the same time.** Run `git status` first. Stage only files this
  plan touches. Never switch branches, stash, or reset. Its in-progress files at writing time: `src/main.js`,
  `src/ui/instruments.js`, `src/render/engOverlay.js`, and edits to `hud.js` / `style.css` (instruments panel).
  Edit `hud.js` with small exact replacements so its edits stay intact.
* **Port 5173** may be the other session's `scripts/launch.sh` (it serves a tagged release from
  `~/.hunch22-versions/<tag>`). For testing the working tree use `npm run dev` and read the actual port from its
  output (5174 last time). "Launch" for the user means `bash scripts/launch.sh latest`, not `npm run dev`.
* **Memory:** don't run `scripts/ai_probe.mjs` while the test suite runs; the 9B model slowed 3-4x and the Mac swapped.
  Run `ollama stop <model>` after benchmarking.
* `CLAUDE.md` describes a Python/ROS2 project; this repo is JavaScript. Verify with `npm test` / `node tests/...`,
  not pytest.
* Chrome tab groups were unstable last time (likely the other session using Chrome). If browser tools fail twice,
  verify headless instead (the probe + unit test cover the logic).

## 2. Steps

Run 0 first, then 1-6 in order. Each step lists how to check it.

### Step 0: check the setup (5 min)
* `git status`, `git log --oneline -5`, `ollama list`, `tailscale serve status`,
  `curl -s http://100.92.134.100:11434/api/version`.
* `node tests/ai_copilot.mjs` → 17 PASS.
* If Tailscale serve is gone, re-run `tailscale serve --bg --tcp 11434 tcp://127.0.0.1:11434`. If a model is missing,
  `ollama pull qwen3.5:9b`.

### Step 1: give the model computed facts instead of making it do maths
The model wrote "derated *to* 25 %" for a 25 % reduction. Put the derived numbers in `snapshot()`:
* `speed_derate_pct` from `sim.health.fx.speedK` (`Math.round((1 - k) * 100)`), `wheels_driven` (6 minus faulted or
  declutched wheels), `moving` (speed > 0.05), `at_dock` (`health.atDock()`),
  `repairable_now` (alarm keys that `repair` accepts: rover stopped, and not a dock-only unit away from the dock).
* For each alarm, add `{key, name, sev, auto_response, repair_where, repair_min}` from `MODE` / `LRUS`
  instead of only the summary string.
* Check: unit test asserts the new fields; the probe's MC-3 answer states "by 25 %" and "5 wheels".

### Step 2: make answers faster (target: typical answer < 8 s on 9B)
* Log `prompt_eval_count`, `prompt_eval_cached_count` and `prompt_eval_duration` from the Ollama reply into `stats`;
  show them in `ai status`.
* Shorten the system prompt: list only the allowlisted commands (filter `HELP` by `ALLOW`), drop the long
  descriptions for commands the AI can't run. Keep the system prompt identical between calls so the prefix can be reused.
* Keep the state snapshot as the last message (already true).
* Check: `node scripts/ai_probe.mjs` before/after; record times in AI_COPILOT.md §6.

### Step 3: automatic small-model fallback
* If a 9B call times out or takes > 30 s, switch to `qwen3.5:4b` for the rest of the session and log
  `AI: switched to qwen3.5:4b (9b too slow: N s)`. `ai model qwen3.5:9b` switches back.
* Lower the default timeout from 90 s to 45 s, except the first call, which also loads the model.
* Check: unit test with a slow fake fetch; the second call goes to the 4B model.

### Step 4: show pending suggestions on screen, and record them
* When the FDIR advisor has pending commands, show them next to the AI pill with a **Run** button
  (same as `ai ok`) and a dismiss ×. Keep it small; don't overlap the other session's `#instPanel`.
* Keep `copilot.journal`: `{t, kind: ask|advisor|brief, question, reply, commands, rejected, ran, latency_ms, model}`.
  Add it to the existing "mission report" JSON export in `hud.js`.
* Check: browser test (inject `fault mc 3` during `mission 1`, click Run, see `status` output); the export
  contains the journal.

### Step 5: more live scenarios and offline checks
Add to `scripts/ai_probe.mjs`:
* "make the 20-ft container life-support priority" → `priority B life-support`
* comms loss (`fault comms`) then a request → command is queued (shows the existing comms-loss path works)
* rover moving + "repair the motor controller" → `stop` then `repair mc 3`, or an explanation that it must stop first
* "battery is low, what should I do" with SoC set to 15 % → `charge` or a recharge explanation
* E-stop latched + "keep going" → refuses to re-arm

Add to `tests/ai_copilot.mjs`: the new snapshot fields, the fallback, the journal, the `ai ok` button path
(logic only).

### Step 6: docs, tests, and shipping
* Update `docs/AI_COPILOT.md` §5-6 with the new measurements, and the README AI section if commands changed.
* Run `node tests/ai_copilot.mjs`, `npx vite build`, then `TEST_JOBS=3 npm run test:quick` with **no model loaded**.
  Expect everything to pass except the known `dock_cyl40` failure; report it if it still fails.
* Run `node scripts/ai_probe.mjs` (9B, then `AI_MODEL=qwen3.5:4b`) on a quiet machine.
* **Commit only if the user asks.** If they do: stage only the AI files (`src/ai/`, `tests/ai_copilot.mjs`,
  `scripts/ai_probe.mjs`, `docs/AI_COPILOT*.md`, the README AI section, and the AI-related `hud.js` / `style.css`
  hunks via `git add -p`). For a release, follow the repo convention: bump `package.json` (minor → v1.2.0), add a
  row at the top of `VERSIONS.md`, `git tag -a v1.2.0`.

## 3. Out of scope (decided)

* Not in the control loop: no AI in physics, SLAM, planning, FDIR detection, automatic responses, or E-stop.
* The AI never gets `reset` / `start`, `drive`, `turbo on`, `fault`, `failures`, `clear`, `newmap`, `map`,
  `restricted`, or time / sun / world commands.
* The physical `pi-top-rover` on the tailnet: later, separate project. Its 250 ms watchdog stays in firmware.
