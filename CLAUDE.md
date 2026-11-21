# NASA HUNCH Cargo Rover Directives
- Tech Stack: Python 3.10+ (ROS2/Pytest) & Microcontroller C++ (PlatformIO).
- Verification: Always run unit tests with `pytest` before marking Python tasks complete.
- Architecture: 6-wheel rocker-bogie rover (wheels, not tracks), 4 driven corner wheels; track = 0.52 m (wheel centre planes, 4.24 m full scale), wheelbase 0.49 m; REP-105 coordinate frame.
- Protocol: Frame header `0xAA 0x55`, CRC16 checksum required on all packets.
- Fail-Safe: 250ms watchdog timer (kill PWM if heartbeat packet drops).

# Simulator in this repo (AEGIS-V1, JavaScript)
The directives above are for the physical prototype's firmware (Python / ROS2 / PlatformIO), which is not in this repo.
This repo is the browser simulator of the full-scale rover:
- Stack: JavaScript ES modules, Vite, Three.js (render), Rapier (physics). No Python here.
- Vehicle: AEGIS-V1, 6-wheel rocker-bogie skid steer at 1:1 (the prototype is 1:8.16; K = 8.16 in `config.js`, 12.192 m x 3.0 m module = 1.494 x 0.368 m print). Decisions: 150 kWh battery, 4 driven outer wheels, 5000 kg rover. Constants: `src/sim/config.js`.
- Verification: `npm run test:quick` before marking sim work complete; long runs with `npm run test:long`.
- Record every behaviour change in `CHANGES.md`; values without a source are tagged `[ASSUMED]` in `config.js`.

# Docs (read on demand, only when relevant)
`docs/`: AI_COPILOT(_PLAN), FAULT_PROCEDURES, POWER_BUDGET, PROTOTYPE_POWER_BUDGET, THERMAL_BRAIN, SURVIVABILITY, SPACE_SURVIVAL_PARTS, GRAPHICS_PHYSICS_RESEARCH, LUNAR_CARGO_RESEARCH, REQUIREMENTS, agent-prompts/.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- When graphify-out/graph.json exists, run `graphify query "<question>"` before your first grep, search or source-file read for a codebase question, and put this rule in every subagent prompt that explores code. Read raw files only for lines the graph or an earlier read already located. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. When the answer is under a Markdown heading, `graphify section "<heading>"` prints that section's text, so the .md need not be opened. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
- A committed graphify-out/graph.json is rewritten whole on every build, so a large diff to it in a pull request is expected. Do not flag it as churn or suggest skipping the graph rebuild or its hooks.
