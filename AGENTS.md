# NASA HUNCH Cargo Rover Directives
- Tech Stack: Python 3.10+ (ROS2/Pytest) & Microcontroller C++ (PlatformIO).
- Verification: Always run unit tests with `pytest` before marking Python tasks complete.
- Architecture: 6WD tracked skid-steer; track separation = 0.42 m; REP-105 coordinate frame.
- Protocol: Frame header `0xAA 0x55`, CRC16 checksum required on all packets.
- Fail-Safe: 250ms watchdog timer (kill PWM if heartbeat packet drops).

# Simulator in this repo (AEGIS-V1, JavaScript)
The directives above are for the physical prototype's firmware (Python / ROS2 / PlatformIO), which is not in this repo.
This repo is the browser simulator of the full-scale rover:
- Stack: JavaScript ES modules, Vite, Three.js (render), Rapier (physics). No Python here.
- Vehicle: AEGIS-V1, 6-wheel rocker-bogie skid steer at 1:1 (prototype is 1:16.3). Constants: `src/sim/config.js`.
- Verification: `npm run test:quick` before marking sim work complete; long runs with `npm run test:long`.
- Record every behaviour change in `CHANGES.md`; values without a source are tagged `[ASSUMED]` in `config.js`.
