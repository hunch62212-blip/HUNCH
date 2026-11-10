# NASA HUNCH Cargo Rover Directives
- Tech Stack: Python 3.10+ (ROS2/Pytest) & Microcontroller C++ (PlatformIO).
- Verification: Always run unit tests with `pytest` before marking Python tasks complete.
- Architecture: 6WD tracked skid-steer; track separation = 0.42 m; REP-105 coordinate frame.
- Protocol: Frame header `0xAA 0x55`, CRC16 checksum required on all packets.
- Fail-Safe: 250ms watchdog timer (kill PWM if heartbeat packet drops).
