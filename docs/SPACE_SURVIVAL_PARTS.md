# AEGIS-V1 sim: parts that keep the rover alive in space

Separate from the Earth prototype buy list (`~/HUNCH/Hunch/nasa-llaso-cad/docs/EARTH_PROTOTYPE_PARTS.md`). These are the full-scale parts the sim models (README, docs/SURVIVABILITY.md), not things to buy. All values [ASSUMED] unless the README says [script].

| System | Parts | What the space environment requires of them |
|---|---|---|
| Thermal | avionics vault + battery thermal model, louvered radiators under both deck edges, louver actuators, A/B thermostatic heaters, heater controllers | no air to carry heat away, so radiators reject heat; heaters hold electronics and battery warm through the 14-day night |
| Power | 420 kWh pack in 4 strings (105 kWh each) with contactors, PDU A/B, insulation monitor, charge coupler, 12 m2 keep-alive solar array (4.8 kW) | survives a flat battery away from the dock; solar top-up opens when parked in sunlight |
| Drive | 6 hub gearmotors (5,600 N.m peak), 6 motor controllers (3 per side), spring brakes, resolvers, per-wheel declutch, woven steel hoop tyres | no rubber or air tyres; a seized wheel can free-wheel and the rover drives on 5 |
| Suspension | rocker-bogie with 6 active struts, load cells | handles rocks up to 0.9 m and soft regolith |
| Avionics | flight computers A/B (hot standby), IMU A/B + third-gyro voter, primary and UHF radios, 4 corner omni antennas, locator beacon, hardware E-STOP loop | radiation upsets are handled by redundancy and watchdog; one failure never ends the mission |
| Sensors | 32-beam LiDAR on the mast (spare head B), stereo depth camera, sun sensor, front and rear stereo hazard cameras, headlights, wheel encoders, docking-ring camera | works in sun glare and in the dark (35 m headlights) |
| Dust | electrodynamic dust shield on LiDAR and camera windows, louver cleaning, fenders | regolith fouls windows and radiators |
| Cargo | 2 ring clamps with dual-winding jaw motors, bellows port coupling, seal interlock, leak check | docks to habitat ports in vacuum |
| Service | field kit (spare motor controller, resolver, harness, load cell, E-STOP harness), bays at 1.0-1.6 m, dock spares, gantry | EVA repair with one tool |

## Earth-prototype parts that do NOT survive on the Moon (do not carry into this list)
Brushed 12 V gear motors (vacuum wear), 3007 cooling fans (no air), PETG shell (softens above about 80 C), YDLIDAR X2L (sunlight), terrestrial LiFePO4 and solar panel without thermal control.
