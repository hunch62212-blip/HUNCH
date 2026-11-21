# motion_guard

Header-only C++17 protection layer for the Earth prototype rover (`motion_guard.h`, no Arduino/ESP-IDF
dependencies, no heap). Background and numbers: `docs/THERMAL_LOAD_PROTOTYPE.md` and
`scripts/thermal_load_proto.mjs`.

The Cytron MDD10A has no current limit, so a stalled 37D motor on a 13-14 V pack draws 6+ A and reaches
100 C in about 17 s. This module sits between the drive command and the PWM output and adds, per wheel:

- voltage cap: `|duty| <= min(1, 12 V / vbat)`, so a 14.4 V pack never puts more than 12 V on the motor
- two-node thermal estimator (winding + housing) with derate (1.0 at 90 C, linear to 0 at 110 C) and a
  cutoff latch at 110 C that releases below 70 C
- stall detector: `|duty| > 0.3` and `|omega| < 0.5 rad/s` for 1.5 s trips, 10 s rest, retry; 3 trips with
  no 5 s of healthy motion in between latch until `reset()`

Plus a skid-steer pivot limiter (3 s continuous pivot, then a 10 s pivot-only rest) and an arm servo hold
guard (loaded and not stowed for more than 20 s requests a set-down; stowed for 0.5 s turns servos off).

It complements the existing 250 ms heartbeat watchdog (link loss). It does not replace it: keep the
watchdog, and keep zeroing the outputs on link loss.

## Use from the 100 Hz motion loop

```cpp
#include "motion_guard.h"
motion_guard::MotionGuard guard;           // index order FL, FR, RL, RR

void motionTask() {                        // every 10 ms
  motion_guard::GuardInput in;
  for (int i = 0; i < 4; ++i) {
    in.duty[i]  = cmd.duty[i];             // -1..1 from the drive mixer
    in.omega[i] = pcntDeltaToRadPerS(i);   // PCNT counts -> output rad/s, signed like duty
  }
  in.vbat      = readBatteryVolts();
  in.ambientC  = 25.f;                     // or an enclosure temperature sensor
  in.armLoaded = armLoadedFlag;
  in.armStowed = cradleSwitch();           // cradle microswitch

  motion_guard::GuardOutput out = guard.step(0.01f, in);

  for (int i = 0; i < 4; ++i) mdd10aWrite(i, out.duty[i]);   // PWM + DIR
  if (out.servosOff)  pca9685FullOff(ARM_JOINT_CHANNELS);    // cradle carries the load
  if (out.armSetDown) telemetrySetDownRequest();
  telemetryFaults(out.faults, out.windingC, out.currentA);
}
// A latched stall needs an operator action: guard.reset(ambientC)
```

Run the MDD10A PWM at 20 kHz. A low PWM frequency raises motor current ripple and with it the ripple
losses in the winding (and is audible).

## Parameters to replace after the bench test (report section 6)

In `MotorParams`: `Rwh`, `Cw`, `RhaStill`, `RhaMoving`, `Ch` are [ASSUMED] hand values. Verify `R25`, `k`
and `I0` against the real motors too. The thresholds (90/110/70 C, 0.3 duty, 1.5 s, 10 s, 3 s, 20 s) are
constants at the top of each class.

## Tests

`./run_tests.sh` builds with `clang++ -std=c++17 -O2 -Wall -Wextra -Werror` in a temp dir and runs
`test/test_motion_guard.cpp`; it exits non-zero on any failure.
