// motion_guard.h - firmware protection layer for the NASA HUNCH Earth prototype rover.
//
// Purpose: the 2x Cytron MDD10A drivers have no current limit, so a stalled Pololu 37D 30:1 motor
// at 13-14 V draws 6+ A and cooks its winding in ~17-30 s (see docs/THERMAL_LOAD_PROTOTYPE.md,
// sections 2, 4.1, 4.2, 4.7, and scripts/thermal_load_proto.mjs for the JS reference model).
// This header-only, host-testable C++17 library adds, per wheel: a voltage cap (never more than the
// 12 V motor rating), a two-node thermal estimator with derate / cutoff, and a stall detector with
// rest-and-retry then latch; plus a skid-steer pivot duty limiter and a servo hold guard for the arm.
//
// It COMPLEMENTS the existing 250 ms heartbeat watchdog (link loss); it does not replace it.
//
// Wiring notes (ESP32-S3 motion loop, call MotionGuard::step() at 100 Hz):
//   in.duty[i]   signed commanded duty -1..1, index order FL, FR, RL, RR (positive = forward)
//   in.omega[i]  signed output-shaft speed in rad/s from the PCNT encoders, same sign convention
//   in.vbat      pack voltage in volts (LiFePO4 12.4 .. 14.4 V) from the ADC divider
//   out.duty[i]  duty to write to the MDD10A PWM (PWM + DIR); use ~20 kHz PWM
//   out.servosOff  -> send PCA9685 full-off on the arm joint channels (cradle carries the load)
//   out.armSetDown -> tell the operator / arm planner to set the arm down in the cradle
//
// No Arduino / ESP-IDF includes, no heap, floats only.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace motion_guard {

// ---------------------------------------------------------------------------------------------
// Motor parameters. Electrical values from the build plan (s1.5, s5.1); thermal values [ASSUMED].
// ---------------------------------------------------------------------------------------------
struct MotorParams {
  float R25 = 2.182f;       // winding resistance at 25 C, ohm (12 V / 5.5 A stall)
  float alphaCu = 0.0039f;  // copper temperature coefficient, 1/K
  float k = 0.334f;         // back-EMF constant, V*s/rad (output-referred)
  float I0 = 0.2f;          // no-load current, A (reserved; mechanical loss is not modelled here)
  float Rwh = 2.5f;         // winding -> housing, K/W            [ASSUMED] replace from bench test (report s6)
  float Cw = 12.f;          // winding heat capacity, J/K         [ASSUMED] replace from bench test (report s6)
  float RhaStill = 9.f;     // housing -> air, still, K/W         [ASSUMED] replace from bench test (report s6)
  float RhaMoving = 6.f;    // housing -> air, driving, K/W       [ASSUMED] replace from bench test (report s6)
  float Ch = 100.f;         // housing heat capacity, J/K         [ASSUMED] replace from bench test (report s6)
  float Vrated = 12.f;      // motor rated voltage, V
};

// ---------------------------------------------------------------------------------------------
// Two-node (winding, housing) thermal model, explicit Euler with internal sub-stepping.
// ---------------------------------------------------------------------------------------------
class MotorThermalModel {
 public:
  explicit MotorThermalModel(const MotorParams& p = MotorParams()) : p_(p) { reset(25.f); }

  void reset(float ambientC = 25.f) {
    tw_ = ambientC;
    th_ = ambientC;
    i_ = 0.f;
  }

  void step(float dt, float duty, float vbat, float omega, float ambientC) {
    if (!(dt > 0.f)) return;
    const int n = std::max(1, static_cast<int>(std::ceil(dt / kMaxSubStep - 1e-6f)));
    const float h = dt / static_cast<float>(n);
    const float rha = std::fabs(omega) > 5.f ? p_.RhaMoving : p_.RhaStill;
    const float vm = duty * vbat;
    for (int s = 0; s < n; ++s) {
      const float r = p_.R25 * (1.f + p_.alphaCu * (tw_ - 25.f));
      i_ = (vm - p_.k * omega) / r;
      const float pcu = i_ * i_ * r;
      const float qwh = (tw_ - th_) / p_.Rwh;
      const float qha = (th_ - ambientC) / rha;
      tw_ += (pcu - qwh) / p_.Cw * h;
      th_ += (qwh - qha) / p_.Ch * h;
    }
  }

  float currentA() const { return i_; }
  float windingC() const { return tw_; }
  float housingC() const { return th_; }

  // Test / init helpers (e.g. restore a remembered temperature, or force a state in tests).
  void setWindingC(float c) { tw_ = c; }
  void setHousingC(float c) { th_ = c; }

  const MotorParams& params() const { return p_; }

 private:
  static constexpr float kMaxSubStep = 0.05f;
  MotorParams p_;
  float tw_ = 25.f, th_ = 25.f, i_ = 0.f;
};

// ---------------------------------------------------------------------------------------------
// Stall detector: |duty| > 0.3 and |omega| < 0.5 rad/s for >= 1.5 s trips. 10 s rest, then retry.
// 3 trips without 5 s of healthy motion in between latch until reset().
// ---------------------------------------------------------------------------------------------
class StallDetector {
 public:
  static constexpr float kDutyMin = 0.3f;
  static constexpr float kOmegaMax = 0.5f;
  static constexpr float kTripTime = 1.5f;
  static constexpr float kRestTime = 10.f;
  static constexpr float kHealthyTime = 5.f;
  static constexpr int kMaxTrips = 3;

  void reset() { *this = StallDetector(); }

  // Returns true if the wheel may be driven this step.
  bool update(float dt, float duty, float omega) {
    if (latched_) return false;
    if (resting_) {
      restT_ += dt;
      if (restT_ >= kRestTime - kEps) {
        resting_ = false;
        restT_ = 0.f;
        stallT_ = 0.f;
      }
      return !resting_;
    }
    const bool driven = std::fabs(duty) > kDutyMin;
    const bool stalled = driven && std::fabs(omega) < kOmegaMax;
    if (stalled) {
      stallT_ += dt;
      if (stallT_ >= kTripTime - kEps) {
        ++trips_;
        stallT_ = 0.f;
        healthyT_ = 0.f;
        if (trips_ >= kMaxTrips) {
          latched_ = true;
        } else {
          resting_ = true;
          restT_ = 0.f;
        }
        return false;
      }
    } else {
      stallT_ = 0.f;
      if (driven) {
        healthyT_ += dt;
        if (healthyT_ >= kHealthyTime - kEps) trips_ = 0;
      }
    }
    return true;
  }

  bool tripped() const { return resting_ || latched_; }
  bool latched() const { return latched_; }
  int tripCount() const { return trips_; }

 private:
  static constexpr float kEps = 1e-4f;
  float stallT_ = 0.f, restT_ = 0.f, healthyT_ = 0.f;
  int trips_ = 0;
  bool resting_ = false, latched_ = false;
};

// ---------------------------------------------------------------------------------------------
// Thermal derate: 1.0 up to 90 C, linear to 0 at 110 C. At >= 110 C a cutoff latches until < 70 C.
// ---------------------------------------------------------------------------------------------
class ThermalDerate {
 public:
  static constexpr float kStartC = 90.f;
  static constexpr float kCutoffC = 110.f;
  static constexpr float kReleaseC = 70.f;

  void reset() { cutoff_ = false; factor_ = 1.f; }

  float update(float windingC) {
    if (windingC >= kCutoffC) cutoff_ = true;
    else if (cutoff_ && windingC < kReleaseC) cutoff_ = false;
    if (cutoff_) factor_ = 0.f;
    else if (windingC <= kStartC) factor_ = 1.f;
    else factor_ = (kCutoffC - windingC) / (kCutoffC - kStartC);
    return factor_;
  }

  float factor() const { return factor_; }
  bool cutoff() const { return cutoff_; }

 private:
  bool cutoff_ = false;
  float factor_ = 1.f;
};

// Voltage cap: a 14.4 V pack must never put more than Vrated on a 12 V motor.
inline float voltageDutyCap(float vbat, float vrated = 12.f) {
  return std::min(1.f, vrated / std::max(vbat, 0.1f));
}

// ---------------------------------------------------------------------------------------------
// Pivot limiter: at most 3 s of continuous pivot (opposite signs, both |cmd| > 0.1), then both
// sides are forced to 0 for a 10 s rest. Only pivots are blocked during the rest.
// ---------------------------------------------------------------------------------------------
class PivotLimiter {
 public:
  static constexpr float kCmdMin = 0.1f;
  static constexpr float kMaxPivotTime = 3.f;
  static constexpr float kRestTime = 10.f;

  void reset() { *this = PivotLimiter(); }

  // left/right: duty-like commands -1..1. Writes the permitted commands to outL/outR.
  // Returns true if the pivot was blocked this step (outputs forced to 0).
  bool step(float dt, float left, float right, float& outL, float& outR) {
    outL = left;
    outR = right;
    if (resting_) {
      restT_ += dt;
      if (restT_ >= kRestTime - kEps) {
        resting_ = false;
        restT_ = 0.f;
      }
    }
    const bool pivot = ((left > kCmdMin && right < -kCmdMin) || (left < -kCmdMin && right > kCmdMin));
    if (!pivot) {
      pivotT_ = 0.f;
      return false;
    }
    if (resting_) {
      outL = outR = 0.f;
      return true;
    }
    pivotT_ += dt;
    if (pivotT_ >= kMaxPivotTime - kEps) {
      resting_ = true;
      restT_ = 0.f;
      pivotT_ = 0.f;
      outL = outR = 0.f;
      return true;
    }
    return false;
  }

  bool resting() const { return resting_; }

 private:
  static constexpr float kEps = 1e-4f;
  float pivotT_ = 0.f, restT_ = 0.f;
  bool resting_ = false;
};

// ---------------------------------------------------------------------------------------------
// Servo hold guard: loaded and not in the cradle for > 20 s -> ask to set the arm down.
// Stowed (cradle microswitch) continuously >= 0.5 s -> servos off (PCA9685 full-off).
// ---------------------------------------------------------------------------------------------
class ServoHoldGuard {
 public:
  static constexpr float kHoldTimeout = 20.f;
  static constexpr float kStowDebounce = 0.5f;

  void reset() { *this = ServoHoldGuard(); }

  void step(float dt, bool armLoaded, bool armStowed) {
    if (armLoaded && !armStowed) {
      heldT_ += dt;
      if (heldT_ > kHoldTimeout) setDown_ = true;
    } else {
      heldT_ = 0.f;
      setDown_ = false;
    }
    if (armStowed) {
      stowedT_ += dt;
      if (stowedT_ >= kStowDebounce - 1e-4f) servosOff_ = true;
    } else {
      stowedT_ = 0.f;
      servosOff_ = false;
    }
  }

  bool setDownRequest() const { return setDown_; }
  bool servosOff() const { return servosOff_; }

 private:
  float heldT_ = 0.f, stowedT_ = 0.f;
  bool setDown_ = false, servosOff_ = false;
};

// ---------------------------------------------------------------------------------------------
// Aggregate guard for the 4 driven wheels. Index order FL, FR, RL, RR (left = FL, RL; right = FR, RR).
// ---------------------------------------------------------------------------------------------
enum Wheel : int { FL = 0, FR = 1, RL = 2, RR = 3 };

enum FaultBits : uint32_t {
  STALL_FL = 1u << 0, STALL_FR = 1u << 1, STALL_RL = 1u << 2, STALL_RR = 1u << 3,
  STALL_LATCH_FL = 1u << 4, STALL_LATCH_FR = 1u << 5, STALL_LATCH_RL = 1u << 6, STALL_LATCH_RR = 1u << 7,
  OVERTEMP_FL = 1u << 8, OVERTEMP_FR = 1u << 9, OVERTEMP_RL = 1u << 10, OVERTEMP_RR = 1u << 11,
  DERATE_FL = 1u << 12, DERATE_FR = 1u << 13, DERATE_RL = 1u << 14, DERATE_RR = 1u << 15,
  PIVOT_REST = 1u << 16,
  ARM_HOLD_TIMEOUT = 1u << 17,
  VBAT_CAP_ACTIVE = 1u << 18,
};

struct GuardInput {
  float duty[4] = {0, 0, 0, 0};   // commanded duty, -1..1
  float omega[4] = {0, 0, 0, 0};  // measured output speed, rad/s (same sign convention as duty)
  float vbat = 12.8f;             // pack voltage, V
  float ambientC = 25.f;          // ambient temperature, C
  bool armLoaded = false;
  bool armStowed = false;
};

struct GuardOutput {
  float duty[4] = {0, 0, 0, 0};      // duty to apply to the MDD10A
  float windingC[4] = {0, 0, 0, 0};  // estimated winding temperatures
  float currentA[4] = {0, 0, 0, 0};  // estimated motor currents
  uint32_t faults = 0;
  bool servosOff = false;
  bool armSetDown = false;
};

class MotionGuard {
 public:
  explicit MotionGuard(const MotorParams& p = MotorParams()) : params_(p) {
    for (auto& m : motor_) m = MotorThermalModel(p);
  }

  void reset(float ambientC = 25.f) {
    for (int i = 0; i < 4; ++i) {
      motor_[i].reset(ambientC);
      stall_[i].reset();
      derate_[i].reset();
    }
    pivot_.reset();
    servo_.reset();
  }

  GuardOutput step(float dt, const GuardInput& in) {
    GuardOutput out;
    float d[4];
    for (int i = 0; i < 4; ++i) d[i] = std::max(-1.f, std::min(1.f, in.duty[i]));

    // 1. Pivot limiter on the side means of the commanded duties.
    const float left = 0.5f * (d[FL] + d[RL]);
    const float right = 0.5f * (d[FR] + d[RR]);
    float pl, pr;
    if (pivot_.step(dt, left, right, pl, pr)) {
      for (int i = 0; i < 4; ++i) d[i] = 0.f;
    }

    // 2..5 per wheel.
    const float cap = voltageDutyCap(in.vbat, params_.Vrated);
    bool capActive = false;
    uint32_t faults = 0;
    for (int i = 0; i < 4; ++i) {
      if (std::fabs(d[i]) > cap) {
        d[i] = d[i] < 0.f ? -cap : cap;
        capActive = true;
      }
      const float factor = derate_[i].update(motor_[i].windingC());
      d[i] *= factor;
      if (!stall_[i].update(dt, d[i], in.omega[i])) d[i] = 0.f;

      motor_[i].step(dt, d[i], in.vbat, in.omega[i], in.ambientC);

      out.duty[i] = d[i];
      out.windingC[i] = motor_[i].windingC();
      out.currentA[i] = motor_[i].currentA();
      if (stall_[i].tripped()) faults |= (STALL_FL << i);
      if (stall_[i].latched()) faults |= (STALL_LATCH_FL << i);
      if (derate_[i].cutoff()) faults |= (OVERTEMP_FL << i);
      if (factor < 1.f) faults |= (DERATE_FL << i);
    }
    if (pivot_.resting()) faults |= PIVOT_REST;

    servo_.step(dt, in.armLoaded, in.armStowed);
    if (servo_.setDownRequest()) faults |= ARM_HOLD_TIMEOUT;
    if (capActive) faults |= VBAT_CAP_ACTIVE;

    out.faults = faults;
    out.servosOff = servo_.servosOff();
    out.armSetDown = servo_.setDownRequest();
    return out;
  }

  // Accessors (diagnostics and tests).
  MotorThermalModel& motor(int i) { return motor_[i]; }
  const StallDetector& stall(int i) const { return stall_[i]; }
  const ThermalDerate& derate(int i) const { return derate_[i]; }

 private:
  MotorParams params_;
  MotorThermalModel motor_[4];
  StallDetector stall_[4];
  ThermalDerate derate_[4];
  PivotLimiter pivot_;
  ServoHoldGuard servo_;
};

}  // namespace motion_guard
