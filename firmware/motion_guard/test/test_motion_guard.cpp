// Host test for motion_guard.h. Build/run with ../run_tests.sh.
#include <cmath>
#include <cstdio>

#include "motion_guard.h"

using namespace motion_guard;

static int g_fail = 0;
static int g_total = 0;

#define CHECK(cond, ...)                          \
  do {                                            \
    ++g_total;                                    \
    if (cond) {                                   \
      std::printf("PASS: ");                      \
    } else {                                      \
      std::printf("FAIL: ");                      \
      ++g_fail;                                   \
    }                                             \
    std::printf(__VA_ARGS__);                     \
    std::printf("  [%s]\n", #cond);               \
  } while (0)

static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

static const float DT = 0.01f;  // 100 Hz

static GuardInput allWheels(float duty, float omega, float vbat) {
  GuardInput in;
  for (int i = 0; i < 4; ++i) {
    in.duty[i] = duty;
    in.omega[i] = omega;
  }
  in.vbat = vbat;
  in.ambientC = 25.f;
  return in;
}

// Unprotected stall time to reach a winding temperature (mirrors timeToWinding in the JS script).
static float stallTimeTo(float vbat, float limitC) {
  MotorThermalModel m;
  m.reset(25.f);
  float t = 0.f;
  while (t < 3600.f) {
    m.step(DT, 1.f, vbat, 0.f, 25.f);
    t += DT;
    if (m.windingC() >= limitC) return t;
  }
  return INFINITY;
}

static void testThermalMatchesJs() {
  std::printf("-- a. thermal model vs JS calculator\n");
  const float t100 = stallTimeTo(13.2f, 100.f);
  const float t130 = stallTimeTo(13.2f, 130.f);
  CHECK(near(t100, 17.3f, 17.3f * 0.15f), "13.2 V stall reaches 100 C at %.2f s (JS 17.3 s +/-15%%)", t100);
  CHECK(near(t130, 32.1f, 32.1f * 0.15f), "13.2 V stall reaches 130 C at %.2f s (JS 32.1 s +/-15%%)", t130);
  MotorThermalModel m;
  m.reset(25.f);
  m.step(DT, 1.f, 14.4f, 0.f, 25.f);
  CHECK(near(m.currentA(), 6.6f, 0.1f), "14.4 V stall current at 25 C = %.3f A (6.6 +/-0.1)", m.currentA());
}

static void testGuardedStall() {
  std::printf("-- b. guarded stall at 13.2 V\n");
  MotionGuard g;
  GuardInput in = allWheels(1.f, 0.f, 13.2f);
  float tripT = -1.f, peak = 0.f, t = 0.f;
  bool zeroAfter = true;
  for (int k = 0; k < 1100; ++k) {  // 11 s: covers the first 10 s rest
    GuardOutput o = g.step(DT, in);
    t += DT;
    if (tripT < 0.f && o.duty[0] == 0.f) tripT = t;
    if (tripT >= 0.f && t > tripT && t < tripT + 9.9f && o.duty[0] != 0.f) zeroAfter = false;
    for (int i = 0; i < 4; ++i) peak = std::fmax(peak, o.windingC[i]);
  }
  CHECK(near(tripT, 1.5f, 0.05f), "trip (duty -> 0) at %.3f s (1.5 +/-0.05)", tripT);
  CHECK(zeroAfter, "applied duty stays 0 for the whole rest");
  CHECK(peak < 50.f, "peak winding %.1f C < 50 C", peak);
}

static void testLatch() {
  std::printf("-- c. three stalls -> latch\n");
  MotionGuard g;
  GuardInput in = allWheels(1.f, 0.f, 13.2f);
  GuardOutput o;
  for (int k = 0; k < 3000; ++k) o = g.step(DT, in);  // 30 s: 3 trips complete at ~24.5 s
  CHECK(g.stall(0).latched() && g.stall(0).tripCount() == 3, "latched after 3 trips (count %d)", g.stall(0).tripCount());
  CHECK((o.faults & STALL_LATCH_FL) && (o.faults & STALL_LATCH_RR), "STALL_LATCH fault bits set (0x%x)", (unsigned)o.faults);
  for (int k = 0; k < 6000; ++k) o = g.step(DT, in);  // 60 s more, several rest periods
  CHECK(g.stall(0).latched() && o.duty[0] == 0.f, "still latched with duty 0 after 60 s more");
  g.reset(25.f);
  GuardInput run = allWheels(0.5f, 10.f, 13.2f);
  o = g.step(DT, run);
  CHECK(!g.stall(0).latched() && o.duty[0] > 0.4f && !(o.faults & STALL_LATCH_FL), "reset() clears the latch (duty %.3f)", o.duty[0]);

  // Healthy motion for 5 s between trips resets the trip count.
  StallDetector sd;
  for (int k = 0; k < 160; ++k) sd.update(DT, 0.8f, 0.f);    // trip 1 at 1.5 s
  for (int k = 0; k < 1000; ++k) sd.update(DT, 0.8f, 0.f);   // rest 10 s
  for (int k = 0; k < 520; ++k) sd.update(DT, 0.8f, 10.f);   // 5.2 s healthy
  CHECK(sd.tripCount() == 0, "5 s healthy motion clears trip count (%d)", sd.tripCount());
}

static void testVoltageCap() {
  std::printf("-- d. voltage cap\n");
  MotionGuard g;
  GuardOutput o = g.step(DT, allWheels(1.f, 10.f, 14.4f));
  CHECK(near(o.duty[0], 0.8333f, 0.001f), "14.4 V, duty 1 -> %.4f", o.duty[0]);
  CHECK(o.faults & VBAT_CAP_ACTIVE, "VBAT_CAP_ACTIVE set");
  MotionGuard g2;
  o = g2.step(DT, allWheels(1.f, 10.f, 12.0f));
  CHECK(near(o.duty[0], 1.0f, 0.001f), "12.0 V, duty 1 -> %.4f", o.duty[0]);
  CHECK(!(o.faults & VBAT_CAP_ACTIVE), "VBAT_CAP_ACTIVE clear at 12 V");
  MotionGuard g3;
  o = g3.step(DT, allWheels(-1.f, -10.f, 14.4f));
  CHECK(near(o.duty[0], -0.8333f, 0.001f), "reverse: 14.4 V, duty -1 -> %.4f", o.duty[0]);
}

static void testDerate() {
  std::printf("-- e. thermal derate and cutoff\n");
  ThermalDerate d;
  CHECK(near(d.update(80.f), 1.f, 1e-6f), "80 C -> factor 1");
  CHECK(near(d.update(100.f), 0.5f, 0.05f), "100 C -> factor %.3f", d.factor());
  CHECK(near(d.update(90.f), 1.f, 1e-6f), "90 C -> factor 1");
  CHECK(near(d.update(110.f), 0.f, 1e-6f) && d.cutoff(), "110 C -> cutoff latched");
  d.update(100.f);
  CHECK(d.cutoff(), "still latched at 100 C");
  d.update(75.f);
  CHECK(d.cutoff(), "still latched at 75 C");
  d.update(69.f);
  CHECK(!d.cutoff() && near(d.factor(), 1.f, 1e-6f), "released below 70 C");

  // Through the whole guard (forced winding temperature).
  MotionGuard g;
  g.motor(0).setWindingC(100.f);
  GuardOutput o = g.step(DT, allWheels(0.4f, 10.f, 13.2f));
  CHECK(near(o.duty[0], 0.2f, 0.02f) && (o.faults & DERATE_FL), "guard at 100 C: duty %.3f (0.4 x 0.5)", o.duty[0]);
  CHECK(near(o.duty[1], 0.4f, 0.001f) && !(o.faults & DERATE_FR), "other wheel unaffected (%.3f)", o.duty[1]);
  g.motor(0).setWindingC(110.5f);
  o = g.step(DT, allWheels(0.4f, 10.f, 13.2f));
  o = g.step(DT, allWheels(0.4f, 10.f, 13.2f));
  CHECK(o.duty[0] == 0.f && (o.faults & OVERTEMP_FL), "guard at >=110 C: duty 0, OVERTEMP set");
  g.motor(0).setWindingC(75.f);
  o = g.step(DT, allWheels(0.4f, 10.f, 13.2f));
  CHECK(o.duty[0] == 0.f && (o.faults & OVERTEMP_FL), "guard at 75 C: still cut off");
  g.motor(0).setWindingC(69.f);
  o = g.step(DT, allWheels(0.4f, 10.f, 13.2f));
  CHECK(o.duty[0] > 0.3f && !(o.faults & OVERTEMP_FL), "guard below 70 C: released (duty %.3f)", o.duty[0]);
}

static void testPivot() {
  std::printf("-- f. pivot limiter\n");
  PivotLimiter p;
  float l, r, t = 0.f;
  float passUntil = -1.f, restUntil = -1.f;
  bool arcBlocked = false;
  bool passedBefore3 = true, zeroDuringRest = true, passedAfter = false;
  for (int k = 0; k < 1500; ++k) {
    p.step(DT, 0.6f, -0.6f, l, r);
    t += DT;
    const bool passed = (l == 0.6f && r == -0.6f);
    if (t < 2.95f && !passed) passedBefore3 = false;
    if (t > 3.05f && t < 12.95f && (l != 0.f || r != 0.f)) zeroDuringRest = false;
    if (t > 13.05f && passed) passedAfter = true;
    if (passUntil < 0.f && !passed) passUntil = t;
    (void)restUntil;
  }
  CHECK(passedBefore3, "pivot passes for the first 3 s");
  CHECK(near(passUntil, 3.0f, 0.05f), "blocked from %.3f s", passUntil);
  CHECK(zeroDuringRest, "both sides 0 from 3 s to 13 s");
  CHECK(passedAfter, "pivot passes again after the 10 s rest");

  PivotLimiter q;
  t = 0.f;
  for (int k = 0; k < 4000; ++k) {  // arcs through the whole sequence, plus interleaved pivots
    float cl = 0.6f, cr = 0.3f;
    if (k < 400) { cl = 0.6f; cr = -0.6f; }  // trigger the rest first
    q.step(DT, cl, cr, l, r);
    if (k >= 400 && (l != cl || r != cr)) arcBlocked = true;
    if (k == 500) CHECK(q.resting(), "resting() true after the 3 s pivot");
  }
  CHECK(!arcBlocked, "same-sign arc (0.6 / 0.3) never blocked, even during the rest");
}

static void testServoHold() {
  std::printf("-- g. servo hold guard\n");
  ServoHoldGuard s;
  for (int k = 0; k < 1900; ++k) s.step(DT, true, false);
  CHECK(!s.setDownRequest(), "no set-down request at 19 s");
  for (int k = 0; k < 200; ++k) s.step(DT, true, false);
  CHECK(s.setDownRequest(), "set-down requested by 21 s");
  CHECK(!s.servosOff(), "servos stay on while not stowed");
  for (int k = 0; k < 30; ++k) s.step(DT, true, true);
  CHECK(!s.servosOff(), "not off after 0.3 s stowed");
  for (int k = 0; k < 30; ++k) s.step(DT, true, true);
  CHECK(s.servosOff(), "servos off after 0.6 s stowed");
  CHECK(!s.setDownRequest(), "set-down request cleared once stowed");
  s.step(DT, true, false);
  CHECK(!s.servosOff(), "servosOff clears when un-stowed");

  // Through the guard.
  MotionGuard g;
  GuardInput in;
  in.armLoaded = true;
  GuardOutput o;
  for (int k = 0; k < 2100; ++k) o = g.step(DT, in);
  CHECK(o.armSetDown && (o.faults & ARM_HOLD_TIMEOUT), "guard: armSetDown + ARM_HOLD_TIMEOUT after 21 s");
  in.armStowed = true;
  for (int k = 0; k < 60; ++k) o = g.step(DT, in);
  CHECK(o.servosOff && !o.armSetDown, "guard: servosOff after stow, set-down cleared");
}

static void testCruise() {
  std::printf("-- h. 20 min normal cruise\n");
  MotionGuard g;
  GuardInput in = allWheels(0.25f, 12.f, 13.2f);
  uint32_t faultsOr = 0;
  float peak = 0.f;
  float lastDuty = 0.f;
  for (int k = 0; k < 120000; ++k) {
    GuardOutput o = g.step(DT, in);
    faultsOr |= o.faults;
    lastDuty = o.duty[0];
    for (int i = 0; i < 4; ++i) peak = std::fmax(peak, o.windingC[i]);
  }
  CHECK(faultsOr == 0, "no faults over 20 min (OR of fault words = 0x%x)", (unsigned)faultsOr);
  CHECK(peak < 60.f, "peak winding %.1f C < 60 C", peak);
  CHECK(near(lastDuty, 0.25f, 1e-6f), "duty passes through unchanged (%.3f)", lastDuty);
}

int main() {
  testThermalMatchesJs();
  testGuardedStall();
  testLatch();
  testVoltageCap();
  testDerate();
  testPivot();
  testServoHold();
  testCruise();
  std::printf("\n%d checks, %d failed\n", g_total, g_fail);
  return g_fail == 0 ? 0 : 1;
}
