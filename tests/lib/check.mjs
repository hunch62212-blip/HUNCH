// Pass/fail helpers for the headless tests. Each check prints one line:
//   PASS <name> | <value> (want <rule>)
//   FAIL <name> | <value> (want <rule>)
// Any FAIL sets process.exitCode = 1 so `npm test` reports the file as failed.

const fmt = (v, d) => (typeof v === 'number' ? (Number.isFinite(v) ? +v.toFixed(d) : String(v)) : String(v));

export function check(name, ok, detail = '') {
  console.log(`${ok ? 'PASS' : 'FAIL'} ${name}${detail ? ` | ${detail}` : ''}`);
  if (!ok) process.exitCode = 1;
  return !!ok;
}

// value must be within [lo, hi] (either bound may be null for one-sided checks)
export function within(name, value, lo, hi, unit = '', d = 3) {
  const ok = Number.isFinite(value) && (lo == null || value >= lo) && (hi == null || value <= hi);
  const rule = lo != null && hi != null ? `${lo}..${hi}` : lo != null ? `>= ${lo}` : `<= ${hi}`;
  return check(name, ok, `${fmt(value, d)}${unit} (want ${rule}${unit})`);
}

export const atLeast = (name, value, lo, unit = '', d = 3) => within(name, value, lo, null, unit, d);
export const atMost = (name, value, hi, unit = '', d = 3) => within(name, value, null, hi, unit, d);

export function equal(name, value, want) {
  return check(name, value === want, `${String(value)} (want ${String(want)})`);
}

// Fail the file if it throws or never reaches its checks.
process.on('uncaughtException', (e) => { console.log(`FAIL uncaught exception | ${e && e.message}`); console.error(e); process.exit(1); });
process.on('unhandledRejection', (e) => { console.log(`FAIL unhandled rejection | ${e && e.message}`); console.error(e); process.exit(1); });
