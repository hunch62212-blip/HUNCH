// Runs every headless test in parallel and prints one PASS/FAIL line per check, plus a summary.
// Full logs go to tests/out/<name>.log (gitignored).
//   npm test             all tests (the full logistics cycle takes ~25+ min wall time)
//   npm run test:quick   skips the long autonomy runs (marked long below)
//   node tests/run-all.mjs dock0 rock   run only the named entries
import { spawn } from 'node:child_process';
import { mkdirSync, createWriteStream, writeFileSync } from 'node:fs';
import { execSync } from 'node:child_process';
import { cpus } from 'node:os';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..');
const outDir = join(here, 'out');
mkdirSync(outDir, { recursive: true });

// name, script, args, long (skipped by --quick), timeout (wall s)
const TESTS = [
  ['physics', 'headless.mjs', [], false, 600],
  ['rocks', 'rock_climb.mjs', [], false, 600],
  ['v17', 'v17_suite.mjs', [], false, 900],
  ['stand_exit', 'stand_exit.mjs', [], false, 1500],
  ['lander', 'lander.mjs', [], false, 600],
  ['thermal', 'thermal.mjs', [], false, 900],
  ['planner', 'planner.mjs', [], false, 600],
  ['nav_retry', 'nav_retry.mjs', [], false, 600],
  ['frame_shift', 'frame_shift.mjs', [], false, 300],
  ['ai_copilot', 'ai_copilot.mjs', [], false, 300],
  ['creep', 'creep.mjs', [], false, 600],
  ['odo', 'odo.mjs', [], false, 600],
  ['actuators', 'actuators.mjs', [], false, 300],
  ['acm_soil', 'acm_soil.mjs', [], false, 300],
  ['arm', 'arm.mjs', [], false, 600],
  ['drive4', 'drive4.mjs', [], false, 600],
  ['arm_pickup', 'arm_pickup.mjs', [], false, 1200],
  ['pickup_gt_cyl40', 'pickup_gt.mjs', ['0'], false, 900],
  ['pickup_gt_box20', 'pickup_gt.mjs', ['1'], false, 900],
  ['pickup_gt_pod4', 'pickup_gt.mjs', ['2'], false, 900],
  ['pickup_time_cyl40', 'pickup_time.mjs', ['0'], false, 1200],
  ['pickup_time_box20', 'pickup_time.mjs', ['1'], false, 1200],
  ['pickup_time_pod4', 'pickup_time.mjs', ['2'], false, 1200],
  ['dock_cyl40', 'dock.mjs', ['0'], false, 1200],
  ['dock_box20', 'dock.mjs', ['1'], false, 1200],
  ['dock_pod4', 'dock.mjs', ['2'], false, 1200],
  ['dock_capture', 'unit_dock_capture.mjs', [], false, 1200],
  ['timestep', 'unit_timestep.mjs', [], false, 300],
  ['scout', 'scout.mjs', [], false, 900],
  ['m1', 'autonomy.mjs', ['m1'], false, 1200],
  ['logistics', 'logistics.mjs', [], false, 600],
  ['faults_m10', 'faults_m10.mjs', [], false, 1800],
  ['m3', 'autonomy.mjs', ['m3'], true, 2400],
  ['drift', 'drift.mjs', [], true, 2400],
  ['faults', 'faults.mjs', [], true, 3600],
  ['fdir', 'fdir.mjs', [], true, 3600],
  ['power_dock', 'power_dock.mjs', [], true, 3600],
  ['nav_bench', 'nav_bench.mjs', ['hab,pad'], true, 5400],
  // M6 is 12,000 s sim (docs/REQUIREMENTS.md). v1.7.0 passed only with the cap raised to 24,000 s; the test now uses the requirement.
  ['cycle', 'autonomy.mjs', ['cycle', '12000'], true, 7200],
];

const argv = process.argv.slice(2);
const quick = argv.includes('--quick');
const only = argv.filter((a) => !a.startsWith('--'));
const list = TESTS.filter(([n, , , long]) => (only.length ? only.includes(n) : !(quick && long)));
const unknown = only.filter((n) => !TESTS.some((t) => t[0] === n));
if (unknown.length) { console.log(`Unknown test(s): ${unknown.join(', ')}. Known: ${TESTS.map((t) => t[0]).join(', ')}`); process.exit(2); }

const limit = Math.max(1, Math.min(list.length, Number(process.env.TEST_JOBS) || cpus().length - 1));
const results = [];
const t0 = Date.now();

function runOne([name, script, args, , timeout]) {
  return new Promise((resolve) => {
    const start = Date.now();
    const log = createWriteStream(join(outDir, `${name}.log`));
    const child = spawn(process.execPath, [join(here, script), ...args], { cwd: root });
    const checks = [];
    let buf = '';
    const onData = (d) => {
      log.write(d); buf += d;
      let i;
      while ((i = buf.indexOf('\n')) >= 0) {
        const line = buf.slice(0, i); buf = buf.slice(i + 1);
        if (/^(PASS|FAIL) /.test(line)) { checks.push(line); console.log(`[${name}] ${line}`); }
      }
    };
    child.stdout.on('data', onData); child.stderr.on('data', (d) => log.write(d));
    const timer = setTimeout(() => { checks.push(`FAIL timeout | exceeded ${timeout} s wall`); console.log(`[${name}] FAIL timeout | exceeded ${timeout} s wall`); child.kill('SIGKILL'); }, timeout * 1000);
    child.on('close', (code) => {
      clearTimeout(timer); log.end();
      const fails = checks.filter((c) => c.startsWith('FAIL')).length;
      const pass = code === 0 && fails === 0 && checks.length > 0;
      if (!checks.length) console.log(`[${name}] FAIL no checks reported (exit ${code})`);
      results.push({ name, pass, checks: checks.length, fails, code, wall: (Date.now() - start) / 1000, lines: checks });
      resolve();
    });
  });
}

const queue = [...list];
await Promise.all(Array.from({ length: limit }, async () => { while (queue.length) await runOne(queue.shift()); }));

console.log('\n==== SUMMARY ====');
const order = new Map(TESTS.map((t, i) => [t[0], i]));
results.sort((a, b) => order.get(a.name) - order.get(b.name));
for (const r of results) console.log(`${r.pass ? 'PASS' : 'FAIL'} ${r.name.padEnd(18)} ${String(r.checks - r.fails).padStart(2)}/${r.checks} checks  exit ${r.code}  ${r.wall.toFixed(0)} s`);
const failed = results.filter((r) => !r.pass);
console.log(`${results.length - failed.length}/${results.length} test files passed in ${((Date.now() - t0) / 1000).toFixed(0)} s (logs: tests/out/)`);
// Machine-readable record for scripts/record-results.mjs (fills the Result column of docs/REQUIREMENTS.md).
let commit = 'unknown', dirty = false;
try { commit = execSync('git rev-parse --short HEAD', { cwd: root }).toString().trim(); dirty = execSync('git status --porcelain -- src tests', { cwd: root }).toString().trim().length > 0; } catch { /* not a git checkout */ }
writeFileSync(join(outDir, 'results.json'), JSON.stringify({ date: new Date().toISOString(), commit, dirty, seed: process.env.SEED ? Number(process.env.SEED) : null, results }, null, 1));
if (failed.length) process.exitCode = 1;
