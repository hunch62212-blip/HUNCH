// Run one headless test across N map seeds and report a success rate (review item 2: one fixed seed proves little).
//   node tests/seed_sweep.mjs cycle 10          M6 at the requirement's 12,000 s on 10 seeds
//   node tests/seed_sweep.mjs nav_bench 10      M7 legs hab,pad on 10 seeds
//   SEEDS=1,2,3 node tests/seed_sweep.mjs drift  explicit seed list
// A run passes when every PASS/FAIL line it prints is PASS and it exits 0. Output: tests/out/<name>.seeds.json and
// one log per seed (tests/out/<name>.seed<N>.log). Jobs run in parallel (TEST_JOBS, default cores - 1).
import { spawn, execSync } from 'node:child_process';
import { mkdirSync, createWriteStream, writeFileSync } from 'node:fs';
import { cpus } from 'node:os';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..');
const outDir = join(here, 'out');
mkdirSync(outDir, { recursive: true });

// name -> script, args, wall timeout (s). Thresholds are the requirement's own (docs/REQUIREMENTS.md).
const SUITES = {
  cycle: ['autonomy.mjs', ['cycle', '12000'], 7200],
  m3: ['autonomy.mjs', ['m3'], 2400],
  m1: ['autonomy.mjs', ['m1'], 1200],
  nav_bench: ['nav_bench.mjs', ['hab,pad'], 5400],
  drift: ['drift.mjs', [], 2400],
};

const [name, nArg] = process.argv.slice(2);
if (!SUITES[name]) { console.log(`usage: node tests/seed_sweep.mjs <${Object.keys(SUITES).join('|')}> [N]`); process.exit(2); }
// Default seed list: the fixed test seed first, then a deterministic spread (no hand-picked "good" maps).
const BASE = 20262027;
const seeds = process.env.SEEDS ? process.env.SEEDS.split(',').map(Number)
  : Array.from({ length: Math.max(1, Number(nArg) || 10) }, (_, i) => (i === 0 ? BASE : (BASE + i * 104729) % 2147483647));
const [script, args, timeout] = SUITES[name];
const jobs = Math.max(1, Math.min(seeds.length, Number(process.env.TEST_JOBS) || cpus().length - 1));
const runs = [];
const t0 = Date.now();

function runSeed(seed) {
  return new Promise((resolve) => {
    const start = Date.now();
    const log = createWriteStream(join(outDir, `${name}.seed${seed}.log`));
    const child = spawn(process.execPath, [join(here, script), ...args], { cwd: root, env: { ...process.env, SEED: String(seed) } });
    const checks = []; let buf = '';
    child.stdout.on('data', (d) => {
      log.write(d); buf += d; let i;
      while ((i = buf.indexOf('\n')) >= 0) { const line = buf.slice(0, i); buf = buf.slice(i + 1); if (/^(PASS|FAIL) /.test(line)) checks.push(line); }
    });
    child.stderr.on('data', (d) => log.write(d));
    const timer = setTimeout(() => { checks.push(`FAIL timeout | exceeded ${timeout} s wall`); child.kill('SIGKILL'); }, timeout * 1000);
    child.on('close', (code) => {
      clearTimeout(timer); log.end();
      const fails = checks.filter((c) => c.startsWith('FAIL'));
      const pass = code === 0 && fails.length === 0 && checks.length > 0;
      runs.push({ seed, pass, code, checks: checks.length, fails, wall: (Date.now() - start) / 1000 });
      console.log(`${pass ? 'PASS' : 'FAIL'} ${name} seed ${seed}${pass ? '' : ` | ${fails[0] || `exit ${code}, no checks`}`}`);
      resolve();
    });
  });
}

const queue = [...seeds];
await Promise.all(Array.from({ length: jobs }, async () => { while (queue.length) await runSeed(queue.shift()); }));
runs.sort((a, b) => seeds.indexOf(a.seed) - seeds.indexOf(b.seed));
const passed = runs.filter((r) => r.pass).length;
// Wilson 95 % lower bound: with 10 seeds, 10/10 only shows p >= 0.72
const n = runs.length, p = passed / n, z = 1.96;
const lower = n ? (p + z * z / (2 * n) - z * Math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / (1 + z * z / n) : 0;
let commit = 'unknown', dirty = false;
try { commit = execSync('git rev-parse --short HEAD', { cwd: root }).toString().trim(); dirty = execSync('git status --porcelain -- src tests', { cwd: root }).toString().trim().length > 0; } catch { /* not git */ }
const out = { name, script, args, date: new Date().toISOString(), commit, dirty, seeds, passed, n, rate: p, wilson95Lower: lower, runs };
writeFileSync(join(outDir, `${name}.seeds.json`), JSON.stringify(out, null, 1));
console.log(`\n${name}: ${passed}/${n} seeds passed (${(p * 100).toFixed(0)} %, 95 % lower bound ${(lower * 100).toFixed(0)} %) in ${((Date.now() - t0) / 1000).toFixed(0)} s; tests/out/${name}.seeds.json`);
if (passed < n) process.exitCode = 1;
