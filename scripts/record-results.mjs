// Fill the Result column of docs/REQUIREMENTS.md from the last test records (review item 2: the column was empty
// placeholders). Sources: tests/out/results.json (tests/run-all.mjs) and tests/out/<name>.seeds.json (tests/seed_sweep.mjs).
// A cell is written as "pass 12/12 (abc1234, 2026-11-21)" or "FAIL 3/5 ...", seed sweeps as "7/10 seeds (...)".
// A requirement whose tests were not all in the record reads "not run at <commit>" instead of keeping an old value.
//   node scripts/record-results.mjs [--dry]
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { execSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const reqPath = join(root, 'docs', 'REQUIREMENTS.md');
const out = join(root, 'tests', 'out');
const dry = process.argv.includes('--dry');
const head = (() => { try { return execSync('git rev-parse --short HEAD', { cwd: root }).toString().trim(); } catch { return 'unknown'; } })();

// marker -> run-all test names, plus an optional seed sweep that must also exist
const MAP = {
  m1: { tests: ['m1'] },
  m2: { tests: ['m3', 'pickup_time_cyl40', 'pickup_time_box20'] },
  m3: { tests: ['pickup_gt_cyl40', 'pickup_gt_box20', 'pickup_time_cyl40', 'pickup_time_box20', 'pickup_time_pod4', 'm3'] },
  m4: { tests: ['cycle'] },
  m5: { tests: ['dock_cyl40', 'dock_box20', 'dock_capture'] },
  m6: { tests: ['cycle'], sweep: 'cycle' },
  m7: { tests: ['nav_bench', 'drift', 'faults'], sweep: 'nav_bench' },
  m8: { tests: ['pickup_gt_box20', 'pickup_time_box20', 'dock_box20'] },
  m9: { tests: ['scout'] },
  m10: { tests: ['faults', 'fdir', 'faults_m10'], sweep: 'fdir_mc' },
  m11: { tests: ['logistics'] },
  m12a: { tests: ['nav_bench'] },
  m12b: { tests: ['odo'] },
  m12c: { tests: ['pickup_gt_cyl40'] },
  fblock: { tests: ['faults'] }, fwheel: { tests: ['faults'] },
  flidar: { tests: ['faults_m10'] }, fcamera: { tests: ['faults_m10'] }, fcomms: { tests: ['faults_m10'] },
  fbattery: { tests: ['faults_m10'] }, fshift: { tests: ['faults_m10'] }, fport: { tests: ['faults_m10'] },
  fdirA: { tests: ['fdir'] }, fdirB: { tests: ['fdir'] }, fdirC: { tests: ['fdir'] }, fdirD: { tests: ['fdir'] }, fdirE: { tests: ['fdir'] }, fdirF: { tests: ['fdir'] },
  base: { tests: ['physics', 'creep', 'rocks'] },
};

const read = (f) => (existsSync(f) ? JSON.parse(readFileSync(f, 'utf8')) : null);
const all = read(join(out, 'results.json'));
const day = (iso) => (iso || '').slice(0, 10);
const tag = (rec) => `${rec.commit}${rec.dirty ? '+dirty' : ''}, ${day(rec.date)}`;

function cell(key) {
  const m = MAP[key];
  const parts = [];
  const got = all ? m.tests.map((t) => all.results.find((r) => r.name === t)).filter(Boolean) : [];
  if (got.length === m.tests.length) {
    const ok = got.every((r) => r.pass);
    const pc = got.reduce((s, r) => s + r.checks - r.fails, 0), tc = got.reduce((s, r) => s + r.checks, 0);
    parts.push(`${ok ? 'pass' : 'FAIL'} ${pc}/${tc} (${tag(all)})`);
  } else parts.push(`not run at ${head}`);
  if (m.sweep) {
    const s = read(join(out, `${m.sweep}.seeds.json`));
    parts.push(s ? `${s.passed}/${s.n} seeds, 95 % lower bound ${(s.wilson95Lower * 100).toFixed(0)} % (${tag(s)})` : 'seed sweep not run');
  }
  return parts.join('; ');
}

let md = readFileSync(reqPath, 'utf8');
let n = 0;
// Markers are kept so the next run can rewrite the cell: <!--m6-->text<!--/m6--> (bare <!--m6--> is the empty form).
md = md.replace(/<!--(\w+)-->(?:.*?<!--\/\1-->)?/g, (all0, key) => { if (!MAP[key]) return all0; n++; return `<!--${key}-->${cell(key)}<!--/${key}-->`; });
if (dry) console.log(md.split('\n').filter((l) => /<!--\w+-->/.test(l)).join('\n'));
else { writeFileSync(reqPath, md); console.log(`docs/REQUIREMENTS.md: ${n} result cells written (results.json ${all ? tag(all) : 'missing'})`); }
