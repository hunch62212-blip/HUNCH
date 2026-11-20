// CAD parameter table (cad/fusion/params.csv): the Fusion 360 build reads this file, so keep it consistent.
// Checks: required names present, values numeric, no duplicates, and the derived values the plan relies on.
import { readFileSync } from 'node:fs';
import { check, within, equal } from './lib/check.mjs';

const text = readFileSync(new URL('../cad/fusion/params.csv', import.meta.url), 'utf8');
function parseCsv(t) {
  const rows = []; let row = [], f = '', q = false;
  for (let i = 0; i < t.length; i++) {
    const c = t[i];
    if (q) { if (c === '"') { if (t[i + 1] === '"') { f += '"'; i++; } else q = false; } else f += c; }
    else if (c === '"') q = true;
    else if (c === ',') { row.push(f); f = ''; }
    else if (c === '\n' || c === '\r') { if (c === '\r' && t[i + 1] === '\n') i++; row.push(f); f = ''; if (row.length > 1 || row[0]) rows.push(row); row = []; }
    else f += c;
  }
  if (f || row.length) { row.push(f); rows.push(row); }
  return rows;
}
const rows = parseCsv(text);
equal('header', rows[0].slice(0, 4).join(','), 'Name,Unit,Expression,Value');
const P = {}, dup = [], bad = [];
for (const r of rows.slice(1)) {
  const [name, , expr, val] = r;
  if (name in P) dup.push(name);
  const v = Number(val);
  if (!name || !Number.isFinite(v) || !expr) bad.push(name || '(blank)');
  P[name] = v;
}
check('params parsed', rows.length > 150, `${rows.length - 1} rows`);
check('no duplicate names', dup.length === 0, dup.join(','));
check('every value numeric, expression present', bad.length === 0, bad.join(','));

const need = ['bed_max', 'bed_z', 'clr_slip', 'clr_snug', 'clr_press', 'rho_petg', 'infill_eff', 'srv_big_L', 'srv_big_W', 'srv_big_H',
  'srv_std_H', 'srv_mic_H', 'arm_z', 'arm_plate_t', 'arm_plate_gap', 'arm_L1', 'arm_L2', 'arm_sh_x', 'hook_off', 'hook_cheek_ang',
  'trun_d', 'trun_len', 'acm_L', 'acm_W', 'acm_trun_y', 'acm_cg_y', 'acm_mass', 'dk_place_x', 'dk_latch_x', 'stow_x', 'stow_y', 'pick_x',
  'sweep_x0', 'sweep_x1', 'sweep_halfW', 'sweep_top', 'ins_m3_d', 'ins_m3_depth'];
const missing = need.filter((n) => !(n in P));
check('required parameters present', missing.length === 0, missing.join(','));

// derived values (expressions are re-evaluated by Fusion; these are the plan's invariants)
within('arm_plate_gap = srv_big_H + 2 clr_slip', P.arm_plate_gap, P.srv_big_H + 2 * P.clr_slip - 1e-6, P.srv_big_H + 2 * P.clr_slip + 1e-6, ' mm');
within('dk_latch_x = dk_place_x - acm_L/2 - 16', P.dk_latch_x, P.dk_place_x - P.acm_L / 2 - 16 - 1e-6, P.dk_place_x - P.acm_L / 2 - 16 + 1e-6, ' mm');
within('hook cheek lead-in 30 deg (plan s7.3)', P.hook_cheek_ang, 30, 30, ' deg');
within('trunnion dia 12 mm', P.trun_d, 12, 12, ' mm');
within('trunnion tip clear of forearm plate (acm_W/2 + trun_len < arm_z - 40)', P.acm_W / 2 + P.trun_len, 0, P.arm_z - 40, ' mm');
within('FDM clearances ordered snug < slip', P.clr_slip - P.clr_snug, 0.01, 1, ' mm');
within('bed 220 x 250 default', P.bed_max, 150, 400, ' mm');
within('arm reach L1+L2 covers the ground pick at pick_x', Math.hypot(P.pick_x - P.arm_sh_x, 90 - 275), 0, P.arm_L1 + P.arm_L2, ' mm');
within('place point inside sweep box', P.dk_place_x, P.sweep_x0, P.sweep_x1, ' mm');
