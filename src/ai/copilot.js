// AI copilot: a local LLM (Ollama on the team Mac mini, reached over Tailscale) beside the operator console.
// It is never in the control loop. Physics, SLAM, planning, FDIR detection and the automatic fault responses stay
// deterministic. The copilot reads a state snapshot, answers in plain text, and anything it wants done goes
// through the same console commands an operator types, filtered by the allowlist below.
// Works in the browser (via the Vite /ollama proxy) and in node (tests pass `fetch` and `url`).
import { HELP } from '../sim/commands.js';
import { MODE } from '../sim/health.js';
import { ROVER, WORLD } from '../sim/config.js';

export const AI_DEFAULTS = {
  url: '/ollama',          // Vite proxies this to OLLAMA_URL (default: the Mac mini over Tailscale), see vite.config.js
  model: 'qwen3.5:9b',
  numCtx: 8192,
  temperature: 0.2,
  timeoutMs: 90000,        // the first request also loads the model into memory (~5-10 s)
  keepAlive: '30m',
};

// Commands the copilot may run. Excluded on purpose: reset / start (re-arming after an E-stop stays a human
// decision), drive (raw velocity), fault / failures / clear (test injection, clearing comms), newmap / map
// (destroys the map), restricted (geofence), timescale / sun / day / night / lunar (world settings).
const ALLOW = new Set(['status', 'health', 'procedure', 'proc', 'repair', 'mission', 'abort', 'resume', 'goto', 'speed',
  'limit', 'rpm', 'turbo', 'stop', 'brake', 'lift', 'clamp', 'level', 'lights', 'charge', 'dock', 'survival', 'array',
  'reroute', 'plan', 'priority', 'reserve', 'scout', 'routes', 'estop', 'e-stop', 'view', 'camera', 'lidar']);
const MAX_COMMANDS = 4;

export function validateCommand(line) {
  const s = String(line ?? '').trim().replace(/^>\s*/, '');
  if (!s) return { ok: false, why: 'empty' };
  if (s.length > 80 || /[;&|`\n]/.test(s)) return { ok: false, why: 'malformed' };
  const [v, ...a] = s.split(/\s+/), verb = v.toLowerCase();
  if (!ALLOW.has(verb)) return { ok: false, why: `"${verb}" is not on the AI allowlist` };
  if (verb === 'dock' && (a[0] || '').toLowerCase() !== 'status') return { ok: false, why: 'only "dock status"' };
  if (verb === 'turbo' && (a[0] || '').toLowerCase() === 'on') return { ok: false, why: 'turbo on bypasses the terrain governor' };
  return { ok: true, cmd: s };
}

// mission / goto / charge each start a new autonomy job and abort the running one, so a second one in the
// same answer would cancel the first
const MOTION = /^(mission|goto|charge)\b/i;

// The model's JSON answer -> { reply, commands (allowed), rejected }
export function parseReply(text) {
  let o;
  try { o = JSON.parse(text); } catch {
    const m = String(text).match(/\{[\s\S]*\}/);
    try { o = m ? JSON.parse(m[0]) : null; } catch { o = null; }
  }
  if (!o || typeof o !== 'object') return { reply: String(text || '').trim().slice(0, 600), commands: [], rejected: [] };
  const reply = String(o.reply ?? o.answer ?? '').trim().slice(0, 900);
  const commands = [], rejected = [];
  let motion = false;
  for (const c of (Array.isArray(o.commands) ? o.commands : []).slice(0, MAX_COMMANDS)) {
    const r = validateCommand(c);
    if (r.ok && MOTION.test(r.cmd) && motion) { rejected.push({ cmd: r.cmd, why: 'one mission/goto/charge per answer (a second one aborts the first)' }); continue; }
    if (r.ok) { commands.push(r.cmd); motion ||= MOTION.test(r.cmd); } else rejected.push({ cmd: String(c), why: r.why });
  }
  return { reply, commands, rejected };
}

const r1 = (v) => Math.round(v * 10) / 10;

// Compact picture of the vehicle for the prompt (~1-2k tokens)
export function snapshot(sim) {
  const R = sim.rover, A = sim.autonomy, H = sim.health, t = R.telemetry(), e = A.est();
  const P = R.power, cap = ROVER.battery.capacityWh;
  return {
    met_min: r1(sim.time / 60),
    drive: R.estop ? 'E-STOP latched' : R.powered ? 'ARMED' : 'DISARMED',
    autonomy: { mode: A.mode, mission: A.missionName, paused: !!A.paused, phase: A.phase, detail: A.detail },
    position_est_m: { x: Math.round(e.x), z: Math.round(e.z), site: A.site(e.x, e.z) },
    dist_to_habitat_m: Math.round(Math.hypot(WORLD.habitat.x - e.x, WORLD.habitat.z - e.z)),
    dist_to_pad_m: Math.round(Math.hypot(WORLD.pad.x - e.x, WORLD.pad.z - e.z)),
    speed_mps: Math.round(t.speed * 100) / 100,
    battery: { soc_pct: r1(P.soc * 100), kwh_left: Math.round(P.wh / 1000), capacity_kwh: Math.round(cap / 1000), reserve_pct: Math.round(A.reserveSoc * 100), net_w: Math.round(P.netW || 0) },
    tip_margin_deg: r1(t.margin), roll_deg: r1(t.roll * 57.3), pitch_deg: r1(t.pitch * 57.3),
    carrying: R.loaded ? R.lift.cargo.id : null,
    cargo: sim.cargo.map((c) => ({ id: c.id, type: c.T.label, mass_kg: Math.round(c.T.mass), state: c.state, dest_port: c.dest })),
    manifest: A.manifest.map((m) => ({ id: m.id, category: m.category, status: m.status })),
    link_faults: { comms_lost: !!A.faults.comms, queued_commands: (A.queuedCommands || []).length, ports_faulted: Object.keys(A.faults.port || {}).filter((k) => A.faults.port[k]) },
    fdir_alarms: H.summary(),
    repair_in_progress: H.job ? H.job.k : null,
    survival_mode: !!H.survival.on,
    sun_elevation_deg: r1(sim.sunEl),
    scout: sim.scout?.enabled ? `${sim.scout.mode}: ${sim.scout.msg}` : 'parked',
    recent_log: sim.logs.slice(-14).map((l) => `${r1(l.t / 60)}m [${l.level}] ${String(l.msg).slice(0, 200)}`),
  };
}

export function systemPrompt() {
  const cmds = HELP.map(([c, d]) => `  ${c} : ${d}`).join('\n');
  return `You are the AI copilot for ${ROVER.name}, a lunar cargo rover in the NASA HUNCH LLASO-P2-XPORT-2026 simulator.
Mission: carry landed cargo modules (CM-40A 40-ft module, CC-20B 20-ft container) about 2 km from the landing pad
to the habitat and dock them to a pressurised port, with no EVA. 6 wheels, rocker-bogie skid steer, ${Math.round(ROVER.battery.capacityWh / 1000)} kWh battery,
no on-board solar (recharges at the solar power dock), a deterministic autonomy stack (SLAM, A* planner, mission
scripts) and FDIR covering 24 hardware failure modes with automatic responses. Cargo priority: life-support and
emergency-repair 5, power 4, habitat-supplies 3, science 2.

Your job: help the human operator. Explain what is happening, assess FDIR alarms, and suggest or issue console
commands. You are advisory and slow (seconds); the rover's own control loop and automatic fault responses act
first. Never invent sensor values: use only the STATE you are given. If unsure, say so and suggest "health" or
"procedure <id>".

Safety rules:
- Prefer the least drastic command that works. Use "estop" only for an immediate hazard (tip margin < 5 deg, a
  critical alarm while moving, collision risk).
- You cannot re-arm the drive (reset / start) or inject or clear faults; tell the operator when they must.
- A hardware alarm is fixed with "repair <id> <unit>" (e.g. "repair mc 3"), never "clear faults" (a test tool that
  only hides it). Repairs need the rover stopped ("stop" first if moving); heavy units only at the power dock pad.
- If the operator asks for something you are not allowed to do, say so and put nothing for it in "commands".
- Do not start a loaded drive if the battery is below the reserve and the delivery is not covered.

Missions (A = CM-40A, B = CC-20B):
  mission 1          leave the landing zone to the staging area (no cargo)
  mission 2 A|B      find and approach that cargo, no pickup
  mission 3 A|B      approach, pick up and secure that cargo (ends holding it at the pad)
  mission 4          carry the held cargo to the habitat approach
  mission 5          carry the held cargo to its port and dock it (includes the drive; needs cargo held)
  mission cycle      full logistics cycle: every waiting cargo in priority order (survey, pick up, deliver, return)
To deliver one cargo that is not held: "mission 3 A" now, then "mission 5" after the pickup finishes.
Only ONE of mission / goto / charge per answer: each new one aborts the running job.

Console commands (the only actions you can take; use the exact syntax):
${cmds}

Answer ONLY with JSON: {"reply": "<= 3 short sentences for the operator", "commands": ["<console command>", ...]}
"commands" is [] when nothing should be done. At most ${MAX_COMMANDS} commands, run in order.`;
}

const SCHEMA = {
  type: 'object',
  properties: { reply: { type: 'string' }, commands: { type: 'array', items: { type: 'string' } } },
  required: ['reply', 'commands'],
};

export class Copilot {
  // exec(line): run a console command; say(msg, level): write to the operator log
  constructor(sim, { exec, say, fetch: f, ...opts } = {}) {
    this.sim = sim;
    this.o = { ...AI_DEFAULTS, ...opts };
    this.exec = exec || (() => {});
    this.say = say || ((m, l) => sim.log(m, l));
    this.fetch = f || globalThis.fetch.bind(globalThis);
    this.advisor = true;          // auto-assess new FDIR alarms
    this.online = null;           // null = unknown, true / false after a probe
    this.busy = false;
    this.pending = [];            // suggested commands waiting for "ai ok"
    this.history = [];            // last few exchanges, for follow-up questions
    this.stats = { calls: 0, lastMs: 0, lastTokS: 0, errors: 0 };
    this._faultTimer = null; this._faultKeys = new Set();
    sim.on('fault', () => this._onFault());
  }

  async probe() {
    try {
      const r = await this._fetch('/api/tags', null, 5000);
      const names = (r.models || []).map((m) => m.name);
      this.online = true;
      this.hasModel = names.includes(this.o.model) || names.includes(`${this.o.model}:latest`);
      return { ok: true, hasModel: this.hasModel, models: names };
    } catch (e) { this.online = false; return { ok: false, why: e.message }; }
  }

  async _fetch(path, body, timeoutMs = this.o.timeoutMs) {
    const ctl = typeof AbortController !== 'undefined' ? new AbortController() : null;
    const timer = ctl && setTimeout(() => ctl.abort(), timeoutMs);
    try {
      const res = await this.fetch(this.o.url + path, body
        ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body), signal: ctl?.signal }
        : { signal: ctl?.signal });
      if (!res.ok) throw new Error(`HTTP ${res.status} ${(await res.text()).slice(0, 160)}`);
      return await res.json();
    } catch (e) {
      throw new Error(e.name === 'AbortError' ? `timed out after ${timeoutMs / 1000} s` : e.message);
    } finally { if (timer) clearTimeout(timer); }
  }

  // One LLM call: state + question -> { reply, commands, rejected }
  async query(question, extra = '') {
    const user = `STATE:\n${JSON.stringify(snapshot(this.sim))}\n${extra ? `\n${extra}\n` : ''}\nOPERATOR: ${question}`;
    const messages = [{ role: 'system', content: systemPrompt() }, ...this.history.slice(-4), { role: 'user', content: user }];
    const t0 = Date.now();
    const r = await this._fetch('/api/chat', {
      model: this.o.model, messages, stream: false, think: false, format: SCHEMA, keep_alive: this.o.keepAlive,
      options: { temperature: this.o.temperature, num_ctx: this.o.numCtx },
    });
    this.stats.calls++; this.stats.lastMs = Date.now() - t0;
    if (r.eval_count && r.eval_duration) this.stats.lastTokS = r.eval_count / (r.eval_duration / 1e9);
    this.stats.promptTokens = r.prompt_eval_count;
    const content = r.message?.content ?? '';
    // keep only the question (not the bulky state) in the follow-up memory
    this.history.push({ role: 'user', content: `OPERATOR: ${question}` }, { role: 'assistant', content });
    this.history = this.history.slice(-6);
    return parseReply(content);
  }

  // Operator asked something ("ai <text>"): commands the AI returns are run right away (allowlisted only).
  async ask(text, { run = true } = {}) {
    if (this.busy) { this.say('AI: still answering the previous request', 'warn'); return null; }
    this.busy = true; this.say(`AI thinking (${this.o.model})...`, 'ai');
    try {
      const a = await this.query(text);
      this._report(a, run);
      this.online = true;
      return a;
    } catch (e) {
      this.stats.errors++; this.online = false;
      this.say(`AI offline: ${e.message}. Is Ollama running on the Mac mini? ("ai status")`, 'warn');
      return null;
    } finally { this.busy = false; }
  }

  _report(a, run) {
    if (a.reply) this.say(`AI: ${a.reply}`, 'ai');
    for (const r of a.rejected) this.say(`AI wanted "${r.cmd}" - blocked (${r.why})`, 'warn');
    if (!a.commands.length) return;
    if (run) { for (const c of a.commands) { this.say(`AI > ${c}`, 'ai'); this.exec(c); } }
    else { this.pending = a.commands; this.say(`AI suggests: ${a.commands.join(' ; ')}  - type "ai ok" to run`, 'ai'); }
  }

  runPending() {
    if (!this.pending.length) { this.say('AI: nothing pending', 'info'); return; }
    const p = this.pending; this.pending = [];
    for (const c of p) { this.say(`AI > ${c}`, 'ai'); this.exec(c); }
  }

  // New FDIR alarm(s): wait 1.5 s so alarms raised together go in one request, then ask for an assessment.
  // Suggestions only: the operator confirms with "ai ok".
  _onFault() {
    if (!this.advisor || this.online === false) return;
    clearTimeout(this._faultTimer);
    this._faultTimer = setTimeout(() => this._adviseFaults(), 1500);
  }

  async _adviseFaults() {
    if (this.busy) { this._faultTimer = setTimeout(() => this._adviseFaults(), 3000); return; }
    const H = this.sim.health, fresh = [...H.alarms.values()].filter((a) => !this._faultKeys.has(a.key));
    if (!fresh.length) return;
    for (const a of fresh) this._faultKeys.add(a.key);
    const cards = fresh.map((a) => (H.procedure(a.id)?.lines || [MODE[a.id]?.name]).join('\n')).join('\n\n');
    this.busy = true;
    try {
      const a = await this.query(`New FDIR alarm(s): ${fresh.map((x) => x.key).join(', ')}. Assess the impact on the current mission and suggest what the operator should do next.`, `PROCEDURE CARDS:\n${cards}`);
      if (a.reply) a.reply = `[FDIR advisor] ${a.reply}`;
      this._report(a, false);
      this.online = true;
    } catch (e) { this.stats.errors++; this.say(`AI advisor unavailable: ${e.message}`, 'warn'); }
    finally { this.busy = false; }
  }

  status() {
    const s = this.stats;
    return `AI copilot: ${this.online === null ? 'not probed' : this.online ? 'online' : 'OFFLINE'} | model ${this.o.model}${this.hasModel === false ? ' (NOT downloaded)' : ''} | via ${this.o.url} | fault advisor ${this.advisor ? 'on' : 'off'} | ${s.calls} calls, last ${(s.lastMs / 1000).toFixed(1)} s, ${s.lastTokS.toFixed(1)} tok/s, ${s.promptTokens ?? 0} prompt tokens | ${s.errors} errors`;
  }

  // Console entry point for lines starting with "ai"
  async command(args) {
    const sub = (args[0] || '').toLowerCase();
    if (!sub || sub === 'help') {
      this.say('ai <question or request> : ask the copilot (it may run allowlisted commands)', 'info');
      this.say('ai ok : run the pending suggestion | ai brief : situation summary | ai on|off : FDIR advisor | ai status | ai model <name>', 'info');
      return;
    }
    if (sub === 'status') { await this.probe(); this.say(this.status(), this.online ? 'ok' : 'warn'); return; }
    if (sub === 'ok' || sub === 'yes') { this.runPending(); return; }
    if (sub === 'on' || sub === 'off') { this.advisor = sub === 'on'; if (this.advisor) this.online = null; this.say(`AI FDIR advisor ${sub}`, 'ok'); return; }
    if (sub === 'model' && args[1]) { this.o.model = args[1]; this.history = []; await this.probe(); this.say(this.status(), 'info'); return; }
    if (sub === 'forget') { this.history = []; this.pending = []; this.say('AI: conversation memory cleared', 'ok'); return; }
    if (sub === 'brief') { await this.ask('Give a situation brief: what the rover is doing, any risks (battery, alarms, tip margin, comms), and the next sensible step. Do not issue commands.', { run: false }); return; }
    await this.ask(args.join(' '));
  }
}
