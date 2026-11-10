// AI copilot checks with a scripted model (no network): allowlist, one-motion-command rule, reply parsing,
// snapshot contents, commands reaching the real console, fault advisor suggestions waiting for "ai ok",
// and an offline model leaving the sim untouched.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../src/sim/sim.js';
import { runCommand } from '../src/sim/commands.js';
import { Copilot, validateCommand, parseReply, snapshot, systemPrompt } from '../src/ai/copilot.js';
import { check } from './lib/check.mjs';
await RAPIER.init();

// ---- 1. allowlist
const allowed = ['status', 'mission cycle', 'goto hab', 'speed 2', 'estop', 'repair mc 3', 'lights on', 'turbo off', 'dock status'];
const blocked = ['reset', 'start', 'newmap', 'map forget', 'fault wheel 3', 'failures on demo', 'clear faults', 'drive 3 0', 'restricted off', 'turbo on', 'dock', 'timescale 50', 'status; newmap', 'x'.repeat(90)];
check('allowlist accepts operator commands', allowed.every((c) => validateCommand(c).ok), allowed.filter((c) => !validateCommand(c).ok).join(', ') || `${allowed.length} ok`);
check('allowlist blocks re-arm, map, fault, raw drive, world', blocked.every((c) => !validateCommand(c).ok), blocked.filter((c) => validateCommand(c).ok).join(', ') || `${blocked.length} blocked`);

// ---- 2. parsing
const p1 = parseReply('{"reply":"ok","commands":["mission 3 A","mission 5","speed 2","newmap"]}');
check('second mission in one answer is dropped', p1.commands.join('|') === 'mission 3 A|speed 2' && p1.rejected.length === 2, JSON.stringify(p1.commands));
const p2 = parseReply('Sure! {"reply":"hi","commands":[]} hope that helps');
check('JSON embedded in prose is recovered', p2.reply === 'hi' && p2.commands.length === 0, JSON.stringify(p2));
const p3 = parseReply('not json at all');
check('non-JSON answer becomes a plain reply', p3.reply === 'not json at all' && !p3.commands.length);

// ---- 3. snapshot and prompt
const sim = new Sim(RAPIER);
for (let i = 0; i < 60; i++) sim.stepOnce();
const s = snapshot(sim), js = JSON.stringify(s);
check('snapshot has drive, battery, cargo, alarms, log', s.drive && s.battery.soc_pct > 0 && s.cargo.length === 2 && Array.isArray(s.fdir_alarms) && s.recent_log.length > 0, `${js.length} chars`);
check('snapshot is compact (< 8 kB)', js.length < 8000, `${js.length} chars`);
const sp = systemPrompt();
check('system prompt lists console commands and missions', sp.includes('mission cycle') && sp.includes('repair <id>') && sp.includes('Only ONE'), `${sp.length} chars`);

// ---- 4. copilot with a scripted model
let answer = '{"reply":"Slowing down.","commands":["speed 2","lights on","reset"]}', calls = 0, lastBody = null;
const fakeFetch = async (url, init) => {
  calls++;
  if (url.endsWith('/api/tags')) return { ok: true, json: async () => ({ models: [{ name: 'qwen3.5:9b' }] }) };
  lastBody = JSON.parse(init.body);
  return { ok: true, json: async () => ({ message: { content: answer }, eval_count: 20, eval_duration: 1e9, prompt_eval_count: 1500 }) };
};
const said = [];
const ai = new Copilot(sim, { url: 'http://x', fetch: fakeFetch, exec: (c) => runCommand(sim, null, c), say: (m, l) => said.push([m, l]) });
await ai.probe();
check('probe sees the model', ai.online && ai.hasModel);
await ai.ask('slow down and turn the lights on');
check('allowed commands reach the console', sim.autonomy.speedSetting === 2 && sim.rover.lights === true, `speed ${sim.autonomy.speedSetting}`);
check('blocked command reported, not run', said.some(([m]) => /reset.*blocked/.test(m)));
check('request is non-thinking, JSON-schema, state included', lastBody.think === false && lastBody.format?.type === 'object' && lastBody.messages.at(-1).content.includes('"battery"'));

// fault advisor: suggestion only, runs after "ai ok"
answer = '{"reply":"MC-3 lost, derated.","commands":["status"]}';
said.length = 0;
sim.health.inject('mc', 3);
for (let i = 0; i < 120 && !sim.health.alarms.size; i++) sim.stepOnce();
check('alarm raised for the advisor', sim.health.alarms.size > 0, [...sim.health.alarms.keys()].join(','));
await new Promise((r) => setTimeout(r, 1700));
await new Promise((r) => setTimeout(r, 50));
check('advisor suggests, does not run', ai.pending.join() === 'status' && said.some(([m]) => m.startsWith('AI: [FDIR advisor]')), JSON.stringify(ai.pending));
check('advisor sends the procedure card', lastBody.messages.at(-1).content.includes('PROCEDURE CARDS'));
ai.runPending();
check('"ai ok" runs the suggestion', ai.pending.length === 0 && said.some(([m]) => m === 'AI > status'));

// ---- 5. offline model: no throw, sim untouched
const dead = new Copilot(sim, { url: 'http://x', fetch: async () => { throw new Error('connect ECONNREFUSED'); }, exec: () => { throw new Error('must not run'); }, say: (m, l) => said.push([m, l]) });
const sp0 = sim.autonomy.speedSetting;
const r = await dead.ask('go to the hab');
check('offline model is reported and nothing runs', r === null && dead.online === false && sim.autonomy.speedSetting === sp0 && said.some(([m]) => m.startsWith('AI offline')));
process.exit();
