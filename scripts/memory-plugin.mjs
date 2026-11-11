// Rover memory store (v1.5.0): a small HTTP API on the Vite dev / preview server that keeps the SLAM map and the
// proven-route memory on disk, outside the browser, so it survives cleared browser data, other browsers and every
// launched version (scripts/launch.sh serves each version from its own folder; they all share this store).
//
//   GET  /api/memory/sites         -> [{ seed, savedAt, bytes, history }]   newest first
//   GET  /api/memory/<seed>        -> the stored blob (gzip; format in src/sim/memoryStore.js), 404 if none
//   PUT  /api/memory/<seed>        <- blob; written atomically, plus a timestamped history copy (at most one per 10 min,
//                                     newest 10 kept per site)
//   DELETE /api/memory/<seed>      -> removes the site's current memory (history copies are kept)
//
// Location: $HUNCH22_MEMORY_DIR, default ~/.hunch22-data/memory (outside iCloud: no sync conflicts, no evicted files).
import { mkdirSync, readFileSync, writeFileSync, renameSync, readdirSync, statSync, unlinkSync, existsSync } from 'node:fs';
import { join } from 'node:path';
import { homedir } from 'node:os';

const DIR = process.env.HUNCH22_MEMORY_DIR || join(homedir(), '.hunch22-data', 'memory');
const HIST = join(DIR, 'history');
const MAX_BYTES = 200 * 1024 * 1024;
const fileOf = (seed) => join(DIR, `site-${seed}.bin`);

function readBody(req) {
  return new Promise((res, rej) => {
    const parts = []; let n = 0;
    req.on('data', (c) => { n += c.length; if (n > MAX_BYTES) { rej(new Error('too large')); req.destroy(); } else parts.push(c); });
    req.on('end', () => res(Buffer.concat(parts))); req.on('error', rej);
  });
}

function sites() {
  if (!existsSync(DIR)) return [];
  const hist = existsSync(HIST) ? readdirSync(HIST) : [];
  return readdirSync(DIR).filter((f) => /^site-\d+\.bin$/.test(f)).map((f) => {
    const seed = Number(f.slice(5, -4)), st = statSync(join(DIR, f));
    return { seed, savedAt: st.mtimeMs, bytes: st.size, history: hist.filter((h) => h.startsWith(`site-${seed}-`)).length };
  }).sort((a, b) => b.savedAt - a.savedAt);
}

function save(seed, buf) {
  mkdirSync(HIST, { recursive: true });
  const f = fileOf(seed), tmp = `${f}.tmp`;
  writeFileSync(tmp, buf); renameSync(tmp, f);
  const mine = readdirSync(HIST).filter((h) => h.startsWith(`site-${seed}-`)).sort();
  const lastT = mine.length ? statSync(join(HIST, mine[mine.length - 1])).mtimeMs : 0;
  if (Date.now() - lastT > 10 * 60 * 1000) {
    const stamp = new Date().toISOString().replace(/[:.]/g, '-');
    writeFileSync(join(HIST, `site-${seed}-${stamp}.bin`), buf);
    mine.push('new');
  }
  const all = readdirSync(HIST).filter((h) => h.startsWith(`site-${seed}-`)).sort();
  for (const old of all.slice(0, Math.max(0, all.length - 10))) unlinkSync(join(HIST, old));
}

function handler(req, res, next) {
  if (!req.url || !req.url.startsWith('/api/memory')) return next();
  const send = (code, body, type = 'application/json') => { res.statusCode = code; res.setHeader('Content-Type', type); res.setHeader('Cache-Control', 'no-store'); res.end(body); };
  const path = req.url.split('?')[0].replace(/\/+$/, '');
  try {
    if (path === '/api/memory/sites' && req.method === 'GET') return send(200, JSON.stringify({ dir: DIR, sites: sites() }));
    const m = path.match(/^\/api\/memory\/(\d{1,12})$/);
    if (!m) return send(404, JSON.stringify({ error: 'unknown memory route' }));
    const seed = Number(m[1]);
    if (req.method === 'GET') {
      const f = fileOf(seed);
      if (!existsSync(f)) return send(404, JSON.stringify({ error: 'no memory for this site' }));
      return send(200, readFileSync(f), 'application/octet-stream');
    }
    if (req.method === 'PUT' || req.method === 'POST') {
      readBody(req).then((buf) => {
        if (buf.length < 8) return send(400, JSON.stringify({ error: 'empty' }));
        save(seed, buf);
        send(200, JSON.stringify({ ok: true, seed, bytes: buf.length, dir: DIR }));
      }).catch((e) => send(413, JSON.stringify({ error: String(e.message || e) })));
      return;
    }
    if (req.method === 'DELETE') { const f = fileOf(seed); if (existsSync(f)) unlinkSync(f); return send(200, JSON.stringify({ ok: true })); }
    return send(405, JSON.stringify({ error: 'method not allowed' }));
  } catch (e) { return send(500, JSON.stringify({ error: String(e.message || e) })); }
}

export function memoryPlugin() {
  return {
    name: 'hunch22-memory-store',
    configureServer(server) { server.middlewares.use(handler); },
    configurePreviewServer(server) { server.middlewares.use(handler); },
  };
}
export const MEMORY_DIR = DIR;
