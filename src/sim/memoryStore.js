// Rover memory: the SLAM map and proven routes, saved to the on-disk store served by the Vite dev / preview server
// (scripts/memory-plugin.mjs, ~/.hunch22-data/memory) with the browser's IndexedDB kept as a fallback.
//
// Blob format (gzip-compressed): [u32 header length][header JSON (utf-8)][tile payload]
//   header: { format: 'hunch22-memory', v: 1, seed, savedAt, tiles: [{ tx, tz }], routes }
//   payload: per tile, in header order: h, h0, v (Float32 x 4096 each), then n (Uint8 x 4096); 4-byte aligned
// The per-site file is keyed by the terrain seed: a map only fits the site it was built on (Slam.deserialize checks).
import { TILE } from './slam.js';

const CELLS = TILE * TILE;
const PER_TILE = CELLS * 4 * 3 + CELLS;           // bytes: three Float32 layers + one Uint8 layer

async function gzip(bytes) {
  if (typeof CompressionStream === 'undefined') return bytes;
  return new Uint8Array(await new Response(new Blob([bytes]).stream().pipeThrough(new CompressionStream('gzip'))).arrayBuffer());
}
async function gunzip(bytes) {
  if (bytes[0] !== 0x1f || bytes[1] !== 0x8b) return bytes;  // stored uncompressed
  return new Uint8Array(await new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer());
}

export async function encodeMemory(d) {
  const header = new TextEncoder().encode(JSON.stringify({ format: 'hunch22-memory', v: 1, seed: d.seed, savedAt: d.savedAt, tiles: d.tiles.map((t) => ({ tx: t.tx, tz: t.tz })), routes: d.routes }));
  const hpad = (4 - ((4 + header.length) % 4)) % 4;
  const out = new Uint8Array(4 + header.length + hpad + d.tiles.length * PER_TILE);
  new DataView(out.buffer).setUint32(0, header.length + hpad, true);
  out.set(header, 4); out.fill(0x20, 4 + header.length, 4 + header.length + hpad);   // pad with spaces (still valid JSON)
  let o = 4 + header.length + hpad;
  for (const t of d.tiles) {
    for (const a of [t.h, t.h0, t.v]) { out.set(new Uint8Array(a.buffer, a.byteOffset, CELLS * 4), o); o += CELLS * 4; }
    out.set(t.n, o); o += CELLS;
  }
  return gzip(out);
}

export async function decodeMemory(bytes) {
  const b = await gunzip(new Uint8Array(bytes));
  const hl = new DataView(b.buffer, b.byteOffset).getUint32(0, true);
  const head = JSON.parse(new TextDecoder().decode(b.subarray(4, 4 + hl)));
  if (head.format !== 'hunch22-memory') throw new Error('not a rover memory file');
  let o = 4 + hl;
  const copyF = () => { const a = new Float32Array(CELLS); new Uint8Array(a.buffer).set(b.subarray(o, o + CELLS * 4)); o += CELLS * 4; return a; };
  const tiles = head.tiles.map((t) => { const h = copyF(), h0 = copyF(), v = copyF(); const n = b.slice(o, o + CELLS); o += CELLS; return { tx: t.tx, tz: t.tz, h, h0, v, n }; });
  return { seed: head.seed, savedAt: head.savedAt, tiles, routes: head.routes || [] };
}

// disk store (dev / preview server); every call fails soft so the sim runs without it (e.g. a static build)
export const DiskMemory = {
  async sites() { try { const r = await fetch('/api/memory/sites', { cache: 'no-store' }); if (!r.ok) return null; return await r.json(); } catch (e) { return null; } },
  async load(seed) {
    try { const r = await fetch(`/api/memory/${seed}`, { cache: 'no-store' }); if (!r.ok) return null; return await decodeMemory(await r.arrayBuffer()); } catch (e) { return null; }
  },
  async save(d) {
    try { const body = await encodeMemory(d); const r = await fetch(`/api/memory/${d.seed}`, { method: 'PUT', body, headers: { 'Content-Type': 'application/octet-stream' } }); return r.ok ? await r.json() : null; } catch (e) { return null; }
  },
  // keepalive PUT for page unload (best effort; browsers cap keepalive bodies at 64 kB, so only small maps make it)
  saveBeacon(d) { encodeMemory(d).then((body) => { try { if (body.length < 60000) fetch(`/api/memory/${d.seed}`, { method: 'PUT', body, keepalive: true }); } catch (e) { /* ignore */ } }); },
  async forget(seed) { try { await fetch(`/api/memory/${seed}`, { method: 'DELETE' }); } catch (e) { /* ignore */ } },
};
