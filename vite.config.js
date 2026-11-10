import { defineConfig } from 'vite';

// AI copilot (src/ai/copilot.js): the page calls /ollama/..., proxied to Ollama on the team Mac mini over Tailscale
// (`tailscale serve --bg --tcp 11434 tcp://127.0.0.1:11434` on the Mac mini). Override with OLLAMA_URL, e.g.
// OLLAMA_URL=http://127.0.0.1:11434 npm run dev. The Origin header is dropped so Ollama's CORS check never applies.
const OLLAMA_URL = process.env.OLLAMA_URL || 'http://100.92.134.100:11434';
const proxy = {
  '/ollama': {
    target: OLLAMA_URL,
    rewrite: (p) => p.replace(/^\/ollama/, ''),
    configure: (px) => px.on('proxyReq', (req) => req.removeHeader('origin')),
  },
};

export default defineConfig({
  build: { chunkSizeWarningLimit: 4000, target: 'es2022' },
  optimizeDeps: { exclude: ['@dimforge/rapier3d-compat'] },
  server: { host: '127.0.0.1', port: 5173, proxy },
  preview: { proxy },
});
