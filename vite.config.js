import { defineConfig } from 'vite';
export default defineConfig({
  build: { chunkSizeWarningLimit: 4000, target: 'es2022' },
  optimizeDeps: { exclude: ['@dimforge/rapier3d-compat'] },
  server: { host: '127.0.0.1', port: 5173 },
});
