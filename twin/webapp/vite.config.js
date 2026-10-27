import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// Relative asset paths: dist/ works from any static server and any host.
export default defineConfig({
  base: './',
  plugins: [react()],
  server: { port: 3000 },
  build: { chunkSizeWarningLimit: 2000 },
});
