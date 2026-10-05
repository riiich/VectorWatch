import { defineConfig } from 'vite';
export default defineConfig({
  server: {
    port: 5173, strictPort: true,
    proxy: {
      '/api': 'http://127.0.0.1:5080',
      '/hubs': { target: 'http://127.0.0.1:5080', ws: true },
    },
  },
  build: { outDir: '../server/wwwroot', emptyOutDir: true },
});
