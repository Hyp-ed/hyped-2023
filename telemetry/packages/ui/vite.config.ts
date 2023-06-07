import { resolve } from 'path';
import { defineConfig } from 'vite';
import { viteStaticCopy } from 'vite-plugin-static-copy';

// https://vitejs.dev/config/
export default defineConfig(({ command, mode }) => {
  return {
    optimizeDeps: {
      include: ['@hyped/telemetry-constants'],
    },
    build: {
      rollupOptions: {
        input: {
          main: resolve(__dirname, 'index.html'),
          app: resolve(__dirname, 'app/index.html'),
          openmct: resolve(__dirname, 'openmct/index.html'),
        },
      },
      commonjsOptions: {
        include: [/node_modules/, '@hyped/telemetry-constants'],
      },
    },
    plugins: [
      viteStaticCopy({
        targets: [
          {
            src: '../../node_modules/openmct/dist/*',
            dest: 'openmct-lib',
          },
        ],
      }),
    ],
  };
});
