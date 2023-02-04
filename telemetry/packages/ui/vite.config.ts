import { resolve } from 'path';
import { defineConfig } from 'vite';
// import react from '@vitejs/plugin-react-swc'

// https://vitejs.dev/config/
export default defineConfig({
  build: {
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'app/index.html'),
        openmct: resolve(__dirname, 'openmct/index.html'),
      },
      onwarn(warning, warn) {
        if (warning.code === 'EVAL') return;
        warn(warning);
      },
    },
  },
});
