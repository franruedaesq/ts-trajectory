import { defineConfig } from 'tsup';

export default defineConfig({
  entry: ['src/index.ts'],
  format: ['cjs', 'esm'],
  dts: true,
  sourcemap: true,
  clean: true,
  // Copy WASM assets so they are available alongside the bundled output.
  // The bundler-target pkg/ is used for ESM/web, pkg-node/ is used for CJS/Node.js.
  publicDir: false,
});
