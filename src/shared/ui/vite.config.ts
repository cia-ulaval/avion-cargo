import { resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";

const sharedUiRoot = fileURLToPath(new URL(".", import.meta.url));
// npm executes workspace scripts from the selected UI package directory.
const appRoot = resolve(process.cwd());

export default defineConfig({
  root: appRoot,
  envDir: appRoot,
  plugins: [vue()],
  clearScreen: false,
  server: {
    host: "0.0.0.0",
    port: 1421,
    strictPort: true,
    fs: { allow: [appRoot, sharedUiRoot] },
  },
  build: {
    outDir: resolve(sharedUiRoot, "dist"),
    emptyOutDir: true,
  },
});
