import { resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";

const sharedUiRoot = fileURLToPath(new URL(".", import.meta.url));
const appRootValue = process.env.AVION_UI_APP_DIR;
if (!appRootValue) {
  throw new Error("AVION_UI_APP_DIR must point to the selected entry point's ui directory.");
}
const appRoot = resolve(appRootValue);

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
