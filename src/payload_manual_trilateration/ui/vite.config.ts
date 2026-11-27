import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";

export default defineConfig({
  plugins: [vue()],
  clearScreen: false,
  server: { host: "0.0.0.0", port: 1421, strictPort: true },
});
