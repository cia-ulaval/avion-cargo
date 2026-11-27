# Shared Avion UI build scaffold

This directory owns the common build infrastructure for the Tauri/Vue entry points:
one Node dependency installation, Vite and TypeScript configuration, a Cargo workspace
with shared dependency versions, a shared Tauri build script, common icons, and the
base Tauri build configuration. The two Vue applications are npm workspaces rooted here.

Application code stays separate. Each entry point owns its Vue components and styles,
Rust controller bridge, thin Cargo package manifest, and a small `tauri.identity.json`
overlay containing its name, identifier, and window metadata. Tauri's native `--config`
option merges the shared base configuration and the app-specific identity overlay when
the application is launched.

Install dependencies once from this directory, then run either frontend with:

```bash
npm install
npm --workspace payload-manual-ui run tauri:dev
npm --workspace payload-manual-trilateration-ui run tauri:dev
```

Dependencies are installed once under this directory and shared by both workspaces.
Each workspace runs Vite from its own UI directory; the shared Vite configuration
builds the selected app's source into the common `dist` directory.
