# Shared Avion UI build scaffold

This directory owns the common build infrastructure for the Tauri/Vue entry points:
one Node dependency installation, Vite and TypeScript configuration, a Cargo workspace
with shared dependency versions, a shared Tauri build script, common icons, and the
base Tauri build configuration.

Application code stays separate. Each entry point owns its Vue components and styles,
Rust controller bridge, thin Cargo package manifest, and a small `tauri.identity.json`
overlay containing its name, identifier, and window metadata. The runner merges that
overlay with `src-tauri/tauri.base.conf.json` into an ignored app-local
`src-tauri/tauri.conf.json` before launching Tauri.

Run either frontend with:

```bash
./src/shared/ui/run.sh payload_manual
./src/shared/ui/run.sh payload_manual_trilateration
```

Dependencies are installed once under this directory. The runner links that
installation into the selected frontend for module resolution and sets
`AVION_UI_APP_DIR` so the shared Vite configuration builds the selected app's
source into the common `dist` directory.
