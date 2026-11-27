// Keep Tauri's build logic in src/shared/ui; this wrapper belongs to this app
// because its Rust source and Tauri configuration are entrypoint-specific.
include!("../../../shared/ui/src-tauri/build.rs");
