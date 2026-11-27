import json
import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_launcher_installer_links_all_source_launchers(tmp_path):
    bin_dir = tmp_path / "bin"
    result = subprocess.run(
        [str(REPO_ROOT / "docker" / "install-launchers.sh")],
        cwd=REPO_ROOT,
        env={
            **os.environ,
            "AVION_CARGO_ROOT": str(REPO_ROOT),
            "AVION_LAUNCHER_BIN_DIR": str(bin_dir),
        },
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    names = [
        "avion-sitl",
        "avion-gazebo",
        "avion-image-bridge",
        "avion-autolander",
        "avion-entrypoint",
        "avion-web-proxy",
        "avion-vtol",
        "avion-payload",
        "avion-payload-manual",
        "avion-payload-manual-trilateration",
        "avion-payload-manual-ui",
        "avion-payload-manual-trilateration-ui",
        "avion-d3k-diagnostics",
    ]
    for name in names:
        link = bin_dir / name
        assert link.is_symlink()
        assert link.resolve() == REPO_ROOT / "docker" / "launchers" / name
        assert link.resolve().is_file()


def test_trilateration_launcher_uses_its_own_poetry_entrypoint(tmp_path):
    config = tmp_path / "manual-payload.json"
    config.write_text(json.dumps({"payload_id": "test"}), encoding="utf-8")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    call_log = tmp_path / "poetry-args.txt"
    poetry = bin_dir / "poetry"
    poetry.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "$POETRY_CALL_LOG"\n')
    poetry.chmod(0o755)

    result = subprocess.run(
        [str(REPO_ROOT / "docker" / "launchers" / "avion-payload-manual-trilateration")],
        cwd=REPO_ROOT,
        env={
            **os.environ,
            "PATH": f"{bin_dir}:{os.environ['PATH']}",
            "POETRY_CALL_LOG": str(call_log),
            "AVION_PAYLOAD_MANUAL_TRILATERATION_CONFIG": str(config),
            "AVION_CARGO_ROOT": str(REPO_ROOT),
        },
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert call_log.read_text(encoding="utf-8").splitlines() == [
        "run",
        "manual_payload_trilateration",
        "--config",
        str(config),
    ]


def test_trilateration_compose_config_path_exists():
    compose = (REPO_ROOT / "compose-payload-manual-trilateration.yaml").read_text(encoding="utf-8")
    config_path = REPO_ROOT / "simulation" / "config" / "manual_payload_trilateration_config.json"

    assert config_path.is_file()
    assert "/simulation/config/manual_payload_trilateration_config.json" in compose
    assert "avion-payload-manual-trilateration-ui" in compose


def test_poetry_registers_the_independent_trilateration_package_and_entrypoint():
    pyproject = (REPO_ROOT / "pyproject.toml").read_text(encoding="utf-8")

    assert '{ include = "payload_manual_trilateration", from = "src" }' in pyproject
    assert 'manual_payload_trilateration = "payload_manual_trilateration.main:main"' in pyproject
    assert 'run_payload_manual = "payload_manual.main:main"' in pyproject


def test_entrypoint_ui_launchers_use_the_shared_tauri_scaffold(tmp_path):
    root = tmp_path / "workspace"
    shared_ui = root / "src" / "shared" / "ui"
    shared_ui.mkdir(parents=True)
    (shared_ui / "run.sh").write_text(
        (REPO_ROOT / "src" / "shared" / "ui" / "run.sh").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    (shared_ui / "run.sh").chmod(0o755)
    (shared_ui / "package.json").write_text(
        (REPO_ROOT / "src" / "shared" / "ui" / "package.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    (shared_ui / "merge-tauri-config.py").write_text(
        (REPO_ROOT / "src" / "shared" / "ui" / "merge-tauri-config.py").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    shared_tauri = shared_ui / "src-tauri"
    shared_tauri.mkdir()
    (shared_tauri / "Cargo.toml").write_text(
        (REPO_ROOT / "src" / "shared" / "ui" / "src-tauri" / "Cargo.toml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    (shared_tauri / "tauri.base.conf.json").write_text(
        (REPO_ROOT / "src" / "shared" / "ui" / "src-tauri" / "tauri.base.conf.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    tauri = shared_ui / "node_modules" / ".bin" / "tauri"
    tauri.parent.mkdir(parents=True)
    tauri.write_text(
        '#!/usr/bin/env bash\nprintf "cwd=%s\\napp=%s\\n" "$PWD" "$AVION_UI_APP_DIR" > "$TAURI_CALL_LOG"\nprintf "%s\\n" "$@" >> "$TAURI_CALL_LOG"\n',
        encoding="utf-8",
    )
    tauri.chmod(0o755)

    for launcher_name, entrypoint in [
        ("avion-payload-manual-ui", "payload_manual"),
        ("avion-payload-manual-trilateration-ui", "payload_manual_trilateration"),
    ]:
        app_ui = root / "src" / entrypoint / "ui"
        app_tauri = app_ui / "src-tauri"
        (app_ui / "src").mkdir(parents=True)
        (app_tauri / "src").mkdir(parents=True)
        (app_ui / "index.html").write_text("<div id=app></div>", encoding="utf-8")
        (app_ui / "src" / "App.vue").write_text("<template><main /></template>", encoding="utf-8")
        (app_ui / "src" / "main.ts").write_text("", encoding="utf-8")
        (app_ui / "src" / "style.css").write_text("", encoding="utf-8")
        identity = {
            "productName": entrypoint,
            "identifier": f"ca.ulaval.cia.{entrypoint}",
            "app": {"windows": [{"label": "main", "title": entrypoint}]},
        }
        (app_tauri / "tauri.identity.json").write_text(json.dumps(identity), encoding="utf-8")
        (app_tauri / "Cargo.toml").write_text("[package]\nname='test'\n", encoding="utf-8")
        (app_tauri / "src" / "lib.rs").write_text("", encoding="utf-8")
        call_log = tmp_path / f"{entrypoint}-tauri-call.txt"
        result = subprocess.run(
            [str(REPO_ROOT / "docker" / "launchers" / launcher_name)],
            cwd=REPO_ROOT,
            env={
                **os.environ,
                "TAURI_CALL_LOG": str(call_log),
                "AVION_CARGO_ROOT": str(root),
            },
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr
        lines = call_log.read_text(encoding="utf-8").splitlines()
        assert lines == [
            f"cwd={app_ui}",
            f"app={app_ui}",
            "dev",
        ]
        generated_config = json.loads((app_tauri / "tauri.conf.json").read_text(encoding="utf-8"))
        assert generated_config["productName"] == entrypoint
        assert generated_config["identifier"] == f"ca.ulaval.cia.{entrypoint}"
        assert generated_config["app"]["windows"][0]["title"] == entrypoint
        assert generated_config["build"]["devUrl"] == "http://localhost:1421"
        assert "npm --prefix ../../shared/ui run dev" in generated_config["build"]["beforeDevCommand"]
        assert generated_config["build"]["frontendDist"] == "../../../shared/ui/dist"
        assert (app_ui / "node_modules").is_symlink()
        assert (app_ui / "node_modules").resolve() == (shared_ui / "node_modules").resolve()


def test_simulation_launcher_checks_the_new_tauri_config_locations():
    launcher = (REPO_ROOT / "scripts" / "launch-simulation.sh").read_text(encoding="utf-8")

    assert "/src/payload_manual/ui/src-tauri/tauri.identity.json" in launcher
    assert "/src/payload_manual_trilateration/ui/src-tauri/tauri.identity.json" in launcher
    assert "/src/payload_manual/ui/tauri.conf.json" not in launcher
    assert "/src/payload_manual_trilateration/ui/tauri.conf.json" not in launcher


def test_entrypoint_ui_keeps_vue_and_rust_code_separate_but_shares_build_scaffolding():
    shared_ui = REPO_ROOT / "src" / "shared" / "ui"
    shared_manifest = (shared_ui / "src-tauri" / "Cargo.toml").read_text(encoding="utf-8")
    assert (shared_ui / "package.json").is_file()
    assert (shared_ui / "package-lock.json").is_file()
    assert (shared_ui / "vite.config.ts").is_file()
    assert (shared_ui / "tsconfig.json").is_file()
    assert (shared_ui / "src-tauri" / "build.rs").is_file()
    assert (shared_ui / "src-tauri" / "Cargo.lock").is_file()
    assert (shared_ui / "src-tauri" / "tauri.base.conf.json").is_file()
    assert (shared_ui / "src-tauri" / "icons" / "icon.png").is_file()
    assert (shared_ui / "merge-tauri-config.py").is_file()
    assert "[workspace.dependencies]" in shared_manifest
    assert "trilateration = []" not in shared_manifest
    assert not (shared_ui / "src-tauri" / "src").exists()
    assert not (shared_ui / "style.css").exists()

    rust_sources = []
    vue_sources = []
    for entrypoint in ("payload_manual", "payload_manual_trilateration"):
        ui_dir = REPO_ROOT / "src" / entrypoint / "ui"
        tauri_dir = ui_dir / "src-tauri"
        assert (ui_dir / "index.html").is_file()
        assert (ui_dir / "src" / "App.vue").is_file()
        assert (ui_dir / "src" / "main.ts").is_file()
        assert (ui_dir / "src" / "style.css").is_file()
        assert (tauri_dir / "tauri.identity.json").is_file()
        assert (tauri_dir / "Cargo.toml").is_file()
        assert (tauri_dir / "build.rs").is_file()
        assert (tauri_dir / "src" / "lib.rs").is_file()
        assert (tauri_dir / "src" / "main.rs").is_file()
        assert (tauri_dir / "src" / "d3k_protocol.rs").is_file()
        assert not (ui_dir / "package.json").exists()
        assert not (ui_dir / "package-lock.json").exists()
        assert not (ui_dir / "vite.config.ts").exists()
        assert not (tauri_dir / "Cargo.lock").exists()
        assert not (tauri_dir / "icons").exists()
        assert not (tauri_dir / "capabilities").exists()
        assert "import './style.css';" in (ui_dir / "src" / "main.ts").read_text(encoding="utf-8")
        identity = json.loads((tauri_dir / "tauri.identity.json").read_text(encoding="utf-8"))
        assert identity["identifier"].startswith("ca.ulaval.cia.")
        rust_sources.append((tauri_dir / "src" / "lib.rs").read_text(encoding="utf-8"))
        vue_sources.append((ui_dir / "src" / "App.vue").read_text(encoding="utf-8"))

    assert rust_sources[0] != rust_sources[1]
    assert vue_sources[0] != vue_sources[1]

    legacy_manifest = (REPO_ROOT / "src" / "payload_manual" / "ui" / "src-tauri" / "Cargo.toml").read_text(encoding="utf-8")
    trilateration_manifest = (REPO_ROOT / "src" / "payload_manual_trilateration" / "ui" / "src-tauri" / "Cargo.toml").read_text(encoding="utf-8")
    assert 'name = "payload-manual-controller"' in legacy_manifest
    assert 'name = "payload-manual-trilateration-controller"' in trilateration_manifest
    assert "tauri.workspace = true" in legacy_manifest
    assert "tauri.workspace = true" in trilateration_manifest
    assert "trilateration = []" not in legacy_manifest
    assert "trilateration = []" not in trilateration_manifest
    assert "payload_manual_controller_lib::run()" in (
        REPO_ROOT / "src" / "payload_manual" / "ui" / "src-tauri" / "src" / "main.rs"
    ).read_text(encoding="utf-8")
    assert "payload_manual_trilateration_controller_lib::run()" in (
        REPO_ROOT / "src" / "payload_manual_trilateration" / "ui" / "src-tauri" / "src" / "main.rs"
    ).read_text(encoding="utf-8")

    base_config = json.loads((shared_ui / "src-tauri" / "tauri.base.conf.json").read_text(encoding="utf-8"))
    assert "features" not in base_config.get("build", {})
    assert base_config["build"]["frontendDist"] == "../../../shared/ui/dist"
    assert base_config["build"]["beforeDevCommand"] == "npm --prefix ../../shared/ui run dev"
    assert (shared_ui / "merge-tauri-config.py").is_file()
