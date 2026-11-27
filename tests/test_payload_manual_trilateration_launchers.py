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


def test_trilateration_ui_launcher_runs_from_the_separate_ui_directory(tmp_path):
    root = tmp_path / "workspace"
    ui = root / "src" / "payload_manual_trilateration" / "ui"
    tauri = ui / "node_modules" / ".bin" / "tauri"
    tauri.parent.mkdir(parents=True)
    tauri.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    tauri.chmod(0o755)

    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    call_log = tmp_path / "npm-call.txt"
    npm = bin_dir / "npm"
    npm.write_text(
        '#!/usr/bin/env bash\nprintf "cwd=%s\\n" "$PWD" > "$NPM_CALL_LOG"\nprintf "%s\\n" "$@" >> "$NPM_CALL_LOG"\n',
        encoding="utf-8",
    )
    npm.chmod(0o755)

    result = subprocess.run(
        [str(REPO_ROOT / "docker" / "launchers" / "avion-payload-manual-trilateration-ui")],
        cwd=REPO_ROOT,
        env={
            **os.environ,
            "PATH": f"{bin_dir}:{os.environ['PATH']}",
            "NPM_CALL_LOG": str(call_log),
            "AVION_CARGO_ROOT": str(root),
        },
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert call_log.read_text(encoding="utf-8").splitlines() == [
        f"cwd={ui}",
        "run",
        "tauri",
        "--",
        "dev",
    ]
