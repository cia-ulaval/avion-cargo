import os
import shlex
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = REPO_ROOT / "scripts" / "launch-sitl.sh"


@pytest.fixture(params=["checkout", "docker"])
def sitl(tmp_path, request):
    root = tmp_path / "ardupilot checkout"
    source = root / "libraries/AP_Scripting/applets/plane_precland.lua"
    source.parent.mkdir(parents=True)
    source.write_text("-- bundled QuadPlane precision landing applet\n")
    target = root / "scripts/plane_precland.lua"
    invocation = tmp_path / "sitl-invocation"
    simulator = root / "Tools/autotest/sim_vehicle.py"
    simulator.parent.mkdir(parents=True)
    simulator.write_text('#!/bin/bash\nprintf "%s\\n" "$PWD" "$@" > "$SITL_INVOCATION"\nexit "${SITL_EXIT_CODE:-0}"\n')
    simulator.chmod(0o755)

    launcher = LAUNCHER
    if request.param == "docker":
        # Execute the wrapper shipped and mounted by Docker, relocating only the
        # path so this integration test also works outside Docker.
        wrapper = (REPO_ROOT / "docker" / "avion-sitl.sh").read_text()
        launcher = tmp_path / "avion-sitl"
        launcher.write_text(
            wrapper.replace("/workspace/avion-cargo/scripts/launch-sitl.sh", shlex.quote(str(LAUNCHER)))
        )

    env = {key: value for key, value in os.environ.items() if not key.startswith("AVION_SITL_")}
    env.update(
        AVION_ARDUPILOT_ROOT=str(root),
        AVION_SITL_VEHICLE="ArduPlane",
        AVION_SITL_FRAME="quadplane-tilttrivec",
        AVION_SITL_MODEL="JSON",
        AVION_SITL_PARAM_FILE=str(tmp_path / "custom parameters.parm"),
        SITL_INVOCATION=str(invocation),
    )

    def run(**overrides):
        return subprocess.run(
            ["bash", str(launcher)], cwd=tmp_path, env={**env, **overrides}, capture_output=True, text=True
        )

    return SimpleNamespace(root=root, source=source, target=target, invocation=invocation, env=env, run=run)


@pytest.mark.parametrize("broken_link", [False, True])
def test_plane_repairs_missing_applet_before_launch_and_is_idempotent(sitl, broken_link):
    if broken_link:
        sitl.target.parent.mkdir()
        sitl.target.symlink_to("missing-old-applet.lua")

    result = sitl.run()

    assert result.returncode == 0, result.stderr
    assert "QuadPlane precision landing script ready" in result.stdout
    assert sitl.target.is_symlink()
    assert sitl.target.resolve() == sitl.source
    assert sitl.invocation.read_text().splitlines() == [
        str(sitl.root),
        "-D",
        "-v",
        "ArduPlane",
        "-f",
        "quadplane-tilttrivec",
        "--model",
        "JSON",
        f"--add-param-file={sitl.env['AVION_SITL_PARAM_FILE']}",
    ]
    inode = sitl.target.lstat().st_ino
    assert sitl.run().returncode == 0
    assert sitl.target.lstat().st_ino == inode


@pytest.mark.parametrize("source_state", ["missing", "empty"])
def test_plane_does_not_start_without_bundled_applet(sitl, source_state):
    if source_state == "missing":
        sitl.source.unlink()
    else:
        sitl.source.write_text("")

    result = sitl.run()

    assert result.returncode != 0
    assert "precision landing applet missing or unreadable" in result.stderr
    assert not sitl.invocation.exists()
    assert not sitl.target.exists()


@pytest.mark.parametrize("custom_symlink", [False, True])
def test_plane_preserves_custom_script_and_stops_with_actionable_error(sitl, custom_symlink):
    sitl.target.parent.mkdir()
    if custom_symlink:
        custom = sitl.root / "custom.lua"
        custom.write_text("-- custom control\n")
        sitl.target.symlink_to(custom)
    else:
        sitl.target.write_text("-- custom control\n")

    result = sitl.run()

    assert result.returncode != 0
    assert "Inspect and move the custom script" in result.stderr
    assert sitl.target.read_text() == "-- custom control\n"
    assert not sitl.invocation.exists()


def test_plane_accepts_existing_copy_of_bundled_applet(sitl):
    sitl.target.parent.mkdir()
    sitl.target.write_bytes(sitl.source.read_bytes())

    assert sitl.run().returncode == 0
    assert not sitl.target.is_symlink()


def test_copter_does_not_require_plane_applet(sitl):
    sitl.source.unlink()

    result = sitl.run(
        AVION_SITL_VEHICLE="ArduCopter", AVION_SITL_FRAME="JSON", AVION_SITL_MODEL="", AVION_SITL_PARAM_FILE=""
    )

    assert result.returncode == 0, result.stderr
    assert sitl.invocation.read_text().splitlines() == [str(sitl.root), "-D", "-v", "ArduCopter", "-f", "JSON"]
    assert not sitl.target.exists()


def test_launcher_propagates_simulator_failure(sitl):
    assert sitl.run(SITL_EXIT_CODE="23").returncode == 23
