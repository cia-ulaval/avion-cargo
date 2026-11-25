import os
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DOCKERFILE = REPO_ROOT / "Dockerfile"


@pytest.fixture
def sitl(tmp_path):
    root = tmp_path / "ardupilot checkout"
    source = root / "libraries/AP_Scripting/applets/plane_precland.lua"
    source.parent.mkdir(parents=True)
    source.write_text("-- bundled QuadPlane precision landing applet\n")
    (source.parent / "qland_yaw_alignment.lua").write_text("local enabled = false\n")
    target = root / "scripts/plane_precland.lua"
    invocation = tmp_path / "sitl-invocation"
    simulator = root / "Tools/autotest/sim_vehicle.py"
    simulator.parent.mkdir(parents=True)
    simulator.write_text('#!/bin/bash\nprintf "%s\\n" "$PWD" "$@" > "$SITL_INVOCATION"\nexit "${SITL_EXIT_CODE:-0}"\n')
    simulator.chmod(0o755)

    env = {key: value for key, value in os.environ.items() if not key.startswith("AVION_")}
    _, start, remainder = DOCKERFILE.read_text().partition("cat > /usr/local/bin/avion-sitl <<'SCRIPT'\n")
    script, end, _ = remainder.partition("\nSCRIPT\n")
    assert start and end, "Dockerfile must define the avion-sitl launcher"
    launcher = tmp_path / "image/usr/local/bin/avion-sitl"
    launcher.parent.mkdir(parents=True)
    launcher.write_text(script + "\n")
    launcher.chmod(0o755)

    env.update(
        AVION_ARDUPILOT_ROOT=str(root),
        AVION_SITL_VEHICLE="ArduPlane",
        AVION_SITL_FRAME="quadplane-tilttrivec",
        AVION_SITL_MODEL="JSON",
        AVION_SITL_PARAM_FILE=str(tmp_path / "custom parameters.parm"),
        SITL_INVOCATION=str(invocation),
    )

    def run(**overrides):
        environment = {key: value for key, value in {**env, **overrides}.items() if value is not None}
        return subprocess.run([str(launcher)], cwd=tmp_path, env=environment, capture_output=True, text=True)

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


def test_plane_installs_the_yaw_script_from_the_image(sitl):
    result = sitl.run()
    assert result.returncode == 0, result.stderr
    target = sitl.root / "scripts/qland_yaw_alignment.lua"
    assert target.resolve() == sitl.source.parent / "qland_yaw_alignment.lua"
    assert "QuadPlane QLAND yaw alignment script ready" in result.stdout


def test_plane_requires_the_image_yaw_script(sitl):
    (sitl.source.parent / "qland_yaw_alignment.lua").unlink()
    result = sitl.run()
    assert result.returncode != 0
    assert "QLAND yaw alignment applet missing or unreadable" in result.stderr
    assert not sitl.invocation.exists()


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


def test_default_parameters_resolve_from_the_workspace(sitl):
    result = sitl.run(AVION_SITL_PARAM_FILE=None)

    assert result.returncode == 0, result.stderr
    assert sitl.invocation.read_text().splitlines()[-1] == (
        "--add-param-file=/workspace/avion-cargo/simulation/config/gazebo_iris.parm"
    )


@pytest.mark.parametrize("output", [None, "", "tcpin:0.0.0.0:5761"])
def test_launcher_reads_optional_gcs_output_from_its_environment(sitl, output):
    result = sitl.run(AVION_SITL_GCS_OUT=output)

    assert result.returncode == 0, result.stderr
    arguments = sitl.invocation.read_text().splitlines()
    if output:
        assert arguments[-2:] == ["--out", output]
    else:
        assert "--out" not in arguments
