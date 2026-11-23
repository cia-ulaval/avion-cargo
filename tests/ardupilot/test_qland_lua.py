import shutil
import subprocess
from pathlib import Path

import pytest


def test_lua_controller():
    lua = shutil.which("lua5.4") or shutil.which("lua")
    if lua is None:
        pytest.skip("Install lua5.4 to run the flight-controller script tests")
    root = Path(__file__).resolve().parents[2]
    result = subprocess.run(
        [
            lua,
            str(root / "tests/ardupilot/qland_yaw_alignment_test.lua"),
            str(root / "ardupilot/scripts/qland_yaw_alignment.lua"),
        ],
        text=True,
        capture_output=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr
