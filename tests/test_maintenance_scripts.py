import os
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize("script", ["code-formatting.sh", "code-format-checking.sh"])
def test_format_scripts_stop_when_first_tool_fails(tmp_path, script):
    tool_dir = tmp_path / "bin"
    tool_dir.mkdir()
    marker = tmp_path / "later-tool-ran"
    for name in ("isort", "black", "flake8", "ruff"):
        tool = tool_dir / name
        tool.write_text("#!/bin/sh\nexit 17\n" if name == "isort" else f'#!/bin/sh\ntouch "{marker}"\n')
        tool.chmod(0o755)
    script_path = Path(__file__).resolve().parents[1] / "scripts" / script
    result = subprocess.run(
        ["bash", str(script_path)], env={**os.environ, "PATH": f"{tool_dir}:{os.environ['PATH']}"}, cwd=tmp_path
    )
    assert result.returncode == 17
    assert not marker.exists()
