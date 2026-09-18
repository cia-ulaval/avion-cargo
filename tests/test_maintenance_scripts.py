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


def test_cleaner_preserves_environments_protected_trees_symlinks_and_root(tmp_path):
    from scripts.project_cleaner import ProjectCleaner

    for folder in (".venv", ".git", "docs", "custom-python"):
        cache = tmp_path / folder / "__pycache__"
        cache.mkdir(parents=True)
        (cache / "keep.pyc").write_text("keep")
    (tmp_path / "custom-python" / "pyvenv.cfg").write_text("home = /usr/bin")
    (tmp_path / "source" / "__pycache__").mkdir(parents=True)
    (tmp_path / "source" / "code.py").write_text("pass")
    (tmp_path / "empty").mkdir()
    (tmp_path / "linked-env").symlink_to(tmp_path / ".venv", target_is_directory=True)

    ProjectCleaner(tmp_path).clean()

    assert tmp_path.is_dir()
    assert (tmp_path / "source" / "code.py").exists()
    assert not (tmp_path / "source" / "__pycache__").exists()
    assert not (tmp_path / "empty").exists()
    assert (tmp_path / "linked-env").is_symlink()
    for folder in (".venv", ".git", "docs", "custom-python"):
        assert (tmp_path / folder / "__pycache__" / "keep.pyc").read_text() == "keep"
