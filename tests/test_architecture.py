"""Keep technical adapters and presentation outside the application core."""

import ast
import importlib.util
import subprocess
import sys
from pathlib import Path

import pytest

SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
OUTER_MODULES = {
    "infrastructure",
    "ui",
    "composition",
    "simulation",
    "cv2",
    "picamera2",
    "pymavlink",
    "aiohttp",
    "aiortc",
    "av",
    "rclpy",
    "sensor_msgs",
    "cv_bridge",
    "tabulate",
    "click",
}


@pytest.mark.parametrize("layer", ["domain", "application"])
def test_core_imports_follow_layer_boundaries(layer: str) -> None:
    forbidden = OUTER_MODULES | ({"application"} if layer == "domain" else set())
    violations = []
    for path in sorted((SOURCE_ROOT / layer).rglob("*.py")):
        package = ".".join(path.parent.relative_to(SOURCE_ROOT).parts)
        for node in ast.walk(ast.parse(path.read_text(), filename=str(path))):
            if isinstance(node, ast.Import):
                imports = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                module = "." * node.level + (node.module or "")
                imports = [importlib.util.resolve_name(module, package)]
            else:
                continue
            for module in imports:
                if module.split(".")[0] in forbidden:
                    violations.append(f"{path.relative_to(SOURCE_ROOT)}:{node.lineno}: {module}")
    assert not violations, "Forbidden core imports:\n" + "\n".join(violations)


def test_core_can_be_imported_without_technical_adapters_or_ui() -> None:
    script = """
import importlib
import pkgutil
import sys

sys.path.insert(0, sys.argv[1])
for name in sys.argv[2:]:
    sys.modules[name] = None
for name in ('domain', 'application'):
    package = importlib.import_module(name)
    for module in pkgutil.walk_packages(package.__path__, package.__name__ + '.'):
        importlib.import_module(module.name)
"""
    result = subprocess.run(
        [sys.executable, "-c", script, str(SOURCE_ROOT), *sorted(OUTER_MODULES)],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stderr
