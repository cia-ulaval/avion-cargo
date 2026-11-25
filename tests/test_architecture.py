import ast
import importlib.util
import subprocess
import sys
from pathlib import Path

import pytest

SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
TECHNICAL_MODULES = {
    "vtol.infrastructure",
    "vtol.ui",
    "vtol.composition",
    "vtol.simulation",
    "vtol.utils",
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


LAYER_RULES = (
    (
        "vtol/domain",
        {
            "vtol.application",
            "vtol.infrastructure",
            "vtol.ui",
            "vtol.composition",
            "vtol.simulation",
        },
    ),
    (
        "vtol/application",
        {"vtol.infrastructure", "vtol.ui", "vtol.composition", "vtol.simulation"},
    ),
    (
        "payload/domain",
        {"payload.application", "payload.infrastructure", "payload.composition"},
    ),
    (
        "payload/application",
        {"payload.infrastructure", "payload.composition"},
    ),
    (
        "shared",
        {"vtol", "payload"},
    ),
)


def _is_forbidden(module: str, forbidden: set[str]) -> bool:
    return any(module == prefix or module.startswith(f"{prefix}.") for prefix in forbidden)


@pytest.mark.parametrize(("relative_root", "forbidden"), LAYER_RULES)
def test_core_imports_follow_layer_boundaries(relative_root: str, forbidden: set[str]) -> None:
    violations = []
    layer_root = SOURCE_ROOT / relative_root
    for path in sorted(layer_root.rglob("*.py")):
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
                if _is_forbidden(module, forbidden):
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
for name in ('vtol.domain', 'vtol.application', 'payload.domain', 'payload.application', 'shared.contracts'):
    package = importlib.import_module(name)
    for module in pkgutil.walk_packages(package.__path__, package.__name__ + '.'):
        importlib.import_module(module.name)
"""
    result = subprocess.run(
        [sys.executable, "-c", script, str(SOURCE_ROOT), *sorted(TECHNICAL_MODULES)],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stderr
