#!/usr/bin/env python3
"""Generate an app-local Tauri config from the shared base and app identity."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any


def merge(base: Any, override: Any) -> Any:
    if isinstance(base, dict) and isinstance(override, dict):
        result = dict(base)
        for key, value in override.items():
            result[key] = merge(result[key], value) if key in result else value
        return result
    # Scalars and arrays are replaced by the app-specific override.
    return override


def main() -> int:
    if len(sys.argv) != 3:
        print(f"Usage: {Path(sys.argv[0]).name} IDENTITY_JSON OUTPUT_JSON", file=sys.stderr)
        return 2

    shared_dir = Path(__file__).resolve().parent
    base_path = shared_dir / "src-tauri" / "tauri.base.conf.json"
    identity_path = Path(sys.argv[1]).resolve()
    output_path = Path(sys.argv[2]).resolve()

    with base_path.open(encoding="utf-8") as stream:
        base = json.load(stream)
    with identity_path.open(encoding="utf-8") as stream:
        identity = json.load(stream)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(merge(base, identity), indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
