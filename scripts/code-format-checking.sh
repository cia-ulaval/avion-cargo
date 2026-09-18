#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
isort --settings-path pyproject.toml --check-only .
black --check --verbose .
flake8 .
ruff check .
