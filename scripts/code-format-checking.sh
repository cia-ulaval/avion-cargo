#! /bin/bash

isort --check-only .
black --check --verbose .
flake8 .