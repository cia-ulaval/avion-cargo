from os import system
from pathlib import Path

from loguru import logger

from scripts.project_cleaner import ProjectCleaner


def run_code_formatting():
    system("./scripts/code-formatting.sh")


def run_code_format_checking():
    system("./scripts/code-format-checking.sh")


def clean_project():
    project_cleaner = ProjectCleaner(Path("."))
    logger.info("Start cleaning project")
    project_cleaner.clean()
    logger.success("Project cleaned")
