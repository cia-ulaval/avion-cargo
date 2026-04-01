import subprocess
from pathlib import Path

from loguru import logger

from scripts.project_cleaner import ProjectCleaner

REPO_ROOT = Path(__file__).resolve().parent
SCRIPTS_DIR = REPO_ROOT / "scripts"


def _run_shell_script(path: Path) -> None:
    if not path.exists():
        raise FileNotFoundError(f"Script not found: {path}")

    subprocess.run(["bash", str(path)], check=True, cwd=str(REPO_ROOT))


def run_code_formatting() -> None:
    _run_shell_script(SCRIPTS_DIR / "code-formatting.sh")


def run_code_format_checking() -> None:
    _run_shell_script(SCRIPTS_DIR / "code-format-checking.sh")


def clean_project() -> None:
    project_cleaner = ProjectCleaner(REPO_ROOT)
    logger.info("Start cleaning project")
    project_cleaner.clean()
    logger.success("Project cleaned")

if __name__ == "__main__":
    clean_project()