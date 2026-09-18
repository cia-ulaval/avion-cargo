import os
import shutil
from pathlib import Path

from loguru import logger

PATHS_MUST_BE_SKIPPED = {".git", ".github", "docs", ".venv", "venv", "env", "ENV", ".tox", ".nox", "node_modules"}


class ProjectCleaner:
    def __init__(self, project_root_path: Path) -> None:
        self.project_root_path = project_root_path.resolve()

    @staticmethod
    def _protected(path: Path) -> bool:
        return path.is_symlink() or path.name in PATHS_MUST_BE_SKIPPED or (path / "pyvenv.cfg").exists()

    def clean(self) -> None:
        root = self.project_root_path
        if self._protected(root):
            logger.info("Skipping protected directory: {}", root)
            return

        # Prune protected trees before traversing them, including custom virtual environments.
        visited = []
        for dirpath, dirnames, _ in os.walk(root, topdown=True, followlinks=False):
            current = Path(dirpath)
            dirnames[:] = [name for name in dirnames if not self._protected(current / name)]
            if "__pycache__" in dirnames:
                cache = current / "__pycache__"
                logger.info("Removing {}", cache)
                shutil.rmtree(cache)
                dirnames.remove("__pycache__")
            visited.append(current)

        for directory in reversed(visited):
            if directory != root and not any(directory.iterdir()):
                logger.info("Removing empty directory: {}", directory)
                directory.rmdir()
