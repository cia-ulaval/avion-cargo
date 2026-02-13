import os
import shutil
from pathlib import Path

from loguru import logger

PATHS_MUST_BE_SKIPPED: list[str] = [".git", ".github", "docs"]


class ProjectCleaner:
    def __init__(self, project_root_path: Path) -> None:
        self.project_root_path = project_root_path

    @staticmethod
    def __is_only_pycache(path: Path | str) -> bool:
        try:
            items = os.listdir(path)
            return len(items) == 1 and items[0] == "__pycache__"

        except Exception:
            return False

    @staticmethod
    def __should_skip__(path: str) -> bool:
        for pathname in PATHS_MUST_BE_SKIPPED:
            if pathname in path.split(os.sep):
                return True
        return False

    @staticmethod
    def __is_empty_dir__(path: Path | str) -> bool:
        try:
            return os.path.isdir(path) and not os.listdir(path)

        except Exception:
            return False

    def __remove_pycache_and_empty_parents(self, root_path: Path | str) -> None:
        for dirpath, dirnames, filenames in os.walk(root_path, topdown=False):
            if self.__should_skip__(dirpath):
                continue

            if "__pycache__" in dirnames:
                pycache_path = os.path.join(dirpath, "__pycache__")
                logger.info(f"Removing {pycache_path}")
                shutil.rmtree(pycache_path)

            if self.__is_only_pycache(dirpath):
                logger.info(f"Removing parent of __pycache__: {dirpath}")
                shutil.rmtree(dirpath, ignore_errors=True)

            elif self.__is_empty_dir__(dirpath):
                logger.info(f"Removing empty directory: {dirpath}")
                shutil.rmtree(dirpath, ignore_errors=True)

    def clean(self):
        self.__remove_pycache_and_empty_parents(self.project_root_path)
