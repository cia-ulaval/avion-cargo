from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ValidationError(ValueError):
    message: str
    path: str = "root"

    def __str__(self) -> str:
        return f"[config:{self.path}] {self.message}"