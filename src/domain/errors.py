class DomainError(Exception):
    """Base class for domain-level errors."""


class InvalidMarkerLengthError(DomainError):
    pass


class InvalidCalibrationError(DomainError):
    def __init__(self, msg) -> None:
        super().__init__(msg)
        pass


class InvalidPoseError(DomainError):
    pass
