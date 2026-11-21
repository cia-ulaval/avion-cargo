from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from vtol.infrastructure.d3kble.configuration import VtolRuntimeConfiguration
from vtol.infrastructure.persistence.autolander_configuration_reader import AutolanderConfigurationReader
from vtol.infrastructure.persistence.configuration_models import AutolanderConfiguration


@dataclass(frozen=True, slots=True)
class VtolApplicationConfiguration:
    """
    Unified configuration for all software running on the VTOL's Pi.
    """

    autolanding: AutolanderConfiguration
    d3kble: VtolRuntimeConfiguration

    @classmethod
    def from_file(cls, path: str | Path) -> VtolApplicationConfiguration:
        config_path = Path(path)
        return cls(
            autolanding=AutolanderConfigurationReader(config_path).read(),
            d3kble=VtolRuntimeConfiguration.from_file(config_path),
        )
