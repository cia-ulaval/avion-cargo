from vtol import composition
from vtol.domain.camera_mount import CameraMount
from vtol.domain.models import TargetedMarker
from vtol.infrastructure.d3kble.configuration import VtolRuntimeConfiguration
from vtol.infrastructure.persistence.configuration_models import (
    AutolanderConfiguration,
    CameraConfiguration,
    DroneConnectionConfiguration,
    StreamingConfiguration,
    StreamingDataConfiguration,
    StreamingVideoConfiguration,
)
from vtol.infrastructure.persistence.vtol_application_configuration import VtolApplicationConfiguration


class FakeD3kbleApplication:
    nodes = ()
    payload_peers = ()

    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class FakeDrone:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def make_configuration():
    autolanding = AutolanderConfiguration(
        targeted_marker=TargetedMarker(id=1, length=0.2),
        camera_config=CameraConfiguration(id=0, use_picamera=False, fps=30, mount=CameraMount()),
        streaming_config=StreamingConfiguration(
            port=8085,
            data=StreamingDataConfiguration(dps=5),
            video=StreamingVideoConfiguration(fps=15),
        ),
        drone_connection_config=DroneConnectionConfiguration(
            use_serial=False,
            address="127.0.0.1",
            port=14550,
            baud_rate=57600,
        ),
    )
    return VtolApplicationConfiguration(autolanding, VtolRuntimeConfiguration(devices=()))


def test_build_vtol_application_composes_one_mavlink_source_with_d3k_and_mission(monkeypatch):
    d3kble = FakeD3kbleApplication()
    drone = FakeDrone()
    monkeypatch.setattr(composition, "build_vtol_d3kble_application", lambda config, node_factory=None: d3kble)

    application = composition.build_vtol_application(
        make_configuration(),
        drone_factory=lambda config: drone,
    )

    assert application.drone is drone
    assert application.d3kble is d3kble
    assert application.runtime.flight_controller.drone is drone
    assert application.runtime.mission_orchestrator.current_state_name == "idle"
    assert application.runtime.payload_communication.payload_peers == ()

    application.close()
    application.close()
    assert drone.closed
    assert d3kble.closed


def test_run_vtol_remains_the_single_vtol_cli_entry_point():
    import tomllib
    from pathlib import Path

    project = tomllib.loads(Path("pyproject.toml").read_text())
    scripts = project["tool"]["poetry"]["scripts"]
    assert scripts["run_vtol"] == "vtol.main:main"
    assert sum(value.startswith("vtol.main:") for value in scripts.values()) == 1
