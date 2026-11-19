from vtol.infrastructure.d3kble.configuration import (
    D3KDeviceConfiguration,
    PayloadPeerConfiguration,
    UwbDeviceConfiguration,
    VtolRuntimeConfiguration,
)
from vtol.infrastructure.d3kble.runtime import build_vtol_d3kble_application


class FakeComms:
    def bind_asyncio_loop(self, loop):
        self.loop = loop

    def on_message(self, callback=None):
        return callback

    def on_request(self, callback=None):
        return callback


class FakeBle:
    def __init__(self):
        self.started = False
        self.stopped = False
        self.on_start = None

    def start_ble(self):
        if self.on_start is not None:
            self.on_start()
        self.started = True

    def stop_ble(self):
        self.stopped = True
        self.started = False


class FakeUwb:
    def __init__(self):
        # These represent settings already configured on the board (for
        # example through d3km) before the application connects.
        self.role = "responder"
        self.targets = (200, 201)
        self.schedule = (100, 120, 1)
        self.ranging_started = False
        self.ranging_stopped = False

    def stop_ranging(self):
        self.ranging_started = False
        self.ranging_stopped = True


class FakeConfigManager:
    def __init__(self, node):
        self.node = node
        self.desired = None
        self.mode = None
        self.approved = None

    def set_desired(self, desired):
        self.desired = dict(desired)

    def set_mode(self, mode):
        self.mode = mode

    def reconcile(self, approve=False):
        self.approved = approve
        desired = self.desired or {}
        uwb = self.node.uwb
        if "role" in desired:
            uwb.role = desired["role"]
        if "targets" in desired:
            uwb.targets = tuple(desired["targets"])
        if any(key in desired for key in ("period_ms", "timeout_ms", "retries")):
            current = dict(zip(("period_ms", "timeout_ms", "retries"), uwb.schedule))
            current.update({key: value for key, value in desired.items() if key in current})
            uwb.schedule = (current["period_ms"], current["timeout_ms"], current["retries"])
        if "ranging_enabled" in desired:
            uwb.ranging_started = desired["ranging_enabled"]
        return {"success": True}


class FakeNode:
    def __init__(self, uart_device, baudrate):
        self.args = (uart_device, baudrate)
        self.comms = FakeComms()
        self.ble = FakeBle()
        self.uwb = FakeUwb()
        self.config = FakeConfigManager(self)
        self.started = False
        self.ready_timeout = None
        self.closed = False
        self.fail_ready = False

    def start(self):
        self.started = True

    def wait_until_ready(self, timeout):
        self.ready_timeout = timeout
        if self.fail_ready:
            raise TimeoutError("node did not become ready")

    def on(self, event, callback):
        return callback

    def off(self, event, callback):
        pass

    def get_state(self):
        return type("State", (), {"status": {"ranging_enabled": self.uwb.ranging_started}})()

    def close(self):
        self.closed = True


def make_device(
    device_id,
    uart_device,
    *,
    ble_enabled=None,
    payloads=(),
    role="responder",
    target_ids=(),
    uwb=None,
):
    return D3KDeviceConfiguration(
        device_id=device_id,
        uart_device=uart_device,
        baudrate=460800,
        ready_timeout_s=5.0,
        uwb=uwb or UwbDeviceConfiguration(
            enabled=True,
            role=role,
            target_ids=tuple(target_ids) if target_ids else None,
        ),
        ble_enabled=ble_enabled,
        payloads=tuple(payloads),
    )


def test_vtol_composition_connects_every_node_but_starts_ble_on_only_one():
    config = VtolRuntimeConfiguration(
        devices=(
            make_device("anchor-a", "/dev/a"),
            make_device("anchor-b", "/dev/b"),
            make_device(
                "anchor-comms",
                "/dev/c",
                ble_enabled=True,
                payloads=(PayloadPeerConfiguration("payload-a", 20), PayloadPeerConfiguration("payload-b", 21)),
            ),
        )
    )
    captured = []

    def factory(*args, **kwargs):
        node = FakeNode(*args, **kwargs)

        def assert_all_nodes_are_ready_for_ble():
            assert len(captured) == 3
            assert all(item.uwb.ranging_started for item in captured)

        node.ble.on_start = assert_all_nodes_are_ready_for_ble
        captured.append(node)
        return node

    app = build_vtol_d3kble_application(config, node_factory=factory)

    assert len(captured) == 3
    assert all(node.started and node.ready_timeout == 5.0 for node in captured)
    assert all(node.uwb.role == "responder" for node in captured)
    # Unspecified target lists and schedule parameters retain the values
    # already configured on each board.
    assert all(node.uwb.targets == (200, 201) for node in captured)
    assert all(node.uwb.schedule == (100, 120, 1) for node in captured)
    assert all(node.config.desired == {"ranging_enabled": True, "role": "responder"} for node in captured)
    assert all(node.config.mode == "manual" and node.config.approved is True for node in captured)
    assert all(node.uwb.ranging_started for node in captured)
    assert [node.ble.started for node in captured] == [False, False, True]
    assert app.nodes == tuple(captured)
    assert app.ble_node is captured[2]
    assert [peer.payload_id for peer in app.payload_peers] == ["payload-a", "payload-b"]
    assert [peer.ble_peer_id for peer in app.payload_peers] == [20, 21]

    app.close()
    assert all(node.uwb.ranging_stopped for node in captured)
    assert [node.ble.stopped for node in captured] == [False, False, True]
    assert all(node.closed for node in captured)


def test_vtol_composition_applies_per_device_initiator_targets():
    config = VtolRuntimeConfiguration(
        devices=(
            make_device(
                "uwb-initiator",
                "/dev/a",
                ble_enabled=True,
                payloads=(PayloadPeerConfiguration("payload-a", 20),),
                role="initiator",
                target_ids=(200, 201),
            ),
        )
    )
    captured = []

    def factory(*args, **kwargs):
        node = FakeNode(*args, **kwargs)
        captured.append(node)
        return node

    app = build_vtol_d3kble_application(config, node_factory=factory)
    assert captured[0].uwb.targets == (200, 201)
    app.close()


def test_vtol_composition_applies_only_explicit_partial_uwb_overrides():
    config = VtolRuntimeConfiguration(
        devices=(
            make_device(
                "anchor-a",
                "/dev/a",
                ble_enabled=True,
                payloads=(PayloadPeerConfiguration("payload-a", 20),),
                uwb=UwbDeviceConfiguration(
                    enabled=True,
                    role="responder",
                    period_ms=175,
                    channel=9,
                    tx_antenna_delay=16000,
                ),
            ),
        )
    )

    captured = []

    def factory(*args, **kwargs):
        node = FakeNode(*args, **kwargs)
        captured.append(node)
        return node

    app = build_vtol_d3kble_application(config, node_factory=factory)
    node = captured[0]

    assert node.config.desired == {
        "ranging_enabled": True,
        "role": "responder",
        "period_ms": 175,
        "channel": 9,
        "tx_antenna_delay": 16000,
    }
    assert node.uwb.schedule == (175, 120, 1)
    assert node.uwb.targets == (200, 201)
    assert node.uwb.ranging_started

    app.close()


def test_vtol_composition_does_not_touch_uwb_when_no_overrides_are_configured():
    config = VtolRuntimeConfiguration(
        devices=(
            make_device(
                "anchor-a",
                "/dev/a",
                ble_enabled=True,
                payloads=(PayloadPeerConfiguration("payload-a", 20),),
                uwb=UwbDeviceConfiguration(),
            ),
        )
    )

    captured = []

    def factory(*args, **kwargs):
        node = FakeNode(*args, **kwargs)
        # Simulate ranging already enabled by the board's d3km/NVS setup.
        node.uwb.ranging_started = True
        captured.append(node)
        return node

    app = build_vtol_d3kble_application(config, node_factory=factory)
    node = captured[0]

    assert node.config.desired is None
    assert node.uwb.ranging_started is True
    assert node.uwb.role == "responder"
    assert node.uwb.targets == (200, 201)
    assert node.uwb.schedule == (100, 120, 1)

    app.close()
    # The application did not take ownership of the pre-existing UWB service.
    assert not node.uwb.ranging_stopped


def test_vtol_composition_closes_all_nodes_if_a_later_node_fails_startup():
    config = VtolRuntimeConfiguration(
        devices=(
            make_device("first", "/dev/a"),
            make_device(
                "second",
                "/dev/b",
                ble_enabled=True,
                payloads=(PayloadPeerConfiguration("payload-a", 20),),
            ),
        )
    )
    captured = []

    def factory(*args, **kwargs):
        node = FakeNode(*args, **kwargs)
        if args[0] == "/dev/b":
            node.fail_ready = True
        captured.append(node)
        return node

    try:
        build_vtol_d3kble_application(config, node_factory=factory)
    except TimeoutError:
        pass
    else:
        raise AssertionError("startup failure should propagate")

    assert len(captured) == 2
    assert all(node.closed for node in captured)
    assert captured[0].uwb.ranging_stopped
    assert not captured[1].uwb.ranging_stopped
    assert not captured[1].ble.stopped


def test_vtol_composition_preserves_firmware_ble_state_when_override_is_omitted():
    config = VtolRuntimeConfiguration(
        devices=(
            make_device(
                "anchor-comms",
                "/dev/a",
                payloads=(PayloadPeerConfiguration("payload-a", 20),),
                ble_enabled=None,
                uwb=UwbDeviceConfiguration(),
            ),
            make_device("anchor-back", "/dev/b", ble_enabled=None, uwb=UwbDeviceConfiguration()),
        )
    )
    captured = []

    def factory(*args, **kwargs):
        node = FakeNode(*args, **kwargs)
        # Model board firmware defaults: BLE on for the communication board,
        # off for the other local D3K boards.
        node.ble.started = args[0] == "/dev/a"
        captured.append(node)
        return node

    app = build_vtol_d3kble_application(config, node_factory=factory)

    assert captured[0].ble.started is True
    assert captured[0].ble.stopped is False
    assert captured[1].ble.started is False
    assert captured[1].ble.stopped is False
    assert app.ble_node is captured[0]
    assert [peer.payload_id for peer in app.payload_peers] == ["payload-a"]

    app.close()
    # The application did not claim ownership of either firmware default.
    assert captured[0].ble.started is True
    assert captured[0].ble.stopped is False


def test_vtol_composition_applies_explicit_ble_off_without_dropping_peer_mapping():
    config = VtolRuntimeConfiguration(
        devices=(
            make_device(
                "anchor-comms",
                "/dev/a",
                ble_enabled=False,
                payloads=(PayloadPeerConfiguration("payload-a", 20),),
                uwb=UwbDeviceConfiguration(),
            ),
        )
    )
    captured = []

    def factory(*args, **kwargs):
        node = FakeNode(*args, **kwargs)
        node.ble.started = True
        captured.append(node)
        return node

    app = build_vtol_d3kble_application(config, node_factory=factory)
    node = captured[0]

    assert node.ble.started is False
    assert node.ble.stopped is True
    assert app.ble_node is node
    assert [peer.payload_id for peer in app.payload_peers] == ["payload-a"]

    app.close()
    assert node.ble.stopped is True


def test_vtol_composition_runs_without_any_ble_host():
    config = VtolRuntimeConfiguration(
        devices=(
            make_device("anchor-a", "/dev/a", uwb=UwbDeviceConfiguration()),
            make_device("anchor-b", "/dev/b", uwb=UwbDeviceConfiguration()),
        )
    )
    captured = []

    def factory(*args, **kwargs):
        node = FakeNode(*args, **kwargs)
        captured.append(node)
        return node

    app = build_vtol_d3kble_application(config, node_factory=factory)

    assert len(captured) == 2
    assert app.ble_node is None
    assert app.router is None
    assert app.payload_peers == ()

    app.close()
    assert all(node.closed for node in captured)
