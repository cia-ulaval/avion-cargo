from payload.composition import build_payload_application
from payload.infrastructure.configuration import PayloadRuntimeConfiguration


class FakeComms:
    def bind_asyncio_loop(self, loop):
        self.loop = loop

    def on_message(self, callback=None):
        return callback

    def on_request(self, callback=None):
        return callback


class FakeConfig:
    def __init__(self):
        self.desired = []
        self.modes = []
        self.approvals = []

    def set_desired(self, desired):
        self.desired.append(dict(desired))

    def set_mode(self, mode):
        self.modes.append(mode)

    def reconcile(self, approve=False):
        self.approvals.append(approve)
        return {"success": True, "results": []}


class FakeBle:
    def __init__(self):
        self.started = False
        self.stopped = False

    def start_ble(self):
        self.started = True

    def stop_ble(self):
        self.stopped = True


class FakeUwb:
    def __init__(self):
        self.roles = []
        self.targets = []
        self.schedules = []
        self.measurements = type(
            "Measurements",
            (),
            {
                "subscribe": lambda _self, **kwargs: type(
                    "Subscription",
                    (),
                    {
                        "get": lambda _subscription, timeout=None: None,
                        "dropped_count": 0,
                    },
                )(),
                "unsubscribe": lambda _self, _subscription: None,
            },
        )()

    def set_role(self, role):
        self.roles.append(role)

    def set_targets(self, target_ids):
        self.targets.append(tuple(target_ids))

    def set_schedule(self, **kwargs):
        self.schedules.append(kwargs)

    def start_ranging(self):
        pass

    def stop_ranging(self):
        pass


class FakeNode:
    def __init__(self, uart_device, baudrate):
        self.args = (uart_device, baudrate)
        self.comms = FakeComms()
        self.config = FakeConfig()
        self.ble = FakeBle()
        self.uwb = FakeUwb()
        self.started = False
        self.ready_timeout = None
        self.closed = False

    def start(self):
        self.started = True

    def wait_until_ready(self, timeout):
        self.ready_timeout = timeout

    def on(self, event, callback):
        return callback

    def off(self, event, callback):
        pass

    def close(self):
        self.closed = True


def test_payload_composition_configures_one_shared_node_and_adapters():
    config = PayloadRuntimeConfiguration(
        payload_id="payload-a",
        uart_device="/dev/test-uart",
        plane_peer_id=10,
        anchor_id_by_target_id={200: "front-left", 201: "front-right", 202: "rear"},
        uwb_role=None,
        uwb_period_ms=None,
        uwb_timeout_ms=None,
        uwb_retries=None,
    )
    captured = []

    def factory(*args, **kwargs):
        node = FakeNode(*args, **kwargs)
        captured.append(node)
        return node

    app = build_payload_application(config, node_factory=factory)
    node = captured[0]
    assert node.started
    assert node.ready_timeout == config.ready_timeout_s
    assert node.ble.started
    assert node.uwb.roles == []
    assert node.uwb.targets == []
    assert node.uwb.schedules == []
    assert node.config.desired == []
    assert app.plane_peer.plane_ble_peer_id == 10

    app.close()
    assert node.ble.stopped
    assert node.closed


def test_payload_composition_applies_only_explicit_uwb_overrides():
    config = PayloadRuntimeConfiguration(
        payload_id="payload-a",
        uart_device="/dev/test-uart",
        plane_peer_id=10,
        anchor_id_by_target_id={200: "front-left", 201: "front-right"},
        uwb_role="initiator",
        uwb_period_ms=150,
    )

    app = build_payload_application(config, node_factory=FakeNode)
    node = app.node

    assert node.config.desired == [{"role": "initiator", "period_ms": 150}]
    assert node.config.modes == ["manual"]
    assert node.config.approvals == [True]
    assert node.uwb.roles == []
    assert node.uwb.targets == []
    assert node.uwb.schedules == []

    app.close()
