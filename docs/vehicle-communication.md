# VTOL / payload communication adapters

This document describes the low-level `d3kble` integration between the VTOL
application and a payload application. It covers transport setup, message
routing, serialization, ranging subscriptions, and command acknowledgement.

It deliberately does **not** implement:

- Mission Planner flight sections or QLAND/QHOVER/QMANUAL selection;
- a decision about which system owns flight-section transitions;
- VTOL grounding, take-off, landing, or flight-mode interlocks;
- payload movement, docking, latch actuation, or payload state transitions;
- a payload state machine or a VTOL-side `PayloadRegistry` implementation.

Those remain application-level decisions for a later change.

## 1. D3KNode and router ownership

Each physical vehicle runs one local `D3KNode` for its attached ESP32. All
local adapters share that node. The node lifecycle remains owned by the
application composition root; adapters must not start, stop, or close it.

Create one `D3kbleMessageRouter` per local `D3KNode`. It registers a single
`on_message` and a single `on_request` callback, then dispatches incoming
messages to the adapter registered for the sender's BLE peer ID. This is
important because the current `d3kble` `CommsManager` dispatches incoming
requests to its first registered request handler only.

The router must therefore be the **only** `node.comms.on_request` handler for
that node. Do not register additional request handlers directly on the same
`CommsManager`. Other `on_message` callbacks can coexist, but vehicle adapter
messages should be consumed through the router.

The router binds `node.comms` async handlers to the currently running asyncio
event loop. Start the node and wait for it to be ready before entering the
application event loop; start adapters from inside that loop.

## 2. Payload application setup

The payload has:

- one `D3KNode` connected to its local UWB/BLE module;
- one `D3kbleMessageRouter` shared by its communication adapters;
- one `D3kblePlanePeer` for the plane's BLE peer ID;
- one `D3kbleDistanceSensor` with an explicit UWB target-ID-to-anchor-ID map.

Example composition (hardware paths and IDs are deployment configuration):

```python
import asyncio

from d3kble import D3KNode
from payload.infrastructure.d3kble import D3kbleDistanceSensor, D3kblePlanePeer
from shared.infrastructure.d3kble import D3kbleMessageRouter

node = D3KNode("/dev/ttyAMA2")
node.start()
node.wait_until_ready(timeout=10)


async def main() -> None:
    router = D3kbleMessageRouter(node)
    plane = D3kblePlanePeer(
        node,
        router,
        plane_peer_id=10,
    )
    distance_sensor = D3kbleDistanceSensor(
        node,
        anchor_id_by_target_id={
            200: "plane-anchor-front-left",
            201: "plane-anchor-front-right",
            202: "plane-anchor-rear-left",
            203: "plane-anchor-rear-right",
        },
    )

    try:
        await plane.start()

        # Start the ranging consumer before enabling ranging so its UWB
        # measurement and range-failure subscriptions are already active.
        distance_task = asyncio.create_task(
            consume_distance_events(distance_sensor.events())
        )

        # Configure UWB role, target list, and ranging schedule separately,
        # according to the deployment's selected anchor/ranging topology.
        await distance_sensor.start_ranging()

        # Application-owned coroutines consume plane.events() and
        # explicitly report their results.
        await run_payload_application(plane, distance_sensor, distance_task)
    finally:
        await distance_sensor.stop_ranging()
        plane.close()
        router.close()


try:
    asyncio.run(main())
finally:
    node.close()
```

`run_payload_application` is an application placeholder, not part of the
adapter package. In particular, this integration does not connect incoming
dock/undock requests to a movement controller.

The UWB target IDs in `anchor_id_by_target_id` identify ranging anchors. They
are not assumed to be equal to BLE peer IDs. `D3kbleDistanceSensor` observes
measurements and ranging failures; it does not configure the UWB role, target
list, or schedule. `start_ranging()` starts the device's already-configured
schedule and `stop_ranging()` stops it.

## 3. VTOL application setup

The VTOL has one local node and router, plus one `D3kblePayloadPeer` per
configured payload BLE peer. All those adapters share the same router.

```python
import asyncio

from d3kble import D3KNode
from shared.infrastructure.d3kble import D3kbleMessageRouter
from vtol.infrastructure.d3kble import D3kblePayloadPeer

node = D3KNode("/dev/ttyAMA2")
node.start()
node.wait_until_ready(timeout=10)


async def main() -> None:
    router = D3kbleMessageRouter(node)
    payloads = [
        D3kblePayloadPeer(
            node,
            router,
            payload_id="payload-1",
            peer_id=20,
        ),
        D3kblePayloadPeer(
            node,
            router,
            payload_id="payload-2",
            peer_id=21,
        ),
    ]

    try:
        await asyncio.gather(*(payload.start() for payload in payloads))

        # Application-owned coordination consumes payload.events() and
        # decides when to publish heading / DLZ snapshots or issue commands.
        await run_vtol_application(payloads)
    finally:
        for payload in payloads:
            payload.close()
        router.close()


try:
    asyncio.run(main())
finally:
    node.close()
```

The `payload_id` is a stable application identity. `peer_id` is the numeric
BLE peer ID configured by the D3Kble firmware. Keep the two mappings explicit.
The current firmware supports up to eight simultaneous BLE peer links per
local node.

## 4. Message flow

### Plane to payload

| Operation | D3Kble communication | Payload-side result |
|---|---|---|
| Publish heading | One-way `comms.send()` | `PlaneHeadingUpdated` event |
| Publish DLZ snapshot | One-way `comms.send()` | `DLZStateUpdated` event |
| Request docking | Request/response | Immediate `CommandReceipt` |
| Request parking at target | Request/response | Immediate `CommandReceipt` |
| Request payload status | Request/response | Latest reported `PayloadStatusReport` |

`request_dock()` and `request_undock()` return after the payload application
has acknowledged or rejected the command. They do not wait for docking or
parking to finish.

For one-way telemetry, the D3Kble application ACK only confirms that the
remote communications manager received and queued the message. It does not
prove that the vehicle application consumed the event or acted on it.

On the payload, incoming dock/undock requests are emitted through
`D3kblePlanePeer.events()`. The application must call
`acknowledge_command(receipt)` for the matching `command_id`. If it does not
acknowledge within the configured `command_ack_timeout` (default 0.6 seconds),
the adapter responds with an explicit rejection receipt. This deadline is
shorter than the current `d3kble` default one-second request-response deadline.

### Payload to plane

| Report | D3Kble communication | VTOL-side result |
|---|---|---|
| Position estimate | One-way `comms.send()` | `PayloadPositionUpdated` event |
| Payload/latch state | One-way `comms.send()` | `PayloadStateUpdated` event |
| Parking completed/aborted | One-way `comms.send()` | Matching operation event |
| Docking succeeded/aborted | One-way `comms.send()` | Matching operation event |

The operation-result events are independent of the initial command receipt.
They carry the original command ID so the application can correlate them. The
adapters transport these events; they do not create them based on physical
movement or latch feedback.

### Status query behavior

A VTOL `request_status()` is forwarded by the payload adapter as a
`PayloadStatusRequested` event. The payload application supplies the report
with `respond_status_request(request_id, report)`. The adapter does not cache,
infer, or manage payload state; it only correlates the request and response.
If the application does not answer before `status_response_timeout` (default
0.6 seconds), the request receives an error response.

## 5. Ranging adapter behavior

`D3kbleDistanceSensor.events()` creates an independent subscription to
`node.uwb.measurements` and bridges the blocking subscription queue into an
async iterator. It also listens to the separate `range_failed` node event.

Successful measurements become `DistanceSample` events. Invalid measurement
records and ranging failures become `DistanceFailure` events. Unknown UWB
target IDs are ignored because there is no application anchor mapping for
them. Closing the async iterator removes both subscriptions and stops its
worker thread.

The adapter does not perform trilateration. The payload's `PositionEstimator`
remains responsible for combining measurements with calibrated anchor
coordinates and sensor transforms.

All `Position` values exchanged by the adapters are interpreted in the
plane-local reference frame and use metres; `Heading.angle_rad` is in radians.
The adapters do not rotate or transform coordinates. Both applications must
agree on the X/Y/Z axis directions and the heading zero/rotation convention
before they exchange movement targets. Those conventions are not inferred by
the transport layer.

## 6. Wire protocol and limits

The shared codec uses a versioned JSON-compatible message envelope:

- protocol: `vehicle-dlz`;
- protocol version: `1`;
- message type and typed data payload.

DLZ snapshots are zlib-compressed and base64-encoded before transmission to
make a small group of payload snapshots fit the D3Kble transport limit. The
codec checks the complete outer D3Kble envelope, including request/response
metadata, against the current 496-byte maximum. Oversized messages raise
`D3kblePayloadTooLarge`; the adapters do not fragment messages. The payload
count that fits depends on identifier lengths and the amount of position data.

## 7. Installation and lifecycle

The D3Kble host library is maintained in the separate
`dwm3000-esp32-interface/d3kble-api` project. Install it in the vehicle
application's Python environment, for example from a local checkout:

```bash
python -m pip install -e /path/to/dwm3000-esp32-interface/d3kble-api
```

The application project deliberately does not vendor or copy the D3Kble
library. Adapters accept the public `D3KNode` API structurally and do not
import D3Kble at module import time. This keeps adapter tests independent of a
serial device and lets tests use a fake node.

Call `node.start()` and `node.wait_until_ready()` before starting the async
application. Start the router and peer adapters inside the application event
loop. Close peer adapters, then the router, then the node. The adapters never
own the node lifecycle.

## 8. Safety and security boundary

The current D3Kble BLE transport does not provide BLE pairing or encryption,
and a BLE peer ID is not an authentication credential. These messages should
therefore be treated as an unauthenticated coordination channel in this
iteration. Do not use successful message delivery or a `CommandReceipt` as a
flight-safety interlock, proof of physical docking, or proof that a payload is
safe to move. Any flight-mode gating, grounding/take-off authority, watchdogs,
physical latch verification, and loss-of-link failsafes must be designed and
validated separately before these messages are allowed to influence flight
safety decisions.
