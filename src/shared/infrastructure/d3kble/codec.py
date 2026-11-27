from __future__ import annotations

import base64
import json
import math
import zlib
from typing import Any

from shared.contracts.geometry import Heading, Position, PositionEstimate
from shared.contracts.messages import (
    CommandReceipt,
    DLZState,
    DLZStateUpdated,
    DockingAborted,
    DockingSucceeded,
    DockRequest,
    FailureReason,
    ParkingAborted,
    ParkingCompleted,
    PayloadEvent,
    PayloadPositionUpdated,
    PayloadSnapshot,
    PayloadStateUpdated,
    PayloadStatusReport,
    PlaneEvent,
    PlaneHeadingUpdated,
    UndockRequest,
)
from shared.contracts.payload_state import DockingLatchState, PayloadState, PayloadStatus

PROTOCOL_NAME = "vehicle-dlz"
PROTOCOL_VERSION = 1
D3KBLE_MAX_ENVELOPE_SIZE = 496


class D3kbleProtocolError(ValueError):
    """
    A received or outgoing application message violates the wire contract.
    """


class D3kblePayloadTooLarge(D3kbleProtocolError):
    """
    An application message would exceed d3kble's 496-byte envelope limit.
    """


def encode_message(message_type: str, data: dict[str, Any], *, kind: str = "message") -> dict[str, Any]:
    if not isinstance(message_type, str) or not message_type or not isinstance(data, dict):
        raise D3kbleProtocolError("message type and data are required")

    if kind not in {"message", "request"}:
        raise D3kbleProtocolError("outgoing message kind must be 'message' or 'request'")

    payload = {
        "protocol": PROTOCOL_NAME,
        "version": PROTOCOL_VERSION,
        "type": message_type,
        "data": data,
    }
    ensure_envelope_fits(payload, kind=kind)
    return payload


def decode_message(payload: Any) -> tuple[str, dict[str, Any]]:
    record = _mapping(payload, "message")

    if record.get("protocol") != PROTOCOL_NAME:
        raise D3kbleProtocolError("unexpected application protocol")

    if record.get("version") != PROTOCOL_VERSION:
        raise D3kbleProtocolError(f"unsupported protocol version: {record.get('version')!r}")

    message_type = _string(record.get("type"), "message.type")
    data = _mapping(record.get("data"), "message.data")
    return message_type, data


def ensure_envelope_fits(payload: dict[str, Any], kind: str = "message") -> None:
    """
    Preflight an outgoing message/request against d3kble's envelope limit.
    """
    envelope = {
        "v": 1,
        "id": "0" * 32,
        "kind": kind,
        "payload": payload,
    }
    _ensure_encoded_size(envelope)


def ensure_response_fits(content: dict[str, Any]) -> None:
    """
    Preflight a request response, including d3kble's correlation metadata.
    """
    envelope = {
        "v": 1,
        "id": "0" * 32,
        "kind": "response",
        "payload": {
            "status": "ok",
            "error": None,
            "content": content,
        },
        "reply_to": "0" * 32,
    }
    _ensure_encoded_size(envelope)


def _ensure_encoded_size(envelope: dict[str, Any]) -> None:
    encoded = json.dumps(envelope, separators=(",", ":"), ensure_ascii=False).encode("utf-8")

    if len(encoded) > D3KBLE_MAX_ENVELOPE_SIZE:
        raise D3kblePayloadTooLarge(
            f"encoded d3kble envelope is {len(encoded)} bytes; maximum is {D3KBLE_MAX_ENVELOPE_SIZE}"
        )


def position_to_data(position: Position) -> dict[str, float]:
    return {
        "x": _finite(position.x, "position.x"),
        "y": _finite(position.y, "position.y"),
        "z": _finite(position.z, "position.z"),
    }


def position_from_data(data: Any) -> Position:
    item = _mapping(data, "position")
    return Position(
        x=_finite(item.get("x"), "position.x"),
        y=_finite(item.get("y"), "position.y"),
        z=_finite(item.get("z"), "position.z"),
    )


def heading_to_data(heading: Heading) -> dict[str, float]:
    return {"angle_rad": _finite(heading.angle_rad, "heading.angle_rad")}


def heading_from_data(data: Any) -> Heading:
    item = _mapping(data, "heading")
    return Heading(angle_rad=_finite(item.get("angle_rad"), "heading.angle_rad"))


def estimate_to_data(estimate: PositionEstimate) -> dict[str, Any]:
    residual = estimate.residual_m

    if residual is not None:
        residual = _finite(residual, "position_estimate.residual_m")

    anchor_ids = [_string(anchor_id, "position_estimate.anchor_id") for anchor_id in estimate.anchor_ids]

    if len(anchor_ids) != len(set(anchor_ids)):
        raise D3kbleProtocolError("position_estimate.anchor_ids must be unique")

    result = {
        "position": position_to_data(estimate.position),
        "residual_m": residual,
        "anchor_ids": anchor_ids,
        "sequence": _integer(estimate.sequence, "position_estimate.sequence", minimum=0),
    }
    if estimate.device_monotonic_us is not None:
        result["device_monotonic_us"] = _integer(
            estimate.device_monotonic_us, "position_estimate.device_monotonic_us", minimum=0
        )
    if estimate.epoch_us is not None:
        result["epoch_us"] = _integer(estimate.epoch_us, "position_estimate.epoch_us", minimum=0)
    if estimate.received_at is not None:
        result["received_at"] = _finite(estimate.received_at, "position_estimate.received_at")
    if estimate.covariance_xy_m2 is not None:
        if len(estimate.covariance_xy_m2) != 3:
            raise D3kbleProtocolError("position_estimate.covariance_xy_m2 must contain three values")
        result["covariance_xy_m2"] = [
            _finite(value, "position_estimate.covariance_xy_m2") for value in estimate.covariance_xy_m2
        ]
    if estimate.quality != "unknown":
        result["quality"] = _string(estimate.quality, "position_estimate.quality")
    return result


def estimate_from_data(data: Any) -> PositionEstimate:
    item = _mapping(data, "position_estimate")
    residual = item.get("residual_m")

    if residual is not None:
        residual = _finite(residual, "position_estimate.residual_m")

    anchors = item.get("anchor_ids")

    if not isinstance(anchors, list) or any(not isinstance(anchor, str) or not anchor for anchor in anchors):
        raise D3kbleProtocolError("position_estimate.anchor_ids must be a list of non-empty strings")
    covariance = item.get("covariance_xy_m2")
    if covariance is not None and (not isinstance(covariance, list) or len(covariance) != 3):
        raise D3kbleProtocolError("position_estimate.covariance_xy_m2 must be a three-value list")

    return PositionEstimate(
        position=position_from_data(item.get("position")),
        residual_m=residual,
        anchor_ids=tuple(anchors),
        sequence=_integer(item.get("sequence"), "position_estimate.sequence", minimum=0),
        device_monotonic_us=(
            _integer(item.get("device_monotonic_us"), "position_estimate.device_monotonic_us", minimum=0)
            if item.get("device_monotonic_us") is not None else None
        ),
        epoch_us=(
            _integer(item.get("epoch_us"), "position_estimate.epoch_us", minimum=0)
            if item.get("epoch_us") is not None else None
        ),
        received_at=(
            _finite(item.get("received_at"), "position_estimate.received_at")
            if item.get("received_at") is not None else None
        ),
        covariance_xy_m2=(
            tuple(
                _finite(value, "position_estimate.covariance_xy_m2")
                for value in item["covariance_xy_m2"]
            )
            if covariance is not None
            else None
        ),
        quality=_string(item.get("quality", "unknown"), "position_estimate.quality"),
    )


def state_to_data(state: PayloadState) -> dict[str, str]:
    if not isinstance(state.status, PayloadStatus) or not isinstance(state.latch, DockingLatchState):
        raise D3kbleProtocolError("payload state contains an unsupported status or latch value")

    return {"status": state.status.name, "latch": state.latch.name}


def state_from_data(data: Any) -> PayloadState:
    item = _mapping(data, "payload_state")

    return PayloadState(
        status=_enum(PayloadStatus, item.get("status"), "payload_state.status"),
        latch=_enum(DockingLatchState, item.get("latch"), "payload_state.latch"),
    )


def failure_to_data(reason: FailureReason) -> dict[str, str]:
    return {
        "code": _string(reason.code, "failure_reason.code"),
        "message": _string(reason.message, "failure_reason.message"),
    }


def failure_from_data(data: Any) -> FailureReason:
    item = _mapping(data, "failure_reason")

    return FailureReason(
        code=_string(item.get("code"), "failure_reason.code"),
        message=_string(item.get("message"), "failure_reason.message"),
    )


def receipt_to_data(receipt: CommandReceipt) -> dict[str, Any]:
    if receipt.rejection_reason is not None and receipt.accepted:
        raise D3kbleProtocolError("an accepted command cannot have a rejection reason")

    return {
        "command_id": _string(receipt.command_id, "command_receipt.command_id"),
        "accepted": _boolean(receipt.accepted, "command_receipt.accepted"),
        "rejection_reason": failure_to_data(receipt.rejection_reason) if receipt.rejection_reason else None,
    }


def receipt_from_data(data: Any) -> CommandReceipt:
    item = _mapping(data, "command_receipt")
    accepted = _boolean(item.get("accepted"), "command_receipt.accepted")
    rejection = item.get("rejection_reason")

    if accepted and rejection is not None:
        raise D3kbleProtocolError("an accepted command cannot have a rejection reason")

    return CommandReceipt(
        command_id=_string(item.get("command_id"), "command_receipt.command_id"),
        accepted=accepted,
        rejection_reason=failure_from_data(rejection) if rejection is not None else None,
    )


def status_report_to_data(report: PayloadStatusReport) -> dict[str, Any]:
    return {
        "state": state_to_data(report.state),
        "position": estimate_to_data(report.position) if report.position is not None else None,
        "sequence": _integer(report.sequence, "payload_status_report.sequence", minimum=0),
    }


def status_report_from_data(data: Any) -> PayloadStatusReport:
    item = _mapping(data, "payload_status_report")
    position = item.get("position")
    return PayloadStatusReport(
        state=state_from_data(item.get("state")),
        position=estimate_from_data(position) if position is not None else None,
        sequence=_integer(item.get("sequence"), "payload_status_report.sequence", minimum=0),
    )


def dlz_state_to_data(state: DLZState) -> dict[str, Any]:
    return {
        "revision": _integer(state.revision, "dlz_state.revision", minimum=0),
        "payloads": [
            {
                "payload_id": _string(snapshot.payload_id, "payload_snapshot.payload_id"),
                "state": state_to_data(snapshot.state),
                "position": estimate_to_data(snapshot.position) if snapshot.position is not None else None,
                "sequence": _integer(snapshot.sequence, "payload_snapshot.sequence", minimum=0),
            }
            for snapshot in state.payloads
        ],
    }


def dlz_state_from_data(data: Any) -> DLZState:
    item = _mapping(data, "dlz_state")
    payloads = item.get("payloads")

    if not isinstance(payloads, list):
        raise D3kbleProtocolError("dlz_state.payloads must be a list")

    snapshots = []

    for raw in payloads:
        payload = _mapping(raw, "payload_snapshot")
        position = payload.get("position")
        snapshots.append(
            PayloadSnapshot(
                payload_id=_string(payload.get("payload_id"), "payload_snapshot.payload_id"),
                state=state_from_data(payload.get("state")),
                position=estimate_from_data(position) if position is not None else None,
                sequence=_integer(payload.get("sequence"), "payload_snapshot.sequence", minimum=0),
            )
        )

    return DLZState(
        revision=_integer(item.get("revision"), "dlz_state.revision", minimum=0),
        payloads=tuple(snapshots),
    )


def encode_plane_event(event: PlaneEvent) -> dict[str, Any]:
    if isinstance(event, PlaneHeadingUpdated):
        return encode_message(
            "plane.heading_updated",
            {
                "heading": heading_to_data(event.heading),
                "sequence": _integer(event.sequence, "heading.sequence", minimum=0),
            },
        )

    if isinstance(event, DLZStateUpdated):
        raw_state = json.dumps(dlz_state_to_data(event.state), separators=(",", ":"), ensure_ascii=False).encode(
            "utf-8"
        )
        compressed_state = base64.b64encode(zlib.compress(raw_state, level=6)).decode("ascii")
        return encode_message(
            "plane.dlz_state_updated",
            {"encoding": "zlib+json+base64", "state": compressed_state},
        )

    raise D3kbleProtocolError(f"unsupported plane event: {type(event).__name__}")


def decode_plane_event(payload: Any) -> PlaneEvent:
    message_type, data = decode_message(payload)

    if message_type == "plane.heading_updated":
        return PlaneHeadingUpdated(
            heading=heading_from_data(data.get("heading")),
            sequence=_integer(data.get("sequence"), "heading.sequence", minimum=0),
        )

    if message_type == "plane.dlz_state_updated":
        if data.get("encoding") != "zlib+json+base64":
            raise D3kbleProtocolError("unsupported DLZ state encoding")

        encoded = data.get("state")

        if not isinstance(encoded, str):
            raise D3kbleProtocolError("compressed DLZ state must be a base64 string")

        try:
            compressed = base64.b64decode(encoded, validate=True)
            decompressor = zlib.decompressobj()
            raw_state = decompressor.decompress(compressed, max_length=64 * 1024 + 1)

            if (
                len(raw_state) > 64 * 1024
                or decompressor.unconsumed_tail
                or decompressor.unused_data
                or not decompressor.eof
            ):
                raise D3kbleProtocolError("decompressed DLZ state exceeds the 64 KiB limit")

            state_data = json.loads(raw_state.decode("utf-8"))
        except D3kbleProtocolError:
            raise
        except (ValueError, UnicodeDecodeError, zlib.error) as exc:
            raise D3kbleProtocolError("invalid compressed DLZ state") from exc

        return DLZStateUpdated(state=dlz_state_from_data(state_data))

    raise D3kbleProtocolError(f"unsupported plane message type: {message_type}")


def encode_payload_event(event: PayloadEvent) -> dict[str, Any]:
    if isinstance(event, PayloadPositionUpdated):
        return encode_message("payload.position_updated", {"estimate": estimate_to_data(event.estimate)})

    if isinstance(event, PayloadStateUpdated):
        return encode_message(
            "payload.state_updated",
            {"state": state_to_data(event.state), "sequence": _integer(event.sequence, "state.sequence", minimum=0)},
        )

    if isinstance(event, ParkingCompleted):
        return encode_message(
            "payload.parking_completed",
            {
                "command_id": _string(event.command_id, "parking_completed.command_id"),
                "final_position": position_to_data(event.final_position),
            },
        )

    if isinstance(event, ParkingAborted):
        return encode_message(
            "payload.parking_aborted",
            {
                "command_id": _string(event.command_id, "parking_aborted.command_id"),
                "reason": failure_to_data(event.reason),
            },
        )

    if isinstance(event, DockingSucceeded):
        return encode_message(
            "payload.docking_succeeded",
            {"command_id": _string(event.command_id, "docking_succeeded.command_id")},
        )

    if isinstance(event, DockingAborted):
        return encode_message(
            "payload.docking_aborted",
            {
                "command_id": _string(event.command_id, "docking_aborted.command_id"),
                "reason": failure_to_data(event.reason),
            },
        )

    raise D3kbleProtocolError(f"unsupported payload event: {type(event).__name__}")


def decode_payload_event(payload: Any) -> PayloadEvent:
    message_type, data = decode_message(payload)

    def get_command_id() -> str:
        return _string(data.get("command_id"), f"{message_type}.command_id")

    if message_type == "payload.position_updated":
        return PayloadPositionUpdated(estimate=estimate_from_data(data.get("estimate")))

    if message_type == "payload.state_updated":
        return PayloadStateUpdated(
            state=state_from_data(data.get("state")),
            sequence=_integer(data.get("sequence"), "state.sequence", minimum=0),
        )

    if message_type == "payload.parking_completed":
        return ParkingCompleted(
            command_id=get_command_id(),
            final_position=position_from_data(data.get("final_position")),
        )

    if message_type == "payload.parking_aborted":
        return ParkingAborted(command_id=get_command_id(), reason=failure_from_data(data.get("reason")))

    if message_type == "payload.docking_succeeded":
        return DockingSucceeded(command_id=get_command_id())

    if message_type == "payload.docking_aborted":
        return DockingAborted(command_id=get_command_id(), reason=failure_from_data(data.get("reason")))

    raise D3kbleProtocolError(f"unsupported payload message type: {message_type}")


def encode_dock_request(command_id: str) -> dict[str, Any]:
    return encode_message(
        "plane.command.dock",
        {
            "command_id": _string(command_id, "command_id"),
        },
        kind="request",
    )


def encode_undock_request(command_id: str, target_position: Position) -> dict[str, Any]:
    return encode_message(
        "plane.command.undock",
        {
            "command_id": _string(command_id, "command_id"),
            "target_position": position_to_data(target_position),
        },
        kind="request",
    )


def decode_command_request(payload: Any):
    message_type, data = decode_message(payload)
    command_id = _string(data.get("command_id"), "command_id")

    if message_type == "plane.command.dock":
        return DockRequest(command_id=command_id)

    if message_type == "plane.command.undock":
        return UndockRequest(
            command_id=command_id,
            target_position=position_from_data(data.get("target_position")),
        )

    raise D3kbleProtocolError(f"unsupported command type: {message_type}")


def encode_command_receipt(receipt: CommandReceipt) -> dict[str, Any]:
    content = encode_message("payload.command_receipt", receipt_to_data(receipt))
    ensure_response_fits(content)
    return content


def decode_command_receipt(payload: Any) -> CommandReceipt:
    message_type, data = decode_message(payload)

    if message_type != "payload.command_receipt":
        raise D3kbleProtocolError(f"expected payload.command_receipt, received {message_type}")

    return receipt_from_data(data)


def encode_status_request() -> dict[str, Any]:
    return encode_message("payload.request_status", {}, kind="request")


def encode_status_report(report: PayloadStatusReport) -> dict[str, Any]:
    content = encode_message("payload.status_report", status_report_to_data(report))
    ensure_response_fits(content)
    return content


def decode_status_report(payload: Any) -> PayloadStatusReport:
    message_type, data = decode_message(payload)

    if message_type != "payload.status_report":
        raise D3kbleProtocolError(f"expected payload.status_report, received {message_type}")

    return status_report_from_data(data)


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise D3kbleProtocolError(f"{name} must be an object")

    return value


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise D3kbleProtocolError(f"{name} must be a non-empty string")

    return value


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise D3kbleProtocolError(f"{name} must be a boolean")

    return value


def _integer(value: Any, name: str, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise D3kbleProtocolError(f"{name} must be an integer")

    if minimum is not None and value < minimum:
        raise D3kbleProtocolError(f"{name} must be >= {minimum}")

    return value


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise D3kbleProtocolError(f"{name} must be a finite number")

    return float(value)


def _enum(enum_type, value: Any, name: str):
    if not isinstance(value, str):
        raise D3kbleProtocolError(f"{name} must be a string")

    try:
        return enum_type[value]
    except KeyError as exc:
        raise D3kbleProtocolError(f"unknown {name}: {value!r}") from exc
