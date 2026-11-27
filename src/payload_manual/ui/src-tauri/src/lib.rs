mod d3k_protocol;

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use serialport::{SerialPort, SerialPortInfo};
use std::collections::{HashSet, VecDeque};
use std::sync::Mutex;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tauri::{AppHandle, Emitter, Manager};

struct Controller(Mutex<ControllerState>);

struct ControllerState {
    port: Option<Box<dyn SerialPort>>,
    sequence: u32,
    session_id: String,
    payload_peer_id: u16,
    controller_device_id: Option<u16>,
    armed: bool,
    armed_confirmed: bool,
    session_active: bool,
    pending_action: Option<String>,
    pending_sequence: u32,
    last_error: Option<String>,
    peer_connected: bool,
    outbound: OutboundQueue,
    in_flight: Option<InFlightOutbound>,
    outbound_retry_after: Option<Instant>,
}

#[derive(Debug)]
enum OutboundKind {
    ApplicationAck { received_envelope_id: String },
    SafetyControl { command_type: String, sequence: u32 },
    Control { command_type: String, sequence: u32 },
    Status { sequence: u32 },
    Drive { sequence: u32 },
}

impl OutboundKind {
    fn label(&self) -> &'static str {
        match self {
            Self::ApplicationAck { .. } => "application_ack",
            Self::SafetyControl { command_type, .. } | Self::Control { command_type, .. } => {
                match command_type.as_str() {
                    "arm" => "arm",
                    "stop" => "stop",
                    "disarm" => "disarm",
                    _ => "control",
                }
            }
            Self::Status { .. } => "status",
            Self::Drive { .. } => "drive",
        }
    }

    fn sequence(&self) -> Option<u32> {
        match self {
            Self::ApplicationAck { .. } => None,
            Self::SafetyControl { sequence, .. }
            | Self::Control { sequence, .. }
            | Self::Status { sequence }
            | Self::Drive { sequence } => Some(*sequence),
        }
    }

    fn ack_envelope_id(&self) -> Option<&str> {
        match self {
            Self::ApplicationAck { received_envelope_id } => Some(received_envelope_id),
            _ => None,
        }
    }

    fn is_disarm(&self) -> bool {
        matches!(self, Self::SafetyControl { command_type, .. } if command_type.as_str() == "disarm")
    }
}

#[derive(Debug)]
struct PendingOutbound {
    peer_id: u16,
    bytes: Vec<u8>,
    kind: OutboundKind,
    queued_at: Instant,
    attempts: u32,
    source_d3k_message_id: Option<u16>,
}

#[derive(Debug)]
struct InFlightOutbound {
    peer_id: u16,
    d3k_message_id: u16,
    pending: PendingOutbound,
    started_at: Instant,
}

#[derive(Default)]
struct OutboundQueue {
    safety_controls: VecDeque<PendingOutbound>,
    acknowledgements: VecDeque<PendingOutbound>,
    controls: VecDeque<PendingOutbound>,
    status: Option<PendingOutbound>,
    drive: Option<PendingOutbound>,
    pending_ack_ids: HashSet<String>,
    recently_acked_ids: VecDeque<String>,
    safety_barrier_sequence: Option<u32>,
    coalesced_drive_updates: u64,
    // Alternate application ACKs and drive heartbeats when both are pending.
    // BLE delivery is single-flight and takes roughly 250 ms per message, so
    // strict ACK priority can otherwise starve the payload watchdog.
    prefer_drive_next: bool,
}

fn sequence_precedes(sequence: u32, barrier: u32) -> bool {
    sequence != barrier && barrier.wrapping_sub(sequence) < (1u32 << 31)
}

impl OutboundQueue {
    fn enqueue_ack(&mut self, message: PendingOutbound) -> bool {
        let received_id = match &message.kind {
            OutboundKind::ApplicationAck { received_envelope_id } => received_envelope_id.clone(),
            _ => return false,
        };
        if self.pending_ack_ids.contains(&received_id)
            || self.recently_acked_ids.iter().any(|id| id == &received_id)
        {
            return false;
        }
        self.pending_ack_ids.insert(received_id);
        self.acknowledgements.push_back(message);
        true
    }

    fn enqueue_safety_control(&mut self, message: PendingOutbound) {
        // A stop/disarm supersedes older unsent arm and motion setpoints. Keep
        // the barrier after the safety command leaves the queue: an older
        // in-flight arm may be requeued later by a delivery error/watchdog.
        if let Some(sequence) = message.kind.sequence() {
            self.safety_barrier_sequence = Some(sequence);
        }
        self.drive = None;
        self.controls.retain(|item| {
            !matches!(
                &item.kind,
                OutboundKind::Control { command_type, .. } if command_type.as_str() == "arm"
            )
        });
        self.safety_controls.push_back(message);
    }

    fn enqueue_control(&mut self, message: PendingOutbound) {
        self.controls.push_back(message);
    }

    fn enqueue_status(&mut self, message: PendingOutbound) -> bool {
        self.status.replace(message).is_some()
    }

    fn enqueue_drive(&mut self, message: PendingOutbound) -> bool {
        let replaced = self.drive.replace(message).is_some();
        if replaced {
            self.coalesced_drive_updates = self.coalesced_drive_updates.saturating_add(1);
        }
        replaced
    }

    fn pop_next(&mut self) -> Option<PendingOutbound> {
        if let Some(message) = self.safety_controls.pop_front() {
            return Some(message);
        }
        // Arming is an interactive control transition. Keep it ahead of
        // routine application ACK traffic so continuous telemetry cannot
        // starve a user's arm request.
        if let Some(message) = self.controls.pop_front() {
            return Some(message);
        }

        // ACKs must be delivered so the payload can advance its telemetry
        // stream, but drive setpoints must also reach the payload often enough
        // to refresh its command watchdog. Alternate these two queues rather
        // than allowing a continuous ACK backlog to starve drive heartbeats.
        let next = if self.prefer_drive_next {
            self.drive
                .take()
                .or_else(|| self.acknowledgements.pop_front())
        } else {
            self.acknowledgements
                .pop_front()
                .or_else(|| self.drive.take())
        };
        if let Some(message) = next {
            self.prefer_drive_next = matches!(
                &message.kind,
                OutboundKind::ApplicationAck { .. }
            );
            return Some(message);
        }

        self.status.take()
    }

    fn requeue_front(&mut self, message: PendingOutbound) {
        let is_stale_arm = match &message.kind {
            OutboundKind::Control { command_type, sequence } if command_type.as_str() == "arm" => self
                .safety_barrier_sequence
                .is_some_and(|barrier| sequence_precedes(*sequence, barrier)),
            _ => false,
        };
        if is_stale_arm {
            eprintln!(
                "[controller-outbound] dropped stale arm during retry sequence={:?} safety_barrier={:?}",
                message.kind.sequence(),
                self.safety_barrier_sequence,
            );
            return;
        }

        enum Destination {
            Ack,
            Safety,
            Control,
            Status,
            Drive,
        }
        let destination = match &message.kind {
            OutboundKind::ApplicationAck { .. } => Destination::Ack,
            OutboundKind::SafetyControl { .. } => Destination::Safety,
            OutboundKind::Control { .. } => Destination::Control,
            OutboundKind::Status { .. } => Destination::Status,
            OutboundKind::Drive { .. } => Destination::Drive,
        };
        match destination {
            Destination::Ack => self.acknowledgements.push_front(message),
            Destination::Safety => self.safety_controls.push_front(message),
            Destination::Control => self.controls.push_front(message),
            Destination::Status => {
                if self.status.is_none() {
                    self.status = Some(message);
                }
            }
            Destination::Drive => {
                // Keep a newer drive setpoint if one arrived while this one was in flight.
                if self.drive.is_none() {
                    self.drive = Some(message);
                }
            }
        }
    }

    fn mark_ack_delivered(&mut self, received_envelope_id: &str) {
        self.pending_ack_ids.remove(received_envelope_id);
        if self.recently_acked_ids.iter().any(|id| id == received_envelope_id) {
            return;
        }
        self.recently_acked_ids.push_back(received_envelope_id.to_string());
        while self.recently_acked_ids.len() > 256 {
            self.recently_acked_ids.pop_front();
        }
    }

    fn has_pending_disarm(&self) -> bool {
        self.safety_controls.iter().any(|item| item.kind.is_disarm())
    }

    fn depth(&self) -> usize {
        self.safety_controls.len()
            + self.acknowledgements.len()
            + self.controls.len()
            + (self.status.is_some() as usize)
            + (self.drive.is_some() as usize)
    }
}

#[derive(Serialize)]
struct PortInfo {
    name: String,
    description: String,
}

#[derive(Serialize, Clone)]
struct ControllerStatus {
    connected: bool,
    armed: bool,
    armed_confirmed: bool,
    session_active: bool,
    pending_action: Option<String>,
    payload_peer_id: u16,
    controller_device_id: Option<u16>,
    last_error: Option<String>,
    peer_connected: bool,
}

#[derive(Deserialize)]
struct PayloadManualStatus {
    protocol: String,
    version: u8,
    #[serde(rename = "type")]
    message_type: String,
    session_id: String,
    sequence: u32,
    armed: bool,
    session_active: bool,
}

// d3kble wraps every application message in this envelope before BLE transport.
// ACK envelopes omit `payload` and instead carry `ack_id`.
#[derive(Deserialize)]
struct D3kbleEnvelope {
    v: u8,
    id: String,
    kind: String,
    #[serde(default)]
    payload: Option<Value>,
}

fn unix_time_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn next_sequence(state: &mut ControllerState) -> u16 {
    let current = state.sequence;
    state.sequence = state.sequence.wrapping_add(1);
    (current & 0xffff) as u16
}

fn with_controller<T>(
    app: &AppHandle,
    f: impl FnOnce(&mut ControllerState) -> Result<T, String>,
) -> Result<T, String> {
    let state = app.state::<Controller>();
    let mut guard = state.0.lock().map_err(|e| e.to_string())?;
    f(&mut guard)
}

fn status_snapshot(state: &ControllerState) -> ControllerStatus {
    ControllerStatus {
        connected: state.port.is_some(),
        armed: state.armed,
        armed_confirmed: state.armed_confirmed,
        session_active: state.session_active,
        pending_action: state.pending_action.clone(),
        payload_peer_id: state.payload_peer_id,
        controller_device_id: state.controller_device_id,
        last_error: state.last_error.clone(),
        peer_connected: state.peer_connected,
    }
}

fn is_serial_io_error(error: &str) -> bool {
    error.starts_with("D3K UART write failed:")
        || error.starts_with("D3K UART flush failed:")
        || error.starts_with("D3K UART read failed:")
}

fn emit_status(app: &AppHandle, state: &ControllerState) {
    let _ = app.emit("controller-status", status_snapshot(state));
}

fn queue_comms_ack(
    state: &mut ControllerState,
    peer_id: u16,
    received_d3k_message_id: u16,
    received_message_id: &str,
) -> Result<bool, String> {
    let acknowledgement_sequence = state.sequence;
    state.sequence = state.sequence.wrapping_add(1);
    let acknowledgement_id = format!("{}-ack-{}", state.session_id, acknowledgement_sequence);
    let acknowledgement = json!({
        "v": 1,
        "id": acknowledgement_id,
        "kind": "ack",
        "ack_id": received_message_id,
    });
    let bytes = serde_json::to_vec(&acknowledgement).map_err(|e| e.to_string())?;
    let pending = PendingOutbound {
        peer_id,
        bytes,
        kind: OutboundKind::ApplicationAck {
            received_envelope_id: received_message_id.to_string(),
        },
        queued_at: Instant::now(),
        attempts: 0,
        source_d3k_message_id: Some(received_d3k_message_id),
    };
    let queued = state.outbound.enqueue_ack(pending);
    if queued {
        eprintln!(
            "[controller-outbound] ACK queued peer={} received_d3k_message_id={} received_envelope_id={} queue_depth={}",
            peer_id,
            received_d3k_message_id,
            received_message_id,
            state.outbound.depth(),
        );
    } else {
        eprintln!(
            "[controller-outbound] duplicate ACK suppressed peer={} received_d3k_message_id={} received_envelope_id={}",
            peer_id,
            received_d3k_message_id,
            received_message_id,
        );
    }
    Ok(queued)
}

fn flush_next_outbound(state: &mut ControllerState) -> Vec<d3k_protocol::Event> {
    // BLE_DATA_SENT should normally arrive promptly after the peer consumes a
    // message. If its event is lost, recover by retrying the same application
    // envelope; firmware BUSY responses keep the retry queued until the slot
    // is actually available. The envelope ID makes the retry idempotent.
    let in_flight_expired = state.in_flight.as_ref().is_some_and(|active| {
        active.started_at.elapsed() >= Duration::from_secs(5)
    });
    if in_flight_expired {
        if let Some(active) = state.in_flight.take() {
            eprintln!(
                "[controller-outbound] in-flight watchdog expired kind={} peer={} d3k_message_id={} age_ms={:.2}; requeueing",
                active.pending.kind.label(),
                active.peer_id,
                active.d3k_message_id,
                active.started_at.elapsed().as_secs_f64() * 1000.0,
            );
            state.outbound.requeue_front(active.pending);
            state.outbound_retry_after = Some(Instant::now() + Duration::from_millis(100));
        }
    }
    if state.in_flight.is_some() || state.port.is_none() || !state.peer_connected {
        return Vec::new();
    }
    if state
        .outbound_retry_after
        .is_some_and(|retry_after| Instant::now() < retry_after)
    {
        return Vec::new();
    }
    state.outbound_retry_after = None;

    let Some(pending) = state.outbound.pop_next() else {
        return Vec::new();
    };
    let serial_sequence = next_sequence(state);
    let queue_wait_ms = pending.queued_at.elapsed().as_secs_f64() * 1000.0;
    let started = Instant::now();
    let result = match state.port.as_mut() {
        Some(port) => d3k_protocol::send_message_with_events(
            port,
            serial_sequence,
            pending.peer_id,
            &pending.bytes,
        ),
        None => Err(d3k_protocol::SendMessageFailure {
            error: "Controller disconnected before outbound message could be sent".to_string(),
            events: Vec::new(),
        }),
    };

    match result {
        Ok((d3k_message_id, events)) => {
            eprintln!(
                "[controller-outbound] accepted kind={} peer={} app_sequence={:?} source_d3k_message_id={:?} d3k_sequence={} d3k_message_id={} queue_wait_ms={:.2} uart_round_trip_ms={:.2} attempts={} remaining_queue={}",
                pending.kind.label(),
                pending.peer_id,
                pending.kind.sequence(),
                pending.source_d3k_message_id,
                serial_sequence,
                d3k_message_id,
                queue_wait_ms,
                started.elapsed().as_secs_f64() * 1000.0,
                pending.attempts,
                state.outbound.depth(),
            );
            state.in_flight = Some(InFlightOutbound {
                peer_id: pending.peer_id,
                d3k_message_id,
                pending,
                started_at: Instant::now(),
            });
            events
        }
        Err(failure) => {
            let busy = failure.error.contains("status 4");
            let kind = pending.kind.label();
            let peer_id = pending.peer_id;
            let attempts = pending.attempts;
            let nested_events = failure.events;
            let error = failure.error;
            state.outbound.requeue_front(pending);
            state.outbound_retry_after = Some(
                Instant::now() + if busy { Duration::from_millis(40) } else { Duration::from_millis(200) },
            );
            eprintln!(
                "[controller-outbound] send-rejected kind={} peer={} d3k_sequence={} busy={} attempts={} retry_in_ms={} preserved_events={} error={}",
                kind,
                peer_id,
                serial_sequence,
                busy,
                attempts,
                if busy { 40 } else { 200 },
                nested_events.len(),
                error,
            );
            if is_serial_io_error(&error) {
                state.port = None;
                state.peer_connected = false;
                state.armed = false;
                state.armed_confirmed = false;
                state.session_active = false;
                state.pending_action = None;
                state.last_error = Some(error);
            } else if !busy {
                state.last_error = Some(error);
            }
            nested_events
        }
    }
}

fn requeue_in_flight(
    state: &mut ControllerState,
    peer_id: u16,
    message_id: u16,
    reason: &str,
) {
    let matches = state.in_flight.as_ref().is_some_and(|active| {
        active.peer_id == peer_id && active.d3k_message_id == message_id
    });
    if !matches {
        eprintln!(
            "[controller-outbound] terminal event did not match active message peer={} message_id={} active={:?}",
            peer_id,
            message_id,
            state.in_flight.as_ref().map(|active| (active.peer_id, active.d3k_message_id)),
        );
        return;
    }

    let mut active = state.in_flight.take().expect("matching in-flight message");
    let kind = active.pending.kind.label();
    let elapsed_ms = active.started_at.elapsed().as_secs_f64() * 1000.0;
    active.pending.attempts = active.pending.attempts.saturating_add(1);
    let attempts = active.pending.attempts;
    state.outbound.requeue_front(active.pending);
    let retry_ms = (50u64.saturating_mul(1u64 << attempts.min(3))).min(400);
    state.outbound_retry_after = Some(Instant::now() + Duration::from_millis(retry_ms));
    eprintln!(
        "[controller-outbound] delivery-not-confirmed kind={} peer={} d3k_message_id={} in_flight_ms={:.2} attempts={} retry_in_ms={} reason={}",
        kind,
        peer_id,
        message_id,
        elapsed_ms,
        attempts,
        retry_ms,
        reason,
    );
}

fn mark_in_flight_delivered(state: &mut ControllerState, peer_id: u16, message_id: u16) {
    let matches = state.in_flight.as_ref().is_some_and(|active| {
        active.peer_id == peer_id && active.d3k_message_id == message_id
    });
    if !matches {
        eprintln!(
            "[controller-outbound] delivery event did not match active message peer={} message_id={} active={:?}",
            peer_id,
            message_id,
            state.in_flight.as_ref().map(|active| (active.peer_id, active.d3k_message_id)),
        );
        return;
    }

    let active = state.in_flight.take().expect("matching in-flight message");
    let kind = active.pending.kind.label();
    let elapsed_ms = active.started_at.elapsed().as_secs_f64() * 1000.0;
    if let Some(received_id) = active.pending.kind.ack_envelope_id() {
        state.outbound.mark_ack_delivered(received_id);
    }
    eprintln!(
        "[controller-outbound] delivered kind={} peer={} app_sequence={:?} source_d3k_message_id={:?} d3k_message_id={} delivery_ms={:.2} queue_depth={}",
        kind,
        peer_id,
        active.pending.kind.sequence(),
        active.pending.source_d3k_message_id,
        message_id,
        elapsed_ms,
        state.outbound.depth(),
    );
}

fn handle_events(
    app: &AppHandle,
    state: &mut ControllerState,
    initial_events: Vec<d3k_protocol::Event>,
) {
    let handle_started = Instant::now();
    let initial_event_count = initial_events.len();
    let mut telemetry_count = 0usize;
    let mut queue: VecDeque<d3k_protocol::Event> = initial_events.into();

    // Drain every event already returned by the UART command before starting
    // another BLE message. This lets ACKs and terminal delivery events update
    // the scheduler before it selects the next outbound message.
    loop {
        while let Some(event) = queue.pop_front() {
            // BLE_DATA_SENT: peer_id:u16, message_id:u16, total_length:u16.
            if event.opcode == 0x8D && event.payload.len() == 6 {
                let peer_id = u16::from_le_bytes([event.payload[0], event.payload[1]]);
                let message_id = u16::from_le_bytes([event.payload[2], event.payload[3]]);
                if peer_id == state.payload_peer_id {
                    mark_in_flight_delivered(state, peer_id, message_id);
                }
                continue;
            }

            // BLE_DATA_EXPIRED is an inbound unread-message expiry, not an
            // outbound delivery completion. Outbound failures arrive as
            // BLE_ERROR (direction 0), so do not release our TX slot here.
            if event.opcode == 0x8E && event.payload.len() == 4 {
                let peer_id = u16::from_le_bytes([event.payload[0], event.payload[1]]);
                let message_id = u16::from_le_bytes([event.payload[2], event.payload[3]]);
                eprintln!(
                    "[controller-ble-rx] inbound message expired peer={} message_id={}",
                    peer_id,
                    message_id,
                );
                continue;
            }

            // BLE_ERROR: direction:u8, peer_id:u16, message_id:u16, ...
            // direction 0 denotes a failure for a message sent by this node.
            if event.opcode == 0x8C && event.payload.len() == 8 {
                let direction = event.payload[0];
                let peer_id = u16::from_le_bytes([event.payload[1], event.payload[2]]);
                let message_id = u16::from_le_bytes([event.payload[3], event.payload[4]]);
                let error_code = event.payload[6];
                if direction == 0 && peer_id == state.payload_peer_id {
                    requeue_in_flight(
                        state,
                        peer_id,
                        message_id,
                        &format!("BLE_ERROR code={error_code}"),
                    );
                }
                continue;
            }

            // BLE_PEER_DISCONNECTED: peer_id:u16, reason:u8.
            if event.opcode == 0x89 && event.payload.len() == 3 {
                let peer_id = u16::from_le_bytes([event.payload[0], event.payload[1]]);
                if peer_id == state.payload_peer_id {
                    state.peer_connected = false;
                    if let Some(active) = state.in_flight.take() {
                        if active.peer_id == peer_id {
                            eprintln!(
                                "[controller-outbound] peer disconnected with message in flight kind={} d3k_message_id={}; requeueing",
                                active.pending.kind.label(),
                                active.d3k_message_id,
                            );
                            state.outbound.requeue_front(active.pending);
                        } else {
                            state.in_flight = Some(active);
                        }
                    }
                }
                continue;
            }

            // BLE_DATA_RECEIVED: peer_id:u16, message_id:u16, total_length:u16.
            if event.opcode != 0x8B || event.payload.len() != 6 {
                continue;
            }
            let peer_id = u16::from_le_bytes([event.payload[0], event.payload[1]]);
            let message_id = u16::from_le_bytes([event.payload[2], event.payload[3]]);
            if peer_id != state.payload_peer_id || state.port.is_none() {
                continue;
            }

            let sequence = next_sequence(state);
            let retrieve_started = Instant::now();
            let received = match state.port.as_mut() {
                Some(port) => d3k_protocol::get_received_data(port, sequence, peer_id, message_id),
                None => continue,
            };
            match received {
                Ok((bytes, nested_events)) => {
                    let retrieve_ms = retrieve_started.elapsed().as_secs_f64() * 1000.0;
                    queue.extend(nested_events);
                    let envelope = match serde_json::from_slice::<D3kbleEnvelope>(&bytes) {
                        Ok(envelope) if envelope.v == 1 && !envelope.id.is_empty() => envelope,
                        Ok(_) => {
                            state.last_error = Some("Unsupported d3kble application envelope".into());
                            emit_status(app, state);
                            continue;
                        }
                        Err(error) => {
                            state.last_error = Some(format!("Invalid d3kble application envelope: {error}"));
                            emit_status(app, state);
                            continue;
                        }
                    };

                    // Application ACK envelopes resolve a ticket on the sender
                    // and must not themselves generate another application ACK.
                    if envelope.kind == "ack" {
                        eprintln!(
                            "[controller-ble-rx] app-ack peer={} message_id={} envelope_id={} retrieve_ms={:.2} total_handler_ms={:.2}",
                            peer_id,
                            message_id,
                            envelope.id,
                            retrieve_ms,
                            handle_started.elapsed().as_secs_f64() * 1000.0,
                        );
                        continue;
                    }
                    if !matches!(envelope.kind.as_str(), "message" | "request" | "response") {
                        state.last_error = Some(format!("Unsupported d3kble message kind: {}", envelope.kind));
                        emit_status(app, state);
                        continue;
                    }

                    if let Err(error) = queue_comms_ack(state, peer_id, message_id, &envelope.id) {
                        state.last_error = Some(format!("Could not queue payload acknowledgement: {error}"));
                        emit_status(app, state);
                    }

                    if envelope.kind != "message" {
                        continue;
                    }
                    let Some(mut payload) = envelope.payload else {
                        state.last_error = Some("Payload d3kble message has no payload".into());
                        emit_status(app, state);
                        continue;
                    };
                    let protocol = payload.get("protocol").and_then(Value::as_str).unwrap_or("unknown");
                    let message_type = payload.get("type").and_then(Value::as_str).unwrap_or("unknown");
                    eprintln!(
                        "[controller-ble-rx] message peer={} message_id={} envelope_id={} protocol={} type={} payload_bytes={} retrieve_ms={:.2} since_handler_start_ms={:.2}",
                        peer_id,
                        message_id,
                        envelope.id,
                        protocol,
                        message_type,
                        bytes.len(),
                        retrieve_ms,
                        handle_started.elapsed().as_secs_f64() * 1000.0,
                    );
                    if protocol.starts_with("payload-manual-position") {
                        if let Some(object) = payload.as_object_mut() {
                            object.insert("controller_received_unix_ms".into(), json!(unix_time_ms()));
                        }
                    }
                    if payload.get("protocol").and_then(Value::as_str) == Some("payload-manual-position-frame")
                        && payload.get("version").and_then(Value::as_u64) == Some(1)
                        && payload.get("type").and_then(Value::as_str) == Some("reference_frame")
                    {
                        let emit_started = Instant::now();
                        let emit_result = app.emit("payload-position-frame", payload);
                        eprintln!(
                            "[controller-telemetry] frame emitted ok={} emit_ms={:.2} total_handle_ms={:.2}",
                            emit_result.is_ok(),
                            emit_started.elapsed().as_secs_f64() * 1000.0,
                            handle_started.elapsed().as_secs_f64() * 1000.0,
                        );
                        continue;
                    }
                    if payload.get("protocol").and_then(Value::as_str) == Some("payload-manual-position")
                        && payload.get("version").and_then(Value::as_u64) == Some(1)
                        && payload.get("type").and_then(Value::as_str) == Some("position")
                    {
                        telemetry_count += 1;
                        let telemetry_sequence = payload.get("sequence").and_then(Value::as_u64);
                        let device_us = payload.get("device_monotonic_us").and_then(Value::as_u64);
                        let position = payload.get("position_m").cloned().unwrap_or(Value::Null);
                        let controller_received_ms = payload.get("controller_received_unix_ms").and_then(Value::as_u64);
                        let emit_started = Instant::now();
                        let emit_result = app.emit("payload-position", payload);
                        eprintln!(
                            "[controller-telemetry] position emitted: sequence={:?} device_us={:?} position={} host_received_unix_ms={:?} emit_ok={} emit_ms={:.2} total_handle_ms={:.2}",
                            telemetry_sequence,
                            device_us,
                            position,
                            controller_received_ms,
                            emit_result.is_ok(),
                            emit_started.elapsed().as_secs_f64() * 1000.0,
                            handle_started.elapsed().as_secs_f64() * 1000.0,
                        );
                        continue;
                    }
                    if payload.get("protocol").and_then(Value::as_str) == Some("payload-manual-position-status")
                        && payload.get("version").and_then(Value::as_u64) == Some(1)
                        && payload.get("type").and_then(Value::as_str) == Some("status")
                    {
                        let emit_started = Instant::now();
                        let emit_result = app.emit("payload-position-status", payload);
                        eprintln!(
                            "[controller-telemetry] status emitted ok={} emit_ms={:.2} total_handle_ms={:.2}",
                            emit_result.is_ok(),
                            emit_started.elapsed().as_secs_f64() * 1000.0,
                            handle_started.elapsed().as_secs_f64() * 1000.0,
                        );
                        continue;
                    }
                    match serde_json::from_value::<PayloadManualStatus>(payload) {
                        Ok(payload_status)
                            if payload_status.protocol == "payload-manual-control-status"
                                && payload_status.version == 1
                                && payload_status.message_type == "status"
                                && payload_status.session_id == state.session_id
                                && payload_status.sequence >= state.pending_sequence =>
                        {
                            state.armed = payload_status.armed;
                            state.armed_confirmed = true;
                            state.session_active = payload_status.session_active;
                            state.pending_action = None;
                            state.last_error = None;
                            emit_status(app, state);
                        }
                        Ok(_) => {}
                        Err(error) => {
                            state.last_error = Some(format!("Invalid payload status message: {error}"));
                            emit_status(app, state);
                        }
                    }
                }
                Err(error) => {
                    eprintln!(
                        "[controller-ble-rx] get-received-data-failed peer={} message_id={} error={}",
                        peer_id,
                        message_id,
                        error,
                    );
                    state.last_error = Some(format!("Could not retrieve payload message: {error}"));
                    emit_status(app, state);
                }
            }
        }

        // Outbound BLE messages share one firmware transmit slot per peer.
        // A completed message frees that slot; the scheduler sends the highest
        // priority pending item and then waits for its terminal BLE event.
        let nested_events = flush_next_outbound(state);
        if nested_events.is_empty() {
            break;
        }
        queue.extend(nested_events);
    }

    if initial_event_count > 0 || telemetry_count > 0 {
        eprintln!(
            "[controller-ble-rx] handle_events done initial_events={} telemetry_positions={} total_ms={:.2} outbound_queue_depth={} in_flight={:?}",
            initial_event_count,
            telemetry_count,
            handle_started.elapsed().as_secs_f64() * 1000.0,
            state.outbound.depth(),
            state.in_flight.as_ref().map(|active| (active.peer_id, active.d3k_message_id, active.pending.kind.label())),
        );
    }
}

#[tauri::command]
async fn list_ports() -> Vec<PortInfo> {
    tauri::async_runtime::spawn_blocking(|| {
        serialport::available_ports()
            .unwrap_or_default()
            .into_iter()
            .map(|p: SerialPortInfo| {
                let description = match p.port_type {
                    serialport::SerialPortType::UsbPort(info) => format!(
                        "{} {} · {}",
                        info.manufacturer.unwrap_or_default(),
                        info.product.unwrap_or_default(),
                        info.serial_number.unwrap_or_default()
                    ),
                    serialport::SerialPortType::BluetoothPort => "Bluetooth serial".into(),
                    serialport::SerialPortType::PciPort => "PCI serial".into(),
                    serialport::SerialPortType::Unknown => "Serial device / simulator PTY".into(),
                };
                PortInfo {
                    name: p.port_name,
                    description,
                }
            })
            .collect()
    })
    .await
    .unwrap_or_default()
}

#[tauri::command]
async fn connect_controller(
    port_name: String,
    baud_rate: u32,
    payload_peer_id: u16,
    app: AppHandle,
) -> Result<(), String> {
    if !(1..=65534).contains(&payload_peer_id) {
        return Err("Payload peer ID must be in 1..65534".into());
    }
    if ![115_200, 460_800].contains(&baud_rate) {
        return Err("Serial speed must be 115200 (USB CDC) or 460800 (UART / simulator PTY)".into());
    }

    tauri::async_runtime::spawn_blocking(move || {
        with_controller(&app, |state| {
            state.port = None;
            state.armed = false;
            state.armed_confirmed = false;
            state.session_active = false;
            state.pending_action = None;
            state.pending_sequence = 0;
            state.peer_connected = false;
            state.controller_device_id = None;
            state.outbound = OutboundQueue::default();
            state.in_flight = None;
            state.outbound_retry_after = None;

            let mut port = serialport::new(&port_name, baud_rate)
                .timeout(Duration::from_millis(25))
                .open()
                .map_err(|e| format!("Could not open controller D3K node {port_name}: {e}"))?;

            let mut sequence = 1u16;
            let ping = d3k_protocol::command(&mut port, sequence, d3k_protocol::OP_PING, &[])?;
            sequence = sequence.wrapping_add(1);
            if ping.payload.as_slice() != b"PONG" {
                return Err("Controller D3K node returned an invalid PING response".into());
            }

            let info = d3k_protocol::command(&mut port, sequence, d3k_protocol::OP_GET_INFO, &[])?;
            sequence = sequence.wrapping_add(1);
            if info.payload.len() != 21 {
                return Err(format!(
                    "GET_INFO returned {} bytes; expected 21 (check the D3K firmware version)",
                    info.payload.len()
                ));
            }
            if info.payload[3] != 1 {
                return Err(format!(
                    "Unsupported D3K host protocol version {}; expected 1",
                    info.payload[3]
                ));
            }
            if info.payload[12] != 1 || info.payload[13] & 0x02 == 0 {
                return Err("Controller D3K firmware does not advertise BLE HUB/Central support".into());
            }
            let controller_id = u16::from_le_bytes([info.payload[4], info.payload[5]]);
            if controller_id == payload_peer_id {
                return Err("Controller node ID and payload peer ID must be different".into());
            }

            // Subscribe to BLE lifecycle/data events so received payload status messages
            // can be consumed using GET_RECEIVED_DATA and reported to the UI.
            let subscribe = d3k_protocol::command(
                &mut port,
                sequence,
                d3k_protocol::OP_SUBSCRIBE,
                &d3k_protocol::BLE_EVENT_MASK.to_le_bytes(),
            )?;
            sequence = sequence.wrapping_add(1);
            if subscribe.payload.len() != 2 {
                return Err("SUBSCRIBE returned an invalid response".into());
            }

            state.payload_peer_id = payload_peer_id;
            state.controller_device_id = Some(controller_id);
            state.sequence = sequence as u32;
            state.session_id = format!(
                "ui-{:x}",
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos()
            );
            state.last_error = None;
            state.peer_connected = false;
            state.port = Some(port);
            emit_status(&app, state);
            Ok(())
        })
    })
    .await
    .map_err(|e| e.to_string())?
}

fn disarm_is_pending(state: &ControllerState) -> bool {
    state.outbound.has_pending_disarm()
        || state
            .in_flight
            .as_ref()
            .is_some_and(|active| active.pending.kind.is_disarm())
}

fn wait_for_disarm_delivery(app: &AppHandle, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    loop {
        let completed = with_controller(app, |state| {
            if !disarm_is_pending(state) || state.port.is_none() || !state.peer_connected {
                return Ok(true);
            }
            let sequence = next_sequence(state);
            let poll_result = match state.port.as_mut() {
                Some(port) => d3k_protocol::peer_connected(port, sequence, state.payload_peer_id),
                None => return Ok(true),
            };
            match poll_result {
                Ok((connected, events)) => {
                    state.peer_connected = connected;
                    if !connected {
                        state.armed = false;
                        state.armed_confirmed = false;
                        state.session_active = false;
                        state.pending_action = None;
                    }
                    handle_events(app, state, events);
                    Ok(!disarm_is_pending(state))
                }
                Err(error) => {
                    eprintln!("[controller-outbound] disarm drain poll failed: {error}");
                    if is_serial_io_error(&error) {
                        state.port = None;
                        state.peer_connected = false;
                    }
                    Ok(false)
                }
            }
        });
        match completed {
            Ok(true) => return,
            Ok(false) => {}
            Err(error) => {
                eprintln!("[controller-outbound] disarm drain aborted: {error}");
                return;
            }
        }
        if Instant::now() >= deadline {
            eprintln!("[controller-outbound] timed out waiting for disarm delivery");
            return;
        }
        thread::sleep(Duration::from_millis(20));
    }
}

#[tauri::command]
async fn disconnect_controller(app: AppHandle) -> Result<(), String> {
    let drain_result = tauri::async_runtime::spawn_blocking({
        let app = app.clone();
        move || {
            let already_requested = with_controller(&app, |state| {
                Ok(disarm_is_pending(state) || state.pending_action.as_deref() == Some("disarm"))
            })
            .unwrap_or(false);
            if !already_requested {
                if let Err(error) = send_control_inner(&app, "disarm", 0.0, 0.0, 0.0) {
                    eprintln!("[controller-outbound] could not queue disarm during disconnect: {error}");
                }
            }
            wait_for_disarm_delivery(&app, Duration::from_secs(3));
        }
    })
    .await;
    if let Err(error) = drain_result {
        eprintln!("[controller-outbound] disarm drain worker failed: {error}");
    }
    with_controller(&app, |state| {
        state.armed = false;
        state.armed_confirmed = false;
        state.session_active = false;
        state.pending_action = None;
        state.peer_connected = false;
        state.port = None;
        state.outbound = OutboundQueue::default();
        state.in_flight = None;
        state.outbound_retry_after = None;
        emit_status(&app, state);
        Ok(())
    })
}

#[tauri::command]
async fn get_status(app: AppHandle) -> Result<ControllerStatus, String> {
    with_controller(&app, |state| Ok(status_snapshot(state)))
}

#[tauri::command]
async fn send_control(
    command_type: String,
    vx_m_s: f64,
    vy_m_s: f64,
    yaw_rate_rad_s: f64,
    app: AppHandle,
) -> Result<(), String> {
    tauri::async_runtime::spawn_blocking(move || {
        send_control_inner(&app, &command_type, vx_m_s, vy_m_s, yaw_rate_rad_s)
    })
    .await
    .map_err(|e| e.to_string())?
}

fn send_control_inner(
    app: &AppHandle,
    command_type: &str,
    vx: f64,
    vy: f64,
    yaw_rate: f64,
) -> Result<(), String> {
    if !["arm", "drive", "stop", "disarm", "status"].contains(&command_type) {
        return Err("Unsupported manual-control command".into());
    }
    if command_type == "drive" && (!vx.is_finite() || !vy.is_finite() || !yaw_rate.is_finite()) {
        return Err("Drive values must be finite".into());
    }

    with_controller(app, |state| {
        if state.port.is_none() {
            return Err("Connect a controller D3K node first".into());
        }
        if !state.peer_connected {
            return Err("Controller D3K node is not connected to the payload BLE peer".into());
        }

        let sequence = state.sequence;
        state.sequence = state.sequence.wrapping_add(1);
        let peer_id = state.payload_peer_id;
        let session_id = state.session_id.clone();
        let payload = if command_type == "drive" {
            json!({
                "protocol": "payload-manual-control",
                "version": 2,
                "type": "drive",
                "session_id": session_id,
                "sequence": sequence,
                "vx_m_s": vx,
                "vy_m_s": vy,
                "yaw_rate_rad_s": yaw_rate
            })
        } else {
            json!({
                "protocol": "payload-manual-control",
                "version": 2,
                "type": command_type,
                "session_id": session_id,
                "sequence": sequence
            })
        };
        let message_id = format!("{}-{}", session_id, sequence);
        let envelope = json!({ "v": 1, "id": message_id, "kind": "message", "payload": payload });
        let bytes = serde_json::to_vec(&envelope).map_err(|e| e.to_string())?;
        if bytes.len() > 496 {
            return Err("D3K application message exceeds 496 bytes".into());
        }
        let pending = PendingOutbound {
            peer_id,
            bytes,
            kind: match command_type {
                "stop" | "disarm" => OutboundKind::SafetyControl {
                    command_type: command_type.to_string(),
                    sequence,
                },
                "drive" => OutboundKind::Drive { sequence },
                "status" => OutboundKind::Status { sequence },
                _ => OutboundKind::Control {
                    command_type: command_type.to_string(),
                    sequence,
                },
            },
            queued_at: Instant::now(),
            attempts: 0,
            source_d3k_message_id: None,
        };

        if command_type == "arm" || command_type == "disarm" {
            state.pending_action = Some(command_type.to_string());
            state.pending_sequence = sequence;
            state.armed_confirmed = false;
        }

        let replaced = match command_type {
            "drive" => state.outbound.enqueue_drive(pending),
            "stop" | "disarm" => {
                state.outbound.enqueue_safety_control(pending);
                false
            }
            "status" => state.outbound.enqueue_status(pending),
            _ => {
                state.outbound.enqueue_control(pending);
                false
            }
        };
        state.last_error = None;
        eprintln!(
            "[controller-command] queued type={} sequence={} peer={} replaced_pending={} queue_depth={} coalesced_drive_total={}",
            command_type,
            sequence,
            peer_id,
            replaced,
            state.outbound.depth(),
            state.outbound.coalesced_drive_updates,
        );

        // This attempts at most one outbound message. The scheduler waits for
        // BLE_DATA_SENT or an outbound BLE_ERROR before starting the next one.
        handle_events(app, state, Vec::new());
        emit_status(app, state);
        Ok(())
    })
}

#[tauri::command]
async fn refresh_peer_status(app: AppHandle) -> Result<ControllerStatus, String> {
    tauri::async_runtime::spawn_blocking(move || {
        let poll_started = Instant::now();
        with_controller(&app, |state| {
            let peer_id = state.payload_peer_id;
            let sequence = next_sequence(state);
            if let Some(port) = state.port.as_mut() {
                match d3k_protocol::peer_connected(port, sequence, peer_id) {
                    Ok((connected, events)) => {
                        let was_connected = state.peer_connected;
                        state.peer_connected = connected;
                        if was_connected && !connected {
                            state.armed = false;
                            state.armed_confirmed = false;
                            state.session_active = false;
                            state.pending_action = None;
                            if let Some(active) = state.in_flight.take() {
                                if active.peer_id == peer_id {
                                    eprintln!(
                                        "[controller-outbound] peer status disconnected with message in flight kind={} d3k_message_id={}; requeueing",
                                        active.pending.kind.label(),
                                        active.d3k_message_id,
                                    );
                                    state.outbound.requeue_front(active.pending);
                                } else {
                                    state.in_flight = Some(active);
                                }
                            }
                        }
                        state.last_error = None;
                        handle_events(&app, state, events);
                    }
                    Err(error) => {
                        state.peer_connected = false;
                        if is_serial_io_error(&error) {
                            state.port = None;
                            state.armed = false;
                            state.armed_confirmed = false;
                            state.session_active = false;
                            state.pending_action = None;
                        }
                        // NOT_FOUND is normal while the HUB has not discovered this peer yet.
                        if !error.ends_with("status 10") {
                            state.last_error = Some(error);
                        } else {
                            state.last_error = None;
                        }
                    }
                }
            } else {
                state.peer_connected = false;
            }
            emit_status(&app, state);
            let snapshot = status_snapshot(state);
            eprintln!(
                "[controller-poll] peer_status_poll connected={} peer_connected={} elapsed_ms={:.2}",
                snapshot.connected,
                snapshot.peer_connected,
                poll_started.elapsed().as_secs_f64() * 1000.0,
            );
            Ok(snapshot)
        })
    })
    .await
    .map_err(|e| e.to_string())?
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    tauri::Builder::default()
        .manage(Controller(Mutex::new(ControllerState {
            port: None,
            sequence: 1,
            session_id: format!("ui-{nonce:x}"),
            payload_peer_id: 1,
            controller_device_id: None,
            armed: false,
            armed_confirmed: false,
            session_active: false,
            pending_action: None,
            pending_sequence: 0,
            last_error: None,
            peer_connected: false,
            outbound: OutboundQueue::default(),
            in_flight: None,
            outbound_retry_after: None,
        })))
        .invoke_handler(tauri::generate_handler![
            list_ports,
            connect_controller,
            disconnect_controller,
            get_status,
            refresh_peer_status,
            send_control
        ])
        .run(tauri::generate_context!())
        .expect("error while running payload manual controller");
}

#[cfg(test)]
mod outbound_scheduler_tests {
    use super::*;

    fn pending(kind: OutboundKind) -> PendingOutbound {
        PendingOutbound {
            peer_id: 18,
            bytes: Vec::new(),
            kind,
            queued_at: Instant::now(),
            attempts: 0,
            source_d3k_message_id: None,
        }
    }

    #[test]
    fn safety_then_arm_precede_ack_backlog_and_drive() {
        let mut queue = OutboundQueue::default();
        queue.enqueue_drive(pending(OutboundKind::Drive { sequence: 1 }));
        queue.enqueue_control(pending(OutboundKind::Control {
            command_type: "arm".into(),
            sequence: 2,
        }));
        queue.enqueue_ack(pending(OutboundKind::ApplicationAck {
            received_envelope_id: "payload-message-1".into(),
        }));
        queue.enqueue_safety_control(pending(OutboundKind::SafetyControl {
            command_type: "stop".into(),
            sequence: 4,
        }));

        assert_eq!(queue.pop_next().unwrap().kind.label(), "stop");
        assert_eq!(queue.pop_next().unwrap().kind.label(), "arm");
        assert_eq!(queue.pop_next().unwrap().kind.label(), "application_ack");
        assert_eq!(queue.pop_next().unwrap().kind.label(), "drive");
        assert!(queue.pop_next().is_none());
    }

    #[test]
    fn arm_is_not_starved_by_a_backlog_of_application_acks() {
        let mut queue = OutboundQueue::default();
        for index in 0..8 {
            assert!(queue.enqueue_ack(pending(OutboundKind::ApplicationAck {
                received_envelope_id: format!("payload-message-{index}"),
            })));
        }
        queue.enqueue_control(pending(OutboundKind::Control {
            command_type: "arm".into(),
            sequence: 9,
        }));

        assert_eq!(queue.pop_next().unwrap().kind.label(), "arm");
        assert_eq!(queue.pop_next().unwrap().kind.label(), "application_ack");
    }

    #[test]
    fn duplicate_ack_is_suppressed_until_and_after_delivery() {
        let mut queue = OutboundQueue::default();
        let first = pending(OutboundKind::ApplicationAck {
            received_envelope_id: "payload-message-1".into(),
        });
        let duplicate = pending(OutboundKind::ApplicationAck {
            received_envelope_id: "payload-message-1".into(),
        });

        assert!(queue.enqueue_ack(first));
        assert!(!queue.enqueue_ack(duplicate));
        let active = queue.pop_next().unwrap();
        assert_eq!(active.kind.label(), "application_ack");
        queue.mark_ack_delivered("payload-message-1");
        assert!(!queue.enqueue_ack(pending(OutboundKind::ApplicationAck {
            received_envelope_id: "payload-message-1".into(),
        })));
    }

    #[test]
    fn newest_drive_replaces_older_pending_setpoint() {
        let mut queue = OutboundQueue::default();
        assert!(!queue.enqueue_drive(pending(OutboundKind::Drive { sequence: 10 })));
        assert!(queue.enqueue_drive(pending(OutboundKind::Drive { sequence: 11 })));
        assert_eq!(queue.coalesced_drive_updates, 1);
        let latest = queue.pop_next().unwrap();
        assert!(matches!(latest.kind, OutboundKind::Drive { sequence: 11 }));
    }

    #[test]
    fn stop_cancels_older_unsent_arm_and_drive_but_keeps_later_arm() {
        let mut queue = OutboundQueue::default();
        queue.enqueue_control(pending(OutboundKind::Control {
            command_type: "arm".into(),
            sequence: 1,
        }));
        queue.enqueue_drive(pending(OutboundKind::Drive { sequence: 2 }));
        queue.enqueue_safety_control(pending(OutboundKind::SafetyControl {
            command_type: "stop".into(),
            sequence: 3,
        }));
        queue.enqueue_control(pending(OutboundKind::Control {
            command_type: "arm".into(),
            sequence: 4,
        }));

        assert_eq!(queue.pop_next().unwrap().kind.label(), "stop");
        assert_eq!(queue.pop_next().unwrap().kind.sequence(), Some(4));
        assert!(queue.pop_next().is_none());
    }

    #[test]
    fn failed_in_flight_arm_is_not_requeued_after_newer_safety_command() {
        let mut queue = OutboundQueue::default();
        queue.enqueue_safety_control(pending(OutboundKind::SafetyControl {
            command_type: "stop".into(),
            sequence: 20,
        }));
        queue.requeue_front(pending(OutboundKind::Control {
            command_type: "arm".into(),
            sequence: 19,
        }));

        assert_eq!(queue.pop_next().unwrap().kind.label(), "stop");
        assert!(queue.pop_next().is_none());

        // A new arm explicitly requested after the stop remains eligible.
        queue.enqueue_control(pending(OutboundKind::Control {
            command_type: "arm".into(),
            sequence: 21,
        }));
        assert_eq!(queue.pop_next().unwrap().kind.sequence(), Some(21));
    }

    #[test]
    fn sequence_barrier_handles_u32_wraparound() {
        assert!(sequence_precedes(u32::MAX - 2, 1));
        assert!(!sequence_precedes(2, 1));
        assert!(!sequence_precedes(1, 1));
    }

    #[test]
    fn retrying_ack_preserves_arm_priority_over_regular_messages() {
        let mut queue = OutboundQueue::default();
        let ack = pending(OutboundKind::ApplicationAck {
            received_envelope_id: "payload-message-2".into(),
        });
        assert!(queue.enqueue_ack(ack));
        queue.enqueue_control(pending(OutboundKind::Control {
            command_type: "arm".into(),
            sequence: 7,
        }));
        assert_eq!(queue.pop_next().unwrap().kind.label(), "arm");
        let ack = queue.pop_next().unwrap();
        queue.requeue_front(ack);
        assert_eq!(queue.pop_next().unwrap().kind.label(), "application_ack");
    }

    #[test]
    fn drive_gets_a_transmit_slot_between_backlogged_acknowledgements() {
        let mut queue = OutboundQueue::default();
        for index in 0..4 {
            assert!(queue.enqueue_ack(pending(OutboundKind::ApplicationAck {
                received_envelope_id: format!("payload-message-{index}"),
            })));
        }
        queue.enqueue_drive(pending(OutboundKind::Drive { sequence: 10 }));

        assert_eq!(queue.pop_next().unwrap().kind.label(), "application_ack");
        assert!(matches!(
            queue.pop_next().unwrap().kind,
            OutboundKind::Drive { sequence: 10 }
        ));

        // A fresh heartbeat arrives while ACK backlog remains. It must again
        // get a slot after no more than one additional ACK transmission.
        queue.enqueue_drive(pending(OutboundKind::Drive { sequence: 11 }));
        assert_eq!(queue.pop_next().unwrap().kind.label(), "application_ack");
        assert!(matches!(
            queue.pop_next().unwrap().kind,
            OutboundKind::Drive { sequence: 11 }
        ));
    }
}
