mod d3k_protocol;

use serde::{Deserialize, Serialize};
use serde_json::json;
use serialport::{SerialPort, SerialPortInfo};
use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
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

fn handle_events(
    app: &AppHandle,
    state: &mut ControllerState,
    initial_events: Vec<d3k_protocol::Event>,
) {
    let mut queue: VecDeque<d3k_protocol::Event> = initial_events.into();
    while let Some(event) = queue.pop_front() {
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
        let received = match state.port.as_mut() {
            Some(port) => d3k_protocol::get_received_data(port, sequence, peer_id, message_id),
            None => continue,
        };
        match received {
            Ok((bytes, nested_events)) => {
                queue.extend(nested_events);
                match serde_json::from_slice::<PayloadManualStatus>(&bytes) {
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
                    Ok(_) => {
                        // Other application messages are not part of the manual-control status contract.
                    }
                    Err(error) => {
                        state.last_error = Some(format!("Invalid payload status message: {error}"));
                        emit_status(app, state);
                    }
                }
            }
            Err(error) => {
                state.last_error = Some(format!("Could not retrieve payload status: {error}"));
                emit_status(app, state);
            }
        }
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

#[tauri::command]
async fn disconnect_controller(app: AppHandle) -> Result<(), String> {
    let _ = tauri::async_runtime::spawn_blocking({
        let app = app.clone();
        move || send_control_inner(&app, "disarm", 0.0, 0.0, 0.0)
    })
    .await;
    with_controller(&app, |state| {
        state.armed = false;
        state.armed_confirmed = false;
        state.session_active = false;
        state.pending_action = None;
        state.peer_connected = false;
        state.port = None;
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
                "version": 1,
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
                "version": 1,
                "type": command_type,
                "session_id": session_id,
                "sequence": sequence
            })
        };
        let bytes = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;
        if bytes.len() > 496 {
            return Err("D3K application message exceeds 496 bytes".into());
        }

        let result = match state.port.as_mut() {
            Some(port) => d3k_protocol::send_message(port, (sequence & 0xffff) as u16, peer_id, &bytes),
            None => return Err("Controller disconnected".into()),
        };
        match result {
            Ok((_message_id, events)) => {
                if command_type == "arm" || command_type == "disarm" {
                    state.pending_action = Some(command_type.to_string());
                    state.pending_sequence = sequence;
                    state.armed_confirmed = false;
                }
                state.last_error = None;
                handle_events(app, state, events);
                emit_status(app, state);
                Ok(())
            }
            Err(error) => {
                if is_serial_io_error(&error) {
                    state.port = None;
                    state.peer_connected = false;
                    state.armed = false;
                    state.armed_confirmed = false;
                    state.session_active = false;
                    state.pending_action = None;
                }
                state.last_error = Some(error.clone());
                emit_status(app, state);
                Err(error)
            }
        }
    })
}

#[tauri::command]
async fn refresh_peer_status(app: AppHandle) -> Result<ControllerStatus, String> {
    tauri::async_runtime::spawn_blocking(move || {
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
            Ok(status_snapshot(state))
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
