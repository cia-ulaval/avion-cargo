use serialport::SerialPort;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

pub const OP_PING: u8 = 0x01;
pub const OP_GET_INFO: u8 = 0x02;
pub const OP_SUBSCRIBE: u8 = 0x09;
pub const OP_GET_PEER_STATUS: u8 = 0x54;
pub const OP_SEND_TO_PEER: u8 = 0x55;
pub const OP_GET_RECEIVED_DATA: u8 = 0x56;
pub const BLE_EVENT_MASK: u16 = 0x7FC0; // BLE event opcodes 0x86..0x8E.

#[derive(Debug, Clone)]
pub struct Event {
    pub opcode: u8,
    pub payload: Vec<u8>,
}

#[derive(Debug)]
pub struct Reply {
    pub payload: Vec<u8>,
    pub events: Vec<Event>,
}

#[derive(Debug)]
pub struct SendMessageFailure {
    pub error: String,
    pub events: Vec<Event>,
}

fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for byte in data {
        crc ^= (*byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn cobs_encode(input: &[u8]) -> Vec<u8> {
    let mut out = vec![0];
    let mut code_index = 0usize;
    let mut code = 1u8;
    for byte in input {
        if *byte == 0 {
            out[code_index] = code;
            code_index = out.len();
            out.push(0);
            code = 1;
        } else {
            out.push(*byte);
            code = code.wrapping_add(1);
            if code == 0xff {
                out[code_index] = code;
                code_index = out.len();
                out.push(0);
                code = 1;
            }
        }
    }
    out[code_index] = code;
    out.push(0);
    out
}

fn cobs_decode(input: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < input.len() {
        let code = *input.get(i)? as usize;
        if code == 0 {
            return None;
        }
        i += 1;
        let end = i.checked_add(code - 1)?;
        if end > input.len() {
            return None;
        }
        out.extend_from_slice(&input[i..end]);
        i = end;
        if code != 0xff && i < input.len() {
            out.push(0);
        }
    }
    Some(out)
}

fn frame(seq: u16, opcode: u8, payload: &[u8]) -> Vec<u8> {
    let mut raw = vec![1, 1]; // protocol version 1, request frame
    raw.extend(seq.to_le_bytes());
    raw.extend([opcode, 0]);
    raw.extend((payload.len() as u16).to_le_bytes());
    raw.extend(payload);
    let crc = crc16(&raw);
    raw.extend(crc.to_le_bytes());
    cobs_encode(&raw)
}

fn parse(raw: &[u8]) -> Result<(u8, u16, u8, u8, Vec<u8>), String> {
    let data = cobs_decode(raw).ok_or_else(|| "invalid COBS frame".to_string())?;
    if data.len() < 10 || data[0] != 1 {
        return Err("invalid D3K frame header".into());
    }
    let size = u16::from_le_bytes([data[6], data[7]]) as usize;
    if data.len() != 10 + size {
        return Err("invalid D3K frame length".into());
    }
    let expected = u16::from_le_bytes([data[data.len() - 2], data[data.len() - 1]]);
    if crc16(&data[..data.len() - 2]) != expected {
        return Err("D3K frame CRC mismatch".into());
    }
    Ok((
        data[1],
        u16::from_le_bytes([data[2], data[3]]),
        data[4],
        data[5],
        data[8..8 + size].to_vec(),
    ))
}

fn command_with_status(
    port: &mut Box<dyn SerialPort>,
    seq: u16,
    opcode: u8,
    payload: &[u8],
) -> Result<(Reply, Option<String>), SendMessageFailure> {
    let packet = frame(seq, opcode, payload);
    port.write_all(&packet).map_err(|e| SendMessageFailure {
        error: format!("D3K UART write failed: {e}"),
        events: Vec::new(),
    })?;
    port.flush().map_err(|e| SendMessageFailure {
        error: format!("D3K UART flush failed: {e}"),
        events: Vec::new(),
    })?;

    let start = Instant::now();
    let timeout = Duration::from_millis(700);
    let mut encoded = Vec::new();
    let mut events = Vec::new();
    let mut response: Option<(Vec<u8>, Option<String>)> = None;
    let mut buf = [0u8; 512];

    while start.elapsed() < timeout {
        match port.read(&mut buf) {
            Ok(n) => {
                for byte in &buf[..n] {
                    if *byte == 0 {
                        if !encoded.is_empty() {
                            if let Ok((kind, rseq, rop, flags, frame_payload)) = parse(&encoded) {
                                if kind == 3 {
                                    events.push(Event {
                                        opcode: rop,
                                        payload: frame_payload,
                                    });
                                } else if kind == 2 && rseq == seq && rop == opcode {
                                    let error = if flags != 0 {
                                        Some(format!(
                                            "D3K command 0x{opcode:02X} rejected with status {flags}"
                                        ))
                                    } else {
                                        None
                                    };
                                    response = Some((frame_payload, error));
                                }
                            }
                        }
                        encoded.clear();
                    } else if encoded.len() < 2048 {
                        encoded.push(*byte);
                    } else {
                        encoded.clear();
                    }
                }
                if let Some((payload, error)) = response.take() {
                    return Ok((Reply { payload, events }, error));
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                return Err(SendMessageFailure {
                    error: format!("D3K UART read failed: {e}"),
                    events,
                })
            }
        }
    }
    Err(SendMessageFailure {
        error: format!("Timed out waiting for D3K opcode 0x{opcode:02X}"),
        events,
    })
}

pub fn command(
    port: &mut Box<dyn SerialPort>,
    seq: u16,
    opcode: u8,
    payload: &[u8],
) -> Result<Reply, String> {
    let (reply, error) = command_with_status(port, seq, opcode, payload)
        .map_err(|failure| failure.error)?;
    if let Some(error) = error {
        return Err(error);
    }
    Ok(reply)
}

pub fn send_message_with_events(
    port: &mut Box<dyn SerialPort>,
    seq: u16,
    peer_id: u16,
    message: &[u8],
) -> Result<(u16, Vec<Event>), SendMessageFailure> {
    if message.len() > 496 {
        return Err(SendMessageFailure {
            error: "D3K application message exceeds the 496-byte limit".into(),
            events: Vec::new(),
        });
    }
    let mut payload = Vec::with_capacity(4 + message.len());
    payload.extend(peer_id.to_le_bytes());
    payload.extend((message.len() as u16).to_le_bytes());
    payload.extend(message);
    let (reply, command_error) = command_with_status(port, seq, OP_SEND_TO_PEER, &payload)?;
    if let Some(error) = command_error {
        return Err(SendMessageFailure {
            error,
            events: reply.events,
        });
    }
    if reply.payload.len() != 2 {
        return Err(SendMessageFailure {
            error: format!(
                "SEND_TO_PEER returned {} bytes; expected 2",
                reply.payload.len()
            ),
            events: reply.events,
        });
    }
    Ok((
        u16::from_le_bytes([reply.payload[0], reply.payload[1]]),
        reply.events,
    ))
}

pub fn peer_connected(
    port: &mut Box<dyn SerialPort>,
    seq: u16,
    peer_id: u16,
) -> Result<(bool, Vec<Event>), String> {
    let reply = command(port, seq, OP_GET_PEER_STATUS, &peer_id.to_le_bytes())?;
    if reply.payload.len() != 14 {
        return Err(format!(
            "GET_PEER_STATUS returned {} bytes; expected 14",
            reply.payload.len()
        ));
    }
    Ok((reply.payload[5] == 1, reply.events))
}

pub fn get_received_data(
    port: &mut Box<dyn SerialPort>,
    seq: u16,
    peer_id: u16,
    message_id: u16,
) -> Result<(Vec<u8>, Vec<Event>), String> {
    let mut payload = Vec::with_capacity(4);
    payload.extend(peer_id.to_le_bytes());
    payload.extend(message_id.to_le_bytes());
    let reply = command(port, seq, OP_GET_RECEIVED_DATA, &payload)?;
    if reply.payload.len() < 4 {
        return Err("GET_RECEIVED_DATA returned a truncated response".into());
    }
    let returned_id = u16::from_le_bytes([reply.payload[0], reply.payload[1]]);
    let length = u16::from_le_bytes([reply.payload[2], reply.payload[3]]) as usize;
    if returned_id != message_id || reply.payload.len() != length + 4 {
        return Err("GET_RECEIVED_DATA returned an inconsistent message".into());
    }
    Ok((reply.payload[4..].to_vec(), reply.events))
}
