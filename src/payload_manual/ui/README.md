# Payload Manual Control UI

This is a Tauri 2 desktop application. Its Rust backend connects to the **controller-side D3K node** over the D3K host UART protocol (USB CDC at 115200, or hardware UART at 460800, 8N1). It does not connect to the payload's D3K UART and does not implement BLE itself.

The controller D3K node must already be configured for BLE HUB/Central mode, BLE must be enabled, and its peer allow-list must include the payload D3K node ID. The payload D3K node must be configured for BLE Peripheral mode and allow the controller D3K node ID. The UI sends `payload-manual-control` v1 messages using `SEND_TO_PEER` and consumes payload state acknowledgements using `GET_RECEIVED_DATA`.

## Simulation

From a graphical Linux desktop session with `DISPLAY` and `xauth` configured, run:

```sh
./scripts/launch-manual-payload.sh
```

The launcher prepares the existing X11 authentication overlay before building and starting the container.

The Compose override starts Gazebo, the manual payload application, and this Tauri UI in the repository's development container. In the simulated world, the controller D3K node is ID `2` at `/tmp/manual-controller-node`; the payload D3K node is ID `18` at `/tmp/payload-anchor`. The UI is prefilled with that PTY path, baud rate 460800, and payload peer ID 18.

## Physical controller node

Connect the controller D3K board to the host and expose its serial device to the container (for example, with a Compose `devices` mapping). Enter that device path in the UI and select 115200 for USB CDC or 460800 for UART1. Configure BLE on the controller node separately, for example with D3KM, before connecting.

## Controls

- `W` / `S`: forward / reverse relative to the payload.
- `A` / `D`: left / right relative to the payload.
- `Q` / `E`: counter-clockwise / clockwise yaw.
- Hold a key or an on-screen control to continue sending the setpoint. Releasing it sends a zero setpoint.
- Arm and disarm state is shown only after a status acknowledgement from the payload. A second UI session can disarm an already-armed payload, but cannot send movement commands until it has explicitly armed its own session.

The UI refreshes drive setpoints every 100 ms. The payload's 300 ms command watchdog stops the wheels and disarms if updates cease. This is open-loop control: it does not use IMU or odometry feedback. A physical emergency stop / motor power cutoff remains necessary for hardware testing.
