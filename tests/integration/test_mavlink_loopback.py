"""Exercise UDP bytes against an in-process fake ArduPilot, not a firmware."""

import socket
from threading import Event, Thread

from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from domain.models import Pose3D
from infrastructure.communication.mavlink import DroneMavlinkUDPConnector, MavlinkConnectionParams


def test_udp_connection_mode_confirmation_and_landing_target():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    controller = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    controller.bind(("127.0.0.1", 0))
    controller.settimeout(0.05)
    stop = Event()
    target_received = Event()
    received = []
    errors = []

    class Sender:
        def write(self, packet):
            controller.sendto(packet, ("127.0.0.1", port))

    def autopilot():
        encoder = mavlink2.MAVLink(Sender(), srcSystem=1, srcComponent=1)
        decoder = mavlink2.MAVLink(None)
        mode = 4
        try:
            while not stop.is_set():
                encoder.heartbeat_send(
                    mavlink2.MAV_TYPE_QUADROTOR,
                    mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode,
                    mavlink2.MAV_STATE_ACTIVE,
                )
                try:
                    packet = controller.recv(4096)
                except socket.timeout:
                    continue
                for message in decoder.parse_buffer(packet) or []:
                    if message.get_type() == "COMMAND_LONG" and message.command == mavlink2.MAV_CMD_DO_SET_MODE:
                        mode = int(message.param2)
                        encoder.command_ack_send(message.command, mavlink2.MAV_RESULT_ACCEPTED)
                    elif message.get_type() == "LANDING_TARGET":
                        received.append(message)
                        target_received.set()
        except Exception as error:
            errors.append(error)

    drone = DroneMavlinkUDPConnector(MavlinkConnectionParams("127.0.0.1", port, timeout=2))
    thread = Thread(target=autopilot, daemon=True)
    thread.start()
    try:
        drone.connect()
        assert drone.get_status().connected
        drone.activate_land_mode()
        drone.land_on_target(Pose3D(0.5, -0.25, 2), (0.4, 0.4))
        assert target_received.wait(2)
        target = received[-1]
        assert (target.x, target.y, target.z) == (0.5, -0.25, 2)
        assert target.position_valid == 1
        assert target.frame == mavlink2.MAV_FRAME_BODY_FRD
    finally:
        drone.close()
        stop.set()
        thread.join(2)
        controller.close()
    assert not thread.is_alive()
    assert not errors
