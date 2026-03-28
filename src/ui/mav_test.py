import time

from loguru import logger
from pymavlink import mavutil

connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")

logger.info("Waiting for heartbeat...")
connection.wait_heartbeat()
logger.info(f"Connected. target_system={connection.target_system}, " f"target_component={connection.target_component}")


def wait_cmd_ack(conn, command_id, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if msg and msg.command == command_id:
            return msg
    return None


def set_mode_guided(conn):
    mode = "GUIDED"
    if mode not in conn.mode_mapping():
        raise RuntimeError(f"Mode {mode} non supporté par cet autopilote")

    mode_id = conn.mode_mapping()[mode]
    conn.set_mode(mode_id)

    # attendre que le mode change vraiment
    start = time.time()
    while time.time() - start < 10:
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            current_mode = mavutil.mode_string_v10(hb)
            logger.info(f"Mode courant: {current_mode}")
            if current_mode == mode:
                return
    raise RuntimeError("Impossible de passer en GUIDED")


def arm_vehicle(conn, force=False):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 = arm
        21196 if force else 0,  # force si nécessaire
        0,
        0,
        0,
        0,
        0,
    )

    ack = wait_cmd_ack(conn, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    if ack:
        logger.info(f"ARM ACK result={ack.result}")

    conn.motors_armed_wait()
    logger.info("Vehicle armed")


def takeoff(conn, altitude_m):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude_m
    )

    ack = wait_cmd_ack(conn, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    if ack:
        logger.info(f"TAKEOFF ACK result={ack.result}")


def get_relative_altitude_m(conn):
    msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    if msg:
        return msg.relative_alt / 1000.0
    return None


# 1) GUIDED
set_mode_guided(connection)

# 2) ARM
arm_vehicle(connection)

# 3) TAKEOFF à 10 m
takeoff(connection, 10)

# 4) suivi de montée
while True:
    alt = get_relative_altitude_m(connection)
    if alt is not None:
        logger.info(f"Altitude relative: {alt:.2f} m")
        if alt >= 9.5:
            logger.info("Altitude cible atteinte")
            break
