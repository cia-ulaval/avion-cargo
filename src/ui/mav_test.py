#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

from pymavlink import mavutil


def connect_mavlink(
    connection_string: str,
    source_system: int = 255,
    source_component: int = 190,
    heartbeat_timeout: float = 10.0,
) -> mavutil.mavfile:
    """
    Ouvre une connexion MAVLink et attend un HEARTBEAT.
    """
    print(f"[INFO] Connexion MAVLink sur: {connection_string}")

    master = mavutil.mavlink_connection(
        connection_string,
        source_system=source_system,
        source_component=source_component,
    )

    print("[INFO] Attente du HEARTBEAT...")
    hb = master.wait_heartbeat(timeout=heartbeat_timeout)
    if hb is None:
        raise TimeoutError(
            f"Aucun HEARTBEAT reçu après {heartbeat_timeout:.1f}s sur {connection_string}"
        )

    print(
        "[OK] HEARTBEAT reçu "
        f"(target_system={master.target_system}, "
        f"target_component={master.target_component})"
    )
    return master


def request_message_interval(
    master: mavutil.mavfile,
    message_name: str,
    frequency_hz: float,
) -> None:
    """
    Demande au véhicule d'envoyer un message à une fréquence donnée.
    Ex: GLOBAL_POSITION_INT à 5 Hz.
    """
    if frequency_hz <= 0:
        raise ValueError("frequency_hz doit être > 0")

    message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}", None)
    if message_id is None:
        raise ValueError(f"Message MAVLink inconnu: {message_name}")

    interval_us = int(1_000_000 / frequency_hz)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,                  # confirmation
        message_id,         # param1: ID du message
        interval_us,        # param2: interval en microsecondes
        0, 0, 0, 0, 0
    )

    print(f"[INFO] Demande de {message_name} à {frequency_hz:.1f} Hz")


def send_heartbeat(master: mavutil.mavfile) -> None:
    """
    Envoie un HEARTBEAT depuis le companion computer / GCS-like endpoint.
    """
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )
    print("[INFO] HEARTBEAT envoyé")


def read_messages(master: mavutil.mavfile, duration_s: float = 15.0) -> None:
    """
    Lit quelques messages pendant une durée donnée.
    """
    deadline = time.time() + duration_s
    print(f"[INFO] Lecture des messages pendant {duration_s:.1f}s...\n")

    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            continue

        print(f"[MSG] {msg_type}: {msg}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Test de connexion UDP pymavlink/mavutil"
    )
    parser.add_argument(
        "--connect",
        required=True,
        help=(
            "Chaîne de connexion mavutil. "
            "Exemples: udpout:127.0.0.1:14560 ou udpin:0.0.0.0:14550"
        ),
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=10.0,
        help="Timeout d'attente du heartbeat en secondes",
    )
    parser.add_argument(
        "--request-global-position",
        type=float,
        default=0.0,
        help="Si > 0, demande GLOBAL_POSITION_INT à cette fréquence (Hz)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="Durée de lecture des messages en secondes",
    )
    args = parser.parse_args()

    try:
        master = connect_mavlink(
            connection_string=args.connect,
            heartbeat_timeout=args.heartbeat_timeout,
        )

        send_heartbeat(master)

        if args.request_global_position > 0:
            request_message_interval(
                master,
                message_name="GLOBAL_POSITION_INT",
                frequency_hz=args.request_global_position,
            )

        read_messages(master, duration_s=args.duration)
        return 0

    except Exception as exc:
        print(f"[ERREUR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())