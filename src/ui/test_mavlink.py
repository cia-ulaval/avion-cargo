import argparse
import math
import time

from domain.models import Pose3D
from infrastructure.communication.drone_mavlink_connector import (
    DroneMavlinkSerial,
    DroneMavlinkUDP,
    MavlinkConnectionParams,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--udp", default="127.0.0.1:14550", help="ip:port (ex: 127.0.0.1:14550)")
    ap.add_argument("--serial", default=None, help="ex: /dev/ttyUSB0 (si set, ignore --udp)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--timeout", type=int, default=10)

    ap.add_argument("--rate", type=float, default=10.0, help="Hz pour envoyer move_to()")
    ap.add_argument("--duration", type=float, default=20.0, help="Durée du test en secondes")
    ap.add_argument("--z", type=float, default=5.0, help="Distance z (m) > 0")
    ap.add_argument("--radius", type=float, default=0.5, help="Rayon (m) pour x/y")
    ap.add_argument("--period", type=float, default=8.0, help="Période (s) d'un tour complet")

    ap.add_argument("--guided", action="store_true", help="Tente de mettre GUIDED au départ (si implémenté)")
    ap.add_argument("--land", action="store_true", help="Tente de mettre LAND à la fin")

    args = ap.parse_args()

    if args.serial:
        params = MavlinkConnectionParams(address=args.serial, baud_rate=args.baud, timeout=args.timeout)
        drone = DroneMavlinkSerial(params)
    else:
        ip, port_s = args.udp.split(":")
        params = MavlinkConnectionParams(address=ip, port=int(port_s), timeout=args.timeout, baud_rate=args.baud)
        drone = DroneMavlinkUDP(params)

    print("[CONNECT] connecting...")
    drone.connect()
    print("[CONNECT] OK")

    if args.guided:
        try:
            drone._conn.set_mode("GUIDED")  # type: ignore[attr-defined]
            print("[MODE] GUIDED sent")
        except Exception as e:
            print(f"[MODE] GUIDED failed: {e}")

    period = 1.0 / max(1.0, args.rate)
    next_t = time.perf_counter()
    t0 = time.time()
    last_print = 0.0

    while (time.time() - t0) < args.duration:
        now = time.perf_counter()
        if now < next_t:
            time.sleep(next_t - now)
        next_t += period

        elapsed = time.time() - t0

        w = 2.0 * math.pi / max(0.1, args.period)
        x = 0.5
        y = args.radius * math.sin(w * elapsed)
        z = args.z

        drone.land_on_target(Pose3D(x=x, y=y, z=z))

        if elapsed - last_print >= 0.5:
            st = drone.get_status()
            print(
                f"[STAT] mode={st.mode} armed={st.armed} alt={st.alt_m:.2f}m "
                f"gs={st.groundspeed_mps:.2f}m/s bat={st.battery_voltage_v:.2f}V "
                f"{st.battery_remaining_pct}% gps={st.gps_fix_type}"
            )
            last_print = elapsed

        # resync si on est trop en retard (évite dérive)
        if time.perf_counter() - next_t > 2 * period:
            next_t = time.perf_counter() + period

    if args.land:
        print("[MODE] LAND...")
        try:
            drone.land()
        except Exception as e:
            print(f"[MODE] LAND failed: {e}")

    print("[DONE]")


if __name__ == "__main__":
    main()
