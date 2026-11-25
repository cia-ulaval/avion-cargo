import math
import os
import shutil
import socket
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from vtol.domain.rigid_transform import RigidTransform
from vtol.infrastructure.communication.mavlink import DroneMavlinkUDPConnector, MavlinkConnectionParams


@pytest.mark.skipif(not os.environ.get("ARDUPILOT_SITL_ROOT"), reason="Opt-in native ArduPlane SITL test")
@pytest.mark.parametrize("scenario", ["failsafes", "descent_profile"])
def test_qland_yaw_control(tmp_path, scenario):
    ap = Path(os.environ["ARDUPILOT_SITL_ROOT"])
    root = Path(__file__).resolve().parents[3]
    instance, port = 41, 6170
    with socket.socket() as probe:
        assert probe.connect_ex(("127.0.0.1", port)) != 0, "SITL test port already in use"
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    shutil.copy(ap / "libraries/AP_Scripting/applets/qland_yaw_alignment.lua", scripts)
    shutil.copy(ap / "libraries/AP_Scripting/applets/plane_precland.lua", scripts)
    params = tmp_path / "test.parm"
    flight_parameters = {"Q_LAND_FINAL_ALT": 6, "Q_LAND_FINAL_SPD": 0.5}
    if scenario == "descent_profile":
        names = {"Q_WP_SPD_DN", "Q_LAND_FINAL_ALT", "Q_LAND_FINAL_SPD", "PLND_XY_DIST_MAX"}
        for line in (root / "simulation/config/gazebo_vtol.parm").read_text().splitlines():
            fields = line.split()
            if fields and fields[0] in names:
                flight_parameters[fields[0]] = float(fields[1])
        assert names <= flight_parameters.keys()
    params.write_text(
        "SCR_ENABLE 1\nSCR_HEAP_SIZE 200000\nQ_GUIDED_MODE 1\nQ_WVANE_ENABLE 0\n"
        "ARMING_CHECK 0\nBRD_SAFETY_DEFLT 0\nRC_OPTIONS 0\nMAV_GCS_SYSID 255\n"
        "SIM_SPEEDUP 1\nPLND_ENABLED 1\nPLND_TYPE 1\n"
        + "".join(f"{name} {value}\n" for name, value in flight_parameters.items())
    )
    defaults = ",".join(
        str(p)
        for p in [
            ap / "Tools/autotest/default_params/quadplane.parm",
            ap / "Tools/autotest/default_params/quadplane-tilttrivec.parm",
            params,
        ]
    )
    log = (tmp_path / "sitl.log").open("w+")
    process = subprocess.Popen(
        [
            str(ap / "build/sitl/bin/arduplane"),
            f"-I{instance}",
            "--model",
            "quadplane-tilttrivec",
            "--speedup",
            "1",
            "--defaults",
            defaults,
        ],
        cwd=tmp_path,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    connection = None
    latest, texts, parameters, heartbeat_at = {}, [], {}, 0.0
    publisher = DroneMavlinkUDPConnector(MavlinkConnectionParams("127.0.0.1", 14550))

    def pump():
        nonlocal heartbeat_at
        now = time.monotonic()
        if now - heartbeat_at >= 0.5:
            connection.mav.heartbeat_send(mavlink2.MAV_TYPE_GCS, mavlink2.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)
            heartbeat_at = now
        while True:
            message = connection.recv_match(blocking=False)
            if message is None:
                break
            if message.get_srcComponent() != 1:
                continue
            latest[message.get_type()] = message
            if message.get_type() == "PARAM_VALUE":
                parameters[message.param_id] = message.param_value
            if message.get_type() == "STATUSTEXT":
                texts.append(message.text)
                if "QYAW" in message.text or "error" in message.text.lower():
                    print(message.text, flush=True)

    def until(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, "SITL exited"
            pump()
            if predicate():
                return
            time.sleep(0.02)
        raise AssertionError(
            f"Timeout: {description}; QYAW_ENABLE={parameters.get('QYAW_ENABLE')}; messages: {texts[-15:]}"
        )

    def command(command_id, *values):
        connection.mav.command_long_send(1, 1, command_id, 0, *(list(values) + [0] * (7 - len(values))))

    def set_mode(mode):
        connection.mav.set_mode_send(1, mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
        until(lambda: latest["HEARTBEAT"].custom_mode == mode, 5, f"mode {mode}")

    def altitude():
        return latest["GLOBAL_POSITION_INT"].relative_alt / 1000

    def rotation():
        attitude = latest["ATTITUDE"]
        cr, sr = math.cos(attitude.roll), math.sin(attitude.roll)
        cp, sp = math.cos(attitude.pitch), math.sin(attitude.pitch)
        cy, sy = math.cos(attitude.yaw), math.sin(attitude.yaw)
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ]
        )

    def send_target(heading):
        matrix = np.eye(4)
        matrix[:3, :3] = rotation().T @ RigidTransform.from_vectors([0, 0, heading], [0, 0, 0]).as_matrix()[:3, :3]
        matrix[:3, 3] = rotation().T @ [0, 0, altitude()]
        transform = RigidTransform(matrix)
        publisher.send_landing_target(
            transform.position, (2, 2), body_from_marker=transform, captured_at_s=time.monotonic()
        )

    try:
        deadline = time.monotonic() + 10
        while connection is None and time.monotonic() < deadline:
            try:
                connection = mavutil.mavlink_connection(
                    f"tcp:127.0.0.1:{port}", source_system=255, source_component=191, retries=1
                )
            except OSError:
                time.sleep(0.2)
        assert connection is not None
        publisher.connection = connection
        publisher._configure_mavlink2()
        until(lambda: "HEARTBEAT" in latest, 10, "heartbeat")
        for message_id in (30, 33, 108):
            command(mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, 50000)
        until(lambda: any("QYAW: loaded" in text for text in texts), 15, "Lua controller startup")
        connection.mav.param_request_read_send(1, 1, b"QYAW_ENABLE", -1)
        until(lambda: "QYAW_ENABLE" in parameters, 5, "Lua parameters available")
        assert parameters["QYAW_ENABLE"] == 0
        connection.mav.param_set_send(1, 1, b"QYAW_ENABLE", 1, mavlink2.MAV_PARAM_TYPE_REAL32)
        until(
            lambda: parameters.get("QYAW_ENABLE") == 1,
            5,
            "enable Lua controller",
        )
        until(
            lambda: "GLOBAL_POSITION_INT" in latest
            and "ATTITUDE" in latest
            and any("EKF3 IMU0 is using GPS" in text for text in texts),
            30,
            "GPS/EKF ready",
        )
        until(lambda: "SIM_STATE" in latest, 5, "SITL ground truth")
        ground_alt = latest["SIM_STATE"].alt
        set_mode(15)
        command(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196)
        until(lambda: latest["HEARTBEAT"].base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED, 5, "arming")
        command(mavlink2.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 20.5 if scenario == "descent_profile" else 20)
        until(lambda: altitude() > 19.5 and abs(latest["GLOBAL_POSITION_INT"].vz) < 30, 45, "VTOL takeoff")
        if scenario == "descent_profile":
            until(
                lambda: latest["SIM_STATE"].alt - ground_alt >= 20 and abs(latest["SIM_STATE"].vd) < 0.15,
                10,
                "settled above 20 m",
            )
            initial_height = latest["SIM_STATE"].alt - ground_alt
            initial_yaw = latest["ATTITUDE"].yaw
            start_ms = latest["GLOBAL_POSITION_INT"].time_boot_ms
            set_mode(20)
            slow_entry = None
            final_correction_started = False
            final_aligned = False
            cutoff_checked = False
            deadline = time.monotonic() + 50
            trace = []
            while time.monotonic() < deadline:
                pump()
                height = latest["SIM_STATE"].alt - ground_alt
                elapsed = (latest["GLOBAL_POSITION_INT"].time_boot_ms - start_ms) / 1000
                assert latest["HEARTBEAT"].custom_mode == 20
                if slow_entry is None and height <= flight_parameters["Q_LAND_FINAL_ALT"]:
                    slow_entry = elapsed
                if height > 0.5:
                    if altitude() < 3.5:
                        # Demand another turn below the script's minimum height; it must be ignored.
                        desired = latest["ATTITUDE"].yaw - math.radians(60)
                        if altitude() < 3:
                            assert abs(latest["ATTITUDE"].yawspeed) < math.radians(3)
                            cutoff_checked = True
                    elif height < 7:
                        # Introduce the heading error after entering the final landing phase.
                        desired = initial_yaw + math.radians(35)
                        final_correction_started = True
                        error = math.degrees(
                            math.atan2(
                                math.sin(desired - latest["ATTITUDE"].yaw), math.cos(desired - latest["ATTITUDE"].yaw)
                            )
                        )
                        final_aligned = final_aligned or abs(error) < 4
                    else:
                        desired = initial_yaw
                    send_target(desired)
                trace.append({"elapsed_s": elapsed, "height_m": height, "down_speed_mps": latest["SIM_STATE"].vd})
                if height <= 0.02 and abs(latest["SIM_STATE"].vd) < 0.02:
                    break
                time.sleep(0.05)
            import json

            (tmp_path / "descent-profile.json").write_text(json.dumps(trace))
            assert height <= 0.02, "No ground contact"
            assert elapsed <= 45, f"Nominal descent exceeded budget: {elapsed:.2f}s"
            assert slow_entry is not None and elapsed - slow_entry >= 24
            assert final_correction_started and final_aligned, "No yaw alignment during slow final descent"
            assert cutoff_checked
            print(
                f"Nominal profile: {initial_height:.2f} m -> ground in {elapsed:.2f}s; "
                f"{elapsed-slow_entry:.2f}s below 8 m; final-phase yaw aligned and released below 4 m",
                flush=True,
            )
            return
        initial_yaw, initial_alt = latest["ATTITUDE"].yaw, altitude()
        desired = initial_yaw + math.radians(35)
        set_mode(20)
        observations = []
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            pump()
            assert latest["HEARTBEAT"].custom_mode == 20
            send_target(desired)
            error = math.degrees(
                math.atan2(math.sin(desired - latest["ATTITUDE"].yaw), math.cos(desired - latest["ATTITUDE"].yaw))
            )
            observations.append((altitude(), error))
            if len(observations) > 60 and abs(error) < 4:
                break
            time.sleep(0.05)
        print(f"QLAND alignment: altitude {initial_alt:.2f} -> {altitude():.2f} m; error {error:.2f} deg", flush=True)
        assert abs(error) < 4
        assert altitude() < initial_alt - 1
        assert any("aligning during descent" in text for text in texts)

        desired = latest["ATTITUDE"].yaw - math.radians(60)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            pump()
            send_target(desired)
            time.sleep(0.05)
        assert latest["ATTITUDE"].yawspeed < -math.radians(5)
        until(lambda: texts[-1:] == ["QYAW: waiting for target"], 2, "release after target loss")
        until(lambda: abs(latest["ATTITUDE"].yawspeed) < math.radians(2), 3, "yaw rate settled")
        assert latest["HEARTBEAT"].custom_mode == 20
        print("Target loss: yaw stopped; QLAND descent retained", flush=True)

        def start_turn():
            heading = latest["ATTITUDE"].yaw + math.radians(60)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                pump()
                send_target(heading)
                time.sleep(0.05)
            assert latest["ATTITUDE"].yawspeed > math.radians(5)

        start_turn()
        command(mavlink2.MAV_CMD_SCRIPTING, mavlink2.SCRIPTING_CMD_STOP)
        until(lambda: any("Scripting: stopped" in text for text in texts), 3, "stop Lua VM")
        until(lambda: abs(latest["ATTITUDE"].yawspeed) < math.radians(2), 3, "native yaw timeout")
        assert latest["HEARTBEAT"].custom_mode == 20
        print("Lua VM stopped: native yaw timeout released control", flush=True)

        loaded_count = sum("QYAW: loaded" in text for text in texts)
        command(mavlink2.MAV_CMD_SCRIPTING, mavlink2.SCRIPTING_CMD_STOP_AND_RESTART)
        until(lambda: sum("QYAW: loaded" in text for text in texts) > loaded_count, 5, "restart Lua VM")
        start_turn()
        connection.mav.param_set_send(1, 1, b"SIM_RC_FAIL", 1, mavlink2.MAV_PARAM_TYPE_REAL32)
        deadline = time.monotonic() + 3
        heading = latest["ATTITUDE"].yaw + math.radians(90)
        text_start = len(texts)
        while time.monotonic() < deadline:
            pump()
            send_target(heading)
            time.sleep(0.05)
        failures = texts[text_start:]
        assert any("failsafe" in text.lower() for text in failures), failures
        assert any("QYAW: vehicle unavailable" in text or "QYAW: inactive" in text for text in failures), failures
        print("RC loss with fresh targets: native failsafe remained effective", flush=True)
    finally:
        if connection:
            connection.close()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
