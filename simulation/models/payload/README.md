# Gazebo payload physics prototype

## Purpose

Standalone physics prototype for the Avion Cargo ground payload.

- Equilateral triangular chassis, nominal side length: 0.18 m
- Three motorized wheels arranged at 120 degrees
- Provisional total mass: 0.40 kg (pending hardware confirmation)
- Wheel radius: 0.027 m
- Three revolute joints, each driven by a JointController in force mode
  (PID, effort limited to +/-0.10 through cmd_max / cmd_min)

All parameters are provisional until hardware data is available.

## Prerequisite: Git LFS

The aircraft meshes (.obj, .mtl, .dae, .png) are stored with Git LFS.
Without `git lfs install && git lfs pull`, the .obj files are small pointer
files and Gazebo aborts with
"ODE INTERNAL ERROR 1: assertion vertices failed in buildData()".
This was observed on one machine before running `git lfs pull`, with or
without the payload include. It does not affect payload_standalone.sdf.

## Run with Docker Compose

From the repository root:

    docker compose -f compose.yaml -f compose-payload.yaml up

compose-payload.yaml sets AVION_WORLD to
simulation/worlds/payload_standalone.sdf and runs /usr/local/bin/avion-gazebo,
which sets GZ_SIM_RESOURCE_PATH so the payload model is found.
The first run builds the dev image (it compiles ArduPilot) and takes a long
time. The GUI needs a working X11 display (DISPLAY set).

Stop and remove the containers with:

    docker compose -f compose.yaml -f compose-payload.yaml down

## Headless use (no GUI)

The launcher accepts no extra arguments. To run the server only, inside the
dev container:

    sed "s/gz sim -v4 -r/gz sim -s -v4 -r/" /usr/local/bin/avion-gazebo > /tmp/gz-headless.sh
    bash /tmp/gz-headless.sh &
    export GZ_CONFIG_PATH=/usr/share/gz

Wheel command topics (type gz.msgs.Double), for wheel_0_joint, wheel_1_joint
and wheel_2_joint:

    /model/payload_demo/joint/wheel_0_joint/cmd_vel
    /model/payload_demo/joint/wheel_1_joint/cmd_vel
    /model/payload_demo/joint/wheel_2_joint/cmd_vel

Example:

    gz topic -t /model/payload_demo/joint/wheel_0_joint/cmd_vel -m gz.msgs.Double -p "data: 2.0"

Read the pose and the joint state:

    gz topic -e -n 1 -t /world/payload_standalone/dynamic_pose/info
    gz topic -e -n 1 -t /world/payload_standalone/model/payload_demo/joint_state

## Changing the parameters

Mass and geometry are arguments of the generator, which rewrites model.sdf:

    python3 scripts/generate_payload_physics.py --mass 0.4 --wheel-radius 0.027

Other arguments: --side, --wheel-width, --wheel-offset.
The PID gains, the effort limit (cmd_max / cmd_min) and the friction values
(mu, mu2, fdir1) are written directly in the script and are not arguments.

## Verification status

Run on macOS (Apple M2), Docker Desktop, ARM64 image, server only:

- Image build completed.
- payload_standalone.sdf: the world and payload_demo load; the three
  JointControllers and the JointStatePublisher load. No [Err] or [Wrn] line
  in the server log of the tests below.
- At rest for 30 s of simulated time: pose unchanged.
- Command sweep applied to the three wheels (two runs, 6 s wait after each
  command), wheel velocity read in joint_state:

      command   run 1    run 2
      0.5       0.356    0.356
      1         0.880    0.880
      2         1.879    1.878
      4         3.844    3.844
      8         7.749    7.749

  The two runs agree within about 0.1 %. No saturation up to a command of 8.
  The values are consistent with rad/s but units were not verified.
- After a zero command the wheel velocity decreases; it was about 0.1 after
  a command of 8 and 6 s of wall-clock wait (the simulated time was not read).
- tricopter_full_mission.sdf, server only, no ArduPilot SITL: loads with the
  payload included (payload_demo at x=0.8, y=0.8); the pose was identical over
  three readings about 20 s apart. A command of 2 on the three wheels made the
  model rotate, with a displacement under 1 cm. Only camera/rendering [Wrn]
  lines were seen there (no display).
- GUI: with no DISPLAY, Qt exits with "could not connect to display". With
  software rendering through Qt VNC on macOS Docker, the GUI crashed with a
  segmentation fault in the OpenGL scene rendering.

Not verified: GUI on Linux/X11, command units, behavior under load or on a
slope, ArduPilot SITL, comparison with the real robot. These are preliminary
observations, not calibrated results.

## Known limitations

- Wheels use simplified cylinder collisions, not individual rollers.
- ODE friction parameters mu, mu2 and fdir1 are specified in the SDF;
  their directional behavior under DART has not been verified.
- Mass and dimensions are provisional, pending final CAD.
- Motor effort limit and PID gains are provisional, not derived from a motor.
- Docking, lifting, clamps, and ArUco-based capture are not implemented.
- Standalone dynamic tests do not validate integration with the aircraft.

## Repository integration

The payload is included as payload_demo in
simulation/worlds/tricopter_full_mission.sdf, at x=0.8 m, y=0.8 m
(outside the 1.0185 m landing marker, inside the 2.4384 m DLZ, according to
the comment in that file).

This integration does not intentionally modify the aircraft's
existing model, plugins, or control configuration.
