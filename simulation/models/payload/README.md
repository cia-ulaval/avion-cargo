# Gazebo payload physics prototype

## Purpose

Standalone physics prototype for the Avion Cargo ground payload.

- Equilateral triangular chassis, nominal side length: 0.18 m
- Three motorized wheels arranged at 120 degrees
- Provisional total mass: 0.15 kg
- Wheel radius: 0.027 m
- Three revolute joints with velocity controllers

The payload is modeled as a dynamic object with mass, inertia,
wheel-ground collisions, and limited motor effort.

## Run the standalone simulation

Make the payload model discoverable through GZ_SIM_RESOURCE_PATH.

Launch:

    gz sim -s -r simulation/worlds/payload_physics_test.sdf

Wheel velocity command topics:

    /model/payload_demo/joint/wheel_0_joint/cmd_vel
    /model/payload_demo/joint/wheel_1_joint/cmd_vel
    /model/payload_demo/joint/wheel_2_joint/cmd_vel

Message type: gz.msgs.Double
Units: rad/s

Joint state topic:

    /world/payload_physics_test/model/payload_demo/joint_state

## Preliminary Gazebo Harmonic / DART results

- Stable when stationary.
- Wheels respond to velocity commands.
- Three equal wheel commands produce chassis rotation.
- Commands [0, 0.2, -0.2] produced a 3.62 cm displacement
  at approximately 30 degrees in one test.
- Commands [-0.2, 0.1, 0.1] produced a 3.10 cm displacement
  at approximately 119.4 degrees in another test.
- Wheel velocities decreased toward zero following stop commands.

These are preliminary observations, not calibrated performance
or repeatability guarantees.

## Known limitations

- Wheels use simplified cylinder collisions, not individual rollers.
- ODE friction parameters mu, mu2 and fdir1 are specified in the SDF;
  their directional behavior under DART has not been verified.
- Mass and dimensions are provisional, pending final CAD.
- Docking, lifting, clamps, and ArUco-based capture are not implemented.
- Standalone dynamic tests do not validate integration with the aircraft.
- The full mission world must still be tested in Gazebo.

## Repository integration

The payload is included as payload_demo in
simulation/worlds/tricopter_full_mission.sdf.

This integration does not intentionally modify the aircraft's
existing model, plugins, or control configuration.
