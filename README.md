# Simple simulation launcher

This replaces the earlier launcher with a deliberately small workflow. It does **not** start or stop the X11 relay, start the Compose service, rebuild the image, or shut down the other components just because one command exits.

Prerequisites:
- Run `docker compose up -d dev` from the repository root if the `dev` container isn't already running.
- Start your existing `listen-x11.sh` in its own terminal if you're using that TCP relay.

Install/copy these files into the repository:
- `run-simulation.sh` at the repository root.
- `docker/launch-simulation.sh` under `docker/`.

Make the host launcher executable:

```bash
chmod +x run-simulation.sh
```

Run all four commands with one command:

```bash
./run-simulation.sh
```

It starts `avion-sitl`, `avion-gazebo`, `avion-image-bridge`, and `avion-autolander` concurrently. Output is tagged by component. Each component also writes to `logs/simulation/<component>.log` in the repository. If one component exits, the others keep running and the exit status is logged. Press Ctrl+C to stop the component process groups. The dev container and X11 relay remain running.
