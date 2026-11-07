# RescueSwarm

RescueSwarm is a Bekai Vision research project. A mixed swarm of drones and
ground robots builds a shared map of an unknown disaster zone and marks where
people may be found. All work runs in simulation.

## Status

- **Phase 0, done:** Docker image with ROS 2 Jazzy, Gazebo Harmonic and NVIDIA GPU
  rendering. See [report](docs/PHASE_0_REPORT.md).
- **Phase 1, done:** three ground robots (diff drive, 3D LiDAR, RGB, thermal camera,
  IMU) in separate namespaces. See [report](docs/PHASE_1_REPORT.md).
- **Phase 2, done:** seeded world generator (building, rubble, narrow passages, dead
  ends, people), shared occupancy map from ground-truth poses, frontier exploration
  with a decentralized auction, Nav2 per robot, coverage over time. Three robots
  cover 95–96 % of a random building with no human control. See
  [report](docs/PHASE_2_REPORT.md).
- **Next:** Phase 3, finding people with the thermal camera.

The full research plan is in [docs/PLAN.md](docs/PLAN.md). Plan and reports are
written in Russian.

## Quick start

Everything is built and run in Docker. Nothing is installed on the host.

```bash
make image                    # build rescueswarm-dev:jazzy
make build                    # colcon build inside the container
make explore SEED=1           # Gazebo + RViz, 3 robots explore a generated world
make explore-headless SEED=1  # same without windows
make check-phase2             # scripted Phase 2 check, results in results/phase-2/
```

Run `make help` for all commands.

## Stack

ROS 2 Jazzy, Gazebo Harmonic, Nav2, Cyclone DDS. PX4 SITL for drones comes in
Phase 5.
