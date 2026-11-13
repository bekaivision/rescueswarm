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
- **Phase 3 and 3b, done:** thermal people search with decoys (fire, batteries, warm
  debris), candidate inspection through the auction, and YOLO on RGB as an extra cue
  with the Rescue Randy model. See [report](docs/PHASE_3_REPORT.md).
- **Phase 4, done (with caveats):** Phases 2-3 run on poses from `mrg_slam` fused with
  wheel odometry and gyro instead of ground truth (`LOC=slam`). On 4 seeds all 19
  reachable people are confirmed, no decoys, ATE rmse 0.26 m, but map precision is 0.70
  against 0.97 with ground truth. Loop closures are off and start poses are known. See
  [report](docs/PHASE_4_REPORT.md).
- **Phase 5, done:** two drones join the three ground robots and build one shared map
  (ground and air layers). Drones run either a light flight model or PX4 SITL v1.17 in
  offboard mode with the PX4 EKF pose. Drones spot warm targets from above; ground
  robots confirm people up close. On 4 seeds the mixed swarm confirms all 19 reachable
  people with both drone modes, no decoys, and reaches 90 % coverage in 60-70 s instead
  of 310 s. See [report](docs/PHASE_5_REPORT.md).
- **Phase 6, done:** robots no longer see each other's topics. Maps, people
  observations, auction bids and poses travel as packets through a `comms_sim` radio
  model (range, walls, bandwidth, loss, delay, outages). Each robot keeps its own map
  and people list, and a base station at the entrance shows what reached the rescuer.
  A robot cut off for 140 s keeps exploring, and the base catches up 2 s after the link
  returns. On 4 seeds and 4 link levels all 19 reachable people are confirmed at the
  base with no decoys. A poor link barely changes base coverage, because drones relay
  over the walls, but it stretches the search from 158 s to 618 s. See
  [report](docs/PHASE_6_REPORT.md).
- **Next:** Phase 7, experiment series and swarm size.

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
make search-headless SEED=1 LOC=slam   # people search on SLAM poses instead of ground truth
make check-phase4             # Phase 4 check, ground truth vs SLAM, results in results/phase-4/
make search DRONES=2          # 3 ground robots + 2 light drones, Gazebo + RViz
make search-headless DRONES=2 DBACK=px4   # same with PX4 SITL drones, no windows
make check-phase5             # Phase 5 check, ground only vs mixed swarm, results in results/phase-5/
make search-headless DRONES=2 COMMS=medium   # exchange only through the comms_sim radio model
make check-phase6             # Phase 6 check, 4 link levels + link outage test, results in results/phase-6/
```

Run `make help` for all commands.

## Stack

ROS 2 Jazzy, Gazebo Harmonic, Nav2, Cyclone DDS, PX4 SITL v1.17 with
Micro XRCE-DDS for drones, own radio model (`rs_comms`) for swarm communication.
