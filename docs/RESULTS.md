# Results

All results come from simulation. Each phase has a scripted check (`make check-phaseN`) that runs
the same four worlds and writes a `summary.md` to `results/phase-N/check/`. The `results/`
directory is not in git; the numbers below are copied from those summaries and from the phase
reports in [docs/](.) (written in Russian).

## Setup

- **Worlds.** A seeded generator (`rs_worlds`) builds a collapsed building: a long corridor, rooms
  on both sides, dead ends, 15-40 pieces of rubble and three narrow passages of 0.8 m. People are
  the Rescue Randy model; decoys (fire, hot batteries, warm debris) are placed on purpose. The
  same seed gives the same world.
- **Seeds.** 1, 2, 3 and 7 in every check. Reachable people per seed: 4, 5, 4, 6 (19 in total).
  Some people sit in unreachable places (behind rubble) and can only be marked from the air.
- **Ground robots.** Three differential-drive robots, each in its own namespace, with a 3D LiDAR
  (16 beams x 360 points, 10 Hz), RGB 320x240 at 5 Hz, a thermal camera 160x120 at 5 Hz and an IMU
  at 50 Hz. Navigation is Nav2 per robot.
- **Drones.** Two drones with a nadir depth sensor (160x120, 5 Hz) and a thermal camera; no LiDAR.
  Two flight back ends: `light` (own flight model in Gazebo) and `px4` (PX4 SITL v1.17, offboard
  mode, pose from the PX4 EKF). A drone cannot confirm a person; ground robots do that up close.
- **Limits.** Each run stops at "exploration complete" or at 900 simulated seconds. Real-time
  factor is about 1.0 for ground-only runs and 0.4-0.9 for full-stack runs.
- **Machine.** One laptop: RTX 3060 Laptop GPU (6 GB), everything in Docker.

## Headline table

Mean over seeds 1, 2, 3, 7 unless a range is given. t90 is the simulated time to reach 90 % map
coverage.

| Question | Setup | Result |
| --- | --- | --- |
| Mixed vs ground-only (Phase 5) | 3 ground vs 3 ground + 2 light drones | t90 310 s vs 60 s; last person confirmed at 328 s vs 96 s; unreachable people marked 2/4 vs 4/4 |
| Light vs PX4 drones (Phase 5) | mixed swarm, ground truth poses for ground robots | t90 60 s (light) vs 70 s (PX4); last person 96 s vs 118 s; map P@2 0.98 vs 0.95 |
| People search (Phases 3, 4, 5, 6) | all four seeds | 19/19 reachable people confirmed, 0 decoys confirmed (ground truth poses, SLAM poses, mixed swarm, every comms level); Phase 4 ground-truth run: 18/19 |
| SLAM instead of ground truth (Phase 4) | 3 ground robots | ATE rmse 0.26 m; map P@2 0.70 vs 0.97 with ground truth |
| Comms level (Phase 6) | ideal / good / medium / poor | search ends at 158 / 170 / 229 / 618 s; base map coverage stays 99.9-100 % |
| Outage (Phase 6) | `ground_1` cut off 60-200 s | robot keeps mapping; base catches up 2.0 s after the link returns |
| Full stack (Phase 7a) | SLAM + 2 drones + `medium` link | light 19/19 people, PX4 18/19; 0 decoys; N=5 finds 4/4 |

Layer alignment between the drone map and the ground map: 0.05-0.10 m shift (Phase 5).

## Phase 3: people search (ground robots, ground truth poses)

Thermal detector with LiDAR check, marks fused across robots, candidates inspected through the
auction, YOLO11 on RGB as an extra cue (7.7 ms per frame for yolo11n at 640 px, single GPU stream).

| seed | people / reachable | confirmed (reachable) | decoys rejected / falsely confirmed | position error mean / max, m | first / last confirmed, s | coverage |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 6 / 4 | 6 (4) | 4/4 / 0 | 0.45 / 0.72 | 140 / 193 | 95.1 % |
| 2 | 6 / 5 | 6 (5) | 4/4 / 0 | 0.28 / 0.55 | 33 / 216 | 97.1 % |
| 3 | 5 / 4 | 4 (4) | 6/6 / 0 | 0.32 / 0.42 | 23 / 191 | 95.1 % |
| 7 | 6 / 6 | 6 (6) | 5/6 / 0 | 0.31 / 0.50 | 21 / 169 | 95.7 % |

Reachable people confirmed over all seeds: 19/19. The Phase 3 report's main table shows 17/17 on
worlds from generator v1.2 (17 reachable people). The generator changed in Phase 3b, and the table
above is the re-run on the current worlds.

## Phase 4: SLAM poses instead of ground truth

`mrg_slam` fused with wheel odometry and gyro (`LOC=slam`). Mean over seeds (min / max):

| mode | coverage % | ATE rmse, m | map P@2 | map R@2 | reachable people confirmed | decoys confirmed |
| --- | --- | --- | --- | --- | --- | --- |
| ground truth | 96.3 (95.0 / 99.7) | 0.00 | 0.97 | 0.86 | 18/19 | 0 |
| SLAM | 96.5 (95.2 / 98.9) | 0.26 (0.18 / 0.45) | 0.70 (0.59 / 0.78) | 0.70 (0.58 / 0.77) | 19/19 | 0 |

Max ATE per robot reaches 1.14 m on one seed. The scripted check marks the P@2 threshold (>= 0.7) as
FAIL at 0.698; this is reported as is. One ghost (a false CONFIRMED on a spot with no person or
decoy) occurred with SLAM on seed 2.

## Phase 5: mixed swarm

3 ground robots vs 3 ground robots + 2 drones. Mean over seeds (min / max):

| swarm | t90, s | last person confirmed, s | run end, s | reachable people | unreachable marked | decoys / ghosts confirmed | map P@2 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ground only | 310 (204 / 351) | 328 (209 / 391) | 357 | 19/19 | 2/4 | 0 / 2 | 0.97 |
| mixed, light drones | 60 (52 / 66) | 96 (73 / 113) | 186 | 19/19 | 4/4 | 0 / 0 | 0.98 |
| mixed, PX4 drones | 70 (63 / 81) | 118 (95 / 139) | 167 | 19/19 | 4/4 | 0 / 0 | 0.95 |

Air layer coverage 98.7-99.7 % (light) and ground layer 80.8-88.0 %. The drone map layer is shifted
0.05-0.10 m relative to the ground layer. PX4 EKF pose error is 0.03-0.06 m.

## Phase 6: limited communication

Robots exchange maps, people marks, bids and poses only as packets through a `comms_sim` radio model
(range, walls, bandwidth, loss, delay, outages). Base station at the entrance. Mixed swarm, ground
truth poses, mean over seeds:

| link | base map coverage % | t90 at base, s | links up, % of pairs | path to base, % | map delay median / p90, s | search ends, s | reachable people | decoys |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ideal | 100.0 | 66 | 100 | 100 | 1.0 / 1.0 | 158 | 19/19 | 0 |
| good | 99.9 | 70 | 93 | 100 | 1.0 / 1.2 | 170 | 19/19 | 0 |
| medium | 99.9 | 68 | 67 | 94 | 1.0 / 3.8 | 229 | 19/19 | 0 |
| poor | 99.9 | 70 | 43 | 65 | 1.4 / 115.3 | 618 | 19/19 | 0 |

Outage test (seed 1, ground robots only): `ground_1` is cut off from 60 to 200 s, drives 61.6 m
and builds its own map from version 60 to 200 while the base map stays at version 59. After the
link returns the base catches up in 2.0 s.

## Phase 7a: full stack

SLAM poses + 2 drones + `medium` link, seeds 1, 2, 3, 7.

| drones | reachable people | decoys confirmed | coverage % | t90, s | ATE rmse, m | map P@2 | layer shift, m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| light | 19/19 | 0 | 99.3-100 | 59-68 | 0.11-0.54 | 0.87-0.89 | 0.07-0.11 |
| PX4 | 18/19 | 0 | 100 | 76-85 | 0.13-0.32 | 0.74-0.92 | 0.07-0.11 |

Ghost confirmations: 0 in all light runs; one in one PX4 run (seed 7); one in the N=5 ground-only run. The missed person with PX4 is
on seed 2 (4 of 5). N=5 on seed 1 (5 ground robots, with and without 2 drones; single runs):

| swarm | coverage % | t90, s | reachable people | ATE rmse, m | map P@2 |
| --- | --- | --- | --- | --- | --- |
| 5 ground + 2 drones | 100.0 | 55 | 4/4 | 0.09 | 0.95 |
| 5 ground | 99.3 | 126 | 4/4 | 0.19 | 0.89 |

The `mrg_slam` init-guess patch was compared on four seeds and made ATE worse, so it is not applied
(see [PHASE_7A_REPORT.md](PHASE_7A_REPORT.md)). The comparison used three ground robots, and both the base and the patched runs used
`init_guess:=true` (not the default), so the base ATE here differs from the main 7a ATE above:

| seed | ATE rmse, m, base | ATE rmse, m, with patch |
| --- | --- | --- |
| 1 | 0.081 | 0.165 |
| 2 | 0.149 | 0.365 |
| 3 | 0.219 | 0.250 |
| 7 | 0.296 | 1.085 (run did not finish in 900 s) |

## Methodology

- **Coverage.** The `coverage` node (`rs_eval`) counts the share of free cells of the true floor
  plan (a mask written by the generator) that the shared map marks as known. Runs end after all
  explorers report done. With drones it also writes ground-layer and air-layer coverage.
- **People and decoys.** `scripts/phase3_eval.py` matches final marks to the generator's ground
  truth greedily by nearest pair within 1.0 m, one to one. A person is reachable if a ground robot
  can get to it; CONFIRMED reachable people count toward recall (threshold 90 %). A decoy marked
  CONFIRMED is a failure. A "ghost" is a CONFIRMED mark that matches neither a person nor a decoy.
- **ATE / RPE.** `scripts/ate_eval.py` compares the logged estimated pose with the true pose from
  Gazebo per robot (RMSE, max, RPE over 1 m and 5 m). No alignment is applied by default.
- **Map quality.** `scripts/map_eval.py` compares the final map with the rasterised true walls and
  rubble. P@1 / P@2 and R@1 / R@2 are precision and recall of occupied cells with a tolerance of 1
  or 2 cells.
- **Layers (RQ3).** `scripts/layers_eval.py` measures the shift of the air layer against the
  ground layer.
- **Comms.** `scripts/phase6_eval.py` and `scripts/phase6_summary.py` compute base coverage, map and
  detection delay, link availability and the outage test.
- **Summaries.** `scripts/phase4_summary.py`, `phase5_summary.py`, `phase7a_summary.py`.

## Limitations

- Simulation only. No real sensors, radios, or flights.
- SLAM runs with loop closures off, no SLAM graph exchange between robots by default, and known start poses. Map precision with SLAM (P@2 0.70) is
  clearly below ground truth (0.97).
- Thresholds and perception rules were tuned on the same four seeds (1, 2, 3, 7) that the checks
  use. There is no held-out set of seeds yet.
- Four seeds, one run each (N=5 only one seed). Means over four runs have wide ranges and are not
  statistically tested.
- The videos for Phases 2-6 are rendered from recorded run data, not screen captures. Only Phase 0-1
  footage is real Gazebo/RViz capture.
- The Phase 7 experiment series (swarm composition and size, RQ1 and RQ4) is in progress. The Phase 7a
  numbers above are a stack check, not that series.
- Results for the PX4 full stack have a single run per seed and PX4 runs are slower than real time
  (RTF about 0.4).
