# Design

The numbers in this document come from `results/eval_summary.md`, `results/eval_runs.csv` and
`results/demo/run/summary.json`. The full decision log, with reasons, is in `../DECISIONS.md`.

## Key decisions and trade-offs

| Decision | Why | Trade-off |
|---|---|---|
| ROS 2 Jazzy in Docker, built on `ros-base` | The machine has 7 GB of RAM and a nearly full disk. `ros-base` already ships colcon, gtest, launch_testing and rosbag2. | No GUI in the image. RViz needs a host or X setup. |
| `sim2d` instead of Gazebo | Gazebo needs several GB more, plus rendering and RAM. It would also make the launch tests slow and flaky. The metrics being evaluated are planar. | No wheel-slip dynamics, no 3D effects and no sensor-latency model. |
| ROS-free `*_core` libraries | The maths can be unit-tested and run under sanitizers without ROS. | A small CMake block is repeated in each package. |
| No Eigen | Only 3x3 and 2x2 linear algebra is needed. | The matrix code is hand-written (and tested). |
| 3-state EKF with odometry as the control input | This is the standard design when wheel odometry is the main motion source. | The IMU gyro does not correct the yaw rate; a 5-state filter would allow that. |
| ICP against the prior map, not scan-to-scan | Matching against the map bounds drift, whereas scan-to-scan matching still drifts. | It needs a prior map. In an unknown environment, `grid_mapping` would be the map source. |
| Topics instead of Nav2-style actions | The nodes are small and easy to test, and only one node publishes `/cmd_vel` at a time. | There are no goal IDs and no feedback channel. |
| Latched `/goal_pose` | The first evaluation smoke test lost its initial goal while discovery was slow. | RViz's SetGoal tool cannot send goals, because its publisher is volatile. |
| Planner uses the prior map plus the live map, and ignores live hits within 0.1 m of known walls | Range and localisation noise smear walls by about one cell, which made the robot look as if it was in collision. | New obstacles that touch a known wall are slightly underestimated. |
| The behaviour tree owns recovery; the controller never recovers by itself | This gives a single place for mission logic and avoids two nodes publishing motion commands. | The controller has to report "blocked" reliably (`StuckDetector`). |
| The evaluation runs the real ROS stack | It measures the code that ships. | Runs are wall-clock based and cannot be reproduced bit for bit. Each takes about 25-45 s. |

## Alternatives considered

- **Gazebo.** This was rejected for the resource reasons given above. The stack only uses
  `/scan`, `/odom`, `/imu` and `/cmd_vel`, so switching to Gazebo needs no code changes.
- **A k-d tree for ICP nearest neighbours.** A hash grid with a cell size equal to the maximum
  correspondence distance is simpler and answers each query in O(1).
- **Fetching BehaviorTree.CPP with FetchContent.** The ROS apt package avoids a compile of
  several minutes on every image build.
- **SLAM.** Mapping uses known poses from the EKF instead. Map quality therefore depends on the
  localiser.
- **Hybrid A* or MPC.** Theta* and DWA were chosen as the improved planner and controller. They
  are simpler to explain and test, and they target the measured problems: A*'s 45-degree
  zig-zags, and pure pursuit cutting corners.

## Baseline vs improved

The table compares the baseline, A* with pure pursuit, against the improved pair, Theta* with
DWA. Each row has 12 runs and covers all noise and clutter levels. All values are means.

| Metric | A* + pure pursuit | Theta* + DWA | Change |
|---|---|---|---|
| Success | 12/12 | 12/12 | none |
| Time to goal (s) | 28.9 | 23.9 | 17% faster |
| Mean absolute curvature (1/m) | 0.60 | 0.22 | 63% lower |
| Integral of curvature squared (1/m) | 6.6 | 1.4 | 79% lower |
| Plan turning (rad) | 7.33 | 1.79 | 76% lower |
| Plan length (m) | 11.30 | 10.53 | 7% shorter |
| ATE RMSE (m) | 0.014 | 0.012 | about the same |

Most of the gain comes from the planner. Theta* with pure pursuit already reaches 23.9 s and a
curvature integral of 1.5. Swapping the controller on A* paths (pure pursuit to DWA) brings the
curvature integral from 6.6 down to 2.4.

Localisation is robust to the test conditions, which are listed below with their mean ATE RMSE
(12 runs each).

| Noise | Clutter | ATE RMSE (m) |
|---|---|---|
| Low | None | 0.004 |
| Low | 12 unmapped boxes | 0.011 |
| High (3x every sensor error) | None | 0.013 |
| High (3x every sensor error) | 12 unmapped boxes | 0.024 |

All 48 runs reached the goal.

On the blocked path, the behaviour tree ran a recovery in 11 of the 12 runs. The robot reached
the goal in 10 of those 11 runs (91%).

## Known limitations

- **2D simulator, not Gazebo.** See above. Results show how the software behaves under modelled
  noise, not under real-world physics.
- **Small samples, means only.** Each cell has between 3 and 12 runs (3 seeds), and there are no
  confidence intervals or variance figures in the summary. Differences between Theta* + pure
  pursuit and Theta* + DWA, for example 1.5 vs 1.4, are within what small samples could produce.
- **One recovery failure.** In the blocked-path run with Theta* + DWA and seed 1, all 4 attempts
  were used up. DWA kept stopping close to the dropped wall: it reported blocked, and on the next
  attempt the planner reported `start_in_collision`. The 0.3 m back-up did not break the cycle.
  The failure was left unfixed on purpose, because tuning on the evaluation set would overfit it.
- **No motion compensation within a scan**, and the scan is matched at the latest EKF pose.
- **CI has not yet been run on GitHub.** The workflow mirrors `make native-test` and
  `make docker-test`, which have only been run locally.

## Next steps

1. Make recovery adaptive, for example a longer back-up or a different rotation after repeated
   `start_in_collision` failures. Then re-evaluate on new seeds.
2. Run more seeds per cell and report confidence intervals.
3. Push to GitHub and confirm that both CI jobs pass.
4. Point the stack at Gazebo, or a real robot's topics, to test wheel slip and sensor latency.
5. Replace topic-based goals with Nav2-style actions to get goal IDs and feedback.
