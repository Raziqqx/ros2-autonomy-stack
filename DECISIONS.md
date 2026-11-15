# DECISIONS — ros2-autonomy-stack

Decisions taken without asking, with the reason. Newest at the bottom.

## Environment and workspace

1. **ROS 2 Jazzy in Docker, `ros:jazzy-ros-base` only.** ROS 2 is not installed natively and the
   machine has 7 GB RAM and a nearly full C: drive. `ros-base` (1.3 GB) already contains colcon,
   `ament_cmake_gtest`, `launch_testing(_ament_cmake)`, `tf2_ros` and the `sensor_msgs`/`nav_msgs`
   packages, so the Dockerfile adds no apt layer. One Dockerfile builds the workspace into the
   image; `make docker-test` runs `colcon test` inside it.
2. **Lightweight 2D simulator (`sim2d`) instead of Gazebo.** Gazebo needs the desktop image
   (several GB more), a GPU or software rendering, and far more RAM than is available; it would
   also make the launch tests slow and flaky. The evaluation questions (localisation error,
   time to goal, smoothness, recovery) are planar, so a 2D diff-drive simulator with a raycast
   LiDAR, noisy odometry and IMU, and a ground-truth topic tests the same software with full
   control over seeds and noise levels. Trade-off: no wheel slip dynamics, no 3D effects, no
   sensor latency model. The stack only uses standard topics (`/scan`, `/odom`, `/imu`,
   `/cmd_vel`), so it could be pointed at Gazebo later without code changes.
3. **Pure-maths code lives in five ROS-free `*_core` packages** (`ekf_core`, `icp_core`,
   `grid_core`, `planning_core`, `control_core`). Each `CMakeLists.txt` builds as an ament
   package when `ament_cmake` is found and as a plain CMake project otherwise (GoogleTest via
   FetchContent). `native/CMakeLists.txt` is a superbuild for the native ASan/UBSan run
   (`make native-test`); it has a `COLCON_IGNORE` so colcon skips it. This keeps the maths
   testable in CI without ROS. The small CMake block is repeated per package rather than shared,
   because each ament package must be self-contained.
4. **No Eigen.** It is not installed natively, and the only linear algebra needed is 3x3
   (EKF) and 2x2 / 3x3 solves (ICP), which are a few lines each.

## Algorithms

5. **EKF state is (x, y, theta) with odometry velocities as the control input.** IMU yaw and the
   ICP pose are measurements. A 5-state (with v, w) filter would let the IMU gyro correct the
   rate, but wheel odometry is the primary motion source here and the 3-state form is the
   standard "odometry as control" design. Process noise is a density (variance grows with
   `sigma^2 * dt`), so results do not depend on the odometry rate (tested). Updates use the
   Joseph form and chi-square innovation gates (99.9 %), so an ICP mismatch is rejected rather
   than corrupting the state.
6. **ICP: point-to-point baseline plus point-to-line.** The tests show the depth-bar comparison
   directly: on a sampled room with a 0.18 m / 0.06 rad offset, point-to-point stalls at a fixed
   point (scan points match neighbouring samples on the same wall and "slide"), while
   point-to-line (Gauss-Newton on normal distances, PCA normals) converges to < 1 mm.
   Nearest neighbours use a hash grid with cell = max correspondence distance, which is simpler
   than a k-d tree and O(1) per query for this use. A single straight wall is reported as
   degenerate (not converged) instead of returning an arbitrary slide.
7. **Map-based ICP correction.** `ekf_localization` matches each scan against the occupied cells
   of the prior map (the same PGM the simulator uses), which bounds drift; scan-to-scan
   matching would still drift. `grid_mapping` independently builds a log-odds map from scans and
   the EKF pose, which is what an unknown environment would use and what the launch test checks.
8. **Occupancy grid**: log-odds with clamping (+0.85 / -0.4, clamp +-4) so cells can change their
   mind; max-range returns clear space but never mark an obstacle. Map loading follows
   map_server semantics; pixels between `free_thresh` and `occupied_thresh` (unknown) are treated
   as blocked for the simulator and planners (conservative).
9. **Planners**: 8-connected A* with no corner cutting (baseline) and basic Theta* (any-angle)
   sharing one best-first search; the line-of-sight test is a supercover traversal that also
   refuses to squeeze through a diagonal corner, so both planners agree on what is passable.
   Obstacle inflation is a disc of the robot radius.
10. **Controllers**: pure pursuit (adaptive speed on curvature, slow-down near goal,
    rotate-in-place when the path is behind) as baseline; DWA (dynamic window from acceleration
    limits, braking-distance admissibility, normalised goal/heading/clearance/speed costs) as the
    obstacle-aware alternative. DWA returns `feasible = false` and a zero command when boxed in so
    the behaviour tree (run B) can trigger recovery.

## ROS nodes and scenario test (run A)

11. **Simulator noise model** (`config/sim.yaml`): the true robot follows `/cmd_vel` exactly;
    wheel odometry has a 2 % scale error on v, a 0.02 rad/s bias on w and white noise per 50 Hz
    sample, and reads exactly zero when the wheels are still. The systematic terms are what make
    dead reckoning drift (0.87-1.04 m over the 8.8 m scenario in three runs), so the test exercises the
    corrections rather than just averaging noise. The IMU yaw is absolute (AHRS-style) with
    white noise. All randomness comes from one `std::mt19937` seeded by `seed`.
12. **Collision handling in the sim**: a step that would make the robot disc overlap an occupied
    cell is not applied (the robot stops and a warning is logged). Run B's recovery behaviour
    relies on this "blocked" signal.
13. **ICP target = obstacle surface points** (edge midpoints between occupied and free cells)
    rather than occupied cell centres, which sit half a cell inside the wall and would bias
    point-to-line ICP by up to 2.5 cm whenever only one side of the room is visible.
14. **ICP acceptance** needs convergence, RMSE < 5 cm and >= 50 % inliers, then the EKF's own
    3-DOF chi-square gate. Known simplification: the scan is matched at the latest EKF pose
    (odometry at 50 Hz, so up to 20 ms of motion, < 1 cm at 0.4 m/s) without motion
    compensation within the scan.
15. **Mapping with known poses**: `grid_mapping` inserts a scan only if the latest EKF pose is
    within 0.1 s of the scan stamp. It is not SLAM: map quality depends on the localiser.
16. **Scenario test budgets** (`autonomy_bringup/test/test_localization_mapping.py`, seed 7):
    max position error 0.10 m, RMS 0.05 m, max heading error 0.05 rad, > 500 occupied map cells,
    ground-truth path >= 8 m. The measured run gave 0.012 m max / 0.004 m RMS / 0.026 rad
    against 0.87-1.04 m of raw odometry drift, so the budgets leave margin for timing jitter on a slow
    CI machine while still failing if ICP or the IMU update stops working (odometry alone
    exceeds them by ~9x). The test uses wall-clock timers and an open-loop pattern, so the exact
    trajectory varies slightly between runs; seeds fix the noise draws only.
17. **Docker image**: workspace built in Release inside the image (colcon test needs the build
    tree, so it stays in the image). `make docker-test` rebuilds the image so tests always run
    against the current sources.

## Navigation and task execution (run B)

18. **BehaviorTree.CPP v4 from the ROS apt repo** (`ros-jazzy-behaviortree-cpp` 4.10), in its
    own Dockerfile layer before `COPY src`, so source edits reuse it. FetchContent would add
    a multi-minute compile to every image build. CMake links `${behaviortree_cpp_TARGETS}`
    (the ament export), not `BT::behaviortree_cpp`, which the ROS package does not define.
19. **Topic contract between the nodes**: `task_bt` sends `/goal_pose` (an empty `frame_id`
    cancels); `global_planner` answers with `/plan` (empty = no goal or failure) and
    `/planner/status`; `local_controller` follows `/plan` and reports
    `idle | following | arrived | blocked` on `/controller/state`. The BT owns the mission
    and recovery; the controller never recovers on its own. Recovery drives `/cmd_vel`
    directly while the goal is cancelled, so there is never more than one active publisher
    of motion commands. This is a much smaller version of the Nav2 action interfaces;
    actions would add goal IDs and feedback, but topics keep the nodes easy to test.
20. **`/goal_pose` is latched (transient local)** and the BT waits until the planner
    subscribes. The first evaluation smoke test lost the initial goal when four stacks
    started at once (slow discovery); the robot sat still for 90 s. Consequence: RViz's
    SetGoal tool (volatile publisher) cannot send goals, so the RViz config omits it.
21. **Planner on prior map + live map, replanned at 1 Hz.** Obstacles not in the prior map
    (clutter, the dropped wall) only become known through `grid_mapping`. Live hits within
    0.1 m of a prior obstacle are ignored: localisation and range noise smear walls by about
    a cell, and counting that smear made the robot look in collision when it passed a shelf
    corner (seen in the first smoke run). The cost is that new obstacles touching a known
    wall are slightly underestimated.
22. **Collision vs margin (`planning_core::planOnMap`)**: start within robot radius of an
    obstacle = `start_in_collision` (a planner failure, so the BT recovers); start inside
    the 0.15 m inflation margin but not in collision = plan anyway, using the un-inflated
    grid near the start and goal. Otherwise any controller that cuts a corner (pure
    pursuit does) would trigger a recovery. `lethal_margin` was 0.05 m at first; pure
    pursuit cutting round the dropped wall then hit it repeatedly, so it is 0.
23. **Stuck detection is in the controller** (`control_core::StuckDetector`): commanded
    |v| > 0.05 m/s but wheel odometry < 0.01 m/s for 1 s, or DWA has had no admissible
    velocity for 1 s. Wheel odometry is the right signal here because the sim's encoders
    read exactly zero when the robot is pushed against something (decision 12).
24. **BT shape**: `RetryUntilSuccessful(4)` over `Fallback(NavigateTo, ForceFailure(Recover))`,
    with GoTo / Dock / Patrol trees sharing that subtree. Recover backs up 0.3 m and rotates
    90 degrees (alternating direction) and then lets the retry replan. The tree logic is
    unit-tested with mock leaves (`task_bt/test/test_trees.cpp`); the ROS leaves are covered
    by the launch tests. `NavigateTo` checks arrival with the EKF pose (0.15 m), so it does
    not depend on message ordering with the controller.
25. **DWA local goal** is the point 1.0 m of arc length beyond the closest point on the path,
    interpolated along segments, so it works on sparse Theta* paths as well as dense A*
    paths. DWA obstacles are the scan endpoints within reach of the horizon.
26. **sim2d evaluation knobs**: `noise_scale` scales every sensor error, including the
    systematic odometry terms; `clutter_count` adds random 0.3-0.6 m boxes (seeded, kept 1 m
    away from the start and goal) that are not in the prior map; `drop_after_m` drops a
    1.6 m wall 0.45 m ahead of the robot after it has driven that far. The true world is
    published on `/sim/world` for plots.
