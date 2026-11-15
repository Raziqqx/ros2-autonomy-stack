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
