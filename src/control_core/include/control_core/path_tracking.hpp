// Helpers the local_controller node needs around the controllers: picking a
// local goal on the global path (for DWA) and detecting that the robot is
// stuck (for the behaviour tree's recovery). No ROS types.
#pragma once

#include <vector>

#include "control_core/controllers.hpp"

namespace control_core {

// Point `lookahead` metres of arc length beyond the path point closest to
// `pose`, interpolated along segments (works for dense A* paths and sparse
// Theta* paths). Returns the last point if the path is shorter. Path must be
// non-empty.
Point localGoal(const std::vector<Point>& path, const Pose2& pose, double lookahead);

// Reports "stuck" when the robot has been commanded to drive (|v_cmd| above
// cmd_threshold) but the wheel odometry has read (almost) no motion for
// `timeout` seconds, e.g. because it is pressed against an obstacle.
class StuckDetector {
public:
    StuckDetector(double timeout, double cmd_threshold = 0.05, double motion_epsilon = 0.01)
        : timeout_(timeout), cmd_threshold_(cmd_threshold), motion_epsilon_(motion_epsilon) {}

    // Feed one control cycle (t in seconds, monotonic). Returns true when stuck.
    bool update(double t, double cmd_v, double measured_v);
    void reset() { since_ = -1.0; }

private:
    double timeout_, cmd_threshold_, motion_epsilon_;
    double since_ = -1.0;  // time the stall started, < 0 if not stalled
};

}  // namespace control_core
