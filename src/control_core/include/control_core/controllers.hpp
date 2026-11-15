// Local path-tracking controllers for a differential-drive robot:
// pure pursuit (geometric baseline) and the Dynamic Window Approach (samples
// admissible velocities, forward-simulates them and scores goal progress,
// obstacle clearance and speed). No ROS types.
#pragma once

#include <cstddef>
#include <vector>

namespace control_core {

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Pose2 {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct VelocityCommand {
    double v = 0.0;  // m/s
    double w = 0.0;  // rad/s
};

// Exact unicycle step (arc), shared by DWA's forward simulation and tests.
Pose2 integrateUnicycle(const Pose2& p, double v, double w, double dt);

struct PurePursuitParams {
    double lookahead = 0.6;           // m
    double v_max = 0.5;               // m/s
    double w_max = 1.5;               // rad/s
    double goal_tolerance = 0.1;      // m
    double slow_down_distance = 0.6;  // linear ramp to v_min near the goal
    double v_min = 0.05;
    // Speed is scaled by 1 / (1 + curvature_gain * |kappa|) on tight turns.
    double curvature_gain = 0.5;
    // Rotate in place when the lookahead point is more than this far off the
    // heading (rad), e.g. at the start of a path behind the robot.
    double rotate_in_place_angle = 1.2;
};

struct PurePursuitOutput {
    VelocityCommand cmd;
    Point lookahead_point;
    double curvature = 0.0;
    bool goal_reached = false;
};

class PurePursuit {
public:
    explicit PurePursuit(const PurePursuitParams& params = {});
    void setPath(const std::vector<Point>& path);
    PurePursuitOutput compute(const Pose2& pose);
    std::size_t progressIndex() const { return progress_; }

private:
    PurePursuitParams params_;
    std::vector<Point> path_;
    // Index of the path segment the robot is on; only moves forward, so a
    // path that loops back near itself is not short-cut.
    std::size_t progress_ = 0;
};

struct DwaParams {
    double v_min = 0.0;
    double v_max = 0.5;
    double w_max = 1.5;
    double acc_v = 1.0;           // m/s^2
    double acc_w = 3.0;           // rad/s^2
    double control_period = 0.1;  // s: the dynamic window is +- acc * period
    double horizon = 1.5;         // s of forward simulation
    double sim_dt = 0.1;
    int v_samples = 7;
    int w_samples = 21;
    double robot_radius = 0.2;
    // Cost weights; each term is normalised to roughly [0, 1].
    double w_goal = 1.0;         // distance from trajectory end to the goal
    double w_heading = 0.3;      // heading error to the goal at trajectory end
    double w_clearance = 0.4;    // penalty for passing close to obstacles
    double w_velocity = 0.2;     // preference for speed
    double clearance_cap = 0.6;  // clearance beyond this is not rewarded more
};

struct DwaOutput {
    VelocityCommand cmd;
    bool feasible = false;  // false: every sampled trajectory collides
    double cost = 0.0;
    double clearance = 0.0;  // of the chosen trajectory
    std::size_t evaluated = 0;
};

// Chooses (v, w) given the current pose and velocity, a local goal (world
// frame) and obstacle points (world frame, e.g. the latest scan).
DwaOutput computeDwa(const Pose2& pose, const VelocityCommand& current, const Point& goal,
                     const std::vector<Point>& obstacles, const DwaParams& params = {});

}  // namespace control_core
