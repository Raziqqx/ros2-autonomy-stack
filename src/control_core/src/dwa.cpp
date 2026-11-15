#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core/controllers.hpp"

namespace control_core {

DwaOutput computeDwa(const Pose2& pose, const VelocityCommand& current, const Point& goal,
                     const std::vector<Point>& obstacles, const DwaParams& p) {
    DwaOutput out;
    out.cost = std::numeric_limits<double>::infinity();
    // Dynamic window: velocities reachable within one control period.
    const double v_lo = std::max(p.v_min, current.v - p.acc_v * p.control_period);
    const double v_hi = std::min(p.v_max, current.v + p.acc_v * p.control_period);
    const double w_lo = std::max(-p.w_max, current.w - p.acc_w * p.control_period);
    const double w_hi = std::min(p.w_max, current.w + p.acc_w * p.control_period);
    const double goal_dist0 = std::max(std::hypot(goal.x - pose.x, goal.y - pose.y), 1e-6);
    const int steps = std::max(1, static_cast<int>(std::round(p.horizon / p.sim_dt)));

    for (int iv = 0; iv < p.v_samples; ++iv) {
        const double v = p.v_samples > 1 ? v_lo + (v_hi - v_lo) * iv / (p.v_samples - 1) : v_hi;
        for (int iw = 0; iw < p.w_samples; ++iw) {
            const double w = p.w_samples > 1 ? w_lo + (w_hi - w_lo) * iw / (p.w_samples - 1) : 0.0;
            ++out.evaluated;
            // Forward-simulate and track the closest approach to any obstacle.
            Pose2 q = pose;
            double clearance = std::numeric_limits<double>::infinity();
            for (int k = 0; k < steps && clearance > 0.0; ++k) {
                q = integrateUnicycle(q, v, w, p.sim_dt);
                for (const Point& o : obstacles)
                    clearance =
                        std::min(clearance, std::hypot(o.x - q.x, o.y - q.y) - p.robot_radius);
            }
            if (clearance <= 0.0) continue;  // collides within the horizon
            // Admissible only if the robot can still brake before the obstacle.
            if (v > std::sqrt(2.0 * p.acc_v * clearance)) continue;

            const double goal_term = std::hypot(goal.x - q.x, goal.y - q.y) / goal_dist0;
            const double heading_err = std::abs(
                std::remainder(std::atan2(goal.y - q.y, goal.x - q.x) - q.theta, 2.0 * M_PI));
            const double clear_term =
                (p.clearance_cap - std::min(clearance, p.clearance_cap)) / p.clearance_cap;
            const double vel_term = p.v_max > 0 ? (p.v_max - v) / p.v_max : 0.0;
            const double cost = p.w_goal * goal_term + p.w_heading * heading_err / M_PI +
                                p.w_clearance * clear_term + p.w_velocity * vel_term;
            if (cost < out.cost) {
                out.cost = cost;
                out.cmd = {v, w};
                out.clearance = clearance;
                out.feasible = true;
            }
        }
    }
    if (!out.feasible) out.cmd = {0.0, 0.0};  // caller decides how to recover
    return out;
}

}  // namespace control_core
