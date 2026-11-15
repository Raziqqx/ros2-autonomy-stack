#include <algorithm>
#include <cmath>

#include "control_core/controllers.hpp"

namespace control_core {

Pose2 integrateUnicycle(const Pose2& p, double v, double w, double dt) {
    Pose2 out = p;
    if (std::abs(w) < 1e-9) {
        out.x += v * dt * std::cos(p.theta);
        out.y += v * dt * std::sin(p.theta);
    } else {
        const double r = v / w;
        out.x += r * (std::sin(p.theta + w * dt) - std::sin(p.theta));
        out.y += r * (std::cos(p.theta) - std::cos(p.theta + w * dt));
    }
    out.theta = std::remainder(p.theta + w * dt, 2.0 * M_PI);
    return out;
}

PurePursuit::PurePursuit(const PurePursuitParams& params) : params_(params) {}

void PurePursuit::setPath(const std::vector<Point>& path) {
    path_ = path;
    progress_ = 0;
}

PurePursuitOutput PurePursuit::compute(const Pose2& pose) {
    PurePursuitOutput out;
    if (path_.empty()) {
        out.goal_reached = true;
        return out;
    }
    const Point& goal = path_.back();
    const double goal_dist = std::hypot(goal.x - pose.x, goal.y - pose.y);
    if (goal_dist < params_.goal_tolerance) {
        out.goal_reached = true;
        out.lookahead_point = goal;
        return out;
    }

    // Advance progress to the closest segment within a short window ahead.
    auto segDist = [&](std::size_t i) {
        const Point &a = path_[i], &b = path_[std::min(i + 1, path_.size() - 1)];
        const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        const double t =
            len2 > 0 ? std::clamp(((pose.x - a.x) * dx + (pose.y - a.y) * dy) / len2, 0.0, 1.0)
                     : 0.0;
        return std::hypot(a.x + t * dx - pose.x, a.y + t * dy - pose.y);
    };
    std::size_t best = progress_;
    double best_d = segDist(progress_);
    for (std::size_t i = progress_ + 1; i + 1 < path_.size(); ++i) {
        const double d = segDist(i);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
        if (d > best_d + 2.0 * params_.lookahead) break;  // window: stop once far past
    }
    progress_ = best;

    // Lookahead point: first intersection of the lookahead circle with the
    // path beyond the current segment, else the goal.
    const double L = params_.lookahead;
    out.lookahead_point = goal;
    for (std::size_t i = progress_; i + 1 < path_.size(); ++i) {
        const Point &a = path_[i], &b = path_[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double fx = a.x - pose.x, fy = a.y - pose.y;
        const double qa = dx * dx + dy * dy, qb = 2 * (fx * dx + fy * dy),
                     qc = fx * fx + fy * fy - L * L;
        const double disc = qb * qb - 4 * qa * qc;
        if (qa == 0.0 || disc < 0.0) continue;
        const double t = (-qb + std::sqrt(disc)) / (2 * qa);  // farther root: ahead
        if (t >= 0.0 && t <= 1.0) {
            out.lookahead_point = {a.x + t * dx, a.y + t * dy};
            break;
        }
    }

    // Lookahead point in the robot frame; curvature of the arc through it.
    const double ex = out.lookahead_point.x - pose.x, ey = out.lookahead_point.y - pose.y;
    const double c = std::cos(pose.theta), s = std::sin(pose.theta);
    const double lx = c * ex + s * ey, ly = -s * ex + c * ey;
    const double d2 = lx * lx + ly * ly;
    if (std::abs(std::atan2(ly, lx)) > params_.rotate_in_place_angle) {
        out.cmd = {0.0, std::copysign(params_.w_max, ly)};
        return out;
    }
    out.curvature = d2 > 0 ? 2.0 * ly / d2 : 0.0;
    double v = params_.v_max / (1.0 + params_.curvature_gain * std::abs(out.curvature));
    v = std::min(v,
                 std::max(params_.v_min, params_.v_max * goal_dist / params_.slow_down_distance));
    double w = v * out.curvature;
    if (std::abs(w) > params_.w_max) {
        // Keep the same arc, slower.
        w = std::copysign(params_.w_max, w);
        v = w / out.curvature;
    }
    out.cmd = {v, w};
    return out;
}

}  // namespace control_core
