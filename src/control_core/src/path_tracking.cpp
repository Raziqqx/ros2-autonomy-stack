#include "control_core/path_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace control_core {

Point localGoal(const std::vector<Point>& path, const Pose2& pose, double lookahead) {
    if (path.size() == 1) return path.front();
    // Closest point on any segment (segment index + parameter).
    std::size_t best_seg = 0;
    double best_t = 0.0, best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const double dx = path[i + 1].x - path[i].x, dy = path[i + 1].y - path[i].y;
        const double len2 = dx * dx + dy * dy;
        double t = len2 > 0.0 ? ((pose.x - path[i].x) * dx + (pose.y - path[i].y) * dy) / len2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        const double px = path[i].x + t * dx - pose.x, py = path[i].y + t * dy - pose.y;
        const double d2 = px * px + py * py;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_seg = i;
            best_t = t;
        }
    }
    // Walk forward `lookahead` metres from there.
    double remaining = lookahead;
    Point from{path[best_seg].x + best_t * (path[best_seg + 1].x - path[best_seg].x),
               path[best_seg].y + best_t * (path[best_seg + 1].y - path[best_seg].y)};
    for (std::size_t i = best_seg + 1; i < path.size(); ++i) {
        const double dx = path[i].x - from.x, dy = path[i].y - from.y;
        const double len = std::hypot(dx, dy);
        if (len >= remaining && len > 0.0)
            return {from.x + dx * remaining / len, from.y + dy * remaining / len};
        remaining -= len;
        from = path[i];
    }
    return path.back();
}

bool StuckDetector::update(double t, double cmd_v, double measured_v) {
    const bool stalled =
        std::abs(cmd_v) > cmd_threshold_ && std::abs(measured_v) < motion_epsilon_;
    if (!stalled) {
        since_ = -1.0;
        return false;
    }
    if (since_ < 0.0) since_ = t;
    return t - since_ >= timeout_;
}

}  // namespace control_core
