#include "planning_core/grid_planners.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace planning_core {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

double euclid(GridCell a, GridCell b) { return std::hypot(a.x - b.x, a.y - b.y); }

double octile(GridCell a, GridCell b) {
    const double dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
    return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
}

struct QueueEntry {
    double f;
    double h;  // tie-break on smaller h (closer to goal) to cut expansions
    int index;
    bool operator>(const QueueEntry& o) const { return f > o.f || (f == o.f && h > o.h); }
};

// Shared best-first search. `relax(s, n, g, parent)` returns the best
// (g, parent) for neighbour n reached from expanded cell s; A* always goes
// through s, Theta* may skip to s's parent when it has line of sight.
template <typename Heuristic, typename Relax>
PlanResult search(const PlanningGrid& grid, GridCell start, GridCell goal, Heuristic h,
                  Relax relax) {
    PlanResult result;
    if (grid.isBlocked(start) || grid.isBlocked(goal)) return result;
    const auto idx = [&](GridCell c) { return c.y * grid.width + c.x; };
    const auto cell = [&](int i) { return GridCell{i % grid.width, i / grid.width}; };
    const std::size_t n = static_cast<std::size_t>(grid.width) * grid.height;
    std::vector<double> g(n, kInf);
    std::vector<int> parent(n, -1);
    std::vector<uint8_t> closed(n, 0);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open;

    g[idx(start)] = 0.0;
    parent[idx(start)] = idx(start);
    open.push({h(start), h(start), idx(start)});
    while (!open.empty()) {
        const int s = open.top().index;
        open.pop();
        if (closed[s]) continue;  // stale entry (lazy deletion)
        closed[s] = 1;
        ++result.expansions;
        const GridCell sc = cell(s);
        if (sc == goal) {
            result.found = true;
            result.cost = g[s];
            for (int i = s;; i = parent[i]) {
                result.path.push_back(cell(i));
                if (parent[i] == i) break;
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }
        for (int k = 0; k < 8; ++k) {
            const GridCell nc{sc.x + kDx[k], sc.y + kDy[k]};
            if (grid.isBlocked(nc)) continue;
            // No corner cutting on diagonal moves.
            if (k >= 4 &&
                (grid.isBlocked({sc.x + kDx[k], sc.y}) || grid.isBlocked({sc.x, sc.y + kDy[k]})))
                continue;
            const int ni = idx(nc);
            if (closed[ni]) continue;
            const auto [ng, np] = relax(s, nc, g, parent);
            if (ng < g[ni]) {
                g[ni] = ng;
                parent[ni] = np;
                open.push({ng + h(nc), h(nc), ni});
            }
        }
    }
    return result;  // open list exhausted: unreachable
}

}  // namespace

PlanResult planAStar(const PlanningGrid& grid, GridCell start, GridCell goal,
                     const AStarParams& params) {
    const auto cell = [&](int i) { return GridCell{i % grid.width, i / grid.width}; };
    return search(
        grid, start, goal, [&](GridCell c) { return params.heuristic_weight * octile(c, goal); },
        [&](int s, GridCell n, const std::vector<double>& g, const std::vector<int>&) {
            return std::pair<double, int>{g[s] + euclid(cell(s), n), s};
        });
}

PlanResult planThetaStar(const PlanningGrid& grid, GridCell start, GridCell goal) {
    const auto cell = [&](int i) { return GridCell{i % grid.width, i / grid.width}; };
    return search(
        grid, start, goal, [&](GridCell c) { return euclid(c, goal); },
        [&](int s, GridCell n, const std::vector<double>& g, const std::vector<int>& parent) {
            // Path 2: connect n straight to s's parent if it can see it.
            const int p = parent[s];
            if (lineOfSight(grid, cell(p), n))
                return std::pair<double, int>{g[p] + euclid(cell(p), n), p};
            return std::pair<double, int>{g[s] + euclid(cell(s), n), s};
        });
}

bool lineOfSight(const PlanningGrid& grid, GridCell a, GridCell b) {
    int dx = std::abs(b.x - a.x), dy = std::abs(b.y - a.y);
    const int sx = b.x > a.x ? 1 : -1, sy = b.y > a.y ? 1 : -1;
    int x = a.x, y = a.y;
    int n = dx + dy;
    // error tracks (doubled) which cell boundary the segment crosses next.
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
    if (grid.isBlocked(a)) return false;
    while (n > 0) {
        if (error > 0) {
            x += sx;
            error -= dy;
            --n;
        } else if (error < 0) {
            y += sy;
            error += dx;
            --n;
        } else {
            // Exactly through a corner: both side cells must be free.
            if (grid.isBlocked({x + sx, y}) || grid.isBlocked({x, y + sy})) return false;
            x += sx;
            y += sy;
            error += dx - dy;
            n -= 2;
        }
        if (grid.isBlocked({x, y})) return false;
    }
    return true;
}

PlanningGrid inflate(const PlanningGrid& grid, double radius) {
    PlanningGrid out = grid;
    const int r = static_cast<int>(std::ceil(radius));
    for (int y = 0; y < grid.height; ++y)
        for (int x = 0; x < grid.width; ++x) {
            if (!grid.isBlocked({x, y})) continue;
            for (int dy = -r; dy <= r; ++dy)
                for (int dx = -r; dx <= r; ++dx) {
                    const GridCell c{x + dx, y + dy};
                    if (grid.inBounds(c) && dx * dx + dy * dy <= radius * radius)
                        out.blocked[static_cast<std::size_t>(c.y) * grid.width + c.x] = 1;
                }
        }
    return out;
}

double pathLength(const std::vector<GridCell>& path) {
    double len = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) len += euclid(path[i - 1], path[i]);
    return len;
}

}  // namespace planning_core
