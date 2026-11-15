#include "planning_core/nav_planner.hpp"

#include <cmath>

namespace planning_core {

const char* toString(NavPlanStatus s) {
    switch (s) {
        case NavPlanStatus::Ok: return "ok";
        case NavPlanStatus::StartInCollision: return "start_in_collision";
        case NavPlanStatus::GoalBlocked: return "goal_blocked";
        case NavPlanStatus::NoPath: return "no_path";
    }
    return "unknown";
}

namespace {

// Copies lethal cells over inflated ones in a disc around `centre`.
void relaxAround(PlanningGrid& inflated, const PlanningGrid& lethal, GridCell centre, double r) {
    const int ri = static_cast<int>(std::ceil(r));
    for (int dy = -ri; dy <= ri; ++dy) {
        for (int dx = -ri; dx <= ri; ++dx) {
            const GridCell c{centre.x + dx, centre.y + dy};
            if (!inflated.inBounds(c) || dx * dx + dy * dy > r * r) continue;
            const std::size_t i = static_cast<std::size_t>(c.y) * inflated.width + c.x;
            inflated.blocked[i] = lethal.blocked[i];
        }
    }
}

}  // namespace

NavPlanResult planOnMap(const PlanningGrid& obstacles, GridCell start, GridCell goal,
                        const NavPlanParams& params) {
    NavPlanResult out;
    const PlanningGrid lethal = inflate(obstacles, params.lethal_radius);
    if (lethal.isBlocked(start)) {
        out.status = NavPlanStatus::StartInCollision;
        return out;
    }
    if (lethal.isBlocked(goal)) {
        out.status = NavPlanStatus::GoalBlocked;
        return out;
    }
    PlanningGrid grid = inflate(obstacles, params.inflation_radius);
    relaxAround(grid, lethal, start, params.inflation_radius);
    relaxAround(grid, lethal, goal, params.inflation_radius);
    out.plan = params.algorithm == Algorithm::ThetaStar ? planThetaStar(grid, start, goal)
                                                        : planAStar(grid, start, goal);
    out.status = out.plan.found ? NavPlanStatus::Ok : NavPlanStatus::NoPath;
    return out;
}

}  // namespace planning_core
