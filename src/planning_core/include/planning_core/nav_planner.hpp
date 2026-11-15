// Planning on a robot's map with obstacle inflation, as the global_planner
// node does it. Separates "the robot is in collision" (fail, so the behaviour
// tree can recover) from "the robot has drifted into the safety margin"
// (plan anyway, so the robot can leave the margin). No ROS types.
#pragma once

#include "planning_core/grid_planners.hpp"

namespace planning_core {

enum class Algorithm { AStar, ThetaStar };

enum class NavPlanStatus { Ok, StartInCollision, GoalBlocked, NoPath };

const char* toString(NavPlanStatus s);

struct NavPlanParams {
    Algorithm algorithm = Algorithm::AStar;
    // Both radii in cells. Within lethal_radius of an obstacle the robot disc
    // touches it; inflation_radius >= lethal_radius adds a clearance margin.
    double lethal_radius = 5.0;
    double inflation_radius = 6.0;
};

struct NavPlanResult {
    NavPlanStatus status = NavPlanStatus::NoPath;
    PlanResult plan;
};

// `obstacles` is the raw (un-inflated) grid. Cells within inflation_radius of
// the start or goal use the lethal grid instead of the inflated one, so a
// start or goal inside the margin (but not in collision) is still plannable.
NavPlanResult planOnMap(const PlanningGrid& obstacles, GridCell start, GridCell goal,
                        const NavPlanParams& params);

}  // namespace planning_core
