#include <gtest/gtest.h>
#include <algorithm>

#include "planning_core/nav_planner.hpp"

using namespace planning_core;

namespace {

PlanningGrid emptyGrid(int w, int h) { return {w, h, std::vector<uint8_t>(w * h, 0)}; }

void wallX(PlanningGrid& g, int x, int y0, int y1) {
    for (int y = y0; y <= y1; ++y) g.blocked[y * g.width + x] = 1;
}

NavPlanParams params(Algorithm a) {
    NavPlanParams p;
    p.algorithm = a;
    p.lethal_radius = 2.0;
    p.inflation_radius = 4.0;
    return p;
}

}  // namespace

TEST(NavPlanner, PlansAroundInflatedWall) {
    for (Algorithm a : {Algorithm::AStar, Algorithm::ThetaStar}) {
        PlanningGrid g = emptyGrid(40, 30);
        wallX(g, 20, 0, 20);
        const auto r = planOnMap(g, {5, 5}, {35, 5}, params(a));
        ASSERT_EQ(r.status, NavPlanStatus::Ok);
        // Must go over the wall end (y = 20) plus the inflation margin.
        int max_y = 0;
        for (const auto& c : r.plan.path) max_y = std::max(max_y, c.y);
        EXPECT_GE(max_y, 24);
    }
}

TEST(NavPlanner, StartInCollisionFails) {
    PlanningGrid g = emptyGrid(30, 30);
    wallX(g, 10, 0, 29);
    // 1 cell from the wall: within the lethal radius of 2.
    EXPECT_EQ(planOnMap(g, {9, 5}, {3, 5}, params(Algorithm::AStar)).status,
              NavPlanStatus::StartInCollision);
}

TEST(NavPlanner, StartInsideMarginStillPlans) {
    PlanningGrid g = emptyGrid(30, 30);
    wallX(g, 10, 0, 29);
    // 3 cells from the wall: inside the inflation (4) but not lethal (2).
    const auto r = planOnMap(g, {7, 5}, {2, 20}, params(Algorithm::AStar));
    EXPECT_EQ(r.status, NavPlanStatus::Ok);
}

TEST(NavPlanner, GoalInObstacleIsReported) {
    PlanningGrid g = emptyGrid(30, 30);
    wallX(g, 10, 0, 29);
    EXPECT_EQ(planOnMap(g, {3, 5}, {10, 5}, params(Algorithm::ThetaStar)).status,
              NavPlanStatus::GoalBlocked);
}

TEST(NavPlanner, FullyBlockedCorridorIsNoPath) {
    PlanningGrid g = emptyGrid(30, 30);
    wallX(g, 15, 0, 29);
    EXPECT_EQ(planOnMap(g, {3, 5}, {27, 5}, params(Algorithm::AStar)).status,
              NavPlanStatus::NoPath);
}
