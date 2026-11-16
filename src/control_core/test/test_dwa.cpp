#include <gtest/gtest.h>
#include <algorithm>

#include <cmath>

#include "control_core/controllers.hpp"

using namespace control_core;

namespace {

std::vector<Point> wall(double x0, double y0, double x1, double y1, double step = 0.05) {
    std::vector<Point> pts;
    const double len = std::hypot(x1 - x0, y1 - y0);
    for (double t = 0.0; t <= len; t += step)
        pts.push_back({x0 + (x1 - x0) * t / len, y0 + (y1 - y0) * t / len});
    return pts;
}

}  // namespace

TEST(Dwa, OpenSpaceDrivesTowardsGoal) {
    const auto out = computeDwa({0, 0, 0}, {0.4, 0.0}, {5.0, 0.0}, {});
    ASSERT_TRUE(out.feasible);
    EXPECT_NEAR(out.cmd.v, 0.5, 1e-9);
    EXPECT_NEAR(out.cmd.w, 0.0, 1e-9);
    EXPECT_EQ(out.evaluated, 7u * 21u);
}

TEST(Dwa, TurnsTowardsGoalOnTheLeft) {
    const auto out = computeDwa({0, 0, 0}, {0.2, 0.0}, {0.0, 3.0}, {});
    ASSERT_TRUE(out.feasible);
    EXPECT_GT(out.cmd.w, 0.0);
}

TEST(Dwa, RespectsDynamicWindow) {
    DwaParams p;
    const auto out = computeDwa({0, 0, 0}, {0.0, 0.0}, {5.0, 2.0}, {}, p);
    EXPECT_LE(out.cmd.v, p.acc_v * p.control_period + 1e-12);
    EXPECT_LE(std::abs(out.cmd.w), p.acc_w * p.control_period + 1e-12);
}

TEST(Dwa, AvoidsWallAheadAndKeepsClearance) {
    // Wall across the path 0.8 m ahead, goal behind it and to the side.
    const auto obstacles = wall(0.8, -1.0, 0.8, 0.5);
    DwaParams p;
    Pose2 pose{0, 0, 0};
    VelocityCommand vel{0.3, 0.0};
    const Point goal{2.0, 1.5};
    double min_clear = 1e9;
    for (int k = 0; k < 300; ++k) {
        const auto out = computeDwa(pose, vel, goal, obstacles, p);
        ASSERT_TRUE(out.feasible) << "step " << k;
        vel = out.cmd;
        pose = integrateUnicycle(pose, vel.v, vel.w, p.control_period);
        for (const auto& o : obstacles)
            min_clear =
                std::min(min_clear, std::hypot(o.x - pose.x, o.y - pose.y) - p.robot_radius);
        if (std::hypot(goal.x - pose.x, goal.y - pose.y) < 0.15) break;
    }
    EXPECT_GT(min_clear, 0.0);  // never collided
    EXPECT_LT(std::hypot(goal.x - pose.x, goal.y - pose.y), 0.15);
}

TEST(Dwa, BoxedInIsInfeasibleAndStops) {
    std::vector<Point> ring;
    for (int i = 0; i < 72; ++i)
        ring.push_back({0.25 * std::cos(i * M_PI / 36), 0.25 * std::sin(i * M_PI / 36)});
    const auto out = computeDwa({0, 0, 0}, {0.2, 0.0}, {3.0, 0.0}, ring);
    EXPECT_FALSE(out.feasible);
    EXPECT_EQ(out.cmd.v, 0.0);
    EXPECT_EQ(out.cmd.w, 0.0);
}
