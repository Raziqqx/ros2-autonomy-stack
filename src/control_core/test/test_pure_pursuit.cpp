#include <gtest/gtest.h>
#include <algorithm>

#include <cmath>

#include "control_core/controllers.hpp"

using namespace control_core;

namespace {

std::vector<Point> straightPath(double length, double step = 0.1) {
    std::vector<Point> path;
    for (double x = 0.0; x <= length + 1e-9; x += step) path.push_back({x, 0.0});
    return path;
}

double pointToPathDistance(const Point& p, const std::vector<Point>& path) {
    double best = 1e9;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Point &a = path[i], &b = path[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0);
        best = std::min(best, std::hypot(a.x + t * dx - p.x, a.y + t * dy - p.y));
    }
    return best;
}

}  // namespace

TEST(Unicycle, QuarterCircle) {
    const Pose2 p = integrateUnicycle({0, 0, 0}, 1.0, M_PI / 2, 1.0);
    EXPECT_NEAR(p.x, 2.0 / M_PI, 1e-12);
    EXPECT_NEAR(p.y, 2.0 / M_PI, 1e-12);
    EXPECT_NEAR(p.theta, M_PI / 2, 1e-12);
}

TEST(PurePursuit, StraightPathGoesStraightAtFullSpeed) {
    PurePursuit pp;
    pp.setPath(straightPath(5.0));
    const auto out = pp.compute({0.0, 0.0, 0.0});
    EXPECT_NEAR(out.cmd.w, 0.0, 1e-9);
    EXPECT_NEAR(out.cmd.v, PurePursuitParams{}.v_max, 1e-9);
    EXPECT_NEAR(out.lookahead_point.x, 0.6, 1e-9);
}

TEST(PurePursuit, CurvatureOfCircularPathIsInverseRadius) {
    // Robot on a circle of radius R, tangent to it: the pure pursuit arc
    // through any point of that circle is the circle itself, so kappa = 1/R.
    const double R = 2.0;
    std::vector<Point> path;
    for (int i = 0; i <= 200; ++i) {
        const double a = -M_PI / 2 + i * M_PI / 200;
        path.push_back({R * std::cos(a), R + R * std::sin(a)});  // starts at (0,0) heading +x
    }
    PurePursuit pp;
    pp.setPath(path);
    const auto out = pp.compute({0.0, 0.0, 0.0});
    EXPECT_NEAR(out.curvature, 1.0 / R, 2e-3);  // path is a polyline, not an exact circle
    EXPECT_GT(out.cmd.w, 0.0);
    EXPECT_NEAR(out.cmd.w / out.cmd.v, out.curvature, 1e-9);
}

TEST(PurePursuit, TurnsTowardsPathOnTheRight) {
    PurePursuit pp;
    std::vector<Point> path;
    for (double x = 0.0; x <= 5.0; x += 0.1) path.push_back({x, -0.5});
    pp.setPath(path);
    EXPECT_LT(pp.compute({0.0, 0.0, 0.0}).cmd.w, 0.0);
}

TEST(PurePursuit, RotatesInPlaceWhenPathIsBehind) {
    PurePursuit pp;
    pp.setPath(straightPath(5.0));
    const auto out = pp.compute({0.0, 0.1, M_PI});  // facing away from the path
    EXPECT_EQ(out.cmd.v, 0.0);
    EXPECT_NEAR(std::abs(out.cmd.w), PurePursuitParams{}.w_max, 1e-12);
}

TEST(PurePursuit, GoalReachedAndEmptyPath) {
    PurePursuit pp;
    pp.setPath(straightPath(1.0));
    EXPECT_TRUE(pp.compute({0.97, 0.0, 0.0}).goal_reached);
    pp.setPath({});
    EXPECT_TRUE(pp.compute({0.0, 0.0, 0.0}).goal_reached);
}

TEST(PurePursuit, ProgressDoesNotJumpBackOnLoopingPath) {
    // Out along y=0, then back along y=0.3: near the start the return leg is
    // close, but progress must stay on the outbound leg.
    std::vector<Point> path = straightPath(3.0);
    for (double x = 3.0; x >= 0.0; x -= 0.1) path.push_back({x, 0.3});
    PurePursuit pp;
    pp.setPath(path);
    pp.compute({0.5, 0.0, 0.0});
    EXPECT_LT(pp.progressIndex(), 30u);
    const auto out = pp.compute({0.5, 0.05, 0.0});
    EXPECT_GT(out.lookahead_point.x, 0.5);  // still heading outward
}

// Closed-loop scenario: track an L-shaped path from an offset start; the
// robot reaches the goal and stays near the path.
TEST(PurePursuit, ClosedLoopTracksLPath) {
    std::vector<Point> path = straightPath(4.0);
    for (double y = 0.1; y <= 3.0 + 1e-9; y += 0.1) path.push_back({4.0, y});
    PurePursuit pp;
    pp.setPath(path);
    Pose2 pose{0.0, 0.2, 0.0};
    double max_err = 0.0;
    bool reached = false;
    for (int k = 0; k < 2000 && !reached; ++k) {
        const auto out = pp.compute(pose);
        reached = out.goal_reached;
        pose = integrateUnicycle(pose, out.cmd.v, out.cmd.w, 0.02);
        if (k > 100) max_err = std::max(max_err, pointToPathDistance({pose.x, pose.y}, path));
    }
    EXPECT_TRUE(reached);
    EXPECT_LT(max_err, 0.3);  // corner cutting is bounded by the lookahead
}
