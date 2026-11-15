#include <gtest/gtest.h>

#include "control_core/path_tracking.hpp"

using namespace control_core;

TEST(LocalGoal, InterpolatesAlongSparsePath) {
    // L-shaped Theta*-style path with only its turning points.
    const std::vector<Point> path{{0, 0}, {2, 0}, {2, 3}};
    const Point g = localGoal(path, {0.5, 0.1, 0.0}, 1.0);
    EXPECT_NEAR(g.x, 1.5, 1e-9);
    EXPECT_NEAR(g.y, 0.0, 1e-9);
    // Crosses the corner: 0.5 m left on the first leg, 0.5 m up the second.
    const Point h = localGoal(path, {1.5, -0.2, 0.0}, 1.0);
    EXPECT_NEAR(h.x, 2.0, 1e-9);
    EXPECT_NEAR(h.y, 0.5, 1e-9);
}

TEST(LocalGoal, ClampsToPathEnd) {
    const std::vector<Point> path{{0, 0}, {1, 0}};
    const Point g = localGoal(path, {0.8, 0.0, 0.0}, 2.0);
    EXPECT_DOUBLE_EQ(g.x, 1.0);
    EXPECT_DOUBLE_EQ(localGoal(std::vector<Point>{Point{3, 4}}, Pose2{}, 1.0).y, 4.0);
}

TEST(LocalGoal, UsesClosestSegmentNotFirst) {
    // Robot near the end of a U-shaped path: the goal must be ahead of it,
    // not near the path start which is equally close in x.
    const std::vector<Point> path{{0, 0}, {3, 0}, {3, 1}, {0, 1}};
    const Point g = localGoal(path, {1.0, 1.05, 3.14159}, 0.5);
    EXPECT_NEAR(g.x, 0.5, 1e-9);
    EXPECT_NEAR(g.y, 1.0, 1e-9);
}

TEST(StuckDetector, NeedsSustainedStall) {
    StuckDetector d(1.0);
    EXPECT_FALSE(d.update(0.0, 0.3, 0.0));
    EXPECT_FALSE(d.update(0.5, 0.3, 0.0));
    EXPECT_TRUE(d.update(1.0, 0.3, 0.0));
}

TEST(StuckDetector, MotionOrZeroCommandResets) {
    StuckDetector d(1.0);
    d.update(0.0, 0.3, 0.0);
    EXPECT_FALSE(d.update(0.6, 0.3, 0.25));  // moving again
    EXPECT_FALSE(d.update(1.2, 0.3, 0.0));   // stall restarts here
    EXPECT_FALSE(d.update(1.5, 0.0, 0.0));   // not commanded: not stuck
    EXPECT_FALSE(d.update(2.3, 0.3, 0.0));
    EXPECT_TRUE(d.update(3.3, -0.3, 0.0));   // reversing into something counts too
}
