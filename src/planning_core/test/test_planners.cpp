#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "planning_core/grid_planners.hpp"

using namespace planning_core;

namespace {

PlanningGrid emptyGrid(int w, int h) { return {w, h, std::vector<uint8_t>(w * h, 0)}; }

void block(PlanningGrid& g, int x, int y) { g.blocked[y * g.width + x] = 1; }

PlanningGrid randomGrid(std::mt19937& rng, int w, int h, double density) {
    std::bernoulli_distribution occ(density);
    PlanningGrid g = emptyGrid(w, h);
    for (auto& c : g.blocked) c = occ(rng) ? 1 : 0;
    g.blocked[0] = 0;
    g.blocked[w * h - 1] = 0;
    return g;
}

}  // namespace

TEST(AStar, DiagonalOnEmptyGrid) {
    const auto r = planAStar(emptyGrid(10, 10), {0, 0}, {9, 9});
    ASSERT_TRUE(r.found);
    EXPECT_NEAR(r.cost, 9 * std::sqrt(2.0), 1e-9);
    EXPECT_EQ(r.path.size(), 10u);
}

TEST(AStar, OctileCostToOffAxisGoal) {
    const auto r = planAStar(emptyGrid(10, 10), {0, 0}, {9, 3});
    ASSERT_TRUE(r.found);
    EXPECT_NEAR(r.cost, 6 + 3 * std::sqrt(2.0), 1e-9);
}

TEST(AStar, GoesThroughGapInWall) {
    PlanningGrid g = emptyGrid(11, 11);
    for (int y = 0; y < 11; ++y)
        if (y != 8) block(g, 5, y);
    const auto r = planAStar(g, {0, 0}, {10, 0});
    ASSERT_TRUE(r.found);
    bool through_gap = false;
    for (const auto& c : r.path) {
        EXPECT_FALSE(g.isBlocked(c));
        through_gap |= (c == GridCell{5, 8});
    }
    EXPECT_TRUE(through_gap);
}

TEST(AStar, NoCornerCutting) {
    // Two diagonal obstacles touching at a corner: squeezing between them
    // diagonally is forbidden, so with the rest walled off there is no path.
    PlanningGrid g = emptyGrid(2, 2);
    block(g, 1, 0);
    block(g, 0, 1);
    EXPECT_FALSE(planAStar(g, {0, 0}, {1, 1}).found);
    EXPECT_FALSE(planThetaStar(g, {0, 0}, {1, 1}).found);
}

TEST(AStar, UnreachableBlockedAndTrivialCases) {
    PlanningGrid g = emptyGrid(5, 5);
    for (int y = 0; y < 5; ++y) block(g, 2, y);
    EXPECT_FALSE(planAStar(g, {0, 0}, {4, 4}).found);
    EXPECT_FALSE(planAStar(g, {2, 2}, {0, 0}).found);  // start blocked
    EXPECT_FALSE(planAStar(g, {0, 0}, {9, 9}).found);  // goal out of bounds
    const auto same = planAStar(g, {1, 1}, {1, 1});
    ASSERT_TRUE(same.found);
    EXPECT_EQ(same.path.size(), 1u);
    EXPECT_EQ(same.cost, 0.0);
}

// Randomised property: A* (consistent heuristic) is optimal, so its cost
// equals Dijkstra's on the same grid, and it never expands more nodes.
TEST(AStar, MatchesDijkstraOnRandomGrids) {
    std::mt19937 rng(5);
    int solved = 0;
    for (int k = 0; k < 100; ++k) {
        const PlanningGrid g = randomGrid(rng, 30, 30, 0.25);
        const auto a = planAStar(g, {0, 0}, {29, 29});
        const auto d = planAStar(g, {0, 0}, {29, 29}, {0.0});
        ASSERT_EQ(a.found, d.found);
        if (!a.found) continue;
        ++solved;
        EXPECT_NEAR(a.cost, d.cost, 1e-9);
        EXPECT_NEAR(pathLength(a.path), a.cost, 1e-9);
        EXPECT_LE(a.expansions, d.expansions);
    }
    EXPECT_GT(solved, 30);
}

TEST(ThetaStar, StraightLineToOffAxisGoal) {
    const auto r = planThetaStar(emptyGrid(10, 10), {0, 0}, {9, 3});
    ASSERT_TRUE(r.found);
    EXPECT_EQ(r.path.size(), 2u);  // a single any-angle segment
    EXPECT_NEAR(r.cost, std::sqrt(90.0), 1e-9);
}

TEST(ThetaStar, WrapsAroundObstacleCorner) {
    PlanningGrid g = emptyGrid(20, 20);
    for (int y = 0; y < 15; ++y) block(g, 10, y);
    const auto r = planThetaStar(g, {0, 0}, {19, 0});
    ASSERT_TRUE(r.found);
    EXPECT_LE(r.path.size(), 4u);
    EXPECT_LT(r.cost, planAStar(g, {0, 0}, {19, 0}).cost);
}

// Randomised property: Theta* paths are valid (every segment has line of
// sight) and their reported cost is their length. Theta* is not guaranteed
// optimal, but on these grids it is never longer than 8-connected A*.
TEST(ThetaStar, ValidAndNoLongerThanAStarOnRandomGrids) {
    std::mt19937 rng(9);
    double sum_a = 0.0, sum_t = 0.0;
    for (int k = 0; k < 100; ++k) {
        const PlanningGrid g = randomGrid(rng, 40, 40, 0.2);
        const auto a = planAStar(g, {0, 0}, {39, 39});
        const auto t = planThetaStar(g, {0, 0}, {39, 39});
        ASSERT_EQ(a.found, t.found);
        if (!t.found) continue;
        for (std::size_t i = 1; i < t.path.size(); ++i)
            ASSERT_TRUE(lineOfSight(g, t.path[i - 1], t.path[i]));
        EXPECT_NEAR(pathLength(t.path), t.cost, 1e-9);
        EXPECT_LE(t.cost, a.cost + 1e-9);
        sum_a += a.cost;
        sum_t += t.cost;
    }
    EXPECT_LT(sum_t, sum_a);
}

TEST(LineOfSight, KnownCases) {
    PlanningGrid g = emptyGrid(10, 10);
    block(g, 5, 5);
    EXPECT_TRUE(lineOfSight(g, {0, 0}, {9, 0}));
    EXPECT_FALSE(lineOfSight(g, {0, 0}, {9, 9}));  // passes through (5,5)
    EXPECT_FALSE(lineOfSight(g, {5, 0}, {5, 9}));
    EXPECT_TRUE(lineOfSight(g, {0, 1}, {9, 2}));
    // The anti-diagonal through (5,5)'s centre is blocked.
    EXPECT_FALSE(lineOfSight(g, {4, 6}, {6, 4}));
    // (4,5)->(5,6) passes the corner between (4,6) and (5,5): blocked.
    EXPECT_FALSE(lineOfSight(g, {4, 5}, {5, 6}));
}

TEST(Inflate, DiscAroundObstacle) {
    PlanningGrid g = emptyGrid(9, 9);
    block(g, 4, 4);
    const PlanningGrid inf = inflate(g, 2.0);
    EXPECT_TRUE(inf.isBlocked({6, 4}));
    EXPECT_TRUE(inf.isBlocked({5, 5}));
    EXPECT_FALSE(inf.isBlocked({6, 6}));  // distance sqrt(8) > 2
    EXPECT_FALSE(inf.isBlocked({4, 7}));
}
