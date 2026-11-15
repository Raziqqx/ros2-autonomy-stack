#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "icp_core/icp.hpp"

using namespace icp_core;

namespace {

// Densely sampled walls of a room with an interior pillar: enough structure to
// constrain x, y and theta (a single straight wall would not).
std::vector<Point2> roomPoints(double step = 0.05) {
    std::vector<Point2> pts;
    for (double t = 0.0; t <= 6.0; t += step) {
        pts.push_back({t, 0.0});
        pts.push_back({t, 4.0});
    }
    for (double t = step; t < 4.0; t += step) {
        pts.push_back({0.0, t});
        pts.push_back({6.0, t});
    }
    for (double t = 0.0; t <= 0.6; t += step) {
        pts.push_back({2.0 + t, 1.5});
        pts.push_back({2.0, 1.5 + t});
    }
    return pts;
}

std::vector<Point2> transformAll(const std::vector<Point2>& pts, const Transform2D& t) {
    std::vector<Point2> out;
    for (const auto& p : pts) out.push_back(t.apply(p));
    return out;
}

Transform2D inverse(const Transform2D& t) {
    const double c = std::cos(t.theta), s = std::sin(t.theta);
    return {-(c * t.x + s * t.y), s * t.x - c * t.y, -t.theta};
}

}  // namespace

TEST(Transform2D, ComposeWithInverseIsIdentity) {
    const Transform2D t{1.0, -2.0, 0.7};
    const Transform2D id = t.compose(inverse(t));
    EXPECT_NEAR(id.x, 0.0, 1e-12);
    EXPECT_NEAR(id.y, 0.0, 1e-12);
    EXPECT_NEAR(id.theta, 0.0, 1e-12);
}

TEST(BestFit, RecoversExactTransform) {
    const std::vector<Point2> src{{0, 0}, {1, 0}, {0, 2}, {3, 1}};
    const Transform2D truth{0.5, -1.2, 0.9};
    const Transform2D est = bestFitTransform(src, transformAll(src, truth));
    EXPECT_NEAR(est.x, truth.x, 1e-12);
    EXPECT_NEAR(est.y, truth.y, 1e-12);
    EXPECT_NEAR(est.theta, truth.theta, 1e-12);
}

TEST(PointIndex, NearestMatchesBruteForce) {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-5.0, 5.0);
    std::vector<Point2> pts(500);
    for (auto& p : pts) p = {u(rng), u(rng)};
    const PointIndex index(pts, 0.5);
    for (int k = 0; k < 200; ++k) {
        const Point2 q{u(rng), u(rng)};
        long brute = -1;
        double best = 0.5 * 0.5;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const double d2 = std::pow(pts[i].x - q.x, 2) + std::pow(pts[i].y - q.y, 2);
            if (d2 <= best) {
                best = d2;
                brute = static_cast<long>(i);
            }
        }
        EXPECT_EQ(index.nearest(q, 0.5), brute);
    }
}

IcpParams lineParams() {
    IcpParams p;
    p.method = IcpMethod::PointToLine;
    return p;
}

PointIndex indexWithNormals(const std::vector<Point2>& pts) {
    PointIndex index(pts, 0.5);
    index.computeNormals(0.12);
    return index;
}

TEST(PointIndex, NormalsArePerpendicularToWalls) {
    const PointIndex index = indexWithNormals(roomPoints());
    // Mid-way along the bottom wall the normal is +-y; on the left wall +-x.
    const auto bottom = static_cast<std::size_t>(index.nearest({3.0, 0.0}, 0.1));
    const auto left = static_cast<std::size_t>(index.nearest({0.0, 2.0}, 0.1));
    ASSERT_TRUE(index.hasNormal(bottom) && index.hasNormal(left));
    EXPECT_NEAR(std::abs(index.normal(bottom).y), 1.0, 1e-9);
    EXPECT_NEAR(std::abs(index.normal(left).x), 1.0, 1e-9);
}

TEST(Icp, PointToLineRecoversKnownOffsetWherePointToPointStalls) {
    const auto target = roomPoints();
    const Transform2D truth{0.15, -0.1, 0.06};
    // The scan is the room seen from pose `truth`: source = truth^-1 * target.
    const auto source = transformAll(target, inverse(truth));
    const PointIndex index = indexWithNormals(target);

    // Baseline: point-to-point locks onto a fixed point where every scan point
    // matches a neighbouring sample on the same wall (sliding ambiguity).
    const IcpResult p2p = align(source, index, {});
    const double p2p_err = std::hypot(p2p.transform.x - truth.x, p2p.transform.y - truth.y);
    EXPECT_GT(p2p_err, 0.01);

    const IcpResult r = align(source, index, {}, lineParams());
    ASSERT_TRUE(r.converged);
    EXPECT_LT(r.iterations, p2p.iterations);
    EXPECT_NEAR(r.transform.x, truth.x, 1e-3);
    EXPECT_NEAR(r.transform.y, truth.y, 1e-3);
    EXPECT_NEAR(r.transform.theta, truth.theta, 1e-3);
    EXPECT_LT(r.rmse, 1e-3);
    EXPECT_GT(r.inlier_ratio, 0.99);
}

TEST(Icp, NoisyPartialScanWithOutliers) {
    std::mt19937 rng(3);
    std::normal_distribution<double> noise(0.0, 0.01);
    std::uniform_real_distribution<double> u(-10.0, 10.0);
    const auto target = roomPoints();
    const Transform2D truth{-0.2, 0.12, -0.08};
    // Keep only the lower half of the room (partial view), add range noise and
    // clutter points that exist only in the scan (e.g. a person).
    std::vector<Point2> visible;
    for (const auto& p : target)
        if (p.y < 2.5) visible.push_back(p);
    auto source = transformAll(visible, inverse(truth));
    for (auto& p : source) {
        p.x += noise(rng);
        p.y += noise(rng);
    }
    for (int i = 0; i < 30; ++i) source.push_back({u(rng), u(rng)});
    for (const IcpParams& p : {IcpParams{}, lineParams()}) {
        const IcpResult r = align(source, indexWithNormals(target), {}, p);
        ASSERT_TRUE(r.converged);
        EXPECT_NEAR(r.transform.x, truth.x, 0.01);
        EXPECT_NEAR(r.transform.y, truth.y, 0.01);
        EXPECT_NEAR(r.transform.theta, truth.theta, 0.005);
    }
}

TEST(Icp, SingleWallIsDegenerateForPointToLine) {
    // A lone straight wall cannot constrain motion along it.
    std::vector<Point2> wall;
    for (double t = 0.0; t <= 5.0; t += 0.05) wall.push_back({t, 0.0});
    const IcpResult r = align(wall, indexWithNormals(wall), {}, lineParams());
    EXPECT_FALSE(r.converged);
}

TEST(Icp, TooFewCorrespondencesIsNotConverged) {
    const auto target = roomPoints();
    // A scan 50 m away from anything in the target.
    std::vector<Point2> source{{50, 50}, {51, 50}, {50, 51}};
    const IcpResult r = align(source, PointIndex(target, 0.5), {});
    EXPECT_FALSE(r.converged);
    EXPECT_EQ(r.num_correspondences, 0u);
}

// Randomised property: from initial errors within the basin (<= 0.2 m,
// <= 0.1 rad) point-to-line ICP converges to the true pose on the room.
TEST(Icp, RandomInitialErrorsConverge) {
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> ut(-0.2, 0.2), ur(-0.1, 0.1);
    const auto target = roomPoints();
    const PointIndex index = indexWithNormals(target);
    const Transform2D truth{0.3, 0.2, 0.05};
    const auto source = transformAll(target, inverse(truth));
    for (int k = 0; k < 25; ++k) {
        const Transform2D guess{truth.x + ut(rng), truth.y + ut(rng), truth.theta + ur(rng)};
        const IcpResult r = align(source, index, guess, lineParams());
        ASSERT_TRUE(r.converged) << "trial " << k;
        EXPECT_NEAR(r.transform.x, truth.x, 5e-3);
        EXPECT_NEAR(r.transform.y, truth.y, 5e-3);
        EXPECT_NEAR(r.transform.theta, truth.theta, 2e-3);
    }
}
