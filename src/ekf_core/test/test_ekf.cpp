#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ekf_core/ekf.hpp"

using namespace ekf_core;

namespace {

bool isSymmetricPsd(const Mat3& p) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (std::abs(p[i][j] - p[j][i]) > 1e-12) return false;
    // Sylvester's criterion on leading minors (allowing round-off at zero).
    const double m1 = p[0][0];
    const double m2 = p[0][0] * p[1][1] - p[0][1] * p[1][0];
    const double m3 = p[0][0] * (p[1][1] * p[2][2] - p[1][2] * p[2][1]) -
                      p[0][1] * (p[1][0] * p[2][2] - p[1][2] * p[2][0]) +
                      p[0][2] * (p[1][0] * p[2][1] - p[1][1] * p[2][0]);
    return m1 >= -1e-12 && m2 >= -1e-12 && m3 >= -1e-12;
}

}  // namespace

TEST(NormalizeAngle, WrapsIntoHalfOpenInterval) {
    EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
    EXPECT_NEAR(normalizeAngle(3.0 * M_PI), M_PI, 1e-12);
    EXPECT_NEAR(normalizeAngle(-M_PI), M_PI, 1e-12);
    EXPECT_NEAR(normalizeAngle(2.0 * M_PI + 0.1), 0.1, 1e-12);
    EXPECT_NEAR(normalizeAngle(-2.0 * M_PI - 0.1), -0.1, 1e-12);
}

TEST(Mat3, InverseTimesMatrixIsIdentity) {
    const Mat3 a{{{4, 1, 0.5}, {1, 3, 0.2}, {0.5, 0.2, 2}}};
    Mat3 inv;
    ASSERT_TRUE(invert(a, inv));
    const Mat3 id = multiply(a, inv);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) EXPECT_NEAR(id[i][j], i == j ? 1.0 : 0.0, 1e-12);
    Mat3 singular{};
    EXPECT_FALSE(invert(singular, inv));
}

TEST(Predict, StraightLineKnownAnswer) {
    DiffDriveEkf ekf({1.0, 2.0, M_PI / 2}, diag3(0, 0, 0));
    for (int i = 0; i < 10; ++i) ekf.predict(0.5, 0.0, 0.2);  // 1 m along +y
    EXPECT_NEAR(ekf.pose().x, 1.0, 1e-9);
    EXPECT_NEAR(ekf.pose().y, 3.0, 1e-9);
    EXPECT_NEAR(ekf.pose().theta, M_PI / 2, 1e-12);
}

TEST(Predict, QuarterCircleKnownAnswer) {
    // v = 1, w = pi/2 for 1 s: a quarter circle of radius 2/pi.
    DiffDriveEkf ekf({0, 0, 0}, diag3(0, 0, 0));
    for (int i = 0; i < 100; ++i) ekf.predict(1.0, M_PI / 2, 0.01);
    const double r = 2.0 / M_PI;
    EXPECT_NEAR(ekf.pose().x, r, 1e-9);
    EXPECT_NEAR(ekf.pose().y, r, 1e-9);
    EXPECT_NEAR(ekf.pose().theta, M_PI / 2, 1e-9);
}

TEST(Predict, CovarianceGrowthIndependentOfStepSize) {
    // Same motion split into 1 or 100 steps: heading variance must match.
    EkfParams p;
    DiffDriveEkf coarse({0, 0, 0}, diag3(0, 0, 0), p), fine({0, 0, 0}, diag3(0, 0, 0), p);
    coarse.predict(0.0, 0.3, 1.0);
    for (int i = 0; i < 100; ++i) fine.predict(0.0, 0.3, 0.01);
    EXPECT_NEAR(coarse.covariance()[2][2], p.sigma_w * p.sigma_w, 1e-12);
    EXPECT_NEAR(fine.covariance()[2][2], p.sigma_w * p.sigma_w, 1e-12);
}

TEST(Predict, ArcJacobianMatchesFiniteDifferences) {
    // Perturb the initial heading and compare the propagated covariance's
    // implied sensitivity with a numerical derivative of the motion model.
    const double v = 0.8, w = 0.6, dt = 0.5, th = 0.3, eps = 1e-6;
    auto propagate = [&](double theta0) {
        DiffDriveEkf e({0, 0, theta0}, diag3(0, 0, 0));
        e.predict(v, w, dt);
        return e.pose();
    };
    const Pose2D plus = propagate(th + eps), minus = propagate(th - eps);
    // With P0 = diag(0,0,1) and zero process noise, P[0][2] = F[0][2].
    EkfParams no_noise;
    no_noise.sigma_v = no_noise.sigma_w = 0.0;
    DiffDriveEkf e({0, 0, th}, diag3(0, 0, 1.0), no_noise);
    // sigma = 0 makes M = 0/dt = 0, i.e. pure Jacobian propagation.
    e.predict(v, w, dt);
    EXPECT_NEAR(e.covariance()[0][2], (plus.x - minus.x) / (2 * eps), 1e-6);
    EXPECT_NEAR(e.covariance()[1][2], (plus.y - minus.y) / (2 * eps), 1e-6);
}

TEST(UpdateYaw, ScalarKalmanKnownAnswer) {
    // Prior theta = 0 with variance 0.04, measurement 0.1 with variance 0.01:
    // posterior mean 0.08, variance 0.008.
    DiffDriveEkf ekf({0, 0, 0}, diag3(1, 1, 0.04));
    ASSERT_TRUE(ekf.updateYaw(0.1, 0.1));
    EXPECT_NEAR(ekf.pose().theta, 0.08, 1e-12);
    EXPECT_NEAR(ekf.covariance()[2][2], 0.008, 1e-12);
    EXPECT_NEAR(ekf.covariance()[0][0], 1.0, 1e-12);  // uncorrelated: untouched
}

TEST(UpdateYaw, InnovationWrapsAcrossPi) {
    DiffDriveEkf ekf({0, 0, M_PI - 0.05}, diag3(1, 1, 0.01));
    ASSERT_TRUE(ekf.updateYaw(-M_PI + 0.05, 0.1));
    // The innovation is +0.1 rad, not -2pi+0.1, so theta ends near +-pi.
    EXPECT_LT(std::abs(normalizeAngle(ekf.pose().theta - M_PI)), 0.06);
}

TEST(UpdateYaw, GateRejectsOutlier) {
    DiffDriveEkf ekf({0, 0, 0}, diag3(1, 1, 0.0001));
    EXPECT_FALSE(ekf.updateYaw(1.5, 0.01));
    EXPECT_EQ(ekf.pose().theta, 0.0);
    EXPECT_GT(ekf.lastMahalanobis2(), 10.83);
}

TEST(UpdatePose, EqualCovariancesAverage) {
    DiffDriveEkf ekf({0, 0, 0}, diag3(0.1, 0.1, 0.1));
    ASSERT_TRUE(ekf.updatePose({0.2, -0.2, 0.1}, diag3(0.1, 0.1, 0.1)));
    EXPECT_NEAR(ekf.pose().x, 0.1, 1e-12);
    EXPECT_NEAR(ekf.pose().y, -0.1, 1e-12);
    EXPECT_NEAR(ekf.pose().theta, 0.05, 1e-12);
    EXPECT_NEAR(ekf.covariance()[0][0], 0.05, 1e-12);
}

TEST(UpdatePose, GateRejectsFarMeasurement) {
    DiffDriveEkf ekf({0, 0, 0}, diag3(0.01, 0.01, 0.01));
    EXPECT_FALSE(ekf.updatePose({2.0, 0.0, 0.0}, diag3(0.01, 0.01, 0.01)));
    EXPECT_EQ(ekf.pose().x, 0.0);
}

// Randomised property test: under long random sequences of predictions and
// updates the covariance stays symmetric PSD, and fusing yaw keeps the error
// lower than dead reckoning on the same noisy odometry.
TEST(Property, RandomSequencesStayConsistentAndYawHelps) {
    std::mt19937 rng(42);
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> uv(0.0, 1.0), uw(-0.8, 0.8);
    double err_fused = 0.0, err_dead = 0.0;
    for (int trial = 0; trial < 20; ++trial) {
        Pose2D truth{0, 0, 0};
        DiffDriveEkf fused(truth, diag3(1e-4, 1e-4, 1e-4)), dead(truth, diag3(1e-4, 1e-4, 1e-4));
        const double dt = 0.05;
        for (int k = 0; k < 600; ++k) {
            const double v = uv(rng), w = uw(rng);
            DiffDriveEkf step(truth, diag3(0, 0, 0));
            step.predict(v, w, dt);
            truth = step.pose();
            // Wheel odometry with a heading-rate bias and white noise.
            const double vm = v + 0.05 * n01(rng), wm = w + 0.05 + 0.05 * n01(rng);
            fused.predict(vm, wm, dt);
            dead.predict(vm, wm, dt);
            if (k % 5 == 0) fused.updateYaw(truth.theta + 0.02 * n01(rng), 0.02);
            ASSERT_TRUE(isSymmetricPsd(fused.covariance()));
            ASSERT_TRUE(isSymmetricPsd(dead.covariance()));
        }
        err_fused += std::hypot(fused.pose().x - truth.x, fused.pose().y - truth.y);
        err_dead += std::hypot(dead.pose().x - truth.x, dead.pose().y - truth.y);
    }
    EXPECT_LT(err_fused, 0.5 * err_dead);
}
